use crate::{
    Result,
    comms::{FrontendSender, messages::*},
    map_gen::{
        self,
        egui_map::{AreaSymbol, TempMap},
        pipeline::{DeferredHydrologyTile, PreparedTile},
    },
    neighbors::NeighborSide,
    parameters::{FileParameters, MapParameters},
    raster::{
        BuildingProbability, Dfm, Elevation, FilteredSurface, FlowAccumulation, GroundRelief2m,
        GroundRelief5m, HardObjectConfidence, HardObjectHeight, HeightAboveGround,
        HeightAboveGroundMean, Hillshade, Intensity, LastReturn, MarshProbability, MarshReason,
        MarshSupport, Ndvd, PlanarPointFraction, PlaneResidual, PointDensity, RasterMarker, Slope,
        SurfaceObjects, VegetationLikelihood, WetnessScore,
    },
    statistics::LidarStats,
};
use anyhow::Context;
use geo::{Area, BooleanOps, Intersects};
use rayon::{ThreadPool, prelude::*};

use std::{
    cmp::Ordering,
    sync::{Arc, Mutex},
};

struct DeferredStreamTile {
    key: (usize, usize),
    tile: DeferredHydrologyTile,
}

pub fn make_map(
    sender: FrontendSender,
    thread_pool: &ThreadPool,
    map_params: MapParameters,
    file_params: FileParameters,
    mut polygon_filter: Option<geo::Polygon>,
    stats: LidarStats,
) -> Result<()> {
    let _ = sender.send(FrontendTask::Log("Starting map generation!".to_string()));

    let num_threads = thread_pool.current_num_threads();

    let _ = sender.send(FrontendTask::Log(format!(
        "Running on {} threads",
        num_threads
    )));

    // Figure out spatial relationships of the lidar files, assuming they are divided from a big lidar-project by a square-ish grid
    let (laz_paths, laz_neighbor_map, bounds, ref_point, masl) =
        super::map_laz(&file_params.paths, &polygon_filter)?;

    let map = Arc::new(Mutex::new(TempMap::new(
        ref_point,
        map_params.scale,
        map_params.output.crs.clone(),
    )));
    let saved_dem_rasters = file_params
        .save_dem_raster
        .then(|| Arc::new(Mutex::new(Vec::<Dfm<Elevation>>::new())));
    let saved_slope_rasters = file_params
        .save_slope_raster
        .then(|| Arc::new(Mutex::new(Vec::<Dfm<Slope>>::new())));
    let saved_hillshade_rasters = file_params
        .save_hillshade_raster
        .then(|| Arc::new(Mutex::new(Vec::<Dfm<Hillshade>>::new())));
    let saved_intensity_rasters = file_params
        .save_intensity_raster
        .then(|| Arc::new(Mutex::new(Vec::<Dfm<Intensity>>::new())));
    let saved_last_return_rasters = file_params
        .save_last_return_raster
        .then(|| Arc::new(Mutex::new(Vec::<Dfm<LastReturn>>::new())));
    let saved_canopy_height_rasters = file_params
        .save_canopy_height_raster
        .then(|| Arc::new(Mutex::new(Vec::<Dfm<HeightAboveGround>>::new())));
    let saved_surface_objects_rasters = file_params
        .save_surface_objects_raster
        .then(|| Arc::new(Mutex::new(Vec::<Dfm<SurfaceObjects>>::new())));
    let saved_ground_relief_2m_rasters = file_params
        .save_ground_relief_2m_raster
        .then(|| Arc::new(Mutex::new(Vec::<Dfm<GroundRelief2m>>::new())));
    let saved_ground_relief_5m_rasters = file_params
        .save_ground_relief_5m_raster
        .then(|| Arc::new(Mutex::new(Vec::<Dfm<GroundRelief5m>>::new())));
    let saved_hard_object_height_rasters = file_params
        .save_hard_object_height_raster
        .then(|| Arc::new(Mutex::new(Vec::<Dfm<HardObjectHeight>>::new())));
    let saved_hard_object_confidence_rasters = file_params
        .save_hard_object_confidence_raster
        .then(|| Arc::new(Mutex::new(Vec::<Dfm<HardObjectConfidence>>::new())));
    let saved_vegetation_likelihood_rasters = file_params
        .save_vegetation_likelihood_raster
        .then(|| Arc::new(Mutex::new(Vec::<Dfm<VegetationLikelihood>>::new())));
    let saved_filtered_surface_rasters = file_params
        .save_filtered_surface_raster
        .then(|| Arc::new(Mutex::new(Vec::<Dfm<FilteredSurface>>::new())));
    let saved_ndvd_rasters = file_params
        .save_ndvd_raster
        .then(|| Arc::new(Mutex::new(Vec::<Dfm<Ndvd>>::new())));
    let saved_point_density_rasters = file_params
        .save_point_density_raster
        .then(|| Arc::new(Mutex::new(Vec::<Dfm<PointDensity>>::new())));
    let saved_flow_accumulation_rasters = file_params
        .save_flow_accumulation_raster
        .then(|| Arc::new(Mutex::new(Vec::<Dfm<FlowAccumulation>>::new())));
    let saved_building_height_rasters = file_params
        .save_building_height_raster
        .then(|| Arc::new(Mutex::new(Vec::<Dfm<HeightAboveGroundMean>>::new())));
    let saved_building_planarity_rasters = file_params
        .save_building_planarity_raster
        .then(|| Arc::new(Mutex::new(Vec::<Dfm<PlanarPointFraction>>::new())));
    let saved_building_residual_rasters = file_params
        .save_building_residual_raster
        .then(|| Arc::new(Mutex::new(Vec::<Dfm<PlaneResidual>>::new())));
    let saved_building_probability_rasters = file_params
        .save_building_probability_raster
        .then(|| Arc::new(Mutex::new(Vec::<Dfm<BuildingProbability>>::new())));
    let saved_building_plane_rejected_rasters = file_params
        .save_building_plane_rejected_raster
        .then(|| Arc::new(Mutex::new(Vec::<Dfm<BuildingProbability>>::new())));
    let saved_marsh_probability_rasters = file_params
        .save_marsh_probability_raster
        .then(|| Arc::new(Mutex::new(Vec::<Dfm<MarshProbability>>::new())));
    let saved_marsh_support_rasters = file_params
        .save_marsh_support_raster
        .then(|| Arc::new(Mutex::new(Vec::<Dfm<MarshSupport>>::new())));
    let saved_marsh_wetness_rasters = file_params
        .save_marsh_wetness_raster
        .then(|| Arc::new(Mutex::new(Vec::<Dfm<WetnessScore>>::new())));
    let saved_marsh_reason_rasters = file_params
        .save_marsh_reason_raster
        .then(|| Arc::new(Mutex::new(Vec::<Dfm<MarshReason>>::new())));
    let deferred_stream_tiles = Arc::new(Mutex::new(Vec::<DeferredStreamTile>::new()));

    if let Some(polygon) = &mut polygon_filter {
        polygon.exterior_mut(|l| {
            for c in l.0.iter_mut() {
                *c = *c - ref_point;
            }
        });
    }

    for fi in 0..laz_paths.len() {
        #[rustfmt::skip]
        let _ = sender.send(FrontendTask::Log("\n***********************************************".to_string()));
        #[rustfmt::skip]
        let _ = sender.send(FrontendTask::Log(format!("\t Processing Lidar-file {} of {}", fi + 1, laz_paths.len())));
        #[rustfmt::skip]
        let _ = sender.send(FrontendTask::Log(format!(
            "\t{:?}",
            laz_paths[fi]
                .file_name()
                .map(|name| name.to_string_lossy().into_owned())
                .unwrap_or_else(|| laz_paths[fi].display().to_string())
        )));
        #[rustfmt::skip]
        let _ = sender.send(FrontendTask::Log("-----------------------------------------------".to_string()));
        let _ = sender.send(FrontendTask::ProgressBar(ProgressBar::Start));

        // first get the sub-tile bounds for the current lidar file
        // need tile-neighbor maps, bounds, cut-bounds and touched files (for the edge tiles)
        let (tile_bounds, mut cut_bounds, nx, ny) =
            map_gen::common::retile_bounds(&bounds[fi], &laz_neighbor_map[fi]);

        for cb in cut_bounds.iter_mut() {
            *cb = geo::Rect::new(cb.min() - ref_point, cb.max() - ref_point);
        }

        let num_tiles = nx * ny;
        let inc = 1. / num_tiles as f32;

        thread_pool.install(|| {
            (0..num_tiles).into_par_iter().for_each(|tile_i| {
                let edge_tile = NeighborSide::is_edge_tile(tile_i, nx, ny);

                if let Some(polygon) = &polygon_filter
                    && !cut_bounds[tile_i].intersects(polygon)
                {
                    return;
                }

                let (cloud, all_point_cloud, mut hull) = match super::read_laz(
                    &laz_paths,
                    &laz_neighbor_map[fi],
                    tile_bounds[tile_i],
                    edge_tile,
                    ref_point,
                ) {
                    Ok(p) => p,
                    Err(e) => {
                        if e.downcast_ref::<crate::Error>()
                            .is_some_and(|e| matches!(e, crate::Error::NoGroundPoints))
                        {
                            return;
                        }
                        let _ = sender.send(FrontendTask::Error(e.to_string(), true));
                        return;
                    }
                };

                if let Some(polygon) = &polygon_filter {
                    let mut mp = polygon.intersection(&hull);

                    if mp.0.is_empty() {
                        return;
                    }

                    mp.0.sort_by(|a, b| {
                        a.signed_area()
                            .partial_cmp(&b.signed_area())
                            .unwrap_or(Ordering::Equal)
                    });
                    hull = mp.0.swap_remove(0);
                }

                let tile = match PreparedTile::from_cloud(
                    cloud,
                    all_point_cloud,
                    &stats,
                    hull,
                    cut_bounds[tile_i],
                ) {
                    Ok(Some(tile)) => tile,
                    Ok(None) => return,
                    Err(e) => {
                        let _ = sender.send(FrontendTask::Error(e.to_string(), true));
                        return;
                    }
                };

                let objects = match super::compute_tile_map_objects(&map_params, &tile) {
                    Ok(objects) => objects,
                    Err(e) => {
                        let _ = sender.send(FrontendTask::Error(e.to_string(), true));
                        return;
                    }
                };

                let needs_building_fit = saved_building_height_rasters.is_some()
                    || saved_building_planarity_rasters.is_some()
                    || saved_building_residual_rasters.is_some();
                let building_fit = if needs_building_fit {
                    match tile.building_surface_fit(&map_params.building) {
                        Ok(fit) => fit,
                        Err(e) => {
                            let _ = sender.send(FrontendTask::Error(e.to_string(), true));
                            return;
                        }
                    }
                } else {
                    None
                };
                if let (Some(saved_rasters), Some(fit)) =
                    (&saved_building_height_rasters, &building_fit)
                    && !push_saved_raster(
                        saved_rasters,
                        fit.height_mean.clone(),
                        "Building height",
                        &sender,
                    )
                {
                    return;
                }
                if let (Some(saved_rasters), Some(fit)) =
                    (&saved_building_planarity_rasters, &building_fit)
                    && !push_saved_raster(
                        saved_rasters,
                        fit.planar_point_fraction.clone(),
                        "Building planarity",
                        &sender,
                    )
                {
                    return;
                }
                if let (Some(saved_rasters), Some(fit)) =
                    (&saved_building_residual_rasters, &building_fit)
                    && !push_saved_raster(
                        saved_rasters,
                        fit.plane_residual.clone(),
                        "Building plane residual",
                        &sender,
                    )
                {
                    return;
                }
                if let Some(saved_rasters) = &saved_building_probability_rasters {
                    match tile.building_detection(&map_params.building) {
                        Ok(Some(detection)) => {
                            if !push_saved_raster(
                                saved_rasters,
                                detection.probability.clone(),
                                "Building probability",
                                &sender,
                            ) {
                                return;
                            }
                        }
                        Ok(None) => {}
                        Err(e) => {
                            let _ = sender.send(FrontendTask::Error(e.to_string(), true));
                            return;
                        }
                    }
                }
                if let Some(saved_rasters) = &saved_building_plane_rejected_rasters {
                    match tile.building_detection(&map_params.building) {
                        Ok(Some(detection)) => {
                            if !push_saved_raster(
                                saved_rasters,
                                detection.plane_rejected_mask.clone(),
                                "Rejected building-plane mask",
                                &sender,
                            ) {
                                return;
                            }
                        }
                        Ok(None) => {}
                        Err(e) => {
                            let _ = sender.send(FrontendTask::Error(e.to_string(), true));
                            return;
                        }
                    }
                }

                if let Some(saved_rasters) = &saved_dem_rasters
                    && !push_saved_raster(saved_rasters, tile.rasters.dem.clone(), "DEM", &sender)
                {
                    return;
                }

                if let Some(saved_rasters) = &saved_slope_rasters
                    && !push_saved_raster(
                        saved_rasters,
                        tile.rasters.slope.clone(),
                        "Slope",
                        &sender,
                    )
                {
                    return;
                }

                if let Some(saved_rasters) = &saved_hillshade_rasters
                    && !push_saved_raster(
                        saved_rasters,
                        tile.rasters.dem.hillshade(3. * std::f64::consts::FRAC_PI_4),
                        "Hillshade",
                        &sender,
                    )
                {
                    return;
                }

                if let Some(saved_rasters) = &saved_last_return_rasters
                    && !push_saved_raster(
                        saved_rasters,
                        tile.rasters.last_return.clone(),
                        "Last-return",
                        &sender,
                    )
                {
                    return;
                }

                if let Some(saved_rasters) = &saved_intensity_rasters
                    && !push_saved_raster(
                        saved_rasters,
                        tile.rasters.intensity.clone(),
                        "Intensity",
                        &sender,
                    )
                {
                    return;
                }

                if let Some(saved_rasters) = &saved_canopy_height_rasters
                    && !push_saved_raster(
                        saved_rasters,
                        tile.rasters.canopy_height.clone(),
                        "Canopy Height",
                        &sender,
                    )
                {
                    return;
                }

                if let Some(saved_rasters) = &saved_surface_objects_rasters
                    && !push_saved_raster(
                        saved_rasters,
                        tile.rasters.surface_objects.clone(),
                        "Surface objects",
                        &sender,
                    )
                {
                    return;
                }

                if let Some(saved_rasters) = &saved_ground_relief_2m_rasters
                    && !push_saved_raster(
                        saved_rasters,
                        tile.rasters.ground_relief_2m.clone(),
                        "2 m ground relief",
                        &sender,
                    )
                {
                    return;
                }

                if let Some(saved_rasters) = &saved_ground_relief_5m_rasters
                    && !push_saved_raster(
                        saved_rasters,
                        tile.rasters.ground_relief_5m.clone(),
                        "5 m ground relief",
                        &sender,
                    )
                {
                    return;
                }

                if let Some(saved_rasters) = &saved_hard_object_height_rasters
                    && !push_saved_raster(
                        saved_rasters,
                        tile.rasters.hard_object_height.clone(),
                        "Hard-object height",
                        &sender,
                    )
                {
                    return;
                }

                if let Some(saved_rasters) = &saved_hard_object_confidence_rasters
                    && !push_saved_raster(
                        saved_rasters,
                        tile.rasters.hard_object_confidence.clone(),
                        "Hard-object confidence",
                        &sender,
                    )
                {
                    return;
                }

                if let Some(saved_rasters) = &saved_vegetation_likelihood_rasters
                    && !push_saved_raster(
                        saved_rasters,
                        tile.rasters.vegetation_likelihood.clone(),
                        "Vegetation likelihood",
                        &sender,
                    )
                {
                    return;
                }

                if let Some(saved_rasters) = &saved_filtered_surface_rasters
                    && !push_saved_raster(
                        saved_rasters,
                        tile.rasters.filtered_surface.clone(),
                        "Vegetation-filtered surface",
                        &sender,
                    )
                {
                    return;
                }

                if let Some(saved_rasters) = &saved_ndvd_rasters
                    && !push_saved_raster(
                        saved_rasters,
                        tile.rasters.compute_ndvd(map_params.vegetation.weights),
                        "NDVD",
                        &sender,
                    )
                {
                    return;
                }

                if let Some(saved_rasters) = &saved_point_density_rasters
                    && !push_saved_raster(
                        saved_rasters,
                        tile.rasters.point_density.clone(),
                        "Lidar point-density",
                        &sender,
                    )
                {
                    return;
                }

                let tile = match tile.into_deferred_hydrology(&map_params) {
                    Ok(tile) => tile,
                    Err(e) => {
                        let _ = sender.send(FrontendTask::Error(e.to_string(), true));
                        return;
                    }
                };
                if let Ok(mut stream_tiles) = deferred_stream_tiles.lock() {
                    stream_tiles.push(DeferredStreamTile {
                        key: (fi, tile_i),
                        tile,
                    });
                } else {
                    let _ = sender.send(FrontendTask::Error(
                        "Deferred stream tile mutex was poisoned".to_string(),
                        true,
                    ));
                    return;
                }
                {
                    if let Ok(mut map) = map.lock() {
                        for object in objects {
                            map.add_object(object);
                        }
                    } else {
                        let _ = sender.send(FrontendTask::Error(
                            "Map generation mutex was poisoned".to_string(),
                            true,
                        ));
                        return;
                    }
                }
                let _ = sender.send(FrontendTask::ProgressBar(ProgressBar::Inc(inc)));
            });
        });

        let _ = sender.send(FrontendTask::ProgressBar(ProgressBar::Finish));
    }

    let mut stream_tiles = Arc::<Mutex<Vec<DeferredStreamTile>>>::into_inner(deferred_stream_tiles)
        .context("Could not get deferred stream tiles; a worker still holds a reference")?
        .into_inner()
        .map_err(|_| anyhow::anyhow!("Deferred stream tile mutex was poisoned"))?;
    stream_tiles.sort_by_key(|tile| tile.key);

    let _ = sender.send(FrontendTask::Log(
        "Accumulating flow across tile boundaries...".to_string(),
    ));
    crate::raster::accumulate_cross_tile_flow(
        stream_tiles
            .iter_mut()
            .map(|tile| &mut tile.tile.stream_flow),
    )?;

    if let Some(saved_rasters) = &saved_flow_accumulation_rasters {
        let mut saved_rasters = saved_rasters
            .lock()
            .map_err(|_| anyhow::anyhow!("Flow accumulation raster mutex was poisoned"))?;
        saved_rasters.extend(
            stream_tiles
                .iter()
                .map(|tile| tile.tile.stream_flow.flow_accumulation().clone()),
        );
    }

    let mut map = Arc::<Mutex<TempMap>>::into_inner(map)
        .context("Could not get inner map value; a worker still holds a reference")?
        .into_inner()
        .map_err(|_| anyhow::anyhow!("Map mutex was poisoned during generation"))?;

    let mut final_marsh_params = map_params.clone();
    // Apply the physical minimum area after fragments have been stitched, so
    // a valid cross-boundary marsh is not discarded independently by both
    // owning tiles.
    final_marsh_params.marsh.minimum_polygon_area_m2 = 0.;
    for tile in &stream_tiles {
        for object in map_gen::common::compute_streams(
            &tile.tile.stream_flow,
            &tile.tile.cut_overlay,
            &map_params,
        ) {
            map.add_object(object);
        }

        if map_params.marsh.enabled {
            let marsh = tile.tile.compute_marsh(&final_marsh_params)?;
            if let Some(saved_rasters) = &saved_marsh_probability_rasters {
                push_saved_raster(
                    saved_rasters,
                    marsh.detection.probability.clone(),
                    "Marsh probability",
                    &sender,
                );
            }
            if let Some(saved_rasters) = &saved_marsh_support_rasters {
                push_saved_raster(
                    saved_rasters,
                    marsh.detection.support.clone(),
                    "Marsh observation support",
                    &sender,
                );
            }
            if let Some(saved_rasters) = &saved_marsh_wetness_rasters {
                push_saved_raster(
                    saved_rasters,
                    marsh.detection.wetness_score.clone(),
                    "Marsh wetness score",
                    &sender,
                );
            }
            if let Some(saved_rasters) = &saved_marsh_reason_rasters {
                push_saved_raster(
                    saved_rasters,
                    marsh.detection.reason.clone(),
                    "Marsh reason code",
                    &sender,
                );
            }
            for object in marsh.objects {
                map.add_object(object);
            }
        }
    }

    // Ownership clipping can split one roof at an internal tile boundary.
    // Reconcile those exactly adjoining fragments before minimum-size checks.
    map.merge_areas(AreaSymbol::Building, 2. * crate::SIMPLIFICATION_DIST)?;
    map.merge_areas(AreaSymbol::Marsh, 2. * crate::CELL_SIZE_METERS)?;
    map.merge_areas(
        AreaSymbol::UncrossableWaterWithBankLine,
        2. * crate::CELL_SIZE_METERS,
    )?;
    map.subtract_area_symbol(AreaSymbol::Marsh, AreaSymbol::UncrossableWaterWithBankLine)?;
    map.subtract_area_symbol(AreaSymbol::Marsh, AreaSymbol::Building)?;
    map.filter_area_min_size(AreaSymbol::Marsh, map_params.marsh.minimum_polygon_area_m2);

    let min_size_filter_symbols =
        map_params.min_size_filter_symbols(true, true, true, true, true, true);
    if !min_size_filter_symbols.is_empty() {
        let _ = sender.send(FrontendTask::Log(
            "Filtering polygons by minimum symbol size...".to_string(),
        ));
        map.merge_and_filter_min_size(min_size_filter_symbols)?;
    }

    let _ = sender.send(FrontendTask::Log("Post-processing contours...".to_string()));

    map.mark_basemap_depressions();

    map.merge_lines(5. * crate::SIMPLIFICATION_DIST);

    // convert the smallest knolls and depressions to point symbols
    map.make_dotknolls_and_depressions(
        map_params.contour.dot_knoll_area.0,
        map_params.contour.dot_knoll_area.1,
        1.5,
    );

    let _ = sender.send(FrontendTask::Log("Writing Omap file...".to_string()));

    let omap = map.into_omap(masl, &map_params.geometry)?;

    omap.to_file(file_params.save_location.clone())?;

    write_saved_rasters(
        &sender,
        saved_dem_rasters,
        ("DEM", "dem"),
        &file_params,
        ref_point,
        map_params.output.crs.as_ref(),
    )?;
    write_saved_rasters(
        &sender,
        saved_marsh_probability_rasters,
        ("marsh probability", "marsh_probability"),
        &file_params,
        ref_point,
        map_params.output.crs.as_ref(),
    )?;
    write_saved_rasters(
        &sender,
        saved_marsh_support_rasters,
        ("marsh observation support", "marsh_support"),
        &file_params,
        ref_point,
        map_params.output.crs.as_ref(),
    )?;
    write_saved_rasters(
        &sender,
        saved_marsh_wetness_rasters,
        ("marsh wetness score", "marsh_wetness"),
        &file_params,
        ref_point,
        map_params.output.crs.as_ref(),
    )?;
    write_saved_viewer_rasters(
        &sender,
        saved_marsh_reason_rasters,
        ("marsh reason code", "marsh_reason"),
        &file_params,
        ref_point,
        map_params.output.crs.as_ref(),
    )?;
    write_saved_rasters(
        &sender,
        saved_slope_rasters,
        ("slope", "slope"),
        &file_params,
        ref_point,
        map_params.output.crs.as_ref(),
    )?;
    write_saved_viewer_rasters(
        &sender,
        saved_hillshade_rasters,
        ("hillshade", "hillshade"),
        &file_params,
        ref_point,
        map_params.output.crs.as_ref(),
    )?;
    write_saved_viewer_rasters(
        &sender,
        saved_last_return_rasters,
        ("last-return", "last_return"),
        &file_params,
        ref_point,
        map_params.output.crs.as_ref(),
    )?;
    write_saved_rasters(
        &sender,
        saved_intensity_rasters,
        ("intensity", "intensity"),
        &file_params,
        ref_point,
        map_params.output.crs.as_ref(),
    )?;
    write_saved_rasters(
        &sender,
        saved_canopy_height_rasters,
        ("canopy height", "canopy_height"),
        &file_params,
        ref_point,
        map_params.output.crs.as_ref(),
    )?;
    write_saved_viewer_rasters(
        &sender,
        saved_surface_objects_rasters,
        ("surface objects", "surface_objects"),
        &file_params,
        ref_point,
        map_params.output.crs.as_ref(),
    )?;
    write_saved_rasters(
        &sender,
        saved_ground_relief_2m_rasters,
        ("2 m ground relief", "ground_relief_2m"),
        &file_params,
        ref_point,
        map_params.output.crs.as_ref(),
    )?;
    write_saved_rasters(
        &sender,
        saved_ground_relief_5m_rasters,
        ("5 m ground relief", "ground_relief_5m"),
        &file_params,
        ref_point,
        map_params.output.crs.as_ref(),
    )?;
    write_saved_rasters(
        &sender,
        saved_hard_object_height_rasters,
        ("hard-object height", "hard_object_height"),
        &file_params,
        ref_point,
        map_params.output.crs.as_ref(),
    )?;
    write_saved_rasters(
        &sender,
        saved_hard_object_confidence_rasters,
        ("hard-object confidence", "hard_object_confidence"),
        &file_params,
        ref_point,
        map_params.output.crs.as_ref(),
    )?;
    write_saved_rasters(
        &sender,
        saved_vegetation_likelihood_rasters,
        ("vegetation likelihood", "vegetation_likelihood"),
        &file_params,
        ref_point,
        map_params.output.crs.as_ref(),
    )?;
    write_saved_rasters(
        &sender,
        saved_filtered_surface_rasters,
        ("vegetation-filtered surface", "filtered_surface"),
        &file_params,
        ref_point,
        map_params.output.crs.as_ref(),
    )?;
    write_saved_rasters(
        &sender,
        saved_ndvd_rasters,
        ("NDVD", "ndvd"),
        &file_params,
        ref_point,
        map_params.output.crs.as_ref(),
    )?;
    write_saved_rasters(
        &sender,
        saved_point_density_rasters,
        ("lidar point density", "point_density"),
        &file_params,
        ref_point,
        map_params.output.crs.as_ref(),
    )?;
    write_saved_rasters(
        &sender,
        saved_flow_accumulation_rasters,
        ("flow accumulation", "flow_accumulation"),
        &file_params,
        ref_point,
        map_params.output.crs.as_ref(),
    )?;
    write_saved_rasters(
        &sender,
        saved_building_height_rasters,
        ("building height", "building_height"),
        &file_params,
        ref_point,
        map_params.output.crs.as_ref(),
    )?;
    write_saved_rasters(
        &sender,
        saved_building_planarity_rasters,
        ("building planarity", "building_planarity"),
        &file_params,
        ref_point,
        map_params.output.crs.as_ref(),
    )?;
    write_saved_rasters(
        &sender,
        saved_building_residual_rasters,
        ("building plane residual", "building_residual"),
        &file_params,
        ref_point,
        map_params.output.crs.as_ref(),
    )?;
    write_saved_rasters(
        &sender,
        saved_building_probability_rasters,
        ("building probability", "building_probability"),
        &file_params,
        ref_point,
        map_params.output.crs.as_ref(),
    )?;
    write_saved_viewer_rasters(
        &sender,
        saved_building_plane_rejected_rasters,
        ("rejected building-plane mask", "building_plane_rejected"),
        &file_params,
        ref_point,
        map_params.output.crs.as_ref(),
    )?;

    let _ = sender.send(FrontendTask::Log("Done!".to_string()));
    Ok(())
}

fn push_saved_raster<T: RasterMarker>(
    saved_rasters: &Arc<Mutex<Vec<Dfm<T>>>>,
    raster: Dfm<T>,
    label: &str,
    sender: &FrontendSender,
) -> bool {
    if let Ok(mut rasters) = saved_rasters.lock() {
        rasters.push(raster);
        true
    } else {
        let _ = sender.send(FrontendTask::Error(
            format!("{label} raster mutex was poisoned"),
            true,
        ));
        false
    }
}

fn write_saved_rasters<T: RasterMarker>(
    sender: &FrontendSender,
    saved_rasters: Option<Arc<Mutex<Vec<Dfm<T>>>>>,
    naming: (&str, &str),
    file_params: &FileParameters,
    ref_point: geo::Coord,
    crs: Option<&proj_core::CrsDef>,
) -> Result<()> {
    let (label, suffix) = naming;
    let Some(saved_rasters) = saved_rasters else {
        return Ok(());
    };

    let rasters = Arc::<Mutex<Vec<Dfm<T>>>>::into_inner(saved_rasters)
        .with_context(|| {
            format!("Could not get saved {label} rasters; a worker still holds a reference")
        })?
        .into_inner()
        .map_err(|_| anyhow::anyhow!("{label} raster mutex was poisoned during generation"))?;

    if rasters.is_empty() {
        return Ok(());
    }

    let raw_values = file_params.write_raw_raster_values;
    let encoding = if raw_values {
        "raw-valued float32"
    } else {
        "viewer-scaled 8-bit"
    };
    let _ = sender.send(FrontendTask::Log(format!(
        "Writing {label} {encoding} GeoTIFF..."
    )));
    let path = if raw_values {
        crate::raster::geotiff::write_merged_dfm_geotiff_f32(
            &file_params.save_location,
            suffix,
            &rasters,
            ref_point,
            crs,
        )?
    } else {
        crate::raster::geotiff::write_merged_dfm_geotiff(
            &file_params.save_location,
            suffix,
            &rasters,
            ref_point,
            crs,
        )?
    };
    let _ = sender.send(FrontendTask::Log(format!(
        "Wrote {label} raster to {}",
        path.display()
    )));

    Ok(())
}

/// Display-only rasters deliberately bypass the raw-value toggle. Their
/// samples are render products or categorical diagnostics rather than a
/// physical/numeric field that downstream GIS analysis should consume.
fn write_saved_viewer_rasters<T: RasterMarker>(
    sender: &FrontendSender,
    saved_rasters: Option<Arc<Mutex<Vec<Dfm<T>>>>>,
    naming: (&str, &str),
    file_params: &FileParameters,
    ref_point: geo::Coord,
    crs: Option<&proj_core::CrsDef>,
) -> Result<()> {
    let (label, suffix) = naming;
    let Some(saved_rasters) = saved_rasters else {
        return Ok(());
    };

    let rasters = Arc::<Mutex<Vec<Dfm<T>>>>::into_inner(saved_rasters)
        .with_context(|| {
            format!("Could not get saved {label} rasters; a worker still holds a reference")
        })?
        .into_inner()
        .map_err(|_| anyhow::anyhow!("{label} raster mutex was poisoned during generation"))?;

    if rasters.is_empty() {
        return Ok(());
    }

    let _ = sender.send(FrontendTask::Log(format!("Writing {label} GeoTIFF...")));
    let path = crate::raster::geotiff::write_merged_dfm_geotiff(
        &file_params.save_location,
        suffix,
        &rasters,
        ref_point,
        crs,
    )?;
    let _ = sender.send(FrontendTask::Log(format!(
        "Wrote {label} raster to {}",
        path.display()
    )));

    Ok(())
}
