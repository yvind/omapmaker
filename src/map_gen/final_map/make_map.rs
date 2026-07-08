use crate::{
    Result,
    comms::{FrontendSender, messages::*},
    map_gen::{self, egui_map::TempMap, pipeline::PreparedTile},
    neighbors::NeighborSide,
    parameters::{FileParameters, MapParameters},
    raster::{
        Dfm,
        dfm::{Hillshade, Slope},
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
    let saved_slope_rasters = file_params
        .save_slope_raster
        .then(|| Arc::new(Mutex::new(Vec::<Dfm<Slope>>::new())));
    let saved_hillshade_rasters = file_params
        .save_hillshade_raster
        .then(|| Arc::new(Mutex::new(Vec::<Dfm<Hillshade>>::new())));

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

                let (cloud, mut hull) = match super::read_laz(
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

                let tile = match PreparedTile::from_cloud(cloud, &stats, hull, cut_bounds[tile_i]) {
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

                if let Some(saved_slope_rasters) = &saved_slope_rasters {
                    if let Ok(mut rasters) = saved_slope_rasters.lock() {
                        rasters.push(tile.rasters.slope.clone());
                    } else {
                        let _ = sender.send(FrontendTask::Error(
                            "Slope raster mutex was poisoned".to_string(),
                            true,
                        ));
                        return;
                    }
                }

                if let Some(saved_hillshade_rasters) = &saved_hillshade_rasters {
                    if let Ok(mut rasters) = saved_hillshade_rasters.lock() {
                        rasters.push(tile.rasters.dem.hillshade(3. * std::f64::consts::FRAC_PI_4));
                    } else {
                        let _ = sender.send(FrontendTask::Error(
                            "Hillshade raster mutex was poisoned".to_string(),
                            true,
                        ));
                        return;
                    }
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

    let mut map = Arc::<Mutex<TempMap>>::into_inner(map)
        .context("Could not get inner map value; a worker still holds a reference")?
        .into_inner()
        .map_err(|_| anyhow::anyhow!("Map mutex was poisoned during generation"))?;

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

    let bezier_line_error = map_params.geometry.contours.enabled.then(|| {
        map_params
            .scale
            .meters_to_paper_mm(map_params.geometry.contours.error)
    });
    let omap = map.into_omap(masl, bezier_line_error)?;

    omap.write_to_file(file_params.save_location.clone())?;

    if let Some(saved_slope_rasters) = saved_slope_rasters {
        let rasters = Arc::<Mutex<Vec<Dfm<Slope>>>>::into_inner(saved_slope_rasters)
            .context("Could not get saved slope rasters; a worker still holds a reference")?
            .into_inner()
            .map_err(|_| anyhow::anyhow!("Slope raster mutex was poisoned during generation"))?;
        if !rasters.is_empty() {
            let _ = sender.send(FrontendTask::Log("Writing slope GeoTIFF...".to_string()));
            let path = crate::raster::geotiff::write_merged_dfm_geotiff(
                &file_params.save_location,
                "slope",
                &rasters,
                ref_point,
                map_params.output.crs.as_ref(),
            )?;
            let _ = sender.send(FrontendTask::Log(format!(
                "Wrote slope raster to {}",
                path.display()
            )));
        }
    }

    if let Some(saved_hillshade_rasters) = saved_hillshade_rasters {
        let rasters = Arc::<Mutex<Vec<Dfm<Hillshade>>>>::into_inner(saved_hillshade_rasters)
            .context("Could not get saved hillshade rasters; a worker still holds a reference")?
            .into_inner()
            .map_err(|_| {
                anyhow::anyhow!("Hillshade raster mutex was poisoned during generation")
            })?;
        if !rasters.is_empty() {
            let _ = sender.send(FrontendTask::Log(
                "Writing hillshade GeoTIFF...".to_string(),
            ));
            let path = crate::raster::geotiff::write_merged_dfm_geotiff(
                &file_params.save_location,
                "hillshade",
                &rasters,
                ref_point,
                map_params.output.crs.as_ref(),
            )?;
            let _ = sender.send(FrontendTask::Log(format!(
                "Wrote hillshade raster to {}",
                path.display()
            )));
        }
    }

    let _ = sender.send(FrontendTask::Log("Done!".to_string()));
    Ok(())
}
