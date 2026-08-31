use crate::{
    generation,
    map::{AreaSymbol, MapObject},
    parameters::{CliffAlgorithm, ContourAlgo, MapParameters},
    raster::Threshold,
};

use super::{
    conflicts::resolve_building_conflicts,
    tile::{DeferredHydrologyTile, PreparedTile},
};

#[cfg(test)]
use super::cache::{BoundedCache, BuildingFitCacheKey, ContourFieldCacheKey, TerrainFitCacheKey};
#[cfg(test)]
use crate::raster::Dfm;
#[cfg(test)]
use geo::{Area, BooleanOps};
#[cfg(test)]
use std::sync::Arc;

pub struct PipelineOutput {
    pub objects: Vec<MapObject>,
    pub contour_error: f32,
    pub contour_energy: f32,
}

pub struct MarshOutput {
    pub detection: generation::features::MarshDetection,
    pub objects: Vec<MapObject>,
}

#[derive(Clone, Copy, Default)]
pub struct PipelineSteps {
    pub basemap: bool,
    pub contours: bool,
    pub openness: bool,
    pub vegetation: bool,
    pub buildings: bool,
    pub cliffs: bool,
    pub intensity: bool,
    pub water: bool,
    pub streams: bool,
    pub marsh: bool,
}

/// Compute marsh diagnostics and polygons from the current (possibly
/// cross-tile reconciled) D8 accumulation field.
pub fn compute_marsh(tile: &PreparedTile, params: &MapParameters) -> crate::Result<MarshOutput> {
    let water_extent = generation::features::compute_water_extent(
        &tile.rasters.water,
        &tile.rasters.hydro_corrected,
        &tile.rasters.stream_flow,
        params.water.threshold,
        params.water.elevation_tolerance_m,
        &params.water.seed_buffer_rules,
        params.water.allow_downhill_flow,
    );
    let detection = tile.marsh_detection(params, &water_extent)?;

    // Vector exclusions use the same cartographic buffers as the emitted
    // water/building objects. The raster masks prevent classification inside
    // them; this second pass removes sub-cell marching-squares overlap.
    let mut exclusion_polygons = generation::features::compute_vegetation(
        &water_extent,
        Threshold::Lower(0.5),
        &tile.hull,
        &tile.cut_overlay,
        AreaSymbol::UncrossableWaterWithBankLine,
        params,
        &params.geometry.water.buffer_rules,
    )
    .into_iter()
    .filter_map(|object| match object {
        MapObject::Area { object, .. } => Some(object),
        _ => None,
    })
    .collect::<Vec<_>>();
    if let Some(buildings) = tile.building_detection(&params.building)? {
        exclusion_polygons.extend(
            generation::features::building_objects(
                &buildings,
                &tile.hull,
                &tile.cut_overlay,
                &params.building,
                &params.geometry.buildings.buffer_rules,
            )
            .into_iter()
            .filter_map(|object| match object {
                MapObject::Area { object, .. } => Some(object),
                _ => None,
            }),
        );
    }
    let exclusions = geo::MultiPolygon::new(exclusion_polygons);
    let objects = generation::features::marsh_objects(
        &detection,
        &tile.hull,
        &tile.cut_overlay,
        params,
        &exclusions,
    );
    Ok(MarshOutput { detection, objects })
}

impl DeferredHydrologyTile {
    pub fn compute_marsh(&self, params: &MapParameters) -> crate::Result<MarshOutput> {
        let water_extent = generation::features::compute_water_extent(
            &self.water,
            &self.hydro_corrected,
            &self.stream_flow,
            params.water.threshold,
            params.water.elevation_tolerance_m,
            &params.water.seed_buffer_rules,
            params.water.allow_downhill_flow,
        );
        let hydrology = self.stream_flow.marsh_hydrology(
            &self.dem,
            &self.hydro_corrected,
            params.marsh.drainage_initiation_area_m2,
        )?;
        let detection = generation::features::compute_marsh_detection(
            &self.dem,
            &self.stream_flow,
            self.stream_flow.flow_accumulation(),
            &hydrology,
            &self.point_density,
            &self.ground_point_density,
            &water_extent,
            self.building_mask.as_ref(),
            &params.marsh,
        )?;
        let mut exclusions = self.building_exclusions.clone();
        exclusions.0.extend(
            generation::features::compute_vegetation(
                &water_extent,
                Threshold::Lower(0.5),
                &self.hull,
                &self.cut_overlay,
                AreaSymbol::UncrossableWaterWithBankLine,
                params,
                &params.geometry.water.buffer_rules,
            )
            .into_iter()
            .filter_map(|object| match object {
                MapObject::Area { object, .. } => Some(object),
                _ => None,
            }),
        );
        let objects = generation::features::marsh_objects(
            &detection,
            &self.hull,
            &self.cut_overlay,
            params,
            &exclusions,
        );
        Ok(MarshOutput { detection, objects })
    }
}

pub fn compute_tile(
    tile: &PreparedTile,
    params: &MapParameters,
    steps: PipelineSteps,
    compute_contour_score: bool,
) -> crate::Result<PipelineOutput> {
    compute_tile_cancellable(
        tile,
        params,
        steps,
        compute_contour_score,
        &crate::cancellation::CancellationToken::default(),
    )
}

pub fn compute_tile_cancellable(
    tile: &PreparedTile,
    params: &MapParameters,
    steps: PipelineSteps,
    compute_contour_score: bool,
    cancellation: &crate::cancellation::CancellationToken,
) -> crate::Result<PipelineOutput> {
    cancellation.check()?;
    let mut objects = Vec::new();
    let mut contour_error = 0.;
    let mut contour_energy = 0.;

    if steps.basemap && params.contour.basemap_contour && params.contour.basemap_interval >= 0.1 {
        objects.extend(generation::features::compute_basemap(
            &tile.rasters.contour_terrain,
            tile.z_range,
            &tile.cut_overlay,
            params.contour.basemap_interval,
        ));
    }

    if steps.contours {
        let (contours, error, energy) = match params.contour.algorithm {
            ContourAlgo::NaiveIterations => generation::features::compute_naive_contours(
                &tile.rasters.dem,
                tile.z_range,
                &tile.cut_overlay,
                params,
                compute_contour_score,
            )?,
            ContourAlgo::WeightedScalarField => {
                let produced = tile.produced_contour_field(params)?;
                generation::features::compute_scalar_field_contours_from_produced(
                    &tile.rasters.dem,
                    tile.z_range,
                    &tile.cut_overlay,
                    params,
                    compute_contour_score,
                    &produced,
                )?
            }
            ContourAlgo::NormalFieldSmoothing | ContourAlgo::Raw => {
                generation::features::extract_contours(
                    &tile.rasters.dem,
                    &tile.rasters.contour_terrain,
                    tile.z_range,
                    &tile.cut_overlay,
                    params,
                    compute_contour_score,
                )?
            }
        };
        objects.extend(contours);
        contour_error = error;
        contour_energy = energy;
    }

    if steps.openness {
        objects.extend(generation::features::compute_vegetation(
            &tile.rasters.return_number,
            Threshold::Upper(params.vegetation.yellow),
            &tile.hull,
            &tile.cut_overlay,
            AreaSymbol::RoughOpenLand,
            params,
            &params.geometry.openness.buffer_rules,
        ));
    }

    if steps.vegetation {
        let ndvd = tile.rasters.compute_ndvd(params.vegetation.weights);
        for (threshold, symbol) in [
            (params.vegetation.green.0, AreaSymbol::LightGreen),
            (params.vegetation.green.1, AreaSymbol::MediumGreen),
            (params.vegetation.green.2, AreaSymbol::DarkGreen),
        ] {
            objects.extend(generation::features::compute_vegetation(
                &ndvd,
                Threshold::Lower(threshold),
                &tile.hull,
                &tile.cut_overlay,
                symbol,
                params,
                &params.geometry.vegetation.buffer_rules,
            ));
        }
    }

    if steps.buildings
        && let Some(detection) = tile.building_detection(&params.building)?
    {
        objects.extend(generation::features::building_objects(
            &detection,
            &tile.hull,
            &tile.cut_overlay,
            &params.building,
            &params.geometry.buildings.buffer_rules,
        ));
    }

    if steps.cliffs {
        let cliffs = match params.cliff.algorithm {
            CliffAlgorithm::SobelSlope => generation::features::compute_cliffs(
                &tile.rasters.slope,
                &tile.hull,
                &tile.cut_overlay,
                params,
                &params.geometry.cliffs.buffer_rules,
            ),
            CliffAlgorithm::PolynomialFit => {
                let fitted = tile.fitted_terrain(params)?;
                generation::features::compute_cliffs(
                    &fitted.cliff_strength,
                    &tile.hull,
                    &tile.cut_overlay,
                    params,
                    &params.geometry.cliffs.buffer_rules,
                )
            }
        };
        objects.extend(cliffs);
    }

    if steps.water {
        let water_extent = generation::features::compute_water_extent(
            &tile.rasters.water,
            &tile.rasters.hydro_corrected,
            &tile.rasters.stream_flow,
            params.water.threshold,
            params.water.elevation_tolerance_m,
            &params.water.seed_buffer_rules,
            params.water.allow_downhill_flow,
        );
        objects.extend(generation::features::compute_vegetation(
            &water_extent,
            Threshold::Lower(0.5),
            &tile.hull,
            &tile.cut_overlay,
            AreaSymbol::UncrossableWaterWithBankLine,
            params,
            &params.geometry.water.buffer_rules,
        ));
    }

    if steps.marsh && params.marsh.enabled {
        objects.extend(compute_marsh(tile, params)?.objects);
    }

    if steps.streams {
        objects.extend(generation::features::streams::compute_selected(
            &tile.rasters.stream_flow,
            &tile.cut_overlay,
            params,
            || tile.compute_model_streams(&tile.cut_overlay, params, cancellation),
        )?);
    }

    if steps.intensity {
        objects.extend(generation::features::compute_intensity(
            &tile.rasters.intensity,
            &tile.hull,
            &tile.cut_overlay,
            params,
            &params.geometry.intensity.buffer_rules,
        ));
    }

    resolve_building_conflicts(&mut objects);
    cancellation.check()?;

    Ok(PipelineOutput {
        objects,
        contour_error,
        contour_energy,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::generation::features::contours::field::{
        ContourFieldDiagnostics, ContourFieldStageTimings, ContourSolverDiagnostics,
        PersistenceDiagnostics, PersistenceWork, PublishedFieldDiagnostics,
    };
    use crate::raster::AdjustedElevation;
    use crate::raster::DfmGrid;

    fn artifact(value: f32) -> Arc<generation::features::ProducedContourField> {
        let grid = DfmGrid::new(2, 2, 0.5, geo::coord! { x: 0., y: 0. }).unwrap();
        let mut adjusted = Dfm::<AdjustedElevation>::new(grid);
        adjusted.field.fill(value);
        Arc::new(generation::features::ProducedContourField {
            adjusted: Arc::new(adjusted),
            diagnostics: Arc::new(ContourFieldDiagnostics {
                solver: ContourSolverDiagnostics { iterations: 0 },
                published: PublishedFieldDiagnostics {
                    fidelity_energy: 0.,
                    weighted_tv_energy: 0.,
                    alignment_energy: 0.,
                    hessian_energy: 0.,
                    maximum_adjustment: 0.,
                    rms_adjustment: 0.,
                    fraction_at_bound: 0.,
                },
                persistence: PersistenceDiagnostics {
                    requested: 0,
                    verified_removed: 0,
                    preserved: 0,
                    unresolved: 0,
                    target_work: PersistenceWork::default(),
                    publication_work: PersistenceWork::default(),
                },
                timings: ContourFieldStageTimings::default(),
                protected_features: Vec::new(),
                debug_rasters: None,
            }),
        })
    }

    #[test]
    fn contour_field_key_ignores_downstream_parameters() {
        let params = MapParameters::default();
        let expected = ContourFieldCacheKey::new(7, &params);

        let mut downstream = params.clone();
        downstream.contour.form_lines = !downstream.contour.form_lines;
        downstream.contour.form_line_prune_threshold += 1.;
        downstream.contour.dot_knoll_area.0 += 1.;
        downstream.geometry.contours.error += 1.;
        assert_eq!(ContourFieldCacheKey::new(7, &downstream), expected);

        let mut field_change = params.clone();
        field_change.contour.contour_field.fidelity_weight += 1.;
        assert_ne!(ContourFieldCacheKey::new(7, &field_change), expected);
        let mut interval_change = params.clone();
        interval_change.contour.interval += 1.;
        assert_ne!(ContourFieldCacheKey::new(7, &interval_change), expected);
        assert_ne!(ContourFieldCacheKey::new(8, &params), expected);
    }

    #[test]
    fn terrain_fit_key_tracks_only_polynomial_fit_inputs() {
        let params = MapParameters::default();
        let expected = TerrainFitCacheKey::new(7, &params);

        let mut downstream = params.clone();
        downstream.cliff.cliff += 1.;
        downstream.contour.interval += 1.;
        downstream.contour.contour_field.slope_epsilon += 1.;
        assert_eq!(TerrainFitCacheKey::new(7, &downstream), expected);

        let mut slope_radius = params.clone();
        slope_radius.contour.contour_field.slope_fit_radius_m += 1.;
        assert_ne!(TerrainFitCacheKey::new(7, &slope_radius), expected);

        let mut curvature_radius = params.clone();
        curvature_radius
            .contour
            .contour_field
            .curvature_fit_radius_m += 1.;
        assert_ne!(TerrainFitCacheKey::new(7, &curvature_radius), expected);
        assert_ne!(TerrainFitCacheKey::new(8, &params), expected);
    }

    #[test]
    fn building_fit_key_tracks_only_fit_inputs() {
        let params = MapParameters::default();
        let expected = BuildingFitCacheKey::new(7, &params.building);

        let mut threshold_change = params.building.clone();
        threshold_change.confidence_threshold += 0.1;
        threshold_change.minimum_building_area_m2 += 10.;
        assert_eq!(BuildingFitCacheKey::new(7, &threshold_change), expected);

        let mut residual_change = params.building.clone();
        residual_change.maximum_plane_residual_m += 0.1;
        assert_ne!(BuildingFitCacheKey::new(7, &residual_change), expected);

        let mut radius_change = params.building.clone();
        radius_change.plane_fit_radius_m += 0.5;
        assert_ne!(BuildingFitCacheKey::new(7, &radius_change), expected);
        assert_ne!(BuildingFitCacheKey::new(8, &params.building), expected);
    }

    #[test]
    fn buildings_override_vegetation_and_enclosed_boulders() {
        use geo::polygon;

        let vegetation = polygon![
            (x: 0., y: 0.), (x: 10., y: 0.), (x: 10., y: 10.), (x: 0., y: 10.)
        ];
        let building = polygon![
            (x: 2., y: 2.), (x: 8., y: 2.), (x: 8., y: 8.), (x: 2., y: 8.)
        ];
        let boulder = polygon![
            (x: 3., y: 3.), (x: 4., y: 3.), (x: 4., y: 4.), (x: 3., y: 4.)
        ];
        let mut objects = vec![
            MapObject::Area {
                object: vegetation,
                symbol: AreaSymbol::DarkGreen,
                tags: Default::default(),
            },
            MapObject::Area {
                object: building.clone(),
                symbol: AreaSymbol::Building,
                tags: Default::default(),
            },
            MapObject::Area {
                object: boulder,
                symbol: AreaSymbol::GiganticBoulder,
                tags: Default::default(),
            },
        ];

        resolve_building_conflicts(&mut objects);

        assert!(!objects.iter().any(|object| matches!(
            object,
            MapObject::Area {
                symbol: AreaSymbol::GiganticBoulder,
                ..
            }
        )));
        for object in &objects {
            if let MapObject::Area {
                object,
                symbol: AreaSymbol::DarkGreen,
                ..
            } = object
            {
                assert_eq!(object.intersection(&building).unsigned_area(), 0.);
            }
        }
    }

    #[test]
    fn contour_field_cache_hits_are_exact_and_lru_bounded() {
        let params = MapParameters::default();
        let keys = [
            ContourFieldCacheKey::new(1, &params),
            ContourFieldCacheKey::new(2, &params),
            ContourFieldCacheKey::new(3, &params),
        ];
        let first = artifact(1.);
        let second = artifact(2.);
        let third = artifact(3.);
        let mut cache = BoundedCache::<
            ContourFieldCacheKey,
            generation::features::ProducedContourField,
            2,
        >::default();
        cache.insert(keys[0], Arc::clone(&first));
        cache.insert(keys[1], Arc::clone(&second));
        let hit = cache.get(&keys[0]).unwrap();
        assert!(Arc::ptr_eq(&hit, &first));
        cache.insert(keys[2], Arc::clone(&third));
        assert!(cache.get(&keys[1]).is_none());
        assert!(Arc::ptr_eq(&cache.get(&keys[0]).unwrap(), &first));
        assert!(Arc::ptr_eq(&cache.get(&keys[2]).unwrap(), &third));
        assert_eq!(cache.entries.len(), 2);
    }
}
