use std::{
    cmp::Ordering,
    sync::{
        Arc, Mutex,
        atomic::{AtomicU64, Ordering as AtomicOrdering},
    },
};

use geo::{Area, BooleanOps};

use crate::{
    generation::{self, features::ComputedDfms},
    geometry::PointCloud,
    inference::models::ditches_streams_svf_slope as stream_model,
    lidar::LidarStats,
    map::MapObject,
    parameters::{BoulderParameters, BuildingParameters, MapParameters, VegetationWeights},
    raster::{
        BuildingProbability, ContourTerrain, D8Flow, Dfm, Elevation, FilteredSurface, Ground,
        GroundPointDensity, GroundRelief2m, GroundRelief5m, HardObjectConfidence, HardObjectHeight,
        HeightAboveGround, HighVegetation, HydroCorrected, Intensity, LastReturn, LowVegetation,
        MarshHydrology, MediumVegetation, Ndvd, PointDensity, Returns, Slope, SurfaceObjects,
        VegetationLikelihood, Water,
    },
};

use super::cache::{BoundedCache, BuildingFitCacheKey, ContourFieldCacheKey, TerrainFitCacheKey};

static NEXT_TILE_REVISION: AtomicU64 = AtomicU64::new(1);

pub struct TileRasters {
    pub dem: Dfm<Elevation>,
    pub contour_terrain: Dfm<ContourTerrain>,
    pub slope: Dfm<Slope>,
    pub return_number: Dfm<Returns>,
    pub intensity: Dfm<Intensity>,
    pub last_return: Dfm<LastReturn>,
    pub ground_vegetation: Dfm<Ground>,
    pub low_vegetation: Dfm<LowVegetation>,
    pub medium_vegetation: Dfm<MediumVegetation>,
    pub high_vegetation: Dfm<HighVegetation>,
    pub surface_objects: Dfm<SurfaceObjects>,
    pub ground_relief_2m: Dfm<GroundRelief2m>,
    pub ground_relief_5m: Dfm<GroundRelief5m>,
    pub hard_object_height: Dfm<HardObjectHeight>,
    pub hard_object_confidence: Dfm<HardObjectConfidence>,
    pub vegetation_likelihood: Dfm<VegetationLikelihood>,
    pub filtered_surface: Dfm<FilteredSurface>,
    pub water: Dfm<Water>,
    pub canopy_height: Dfm<HeightAboveGround>,
    pub point_density: Dfm<PointDensity>,
    pub ground_point_density: Dfm<GroundPointDensity>,
    pub hydro_corrected: Dfm<HydroCorrected>,
    pub stream_flow: D8Flow,
}

impl<'a> From<&'a TileRasters> for stream_model::Input<'a> {
    fn from(rasters: &'a TileRasters) -> Self {
        Self { dem: &rasters.dem }
    }
}

pub struct PreparedTile {
    pub rasters: TileRasters,
    pub hull: geo::Polygon,
    pub cut_overlay: geo::Polygon,
    pub z_range: (f32, f32),
    revision: u64,
    terrain_fit_cache: Mutex<
        BoundedCache<TerrainFitCacheKey, generation::features::contours::field::FittedTerrain, 2>,
    >,
    contour_field_cache:
        Mutex<BoundedCache<ContourFieldCacheKey, generation::features::ProducedContourField, 2>>,
    source_cloud: Option<Arc<PointCloud>>,
    boulder_evidence_cache: Mutex<BoundedCache<u64, generation::features::BoulderEvidence, 2>>,
    building_fit_cache:
        Mutex<BoundedCache<BuildingFitCacheKey, generation::features::BuildingSurfaceFit, 2>>,
    building_detection_cache:
        Mutex<BoundedCache<(u64, BuildingParameters), generation::features::BuildingDetection, 2>>,
    marsh_hydrology_cache: Mutex<BoundedCache<u32, MarshHydrology, 2>>,
    prediction_cache: Mutex<stream_model::PredictionCache>,
}

pub struct DeferredHydrologyTile {
    pub dem: Dfm<Elevation>,
    pub water: Dfm<Water>,
    pub point_density: Dfm<PointDensity>,
    pub ground_point_density: Dfm<GroundPointDensity>,
    pub hydro_corrected: Dfm<HydroCorrected>,
    pub stream_flow: D8Flow,
    pub hull: geo::Polygon,
    pub cut_overlay: geo::Polygon,
    pub(super) building_mask: Option<Dfm<BuildingProbability>>,
    pub(super) building_exclusions: geo::MultiPolygon,
}

impl PreparedTile {
    pub fn new(dfms: ComputedDfms, hull: geo::Polygon, cut_overlay: geo::Polygon) -> Self {
        let ComputedDfms {
            dem,
            return_number,
            intensity,
            last_return,
            ground_vegetation,
            low_vegetation,
            medium_vegetation,
            high_vegetation,
            surface_objects,
            ground_relief_2m,
            ground_relief_5m,
            hard_object_height,
            hard_object_confidence,
            vegetation_likelihood,
            filtered_surface,
            water,
            canopy_height,
            point_density,
            ground_point_density,
            z_range,
        } = dfms;

        let hydro_corrected = dem.hydrological_correction();
        let stream_flow = dem.hydrological_analysis_with_corrected(&hydro_corrected);
        let contour_terrain = generation::features::contour_terrain(&dem);

        Self {
            rasters: TileRasters {
                slope: dem.slope(),
                contour_terrain,
                dem,
                return_number,
                intensity,
                last_return,
                ground_vegetation,
                low_vegetation,
                medium_vegetation,
                high_vegetation,
                surface_objects,
                ground_relief_2m,
                ground_relief_5m,
                hard_object_height,
                hard_object_confidence,
                vegetation_likelihood,
                filtered_surface,
                water,
                canopy_height,
                point_density,
                ground_point_density,
                hydro_corrected,
                stream_flow,
            },
            hull,
            cut_overlay,
            z_range,
            revision: NEXT_TILE_REVISION.fetch_add(1, AtomicOrdering::Relaxed),
            terrain_fit_cache: Mutex::default(),
            contour_field_cache: Mutex::default(),
            source_cloud: None,
            boulder_evidence_cache: Mutex::default(),
            building_fit_cache: Mutex::default(),
            building_detection_cache: Mutex::default(),
            marsh_hydrology_cache: Mutex::default(),
            prediction_cache: Mutex::default(),
        }
    }

    pub fn with_source_cloud(mut self, cloud: PointCloud) -> Self {
        self.source_cloud = Some(Arc::new(cloud));
        self
    }

    pub fn from_cloud(
        ground_cloud: PointCloud,
        all_point_cloud: PointCloud,
        stats: &LidarStats,
        convex_hull: geo::Polygon,
        cut_bounds: geo::Rect,
    ) -> crate::Result<Option<Self>> {
        let mut mp = cut_bounds.to_polygon().intersection(&convex_hull);
        if mp.0.is_empty() {
            return Ok(None);
        }

        mp.0.sort_by(|a, b| {
            a.signed_area()
                .partial_cmp(&b.signed_area())
                .unwrap_or(Ordering::Equal)
        });

        let dfms =
            generation::features::compute_dfms(ground_cloud, stats, &all_point_cloud, cut_bounds)?;
        Ok(Some(
            Self::new(dfms, convex_hull, mp.0.swap_remove(0)).with_source_cloud(all_point_cloud),
        ))
    }

    pub(super) fn produced_contour_field(
        &self,
        params: &MapParameters,
    ) -> crate::Result<Arc<generation::features::ProducedContourField>> {
        let key = ContourFieldCacheKey::new(self.revision, params);
        let mut cache = self
            .contour_field_cache
            .lock()
            .expect("contour-field cache poisoned");
        if let Some(artifact) = cache.get(&key) {
            return Ok(artifact);
        }
        let fitted = self.fitted_terrain(params)?;
        let artifact = Arc::new(
            generation::features::produce_scalar_contour_field_from_fitted(
                &self.rasters.dem,
                params,
                &fitted,
            )?,
        );
        cache.insert(key, Arc::clone(&artifact));
        Ok(artifact)
    }

    pub(super) fn fitted_terrain(
        &self,
        params: &MapParameters,
    ) -> crate::Result<Arc<generation::features::contours::field::FittedTerrain>> {
        let key = TerrainFitCacheKey::new(self.revision, params);
        let mut cache = self
            .terrain_fit_cache
            .lock()
            .expect("terrain-fit cache poisoned");
        if let Some(fitted) = cache.get(&key) {
            return Ok(fitted);
        }
        let fitted = Arc::new(generation::features::contours::field::fit_terrain(
            &self.rasters.dem,
            &params.contour.contour_field,
        )?);
        cache.insert(key, Arc::clone(&fitted));
        Ok(fitted)
    }

    pub fn building_surface_fit(
        &self,
        params: &BuildingParameters,
    ) -> crate::Result<Option<Arc<generation::features::BuildingSurfaceFit>>> {
        let Some(cloud) = &self.source_cloud else {
            return Ok(None);
        };
        let key = BuildingFitCacheKey::new(self.revision, params);
        let mut cache = self
            .building_fit_cache
            .lock()
            .expect("building-fit cache poisoned");
        if let Some(fit) = cache.get(&key) {
            return Ok(Some(fit));
        }
        let fit = Arc::new(generation::features::compute_building_surface_fit(
            cloud,
            &self.rasters.dem,
            params,
        )?);
        cache.insert(key, Arc::clone(&fit));
        Ok(Some(fit))
    }

    pub fn building_detection(
        &self,
        params: &BuildingParameters,
    ) -> crate::Result<Option<Arc<generation::features::BuildingDetection>>> {
        if !params.enabled {
            return Ok(None);
        }
        let mut cache = self
            .building_detection_cache
            .lock()
            .expect("building-detection cache poisoned");
        let key = (self.revision, params.clone());
        if let Some(detection) = cache.get(&key) {
            return Ok(Some(detection));
        }
        drop(cache);
        let Some(fit) = self.building_surface_fit(params)? else {
            return Ok(None);
        };
        let detection = Arc::new(generation::features::detect_buildings(&fit, params));
        let mut cache = self
            .building_detection_cache
            .lock()
            .expect("building-detection cache poisoned");
        cache.insert(key, Arc::clone(&detection));
        Ok(Some(detection))
    }

    pub(super) fn boulder_detection(
        &self,
        params: &BoulderParameters,
    ) -> generation::features::BoulderDetection {
        // Disabling the detector avoids point indexing and surface-layer work.
        if !params.enabled {
            return generation::features::BoulderDetection::default();
        }
        let Some(cloud) = &self.source_cloud else {
            return generation::features::BoulderDetection::default();
        };
        let key = self.revision;
        let mut cache = self
            .boulder_evidence_cache
            .lock()
            .expect("boulder-evidence cache poisoned");
        let evidence = if let Some(evidence) = cache.get(&key) {
            evidence
        } else {
            let evidence = Arc::new(generation::features::compute_boulder_evidence(
                cloud,
                &self.rasters.dem,
            ));
            cache.insert(key, Arc::clone(&evidence));
            evidence
        };
        drop(cache);
        generation::features::detect_boulders(
            &evidence,
            cloud,
            &self.rasters.dem,
            &self.hull,
            &self.cut_overlay,
            params,
        )
    }

    pub fn marsh_hydrology(&self, drainage_area_m2: f32) -> crate::Result<Arc<MarshHydrology>> {
        let key = drainage_area_m2.to_bits();
        let mut cache = self
            .marsh_hydrology_cache
            .lock()
            .expect("marsh-hydrology cache poisoned");
        if let Some(hydrology) = cache.get(&key) {
            return Ok(hydrology);
        }
        drop(cache);
        let hydrology = Arc::new(self.rasters.stream_flow.marsh_hydrology(
            &self.rasters.dem,
            &self.rasters.hydro_corrected,
            drainage_area_m2,
        )?);
        let mut cache = self
            .marsh_hydrology_cache
            .lock()
            .expect("marsh-hydrology cache poisoned");
        cache.insert(key, Arc::clone(&hydrology));
        Ok(hydrology)
    }

    pub fn marsh_detection(
        &self,
        params: &MapParameters,
        water_extent: &Dfm<crate::raster::FloodFill>,
    ) -> crate::Result<generation::features::MarshDetection> {
        let hydrology = self.marsh_hydrology(params.marsh.drainage_initiation_area_m2)?;
        let building_detection = self.building_detection(&params.building)?;
        generation::features::compute_marsh_detection(
            &self.rasters.dem,
            &self.rasters.stream_flow,
            self.rasters.stream_flow.flow_accumulation(),
            &hydrology,
            &self.rasters.point_density,
            &self.rasters.ground_point_density,
            water_extent,
            building_detection
                .as_deref()
                .map(generation::features::BuildingDetection::accepted_mask),
            &params.marsh,
        )
    }

    pub fn prediction(
        &self,
        cancellation: &crate::cancellation::CancellationToken,
    ) -> crate::Result<Arc<stream_model::Prediction>> {
        stream_model::predict(
            &self.prediction_cache,
            self.revision,
            (&self.rasters).into(),
            cancellation,
        )
    }

    pub fn compute_model_streams(
        &self,
        cut_overlay: &geo::Polygon,
        params: &MapParameters,
        cancellation: &crate::cancellation::CancellationToken,
    ) -> crate::Result<Vec<MapObject>> {
        cancellation.check()?;
        let prediction = self.prediction(cancellation)?;
        cancellation.check()?;
        let objects =
            generation::features::streams::stream_features(&prediction, cut_overlay, params)?;
        cancellation.check()?;
        Ok(objects)
    }

    pub fn into_deferred_hydrology(
        self,
        params: &MapParameters,
    ) -> crate::Result<DeferredHydrologyTile> {
        let building_detection = self.building_detection(&params.building)?;
        let building_mask = building_detection
            .as_deref()
            .map(generation::features::BuildingDetection::accepted_mask)
            .cloned();
        let building_exclusions = building_detection
            .as_deref()
            .map(|detection| {
                geo::MultiPolygon::new(
                    generation::features::building_objects(
                        detection,
                        &self.hull,
                        &self.cut_overlay,
                        &params.building,
                        &params.geometry.buildings,
                    )
                    .into_iter()
                    .filter_map(|object| match object {
                        MapObject::Area { object, .. } => Some(object),
                        _ => None,
                    })
                    .collect(),
                )
            })
            .unwrap_or_else(|| geo::MultiPolygon::new(Vec::new()));
        let TileRasters {
            dem,
            water,
            point_density,
            ground_point_density,
            hydro_corrected,
            stream_flow,
            ..
        } = self.rasters;
        Ok(DeferredHydrologyTile {
            dem,
            water,
            point_density,
            ground_point_density,
            hydro_corrected,
            stream_flow,
            hull: self.hull,
            cut_overlay: self.cut_overlay,
            building_mask,
            building_exclusions,
        })
    }
}

impl TileRasters {
    pub fn compute_ndvd(&self, weights: VegetationWeights) -> Dfm<Ndvd> {
        generation::features::compute_ndvd(
            &self.ground_vegetation,
            &self.low_vegetation,
            &self.medium_vegetation,
            &self.high_vegetation,
            weights,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        geometry::PointLaz,
        raster::{DfmGrid, RasterMarker},
    };

    fn zero<T: RasterMarker>(dem: &Dfm<Elevation>) -> Dfm<T> {
        let mut raster = Dfm::new_like(dem);
        raster.field.fill(0.);
        raster
    }

    fn test_tile() -> PreparedTile {
        // PreparedTile's hydrology preparation currently uses standard tiles.
        let grid = DfmGrid::standard(geo::coord! { x: 0., y: 255.5 });
        let mut dem = Dfm::<Elevation>::new(grid);
        dem.field.fill(0.);
        let dfms = ComputedDfms {
            return_number: zero(&dem),
            intensity: zero(&dem),
            last_return: zero(&dem),
            ground_vegetation: zero(&dem),
            low_vegetation: zero(&dem),
            medium_vegetation: zero(&dem),
            high_vegetation: zero(&dem),
            surface_objects: zero(&dem),
            ground_relief_2m: zero(&dem),
            ground_relief_5m: zero(&dem),
            hard_object_height: zero(&dem),
            hard_object_confidence: zero(&dem),
            vegetation_likelihood: zero(&dem),
            filtered_surface: zero(&dem),
            water: zero(&dem),
            canopy_height: zero(&dem),
            point_density: zero(&dem),
            ground_point_density: zero(&dem),
            z_range: (0., 0.),
            dem,
        };
        let mut points = Vec::new();
        for y in 13..=17 {
            for x in 13..=17 {
                for offset in [-0.1, 0.1] {
                    let mut point =
                        PointLaz::new(125. + x as f64 * 0.5 + offset, 125. + y as f64 * 0.5, 2.5);
                    point.0.classification = las::point::Classification::Unclassified;
                    points.push(point);
                }
            }
        }
        for y in 8..=22 {
            for x in 8..=22 {
                if (13..=17).contains(&x) && (13..=17).contains(&y) {
                    continue;
                }
                points.push(PointLaz::new(
                    125. + x as f64 * 0.5,
                    125. + y as f64 * 0.5,
                    0.,
                ));
            }
        }
        let bounds = las::Bounds {
            min: las::Vector {
                x: 0.,
                y: 0.,
                z: 0.,
            },
            max: las::Vector {
                x: 256.,
                y: 256.,
                z: 3.,
            },
        };
        let hull = geo::Rect::new((0., 0.), (256., 256.)).to_polygon();
        PreparedTile::new(dfms, hull.clone(), hull)
            .with_source_cloud(PointCloud::new(points, bounds))
    }

    #[test]
    fn disabled_generation_steps_skip_work_and_clear_preview_layers() {
        use crate::{
            cancellation::CancellationToken,
            generation::{
                pipeline::{PipelineSteps, compute_tile},
                preview::{MapPreviewSection, RegenerationScope, regenerate_map_tile},
            },
            map::{AreaSymbol, LineSymbol, PointSymbol, Symbol},
            parameters::StreamAlgorithm,
        };

        let tile = test_tile();
        let mut old = MapParameters::default();
        old.intensity.filters.push(Default::default());
        let mut params = old.clone();
        params.contour.enabled = false;
        params.vegetation.openness_enabled = false;
        params.vegetation.enabled = false;
        params.building.enabled = false;
        params.cliff.enabled = false;
        params.boulder.enabled = false;
        params.water.enabled = false;
        params.marsh.enabled = false;
        params.streams.enabled = false;
        params.intensity.enabled = false;
        // A disabled algorithm must not be validated, fitted, or loaded.
        params.contour.algorithm = crate::parameters::ContourAlgo::WeightedScalarField;
        params.streams.algorithm = StreamAlgorithm::DitchesStreamsSvfSlope;
        let output = compute_tile(
            &tile,
            &params,
            PipelineSteps {
                basemap: true,
                contours: true,
                openness: true,
                vegetation: true,
                buildings: true,
                cliffs: true,
                boulders: true,
                intensity: true,
                water: true,
                marsh: true,
                streams: true,
            },
            true,
        )
        .unwrap();
        assert!(output.objects.is_empty());
        assert!(output.boulder_diagnostics.is_none());
        assert!(tile.contour_field_cache.lock().unwrap().entries.is_empty());
        assert!(tile.building_fit_cache.lock().unwrap().entries.is_empty());
        assert!(
            tile.boulder_evidence_cache
                .lock()
                .unwrap()
                .entries
                .is_empty()
        );

        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(1)
            .build()
            .unwrap();
        let update = regenerate_map_tile(
            &pool,
            &[tile],
            geo::coord! { x: 0., y: 0. },
            &params,
            &Some(old),
            RegenerationScope::Changed,
            Some(MapPreviewSection::Intensity),
            &CancellationToken::default(),
        )
        .unwrap()
        .unwrap();
        // Empty entries replace previously generated objects in the preview.
        let cleared: [Symbol; 23] = [
            LineSymbol::BasemapContour.into(),
            LineSymbol::NegBasemapContour.into(),
            LineSymbol::Contour.into(),
            LineSymbol::IndexContour.into(),
            LineSymbol::FormLine.into(),
            PointSymbol::DotKnoll.into(),
            PointSymbol::ElongatedDotKnoll.into(),
            PointSymbol::UDepression.into(),
            PointSymbol::SlopeLineContour.into(),
            PointSymbol::SlopeLineFormLine.into(),
            AreaSymbol::RoughOpenLand.into(),
            AreaSymbol::LightGreen.into(),
            AreaSymbol::MediumGreen.into(),
            AreaSymbol::DarkGreen.into(),
            AreaSymbol::Building.into(),
            AreaSymbol::GiganticBoulder.into(),
            LineSymbol::Cliff.into(),
            LineSymbol::ImpassableCliff.into(),
            PointSymbol::SmallBoulder.into(),
            PointSymbol::LargeBoulder.into(),
            AreaSymbol::UncrossableWaterWithBankLine.into(),
            AreaSymbol::Marsh.into(),
            LineSymbol::SmallCrossableWatercourse.into(),
        ];
        for symbol in cleared.into_iter().chain([AreaSymbol::BareRock.into()]) {
            assert!(
                update
                    .document
                    .objects
                    .get(&symbol)
                    .is_some_and(Vec::is_empty),
                "Missing preview removal for {symbol:?}"
            );
        }
    }

    #[test]
    fn boulder_stage_skips_disabled_work_and_reuses_evidence_for_scoring_changes() {
        use crate::{
            generation::pipeline::{PipelineSteps, compute_tile},
            map::PointSymbol,
        };
        let tile = test_tile();
        let mut params = MapParameters::default();
        let steps = PipelineSteps {
            boulders: true,
            ..Default::default()
        };
        let skipped = compute_tile(&tile, &params, PipelineSteps::default(), false).unwrap();
        assert!(skipped.objects.is_empty());
        assert!(
            tile.boulder_evidence_cache
                .lock()
                .unwrap()
                .entries
                .is_empty()
        );
        params.boulder.enabled = false;
        assert!(
            compute_tile(&tile, &params, steps, false)
                .unwrap()
                .objects
                .is_empty()
        );
        assert!(
            tile.boulder_evidence_cache
                .lock()
                .unwrap()
                .entries
                .is_empty()
        );

        params.boulder.enabled = true;
        let output = compute_tile(&tile, &params, steps, false).unwrap();
        assert_eq!(output.objects.len(), 1);
        assert!(matches!(
            output.objects[0],
            MapObject::Point {
                symbol: PointSymbol::LargeBoulder,
                ..
            }
        ));
        let cached = Arc::clone(&tile.boulder_evidence_cache.lock().unwrap().entries[0].1);
        params.boulder.maximum_height_to_diameter = 0.8;
        params.boulder.maximum_surface_roughness_m = 0.1;
        params.boulder.minimum_top_fraction = 0.4;
        tile.boulder_detection(&params.boulder);
        assert!(Arc::ptr_eq(
            &cached,
            &tile.boulder_evidence_cache.lock().unwrap().entries[0].1
        ));
        params.boulder.minimum_height_m = 1.;
        params.boulder.maximum_nonterminal_fraction = 0.1;
        tile.boulder_detection(&params.boulder);
        assert_eq!(tile.boulder_evidence_cache.lock().unwrap().entries.len(), 1);
        assert!(Arc::ptr_eq(
            &cached,
            &tile.boulder_evidence_cache.lock().unwrap().entries[0].1
        ));
    }
}
