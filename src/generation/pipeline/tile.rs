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
    lidar::LidarStats,
    map::MapObject,
    parameters::{BuildingParameters, MapParameters, VegetationWeights},
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

impl<'a> From<&'a TileRasters> for crate::inference::InputRasters<'a> {
    fn from(rasters: &'a TileRasters) -> Self {
        Self {
            dem: &rasters.dem,
            return_number: &rasters.return_number,
            intensity: &rasters.intensity,
            last_return: &rasters.last_return,
            ground_vegetation: &rasters.ground_vegetation,
            low_vegetation: &rasters.low_vegetation,
            medium_vegetation: &rasters.medium_vegetation,
            high_vegetation: &rasters.high_vegetation,
            ground_relief_2m: &rasters.ground_relief_2m,
            ground_relief_5m: &rasters.ground_relief_5m,
            hard_object_height: &rasters.hard_object_height,
            hard_object_confidence: &rasters.hard_object_confidence,
            vegetation_likelihood: &rasters.vegetation_likelihood,
            filtered_surface: &rasters.filtered_surface,
            water: &rasters.water,
            canopy_height: &rasters.canopy_height,
            point_density: &rasters.point_density,
            ground_point_density: &rasters.ground_point_density,
        }
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
    building_cloud: Option<Arc<PointCloud>>,
    building_fit_cache:
        Mutex<BoundedCache<BuildingFitCacheKey, generation::features::BuildingSurfaceFit, 2>>,
    building_detection_cache:
        Mutex<BoundedCache<(u64, BuildingParameters), generation::features::BuildingDetection, 2>>,
    marsh_hydrology_cache: Mutex<BoundedCache<u32, MarshHydrology, 2>>,
    prediction_cache: Mutex<crate::inference::StreamPredictionCache>,
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
            building_cloud: None,
            building_fit_cache: Mutex::default(),
            building_detection_cache: Mutex::default(),
            marsh_hydrology_cache: Mutex::default(),
            prediction_cache: Mutex::default(),
        }
    }

    pub fn with_building_cloud(mut self, cloud: PointCloud) -> Self {
        self.building_cloud = Some(Arc::new(cloud));
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
            Self::new(dfms, convex_hull, mp.0.swap_remove(0)).with_building_cloud(all_point_cloud),
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
        let Some(cloud) = &self.building_cloud else {
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
    ) -> crate::Result<Arc<crate::inference::StreamPrediction>> {
        crate::inference::predict_stream(
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
                        &params.geometry.buildings.buffer_rules,
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
