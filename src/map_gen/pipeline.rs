use crate::{
    geometry::PointCloud,
    map_gen::{
        self,
        common::ComputedDfms,
        egui_map::{AreaSymbol, MapObject},
    },
    parameters::{
        BuildingParameters, CliffAlgorithm, ContourAlgo, MapParameters, StreamAlgorithm,
        VegetationWeights,
    },
    raster::{
        BuildingProbability, ContourTerrain, D8Flow, Dfm, Elevation, FilteredSurface, Ground,
        GroundPointDensity, GroundRelief2m, GroundRelief5m, HardObjectConfidence, HardObjectHeight,
        HeightAboveGround, HighVegetation, HydroCorrected, Intensity, LastReturn, LowVegetation,
        MarshHydrology, MediumVegetation, Ndvd, PointDensity, Returns, Slope, SurfaceObjects,
        Threshold, VegetationLikelihood, Water,
    },
    statistics::LidarStats,
};
use geo::{Area, BooleanOps};
use std::cmp::Ordering;
use std::collections::VecDeque;
use std::sync::atomic::{AtomicU64, Ordering as AtomicOrdering};
use std::sync::{Arc, Mutex};

const CONTOUR_FIELD_CACHE_ENTRIES_PER_TILE: usize = 2;
const TERRAIN_FIT_CACHE_ENTRIES_PER_TILE: usize = 2;
const BUILDING_FIT_CACHE_ENTRIES_PER_TILE: usize = 2;
const BUILDING_DETECTION_CACHE_ENTRIES_PER_TILE: usize = 2;
const MARSH_HYDROLOGY_CACHE_ENTRIES_PER_TILE: usize = 2;
#[cfg(feature = "deep-learning")]
const PREDICTION_CACHE_ENTRIES_PER_TILE: usize = 2;
static NEXT_TILE_REVISION: AtomicU64 = AtomicU64::new(1);

pub struct TileRasters {
    /// Canonical, unsmoothed ground elevation.
    pub dem: Dfm<Elevation>,
    /// Lightly noise-filtered terrain shared by Raw and basemap contours.
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

pub struct PreparedTile {
    pub rasters: TileRasters,
    pub hull: geo::Polygon,
    pub cut_overlay: geo::Polygon,
    pub z_range: (f32, f32),
    revision: u64,
    terrain_fit_cache: Mutex<TerrainFitCache>,
    contour_field_cache: Mutex<ContourFieldCache>,
    building_cloud: Option<Arc<PointCloud>>,
    building_fit_cache: Mutex<BuildingFitCache>,
    building_detection_cache: Mutex<BuildingDetectionCache>,
    marsh_hydrology_cache: Mutex<MarshHydrologyCache>,
    #[cfg(feature = "deep-learning")]
    prediction_cache: Mutex<PredictionCache>,
}

#[cfg(feature = "deep-learning")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct PredictionCacheKey {
    tile_revision: u64,
    model_id: &'static str,
    onnx_sha256: &'static str,
    manifest_sha256: &'static str,
    contract_version: u32,
}

#[cfg(feature = "deep-learning")]
#[derive(Default)]
struct PredictionCache {
    entries: VecDeque<(
        PredictionCacheKey,
        Arc<crate::feature_extraction::PredictionRaster>,
    )>,
}

#[cfg(feature = "deep-learning")]
impl PredictionCache {
    fn get(
        &mut self,
        key: PredictionCacheKey,
    ) -> Option<Arc<crate::feature_extraction::PredictionRaster>> {
        let position = self
            .entries
            .iter()
            .position(|(candidate, _)| *candidate == key)?;
        let entry = self
            .entries
            .remove(position)
            .expect("cache position exists");
        let prediction = Arc::clone(&entry.1);
        self.entries.push_back(entry);
        Some(prediction)
    }

    fn insert(
        &mut self,
        key: PredictionCacheKey,
        prediction: Arc<crate::feature_extraction::PredictionRaster>,
    ) {
        if self.entries.len() == PREDICTION_CACHE_ENTRIES_PER_TILE {
            self.entries.pop_front();
        }
        self.entries.push_back((key, prediction));
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct TerrainFitCacheKey {
    tile_revision: u64,
    slope_radius_bits: u64,
    curvature_radius_bits: u64,
}

impl TerrainFitCacheKey {
    fn new(tile_revision: u64, params: &MapParameters) -> Self {
        Self {
            tile_revision,
            slope_radius_bits: params.contour.contour_field.slope_fit_radius_m.to_bits(),
            curvature_radius_bits: params
                .contour
                .contour_field
                .curvature_fit_radius_m
                .to_bits(),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct ContourFieldCacheKey {
    tile_revision: u64,
    interval_bits: u32,
    contour_field_fingerprint: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct BuildingFitCacheKey {
    tile_revision: u64,
    minimum_roof_height_bits: u32,
    maximum_roof_height_bits: u32,
    plane_fit_radius_bits: u64,
    maximum_plane_residual_bits: u32,
    ransac_iterations: usize,
    ransac_sample_size: usize,
    minimum_plane_inliers: usize,
    maximum_roof_planes: usize,
    maximum_roof_slope_bits: u32,
    maximum_candidate_hole_area_bits: u64,
    merge_gap_bits: u64,
    class_6_evidence: u8,
}

impl BuildingFitCacheKey {
    fn new(tile_revision: u64, params: &BuildingParameters) -> Self {
        Self {
            tile_revision,
            minimum_roof_height_bits: params.minimum_roof_height_m.to_bits(),
            maximum_roof_height_bits: params.maximum_roof_height_m.to_bits(),
            plane_fit_radius_bits: params.plane_fit_radius_m.to_bits(),
            maximum_plane_residual_bits: params.maximum_plane_residual_m.to_bits(),
            ransac_iterations: params.ransac_iterations,
            ransac_sample_size: params.ransac_sample_size,
            minimum_plane_inliers: params.minimum_plane_inliers,
            maximum_roof_planes: params.maximum_roof_planes,
            maximum_roof_slope_bits: params.maximum_roof_slope_degrees.to_bits(),
            maximum_candidate_hole_area_bits: params.maximum_candidate_hole_area_m2.to_bits(),
            merge_gap_bits: params.merge_gap_m.to_bits(),
            class_6_evidence: match params.class_6_evidence {
                crate::parameters::BuildingClassificationEvidence::Authoritative => 0,
                crate::parameters::BuildingClassificationEvidence::Supporting => 1,
                crate::parameters::BuildingClassificationEvidence::Ignore => 2,
            },
        }
    }
}

impl ContourFieldCacheKey {
    fn new(tile_revision: u64, params: &MapParameters) -> Self {
        Self {
            tile_revision,
            interval_bits: params.contour.interval.to_bits(),
            contour_field_fingerprint: params.contour.contour_field.fingerprint(),
        }
    }
}

#[derive(Default)]
struct TerrainFitCache {
    entries: VecDeque<(
        TerrainFitCacheKey,
        Arc<map_gen::common::contour_field::FittedTerrain>,
    )>,
}

impl TerrainFitCache {
    fn get(
        &mut self,
        key: TerrainFitCacheKey,
    ) -> Option<Arc<map_gen::common::contour_field::FittedTerrain>> {
        let position = self
            .entries
            .iter()
            .position(|(candidate, _)| *candidate == key)?;
        let entry = self
            .entries
            .remove(position)
            .expect("cache position exists");
        let artifact = Arc::clone(&entry.1);
        self.entries.push_back(entry);
        Some(artifact)
    }

    fn insert(
        &mut self,
        key: TerrainFitCacheKey,
        artifact: Arc<map_gen::common::contour_field::FittedTerrain>,
    ) {
        if self.entries.len() == TERRAIN_FIT_CACHE_ENTRIES_PER_TILE {
            self.entries.pop_front();
        }
        self.entries.push_back((key, artifact));
    }
}

#[derive(Default)]
struct ContourFieldCache {
    entries: VecDeque<(
        ContourFieldCacheKey,
        Arc<map_gen::common::ProducedContourField>,
    )>,
}

#[derive(Default)]
struct BuildingFitCache {
    entries: VecDeque<(
        BuildingFitCacheKey,
        Arc<map_gen::common::BuildingSurfaceFit>,
    )>,
}

impl BuildingFitCache {
    fn get(
        &mut self,
        key: BuildingFitCacheKey,
    ) -> Option<Arc<map_gen::common::BuildingSurfaceFit>> {
        let position = self
            .entries
            .iter()
            .position(|(candidate, _)| *candidate == key)?;
        let entry = self
            .entries
            .remove(position)
            .expect("cache position exists");
        let artifact = Arc::clone(&entry.1);
        self.entries.push_back(entry);
        Some(artifact)
    }

    fn insert(
        &mut self,
        key: BuildingFitCacheKey,
        artifact: Arc<map_gen::common::BuildingSurfaceFit>,
    ) {
        if self.entries.len() == BUILDING_FIT_CACHE_ENTRIES_PER_TILE {
            self.entries.pop_front();
        }
        self.entries.push_back((key, artifact));
    }
}

#[derive(Default)]
struct BuildingDetectionCache {
    entries: VecDeque<(
        u64,
        BuildingParameters,
        Arc<map_gen::common::BuildingDetection>,
    )>,
}

#[derive(Default)]
struct MarshHydrologyCache {
    entries: VecDeque<(u32, Arc<MarshHydrology>)>,
}

impl MarshHydrologyCache {
    fn get(&mut self, drainage_area_bits: u32) -> Option<Arc<MarshHydrology>> {
        let position = self
            .entries
            .iter()
            .position(|(candidate, _)| *candidate == drainage_area_bits)?;
        let entry = self
            .entries
            .remove(position)
            .expect("marsh-hydrology cache position exists");
        let artifact = Arc::clone(&entry.1);
        self.entries.push_back(entry);
        Some(artifact)
    }

    fn insert(&mut self, drainage_area_bits: u32, artifact: Arc<MarshHydrology>) {
        if self.entries.len() == MARSH_HYDROLOGY_CACHE_ENTRIES_PER_TILE {
            self.entries.pop_front();
        }
        self.entries.push_back((drainage_area_bits, artifact));
    }
}

impl BuildingDetectionCache {
    fn get(
        &mut self,
        revision: u64,
        params: &BuildingParameters,
    ) -> Option<Arc<map_gen::common::BuildingDetection>> {
        let position =
            self.entries
                .iter()
                .position(|(candidate_revision, candidate_params, _)| {
                    *candidate_revision == revision && candidate_params == params
                })?;
        let entry = self
            .entries
            .remove(position)
            .expect("cache position exists");
        let artifact = Arc::clone(&entry.2);
        self.entries.push_back(entry);
        Some(artifact)
    }

    fn insert(
        &mut self,
        revision: u64,
        params: BuildingParameters,
        artifact: Arc<map_gen::common::BuildingDetection>,
    ) {
        if self.entries.len() == BUILDING_DETECTION_CACHE_ENTRIES_PER_TILE {
            self.entries.pop_front();
        }
        self.entries.push_back((revision, params, artifact));
    }
}

impl ContourFieldCache {
    fn get(
        &mut self,
        key: ContourFieldCacheKey,
    ) -> Option<Arc<map_gen::common::ProducedContourField>> {
        let position = self
            .entries
            .iter()
            .position(|(candidate, _)| *candidate == key)?;
        let entry = self
            .entries
            .remove(position)
            .expect("cache position exists");
        let artifact = Arc::clone(&entry.1);
        self.entries.push_back(entry);
        Some(artifact)
    }

    fn insert(
        &mut self,
        key: ContourFieldCacheKey,
        artifact: Arc<map_gen::common::ProducedContourField>,
    ) {
        if self.entries.len() == CONTOUR_FIELD_CACHE_ENTRIES_PER_TILE {
            self.entries.pop_front();
        }
        self.entries.push_back((key, artifact));
    }
}

pub struct PipelineOutput {
    pub objects: Vec<MapObject>,
    pub contour_error: f32,
    pub contour_energy: f32,
}

pub struct MarshOutput {
    pub detection: map_gen::common::MarshDetection,
    pub objects: Vec<MapObject>,
}

/// Compact set of rasters retained until final-map flow accumulation has been
/// reconciled across tile boundaries. Point clouds, contour products, and
/// unrelated observation rasters are dropped before this is queued.
pub struct DeferredHydrologyTile {
    pub dem: Dfm<Elevation>,
    pub water: Dfm<Water>,
    pub point_density: Dfm<PointDensity>,
    pub ground_point_density: Dfm<GroundPointDensity>,
    pub hydro_corrected: Dfm<HydroCorrected>,
    pub stream_flow: D8Flow,
    pub hull: geo::Polygon,
    pub cut_overlay: geo::Polygon,
    building_mask: Option<Dfm<BuildingProbability>>,
    building_exclusions: geo::MultiPolygon,
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
        let contour_terrain = map_gen::common::contour_terrain(&dem);

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
            terrain_fit_cache: Mutex::new(TerrainFitCache::default()),
            contour_field_cache: Mutex::new(ContourFieldCache::default()),
            building_cloud: None,
            building_fit_cache: Mutex::new(BuildingFitCache::default()),
            building_detection_cache: Mutex::new(BuildingDetectionCache::default()),
            marsh_hydrology_cache: Mutex::new(MarshHydrologyCache::default()),
            #[cfg(feature = "deep-learning")]
            prediction_cache: Mutex::new(PredictionCache::default()),
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
            map_gen::common::compute_dfms(ground_cloud, stats, &all_point_cloud, cut_bounds)?;
        Ok(Some(
            Self::new(dfms, convex_hull, mp.0.swap_remove(0)).with_building_cloud(all_point_cloud),
        ))
    }

    fn produced_contour_field(
        &self,
        params: &MapParameters,
    ) -> crate::Result<Arc<map_gen::common::ProducedContourField>> {
        let key = ContourFieldCacheKey::new(self.revision, params);
        let mut cache = self
            .contour_field_cache
            .lock()
            .expect("contour-field cache poisoned");
        if let Some(artifact) = cache.get(key) {
            return Ok(artifact);
        }
        let fitted = self.fitted_terrain(params)?;
        let artifact = Arc::new(map_gen::common::produce_scalar_contour_field_from_fitted(
            &self.rasters.dem,
            params,
            &fitted,
        )?);
        cache.insert(key, Arc::clone(&artifact));
        Ok(artifact)
    }

    fn fitted_terrain(
        &self,
        params: &MapParameters,
    ) -> crate::Result<Arc<map_gen::common::contour_field::FittedTerrain>> {
        let key = TerrainFitCacheKey::new(self.revision, params);
        let mut cache = self
            .terrain_fit_cache
            .lock()
            .expect("terrain-fit cache poisoned");
        if let Some(fitted) = cache.get(key) {
            return Ok(fitted);
        }
        let fitted = Arc::new(map_gen::common::contour_field::fit_terrain(
            &self.rasters.dem,
            &params.contour.contour_field,
        )?);
        cache.insert(key, Arc::clone(&fitted));
        Ok(fitted)
    }

    pub fn building_surface_fit(
        &self,
        params: &BuildingParameters,
    ) -> crate::Result<Option<Arc<map_gen::common::BuildingSurfaceFit>>> {
        let Some(cloud) = &self.building_cloud else {
            return Ok(None);
        };
        let key = BuildingFitCacheKey::new(self.revision, params);
        let mut cache = self
            .building_fit_cache
            .lock()
            .expect("building-fit cache poisoned");
        if let Some(fit) = cache.get(key) {
            return Ok(Some(fit));
        }
        let fit = Arc::new(map_gen::common::compute_building_surface_fit(
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
    ) -> crate::Result<Option<Arc<map_gen::common::BuildingDetection>>> {
        if !params.enabled {
            return Ok(None);
        }
        let mut cache = self
            .building_detection_cache
            .lock()
            .expect("building-detection cache poisoned");
        if let Some(detection) = cache.get(self.revision, params) {
            return Ok(Some(detection));
        }
        drop(cache);
        let Some(fit) = self.building_surface_fit(params)? else {
            return Ok(None);
        };
        let detection = Arc::new(map_gen::common::detect_buildings(&fit, params));
        let mut cache = self
            .building_detection_cache
            .lock()
            .expect("building-detection cache poisoned");
        cache.insert(self.revision, params.clone(), Arc::clone(&detection));
        Ok(Some(detection))
    }

    pub fn marsh_hydrology(&self, drainage_area_m2: f32) -> crate::Result<Arc<MarshHydrology>> {
        let key = drainage_area_m2.to_bits();
        let mut cache = self
            .marsh_hydrology_cache
            .lock()
            .expect("marsh-hydrology cache poisoned");
        if let Some(hydrology) = cache.get(key) {
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
    ) -> crate::Result<map_gen::common::MarshDetection> {
        let hydrology = self.marsh_hydrology(params.marsh.drainage_initiation_area_m2)?;
        let building_detection = self.building_detection(&params.building)?;
        map_gen::common::compute_marsh_detection(
            &self.rasters.dem,
            &self.rasters.stream_flow,
            self.rasters.stream_flow.flow_accumulation(),
            &hydrology,
            &self.rasters.point_density,
            &self.rasters.ground_point_density,
            water_extent,
            building_detection
                .as_deref()
                .map(map_gen::common::BuildingDetection::accepted_mask),
            &params.marsh,
        )
    }

    #[cfg(feature = "deep-learning")]
    pub fn prediction(
        &self,
        inference: &crate::feature_extraction::InferenceLane,
        cancellation: &crate::comms::messages::CancellationToken,
    ) -> crate::Result<Arc<crate::feature_extraction::PredictionRaster>> {
        cancellation.check()?;
        let descriptor = inference.descriptor();
        let key = PredictionCacheKey {
            tile_revision: self.revision,
            model_id: descriptor.id,
            onnx_sha256: descriptor.onnx_sha256,
            manifest_sha256: descriptor.manifest_sha256,
            contract_version: descriptor.contract_version,
        };
        if let Some(prediction) = self
            .prediction_cache
            .lock()
            .expect("prediction cache poisoned")
            .get(key)
        {
            return Ok(prediction);
        }
        let input = crate::feature_extraction::build_input(&descriptor.input, &self.rasters)?;
        cancellation.check()?;
        let prediction = Arc::new(inference.predict(input, cancellation)?);
        cancellation.check()?;
        self.prediction_cache
            .lock()
            .expect("prediction cache poisoned")
            .insert(key, Arc::clone(&prediction));
        Ok(prediction)
    }

    pub fn into_deferred_hydrology(
        self,
        params: &MapParameters,
    ) -> crate::Result<DeferredHydrologyTile> {
        let building_detection = self.building_detection(&params.building)?;
        let building_mask = building_detection
            .as_deref()
            .map(map_gen::common::BuildingDetection::accepted_mask)
            .cloned();
        let building_exclusions = building_detection
            .as_deref()
            .map(|detection| {
                geo::MultiPolygon::new(
                    map_gen::common::building_objects(
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
        map_gen::common::compute_ndvd(
            &self.ground_vegetation,
            &self.low_vegetation,
            &self.medium_vegetation,
            &self.high_vegetation,
            weights,
        )
    }
}

/// Compute marsh diagnostics and polygons from the current (possibly
/// cross-tile reconciled) D8 accumulation field.
pub fn compute_marsh(tile: &PreparedTile, params: &MapParameters) -> crate::Result<MarshOutput> {
    let water_extent = map_gen::common::compute_water_extent(
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
    let mut exclusion_polygons = map_gen::common::compute_vegetation(
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
            map_gen::common::building_objects(
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
    let objects = map_gen::common::marsh_objects(
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
        let water_extent = map_gen::common::compute_water_extent(
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
        let detection = map_gen::common::compute_marsh_detection(
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
            map_gen::common::compute_vegetation(
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
        let objects = map_gen::common::marsh_objects(
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
        &crate::comms::messages::CancellationToken::default(),
    )
}

pub fn compute_tile_cancellable(
    tile: &PreparedTile,
    params: &MapParameters,
    steps: PipelineSteps,
    compute_contour_score: bool,
    cancellation: &crate::comms::messages::CancellationToken,
) -> crate::Result<PipelineOutput> {
    cancellation.check()?;
    let mut objects = Vec::new();
    let mut contour_error = 0.;
    let mut contour_energy = 0.;

    if steps.basemap && params.contour.basemap_contour && params.contour.basemap_interval >= 0.1 {
        objects.extend(map_gen::common::compute_basemap(
            &tile.rasters.contour_terrain,
            tile.z_range,
            &tile.cut_overlay,
            params.contour.basemap_interval,
        ));
    }

    if steps.contours {
        let (contours, error, energy) = match params.contour.algorithm {
            ContourAlgo::NaiveIterations => map_gen::common::compute_naive_contours(
                &tile.rasters.dem,
                tile.z_range,
                &tile.cut_overlay,
                params,
                compute_contour_score,
            )?,
            ContourAlgo::WeightedScalarField => {
                let produced = tile.produced_contour_field(params)?;
                map_gen::common::compute_scalar_field_contours_from_produced(
                    &tile.rasters.dem,
                    tile.z_range,
                    &tile.cut_overlay,
                    params,
                    compute_contour_score,
                    &produced,
                )?
            }
            ContourAlgo::NormalFieldSmoothing | ContourAlgo::Raw => {
                map_gen::common::extract_contours(
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
        objects.extend(map_gen::common::compute_vegetation(
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
            objects.extend(map_gen::common::compute_vegetation(
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
        objects.extend(map_gen::common::building_objects(
            &detection,
            &tile.hull,
            &tile.cut_overlay,
            &params.building,
            &params.geometry.buildings.buffer_rules,
        ));
    }

    if steps.cliffs {
        let cliffs = match params.cliff.algorithm {
            CliffAlgorithm::SobelSlope => map_gen::common::compute_cliffs(
                &tile.rasters.slope,
                &tile.hull,
                &tile.cut_overlay,
                params,
                &params.geometry.cliffs.buffer_rules,
            ),
            CliffAlgorithm::PolynomialFit => {
                let fitted = tile.fitted_terrain(params)?;
                map_gen::common::compute_cliffs(
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
        let water_extent = map_gen::common::compute_water_extent(
            &tile.rasters.water,
            &tile.rasters.hydro_corrected,
            &tile.rasters.stream_flow,
            params.water.threshold,
            params.water.elevation_tolerance_m,
            &params.water.seed_buffer_rules,
            params.water.allow_downhill_flow,
        );
        objects.extend(map_gen::common::compute_vegetation(
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
        match params.streams.algorithm {
            StreamAlgorithm::Hydrological => {
                objects.extend(map_gen::common::compute_streams(
                    &tile.rasters.stream_flow,
                    &tile.cut_overlay,
                    params,
                ));
            }
            #[cfg(feature = "deep-learning")]
            StreamAlgorithm::DitchesStreamsSvfSlope => {
                cancellation.check()?;
                let inference = crate::feature_extraction::inference_lane()?;
                let prediction = tile.prediction(&inference, cancellation)?;
                cancellation.check()?;
                objects.extend(crate::feature_extraction::postprocess::stream_features(
                    &prediction,
                    &tile.cut_overlay,
                    params,
                )?);
                cancellation.check()?;
            }
        }
    }

    if steps.intensity {
        objects.extend(map_gen::common::compute_intensity(
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

fn resolve_building_conflicts(objects: &mut Vec<MapObject>) {
    let footprints = geo::MultiPolygon::new(
        objects
            .iter()
            .filter_map(|object| match object {
                MapObject::Area {
                    object,
                    symbol: AreaSymbol::Building,
                    ..
                } => Some(object.clone()),
                _ => None,
            })
            .collect(),
    );
    if footprints.0.is_empty() {
        return;
    }

    let mut resolved = Vec::with_capacity(objects.len());
    for object in objects.drain(..) {
        match object {
            MapObject::Area {
                object,
                symbol:
                    symbol @ (AreaSymbol::RoughOpenLand
                    | AreaSymbol::LightGreen
                    | AreaSymbol::MediumGreen
                    | AreaSymbol::DarkGreen
                    | AreaSymbol::Marsh
                    | AreaSymbol::PavedAreaWithBoundary),
                tags,
            } => {
                resolved.extend(object.difference(&footprints).into_iter().map(|object| {
                    MapObject::Area {
                        object,
                        symbol,
                        tags: tags.clone(),
                    }
                }));
            }
            MapObject::Area {
                object,
                symbol: AreaSymbol::GiganticBoulder,
                tags,
            } => {
                let overlap = object.intersection(&footprints).unsigned_area();
                if overlap < object.unsigned_area() * 0.95 {
                    resolved.push(MapObject::Area {
                        object,
                        symbol: AreaSymbol::GiganticBoulder,
                        tags,
                    });
                }
            }
            MapObject::Area {
                object,
                symbol: AreaSymbol::UncrossableWaterWithBankLine,
                tags,
            } => {
                let overlap = object.intersection(&footprints).unsigned_area();
                if overlap > 1. && overlap > object.unsigned_area() * 0.25 {
                    log::warn!(
                        "A detected building overlaps {:.1} m² of mapped water; keeping both for review",
                        overlap
                    );
                }
                resolved.push(MapObject::Area {
                    object,
                    symbol: AreaSymbol::UncrossableWaterWithBankLine,
                    tags,
                });
            }
            _ => resolved.push(object),
        }
    }
    *objects = resolved;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::map_gen::common::contour_field::{
        ContourFieldDiagnostics, ContourFieldStageTimings, ContourSolverDiagnostics,
        PersistenceDiagnostics, PersistenceWork, PublishedFieldDiagnostics,
    };
    use crate::raster::AdjustedElevation;
    use crate::raster::DfmGrid;

    fn artifact(value: f32) -> Arc<map_gen::common::ProducedContourField> {
        let grid = DfmGrid::new(2, 2, 0.5, geo::coord! { x: 0., y: 0. }).unwrap();
        let mut adjusted = Dfm::<AdjustedElevation>::new(grid);
        adjusted.field.fill(value);
        Arc::new(map_gen::common::ProducedContourField {
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
        let mut cache = ContourFieldCache::default();
        cache.insert(keys[0], Arc::clone(&first));
        cache.insert(keys[1], Arc::clone(&second));
        let hit = cache.get(keys[0]).unwrap();
        assert!(Arc::ptr_eq(&hit, &first));
        cache.insert(keys[2], Arc::clone(&third));
        assert!(cache.get(keys[1]).is_none());
        assert!(Arc::ptr_eq(&cache.get(keys[0]).unwrap(), &first));
        assert!(Arc::ptr_eq(&cache.get(keys[2]).unwrap(), &third));
        assert_eq!(cache.entries.len(), CONTOUR_FIELD_CACHE_ENTRIES_PER_TILE);
    }
}
