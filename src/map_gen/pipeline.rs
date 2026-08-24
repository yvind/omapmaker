use crate::{
    geometry::PointCloud,
    map_gen::{
        self,
        common::ComputedDfms,
        egui_map::{AreaSymbol, MapObject},
    },
    parameters::{ContourAlgo, MapParameters, VegetationWeights},
    raster::{
        D8Flow, Dfm, Threshold,
        dfm::{
            Elevation, Ground, HeightAboveGround, HighVegetation, HydroCorrected, Intensity,
            LastReturn, LowVegetation, MediumVegetation, Ndvd, PointDensity, Returns, Slope,
            SurfaceObjects, Water,
        },
    },
    statistics::LidarStats,
};
use geo::{Area, BooleanOps};
use std::cmp::Ordering;
use std::collections::VecDeque;
use std::sync::atomic::{AtomicU64, Ordering as AtomicOrdering};
use std::sync::{Arc, Mutex};

const CONTOUR_FIELD_CACHE_ENTRIES_PER_TILE: usize = 2;
static NEXT_TILE_REVISION: AtomicU64 = AtomicU64::new(1);

pub struct TileRasters {
    pub dem: Dfm<Elevation>,
    pub slope: Dfm<Slope>,
    pub return_number: Dfm<Returns>,
    pub intensity: Dfm<Intensity>,
    pub last_return: Dfm<LastReturn>,
    pub ground_vegetation: Dfm<Ground>,
    pub low_vegetation: Dfm<LowVegetation>,
    pub medium_vegetation: Dfm<MediumVegetation>,
    pub high_vegetation: Dfm<HighVegetation>,
    pub surface_objects: Dfm<SurfaceObjects>,
    pub water: Dfm<Water>,
    pub canopy_height: Dfm<HeightAboveGround>,
    pub point_density: Dfm<PointDensity>,
    pub hydro_corrected: Dfm<HydroCorrected>,
    pub stream_flow: D8Flow,
}

pub struct PreparedTile {
    pub rasters: TileRasters,
    pub hull: geo::Polygon,
    pub cut_overlay: geo::Polygon,
    pub z_range: (f32, f32),
    revision: u64,
    contour_field_cache: Mutex<ContourFieldCache>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct ContourFieldCacheKey {
    tile_revision: u64,
    interval_bits: u32,
    contour_field_fingerprint: u64,
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
struct ContourFieldCache {
    entries: VecDeque<(
        ContourFieldCacheKey,
        Arc<map_gen::common::ProducedContourField>,
    )>,
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

#[derive(Clone, Copy, Default)]
pub struct PipelineSteps {
    pub basemap: bool,
    pub contours: bool,
    pub openness: bool,
    pub vegetation: bool,
    pub cliffs: bool,
    pub intensity: bool,
    pub water: bool,
    pub streams: bool,
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
            water,
            canopy_height,
            point_density,
            z_range,
        } = dfms;

        let hydro_corrected = dem.hydrological_correction();
        let stream_flow = dem.hydrological_analysis_with_corrected(&hydro_corrected);

        Self {
            rasters: TileRasters {
                slope: dem.slope(),
                dem,
                return_number,
                intensity,
                last_return,
                ground_vegetation,
                low_vegetation,
                medium_vegetation,
                high_vegetation,
                surface_objects,
                water,
                canopy_height,
                point_density,
                hydro_corrected,
                stream_flow,
            },
            hull,
            cut_overlay,
            z_range,
            revision: NEXT_TILE_REVISION.fetch_add(1, AtomicOrdering::Relaxed),
            contour_field_cache: Mutex::new(ContourFieldCache::default()),
        }
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
        Ok(Some(Self::new(dfms, convex_hull, mp.0.swap_remove(0))))
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
        let artifact = Arc::new(map_gen::common::produce_scalar_contour_field(
            &self.rasters.dem,
            params,
        )?);
        cache.insert(key, Arc::clone(&artifact));
        Ok(artifact)
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

pub fn compute_tile(
    tile: &PreparedTile,
    params: &MapParameters,
    steps: PipelineSteps,
    compute_contour_score: bool,
) -> crate::Result<PipelineOutput> {
    let mut objects = Vec::new();
    let mut contour_error = 0.;
    let mut contour_energy = 0.;

    if steps.basemap && params.contour.basemap_contour && params.contour.basemap_interval >= 0.1 {
        objects.extend(map_gen::common::compute_basemap(
            &tile.rasters.dem,
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

    if steps.cliffs {
        objects.extend(map_gen::common::compute_cliffs(
            &tile.rasters.slope,
            &tile.hull,
            &tile.cut_overlay,
            params,
            &params.geometry.cliffs.buffer_rules,
        ));
    }

    if steps.water {
        let water_extent = map_gen::common::compute_water_extent(
            &tile.rasters.water,
            &tile.rasters.hydro_corrected,
            params.water.threshold,
            params.water.elevation_tolerance_m,
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

    if steps.streams {
        objects.extend(map_gen::common::compute_streams(
            &tile.rasters.stream_flow,
            &tile.cut_overlay,
            params,
        ));
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

    Ok(PipelineOutput {
        objects,
        contour_error,
        contour_energy,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::map_gen::common::contour_field::{
        ContourFieldDiagnostics, ContourFieldStageTimings, ContourSolverDiagnostics,
        PersistenceDiagnostics, PersistenceWork, PublishedFieldDiagnostics,
    };
    use crate::raster::DfmGrid;
    use crate::raster::dfm::AdjustedElevation;

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
