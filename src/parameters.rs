use std::{fmt::Display, path::PathBuf};

use proj_core::CrsDef;

use crate::map_gen::egui_map::{AreaSymbol, LineSymbol, Symbol};

#[derive(Clone, Debug, Default)]
pub struct MapParameters {
    pub output: OutputParameters,
    pub scale: Scale,
    pub contour: ContourParameters,
    pub vegetation: VegetationParameters,
    pub building: BuildingParameters,
    pub geometry: GeometryParameters,
    pub intensity: IntensityParameters,
    pub cliff: CliffParameters,
    pub water: WaterParameters,
    pub marsh: MarshParameters,
    pub streams: StreamParameters,
}

#[derive(Clone, Debug, Default)]
pub struct OutputParameters {
    pub crs: Option<CrsDef>,
}

impl MapParameters {
    pub fn min_size_filter_symbols(
        &self,
        openness: bool,
        vegetation: bool,
        buildings: bool,
        cliffs: bool,
        intensity: bool,
        water: bool,
    ) -> Vec<AreaSymbol> {
        let mut symbols = Vec::new();

        if openness && self.geometry.openness.min_size_filter {
            push_unique_area_symbol(&mut symbols, AreaSymbol::RoughOpenLand);
        }

        if vegetation && self.geometry.vegetation.min_size_filter {
            push_unique_area_symbol(&mut symbols, AreaSymbol::LightGreen);
            push_unique_area_symbol(&mut symbols, AreaSymbol::MediumGreen);
            push_unique_area_symbol(&mut symbols, AreaSymbol::DarkGreen);
        }

        if buildings && self.geometry.buildings.min_size_filter {
            push_unique_area_symbol(&mut symbols, AreaSymbol::Building);
        }

        if cliffs && self.geometry.cliffs.min_size_filter {
            push_unique_area_symbol(&mut symbols, AreaSymbol::GiganticBoulder);
        }

        if intensity && self.geometry.intensity.min_size_filter {
            for filter in &self.intensity.filters {
                push_unique_area_symbol(&mut symbols, filter.symbol);
            }
        }

        if water && self.geometry.water.min_size_filter {
            push_unique_area_symbol(&mut symbols, AreaSymbol::UncrossableWaterWithBankLine);
        }
        if water && self.geometry.marsh.min_size_filter {
            push_unique_area_symbol(&mut symbols, AreaSymbol::Marsh);
        }

        symbols
    }
}

fn push_unique_area_symbol(symbols: &mut Vec<AreaSymbol>, symbol: AreaSymbol) {
    if !symbols.contains(&symbol) {
        symbols.push(symbol);
    }
}

#[derive(Clone, Debug)]
pub struct ContourParameters {
    pub algorithm: ContourAlgo,
    pub form_line_prune_algorithm: FormlinePruneAlgo,
    pub basemap_interval: f32,
    pub interval: f32,
    pub dot_knoll_area: (f64, f64),
    pub algo_steps: u8,
    pub algo_lambda: f32,
    pub basemap_contour: bool,
    pub form_lines: bool,
    pub form_line_prune_threshold: f32,
    pub form_line_error_threshold: f32,
    pub contour_field: ContourFieldParameters,
    pub form_line_geometry: FormlineGeometryParameters,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FormlineGeometryParameters {
    pub minimum_open_length_m: f64,
    pub minimum_closed_length_m: f64,
    pub reconnect_gap_m: f64,
    pub closed_seed_length_m: f64,
    pub closed_all_or_none_max_length_m: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ContourFieldParameters {
    pub max_iterations: usize,
    pub generalization: ContourGeneralization,
    pub multiresolution_levels_m: Vec<f64>,
    pub iterations_per_level: Vec<usize>,
    pub convergence_tolerance: f32,
    pub fidelity_weight: f32,
    pub weighted_tv_weight: f32,
    pub alignment_weight: f32,
    pub hessian_weight: f32,
    pub minimum_contour_cost: f32,
    pub minimum_smoothness_weight: f32,
    pub smoothness_scale: f32,
    pub salience_power: f32,
    pub smoothness_power: f32,
    pub slope_fit_radius_m: f64,
    pub curvature_fit_radius_m: f64,
    pub slope_weight: f32,
    pub profile_change_weight: f32,
    pub tangent_change_weight: f32,
    pub slope_reference: f32,
    pub profile_change_reference: f32,
    pub tangent_change_reference: f32,
    pub slope_epsilon: f32,
    pub rmse_reference: f32,
    pub persistence_threshold_fraction: f32,
    pub solver_guard_distance_m: f64,
    pub collect_debug_rasters: bool,
}

impl ContourFieldParameters {
    pub(crate) fn fingerprint(&self) -> u64 {
        fn mix(hash: &mut u64, value: u64) {
            for byte in value.to_le_bytes() {
                *hash ^= u64::from(byte);
                *hash = hash.wrapping_mul(0x100000001b3);
            }
        }
        let mut hash = 0xcbf29ce484222325_u64;
        mix(&mut hash, self.max_iterations as u64);
        mix(
            &mut hash,
            match self.generalization {
                ContourGeneralization::Light => 0,
                ContourGeneralization::Balanced => 1,
                ContourGeneralization::Strong => 2,
            },
        );
        mix(&mut hash, self.multiresolution_levels_m.len() as u64);
        for value in &self.multiresolution_levels_m {
            mix(&mut hash, value.to_bits());
        }
        mix(&mut hash, self.iterations_per_level.len() as u64);
        for &value in &self.iterations_per_level {
            mix(&mut hash, value as u64);
        }
        for value in [
            self.convergence_tolerance,
            self.fidelity_weight,
            self.weighted_tv_weight,
            self.alignment_weight,
            self.hessian_weight,
            self.minimum_contour_cost,
            self.minimum_smoothness_weight,
            self.smoothness_scale,
            self.salience_power,
            self.smoothness_power,
        ] {
            mix(&mut hash, u64::from(value.to_bits()));
        }
        for value in [self.slope_fit_radius_m, self.curvature_fit_radius_m] {
            mix(&mut hash, value.to_bits());
        }
        for value in [
            self.slope_weight,
            self.profile_change_weight,
            self.tangent_change_weight,
            self.slope_reference,
            self.profile_change_reference,
            self.tangent_change_reference,
            self.slope_epsilon,
            self.rmse_reference,
            self.persistence_threshold_fraction,
        ] {
            mix(&mut hash, u64::from(value.to_bits()));
        }
        mix(&mut hash, self.solver_guard_distance_m.to_bits());
        mix(&mut hash, u64::from(self.collect_debug_rasters));
        hash
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ContourGeneralization {
    Light,
    #[default]
    Balanced,
    Strong,
}

impl ContourGeneralization {
    pub(crate) const fn factor(self) -> f32 {
        match self {
            Self::Light => 0.6,
            Self::Balanced => 1.,
            Self::Strong => 1.6,
        }
    }
}

impl Display for ContourGeneralization {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Light => "Light",
            Self::Balanced => "Balanced",
            Self::Strong => "Strong",
        })
    }
}

#[derive(Clone, Debug)]
pub struct VegetationParameters {
    pub green: (f32, f32, f32),
    pub weights: VegetationWeights,
    pub yellow: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VegetationWeights {
    pub low: f32,
    pub medium: f32,
    pub high: f32,
}

/// Physical thresholds used by the LiDAR roof-surface detector.
#[derive(Clone, Debug, PartialEq)]
pub struct BuildingParameters {
    pub enabled: bool,
    pub minimum_roof_height_m: f32,
    pub maximum_roof_height_m: f32,
    pub plane_fit_radius_m: f64,
    pub maximum_plane_residual_m: f32,
    pub minimum_planar_point_fraction: f32,
    pub ransac_iterations: usize,
    pub ransac_sample_size: usize,
    pub minimum_plane_inliers: usize,
    pub maximum_roof_planes: usize,
    pub maximum_roof_slope_degrees: f32,
    pub minimum_building_area_m2: f64,
    pub maximum_candidate_hole_area_m2: f64,
    pub merge_gap_m: f64,
    pub minimum_rectangularity_or_compactness: f32,
    pub maximum_vegetation_fraction: f32,
    pub confidence_threshold: f32,
    pub class_6_evidence: BuildingClassificationEvidence,
    pub regularize_footprints: bool,
    pub regularization_simplification_tolerance_m: f64,
    pub regularization_parallel_threshold_m: f64,
    pub regularization_maximum_boundary_displacement_m: f64,
    pub regularization_maximum_angle_deviation_degrees: f64,
    pub regularization_minimum_supported_edge_fraction: f64,
    pub regularization_minimum_iou: f64,
    pub regularization_allow_45_degree_edges: bool,
    pub regularization_diagonal_bias_degrees: f64,
}

impl Default for BuildingParameters {
    fn default() -> Self {
        Self {
            enabled: true,
            minimum_roof_height_m: 2.,
            maximum_roof_height_m: 40.,
            plane_fit_radius_m: 1.0,
            maximum_plane_residual_m: 0.1,
            minimum_planar_point_fraction: 0.7,
            ransac_iterations: 150,
            ransac_sample_size: 5,
            minimum_plane_inliers: 12,
            maximum_roof_planes: 15,
            maximum_roof_slope_degrees: 60.,
            minimum_building_area_m2: 8.,
            maximum_candidate_hole_area_m2: 4.,
            merge_gap_m: 1.5,
            minimum_rectangularity_or_compactness: 0.35,
            maximum_vegetation_fraction: 0.45,
            confidence_threshold: 0.7,
            class_6_evidence: BuildingClassificationEvidence::Supporting,
            regularize_footprints: true,
            regularization_simplification_tolerance_m: 1.0,
            regularization_parallel_threshold_m: 1.0,
            regularization_maximum_boundary_displacement_m: 3.0,
            regularization_maximum_angle_deviation_degrees: 45.,
            regularization_minimum_supported_edge_fraction: 0.5,
            regularization_minimum_iou: 0.5,
            regularization_allow_45_degree_edges: false,
            regularization_diagonal_bias_degrees: 5.,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum BuildingClassificationEvidence {
    Authoritative,
    #[default]
    Supporting,
    Ignore,
}

impl Display for BuildingClassificationEvidence {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Authoritative => "Authoritative",
            Self::Supporting => "Supporting evidence",
            Self::Ignore => "Ignored",
        })
    }
}

#[derive(Clone, Debug)]
pub struct GeometryParameters {
    pub contours: BezierParameters,
    pub openness: BufferedGeometryParameters,
    pub vegetation: BufferedGeometryParameters,
    pub buildings: BufferedGeometryParameters,
    pub cliffs: BufferedGeometryParameters,
    pub intensity: BufferedGeometryParameters,
    pub water: BufferedGeometryParameters,
    pub marsh: BufferedGeometryParameters,
    pub streams: BezierParameters,
}

impl Default for GeometryParameters {
    fn default() -> Self {
        let buildings = BufferedGeometryParameters {
            bezier: BezierParameters {
                error: 0.25,
                enabled: false,
            },
            buffer_rules: vec![
                BufferRule {
                    direction: BufferDirection::Shrink,
                    amount: 2.,
                },
                BufferRule {
                    direction: BufferDirection::Grow,
                    amount: 2.,
                },
            ],
            min_size_filter: true,
        };

        let mut cliffs = BufferedGeometryParameters::default();
        cliffs.bezier.enabled = false;

        let openness = BufferedGeometryParameters {
            bezier: BezierParameters::default(),
            buffer_rules: vec![
                BufferRule {
                    direction: BufferDirection::Shrink,
                    amount: 2.5,
                },
                BufferRule {
                    direction: BufferDirection::Grow,
                    amount: 5.,
                },
                BufferRule {
                    direction: BufferDirection::Shrink,
                    amount: 2.5,
                },
            ],
            min_size_filter: true,
        };

        let vegetation = BufferedGeometryParameters {
            bezier: BezierParameters::default(),
            buffer_rules: vec![
                BufferRule {
                    direction: BufferDirection::Grow,
                    amount: 1.,
                },
                BufferRule {
                    direction: BufferDirection::Shrink,
                    amount: 2.5,
                },
                BufferRule {
                    direction: BufferDirection::Grow,
                    amount: 5.,
                },
                BufferRule {
                    direction: BufferDirection::Shrink,
                    amount: 2.5,
                },
            ],
            min_size_filter: true,
        };

        Self {
            contours: Default::default(),
            openness,
            vegetation,
            buildings,
            cliffs,
            intensity: Default::default(),
            water: Default::default(),
            marsh: Default::default(),
            streams: Default::default(),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct WaterParameters {
    /// Minimum likelihood needed for a cell to seed a water region.
    pub threshold: f32,
    /// Ordered raster-mask buffers applied to thresholded seeds before the
    /// elevation flood fill.
    pub seed_buffer_rules: Vec<BufferRule>,
    /// Maximum elevation difference from a seed across a water region.
    pub elevation_tolerance_m: f32,
    /// Continue a filled water region through the existing D8 receivers.
    pub allow_downhill_flow: bool,
}

impl Default for WaterParameters {
    fn default() -> Self {
        Self {
            threshold: 0.70,
            seed_buffer_rules: Vec::new(),
            elevation_tolerance_m: 0.05,
            allow_downhill_flow: false,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct StreamParameters {
    pub algorithm: StreamAlgorithm,
    /// Minimum upstream area needed to initiate a hydrological stream.
    pub minimum_catchment_area_m2: f32,
    /// Raster-to-vector controls used by the ONNX stream detector.
    pub onnx_vectorization: OnnxStreamVectorizationParameters,
}

impl Default for StreamParameters {
    fn default() -> Self {
        Self {
            algorithm: StreamAlgorithm::Hydrological,
            minimum_catchment_area_m2: 10_000.0,
            onnx_vectorization: Default::default(),
        }
    }
}

impl StreamParameters {
    pub(crate) fn endpoint_merge_distance_m(&self) -> f64 {
        #[cfg(feature = "deep-learning")]
        if self.algorithm == StreamAlgorithm::DitchesStreamsSvfSlope {
            return self.onnx_vectorization.endpoint_merge_distance_m;
        }

        5. * crate::SIMPLIFICATION_DIST
    }
}

/// Physical raster-to-vector controls for ONNX stream predictions.
///
/// Defaults reproduce the original fixed post-processing for the model's
/// canonical 0.5 m output grid.
#[derive(Clone, Debug, PartialEq)]
pub struct OnnxStreamVectorizationParameters {
    /// Minimum probability of the winning foreground class. A value of zero
    /// leaves extraction as pure background/ditch/stream argmax.
    pub confidence_threshold: f32,
    /// Signed buffer applied to prediction polygons before centerlining.
    pub polygon_buffer_m: f64,
    /// Maximum spacing of boundary samples used to construct medial axes.
    pub centerline_sampling_distance_m: f64,
    /// Terminal medial-axis branches shorter than this are pruned.
    pub minimum_branch_length_m: f64,
    /// Components below this area bypass branch-length pruning.
    pub branch_length_exemption_area_m2: f64,
    /// Douglas-Peucker tolerance applied to extracted centerlines.
    pub simplification_tolerance_m: f64,
    /// Maximum endpoint gap used to join adjacent extracted line objects.
    pub endpoint_merge_distance_m: f64,
}

impl Default for OnnxStreamVectorizationParameters {
    fn default() -> Self {
        Self {
            confidence_threshold: 0.5,
            polygon_buffer_m: 1.0,
            centerline_sampling_distance_m: 0.5,
            minimum_branch_length_m: 3.0,
            branch_length_exemption_area_m2: 3.,
            simplification_tolerance_m: 0.1,
            endpoint_merge_distance_m: 10.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum StreamAlgorithm {
    #[default]
    Hydrological,
    #[cfg(feature = "deep-learning")]
    DitchesStreamsSvfSlope,
}

impl StreamAlgorithm {
    pub const fn uses_deferred_hydrology(self) -> bool {
        matches!(self, Self::Hydrological)
    }
}

impl Display for StreamAlgorithm {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Hydrological => "Hydrological flow accumulation",
            #[cfg(feature = "deep-learning")]
            Self::DitchesStreamsSvfSlope => "AI sky-view factor and slope",
        })
    }
}

/// User-facing and expert controls for the weighted marsh detector.
///
/// All spatial values are in physical units so changing raster resolution
/// does not change the represented feature size.
#[derive(Clone, Debug, PartialEq)]
pub struct MarshParameters {
    pub enabled: bool,
    /// General sensitivity. `0.5` leaves the expert seed/growth thresholds
    /// unchanged; larger values lower both thresholds.
    pub sensitivity: f32,
    pub minimum_polygon_area_m2: f64,
    pub planarity_radius_m: f64,
    pub maximum_planarity_rmse_m: f32,
    pub drainage_initiation_area_m2: f32,
    pub maximum_height_above_drainage_m: f32,
    pub maximum_downslope_distance_m: f32,
    pub preferred_depression_depth_m: f32,
    pub minimum_wetness_score: f32,
    pub seed_threshold: f32,
    pub growth_threshold: f32,
    pub closing_radius_m: f64,
    pub opening_radius_m: f64,
    pub maximum_hole_area_m2: f64,
    pub observation_radius_m: f64,
    pub supported_point_density_m2: f32,
    pub supported_ground_density_m2: f32,
    pub weights: MarshEvidenceWeights,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MarshEvidenceWeights {
    pub terrain: f32,
    pub hydrology: f32,
}

impl Default for MarshParameters {
    fn default() -> Self {
        Self {
            enabled: false,
            sensitivity: 0.5,
            minimum_polygon_area_m2: 45.,
            planarity_radius_m: 1.5,
            maximum_planarity_rmse_m: 0.12,
            drainage_initiation_area_m2: 2_500.,
            maximum_height_above_drainage_m: 1.5,
            maximum_downslope_distance_m: 35.,
            preferred_depression_depth_m: 0.35,
            minimum_wetness_score: 0.45,
            seed_threshold: 0.68,
            growth_threshold: 0.48,
            closing_radius_m: 1.5,
            opening_radius_m: 0.5,
            maximum_hole_area_m2: 12.,
            observation_radius_m: 2.,
            supported_point_density_m2: 4.,
            supported_ground_density_m2: 0.75,
            weights: MarshEvidenceWeights::default(),
        }
    }
}

impl Default for MarshEvidenceWeights {
    fn default() -> Self {
        Self {
            terrain: 0.25,
            hydrology: 0.75,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct CliffParameters {
    pub algorithm: CliffAlgorithm,
    pub cliff: f32,
    pub collapse: bool,
    pub collapse_amount_small_cliff: f32,
    pub collapse_amount_large_cliff: f32,
    pub collapse_linearity: f32,
}

impl Default for CliffParameters {
    fn default() -> Self {
        Self {
            algorithm: Default::default(),
            cliff: 0.7,
            collapse: true,
            collapse_amount_small_cliff: 1.,
            collapse_amount_large_cliff: 2.,
            collapse_linearity: 2.,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum CliffAlgorithm {
    #[default]
    SobelSlope,
    PolynomialFit,
}

impl Display for CliffAlgorithm {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::SobelSlope => f.write_str("Sobel slope"),
            Self::PolynomialFit => f.write_str("Polynomial fit"),
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct IntensityParameters {
    pub filters: Vec<IntensityFilter>,
}

impl Default for ContourParameters {
    fn default() -> Self {
        Self {
            algorithm: Default::default(),
            form_line_prune_algorithm: Default::default(),
            basemap_interval: 0.5,
            interval: 5.,
            dot_knoll_area: (10., 160.),
            algo_steps: 0,
            algo_lambda: 0.01,
            basemap_contour: false,
            form_lines: false,
            form_line_prune_threshold: 0.8,
            form_line_error_threshold: 0.15,
            contour_field: Default::default(),
            form_line_geometry: Default::default(),
        }
    }
}

impl Default for FormlineGeometryParameters {
    fn default() -> Self {
        Self {
            minimum_open_length_m: 0.,
            minimum_closed_length_m: 0.,
            reconnect_gap_m: 3.,
            closed_seed_length_m: 1.5,
            closed_all_or_none_max_length_m: 30.,
        }
    }
}

impl Default for ContourFieldParameters {
    fn default() -> Self {
        Self {
            max_iterations: 160,
            generalization: Default::default(),
            multiresolution_levels_m: vec![2., 1., crate::STANDARD_CELL_SIZE_METERS],
            iterations_per_level: vec![40, 50, 70],
            convergence_tolerance: 1e-4,
            fidelity_weight: 0.01,
            weighted_tv_weight: 3.,
            alignment_weight: 20.,
            hessian_weight: 0.03,
            minimum_contour_cost: 0.001,
            minimum_smoothness_weight: 0.001,
            smoothness_scale: 1.,
            salience_power: 4.,
            smoothness_power: 4.,
            slope_fit_radius_m: 3.,
            curvature_fit_radius_m: 5.,
            slope_weight: 0.5,
            profile_change_weight: 0.4,
            tangent_change_weight: 0.1,
            slope_reference: 0.2,
            profile_change_reference: 0.08,
            tangent_change_reference: 0.08,
            slope_epsilon: 0.02,
            rmse_reference: 0.25,
            persistence_threshold_fraction: 0.3,
            solver_guard_distance_m: 5.,
            collect_debug_rasters: false,
        }
    }
}

impl Default for VegetationParameters {
    fn default() -> Self {
        Self {
            green: (0.4, 0.6, 0.8),
            weights: Default::default(),
            yellow: 0.01,
        }
    }
}

impl Default for VegetationWeights {
    fn default() -> Self {
        Self {
            low: 0.5,
            medium: 0.35,
            high: 0.15,
        }
    }
}

impl GeometryParameters {
    pub fn bezier_error_for_symbol(&self, symbol: Symbol) -> Option<f64> {
        let bezier = match symbol {
            Symbol::Line(LineSymbol::Contour)
            | Symbol::Line(LineSymbol::FormLine)
            | Symbol::Line(LineSymbol::IndexContour) => &self.contours,
            Symbol::Area(AreaSymbol::RoughOpenLand) => &self.openness.bezier,
            Symbol::Area(AreaSymbol::LightGreen)
            | Symbol::Area(AreaSymbol::MediumGreen)
            | Symbol::Area(AreaSymbol::DarkGreen) => &self.vegetation.bezier,
            Symbol::Area(AreaSymbol::Building) => &self.buildings.bezier,
            Symbol::Area(AreaSymbol::GiganticBoulder)
            | Symbol::Line(LineSymbol::Cliff)
            | Symbol::Line(LineSymbol::ImpassableCliff) => &self.cliffs.bezier,
            Symbol::Area(AreaSymbol::UncrossableWaterWithBankLine) => &self.water.bezier,
            Symbol::Line(LineSymbol::SmallCrossableWatercourse) => &self.streams,
            Symbol::Area(AreaSymbol::Marsh) => &self.marsh.bezier,
            Symbol::Area(_) => &self.intensity.bezier,
            Symbol::Line(_) | Symbol::Point(_) => return None,
        };

        bezier.enabled.then_some(bezier.error)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct BezierParameters {
    pub error: f64,
    pub enabled: bool,
}

impl Default for BezierParameters {
    fn default() -> Self {
        Self {
            error: 2.0,
            enabled: true,
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct BufferedGeometryParameters {
    pub bezier: BezierParameters,
    pub buffer_rules: Vec<BufferRule>,
    pub min_size_filter: bool,
}

#[derive(Default, Clone)]
pub struct FileParameters {
    pub paths: Vec<PathBuf>,
    pub save_location: PathBuf,
    /// Preserve numeric raster samples instead of normalizing them to an
    /// 8-bit viewer image. Display-only rasters ignore this setting.
    pub write_raw_raster_values: bool,
    pub save_dem_raster: bool,
    pub save_slope_raster: bool,
    pub save_hillshade_raster: bool,
    pub save_intensity_raster: bool,
    pub save_last_return_raster: bool,
    pub save_canopy_height_raster: bool,
    pub save_surface_objects_raster: bool,
    pub save_ground_relief_2m_raster: bool,
    pub save_ground_relief_5m_raster: bool,
    pub save_hard_object_height_raster: bool,
    pub save_hard_object_confidence_raster: bool,
    pub save_vegetation_likelihood_raster: bool,
    pub save_filtered_surface_raster: bool,
    pub save_ndvd_raster: bool,
    pub save_point_density_raster: bool,
    pub save_flow_accumulation_raster: bool,
    pub save_building_height_raster: bool,
    pub save_building_planarity_raster: bool,
    pub save_building_residual_raster: bool,
    pub save_building_probability_raster: bool,
    pub save_building_plane_rejected_raster: bool,
    pub save_marsh_probability_raster: bool,
    pub save_marsh_support_raster: bool,
    pub save_marsh_wetness_raster: bool,
    pub save_marsh_reason_raster: bool,

    // lidar crs's
    pub crs_epsg: Vec<Option<CrsDef>>,
}

#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub enum ContourAlgo {
    NaiveIterations,
    NormalFieldSmoothing,
    WeightedScalarField,
    #[default]
    Raw,
}

impl Display for ContourAlgo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ContourAlgo::NaiveIterations => f.write_str("Naive"),
            ContourAlgo::NormalFieldSmoothing => f.write_str("Smooth"),
            ContourAlgo::WeightedScalarField => f.write_str("Weighted scalar field"),
            ContourAlgo::Raw => f.write_str("Raw"),
        }
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub enum FormlinePruneAlgo {
    #[default]
    None,
    TerrainChange,
    InterpolationError,
}

impl Display for FormlinePruneAlgo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FormlinePruneAlgo::TerrainChange => f.write_str("Terrain change"),
            FormlinePruneAlgo::InterpolationError => f.write_str("Interpolation error"),
            FormlinePruneAlgo::None => f.write_str("None"),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct IntensityFilter {
    pub low: f32,
    pub high: f32,
    pub symbol: AreaSymbol,
}

impl Default for IntensityFilter {
    fn default() -> Self {
        Self {
            low: 0.2,
            high: 0.4,
            symbol: AreaSymbol::BareRock,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BufferRule {
    pub direction: BufferDirection,
    pub amount: f64,
}

impl Default for BufferRule {
    fn default() -> Self {
        Self {
            direction: BufferDirection::Grow,
            amount: 2.,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BufferDirection {
    Grow,
    Shrink,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Scale {
    S10_000,
    #[default]
    S15_000,
}

impl Scale {
    pub fn denominator(self) -> f64 {
        match self {
            Self::S10_000 => 10_000.,
            Self::S15_000 => 15_000.,
        }
    }

    pub fn meters_to_paper_mm(self, meters: f64) -> f64 {
        meters * 1000. / self.denominator()
    }
}
