use std::fmt::Display;

use super::BufferRule;

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
