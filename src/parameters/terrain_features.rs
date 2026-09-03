use std::fmt::Display;

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

#[derive(Clone, Debug, PartialEq)]
pub struct CliffParameters {
    pub algorithm: CliffAlgorithm,
    pub cliff: f32,
    pub collapse: bool,
    pub minimum_cliff_height_m: f32,
    pub impassable_cliff_height_m: f32,
    pub collapse_linearity: f32,
}

impl Default for CliffParameters {
    fn default() -> Self {
        Self {
            algorithm: Default::default(),
            cliff: 0.7,
            collapse: true,
            minimum_cliff_height_m: 1.,
            impassable_cliff_height_m: 2.,
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
        f.write_str(match self {
            Self::SobelSlope => "Sobel slope",
            Self::PolynomialFit => "Polynomial fit",
        })
    }
}
