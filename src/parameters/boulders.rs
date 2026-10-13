/// Metre-based thresholds for opaque objects above the ground DEM.
#[derive(Clone, Debug, PartialEq)]
pub struct BoulderParameters {
    pub enabled: bool,
    pub minimum_height_m: f32,
    pub maximum_height_m: f32,
    pub minimum_diameter_m: f64,
    pub maximum_diameter_m: f64,
    pub minimum_area_m2: f64,
    pub minimum_top_area_m2: f64,
    pub minimum_top_fraction: f32,
    pub maximum_surface_roughness_m: f32,
    pub minimum_point_support: usize,
    pub maximum_nonterminal_fraction: f32,
    pub maximum_under_surface_fraction: f32,
    pub minimum_side_slope_degrees: f32,
    pub maximum_side_runoff_m: f64,
    pub minimum_grounded_side_fraction: f32,
    pub ground_level_tolerance_m: f32,
    pub maximum_elongation: f64,
    pub maximum_height_to_diameter: f32,
    pub confidence_threshold: f32,
    pub minimum_separation_m: f64,
}

impl Default for BoulderParameters {
    fn default() -> Self {
        Self {
            enabled: true,
            minimum_height_m: 2.,
            maximum_height_m: 15.,
            minimum_diameter_m: 0.5,
            maximum_diameter_m: 16.,
            minimum_area_m2: 1.,
            minimum_top_area_m2: 0.5,
            minimum_top_fraction: 0.25,
            maximum_surface_roughness_m: 0.3,
            minimum_point_support: 6,
            maximum_nonterminal_fraction: 0.25,
            maximum_under_surface_fraction: 0.1,
            minimum_side_slope_degrees: 35.,
            maximum_side_runoff_m: 3.,
            minimum_grounded_side_fraction: 0.5,
            ground_level_tolerance_m: 0.5,
            maximum_elongation: 2.5,
            maximum_height_to_diameter: 2.5,
            confidence_threshold: 0.7,
            minimum_separation_m: 2.,
        }
    }
}
