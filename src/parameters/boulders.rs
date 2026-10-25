/// Shape and return evidence for opaque objects above the ground DEM.
/// Size is constrained by proportions, with no upper height or footprint cap.
#[derive(Clone, Debug, PartialEq)]
pub struct BoulderParameters {
    pub enabled: bool,
    pub minimum_height_m: f32,
    pub minimum_top_fraction: f32,
    pub maximum_surface_roughness_m: f32,
    pub maximum_nonterminal_fraction: f32,
    pub maximum_under_surface_fraction: f32,
    pub minimum_side_slope_degrees: f32,
    pub minimum_grounded_side_fraction: f32,
    pub maximum_height_to_diameter: f32,
    pub minimum_separation_m: f64,
}

impl Default for BoulderParameters {
    fn default() -> Self {
        Self {
            enabled: true,
            minimum_height_m: 2.,
            minimum_top_fraction: 0.8,
            maximum_surface_roughness_m: 0.3,
            maximum_nonterminal_fraction: 0.1,
            maximum_under_surface_fraction: 0.1,
            minimum_side_slope_degrees: 60.,
            minimum_grounded_side_fraction: 0.5,
            maximum_height_to_diameter: 1.,
            minimum_separation_m: 2.,
        }
    }
}
