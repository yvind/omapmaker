use std::fmt::Display;

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
        let mut hash = 0xcbf29ce484222325_u64;
        let mut mix = |value: u64| {
            for byte in value.to_le_bytes() {
                hash ^= u64::from(byte);
                hash = hash.wrapping_mul(0x100000001b3);
            }
        };
        mix(self.max_iterations as u64);
        mix(match self.generalization {
            ContourGeneralization::Light => 0,
            ContourGeneralization::Balanced => 1,
            ContourGeneralization::Strong => 2,
        });
        mix(self.multiresolution_levels_m.len() as u64);
        for value in &self.multiresolution_levels_m {
            mix(value.to_bits());
        }
        mix(self.iterations_per_level.len() as u64);
        for &value in &self.iterations_per_level {
            mix(value as u64);
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
            mix(u64::from(value.to_bits()));
        }
        for value in [self.slope_fit_radius_m, self.curvature_fit_radius_m] {
            mix(value.to_bits());
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
            mix(u64::from(value.to_bits()));
        }
        mix(self.solver_guard_distance_m.to_bits());
        mix(u64::from(self.collect_debug_rasters));
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
        f.write_str(match self {
            Self::NaiveIterations => "Naive",
            Self::NormalFieldSmoothing => "Smooth",
            Self::WeightedScalarField => "Weighted scalar field",
            Self::Raw => "Raw",
        })
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
        f.write_str(match self {
            Self::TerrainChange => "Terrain change",
            Self::InterpolationError => "Interpolation error",
            Self::None => "None",
        })
    }
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
