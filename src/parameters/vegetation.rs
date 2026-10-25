#[derive(Clone, Debug)]
pub struct VegetationParameters {
    pub enabled: bool,
    pub openness_enabled: bool,
    pub green: (f32, f32, f32),
    pub weights: VegetationWeights,
    pub yellow: f32,
    pub undergrowth_enabled: bool,
    /// Threshold on the supported 0.2–0.8 m vegetation score.
    pub undergrowth_threshold: f32,
    pub boundaries: VegetationBoundaryParameters,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VegetationBoundaryParameters {
    pub enabled: bool,
    /// Gaussian standard deviation in ground metres; zero disables smoothing.
    pub smoothing_m: f64,
    pub minimum_height_change_m: f32,
    /// Minimum canopy-height gradient in metres of height per ground metre.
    pub minimum_gradient: f32,
    /// Applied after clipping and stitching tile fragments, in ground metres.
    pub minimum_length_m: f64,
}

impl Default for VegetationBoundaryParameters {
    fn default() -> Self {
        Self {
            enabled: true,
            smoothing_m: 0.75,
            minimum_height_change_m: 3.,
            minimum_gradient: 1.,
            minimum_length_m: 20.,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
/// Height bands exclude the separate 0.2–0.8 m undergrowth layer.
pub struct VegetationWeights {
    pub low: f32,
    pub medium: f32,
    pub high: f32,
}

impl Default for VegetationParameters {
    fn default() -> Self {
        Self {
            enabled: true,
            openness_enabled: true,
            green: (0.4, 0.6, 0.8),
            weights: Default::default(),
            yellow: 0.01,
            undergrowth_enabled: true,
            undergrowth_threshold: 0.5,
            boundaries: Default::default(),
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
