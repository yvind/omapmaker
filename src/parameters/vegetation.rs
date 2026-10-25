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
