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
