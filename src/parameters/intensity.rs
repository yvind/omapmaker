use crate::map::AreaSymbol;

#[derive(Clone, Debug, PartialEq)]
pub struct IntensityParameters {
    pub enabled: bool,
    pub filters: Vec<IntensityFilter>,
}

impl Default for IntensityParameters {
    fn default() -> Self {
        Self {
            enabled: true,
            filters: Vec::new(),
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
