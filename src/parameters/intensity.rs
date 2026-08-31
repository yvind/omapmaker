use crate::map::AreaSymbol;

#[derive(Clone, Debug, Default, PartialEq)]
pub struct IntensityParameters {
    pub filters: Vec<IntensityFilter>,
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
