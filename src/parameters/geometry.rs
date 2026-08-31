use crate::map::{AreaSymbol, LineSymbol, Symbol};

#[derive(Clone, Debug)]
pub struct GeometryParameters {
    pub contours: BezierParameters,
    pub openness: BufferedGeometryParameters,
    pub vegetation: BufferedGeometryParameters,
    pub buildings: BufferedGeometryParameters,
    pub cliffs: BufferedGeometryParameters,
    pub intensity: BufferedGeometryParameters,
    pub water: BufferedGeometryParameters,
    pub marsh: BufferedGeometryParameters,
    pub streams: BezierParameters,
}

impl GeometryParameters {
    pub fn bezier_error_for_symbol(&self, symbol: Symbol) -> Option<f64> {
        let bezier = match symbol {
            Symbol::Line(LineSymbol::Contour)
            | Symbol::Line(LineSymbol::FormLine)
            | Symbol::Line(LineSymbol::IndexContour) => &self.contours,
            Symbol::Area(AreaSymbol::RoughOpenLand) => &self.openness.bezier,
            Symbol::Area(AreaSymbol::LightGreen)
            | Symbol::Area(AreaSymbol::MediumGreen)
            | Symbol::Area(AreaSymbol::DarkGreen) => &self.vegetation.bezier,
            Symbol::Area(AreaSymbol::Building) => &self.buildings.bezier,
            Symbol::Area(AreaSymbol::GiganticBoulder)
            | Symbol::Line(LineSymbol::Cliff)
            | Symbol::Line(LineSymbol::ImpassableCliff) => &self.cliffs.bezier,
            Symbol::Area(AreaSymbol::UncrossableWaterWithBankLine) => &self.water.bezier,
            Symbol::Line(LineSymbol::SmallCrossableWatercourse) => &self.streams,
            Symbol::Area(AreaSymbol::Marsh) => &self.marsh.bezier,
            Symbol::Area(_) => &self.intensity.bezier,
            Symbol::Line(_) | Symbol::Point(_) => return None,
        };
        bezier.enabled.then_some(bezier.error)
    }
}

impl Default for GeometryParameters {
    fn default() -> Self {
        let buildings = BufferedGeometryParameters {
            bezier: BezierParameters {
                error: 0.25,
                enabled: false,
            },
            buffer_rules: vec![
                BufferRule {
                    direction: BufferDirection::Shrink,
                    amount: 2.,
                },
                BufferRule {
                    direction: BufferDirection::Grow,
                    amount: 2.,
                },
            ],
            min_size_filter: true,
        };
        let mut cliffs = BufferedGeometryParameters::default();
        cliffs.bezier.enabled = false;
        let openness = BufferedGeometryParameters {
            bezier: BezierParameters::default(),
            buffer_rules: vec![
                BufferRule {
                    direction: BufferDirection::Shrink,
                    amount: 2.5,
                },
                BufferRule {
                    direction: BufferDirection::Grow,
                    amount: 5.,
                },
                BufferRule {
                    direction: BufferDirection::Shrink,
                    amount: 2.5,
                },
            ],
            min_size_filter: true,
        };
        let vegetation = BufferedGeometryParameters {
            bezier: BezierParameters::default(),
            buffer_rules: vec![
                BufferRule {
                    direction: BufferDirection::Grow,
                    amount: 1.,
                },
                BufferRule {
                    direction: BufferDirection::Shrink,
                    amount: 2.5,
                },
                BufferRule {
                    direction: BufferDirection::Grow,
                    amount: 5.,
                },
                BufferRule {
                    direction: BufferDirection::Shrink,
                    amount: 2.5,
                },
            ],
            min_size_filter: true,
        };
        Self {
            contours: Default::default(),
            openness,
            vegetation,
            buildings,
            cliffs,
            intensity: Default::default(),
            water: Default::default(),
            marsh: Default::default(),
            streams: Default::default(),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct BezierParameters {
    pub error: f64,
    pub enabled: bool,
}

impl Default for BezierParameters {
    fn default() -> Self {
        Self {
            error: 2.0,
            enabled: true,
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct BufferedGeometryParameters {
    pub bezier: BezierParameters,
    pub buffer_rules: Vec<BufferRule>,
    pub min_size_filter: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BufferRule {
    pub direction: BufferDirection,
    pub amount: f64,
}

impl Default for BufferRule {
    fn default() -> Self {
        Self {
            direction: BufferDirection::Grow,
            amount: 2.,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BufferDirection {
    Grow,
    Shrink,
}
