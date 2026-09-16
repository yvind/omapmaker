use crate::map::{AreaSymbol, LineSymbol, Symbol};

#[derive(Clone, Debug)]
pub struct GeometryParameters {
    pub contours: BezierParameters,
    pub openness: BufferedGeometryParameters,
    pub vegetation: BufferedGeometryParameters,
    pub buildings: BufferedGeometryParameters,
    pub cliffs: CliffGeometryParameters,
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
                    line_cap: BufferLineCap::Square,
                    line_join: BufferLineJoin::Miter,
                },
                BufferRule {
                    direction: BufferDirection::Grow,
                    amount: 2.,
                    line_cap: BufferLineCap::Square,
                    line_join: BufferLineJoin::Miter,
                },
            ],
            min_size_filter: true,
        };
        let cliffs = CliffGeometryParameters::default();
        let openness = BufferedGeometryParameters {
            bezier: BezierParameters::default(),
            buffer_rules: vec![
                BufferRule {
                    direction: BufferDirection::Shrink,
                    amount: 2.5,
                    ..Default::default()
                },
                BufferRule {
                    direction: BufferDirection::Grow,
                    amount: 5.,
                    ..Default::default()
                },
                BufferRule {
                    direction: BufferDirection::Shrink,
                    amount: 2.5,
                    ..Default::default()
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
                    ..Default::default()
                },
                BufferRule {
                    direction: BufferDirection::Shrink,
                    amount: 2.5,
                    ..Default::default()
                },
                BufferRule {
                    direction: BufferDirection::Grow,
                    amount: 5.,
                    ..Default::default()
                },
                BufferRule {
                    direction: BufferDirection::Shrink,
                    amount: 2.5,
                    ..Default::default()
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

#[derive(Clone, Debug, PartialEq)]
pub struct RdpParameters {
    pub tolerance_m: f64,
    pub enabled: bool,
}

impl Default for RdpParameters {
    fn default() -> Self {
        Self {
            tolerance_m: 1.0,
            enabled: true,
        }
    }
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

#[derive(Clone, Debug, PartialEq)]
pub struct CliffGeometryParameters {
    pub bezier: BezierParameters,
    pub rdp: RdpParameters,
    pub buffer_rules: Vec<BufferRule>,
    pub maximum_hole_area_m2: f64,
    pub min_size_filter: bool,
}

impl Default for CliffGeometryParameters {
    fn default() -> Self {
        Self {
            bezier: BezierParameters {
                enabled: false,
                ..Default::default()
            },
            rdp: RdpParameters::default(),
            buffer_rules: Vec::new(),
            maximum_hole_area_m2: 4.,
            min_size_filter: true,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BufferRule {
    pub direction: BufferDirection,
    pub amount: f64,
    pub line_cap: BufferLineCap,
    pub line_join: BufferLineJoin,
}

impl Default for BufferRule {
    fn default() -> Self {
        Self {
            direction: BufferDirection::Grow,
            amount: 2.,
            line_cap: BufferLineCap::Round,
            line_join: BufferLineJoin::Round,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BufferDirection {
    Grow,
    Shrink,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum BufferLineCap {
    #[default]
    Round,
    Square,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum BufferLineJoin {
    Miter,
    #[default]
    Round,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn building_buffers_use_square_caps_and_miter_joins_by_default() {
        let parameters = GeometryParameters::default();

        assert!(!parameters.buildings.buffer_rules.is_empty());
        assert!(
            parameters
                .buildings
                .buffer_rules
                .iter()
                .all(|rule| rule.line_cap == BufferLineCap::Square)
        );
        assert!(
            parameters
                .buildings
                .buffer_rules
                .iter()
                .all(|rule| rule.line_join == BufferLineJoin::Miter)
        );
    }

    #[test]
    fn new_buffer_rules_preserve_the_previous_round_style() {
        let rule = BufferRule::default();

        assert_eq!(rule.line_cap, BufferLineCap::Round);
        assert_eq!(rule.line_join, BufferLineJoin::Round);
    }

    #[test]
    fn non_building_features_use_round_buffer_styles() {
        let parameters = GeometryParameters::default();
        let non_building_rules = [
            &parameters.openness.buffer_rules,
            &parameters.vegetation.buffer_rules,
            &parameters.cliffs.buffer_rules,
            &parameters.intensity.buffer_rules,
            &parameters.water.buffer_rules,
            &parameters.marsh.buffer_rules,
        ];

        assert!(non_building_rules.into_iter().flatten().all(|rule| {
            rule.line_cap == BufferLineCap::Round && rule.line_join == BufferLineJoin::Round
        }));
    }
}
