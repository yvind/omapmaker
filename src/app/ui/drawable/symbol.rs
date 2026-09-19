use crate::map::{AreaSymbol, LineSymbol, PointSymbol, Symbol};
use crate::parameters::Scale;
use eframe::egui::{Color32, Stroke};

const PURPLE: Color32 = Color32::from_rgba_premultiplied(190, 60, 255, 255);
const ROUGH_YELLOW: Color32 = Color32::from_rgba_premultiplied(255, 220, 155, 255);
const BROWN: Color32 = Color32::from_rgba_premultiplied(180, 50, 0, 255);
const MEDIUM_BROWN: Color32 = Color32::from_rgba_premultiplied(200, 80, 0, 255);
const LIGHT_BROWN: Color32 = Color32::from_rgba_premultiplied(220, 110, 0, 255);
const OLIVE: Color32 = Color32::from_rgba_premultiplied(134, 141, 7, 255);

// Paper measurements mirror the default ISOM symbols in the `omap` crate.
fn paper_mm_to_pixels(paper_mm: f64, pixels_per_meter: f32, scale: Scale) -> f32 {
    scale.paper_mm_to_meters(paper_mm) as f32 * pixels_per_meter
}

pub(crate) trait DrawableSymbol {
    /// what fill to use for drawing symbol equals the stroke of a line or
    /// color of a polygon or color and radius of point
    fn stroke(&self, pixels_per_meter: f32, scale: Scale) -> Option<(bool, Stroke)>;
}

pub trait DrawOrder {
    fn draw_order() -> impl Iterator<Item = Self>
    where
        Self: std::marker::Sized;
}

impl DrawOrder for Symbol {
    fn draw_order() -> impl Iterator<Item = Self> {
        vec![
            Symbol::Area(AreaSymbol::RoughOpenLand),
            Symbol::Area(AreaSymbol::OpenLand),
            Symbol::Area(AreaSymbol::SandyGround),
            Symbol::Area(AreaSymbol::BareRock),
            Symbol::Area(AreaSymbol::LightGreen),
            Symbol::Area(AreaSymbol::MediumGreen),
            Symbol::Area(AreaSymbol::DarkGreen),
            Symbol::Area(AreaSymbol::Marsh),
            Symbol::Area(AreaSymbol::PrivateArea),
            Symbol::Area(AreaSymbol::PavedAreaWithBoundary),
            Symbol::Line(LineSymbol::BasemapContour),
            Symbol::Line(LineSymbol::FormLine),
            Symbol::Line(LineSymbol::Contour),
            Symbol::Line(LineSymbol::IndexContour),
            Symbol::Point(PointSymbol::SlopeLineFormLine),
            Symbol::Point(PointSymbol::SlopeLineContour),
            Symbol::Line(LineSymbol::NegBasemapContour),
            Symbol::Line(LineSymbol::SmallCrossableWatercourse),
            Symbol::Point(PointSymbol::DotKnoll),
            Symbol::Point(PointSymbol::ElongatedDotKnoll),
            Symbol::Point(PointSymbol::UDepression),
            Symbol::Area(AreaSymbol::ShallowWaterWithSolidBankLine),
            Symbol::Area(AreaSymbol::UncrossableWaterWithBankLine),
            Symbol::Area(AreaSymbol::GiganticBoulder),
            Symbol::Line(LineSymbol::Cliff),
            Symbol::Line(LineSymbol::ImpassableCliff),
            Symbol::Area(AreaSymbol::Building),
            Symbol::Point(PointSymbol::SmallBoulder),
            Symbol::Point(PointSymbol::LargeBoulder),
        ]
        .into_iter()
    }
}

impl DrawableSymbol for Symbol {
    fn stroke(&self, pixels_per_meter: f32, scale: Scale) -> Option<(bool, Stroke)> {
        match self {
            Symbol::Area(a) => a.stroke(pixels_per_meter, scale),
            Symbol::Line(l) => l.stroke(pixels_per_meter, scale),
            Symbol::Point(p) => p.stroke(pixels_per_meter, scale),
        }
    }
}

impl DrawOrder for AreaSymbol {
    fn draw_order() -> impl Iterator<Item = AreaSymbol>
    where
        Self: std::marker::Sized,
    {
        Symbol::draw_order().filter_map(|f| {
            if let Symbol::Area(af) = f {
                Some(af)
            } else {
                None
            }
        })
    }
}

impl DrawableSymbol for AreaSymbol {
    fn stroke(&self, pixels_per_meter: f32, scale: Scale) -> Option<(bool, Stroke)> {
        match self {
            AreaSymbol::SandyGround => Some((false, Stroke::new(0., Color32::GOLD))),
            AreaSymbol::BareRock => Some((false, Stroke::new(0., Color32::GRAY))),
            AreaSymbol::UncrossableWaterWithBankLine => {
                Some((false, Stroke::new(0., Color32::LIGHT_BLUE)))
            }
            AreaSymbol::ShallowWaterWithSolidBankLine => Some((
                false,
                Stroke::new(
                    paper_mm_to_pixels(0.10, pixels_per_meter, scale),
                    Color32::BLUE.gamma_multiply(0.4),
                ),
            )),
            AreaSymbol::Marsh => Some((
                true,
                Stroke::new(
                    paper_mm_to_pixels(0.10, pixels_per_meter, scale),
                    Color32::LIGHT_BLUE.gamma_multiply(0.4),
                ),
            )),
            AreaSymbol::GiganticBoulder => Some((false, Stroke::new(0., Color32::BLACK))),
            AreaSymbol::OpenLand => Some((false, Stroke::new(0., Color32::YELLOW))),
            AreaSymbol::RoughOpenLand => Some((false, Stroke::new(0., ROUGH_YELLOW))),
            AreaSymbol::LightGreen => Some((false, Stroke::new(0., Color32::LIGHT_GREEN))),
            AreaSymbol::MediumGreen => Some((false, Stroke::new(0., Color32::GREEN))),
            AreaSymbol::DarkGreen => Some((false, Stroke::new(0., Color32::DARK_GREEN))),
            AreaSymbol::Building => Some((false, Stroke::new(0., Color32::BLACK))),
            AreaSymbol::PavedAreaWithBoundary => Some((false, Stroke::new(0., LIGHT_BROWN))),
            AreaSymbol::PrivateArea => Some((false, Stroke::new(0., OLIVE))),
            AreaSymbol::WhiteForest => None,
        }
    }
}

impl DrawOrder for LineSymbol {
    fn draw_order() -> impl Iterator<Item = Self> {
        Symbol::draw_order().filter_map(|f| {
            if let Symbol::Line(lf) = f {
                Some(lf)
            } else {
                None
            }
        })
    }
}

impl DrawableSymbol for LineSymbol {
    fn stroke(&self, pixels_per_meter: f32, scale: Scale) -> Option<(bool, Stroke)> {
        let (paper_width_mm, color) = match self {
            LineSymbol::Contour => (0.14, BROWN),
            LineSymbol::BasemapContour => (0.04, LIGHT_BROWN),
            LineSymbol::NegBasemapContour => (0.04, PURPLE),
            LineSymbol::IndexContour => (0.25, BROWN),
            LineSymbol::FormLine => (0.10, MEDIUM_BROWN),
            LineSymbol::SmallCrossableWatercourse => (0.18, Color32::BLUE),
            LineSymbol::Cliff => (0.25, Color32::BLACK),
            LineSymbol::ImpassableCliff => (0.35, Color32::BLACK),
        };

        Some((
            false,
            Stroke::new(
                paper_mm_to_pixels(paper_width_mm, pixels_per_meter, scale),
                color,
            ),
        ))
    }
}

impl DrawOrder for PointSymbol {
    fn draw_order() -> impl Iterator<Item = Self> {
        Symbol::draw_order().filter_map(|f| {
            if let Symbol::Point(pf) = f {
                Some(pf)
            } else {
                None
            }
        })
    }
}

impl DrawableSymbol for PointSymbol {
    fn stroke(&self, pixels_per_meter: f32, scale: Scale) -> Option<(bool, Stroke)> {
        let (special, paper_size_mm, color) = match self {
            PointSymbol::SlopeLineContour => (false, 0.14, BROWN),
            PointSymbol::SlopeLineFormLine => (false, 0.10, BROWN),
            PointSymbol::DotKnoll => (false, 0.25, BROWN),
            PointSymbol::ElongatedDotKnoll => (true, 0.20, BROWN),
            // The preview approximates the complex U shape by its 0.8 mm overall width.
            PointSymbol::UDepression => (false, 0.40, PURPLE),
            PointSymbol::SmallBoulder => (false, 0.20, Color32::BLACK),
            PointSymbol::LargeBoulder => (false, 0.30, Color32::BLACK),
        };

        Some((
            special,
            Stroke::new(
                paper_mm_to_pixels(paper_size_mm, pixels_per_meter, scale),
                color,
            ),
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo::BoundingRect;

    fn assert_approximately_equal(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() < 1e-5,
            "expected {expected}, got {actual}"
        );
    }

    fn preview_paper_size(symbol: impl DrawableSymbol) -> f64 {
        let ground_meters = symbol
            .stroke(1., Scale::S15_000)
            .expect("preview symbol should be drawable")
            .1
            .width;
        Scale::S15_000.meters_to_paper_mm(f64::from(ground_meters))
    }

    #[test]
    fn line_width_uses_selected_map_scale() {
        let pixels_per_meter = 2.;
        let width_10_000 = LineSymbol::Contour
            .stroke(pixels_per_meter, Scale::S10_000)
            .unwrap()
            .1
            .width;
        let width_15_000 = LineSymbol::Contour
            .stroke(pixels_per_meter, Scale::S15_000)
            .unwrap()
            .1
            .width;

        assert_eq!(width_10_000, 2.8);
        assert_eq!(width_15_000, 4.2);
    }

    #[test]
    fn point_size_uses_selected_map_scale() {
        let pixels_per_meter = 2.;
        let radius_10_000 = PointSymbol::DotKnoll
            .stroke(pixels_per_meter, Scale::S10_000)
            .unwrap()
            .1
            .width;
        let radius_15_000 = PointSymbol::DotKnoll
            .stroke(pixels_per_meter, Scale::S15_000)
            .unwrap()
            .1
            .width;

        assert_eq!(radius_10_000, 5.);
        assert_eq!(radius_15_000, 7.5);
    }

    #[test]
    fn line_widths_match_default_omap_symbols() {
        let default_map = omap::Omap::default_15_000().unwrap();

        for symbol in [
            LineSymbol::Contour,
            LineSymbol::BasemapContour,
            LineSymbol::NegBasemapContour,
            LineSymbol::IndexContour,
            LineSymbol::FormLine,
            LineSymbol::SmallCrossableWatercourse,
            LineSymbol::Cliff,
            LineSymbol::ImpassableCliff,
        ] {
            let default_symbol = default_map
                .symbols
                .find_by_code(symbol.get_code())
                .expect("symbol should exist in the default map");
            let omap::symbols::Symbol::Line(default_line) = default_symbol.symbol() else {
                panic!("{} should be a line symbol", symbol.get_code());
            };

            assert_approximately_equal(preview_paper_size(symbol), default_line.line_width.get());
        }
    }

    #[test]
    fn circular_point_radii_match_default_omap_symbols() {
        let default_map = omap::Omap::default_15_000().unwrap();

        for symbol in [
            PointSymbol::DotKnoll,
            PointSymbol::SmallBoulder,
            PointSymbol::LargeBoulder,
        ] {
            let default_symbol = default_map
                .symbols
                .find_by_code(symbol.get_code())
                .expect("symbol should exist in the default map");
            let omap::symbols::Symbol::Point(default_point) = default_symbol.symbol() else {
                panic!("{} should be a point symbol", symbol.get_code());
            };

            assert_approximately_equal(
                preview_paper_size(symbol),
                default_point.inner_radius.get(),
            );
        }
    }

    #[test]
    fn slope_line_widths_match_default_omap_symbols() {
        let default_map = omap::Omap::default_15_000().unwrap();

        for symbol in [
            PointSymbol::SlopeLineContour,
            PointSymbol::SlopeLineFormLine,
        ] {
            let default_symbol = default_map
                .symbols
                .find_by_code(symbol.get_code())
                .expect("symbol should exist in the default map");
            let omap::symbols::Symbol::Point(default_point) = default_symbol.symbol() else {
                panic!("{} should be a point symbol", symbol.get_code());
            };
            let Some(omap::symbols::Element::Line {
                symbol: default_line,
                ..
            }) = default_point.elements.first()
            else {
                panic!("{} should contain a line element", symbol.get_code());
            };

            assert_approximately_equal(preview_paper_size(symbol), default_line.line_width.get());
        }
    }

    #[test]
    fn non_circular_point_widths_match_default_omap_footprints() {
        let default_map = omap::Omap::default_15_000().unwrap();
        let flattening_error = omap::NonNegativeF64::clamped_from(0.001);

        let elongated = default_map
            .symbols
            .find_by_code(PointSymbol::ElongatedDotKnoll.get_code())
            .expect("elongated knoll should exist in the default map");
        let omap::symbols::Symbol::Point(elongated) = elongated.symbol() else {
            panic!("elongated knoll should be a point symbol");
        };
        let Some(omap::symbols::Element::Area { object, .. }) = elongated.elements.first() else {
            panic!("elongated knoll should contain an area element");
        };
        let bounds = object
            .flatten(flattening_error)
            .unwrap()
            .into_polygon()
            .bounding_rect()
            .unwrap();
        let preview_minor_radius = preview_paper_size(PointSymbol::ElongatedDotKnoll);
        assert_approximately_equal(2. * preview_minor_radius, bounds.width());
        assert_approximately_equal(4. * preview_minor_radius, bounds.height());

        let depression = default_map
            .symbols
            .find_by_code(PointSymbol::UDepression.get_code())
            .expect("small depression should exist in the default map");
        let omap::symbols::Symbol::Point(depression) = depression.symbol() else {
            panic!("small depression should be a point symbol");
        };
        let Some(omap::symbols::Element::Line { symbol, object }) = depression.elements.first()
        else {
            panic!("small depression should contain a line element");
        };
        let bounds = object
            .flatten(flattening_error)
            .unwrap()
            .geometry()
            .bounding_rect()
            .unwrap();
        let default_overall_width = bounds.width() + symbol.line_width.get();
        assert_approximately_equal(
            2. * preview_paper_size(PointSymbol::UDepression),
            default_overall_width,
        );
    }
}
