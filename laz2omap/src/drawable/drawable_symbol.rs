use eframe::egui::{Color32, Stroke};
use omap::symbols::{AreaSymbol, LineSymbol, PointSymbol, Symbol, TextSymbol};

const PURPLE: Color32 = Color32::from_rgba_premultiplied(190, 60, 255, 255);
const ROUGH_YELLOW: Color32 = Color32::from_rgba_premultiplied(255, 220, 155, 255);
const BROWN: Color32 = Color32::from_rgba_premultiplied(180, 50, 0, 255);
const MEDIUM_BROWN: Color32 = Color32::from_rgba_premultiplied(200, 80, 0, 255);
const LIGHT_BROWN: Color32 = Color32::from_rgba_premultiplied(220, 110, 0, 255);
const OLIVE: Color32 = Color32::from_rgba_premultiplied(134, 141, 7, 255);

const SCALE_FACTOR: f32 = 0.25;

pub(crate) trait DrawableSymbol {
    /// what fill to use for drawing symbol equals the stroke of a line or
    /// color of a polygon or color and radius of point
    fn stroke(&self, pixels_per_meter: f32) -> Option<(bool, Stroke)>;
}

pub trait DrawOrder {
    fn draw_order() -> std::vec::IntoIter<Self>
    where
        Self: std::marker::Sized;
}

impl DrawOrder for Symbol {
    fn draw_order() -> std::vec::IntoIter<Self> {
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
            Symbol::Area(AreaSymbol::Building),
            Symbol::Point(PointSymbol::SmallBoulder),
            Symbol::Point(PointSymbol::LargeBoulder),
            Symbol::Area(AreaSymbol::OutOfBounds),
            Symbol::Text(TextSymbol::ContourValue),
            Symbol::Text(TextSymbol::SpotHeight),
            Symbol::Text(TextSymbol::ControlNumber),
        ]
        .into_iter()
    }
}

impl DrawableSymbol for Symbol {
    fn stroke(&self, pixels_per_meter: f32) -> Option<(bool, Stroke)> {
        match self {
            Symbol::Area(a) => a.stroke(pixels_per_meter),
            Symbol::Line(l) => l.stroke(pixels_per_meter),
            Symbol::Point(p) => p.stroke(pixels_per_meter),
            Symbol::Text(t) => t.stroke(pixels_per_meter),
        }
    }
}

impl DrawOrder for AreaSymbol {
    fn draw_order() -> std::vec::IntoIter<Self>
    where
        Self: std::marker::Sized,
    {
        vec![
            AreaSymbol::RoughOpenLand,
            AreaSymbol::OpenLand,
            AreaSymbol::SandyGround,
            AreaSymbol::BareRock,
            AreaSymbol::LightGreen,
            AreaSymbol::MediumGreen,
            AreaSymbol::DarkGreen,
            AreaSymbol::Marsh,
            AreaSymbol::PrivateArea,
            AreaSymbol::PavedAreaWithBoundary,
            AreaSymbol::ShallowWaterWithSolidBankLine,
            AreaSymbol::UncrossableWaterWithBankLine,
            AreaSymbol::GiganticBoulder,
            AreaSymbol::Building,
            AreaSymbol::OutOfBounds,
        ]
        .into_iter()
    }
}

impl DrawableSymbol for AreaSymbol {
    fn stroke(&self, pixels_per_meter: f32) -> Option<(bool, Stroke)> {
        let scale_factor = SCALE_FACTOR * pixels_per_meter;

        match self {
            AreaSymbol::SandyGround => Some((false, Stroke::new(0., Color32::GOLD))),
            AreaSymbol::BareRock => Some((false, Stroke::new(0., Color32::GRAY))),
            AreaSymbol::UncrossableWaterWithBankLine => {
                Some((false, Stroke::new(0., Color32::LIGHT_BLUE)))
            }
            AreaSymbol::ShallowWaterWithSolidBankLine => Some((
                false,
                Stroke::new(2. * scale_factor, Color32::BLUE.gamma_multiply(0.4)),
            )),
            AreaSymbol::Marsh => Some((
                true,
                Stroke::new(2. * scale_factor, Color32::LIGHT_BLUE.gamma_multiply(0.4)),
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
            AreaSymbol::OutOfBounds => {
                Some((false, Stroke::new(0., Color32::PURPLE.gamma_multiply(0.5))))
            }
            _ => None,
        }
    }
}

impl DrawOrder for LineSymbol {
    fn draw_order() -> std::vec::IntoIter<Self>
    where
        Self: std::marker::Sized,
    {
        vec![
            LineSymbol::BasemapContour,
            LineSymbol::FormLine,
            LineSymbol::Contour,
            LineSymbol::IndexContour,
            LineSymbol::NegBasemapContour,
            LineSymbol::SmallCrossableWatercourse,
        ]
        .into_iter()
    }
}

impl DrawableSymbol for LineSymbol {
    fn stroke(&self, pixels_per_meter: f32) -> Option<(bool, Stroke)> {
        let scale_factor = SCALE_FACTOR * pixels_per_meter;

        match self {
            LineSymbol::Contour => Some((false, Stroke::new(3. * scale_factor, BROWN))),
            LineSymbol::BasemapContour => {
                Some((false, Stroke::new(1. * scale_factor, LIGHT_BROWN)))
            }
            LineSymbol::NegBasemapContour => Some((false, Stroke::new(1. * scale_factor, PURPLE))),
            LineSymbol::IndexContour => Some((false, Stroke::new(5. * scale_factor, BROWN))),
            LineSymbol::FormLine => Some((true, Stroke::new(2. * scale_factor, MEDIUM_BROWN))),
            LineSymbol::SmallCrossableWatercourse => {
                Some((false, Stroke::new(4. * scale_factor, Color32::BLUE)))
            }
            _ => None,
        }
    }
}

impl DrawOrder for PointSymbol {
    fn draw_order() -> std::vec::IntoIter<Self>
    where
        Self: std::marker::Sized,
    {
        vec![
            PointSymbol::SlopeLineFormLine,
            PointSymbol::SlopeLineContour,
            PointSymbol::DotKnoll,
            PointSymbol::ElongatedDotKnoll,
            PointSymbol::UDepression,
            PointSymbol::SmallBoulder,
            PointSymbol::LargeBoulder,
        ]
        .into_iter()
    }
}

impl DrawableSymbol for PointSymbol {
    fn stroke(&self, pixels_per_meter: f32) -> Option<(bool, Stroke)> {
        let scale_factor = SCALE_FACTOR * pixels_per_meter;

        match self {
            PointSymbol::SlopeLineContour => Some((false, Stroke::new(3. * scale_factor, BROWN))),
            PointSymbol::SlopeLineFormLine => Some((false, Stroke::new(2. * scale_factor, BROWN))),
            PointSymbol::DotKnoll => Some((false, Stroke::new(8. * scale_factor, BROWN))),
            PointSymbol::ElongatedDotKnoll => Some((true, Stroke::new(8. * scale_factor, BROWN))),
            PointSymbol::UDepression => Some((false, Stroke::new(8. * scale_factor, PURPLE))),
            PointSymbol::SmallBoulder => {
                Some((false, Stroke::new(8. * scale_factor, Color32::BLACK)))
            }
            PointSymbol::LargeBoulder => {
                Some((false, Stroke::new(12. * scale_factor, Color32::BLACK)))
            }
            _ => None,
        }
    }
}

impl DrawOrder for TextSymbol {
    fn draw_order() -> std::vec::IntoIter<Self>
    where
        Self: std::marker::Sized,
    {
        vec![
            TextSymbol::ContourValue,
            TextSymbol::SpotHeight,
            TextSymbol::ControlNumber,
        ]
        .into_iter()
    }
}

impl DrawableSymbol for TextSymbol {
    fn stroke(&self, pixels_per_meter: f32) -> Option<(bool, Stroke)> {
        let scale_factor = SCALE_FACTOR * pixels_per_meter;

        match self {
            TextSymbol::ContourValue => Some((false, Stroke::new(2. * scale_factor, BROWN))),
            TextSymbol::SpotHeight => Some((false, Stroke::new(2. * scale_factor, Color32::BLACK))),
            TextSymbol::ControlNumber => {
                Some((false, Stroke::new(3. * scale_factor, Color32::PURPLE)))
            }
        }
    }
}
