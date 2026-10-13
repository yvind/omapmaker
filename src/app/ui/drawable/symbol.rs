use crate::map::{AreaSymbol, LineSymbol, PointSymbol, Symbol};

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

impl DrawOrder for AreaSymbol {
    fn draw_order() -> impl Iterator<Item = Self> {
        Symbol::draw_order().filter_map(|symbol| match symbol {
            Symbol::Area(area) => Some(area),
            _ => None,
        })
    }
}
