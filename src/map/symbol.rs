use crate::parameters::Scale;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Symbol {
    Area(AreaSymbol),
    Line(LineSymbol),
    Point(PointSymbol),
}

impl std::fmt::Display for Symbol {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Symbol::Area(area_symbol) => write!(f, "{:?}", area_symbol),
            Symbol::Line(line_symbol) => write!(f, "{:?}", line_symbol),
            Symbol::Point(point_symbol) => write!(f, "{:?}", point_symbol),
        }
    }
}

impl Symbol {
    pub fn get_omap_symbol_ref<'a>(
        &self,
        symbol_set: &'a omap::symbols::SymbolSet,
    ) -> Option<omap::symbols::SymbolRef<'a>> {
        let code = match self {
            Symbol::Area(area_symbol) => area_symbol.get_code(),
            Symbol::Line(line_symbol) => line_symbol.get_code(),
            Symbol::Point(point_symbol) => point_symbol.get_code(),
        };
        symbol_set.find_by_code(code)
    }
}

impl From<AreaSymbol> for Symbol {
    fn from(value: AreaSymbol) -> Self {
        Symbol::Area(value)
    }
}

impl From<LineSymbol> for Symbol {
    fn from(value: LineSymbol) -> Self {
        Symbol::Line(value)
    }
}

impl From<PointSymbol> for Symbol {
    fn from(value: PointSymbol) -> Self {
        Symbol::Point(value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum AreaSymbol {
    WhiteForest,
    RoughOpenLand,
    OpenLand,
    SandyGround,
    BareRock,
    LightGreen,
    MediumGreen,
    DarkGreen,
    Marsh,
    PrivateArea,
    PavedAreaWithBoundary,
    ShallowWaterWithSolidBankLine,
    UncrossableWaterWithBankLine,
    GiganticBoulder,
    Building,
}

impl AreaSymbol {
    pub fn get_code(&self) -> omap::Code {
        match self {
            AreaSymbol::RoughOpenLand => omap::Code::new(403, 0, 0),
            AreaSymbol::OpenLand => omap::Code::new(401, 0, 0),
            AreaSymbol::SandyGround => omap::Code::new(213, 0, 0),
            AreaSymbol::BareRock => omap::Code::new(214, 0, 0),
            AreaSymbol::LightGreen => omap::Code::new(406, 0, 0),
            AreaSymbol::MediumGreen => omap::Code::new(408, 0, 0),
            AreaSymbol::DarkGreen => omap::Code::new(410, 0, 0),
            AreaSymbol::Marsh => omap::Code::new(308, 0, 0),
            AreaSymbol::PrivateArea => omap::Code::new(520, 0, 0),
            AreaSymbol::PavedAreaWithBoundary => omap::Code::new(501, 0, 0),
            AreaSymbol::ShallowWaterWithSolidBankLine => omap::Code::new(302, 0, 0),
            AreaSymbol::UncrossableWaterWithBankLine => omap::Code::new(301, 0, 0),
            AreaSymbol::GiganticBoulder => omap::Code::new(206, 0, 0),
            AreaSymbol::Building => omap::Code::new(521, 0, 0),
            AreaSymbol::WhiteForest => omap::Code::new(405, 0, 0),
        }
    }

    pub fn min_size(&self, scale: &Scale) -> f64 {
        let a = match self {
            AreaSymbol::WhiteForest => 64.,
            AreaSymbol::RoughOpenLand => 225.,
            AreaSymbol::OpenLand => 64.,
            AreaSymbol::SandyGround => 225.,
            AreaSymbol::BareRock => 225.,
            AreaSymbol::LightGreen => 225.,
            AreaSymbol::MediumGreen => 110.,
            AreaSymbol::DarkGreen => 64.,
            AreaSymbol::Marsh => 45.,
            AreaSymbol::PrivateArea => 225.,
            AreaSymbol::PavedAreaWithBoundary => 225.,
            AreaSymbol::ShallowWaterWithSolidBankLine => 64.,
            AreaSymbol::UncrossableWaterWithBankLine => 64.,
            AreaSymbol::GiganticBoulder => 67.,
            AreaSymbol::Building => 56.,
        };
        let multiplier = match scale {
            Scale::S10_000 => 4. / 9.,
            Scale::S15_000 => 1.,
        };
        a * multiplier
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum LineSymbol {
    BasemapContour,
    FormLine,
    Contour,
    IndexContour,
    NegBasemapContour,
    SmallCrossableWatercourse,
    Cliff,
    ImpassableCliff,
}

impl LineSymbol {
    pub fn get_code(&self) -> omap::Code {
        match self {
            LineSymbol::BasemapContour => omap::Code::new(101, 2, 0),
            LineSymbol::FormLine => omap::Code::new(103, 0, 0),
            LineSymbol::Contour => omap::Code::new(101, 0, 0),
            LineSymbol::IndexContour => omap::Code::new(102, 0, 0),
            LineSymbol::NegBasemapContour => omap::Code::new(101, 3, 0),
            LineSymbol::SmallCrossableWatercourse => omap::Code::new(305, 0, 0),
            LineSymbol::Cliff => omap::Code::new(202, 0, 0),
            LineSymbol::ImpassableCliff => omap::Code::new(201, 0, 0),
        }
    }

    pub fn min_length(&self, scale: Scale, is_closed: bool) -> f64 {
        let l = match self {
            LineSymbol::BasemapContour => 3.,
            LineSymbol::FormLine => {
                if is_closed {
                    150.
                } else {
                    250.
                }
            }
            LineSymbol::Contour | LineSymbol::IndexContour => {
                if is_closed {
                    120.
                } else {
                    10.
                }
            }
            LineSymbol::NegBasemapContour => 3.,
            LineSymbol::SmallCrossableWatercourse => 15.,
            LineSymbol::Cliff => 9.,
            LineSymbol::ImpassableCliff => 9.,
        };
        let multiplier = match scale {
            Scale::S10_000 => 2. / 3.,
            Scale::S15_000 => 1.,
        };
        l * multiplier
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum PointSymbol {
    SlopeLineFormLine,
    SlopeLineContour,
    DotKnoll,
    ElongatedDotKnoll,
    UDepression,
    SmallBoulder,
    LargeBoulder,
}

impl PointSymbol {
    pub fn get_code(&self) -> omap::Code {
        match self {
            PointSymbol::SlopeLineFormLine => omap::Code::new(103, 1, 0),
            PointSymbol::SlopeLineContour => omap::Code::new(101, 1, 0),
            PointSymbol::DotKnoll => omap::Code::new(109, 0, 0),
            PointSymbol::ElongatedDotKnoll => omap::Code::new(110, 0, 0),
            PointSymbol::UDepression => omap::Code::new(111, 0, 0),
            PointSymbol::SmallBoulder => omap::Code::new(204, 0, 0),
            PointSymbol::LargeBoulder => omap::Code::new(205, 0, 0),
        }
    }
}
