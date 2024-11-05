#![allow(dead_code)]

use std::fmt::{Display, Formatter, Result};

#[derive(Copy, Clone, Debug)]
pub enum Symbol {
    Contour = 0,
    SlopelineContour = 1,
    BasemapContour = 2,
    IndexContour = 3,
    Formline = 5,
    SlopelineFormline = 6,
    SmallBoulder = 34,
    LargeBoulder = 35,
    GiganticBoulder = 37,
    SandyGround = 48,
    BareRock = 49,
    RoughOpenLand = 79,
    LightGreen = 83,
    MediumGreen = 86,
    DarkGreen = 90,
    Building = 140,
}

impl Symbol {
    pub fn min_size(&self) -> f64 {
        match self {
            Symbol::Contour => 0.,
            Symbol::SlopelineContour => 0.,
            Symbol::BasemapContour => 0.,
            Symbol::IndexContour => 0.,
            Symbol::Formline => 0.,
            Symbol::SlopelineFormline => 0.,
            Symbol::SmallBoulder => 0.,
            Symbol::LargeBoulder => 0.,
            Symbol::GiganticBoulder => 10.,
            Symbol::SandyGround => 225.,
            Symbol::BareRock => 225.,
            Symbol::RoughOpenLand => 225.,
            Symbol::LightGreen => 225.,
            Symbol::MediumGreen => 110.,
            Symbol::DarkGreen => 64.,
            Symbol::Building => 0.,
        }
    }
}

impl Display for Symbol {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "{}", *self as u8)
    }
}
