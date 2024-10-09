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

impl Display for Symbol {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "{}", *self as isize)
    }
}
