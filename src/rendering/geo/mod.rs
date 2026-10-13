//! Converts paper millimetres to geographic and normalized Mercator coordinates.

mod mercator;
mod transform;

pub use mercator::{Mercator, from_mercator, to_mercator};
pub use transform::GeoTransform;

pub const EARTH_CIRCUMFERENCE_M: f64 = 40_075_016.686;
