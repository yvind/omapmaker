//! Resolves OMAP objects and their symbol definitions into drawable primitives.

mod area;
mod color;
mod context;
mod geometry;
mod line;
mod map;
mod point;
mod primitives;

pub use geometry::{dash_runs, triangulate_rings};
pub use map::draw_objects;
pub use primitives::{AreaPattern, BorderSide, Kind, LineDash, PointMark, Primitive};
