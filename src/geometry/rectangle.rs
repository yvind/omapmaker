use geo::{Coord, Rect};
use las::{Bounds, Vector};

pub trait MapRect {
    fn into_bounds(self, min_z: f64, max_z: f64) -> Bounds;
    fn from_bounds(value: Bounds) -> Rect;
    fn touch_margin(&self, other: &Rect, margin: f64) -> bool;
}

impl MapRect for Rect {
    fn into_bounds(self, min_z: f64, max_z: f64) -> Bounds {
        Bounds {
            min: Vector {
                x: self.min().x,
                y: self.min().y,
                z: min_z,
            },
            max: Vector {
                x: self.max().x,
                y: self.max().y,
                z: max_z,
            },
        }
    }

    fn from_bounds(value: Bounds) -> Rect {
        Rect::new(
            Coord {
                x: value.min.x,
                y: value.min.y,
            },
            Coord {
                x: value.max.x,
                y: value.max.y,
            },
        )
    }

    fn touch_margin(&self, other: &Rect, margin: f64) -> bool {
        !(self.max().x < other.min().x - margin
            || self.min().x > other.max().x + margin
            || self.max().y < other.min().y - margin
            || self.min().y > other.max().y + margin)
    }
}
