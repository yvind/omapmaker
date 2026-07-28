use las::{Bounds, Vector};

pub trait MapRect {
    fn into_bounds(self) -> Bounds;
    fn from_bounds(value: Bounds) -> geo::Rect;
    fn touch_margin(&self, other: &geo::Rect, margin: f64) -> bool;
}

impl MapRect for geo::Rect {
    fn into_bounds(self) -> Bounds {
        Bounds {
            min: Vector {
                x: self.min().x,
                y: self.min().y,
                z: (i32::MIN / 1000) as f64,
            },
            max: Vector {
                x: self.max().x,
                y: self.max().y,
                z: (i32::MAX / 1000) as f64,
            },
        }
    }

    fn from_bounds(value: Bounds) -> geo::Rect {
        geo::Rect::new(
            geo::Coord {
                x: value.min.x,
                y: value.min.y,
            },
            geo::Coord {
                x: value.max.x,
                y: value.max.y,
            },
        )
    }

    fn touch_margin(&self, other: &geo::Rect, margin: f64) -> bool {
        !(self.max().x < other.min().x - margin
            || self.min().x > other.max().x + margin
            || self.max().y < other.min().y - margin
            || self.min().y > other.max().y + margin)
    }
}
