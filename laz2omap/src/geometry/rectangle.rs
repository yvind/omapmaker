use geo::{Coord, LineString, Rect};
use las::Bounds;

pub trait MapRect {
    fn from_bounds(value: Bounds) -> Rect;
    fn into_line_string(self) -> LineString;
    fn touch_margin(&self, other: &Rect, margin: f64) -> bool;
}

impl MapRect for Rect {
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

    fn into_line_string(self) -> LineString {
        LineString::new(vec![
            Coord {
                x: self.min().x,
                y: self.max().y,
            },
            self.min(),
            Coord {
                x: self.max().x,
                y: self.min().y,
            },
            self.max(),
            Coord {
                x: self.min().x,
                y: self.max().y,
            },
        ])
    }

    fn touch_margin(&self, other: &Rect, margin: f64) -> bool {
        !(self.max().x < other.min().x - margin
            || self.min().x > other.max().x + margin
            || self.max().y < other.min().y - margin
            || self.min().y > other.max().y + margin)
    }
}
