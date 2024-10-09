#[allow(dead_code)]
use super::{Point, Point2D};

pub struct Rectangle {
    pub min: Point2D,
    pub max: Point2D,
}

impl Rectangle {
    pub fn default() -> Rectangle {
        Rectangle {
            min: Point2D::default(),
            max: Point2D::default(),
        }
    }

    pub fn contains(&self, point: &impl Point) -> bool {
        point.get_x() >= self.min.x
            && point.get_y() >= self.min.y
            && point.get_x() <= self.max.x
            && point.get_y() <= self.max.y
    }

    pub fn touch(&self, other: &Rectangle) -> bool {
        !(self.max.x <= other.min.x
            || self.min.x >= other.max.x
            || self.max.y <= other.min.y
            || self.min.y >= other.max.y)
    }

    pub fn touch_margin(&self, other: &Rectangle, margin: f64) -> bool {
        !(self.max.x < other.min.x - margin
            || self.min.x > other.max.x + margin
            || self.max.y < other.min.y - margin
            || self.min.y > other.max.y + margin)
    }
}
