use std::ops::Add;

#[allow(dead_code)]
use super::{Point, Point2D};
use las::Bounds;

#[derive(Clone, Debug)]
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

    pub fn contains_rectangle(&self, other: &Rectangle) -> bool {
        self.contains(&other.min) && self.contains(&other.max)
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

    pub fn shrink(&mut self, v: f64) {
        self.min.x += v;
        self.min.y += v;
        self.max.x -= v;
        self.max.y -= v;
    }
}

impl From<Bounds> for Rectangle {
    fn from(value: Bounds) -> Self {
        Rectangle {
            min: Point2D {
                x: value.min.x,
                y: value.min.y,
            },
            max: Point2D {
                x: value.max.x,
                y: value.max.y,
            },
        }
    }
}

impl Add for &Rectangle {
    type Output = Rectangle;

    fn add(self, rhs: Self) -> Self::Output {
        Rectangle {
            min: Point2D {
                x: self.min.x + rhs.min.x,
                y: self.min.y + rhs.min.y,
            },
            max: Point2D {
                x: self.max.x + rhs.max.x,
                y: self.max.y + rhs.max.y,
            },
        }
    }
}
