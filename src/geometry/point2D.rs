use super::{Point, Point5D};

use std::convert::From;
use std::ops::{Add, Sub};

#[derive(Copy, Clone, Debug)]
pub struct Point2D {
    pub x: f64,
    pub y: f64,
}

impl Point2D {
    fn new(x: f64, y: f64) -> Point2D {
        Point2D { x, y }
    }
}

impl From<Point5D> for Point2D {
    fn from(p5: Point5D) -> Point2D {
        Point2D::new(p5.x, p5.y)
    }
}

impl Add for Point2D {
    type Output = Point2D;

    fn add(self, other: Point2D) -> Point2D {
        Point2D {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl Sub for Point2D {
    type Output = Self;

    fn sub(self, other: Self) -> Self::Output {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

impl Point for Point2D {
    fn consecutive_orientation(&self, a: &Point2D, b: &Point2D) -> f64 {
        (*a - *self).cross_product(&(*b - *self))
    }

    fn squared_euclidean_distance(&self, other: &Point2D) -> f64 {
        (self.x - other.x).powi(2) + (self.y - other.y).powi(2)
    }

    fn cross_product(&self, other: &Point2D) -> f64 {
        self.x * other.y - other.x * self.y
    }
}
