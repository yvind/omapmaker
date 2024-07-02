use super::{Point, Point5D};

use std::convert::From;
use std::hash::{Hash, Hasher};
use std::ops::{Add, Sub};

#[derive(Copy, Clone, Debug)]
pub struct Point2D {
    pub x: f64,
    pub y: f64,
}

impl Point2D {
    pub fn new(x: f64, y: f64) -> Point2D {
        Point2D { x, y }
    }

    pub fn get_boundary_dist() {}

    pub fn get_box_edge_index() {}

    pub fn to_map_coordinates(&self) -> (i32, i32) {
        (0, 0)
    }
}

impl From<Point5D> for Point2D {
    fn from(p5: Point5D) -> Point2D {
        Point2D::new(p5.x, p5.y)
    }
}

// don't know if it works. Overflow prone.
impl Hash for Point2D {
    fn hash<H: Hasher>(&self, state: &mut H) {
        ((self.x * 1_000_000.) as i64).hash(state);
        ((self.y * 1_000_000.) as i64).hash(state);
    }
}

impl Eq for Point2D {}

impl PartialEq for Point2D {
    fn eq(&self, other: &Self) -> bool {
        if (self.x - other.x).abs() > f64::EPSILON * 2.0 {
            false
        } else if (self.y - other.y).abs() > f64::EPSILON * 2.0 {
            false
        } else {
            true
        }
    }
}

impl Add for Point2D {
    type Output = Point2D;

    fn add(self, rhs: Point2D) -> Point2D {
        Point2D {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl Sub for Point2D {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
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

    fn dist_to_line_squared(&self, a: &Self, b: &Self) -> f64 {
        let diff = *b - *a;

        (self.cross_product(&diff) + b.cross_product(a))
            .abs()
            .powi(2)
            / b.squared_euclidean_distance(a)
    }
}
