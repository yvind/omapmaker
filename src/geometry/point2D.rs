use super::{Line, Point, Point5D};

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

    pub fn get_distance_along_hull(
        &self,
        other: &Point2D,
        convex_hull: &Line,
        epsilon: f64,
    ) -> Result<f64, &'static str> {
        let length = convex_hull.len();

        let last_index = self.on_edge_index(&convex_hull, epsilon)?;
        let first_index = other.on_edge_index(&convex_hull, epsilon)?;

        if last_index == first_index {
            let prev_vertex = &convex_hull.vertices[first_index];

            if self.squared_euclidean_distance(prev_vertex)
                <= other.squared_euclidean_distance(prev_vertex)
            {
                return Ok(self.squared_euclidean_distance(other));
            }
        }

        let range = Line::get_range_on_convex_hull(last_index, first_index, length);

        let mut dist = 0.;

        let mut prev_vertex = self;
        for i in range {
            let next_vertex = &convex_hull.vertices[i];

            dist += prev_vertex.squared_euclidean_distance(next_vertex);
            prev_vertex = next_vertex;
        }
        dist += other.squared_euclidean_distance(prev_vertex);

        Ok(dist)
    }

    pub fn to_map_coordinates(&self) -> Result<(i32, i32), &'static str> {
        let x = (self.x * 1_000_000.).round();
        let y = (self.y * 1_000_000.).round();

        if (x > 2.0_f64.powi(32) - 1.) || (y > 2.0_f64.powi(32) - 1.) {
            Err("map coordinate overflow, try a smaller laz file")
        } else {
            Ok((x as i32, y as i32))
        }
    }

    pub fn on_edge_index(&self, hull: &Line, epsilon: f64) -> Result<usize, &'static str> {
        let len = hull.vertices.len();
        for i in 0..len {
            if self.dist_to_line_squared(&hull.vertices[i], &hull.vertices[(i + 1) % len])
                < epsilon.powi(2)
            {
                return Ok(i);
            }
        }
        Err("The given point is not on the edge of the convex hull")
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
        ((self.x * 10_000_000.) as i64).hash(state);
        ((self.y * 10_000_000.) as i64).hash(state);
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
