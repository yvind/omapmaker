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

    fn find_intersection_parameter(p1: f64, p2: f64, boundary: f64) -> Option<f64> {
        if p1 == p2 {
            return None;
        }
        let t = (boundary - p1) / (p2 - p1);
        if (0.0..=1.0).contains(&t) {
            Some(t)
        } else {
            None
        }
    }

    // Helper function to find intersection with a vertical or horizontal boundary
    fn find_intersection_with_edge(
        &self,
        p1: &Point2D,
        p2: &Point2D,
        boundary: f64,
        is_vertical: bool,
    ) -> Option<Point2D> {
        if is_vertical {
            let t = (boundary - p1.x) / (p2.x - p1.x);
            if (0.0..=1.0).contains(&t) {
                let y = p1.y + t * (p2.y - p1.y);
                Some(Point2D::new(boundary, y))
            } else {
                None
            }
        } else {
            let t = (boundary - p1.y) / (p2.y - p1.y);
            if (0.0..=1.0).contains(&t) {
                let x = p1.x + t * (p2.x - p1.x);
                Some(Point2D::new(x, boundary))
            } else {
                None
            }
        }
    }

    // Helper function to find intersection point of a line segment with rectangle boundary
    pub fn find_intersection(&self, p1: &Point2D, p2: &Point2D) -> Option<Point2D> {
        // Check intersections with vertical boundaries
        if let Some(t) = Self::find_intersection_parameter(p1.x, p2.x, self.min.x) {
            let y = p1.y + t * (p2.y - p1.y);
            if y >= self.min.y && y <= self.max.y {
                return Some(Point2D::new(self.min.x, y));
            }
        }
        if let Some(t) = Self::find_intersection_parameter(p1.x, p2.x, self.max.x) {
            let y = p1.y + t * (p2.y - p1.y);
            if y >= self.min.y && y <= self.max.y {
                return Some(Point2D::new(self.max.x, y));
            }
        }

        // Check intersections with horizontal boundaries
        if let Some(t) = Self::find_intersection_parameter(p1.y, p2.y, self.min.y) {
            let x = p1.x + t * (p2.x - p1.x);
            if x >= self.min.x && x <= self.max.x {
                return Some(Point2D::new(x, self.min.y));
            }
        }
        if let Some(t) = Self::find_intersection_parameter(p1.y, p2.y, self.max.y) {
            let x = p1.x + t * (p2.x - p1.x);
            if x >= self.min.x && x <= self.max.x {
                return Some(Point2D::new(x, self.max.y));
            }
        }

        None
    }

    // Helper function to find both intersection points of a line segment with rectangle
    pub fn find_segment_intersections(
        &self,
        p1: &Point2D,
        p2: &Point2D,
    ) -> Option<(Point2D, Point2D)> {
        let mut intersections = Vec::new();

        // Find all intersections
        let candidates = [
            self.find_intersection_with_edge(p1, p2, self.min.x, true),
            self.find_intersection_with_edge(p1, p2, self.max.x, true),
            self.find_intersection_with_edge(p1, p2, self.min.y, false),
            self.find_intersection_with_edge(p1, p2, self.max.y, false),
        ];

        for point in candidates.iter().flatten() {
            if self.contains(point) {
                intersections.push(point.clone());
            }
        }

        // If we found exactly two intersections, return them
        if intersections.len() == 2 {
            Some((intersections[0].clone(), intersections[1].clone()))
        } else {
            None
        }
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
