use super::{Line, Point, PointLaz};

use std::convert::From;
use std::ops::{Add, Sub};

#[derive(Copy, Clone, Debug)]
pub struct Point2D {
    pub x: f64,
    pub y: f64,
}

impl Point2D {
    pub fn default() -> Point2D {
        Point2D { x: 0., y: 0. }
    }

    pub fn new(x: f64, y: f64) -> Point2D {
        Point2D { x, y }
    }

    pub fn get_distance_along_line_square_sum(
        &self,
        other: &Point2D,
        line: &Line,
        epsilon: f64,
    ) -> Result<f64, &'static str> {
        let length = line.len();

        let last_index = self.on_edge_index(line, epsilon)?;
        let first_index = other.on_edge_index(line, epsilon)?;

        if !line.is_closed() {
            if last_index > first_index {
                return Err("The other point is before the first point on the line");
            }

            if last_index == first_index {
                let prev_vertex = &line.vertices[first_index];

                if self.squared_euclidean_distance(prev_vertex)
                    > other.squared_euclidean_distance(prev_vertex)
                {
                    return Err("The other point is before the first point on the line");
                }
            }
        }

        if last_index == first_index {
            let prev_vertex = &line.vertices[first_index];

            if self.squared_euclidean_distance(prev_vertex)
                <= other.squared_euclidean_distance(prev_vertex)
            {
                return Ok(self.squared_euclidean_distance(other));
            }
        }

        let range = Line::get_range_on_line(last_index, first_index, length);

        let mut dist = 0.;

        let mut prev_vertex = self;
        for i in range {
            let next_vertex = &line.vertices[i];

            dist += prev_vertex.squared_euclidean_distance(next_vertex);
            prev_vertex = next_vertex;
        }
        dist += other.squared_euclidean_distance(prev_vertex);

        Ok(dist)
    }

    pub fn to_map_coordinates(self) -> Result<(i32, i32), &'static str> {
        // 1_000 map units = 15m
        // 1_000 / 15 = 66.66...

        let x = (self.x * 66.66666).round();
        let y = -(self.y * 66.66666).round();

        if (x > 2.0_f64.powi(32) - 1.) || (y > 2.0_f64.powi(32) - 1.) {
            Err("map coordinate overflow, try a smaller laz file")
        } else {
            Ok((x as i32, y as i32))
        }
    }

    pub fn on_edge_index(&self, hull: &Line, epsilon: f64) -> Result<usize, &'static str> {
        let len = hull.vertices.len();
        for i in 0..len - 1 {
            if self.dist_to_line_segment_squared(&hull.vertices[i], &hull.vertices[i + 1])
                < epsilon * epsilon
            {
                return Ok(i);
            }
        }
        Err("The given point is not on the the line")
    }
}

impl From<PointLaz> for Point2D {
    fn from(p5: PointLaz) -> Point2D {
        Point2D::new(p5.x, p5.y)
    }
}

impl Eq for Point2D {}

impl PartialEq for Point2D {
    fn eq(&self, other: &Self) -> bool {
        self.x == other.x && self.y == other.y
    }
}

impl Add for &Point2D {
    type Output = Point2D;

    fn add(self, rhs: Self) -> Self::Output {
        Point2D {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl Sub for &Point2D {
    type Output = Point2D;

    fn sub(self, rhs: Self) -> Self::Output {
        Point2D {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl Point for Point2D {
    fn new(x: f64, y: f64, _z: f64) -> Point2D {
        Point2D { x, y }
    }

    fn get_x(&self) -> f64 {
        self.x
    }

    fn get_y(&self) -> f64 {
        self.y
    }

    fn get_z(&self) -> f64 {
        0.
    }

    fn translate(&mut self, dx: f64, dy: f64, _dz: f64) {
        self.x += dx;
        self.y += dy;
    }

    fn closest_point_on_line_segment(&self, a: &impl Point, b: &impl Point) -> Self {
        let mut diff = self.clone();
        diff.x = b.get_x() - a.get_x();
        diff.y = b.get_y() - a.get_y();
        let len = diff.length();
        diff.norm();

        let mut s = self.clone();
        s.translate(-a.get_x(), -a.get_y(), 0.);

        let image = s.dot(&diff).max(0.).min(len);

        Point2D {
            x: a.get_x() + diff.x * image,
            y: a.get_y() + diff.y * image,
        }
    }

    fn consecutive_orientation(&self, a: &impl Point, b: &impl Point) -> f64 {
        (a.get_x() - self.x) * (b.get_y() - self.y) - (a.get_y() - self.y) * (b.get_x() - self.x)
    }

    fn squared_euclidean_distance(&self, other: &impl Point) -> f64 {
        (self.x - other.get_x()).powi(2) + (self.y - other.get_y()).powi(2)
    }

    fn cross_product(&self, other: &impl Point) -> f64 {
        self.x * other.get_y() - other.get_x() * self.y
    }

    fn dist_to_line_segment_squared(&self, a: &impl Point, b: &impl Point) -> f64 {
        self.squared_euclidean_distance(&self.closest_point_on_line_segment(a, b))
    }

    fn dot(&self, other: &impl Point) -> f64 {
        self.x * other.get_x() + self.y * other.get_y()
    }

    fn norm(&mut self) {
        let l = self.length();
        self.scale(1. / l);
    }

    fn length(&self) -> f64 {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    fn normal(&self) -> Self {
        Self {
            x: self.y,
            y: -self.x,
        }
    }

    fn scale(&mut self, l: f64) {
        self.x *= l;
        self.y *= l;
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn on_edge_index_middle() -> Result<(), &'static str> {
        let mut line = Line::new(Point2D::new(-1., -1.), Point2D::new(-0.5, -1.3));

        line.push(Point2D::new(0.1, 1.2));
        line.push(Point2D::new(1.1, 2.2));
        line.push(Point2D::new(0.1, 3.2));
        line.push(Point2D::new(-1.1, 1.2));
        line.push(line.vertices[0]);

        let point = Point2D::new(0.7, 1.6);

        assert_eq!(point.on_edge_index(&line, 0.2)?, 2);
        Ok(())
    }

    #[test]
    fn on_edge_index_first() -> Result<(), &'static str> {
        let mut line = Line::new(Point2D::new(-1., -1.), Point2D::new(-0.5, -1.3));

        line.push(Point2D::new(0.1, 1.2));
        line.push(Point2D::new(1.1, 2.2));
        line.push(Point2D::new(0.1, 3.2));
        line.push(Point2D::new(-1.1, 1.2));
        line.push(line.vertices[0]);

        let point = Point2D::new(-0.75, -1.1);

        assert_eq!(point.on_edge_index(&line, 0.2)?, 0);
        Ok(())
    }

    #[test]
    fn on_edge_index_last() -> Result<(), &'static str> {
        let mut line = Line::new(Point2D::new(-1., -1.), Point2D::new(-0.5, -1.3));

        line.push(Point2D::new(0.1, 1.2));
        line.push(Point2D::new(1.1, 2.2));
        line.push(Point2D::new(0.1, 3.2));
        line.push(Point2D::new(-1.1, 1.2));
        line.push(line.vertices[0]);

        let point = Point2D::new(-1.05, 0.);

        assert_eq!(point.on_edge_index(&line, 0.2)?, 5);
        Ok(())
    }

    #[test]
    fn distance_along_line_closed() -> Result<(), &'static str> {
        let mut line = Line::new(Point2D::new(-1., -1.), Point2D::new(-1., 1.));

        line.push(Point2D::new(1., 1.));
        line.push(Point2D::new(1., -1.));
        line.push(line.vertices[0]);

        let point = Point2D::new(-1., 0.);
        let other = Point2D::new(-1., -0.1);

        assert_eq!(
            point.get_distance_along_line_square_sum(&other, &line, 0.1)?,
            13.81
        );
        Ok(())
    }

    #[test]
    fn distance_along_line_open() -> Result<(), &'static str> {
        let mut line = Line::new(Point2D::new(-1., -1.), Point2D::new(-1., 1.));

        line.push(Point2D::new(1., 1.));
        line.push(Point2D::new(1., -1.));

        let point = Point2D::new(-1., 0.);
        let other = Point2D::new(1., 0.);

        assert_eq!(
            point.get_distance_along_line_square_sum(&other, &line, 0.1)?,
            6.
        );
        Ok(())
    }

    #[test]
    #[should_panic(expected = "The other point is before the first point on the line")]
    fn distance_along_line_open_panic() {
        let mut line = Line::new(Point2D::new(-1., -1.), Point2D::new(-1., 1.));

        line.push(Point2D::new(1., 1.));
        line.push(Point2D::new(1., -1.));

        let other = Point2D::new(-1., 0.);
        let point = Point2D::new(1., 0.);

        let a = point.get_distance_along_line_square_sum(&other, &line, 0.1);

        match a {
            Err(e) => panic!("{}", e),
            Ok(t) => assert_eq!(t, 6.),
        }
    }
}
