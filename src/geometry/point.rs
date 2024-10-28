#[allow(dead_code)]
use super::Line;

pub trait Point {
    fn new(x: f64, y: f64, z: f64) -> Self;

    fn get_x(&self) -> f64;

    fn get_y(&self) -> f64;

    fn get_z(&self) -> f64;

    fn translate(&mut self, dx: f64, dy: f64, dz: f64);

    fn normal(&self) -> Self;

    fn scale(&mut self, l: f64);

    fn closest_point_on_line_segment(&self, line: &Line) -> Self;

    fn dist_to_line_segment_squared(&self, line: &Line) -> f64;

    fn consecutive_orientation(&self, a: &impl Point, b: &impl Point) -> f64 {
        (a.get_x() - self.get_x()) * (b.get_y() - self.get_y())
            - (a.get_y() - self.get_y()) * (b.get_x() - self.get_x())
    }

    fn squared_euclidean_distance(&self, other: &impl Point) -> f64 {
        (self.get_x() - other.get_x()).powi(2) + (self.get_y() - other.get_y()).powi(2)
    }

    fn cross_product(&self, other: &impl Point) -> f64 {
        self.get_x() * other.get_y() - other.get_x() * self.get_y()
    }

    fn dot(&self, other: &impl Point) -> f64 {
        self.get_x() * other.get_x() + self.get_y() * other.get_y()
    }

    fn norm(&mut self) {
        let l = self.length();
        self.scale(1. / l);
    }

    fn length(&self) -> f64 {
        (self.get_x() * self.get_x() + self.get_y() * self.get_y()).sqrt()
    }
}
