#[allow(dead_code)]

pub trait Point {
    fn new(x: f64, y: f64, z: f64) -> Self;

    fn get_x(&self) -> f64;

    fn get_y(&self) -> f64;

    fn get_z(&self) -> f64;

    fn translate(&mut self, dx: f64, dy: f64, dz: f64);

    fn closest_point_on_line_segment(&self, a: &impl Point, b: &impl Point) -> Self;

    fn consecutive_orientation(&self, a: &impl Point, b: &impl Point) -> f64;

    fn squared_euclidean_distance(&self, other: &impl Point) -> f64;

    fn dist_to_line_segment_squared(&self, a: &impl Point, b: &impl Point) -> f64;

    fn cross_product(&self, other: &impl Point) -> f64;

    fn dot(&self, other: &impl Point) -> f64;

    fn norm(&mut self);

    fn length(&self) -> f64;

    fn normal(&self) -> Self;

    fn scale(&mut self, l: f64);
}
