pub trait Point {
    fn closest_point_on_line_segment(&self, a: &Self, b: &Self) -> Self;

    fn consecutive_orientation(&self, a: &Self, b: &Self) -> f64;

    fn squared_euclidean_distance(&self, other: &Self) -> f64;

    fn dist_to_line_segment_squared(&self, a: &Self, b: &Self) -> f64;

    fn cross_product(&self, other: &Self) -> f64;

    fn dot(&self, other: &Self) -> f64;

    fn norm(self) -> Self;

    fn length(&self) -> f64;

    fn normal(self) -> Self;

    fn scale(self, l: f64) -> Self;
}
