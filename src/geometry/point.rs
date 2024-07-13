pub trait Point {
    fn consecutive_orientation(&self, a: &Self, b: &Self) -> f64;

    fn squared_euclidean_distance(&self, other: &Self) -> f64;

    fn cross_product(&self, other: &Self) -> f64;

    fn dist_to_line_squared(&self, a: &Self, b: &Self) -> f64;

    fn dot(&self, other: &Self) -> f64;

    fn norm(self) -> Self;

    fn length(&self) -> f64;
}
