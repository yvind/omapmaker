#[derive(Copy, Clone, Debug)]
pub struct Point2D{
    pub x: f64,
    pub y: f64,
}

impl Point2D{
    pub fn new() -> Point2D{
        return Point2D{ x = 0., y = 0.,};
    }
}