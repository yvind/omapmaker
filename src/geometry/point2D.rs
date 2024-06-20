use std::conversion::From;

#[derive(Copy, Clone, Debug)]
pub struct Point2D{
    pub x: f64,
    pub y: f64,
}

impl Point2D{
    pub fn new() -> Point2D{
        return Point2D{ x = 0., y = 0.,};
    }

    pub fn new(x: f64, y: f64) -> Point2D{
        return Point2D{x: x, y: y};
    }
}

impl From<Point5D> for Point2D{
    fn from(p5: Point5D) -> Point2D{
        return Point2D::new(x: p5.x, y: p5.y);
    }
}