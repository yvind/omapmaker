use std::convert::Into;

pub struct Point5D{
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub r: u8,
    pub i: u32,
}


impl Point5D{
    pub fn dist_squared(&self, b: &Point5D) -> f64{
        (self.x-b.x).powi(2) + (self.y-b.y).powi(2)
    }

    pub fn consecutive_orientation(&self, a: &Point5D, b: &Point5D) -> f64{
        (a.x-self.x)*(b.y-self.y) - (a.y-self.y)*(b.x-self.x)
    }
}

impl Into<Point2D> for Point5D{
    fn into(&self) -> Point2D{
        Point2D::new(self.x, self.y)
    }
}