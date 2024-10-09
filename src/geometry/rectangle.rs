use crate::geometry::Point2D;

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

    pub fn contains(&self, point: &Point2D) -> bool {
        point.x >= self.min.x
            && point.y >= self.min.y
            && point.x <= self.max.x
            && point.y <= self.max.y
    }
}
