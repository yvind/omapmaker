use super::Point2D;

#[derive(Clone, Debug)]
pub struct Line {
    pub start: Point2D,
    pub end: Point2D,
}

impl Line {
    pub fn new(start: Point2D, end: Point2D) -> Line {
        Line { start, end }
    }

    pub fn length_squared(&self) -> f64 {
        (self.end.x - self.start.x).powi(2) + (self.end.y - self.start.y).powi(2)
    }

    pub fn intersection(&self, other: &Line) -> Option<Point2D> {
        let denom = (self.end.x - self.start.x) * (other.end.y - other.start.y)
            - (self.end.y - self.start.y) * (other.end.x - other.start.x);

        // lines are parallel
        if denom == 0.0 {
            return None;
        }

        let ua = ((other.end.x - other.start.x) * (self.start.y - other.start.y)
            - (other.end.y - other.start.y) * (self.start.x - other.start.x))
            / denom;

        let ub = ((self.end.x - self.start.x) * (self.start.y - other.start.y)
            - (self.end.y - self.start.y) * (self.start.x - other.start.x))
            / denom;

        // Lines intersect but not inside the segments
        if ua < 0.0 || ua > 1.0 || ub < 0.0 || ub > 1.0 {
            return None;
        }

        Some(Point2D {
            x: self.start.x + ua * (self.end.x - self.start.x),
            y: self.start.y + ua * (self.end.y - self.start.y),
        })
    }
}

impl From<&[Point2D]> for Line {
    fn from(window: &[Point2D]) -> Self {
        Line::new(window[0], window[1])
    }
}

impl From<[&Point2D; 2]> for Line {
    fn from(window: [&Point2D; 2]) -> Self {
        Line::new(*window[0], *window[1])
    }
}
