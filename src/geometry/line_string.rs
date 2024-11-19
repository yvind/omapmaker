use super::Polygon;
use geo::BooleanOps;
pub use geo::LineString;

pub trait MapLineString {
    fn line_string_signed_area(&self) -> Option<f64>;
    fn inner_line(&self, other: &LineString) -> Option<Polygon>;
}

impl MapLineString for LineString {
    fn line_string_signed_area(&self) -> Option<f64> {
        if self.0.len() < 3 || !self.is_closed() {
            return None;
        }
        let mut area: f64 = 0.;
        for i in 0..self.0.len() - 1 {
            area += self.0[i].x * self.0[i + 1].y - self.0[i].y * self.0[i + 1].x;
        }
        Some(0.5 * area)
    }

    fn inner_line(&self, other: &LineString) -> Option<Polygon> {
        let p1 = Polygon::new(self.clone(), vec![]);
        let p2 = Polygon::new(other.clone(), vec![]);

        let multipolygon = p1.intersection(&p2);
        if multipolygon.0.is_empty() {
            None
        } else if multipolygon.0.len() == 1 {
            let mut i = multipolygon.into_iter().next().unwrap();

            if i.exterior().line_string_signed_area().unwrap() < 0. {
                i.exterior_mut(|e| e.0.reverse());
            }

            Some(i)
        } else {
            panic!("Multiple disjoint overlaps between the convex hull and the clipping region");
        }
    }
}

#[cfg(test)]
mod tests {
    use geo::Area;

    use super::super::*;

    #[test]
    fn test_signed_area_linestring() {
        let line = LineString::new(vec![
            Coord { x: 0., y: 100. },
            Coord { x: 0., y: 0. },
            Coord { x: 100., y: 0. },
            Coord { x: 100., y: 100. },
            Coord { x: 0., y: 100. },
        ]);

        assert_eq!(Some(10_000.), line.line_string_signed_area());
    }

    #[test]
    fn test_geo_agreed_signed_area_linestring() {
        let line = LineString::new(vec![
            Coord { x: 0., y: 100. },
            Coord { x: 0., y: 0. },
            Coord { x: 100., y: 0. },
            Coord { x: 100., y: 100. },
            Coord { x: 0., y: 100. },
        ]);

        let own_area = line.line_string_signed_area().unwrap();

        let polygon = Polygon::new(line, vec![]);

        assert_eq!(own_area, polygon.signed_area());
    }
}
