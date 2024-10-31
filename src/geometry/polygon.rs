use super::{Coord, MultiLineString};
pub use geo::Polygon;
use geo::{
    algorithm::line_intersection::{line_intersection, LineIntersection},
    Contains, Intersects,
};

pub trait MapPolygon {
    fn clip(&self, lines: &mut MultiLineString);
}

impl MapPolygon for Polygon {
    fn clip(&self, lines: &mut MultiLineString) {
        let mut i = 0;
        while i < lines.0.len() {
            let mut intersection_points: Vec<Coord<f64>> = vec![];
            if self.contains(&lines.0[i]) {
                i += 1;
            } else if !self.intersects(&lines.0[i]) {
                lines.0.swap_remove(i);
            } else {
                for bound_segment in self.exterior().lines() {
                    for segment in lines.0[i].lines().enumerate() {
                        if let Some(wp) = line_intersection(bound_segment, segment) {
                            match wp {
                                LineIntersection::SinglePoint {
                                    intersection: p,
                                    is_proper: _b,
                                } => intersection_points.push(p),
                                LineIntersection::Collinear { intersection: _l } => (),
                            }
                        }
                    }
                }
            }

            for ip in intersection_points {}
        }
    }
}
