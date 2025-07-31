use crate::parameters::BufferRule;

use super::MapLineString;
use geo::{Area, BooleanOps, Buffer, Contains, Simplify};
use geo::{MultiLineString, MultiPolygon, Polygon};

pub trait MapMultiPolygon {
    fn from_contours(
        contours: MultiLineString,
        convex_hull: &Polygon,
        invert: bool,
    ) -> MultiPolygon;

    fn apply_buffer_rule(self, buffer_rule: &BufferRule) -> MultiPolygon;

    fn remove_small_polygons(self, min_size: f64) -> MultiPolygon;
}

impl MapMultiPolygon for MultiPolygon {
    fn from_contours(
        mut contours: MultiLineString,
        convex_hull: &Polygon,
        invert: bool,
    ) -> MultiPolygon {
        let mut polygons = Vec::with_capacity(contours.0.len());

        if contours.0.is_empty() {
            if invert {
                polygons.push(convex_hull.clone())
            }
            return MultiPolygon::new(polygons);
        }

        let mut i = 0;
        while i < contours.0.len() {
            let area = contours.0[i].line_string_signed_area().unwrap();

            if area > 0. {
                polygons.push(Polygon::new(contours.0.swap_remove(i), vec![]));
            } else {
                i += 1;
            }
        }

        // add the holes to the polygons
        for contour in contours {
            for polygon in &mut polygons {
                if polygon.contains(&contour.0[1]) {
                    polygon.interiors_push(contour);
                    break;
                }
            }
        }

        let mut polygons = MultiPolygon::new(polygons);

        // invert the polygons with respect to the convex hull if we want area below the contours
        if invert {
            polygons = convex_hull.difference(&polygons);
        }

        polygons
    }

    fn apply_buffer_rule(self, buffer_rule: &BufferRule) -> MultiPolygon {
        let sign = match buffer_rule.direction {
            crate::parameters::BufferDirection::Grow => 1.,
            crate::parameters::BufferDirection::Shrink => -1.,
        };
        let distance = sign * buffer_rule.amount;
        self.buffer(distance).simplify(crate::SIMPLIFICATION_DIST)
    }

    fn remove_small_polygons(mut self, min_size: f64) -> MultiPolygon {
        let mut i = 0;
        while i < self.0.len() {
            let area = self.0[i].signed_area();

            if area < min_size {
                self.0.swap_remove(i);
            } else {
                i += 1;
            }
        }
        self
    }
}
