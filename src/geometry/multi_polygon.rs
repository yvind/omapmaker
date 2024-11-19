use super::{LineString, MapLineString, MultiLineString, Polygon};
pub use geo::MultiPolygon;
use geo::{BooleanOps, Contains};

pub trait MapMultiPolygon {
    fn from_contours(
        contours: MultiLineString,
        convex_hull: &LineString,
        min_size: f64,
        invert: bool,
    ) -> MultiPolygon;
}

impl MapMultiPolygon for MultiPolygon {
    fn from_contours(
        mut contours: MultiLineString,
        convex_hull: &LineString,
        min_size: f64,
        invert: bool,
    ) -> MultiPolygon {
        let mut polygons = Vec::with_capacity(contours.0.len());

        if contours.0.is_empty() {
            if invert {
                polygons.push(Polygon::new(convex_hull.clone(), vec![]))
            }
            return MultiPolygon::new(polygons);
        }

        // add all contours of the right orientation to its own polygon
        let invert_sign = -(invert as i8 * 2 - 1) as f64;

        let mut i = 0;
        while i < contours.0.len() {
            let contour = &contours.0[i];
            let area = contour.line_string_signed_area().unwrap();
            let filter_area = area * invert_sign;
            if filter_area > -min_size / 10. && filter_area < min_size {
                contours.0.swap_remove(i);
            } else if area > 0. {
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
            let hull = Polygon::new(convex_hull.clone(), vec![]);
            polygons = hull.xor(&polygons);

            // some edge connected polygons makes
            // it through the size filter for some reason
        }

        polygons
    }
}
