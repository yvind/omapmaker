use super::{LineString, MapLineString, MultiLineString, Polygon};
pub use geo::MultiPolygon;
use geo::{Contains, HasDimensions};

pub trait MapMultiPolygon {
    fn from_contours(
        contours: MultiLineString,
        convex_hull: &LineString,
        min_size: f64,
        hint: bool,
    ) -> MultiPolygon;
}

impl MapMultiPolygon for MultiPolygon {
    fn from_contours(
        mut contours: MultiLineString,
        convex_hull: &LineString,
        min_size: f64,
        hint: bool,
    ) -> MultiPolygon {
        let mut polygons = vec![];

        if contours.0.is_empty() {
            // everywhere is either above or below the limit
            // needs to use the hint to classify everywhere correctly
            if hint {
                polygons.push(Polygon::new(convex_hull.clone(), vec![]));
            }
            return MultiPolygon::new(polygons);
        }

        // add all contours of the right orientation to its own polygon
        let mut i = 0;
        while i < contours.0.len() {
            let contour = &contours.0[i];
            let area = contour.line_string_signed_area().unwrap();
            if area > -min_size / 10. && area < min_size {
                contours.0.swap_remove(i);
            } else if area >= min_size {
                polygons.push(Polygon::new(contours.0.swap_remove(i), vec![]));
            } else {
                i += 1;
            }
        }

        // a background polygon must to be added if only holes exist
        if polygons.is_empty() && !contours.is_empty() {
            polygons.push(Polygon::new(convex_hull.clone(), vec![]));
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
        MultiPolygon::new(polygons)
    }
}
