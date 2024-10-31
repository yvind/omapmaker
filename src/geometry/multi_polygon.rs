use super::{LineString, MapLineString, MultiLineString, Polygon, PolygonTrigger};
pub use geo::MultiPolygon;
use geo::{Area, Contains};

pub trait MapMultiPolygon {
    fn from_contours(
        contours: MultiLineString,
        convex_hull: &LineString,
        polygon_type: PolygonTrigger,
        min_size: f64,
        epsilon: f64,
        hint: bool,
    ) -> MultiPolygon;
}

impl MapMultiPolygon for MultiPolygon {
    fn from_contours(
        mut contours: MultiLineString,
        convex_hull: &LineString,
        polygon_type: PolygonTrigger,
        min_size: f64,
        epsilon: f64,
        hint: bool,
    ) -> MultiPolygon {
        let mut polygons = vec![];
        let mut unclosed_contours = vec![];

        if contours.0.is_empty() {
            // everywhere is either above or below the limit
            // needs to use the hint to classify everywhere correctly
            if polygon_type as i8 * (2 * hint as i8 - 1) > 0 {
                polygons.push(Polygon::new(convex_hull.clone(), vec![]));
            }
            return MultiPolygon::new(polygons);
        }

        // reverse all contours if we are interested in the polygons that the areas below the contours build, instead of the areas above
        if polygon_type == PolygonTrigger::Below {
            for c in contours.iter_mut() {
                c.0.reverse();
            }
        }

        // filter out all unclosed contours
        let mut i: usize = 0;
        while i < contours.0.len() {
            if !contours.0[i].is_closed() {
                unclosed_contours.push(contours.0.swap_remove(i));
            } else {
                i += 1;
            }
        }

        // for each unclosed contour wander ccw along the convex hull and merge with the first encountered unclosed contour
        while !unclosed_contours.is_empty() {
            let mut best_neighbour = usize::MAX;
            let mut best_boundary_dist = f64::MAX;
            for (j, other) in unclosed_contours.iter().enumerate() {
                let dist = convex_hull
                    .get_distance_along_line(
                        unclosed_contours[0].last_vertex(),
                        other.first_vertex(),
                        epsilon,
                    )
                    .unwrap();
                if dist < best_boundary_dist {
                    best_neighbour = j;
                    best_boundary_dist = dist;
                }
            }

            if best_neighbour == 0 {
                let mut contour = unclosed_contours.swap_remove(0);
                contour.close_by_line(convex_hull, epsilon).unwrap();
                contours.0.push(contour);
            } else {
                let other = unclosed_contours.swap_remove(best_neighbour);
                unclosed_contours[0]
                    .append_by_line(other, convex_hull, epsilon)
                    .unwrap();
            }
        }

        // add all closed contours of the right orientation to its own polygon
        let mut filtered_out: usize = 0;
        i = 0;
        while i < contours.0.len() {
            let contour = &contours.0[i];
            let area: f64 = contour.signed_area();

            if area >= 0. && area < min_size {
                contours.0.swap_remove(i);
                filtered_out += 1;
            } else if area <= 0. && area > -min_size / 10. {
                contours.0.swap_remove(i);
            } else if area >= min_size {
                polygons.push(Polygon::new(contour.clone(), vec![]));
                contours.0.swap_remove(i);
            } else {
                i += 1;
            }
        }

        // a background polygon must to be added if only large holes exist
        if polygons.is_empty() && filtered_out == 0 {
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
