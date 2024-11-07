use super::{LineString, MapLineString, MapMultiLineString, MultiLineString, Polygon};
use geo::Contains;
pub use geo::MultiPolygon;

pub trait MapMultiPolygon {
    fn from_contours(
        contours: MultiLineString,
        convex_hull: &LineString,
        min_size: f64,
        epsilon: f64,
        hint: bool,
    ) -> MultiPolygon;
}

impl MapMultiPolygon for MultiPolygon {
    fn from_contours(
        mut contours: MultiLineString,
        convex_hull: &LineString,
        min_size: f64,
        epsilon: f64,
        hint: bool,
    ) -> MultiPolygon {
        let mut polygons = vec![];
        let mut unclosed_contours = vec![];

        if contours.0.is_empty() {
            // everywhere is either above or below the limit
            // needs to use the hint to classify everywhere correctly
            if hint {
                polygons.push(Polygon::new(convex_hull.clone(), vec![]));
            }
            return MultiPolygon::new(polygons);
        }

        // snap all line ends to the convex hull before polygon building
        contours.fix_ends_to_line(convex_hull, epsilon);

        // filter out all unclosed contours and remove closed contours of only 3 vertices
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

        // a background polygon must to be added if only holes exist
        let mut only_holes = true;
        for c in contours.0.iter() {
            if c.line_string_signed_area().unwrap() > 0. {
                only_holes = false;
                break;
            }
        }
        if only_holes {
            polygons.push(Polygon::new(convex_hull.clone(), vec![]));
        }

        // add all closed contours of the right orientation to its own polygon
        i = 0;
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

#[cfg(test)]
mod tests {
    use super::super::*;

    #[test]
    fn test_assemble_polygons() {
        let contours = MultiLineString::new(vec![
            LineString::new(vec![
                Coord { x: 0.0, y: 20.0 },
                Coord { x: 10.0, y: 15.0 },
                Coord { x: 20.0, y: 0.0 },
            ]),
            LineString::new(vec![Coord { x: 30.0, y: 0.0 }, Coord { x: 30.0, y: 100.0 }]),
            LineString::new(vec![
                Coord { x: 10.0, y: 70.0 },
                Coord { x: 20.0, y: 70.0 },
                Coord { x: 20.0, y: 60.0 },
                Coord { x: 10.0, y: 60.0 },
                Coord { x: 10.0, y: 70.0 },
            ]),
            LineString::new(vec![
                Coord { x: 60.0, y: 100.0 },
                Coord { x: 40.0, y: 70.0 },
                Coord { x: 100.0, y: 10.0 },
            ]),
        ]);

        let hull = LineString::new(vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 100.0, y: 0.0 },
            Coord { x: 100.0, y: 100.0 },
            Coord { x: 0.0, y: 100.0 },
            Coord { x: 0.0, y: 0.0 },
        ]);

        let polygons = MultiPolygon::from_contours(contours, &hull, 0., 1., true);

        let expected = MultiPolygon::new(vec![
            Polygon::new(
                LineString::new(vec![
                    Coord { x: 0.0, y: 20.0 },
                    Coord { x: 10.0, y: 15.0 },
                    Coord { x: 20.0, y: 0.0 },
                    Coord { x: 30.0, y: 0.0 },
                    Coord { x: 30.0, y: 100.0 },
                    Coord { x: 0.0, y: 100.0 },
                    Coord { x: 0.0, y: 20.0 },
                ]),
                vec![LineString::new(vec![
                    Coord { x: 10.0, y: 70.0 },
                    Coord { x: 20.0, y: 70.0 },
                    Coord { x: 20.0, y: 60.0 },
                    Coord { x: 10.0, y: 60.0 },
                    Coord { x: 10.0, y: 70.0 },
                ])],
            ),
            Polygon::new(
                LineString::new(vec![
                    Coord { x: 60.0, y: 100.0 },
                    Coord { x: 40.0, y: 70.0 },
                    Coord { x: 100.0, y: 10.0 },
                    Coord { x: 100.0, y: 100.0 },
                    Coord { x: 60.0, y: 100.0 },
                ]),
                vec![],
            ),
        ]);

        assert_eq!(polygons, expected);
    }
}
