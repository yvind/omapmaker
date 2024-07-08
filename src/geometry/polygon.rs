#![allow(dead_code)]
use super::{Line, Point2D};

#[derive(Clone, Debug)]
pub struct Polygon {
    pub boundary: Line,
    pub holes: Vec<Line>,
}

impl Polygon {
    pub fn new(mut outer: Line) -> Polygon {
        if !outer.is_closed() {
            outer.close();
        }
        Polygon {
            boundary: outer,
            holes: vec![],
        }
    }

    pub fn parts(&self) -> usize {
        self.holes.len() + 1
    }

    pub fn has_holes(&self) -> bool {
        self.holes.len() > 0
    }

    pub fn add_hole(&mut self, mut hole: Line) {
        if !hole.is_closed() {
            hole.close();
        }
        self.holes.push(hole);
    }

    pub fn area(&self) -> f64 {
        let mut area: f64 = self.boundary.signed_area().unwrap();

        for hole in &self.holes {
            area += hole.signed_area().unwrap();
        }
        return area.abs();
    }

    pub fn contains(&self, point: &Point2D) -> Result<bool, &'static str> {
        let mut inside: bool = self.boundary.contains(point)?;
        if !inside {
            return Ok(false);
        }
        for hole in self.holes.iter() {
            inside &= !hole.contains(point)?;
            if !inside {
                return Ok(false);
            }
        }
        Ok(inside)
    }

    pub fn from_contours(
        mut contours: Vec<Line>,
        convex_hull: &Line,
        polygon_type: PolygonTrigger,
        min_size: f64,
        epsilon: f64,
    ) -> Vec<Polygon> {
        let mut polygons: Vec<Polygon> = Vec::new();
        let mut unclosed_contours: Vec<Line> = Vec::new();

        let mut unclosed_hull = convex_hull.clone();
        unclosed_hull.pop();

        if polygon_type == PolygonTrigger::Below {
            for c in contours.iter_mut() {
                c.vertices.reverse();
            }
        }

        // filter out all unclosed contours
        let mut i: usize = 0;
        while i < contours.len() {
            if !contours[i].is_closed() {
                unclosed_contours.push(contours.swap_remove(i));
            } else {
                i += 1;
            }
        }

        // for each unclosed contour wander ccw along the convex hull and merge with the first encountered unclosed contour
        while unclosed_contours.len() > 0 {
            let mut best_neighbour = usize::MAX;
            let mut best_boundary_dist = f64::MAX;
            for (j, other) in unclosed_contours.iter().enumerate() {
                let dist = unclosed_contours[0]
                    .last_vertex()
                    .get_distance_along_hull(other.first_vertex(), &unclosed_hull, epsilon)
                    .unwrap();
                if dist < best_boundary_dist {
                    best_neighbour = j;
                    best_boundary_dist = dist;
                }
            }

            if best_neighbour == 0 {
                let mut contour = unclosed_contours.swap_remove(0);
                contour.close_by_hull(&unclosed_hull, epsilon).unwrap();
                contours.push(contour);
            } else {
                let other = unclosed_contours.swap_remove(best_neighbour);
                unclosed_contours[0].append_by_hull(other, &unclosed_hull, epsilon);
            }
        }

        // add all closed contours of the right orientation to its own polygon
        i = 0;
        while i < contours.len() {
            let contour = &contours[i];
            let area: f64 = contour.signed_area().unwrap();
            if area > -min_size / 10. && area < min_size {
                contours.swap_remove(i);
            } else if area >= min_size {
                polygons.push(Polygon::new(contour.clone()));
                contours.swap_remove(i);
            } else {
                i += 1;
            }
        }

        // a background polygon must to be added if only holes exist
        if polygons.len() == 0 {
            let mut outer = unclosed_hull.clone();
            outer.close();
            polygons.push(Polygon::new(outer));
        }

        // add the holes to the polygons
        for contour in contours {
            for polygon in &mut polygons {
                if polygon.contains(&contour.vertices[1]).unwrap() {
                    polygon.add_hole(contour);
                    break;
                }
            }
        }
        return polygons;
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PolygonTrigger {
    Above = 1,
    Below = -1,
}
