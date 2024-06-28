#![allow(dead_code)]
use super::{Contour, Point2D};

#[derive(Clone, Debug)]
pub struct Polygon {
    pub boundary: Contour,
    pub holes: Vec<Contour>,
}

impl Polygon {
    pub fn new(mut outer: Contour) -> Polygon {
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

    pub fn add_hole(&mut self, mut hole: Contour) {
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
        for hole in &self.holes {
            inside &= !hole.contains(point)?;
            if !inside {
                return Ok(false);
            }
        }
        Ok(inside)
    }

    pub fn from_contours(
        mut contours: Vec<Contour>,
        mut convex_hull: &Contour,
        polygon_type: PolygonTrigger,
        min_size: f64,
    ) -> Vec<Polygon> {
        let mut polygons: Vec<Polygon> = Vec::new();
        let mut unclosed_contours: Vec<Contour> = Vec::new();
        let mut closed_contours: Vec<Contour> = Vec::new();

        // filter out all unclosed contours
        let mut i: usize = 0;
        while i < contours.len() {
            if !contours[i].is_closed() {
                unclosed_contours.push(contours.swap_remove(i));
            } else {
                i += 1;
            }
        }

        // for each unclosed contour wander counterclockwise along the convex hull and merge with the first encountered unclosed contour.
        while unclosed_contours.len() > 0 {
            let mut best_neighbour = usize::MAX;
            let mut best_boundary_dist = f64::MAX;
            for (j, other) in unclosed_contours.iter().enumerate() {
                let dist = unclosed_contours[0]
                    .last_vertex()
                    .get_boundary_dist(&other.first_vertex(), &convex_hull)
                    .unwrap();
                if dist < best_boundary_dist {
                    best_neighbour = j;
                    best_boundary_dist = dist;
                }
            }

            if best_neighbour == 0 {
                let mut contour = unclosed_contours.swap_remove(0);
                contour.close_by_boundary(&convex_hull);
                closed_contours.push(contour);
            } else {
                let mut other = unclosed_contours.swap_remove(best_neighbour);
                unclosed_contours[0].join_by_boundary(&mut other, &convex_hull);
            }
        }

        // If we want the areas below the contour-value to be the polygons
        // All edge polygons were closed in the wrong direction. Fix it by
        // Taking the inverse of the edge polygons by making a background polygon
        // and adding the edge polygons as holes
        // PRO:
        //   - don't have to implement all the boundary functions again in the clockwise order
        // CON:
        //   - all edge polygons become one object ( one click fix in Omapper )
        match polygon_type {
            PolygonTrigger::Below => {
                convex_hull.elevation = contours[0].elevation;
                polygons.push(Polygon::new(convex_hull.clone()));

                for contour in closed_contours {
                    polygons[0].add_hole(contour);
                }
            }
            PolygonTrigger::Above => {
                for contour in closed_contours {
                    polygons.push(Polygon::new(contour));
                }
            }
        }

        // add all closed contours of the right orientation to its own polygon
        i = 0;
        while i < contours.len() {
            let contour = &contours[i];
            let area: f64 = polygon_type as i32 as f64 * contour.signed_area().unwrap();
            if area > -min_size / 10. && area < min_size {
                contours.swap_remove(i);
            } else if area >= min_size {
                polygons.push(Polygon::new(contour.clone()));
                contours.swap_remove(i);
            } else {
                i += 1;
            }
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

#[derive(Clone, Copy, Debug)]
pub enum PolygonTrigger {
    Above = 1,
    Below = -1,
}
