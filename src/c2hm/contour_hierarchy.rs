use crate::geometry::Line;

#[derive(Clone, Debug)]
pub struct ContourHierarchy {
    pub contours: Vec<Line>,
    closed_contours: Vec<Line>, // just the same as the original contours just all closed
    open_closed_map: Vec<usize>, // maps contours to closed contours
    pub hierarchy: Vec<ContourInfo>,
}

#[derive(Clone, Debug)]
struct ContourInfo {
    pub contour: usize,
    pub siblings: Vec<usize>,
    pub children: Vec<usize>,
    pub parent: usize,
    pub level: f64,
}

impl ContourHierarchy {
    pub fn new() -> ContourHierarchy {
        ContourHierarchy {
            contours: vec![],
            closed_contours: vec![],
            open_closed_map: vec![],
            hierarchy: vec![],
        }
    }

    pub fn from_lines(lines: Vec<Vec<Line>>, levels: Vec<f64>) -> Result<ContourHierarchy, ()> {
        if lines.len() != levels.len() {
            return Err(());
        }

        let mut ch = ContourHierarchy::new();

        for (contours, level) in lines.into_iter().zip(levels.into_iter()) {
            ch.add_level(contours, level);
        }
        Ok(ch)
    }

    pub fn add_level(&mut self, mut contours: Vec<Line>, level: f64) {
        if contours.is_empty() {
            return;
        }

        let mut unclosed_contours



        lines.sort_by(|a, b| {
            b.signed_area()
                .unwrap()
                .partial_cmp(&a.signed_area().unwrap())
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        for contour in lines.into_iter() {

        }
    }
}

pub fn from_contours(
    mut contours: Vec<Line>,
    convex_hull: &Line,
    polygon_type: PolygonTrigger,
    min_size: f64,
    epsilon: f64,
    hint: bool,
) -> Vec<Polygon> {
    let mut polygons = vec![];
    let mut unclosed_contours = vec![];

    if contours.is_empty() {
        // everywhere is either above or below the limit
        // needs to use the hint to classify everywhere correctly
        if polygon_type as i8 * (2 * hint as i8 - 1) > 0 {
            polygons.push(Polygon::new(convex_hull.clone()));
        }
        return polygons;
    }

    // reverse all contours if we are interested in the polygons that the areas below the contours build, instead of the areas above
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
    while !unclosed_contours.is_empty() {
        let mut best_neighbour = usize::MAX;
        let mut best_boundary_dist = f64::MAX;
        for (j, other) in unclosed_contours.iter().enumerate() {
            let dist = unclosed_contours[0]
                .last_vertex()
                .get_distance_along_line_square_sum(other.first_vertex(), convex_hull, epsilon)
                .unwrap();
            if dist < best_boundary_dist {
                best_neighbour = j;
                best_boundary_dist = dist;
            }
        }

        if best_neighbour == 0 {
            let mut contour = unclosed_contours.swap_remove(0);
            contour.close_by_line(convex_hull, epsilon).unwrap();
            contours.push(contour);
        } else {
            let other = unclosed_contours.swap_remove(best_neighbour);
            unclosed_contours[0]
                .append_by_line(other, convex_hull, epsilon)
                .unwrap();
        }
    }

    // add all closed contours of the right orientation to its own polygon
    i = 0;
    while i < contours.len() {
        let contour = &contours[i];
        let area = contour.signed_area().unwrap();
        if area > -min_size / 10. && area < min_size {
            contours.swap_remove(i);
        } else if area >= min_size {
            polygons.push(Polygon::new(contours.swap_remove(i)));
        } else {
            i += 1;
        }
    }

    // a background polygon must to be added if only holes exist
    if polygons.is_empty() {
        polygons.push(Polygon::new(convex_hull.clone()));
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
    polygons
}
