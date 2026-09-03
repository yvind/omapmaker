use std::collections::HashMap;

use crate::{
    geometry::{MapMultiPolygon, centerline},
    map::{AreaSymbol, LineSymbol, MapObject},
    parameters::{BufferRule, MapParameters},
    raster::{Dfm, Elevation, RasterMarker},
};

use geo::{Area, BooleanOps, Simplify};

const MAX_CENTERLINE_SPACING_M: f64 = 2.;
const OUTSIDE_SAMPLE_MARGIN_CELLS: f64 = 1.;
const RAY_INTERSECTION_TOLERANCE: f64 = 1.0e-6;

pub fn compute_cliffs<T: RasterMarker>(
    cliff_strength: &Dfm<T>,
    dem: &Dfm<Elevation>,
    convex_hull: &geo::Polygon,
    cut_overlay: &geo::Polygon,
    params: &MapParameters,
    buffer_rules: &[BufferRule],
) -> Vec<MapObject> {
    let cliff_contours = cliff_strength.marching_squares(params.cliff.cliff);

    let mut cliff_polygons = geo::MultiPolygon::from_contours(cliff_contours, convex_hull, false);

    cliff_polygons = cliff_polygons.simplify(crate::SIMPLIFICATION_DIST);

    for buffer in buffer_rules.iter() {
        cliff_polygons = cliff_polygons.apply_buffer_rule(buffer);
    }
    cliff_polygons =
        remove_small_holes(cliff_polygons, params.geometry.cliffs.maximum_hole_area_m2);

    cliff_polygons = cut_overlay.intersection(&cliff_polygons);

    let (small_cliff_lines, large_cliff_lines, cliff_polygons) = if params.cliff.collapse {
        linearize_and_classify(cliff_polygons, dem, params)
    } else {
        (
            geo::MultiLineString::empty(),
            geo::MultiLineString::empty(),
            cliff_polygons,
        )
    };

    let num_polys = cliff_polygons.0.len();
    let num_lines = small_cliff_lines.0.len() + large_cliff_lines.0.len();

    let mut objects = Vec::with_capacity(num_polys + num_lines);

    for polygon in cliff_polygons.into_iter() {
        let cliff_object = MapObject::Area {
            object: polygon,
            symbol: AreaSymbol::GiganticBoulder,
            tags: HashMap::new(),
        };
        objects.push(cliff_object);
    }

    for line in small_cliff_lines.into_iter() {
        let cliff_object = MapObject::Line {
            object: line,
            symbol: LineSymbol::Cliff,
            tags: HashMap::new(),
        };
        objects.push(cliff_object);
    }

    for line in large_cliff_lines.into_iter() {
        let cliff_object = MapObject::Line {
            object: line,
            symbol: LineSymbol::ImpassableCliff,
            tags: HashMap::new(),
        };
        objects.push(cliff_object);
    }

    objects
}

/// Fill small enclosed gaps after buffering, before clipping or centerline
/// extraction can turn their rings into unwanted cliff branches.
fn remove_small_holes(polygons: geo::MultiPolygon, maximum_hole_area_m2: f64) -> geo::MultiPolygon {
    if !maximum_hole_area_m2.is_finite() || maximum_hole_area_m2 <= 0. {
        return polygons;
    }

    geo::MultiPolygon::new(
        polygons
            .into_iter()
            .map(|polygon| {
                let (exterior, interiors) = polygon.into_inner();
                let interiors = interiors
                    .into_iter()
                    .filter(|interior| {
                        geo::Polygon::new(interior.clone(), Vec::new()).unsigned_area()
                            > maximum_hole_area_m2
                    })
                    .collect();
                geo::Polygon::new(exterior, interiors)
            })
            .collect(),
    )
}

fn linearize_and_classify(
    polygons: geo::MultiPolygon,
    dem: &Dfm<Elevation>,
    params: &MapParameters,
) -> (
    geo::MultiLineString,
    geo::MultiLineString,
    geo::MultiPolygon,
) {
    let spacing = dem
        .grid
        .cell_size_m
        .clamp(crate::SIMPLIFICATION_DIST, MAX_CENTERLINE_SPACING_M);
    let linearity = f64::from(params.cliff.collapse_linearity);
    let minimum_height = finite_height_or_default(params.cliff.minimum_cliff_height_m, 1.).max(1.);
    let impassable_height =
        finite_height_or_default(params.cliff.impassable_cliff_height_m, minimum_height)
            .max(minimum_height);

    let mut small = Vec::new();
    let mut large = Vec::new();
    let mut retained = Vec::new();
    for polygon in polygons {
        let Some(centerlines) = centerline::linearize(&polygon, spacing, linearity) else {
            retained.push(polygon);
            continue;
        };

        let small_start = small.len();
        let large_start = large.len();
        for line in centerlines {
            let classified =
                classify_line_by_height(line, &polygon, dem, minimum_height, impassable_height);
            small.extend(classified.small);
            large.extend(classified.large);
        }

        // Missing DEM samples or a polygon whose entire height is below the
        // line threshold must not silently erase a detected object.
        if small.len() == small_start && large.len() == large_start {
            retained.push(polygon);
        }
    }

    let small = apply_rdp(
        geo::MultiLineString::new(small),
        &params.geometry.cliffs.rdp,
    );
    let large = apply_rdp(
        geo::MultiLineString::new(large),
        &params.geometry.cliffs.rdp,
    );
    (
        small,
        large,
        geo::MultiPolygon::new(retained).simplify(crate::SIMPLIFICATION_DIST),
    )
}

/// RDP is part of cliff-line construction, so subsequent merging and minimum
/// length checks see the simplified geometry. Optional Bézier fitting happens
/// later during preview/export conversion and is therefore always second.
fn apply_rdp(
    lines: geo::MultiLineString,
    parameters: &crate::parameters::RdpParameters,
) -> geo::MultiLineString {
    if parameters.enabled && parameters.tolerance_m.is_finite() && parameters.tolerance_m > 0. {
        lines.simplify(parameters.tolerance_m)
    } else {
        lines
    }
}

fn finite_height_or_default(value: f32, default: f64) -> f64 {
    if value.is_finite() {
        f64::from(value)
    } else {
        default
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum HeightClass {
    BelowMinimum,
    Small,
    Impassable,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct DirectedClass {
    height: HeightClass,
    forward_is_downhill_right: bool,
}

#[derive(Default)]
struct ClassifiedLines {
    small: Vec<geo::LineString>,
    large: Vec<geo::LineString>,
}

/// Split one centerline wherever either the height class or downhill-right
/// direction changes. Adjacent classes share the exact split coordinate, so a
/// single source polygon can change symbols without introducing gaps.
fn classify_line_by_height(
    mut line: geo::LineString,
    polygon: &geo::Polygon,
    dem: &Dfm<Elevation>,
    minimum_height: f64,
    impassable_height: f64,
) -> ClassifiedLines {
    if line.0.len() < 2 {
        return ClassifiedLines::default();
    }

    let mut changes = line
        .lines()
        .map(|segment| elevation_change_across_polygon(segment, polygon, dem))
        .collect::<Vec<_>>();
    resolve_unknown_changes(&mut changes);
    if changes.iter().all(Option::is_none) {
        return ClassifiedLines::default();
    }
    let mut classes = changes
        .into_iter()
        .map(|change| {
            let change = change.expect("at least one cross-cliff sample was resolved");
            let magnitude = change.abs();
            DirectedClass {
                height: if magnitude < minimum_height {
                    HeightClass::BelowMinimum
                } else if magnitude < impassable_height {
                    HeightClass::Small
                } else {
                    HeightClass::Impassable
                },
                forward_is_downhill_right: change >= 0.,
            }
        })
        .collect::<Vec<_>>();

    // Start a closed path at a direction transition. This prevents one run
    // from being needlessly split between the beginning and end of the array.
    if line.is_closed()
        && let Some(start) = (0..classes.len())
            .find(|&index| classes[index] != classes[(index + classes.len() - 1) % classes.len()])
    {
        let segment_count = classes.len();
        let mut coordinates = (0..segment_count)
            .map(|offset| line.0[(start + offset) % segment_count])
            .collect::<Vec<_>>();
        coordinates.push(coordinates[0]);
        line = geo::LineString::new(coordinates);
        classes.rotate_left(start);
    }

    let mut classified = ClassifiedLines::default();
    let mut run_start = 0;
    for run_end in 1..=classes.len() {
        if run_end < classes.len() && classes[run_end] == classes[run_start] {
            continue;
        }

        let class = classes[run_start];
        if class.height != HeightClass::BelowMinimum {
            let mut coordinates = line.0[run_start..=run_end].to_vec();
            if !class.forward_is_downhill_right {
                coordinates.reverse();
            }
            let output = match class.height {
                HeightClass::BelowMinimum => unreachable!(),
                HeightClass::Small => &mut classified.small,
                HeightClass::Impassable => &mut classified.large,
            };
            output.push(geo::LineString::new(coordinates));
        }
        run_start = run_end;
    }
    classified
}

/// Fill an unsampled segment from its nearest resolved neighbor. This avoids
/// losing a short piece at the DEM edge while never inventing evidence when an
/// entire line lies outside the sampled raster.
fn resolve_unknown_changes(changes: &mut [Option<f64>]) {
    let mut previous = None;
    for change in changes.iter_mut() {
        if change.is_some() {
            previous = *change;
        } else if previous.is_some() {
            *change = previous;
        }
    }

    let mut next = None;
    for change in changes.iter_mut().rev() {
        if change.is_some() {
            next = *change;
        } else if next.is_some() {
            *change = next;
        }
    }
}

/// Sample the terrain just beyond both sides of the detected cliff polygon.
/// Positive means the left side is higher, so the input segment already has
/// its lower side on the right.
fn elevation_change_across_polygon(
    segment: geo::Line,
    polygon: &geo::Polygon,
    dem: &Dfm<Elevation>,
) -> Option<f64> {
    let dx = segment.end.x - segment.start.x;
    let dy = segment.end.y - segment.start.y;
    let length = dx.hypot(dy);
    if length <= f64::EPSILON {
        return None;
    }
    let midpoint = geo::coord! {
        x: (segment.start.x + segment.end.x) / 2.,
        y: (segment.start.y + segment.end.y) / 2.,
    };
    let left_direction = geo::coord! {
        x: -dy / length,
        y: dx / length,
    };
    let right_direction = geo::coord! {
        x: -left_direction.x,
        y: -left_direction.y,
    };
    let left_boundary = boundary_distance_along_ray(polygon, midpoint, left_direction)?;
    let right_boundary = boundary_distance_along_ray(polygon, midpoint, right_direction)?;
    let margin = OUTSIDE_SAMPLE_MARGIN_CELLS * dem.grid.cell_size_m;
    let left = outside_sample_coordinate(midpoint, left_direction, left_boundary, margin, polygon)?;
    let right =
        outside_sample_coordinate(midpoint, right_direction, right_boundary, margin, polygon)?;
    let (Some(right_elevation), Some(left_elevation)) =
        (dem.sample_bilinear(right), dem.sample_bilinear(left))
    else {
        return None;
    };
    (right_elevation.is_finite()
        && left_elevation.is_finite()
        && right_elevation != f32::MIN
        && left_elevation != f32::MIN)
        .then(|| f64::from(left_elevation - right_elevation))
}

fn outside_sample_coordinate(
    origin: geo::Coord,
    direction: geo::Coord,
    boundary_distance: f64,
    mut margin: f64,
    polygon: &geo::Polygon,
) -> Option<geo::Coord> {
    use geo::Intersects;

    for _ in 0..8 {
        let coordinate = geo::coord! {
            x: origin.x + (boundary_distance + margin) * direction.x,
            y: origin.y + (boundary_distance + margin) * direction.y,
        };
        if !polygon.intersects(&geo::Point::from(coordinate)) {
            return Some(coordinate);
        }
        margin *= 0.5;
    }
    None
}

fn boundary_distance_along_ray(
    polygon: &geo::Polygon,
    origin: geo::Coord,
    direction: geo::Coord,
) -> Option<f64> {
    std::iter::once(polygon.exterior())
        .chain(polygon.interiors())
        .flat_map(geo::LineString::lines)
        .filter_map(|boundary| ray_segment_distance(origin, direction, boundary))
        .filter(|distance| *distance > RAY_INTERSECTION_TOLERANCE)
        .min_by(f64::total_cmp)
}

fn ray_segment_distance(
    origin: geo::Coord,
    direction: geo::Coord,
    segment: geo::Line,
) -> Option<f64> {
    let boundary_direction = segment.delta();
    let denominator = cross(direction, boundary_direction);
    if denominator.abs() <= f64::EPSILON {
        return None;
    }

    let to_boundary = segment.start - origin;
    let ray_distance = cross(to_boundary, boundary_direction) / denominator;
    let boundary_fraction = cross(to_boundary, direction) / denominator;
    (ray_distance >= -RAY_INTERSECTION_TOLERANCE
        && boundary_fraction >= -RAY_INTERSECTION_TOLERANCE
        && boundary_fraction <= 1. + RAY_INTERSECTION_TOLERANCE)
        .then(|| ray_distance.max(0.))
}

fn cross(first: geo::Coord, second: geo::Coord) -> f64 {
    first.x * second.y - first.y * second.x
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::raster::DfmGrid;
    use geo::polygon;

    fn horizontal_cliff_polygon() -> geo::Polygon {
        polygon![
            (x: 2.0, y: 4.0),
            (x: 18.0, y: 4.0),
            (x: 18.0, y: 6.0),
            (x: 2.0, y: 6.0),
        ]
    }

    #[test]
    fn extracted_cliff_lines_are_oriented_with_low_ground_on_the_right() {
        let grid = DfmGrid::new(21, 11, 1., geo::coord! { x: 0., y: 10. }).unwrap();
        let mut dem = Dfm::<Elevation>::new(grid);
        for y in 0..dem.height() {
            for x in 0..dem.width() {
                dem[(y, x)] = dem.index2coord(y, x).y as f32;
            }
        }
        let line = geo::LineString::new(vec![
            geo::coord! { x: 18., y: 5. },
            geo::coord! { x: 2., y: 5. },
        ]);

        let lines = classify_line_by_height(line, &horizontal_cliff_polygon(), &dem, 1., 10.);

        assert_eq!(lines.small.len(), 1);
        assert!(
            lines.small[0].0.first().unwrap().x < lines.small[0].0.last().unwrap().x,
            "line={:?}",
            lines.small[0]
        );
    }

    #[test]
    fn one_polygon_line_changes_from_below_minimum_to_small_and_impassable() {
        let grid = DfmGrid::new(21, 11, 1., geo::coord! { x: 0., y: 10. }).unwrap();
        let mut dem = Dfm::<Elevation>::new(grid);
        for y in 0..dem.height() {
            for x in 0..dem.width() {
                let coordinate = dem.index2coord(y, x);
                dem[(y, x)] = if coordinate.y > 6. {
                    if coordinate.x < 6. {
                        0.5
                    } else if coordinate.x < 12. {
                        1.5
                    } else {
                        3.
                    }
                } else {
                    0.
                };
            }
        }
        let line = geo::LineString::new(vec![
            geo::coord! { x: 2., y: 5. },
            geo::coord! { x: 6., y: 5. },
            geo::coord! { x: 12., y: 5. },
            geo::coord! { x: 18., y: 5. },
        ]);

        let lines = classify_line_by_height(line, &horizontal_cliff_polygon(), &dem, 1., 2.);

        assert_eq!(lines.small.len(), 1, "small={:?}", lines.small);
        assert_eq!(lines.large.len(), 1, "large={:?}", lines.large);
        assert_eq!(lines.small[0].0.first().unwrap().x, 6.);
        assert_eq!(lines.small[0].0.last().unwrap().x, 12.);
        assert_eq!(lines.large[0].0.first().unwrap().x, 12.);
        assert_eq!(lines.large[0].0.last().unwrap().x, 18.);
        assert!(
            lines
                .small
                .iter()
                .chain(&lines.large)
                .flat_map(|line| line.lines())
                .all(|segment| elevation_change_across_polygon(
                    segment,
                    &horizontal_cliff_polygon(),
                    &dem
                )
                .is_some_and(|change| change > 0.))
        );
    }

    #[test]
    fn a_polygon_without_a_qualifying_line_remains_a_gigantic_boulder() {
        let grid = DfmGrid::new(21, 11, 1., geo::coord! { x: 0., y: 10. }).unwrap();
        let mut dem = Dfm::<Elevation>::new(grid);
        dem.field.fill(0.);
        let source = horizontal_cliff_polygon();

        let (small, large, retained) = linearize_and_classify(
            geo::MultiPolygon::new(vec![source.clone()]),
            &dem,
            &MapParameters::default(),
        );

        assert!(small.0.is_empty());
        assert!(large.0.is_empty());
        assert_eq!(retained.0, vec![source]);
    }

    #[test]
    fn extracted_centerline_uses_multiple_height_classes_from_one_polygon() {
        let grid = DfmGrid::new(41, 15, 1., geo::coord! { x: 0., y: 14. }).unwrap();
        let mut dem = Dfm::<Elevation>::new(grid);
        for y in 0..dem.height() {
            for x in 0..dem.width() {
                let coordinate = dem.index2coord(y, x);
                dem[(y, x)] = if coordinate.y > 7. {
                    if coordinate.x < 20. { 1.5 } else { 3. }
                } else {
                    0.
                };
            }
        }
        let source = polygon![
            (x: 2.0, y: 5.0),
            (x: 38.0, y: 5.0),
            (x: 38.0, y: 7.0),
            (x: 2.0, y: 7.0),
        ];

        let (small, large, retained) = linearize_and_classify(
            geo::MultiPolygon::new(vec![source]),
            &dem,
            &MapParameters::default(),
        );

        assert!(retained.0.is_empty(), "retained={retained:?}");
        assert!(!small.0.is_empty(), "small={small:?}");
        assert!(!large.0.is_empty(), "large={large:?}");
        assert!(small.0.iter().any(|small_line| {
            let small_ends = [small_line.0.first().unwrap(), small_line.0.last().unwrap()];
            large.0.iter().any(|large_line| {
                let large_ends = [large_line.0.first().unwrap(), large_line.0.last().unwrap()];
                small_ends.iter().any(|small_end| {
                    large_ends.iter().any(|large_end| {
                        (small_end.x - large_end.x).hypot(small_end.y - large_end.y) <= 1.0e-6
                    })
                })
            })
        }));
    }

    #[test]
    fn cliff_rdp_can_be_enabled_without_bezier_and_disabled_independently() {
        let defaults = MapParameters::default();
        assert!(defaults.geometry.cliffs.rdp.enabled);
        assert!(!defaults.geometry.cliffs.bezier.enabled);

        let source = geo::MultiLineString::new(vec![geo::LineString::new(vec![
            geo::coord! { x: 0., y: 0. },
            geo::coord! { x: 1., y: 0.1 },
            geo::coord! { x: 2., y: -0.1 },
            geo::coord! { x: 3., y: 0. },
        ])]);
        let disabled = crate::parameters::RdpParameters {
            enabled: false,
            tolerance_m: 0.5,
        };
        let enabled = crate::parameters::RdpParameters {
            enabled: true,
            tolerance_m: 0.5,
        };
        let tighter = crate::parameters::RdpParameters {
            enabled: true,
            tolerance_m: 0.05,
        };

        assert_eq!(apply_rdp(source.clone(), &disabled), source);
        assert_eq!(apply_rdp(source.clone(), &tighter).0[0].0.len(), 4);
        let simplified = apply_rdp(source, &enabled);
        assert_eq!(simplified.0[0].0.len(), 2);
    }

    #[test]
    fn cliff_hole_cleanup_removes_only_holes_within_the_configured_area() {
        let exterior = polygon![
            (x: 0., y: 0.),
            (x: 20., y: 0.),
            (x: 20., y: 20.),
            (x: 0., y: 20.),
        ];
        let small_hole = polygon![
            (x: 2., y: 2.),
            (x: 4., y: 2.),
            (x: 4., y: 4.),
            (x: 2., y: 4.),
        ];
        let large_hole = polygon![
            (x: 10., y: 10.),
            (x: 15., y: 10.),
            (x: 15., y: 15.),
            (x: 10., y: 15.),
        ];
        let source = geo::MultiPolygon::new(vec![geo::Polygon::new(
            exterior.exterior().clone(),
            vec![small_hole.exterior().clone(), large_hole.exterior().clone()],
        )]);

        assert_eq!(remove_small_holes(source.clone(), 0.), source);
        let cleaned = remove_small_holes(source, 4.);
        assert_eq!(cleaned.0[0].interiors(), &[large_hole.exterior().clone()]);
    }
}
