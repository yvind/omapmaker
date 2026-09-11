use std::collections::HashMap;

use geo::{Contains, Distance, Euclidean, Length, Vector2DOps};
use proj_core::CrsDef;
use rstar::{AABB, RTree, RTreeObject};

use super::{LineSymbol, MapObject, PointSymbol, Symbol};
use crate::parameters::Scale;

#[cfg(test)]
use super::AreaSymbol;
#[cfg(test)]
use geo::{Area, BooleanOps};

// ISOM dimensions are paper dimensions at the selected map scale. A slope
// line is 0.47 mm long, and the 0.70 mm minimum depression width leaves
// 0.23 mm between its tip and another contour detail.
const SLOPE_LINE_CLEARANCE_PAPER_MM: f64 = 0.23;
const SLOPE_LINE_TARGET_INTERVAL_PAPER_MM: f64 = 3.;
const SLOPE_LINE_MINIMUM_SEPARATION_PAPER_MM: f64 = 1.5;
const SLOPE_LINE_CANDIDATE_SPACING_M: f64 = 1.;
const SLOPE_LINE_TANGENT_SPAN_PAPER_MM: f64 = 0.25;
const MAX_SLOPE_LINE_CANDIDATES: usize = 512;

#[derive(Clone, Copy)]
struct IndexedContourSegment {
    line: geo::Line,
    line_id: usize,
}

impl RTreeObject for IndexedContourSegment {
    type Envelope = AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners(
            [
                self.line.start.x.min(self.line.end.x),
                self.line.start.y.min(self.line.end.y),
            ],
            [
                self.line.start.x.max(self.line.end.x),
                self.line.start.y.max(self.line.end.y),
            ],
        )
    }
}

#[derive(Clone, Copy)]
struct SlopeLineCandidate {
    anchor: geo::Coord,
    inward: geo::Coord,
    distance: f64,
    reentrant_score: f64,
}

pub struct InternalMap {
    pub ref_point: geo::Coord,
    pub scale: Scale,
    pub crs: Option<CrsDef>,
    pub objects: HashMap<Symbol, Vec<MapObject>>,
}

impl InternalMap {
    pub fn new(ref_point: geo::Coord, scale: Scale, crs: Option<CrsDef>) -> Self {
        InternalMap {
            ref_point,
            scale,
            crs,
            objects: HashMap::new(),
        }
    }

    pub fn add_object(&mut self, map_object: MapObject) {
        let symbol = map_object.get_symbol();

        if let Some(vec) = self.objects.get_mut(&symbol) {
            vec.push(map_object);
        } else {
            self.objects.insert(symbol, vec![map_object]);
        }
    }

    pub fn reserve_capacity(&mut self, symbol: impl Into<Symbol>, additional: usize) {
        let symbol = symbol.into();
        if let Some(vec) = self.objects.get_mut(&symbol) {
            vec.reserve(additional);
        } else {
            self.objects.insert(symbol, Vec::with_capacity(additional));
        }
    }

    pub fn remove_empty_keys(&mut self) {
        self.objects.retain(|_, v| !v.is_empty());
    }

    pub fn mark_basemap_depressions(&mut self) {
        let basemap = self
            .objects
            .get_mut(&Symbol::Line(LineSymbol::BasemapContour));

        let Some(basemap) = basemap else {
            return;
        };

        let mut neg_basemap = Vec::new();

        let mut i = 0;
        while i < basemap.len() {
            if let MapObject::Line {
                object,
                symbol: _,
                tags: _,
            } = &basemap[i]
            {
                if object.is_closed() && line_string_signed_area(object) < 0. {
                    let mut neg = basemap.swap_remove(i);

                    let _ = neg.change_symbol(LineSymbol::NegBasemapContour);

                    neg_basemap.push(neg);
                } else {
                    i += 1;
                }
            }
        }

        if let Some(existing_neg) = self
            .objects
            .get_mut(&Symbol::Line(LineSymbol::NegBasemapContour))
        {
            existing_neg.extend(neg_basemap);
        } else {
            let _ = self
                .objects
                .insert(Symbol::Line(LineSymbol::NegBasemapContour), neg_basemap);
        }
    }

    /// Turn small contour loops to dotknolls and depressions and remove the smallest ones
    /// dot_knolls smaller than (min+max)/2 + min will never be drawn as elongated
    pub fn make_dotknolls_and_depressions(
        &mut self,
        min_area: f64,
        max_area: f64,
        elongated_aspect: f64,
    ) {
        let keys = [
            Symbol::Line(LineSymbol::Contour),
            Symbol::Line(LineSymbol::FormLine),
            Symbol::Line(LineSymbol::IndexContour),
        ];

        let min_elongated_area = (max_area + min_area) / 2. + min_area;

        for key in keys {
            let contours = self.objects.get_mut(&key);

            let Some(contours) = contours else {
                continue;
            };
            let mut small_loops = Vec::with_capacity(contours.len());

            let mut i = 0;
            while i < contours.len() {
                let contour_object = &contours[i];
                if let MapObject::Line {
                    object,
                    symbol: _,
                    tags: _,
                } = contour_object
                {
                    if object.is_closed() {
                        let area = line_string_signed_area(object);

                        if area.abs() <= max_area {
                            small_loops.push(contours.swap_remove(i));
                        } else {
                            i += 1;
                        }
                    } else {
                        i += 1;
                    }
                } else {
                    i += 1;
                }
            }

            for small_loop in small_loops {
                if let MapObject::Line {
                    object,
                    symbol: _,
                    tags: _,
                } = &small_loop
                {
                    let area = line_string_signed_area(object);

                    // ignore too small loops
                    if area.abs() < min_area {
                        continue;
                    }

                    let (aspect, mid_point, rotation) =
                        line_string_aspect_midpoint_rotation(object);

                    let map_object = if area < 0. {
                        MapObject::Point {
                            object: geo::Point(mid_point),
                            symbol: PointSymbol::UDepression,
                            rotation,
                            tags: HashMap::new(),
                        }
                    } else if aspect < elongated_aspect || area < min_elongated_area {
                        MapObject::Point {
                            object: geo::Point(mid_point),
                            symbol: PointSymbol::DotKnoll,
                            rotation,
                            tags: HashMap::new(),
                        }
                    } else {
                        MapObject::Point {
                            object: geo::Point(mid_point),
                            symbol: PointSymbol::ElongatedDotKnoll,
                            rotation,
                            tags: HashMap::new(),
                        }
                    };
                    self.add_object(map_object);
                }
            }
        }
    }

    /// Add downhill slope lines to retained contour depressions.
    ///
    /// Candidates are ranked by clockwise local turning, which is the
    /// re-entrant direction for a clockwise (negative-area) contour. A mark is
    /// only accepted when its full footprint stays inside the depression and
    /// has enough room from the host contour and all other contour lines. The
    /// target count grows with the depression perimeter measured on paper.
    pub fn add_depression_slope_lines(&mut self) {
        let contour_symbols = [
            LineSymbol::Contour,
            LineSymbol::IndexContour,
            LineSymbol::FormLine,
        ];
        let contour_lines = contour_symbols
            .into_iter()
            .flat_map(|symbol| {
                self.objects
                    .get(&Symbol::Line(symbol))
                    .into_iter()
                    .flatten()
                    .filter_map(move |object| match object {
                        MapObject::Line { object, .. } => Some((symbol, object)),
                        _ => None,
                    })
            })
            .collect::<Vec<_>>();

        let segments = contour_lines
            .iter()
            .enumerate()
            .flat_map(|(line_id, (_, line))| {
                line.lines().filter_map(move |line| {
                    (Euclidean.length(&line) > f64::EPSILON)
                        .then_some(IndexedContourSegment { line, line_id })
                })
            })
            .collect::<Vec<_>>();
        let segment_index = RTree::bulk_load(segments);

        let slope_lines = contour_lines
            .iter()
            .enumerate()
            .filter(|(_, (_, line))| line.is_closed() && line_string_signed_area(line) < 0.)
            .flat_map(|(line_id, (line_symbol, line))| {
                let symbol = match line_symbol {
                    LineSymbol::FormLine => PointSymbol::SlopeLineFormLine,
                    LineSymbol::Contour | LineSymbol::IndexContour => PointSymbol::SlopeLineContour,
                    _ => unreachable!("only contour symbols were collected"),
                };
                depression_slope_lines(line, line_id, self.scale, &segment_index)
                    .into_iter()
                    .map(move |(object, rotation)| MapObject::Point {
                        object,
                        symbol,
                        rotation,
                        tags: HashMap::new(),
                    })
            })
            .collect::<Vec<_>>();

        for slope_line in slope_lines {
            self.add_object(slope_line);
        }
    }
}

fn depression_slope_lines(
    line: &geo::LineString,
    line_id: usize,
    scale: Scale,
    segment_index: &RTree<IndexedContourSegment>,
) -> Vec<(geo::Point, f64)> {
    let slope_line_length = PointSymbol::SlopeLineContour
        .slope_line_length_m(scale)
        .expect("contour slope lines have a ground length");
    let slope_line_clearance = scale.paper_mm_to_meters(SLOPE_LINE_CLEARANCE_PAPER_MM);
    let total_length = Euclidean.length(line);
    if line.0.len() < 4 || !total_length.is_finite() || total_length <= f64::EPSILON {
        return Vec::new();
    }
    let target_count = ((scale.meters_to_paper_mm(total_length)
        / SLOPE_LINE_TARGET_INTERVAL_PAPER_MM)
        .floor() as usize)
        .clamp(1, MAX_SLOPE_LINE_CANDIDATES);
    let minimum_separation = scale.paper_mm_to_meters(SLOPE_LINE_MINIMUM_SEPARATION_PAPER_MM);

    let polygon = geo::Polygon::new(line.clone(), Vec::new());
    let candidate_count = ((total_length / SLOPE_LINE_CANDIDATE_SPACING_M).ceil() as usize)
        .clamp(8, MAX_SLOPE_LINE_CANDIDATES);
    let tangent_span = scale
        .paper_mm_to_meters(SLOPE_LINE_TANGENT_SPAN_PAPER_MM)
        .min(total_length / 8.);
    let mut candidates = (0..candidate_count)
        .filter_map(|index| {
            let distance = total_length * index as f64 / candidate_count as f64;
            slope_line_candidate(line, total_length, distance, tangent_span)
        })
        .filter(|candidate| candidate.reentrant_score > 0.)
        .collect::<Vec<_>>();
    candidates.sort_by(|first, second| {
        second
            .reentrant_score
            .total_cmp(&first.reentrant_score)
            .then_with(|| first.distance.total_cmp(&second.distance))
    });

    let mut selected = Vec::<(SlopeLineCandidate, geo::Line)>::with_capacity(target_count);
    for candidate in candidates {
        if selected.len() >= target_count {
            break;
        }
        if selected.iter().any(|(existing, _)| {
            cyclic_distance(existing.distance, candidate.distance, total_length)
                < minimum_separation
        }) {
            continue;
        }
        let tip = candidate.anchor + candidate.inward * slope_line_length;
        if ![0.25, 0.5, 0.75, 1.].into_iter().all(|fraction| {
            polygon.contains(&geo::Point::from(
                candidate.anchor + candidate.inward * (slope_line_length * fraction),
            ))
        }) {
            continue;
        }

        let mark = geo::Line::new(candidate.anchor, tip);
        if selected
            .iter()
            .any(|(_, existing)| Euclidean.distance(&mark, existing) < slope_line_clearance)
        {
            continue;
        }
        let search_envelope = expanded_line_envelope(mark, slope_line_clearance);
        let has_clearance = segment_index
            .locate_in_envelope_intersecting(search_envelope)
            .all(|segment| {
                if segment.line_id != line_id {
                    return Euclidean.distance(&mark, &segment.line) + f64::EPSILON
                        >= slope_line_clearance;
                }

                // Contact with the host contour is expected at the anchor.
                // Away from it, widen the required gap progressively until
                // the full minimum clearance is required at the tip.
                let does_not_recross_host = Euclidean.distance(&mark, &segment.line)
                    > crate::SIMPLIFICATION_DIST
                    || Euclidean.distance(&geo::Point::from(candidate.anchor), &segment.line)
                        <= crate::SIMPLIFICATION_DIST;
                does_not_recross_host
                    && [0.25, 0.5, 0.75, 1.].into_iter().all(|fraction| {
                        let point = geo::Point::from(
                            candidate.anchor + candidate.inward * (slope_line_length * fraction),
                        );
                        Euclidean.distance(&point, &segment.line) + f64::EPSILON
                            >= slope_line_clearance * fraction
                    })
            });
        if !has_clearance {
            continue;
        }

        selected.push((candidate, mark));
    }
    selected
        .into_iter()
        .map(|(candidate, _)| {
            // Point symbols are authored along +Y in omap's Cartesian
            // geometry. Rotate that axis onto the inward direction.
            let rotation =
                candidate.inward.y.atan2(candidate.inward.x) - std::f64::consts::FRAC_PI_2;
            (geo::Point::from(candidate.anchor), rotation)
        })
        .collect()
}

fn cyclic_distance(first: f64, second: f64, total: f64) -> f64 {
    let direct = (first - second).abs();
    direct.min(total - direct)
}

fn slope_line_candidate(
    line: &geo::LineString,
    total_length: f64,
    distance: f64,
    tangent_span: f64,
) -> Option<SlopeLineCandidate> {
    let anchor = closed_line_coordinate_at_distance(line, total_length, distance)?;
    let before = closed_line_coordinate_at_distance(
        line,
        total_length,
        (distance - tangent_span).rem_euclid(total_length),
    )?;
    let after = closed_line_coordinate_at_distance(
        line,
        total_length,
        (distance + tangent_span).rem_euclid(total_length),
    )?;
    let incoming = (anchor - before).try_normalize()?;
    let outgoing = (after - anchor).try_normalize()?;
    let tangent = (after - before).try_normalize()?;
    let reentrant_score = -incoming
        .wedge_product(outgoing)
        .atan2(incoming.dot_product(outgoing));
    let inward = geo::coord! { x: tangent.y, y: -tangent.x };
    Some(SlopeLineCandidate {
        anchor,
        inward,
        distance,
        reentrant_score,
    })
}

fn closed_line_coordinate_at_distance(
    line: &geo::LineString,
    total_length: f64,
    distance: f64,
) -> Option<geo::Coord> {
    let mut remaining = distance.rem_euclid(total_length);
    for segment in line.lines() {
        let segment_length = Euclidean.length(&segment);
        if segment_length <= f64::EPSILON {
            continue;
        }
        if remaining <= segment_length {
            return Some(
                segment.start + (segment.end - segment.start) * (remaining / segment_length),
            );
        }
        remaining -= segment_length;
    }
    line.0.first().copied()
}

fn expanded_line_envelope(line: geo::Line, amount: f64) -> AABB<[f64; 2]> {
    AABB::from_corners(
        [
            line.start.x.min(line.end.x) - amount,
            line.start.y.min(line.end.y) - amount,
        ],
        [
            line.start.x.max(line.end.x) + amount,
            line.start.y.max(line.end.y) + amount,
        ],
    )
}

fn line_string_signed_area(line: &geo::LineString) -> f64 {
    if line.0.len() < 3 {
        return 0.;
    }
    let mut area: f64 = 0.;
    for i in 0..line.0.len() - 1 {
        area += line.0[i].x * line.0[i + 1].y - line.0[i].y * line.0[i + 1].x;
    }
    0.5 * area
}

fn line_string_aspect_midpoint_rotation(line: &geo::LineString) -> (f64, geo::Coord, f64) {
    let mut midpoint = geo::Coord::zero();

    let len_f64 = line.0.len() as f64;
    for c in line.0.iter() {
        midpoint = midpoint + *c;
    }
    midpoint = midpoint / len_f64;

    // Calculate second moments
    let mu20 = line
        .0
        .iter()
        .map(|p| (p.x - midpoint.x).powi(2))
        .sum::<f64>()
        / len_f64;
    let mu02 = line
        .0
        .iter()
        .map(|p| (p.y - midpoint.y).powi(2))
        .sum::<f64>()
        / len_f64;
    let mu11 = line
        .0
        .iter()
        .map(|p| (p.x - midpoint.x) * (p.y - midpoint.y))
        .sum::<f64>()
        / len_f64;

    // Calculate elongation using eigenvalues of the covariance matrix
    let temp = ((mu20 - mu02).powi(2) + 4.0 * mu11.powi(2)).sqrt();
    let lambda1 = (mu20 + mu02 + temp) / 2.0;
    let lambda2 = (mu20 + mu02 - temp) / 2.0;

    // Handle potential numerical issues
    const EPS: f64 = 1000. * f64::EPSILON;
    if lambda2.abs() <= EPS {
        // colinear points
        if mu11.abs() <= EPS {
            // horizontal or vertical
            return (
                f64::INFINITY,
                midpoint,
                if mu20 > mu02 {
                    0.0
                } else {
                    std::f64::consts::FRAC_PI_2
                },
            );
        } else {
            // Diagonal line
            let angle = 0.5 * f64::atan2(2.0 * mu11, mu20 - mu02);
            return (f64::INFINITY, midpoint, normalize_angle(angle));
        }
    }

    let elongation = lambda1 / lambda2;

    // Calculate the angle of the major axis
    // The eigenvector for the larger eigenvalue gives the major axis direction
    let angle = if mu11.abs() <= EPS {
        // Principal axes are aligned with coordinate axes
        if mu20 >= mu02 {
            0.0
        } else {
            std::f64::consts::FRAC_PI_2
        }
    } else {
        // General case: use eigenvector of larger eigenvalue
        // For 2x2 symmetric matrix, eigenvector is [mu11, lambda1 - mu20]
        f64::atan2(lambda1 - mu20, mu11) + std::f64::consts::FRAC_PI_2
    };

    (elongation, midpoint, normalize_angle(angle))
}

fn normalize_angle(angle: f64) -> f64 {
    let mut normalized = angle % std::f64::consts::PI;
    if normalized < 0.0 {
        normalized += std::f64::consts::PI;
    }
    normalized
}

#[cfg(test)]
mod tests {
    use super::*;

    fn seam_stable_formline_at(points: Vec<geo::Coord>, elevation: f32) -> MapObject {
        let mut object = MapObject::Line {
            object: geo::LineString::new(points),
            symbol: LineSymbol::FormLine,
            tags: HashMap::new(),
        };
        object.add_elevation_tag(elevation);
        object.stabilize_contour_seam();
        object
    }

    fn seam_stable_formline(points: Vec<geo::Coord>) -> MapObject {
        seam_stable_formline_at(points, 2.5)
    }

    fn cliff_line(symbol: LineSymbol, start: f64, end: f64, y: f64) -> MapObject {
        MapObject::Line {
            object: geo::LineString::new(vec![
                geo::coord! { x: start, y: y },
                geo::coord! { x: end, y: y },
            ]),
            symbol,
            tags: HashMap::new(),
        }
    }

    fn clockwise_rectangle(width: f64, height: f64) -> geo::LineString {
        geo::LineString::new(vec![
            geo::coord! { x: 0., y: 0. },
            geo::coord! { x: 0., y: height },
            geo::coord! { x: width, y: height },
            geo::coord! { x: width, y: 0. },
            geo::coord! { x: 0., y: 0. },
        ])
    }

    fn contour_object(object: geo::LineString, symbol: LineSymbol) -> MapObject {
        MapObject::Line {
            object,
            symbol,
            tags: HashMap::new(),
        }
    }

    #[test]
    fn retained_depressions_get_the_matching_inward_slope_line() {
        for (line_symbol, point_symbol) in [
            (LineSymbol::Contour, PointSymbol::SlopeLineContour),
            (LineSymbol::IndexContour, PointSymbol::SlopeLineContour),
            (LineSymbol::FormLine, PointSymbol::SlopeLineFormLine),
        ] {
            let depression = clockwise_rectangle(30., 30.);
            let polygon = geo::Polygon::new(depression.clone(), Vec::new());
            let mut map = InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S15_000, None);
            map.add_object(contour_object(depression.clone(), line_symbol));

            map.add_depression_slope_lines();

            let slope_lines = &map.objects[&Symbol::Point(point_symbol)];
            assert_eq!(slope_lines.len(), 2);
            for slope_line in slope_lines {
                let MapObject::Point {
                    object, rotation, ..
                } = slope_line
                else {
                    panic!("expected a slope-line point object");
                };
                assert!(Euclidean.distance(object, &depression) < 1e-9);
                let inward = geo::coord! {
                    x: -rotation.sin(),
                    y: rotation.cos(),
                };
                assert!(polygon.contains(&geo::Point::from(object.0 + inward)));
            }
        }
    }

    #[test]
    fn tile_fragments_get_a_slope_line_only_after_directed_stitching() {
        let fragment = |coordinates| {
            let mut object = contour_object(geo::LineString::new(coordinates), LineSymbol::Contour);
            object.add_elevation_tag(5.);
            object.stabilize_contour_seam();
            object.mark_contour_tile_boundary_endpoints(true, true);
            object
        };
        let first = vec![
            geo::coord! { x: 0., y: 0. },
            geo::coord! { x: 0., y: 30. },
            geo::coord! { x: 30., y: 30.2 },
        ];
        let directed_second = vec![
            geo::coord! { x: 30., y: 30. },
            geo::coord! { x: 30., y: 0. },
            geo::coord! { x: 0., y: 0. },
        ];

        let mut stitched = InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S15_000, None);
        stitched.add_object(fragment(first.clone()));
        stitched.add_object(fragment(directed_second.clone()));
        stitched.add_depression_slope_lines();
        assert!(
            !stitched
                .objects
                .contains_key(&Symbol::Point(PointSymbol::SlopeLineContour))
        );

        stitched.merge_lines(5. * crate::SIMPLIFICATION_DIST);
        let [MapObject::Line { object, .. }] =
            stitched.objects[&Symbol::Line(LineSymbol::Contour)].as_slice()
        else {
            panic!("expected one stitched contour");
        };
        assert!(object.is_closed());
        assert!(line_string_signed_area(object) < 0.);
        stitched.add_depression_slope_lines();
        assert_eq!(
            stitched.objects[&Symbol::Point(PointSymbol::SlopeLineContour)].len(),
            2
        );

        let mut opposed = InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S15_000, None);
        opposed.add_object(fragment(first));
        opposed.add_object(fragment(directed_second.into_iter().rev().collect()));
        opposed.merge_lines(5. * crate::SIMPLIFICATION_DIST);
        opposed.add_depression_slope_lines();
        assert_eq!(opposed.objects[&Symbol::Line(LineSymbol::Contour)].len(), 2);
        assert!(
            !opposed
                .objects
                .contains_key(&Symbol::Point(PointSymbol::SlopeLineContour))
        );
    }

    #[test]
    fn open_and_positive_area_contours_do_not_get_slope_lines() {
        let mut map = InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S15_000, None);
        let mut knoll = clockwise_rectangle(30., 30.);
        knoll.0.reverse();
        map.add_object(contour_object(knoll, LineSymbol::Contour));
        map.add_object(contour_object(
            geo::LineString::new(vec![
                geo::coord! { x: 40., y: 0. },
                geo::coord! { x: 40., y: 30. },
                geo::coord! { x: 70., y: 30. },
            ]),
            LineSymbol::FormLine,
        ));

        map.add_depression_slope_lines();

        assert!(
            !map.objects
                .contains_key(&Symbol::Point(PointSymbol::SlopeLineContour))
        );
        assert!(
            !map.objects
                .contains_key(&Symbol::Point(PointSymbol::SlopeLineFormLine))
        );
    }

    #[test]
    fn slope_lines_are_not_forced_into_too_narrow_depressions() {
        let mut map = InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S15_000, None);
        map.add_object(contour_object(
            clockwise_rectangle(4., 100.),
            LineSymbol::Contour,
        ));

        map.add_depression_slope_lines();

        assert!(
            !map.objects
                .contains_key(&Symbol::Point(PointSymbol::SlopeLineContour))
        );
    }

    #[test]
    fn slope_line_footprint_and_clearance_follow_the_map_scale() {
        assert_eq!(
            PointSymbol::SlopeLineContour.slope_line_length_m(Scale::S15_000),
            Some(7.05)
        );
        assert_eq!(
            PointSymbol::SlopeLineContour.slope_line_length_m(Scale::S10_000),
            Some(4.7)
        );

        let accepts_eight_metre_width = |scale| {
            let mut map = InternalMap::new(geo::coord! { x: 0., y: 0. }, scale, None);
            map.add_object(contour_object(
                clockwise_rectangle(8., 8.),
                LineSymbol::Contour,
            ));
            map.add_depression_slope_lines();
            map.objects
                .contains_key(&Symbol::Point(PointSymbol::SlopeLineContour))
        };
        assert!(!accepts_eight_metre_width(Scale::S15_000));
        assert!(accepts_eight_metre_width(Scale::S10_000));
    }

    #[test]
    fn a_wide_reentrant_is_used_when_the_sharpest_one_is_too_narrow() {
        let depression = geo::LineString::new(vec![
            geo::coord! { x: 0., y: 0. },
            geo::coord! { x: 0., y: 30. },
            geo::coord! { x: 19., y: 30. },
            geo::coord! { x: 20., y: 45. },
            geo::coord! { x: 21., y: 30. },
            geo::coord! { x: 40., y: 30. },
            geo::coord! { x: 40., y: 0. },
            geo::coord! { x: 0., y: 0. },
        ]);
        assert!(line_string_signed_area(&depression) < 0.);
        let mut map = InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S15_000, None);
        map.add_object(contour_object(depression, LineSymbol::Contour));

        map.add_depression_slope_lines();

        let slope_lines = &map.objects[&Symbol::Point(PointSymbol::SlopeLineContour)];
        assert!(slope_lines.len() > 1);
        assert!(slope_lines.iter().all(|slope_line| {
            matches!(slope_line, MapObject::Point { object, .. } if object.y() < 35.)
        }));
    }

    #[test]
    fn marsh_subtraction_removes_all_open_water_overlap() {
        let mut map = InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S15_000, None);
        let marsh = geo::Rect::new(geo::coord! { x: 0., y: 0. }, geo::coord! { x: 10., y: 10. })
            .to_polygon();
        let water = geo::Rect::new(geo::coord! { x: 5., y: 0. }, geo::coord! { x: 15., y: 10. })
            .to_polygon();
        map.add_object(MapObject::Area {
            object: marsh,
            symbol: AreaSymbol::Marsh,
            tags: HashMap::new(),
        });
        map.add_object(MapObject::Area {
            object: water.clone(),
            symbol: AreaSymbol::UncrossableWaterWithBankLine,
            tags: HashMap::new(),
        });

        map.subtract_area_symbol(AreaSymbol::Marsh, AreaSymbol::UncrossableWaterWithBankLine)
            .unwrap();

        let [MapObject::Area { object, .. }] =
            map.objects[&Symbol::Area(AreaSymbol::Marsh)].as_slice()
        else {
            panic!("expected one clipped marsh polygon");
        };
        assert_eq!(object.intersection(&water).unsigned_area(), 0.);
        assert!((object.unsigned_area() - 50.).abs() < 1e-9);
    }

    #[test]
    fn seam_stable_formlines_merge_only_at_identical_endpoints() {
        let mut map = InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S15_000, None);
        map.add_object(seam_stable_formline(vec![
            geo::coord! { x: 0., y: 0. },
            geo::coord! { x: 1., y: 0. },
        ]));
        map.add_object(seam_stable_formline(vec![
            geo::coord! { x: 1., y: 0. },
            geo::coord! { x: 2., y: 0. },
        ]));
        map.add_object(seam_stable_formline(vec![
            geo::coord! { x: 2.001, y: 0. },
            geo::coord! { x: 3., y: 0. },
        ]));

        map.merge_lines(10.);

        let lines = &map.objects[&Symbol::Line(LineSymbol::FormLine)];
        assert_eq!(lines.len(), 2);
        assert!(lines.iter().any(|object| {
            matches!(
                object,
                MapObject::Line { object, .. }
                    if object.0 == vec![
                        geo::coord! { x: 0., y: 0. },
                        geo::coord! { x: 1., y: 0. },
                        geo::coord! { x: 2., y: 0. },
                    ]
            )
        }));
    }

    #[test]
    fn exact_contour_stitching_is_independent_of_tile_order() {
        let stitch = |reverse: bool| {
            let mut segments = vec![
                vec![geo::coord! { x: 0., y: 0. }, geo::coord! { x: 1., y: 0. }],
                vec![geo::coord! { x: 1., y: 0. }, geo::coord! { x: 2., y: 0. }],
                vec![geo::coord! { x: 2., y: 0. }, geo::coord! { x: 3., y: 0. }],
            ];
            if reverse {
                segments.reverse();
            }
            let mut map = InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S15_000, None);
            for segment in segments {
                map.add_object(seam_stable_formline(segment));
            }
            map.merge_lines(10.);
            let [MapObject::Line { object, .. }] =
                map.objects[&Symbol::Line(LineSymbol::FormLine)].as_slice()
            else {
                panic!("expected one stitched line");
            };
            object.clone()
        };

        assert_eq!(stitch(false), stitch(true));
    }

    #[test]
    fn contour_merge_requires_exact_elevation_and_matching_orientation() {
        let line = |points, elevation| seam_stable_formline_at(points, elevation);

        let mut different_elevations =
            InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S15_000, None);
        different_elevations.add_object(line(
            vec![geo::coord! { x: 0., y: 0. }, geo::coord! { x: 1., y: 0. }],
            2.501,
        ));
        different_elevations.add_object(line(
            vec![geo::coord! { x: 1., y: 0. }, geo::coord! { x: 2., y: 0. }],
            2.504,
        ));
        different_elevations.merge_lines(10.);
        assert_eq!(
            different_elevations.objects[&Symbol::Line(LineSymbol::FormLine)].len(),
            2
        );

        let mut opposite_orientation =
            InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S15_000, None);
        opposite_orientation.add_object(line(
            vec![geo::coord! { x: 0., y: 0. }, geo::coord! { x: 1., y: 0. }],
            2.5,
        ));
        opposite_orientation.add_object(line(
            vec![geo::coord! { x: 2., y: 0. }, geo::coord! { x: 1., y: 0. }],
            2.5,
        ));
        opposite_orientation.merge_lines(10.);
        assert_eq!(
            opposite_orientation.objects[&Symbol::Line(LineSymbol::FormLine)].len(),
            2
        );
    }

    #[test]
    fn symbol_specific_endpoint_distance_overrides_the_default() {
        let line = |symbol, start, end| MapObject::Line {
            object: geo::LineString::new(vec![
                geo::coord! { x: start, y: 0. },
                geo::coord! { x: end, y: 0. },
            ]),
            symbol,
            tags: HashMap::new(),
        };
        let mut map = InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S15_000, None);
        for symbol in [LineSymbol::Cliff, LineSymbol::SmallCrossableWatercourse] {
            map.add_object(line(symbol, 0., 1.));
            map.add_object(line(symbol, 1.25, 2.));
        }

        map.merge_lines_with_symbol_distance(0.5, LineSymbol::SmallCrossableWatercourse, 0.1);

        assert_eq!(map.objects[&Symbol::Line(LineSymbol::Cliff)].len(), 1);
        assert_eq!(
            map.objects[&Symbol::Line(LineSymbol::SmallCrossableWatercourse)].len(),
            2
        );
    }

    #[test]
    fn cliff_only_merge_uses_one_metre_and_preserves_direction() {
        let line = |symbol, y, start, end| MapObject::Line {
            object: geo::LineString::new(vec![
                geo::coord! { x: start, y: y },
                geo::coord! { x: end, y: y },
            ]),
            symbol,
            tags: HashMap::new(),
        };
        let mut map = InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S15_000, None);
        map.add_object(line(LineSymbol::Cliff, 0., 0., 2.));
        map.add_object(line(LineSymbol::Cliff, 0., 2.75, 5.));
        map.add_object(line(LineSymbol::Cliff, 2., 0., 2.));
        map.add_object(line(LineSymbol::Cliff, 2., 5., 2.75));
        map.add_object(line(LineSymbol::ImpassableCliff, 4., 0., 2.));
        map.add_object(line(LineSymbol::ImpassableCliff, 4., 2.75, 5.));
        map.add_object(line(LineSymbol::SmallCrossableWatercourse, 6., 0., 2.));
        map.add_object(line(LineSymbol::SmallCrossableWatercourse, 6., 2.75, 5.));

        map.merge_cliff_lines(1.);

        assert_eq!(map.objects[&Symbol::Line(LineSymbol::Cliff)].len(), 3);
        assert_eq!(
            map.objects[&Symbol::Line(LineSymbol::ImpassableCliff)].len(),
            1
        );
        assert_eq!(
            map.objects[&Symbol::Line(LineSymbol::SmallCrossableWatercourse)].len(),
            2
        );
        assert!(
            map.objects[&Symbol::Line(LineSymbol::Cliff)]
                .iter()
                .any(|object| matches!(
                    object,
                    MapObject::Line { object, .. }
                        if object.0 == vec![
                            geo::coord! { x: 0., y: 0. },
                            geo::coord! { x: 2.75, y: 0. },
                            geo::coord! { x: 5., y: 0. },
                        ]
                ))
        );
    }

    #[test]
    fn cliff_minimum_size_uses_connected_classes_but_never_sub_third_fragments() {
        let mut map = InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S15_000, None);

        // A mixed 4 m + 5 m chain becomes one ordinary 9 m cliff after the
        // short impassable part is demoted.
        map.add_object(cliff_line(LineSymbol::Cliff, 0., 4., 0.));
        map.add_object(cliff_line(LineSymbol::ImpassableCliff, 4., 9., 0.));
        // An isolated 8 m line is still too short, while 9 m survives alone.
        map.add_object(cliff_line(LineSymbol::Cliff, 0., 8., 10.));
        map.add_object(cliff_line(LineSymbol::Cliff, 0., 9., 20.));
        // A fragment below 3 m is removed even when attached to a long line.
        map.add_object(cliff_line(LineSymbol::Cliff, 0., 2.9, 30.));
        map.add_object(cliff_line(LineSymbol::ImpassableCliff, 2.9, 12.9, 30.));
        // Nearby but disconnected lines do not form a chain.
        map.add_object(cliff_line(LineSymbol::Cliff, 0., 4., 40.));
        map.add_object(cliff_line(LineSymbol::ImpassableCliff, 0., 5., 41.));

        map.filter_cliff_min_size(0.1);

        let lengths = |symbol| {
            let mut lengths = map.objects[&Symbol::Line(symbol)]
                .iter()
                .map(|object| match object {
                    MapObject::Line { object, .. } => Euclidean.length(object),
                    _ => 0.,
                })
                .collect::<Vec<_>>();
            lengths.sort_by(f64::total_cmp);
            lengths
        };
        assert_eq!(lengths(LineSymbol::Cliff), vec![9., 9.]);
        assert_eq!(lengths(LineSymbol::ImpassableCliff), vec![10.]);
    }

    #[test]
    fn short_impassable_cliffs_merge_or_use_bounded_exaggeration_before_filtering() {
        let mut map = InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S15_000, None);

        // Demotion joins this fragment to the neighboring ordinary cliff and
        // the combined 9.5 m line survives.
        map.add_object(cliff_line(LineSymbol::Cliff, 0., 5., 0.));
        map.add_object(cliff_line(LineSymbol::ImpassableCliff, 5.5, 9.5, 0.));
        // This analogous merged line remains below 9 m and is removed.
        map.add_object(cliff_line(LineSymbol::Cliff, 0., 3., 10.));
        map.add_object(cliff_line(LineSymbol::ImpassableCliff, 3.5, 7.5, 10.));
        // An isolated 8.5 m impassable line reaches 9.5 m after extending each
        // end by 0.5 m, whereas an isolated 7.5 m line still does not qualify.
        map.add_object(cliff_line(LineSymbol::ImpassableCliff, 0., 8.5, 20.));
        map.add_object(cliff_line(LineSymbol::ImpassableCliff, 0., 7.5, 30.));
        // A qualifying impassable cliff keeps its original symbol.
        map.add_object(cliff_line(LineSymbol::ImpassableCliff, 0., 9., 40.));
        // Short impassable fragments merge before demotion is considered, so
        // their qualifying combined line also remains impassable.
        map.add_object(cliff_line(LineSymbol::ImpassableCliff, 0., 5., 50.));
        map.add_object(cliff_line(LineSymbol::ImpassableCliff, 5., 10., 50.));

        map.filter_cliff_min_size(0.1);

        let mut small_lengths = map.objects[&Symbol::Line(LineSymbol::Cliff)]
            .iter()
            .map(|object| match object {
                MapObject::Line { object, .. } => Euclidean.length(object),
                _ => 0.,
            })
            .collect::<Vec<_>>();
        small_lengths.sort_by(f64::total_cmp);
        assert_eq!(small_lengths, vec![9.5, 9.5]);
        let mut large_lengths = map.objects[&Symbol::Line(LineSymbol::ImpassableCliff)]
            .iter()
            .map(|object| match object {
                MapObject::Line { object, .. } => Euclidean.length(object),
                _ => 0.,
            })
            .collect::<Vec<_>>();
        large_lengths.sort_by(f64::total_cmp);
        assert_eq!(large_lengths, vec![9., 10.]);
    }

    #[test]
    fn stream_does_not_merge_with_itself() {
        let almost_closed = |symbol| MapObject::Line {
            object: geo::LineString::new(vec![
                geo::coord! { x: 0., y: 0. },
                geo::coord! { x: 1., y: 0. },
                geo::coord! { x: 0.1, y: 0. },
            ]),
            symbol,
            tags: HashMap::new(),
        };
        let mut map = InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S15_000, None);
        map.add_object(almost_closed(LineSymbol::Cliff));
        map.add_object(almost_closed(LineSymbol::SmallCrossableWatercourse));

        map.merge_lines(0.2);

        let [MapObject::Line { object: cliff, .. }] =
            map.objects[&Symbol::Line(LineSymbol::Cliff)].as_slice()
        else {
            panic!("expected one cliff");
        };
        let [MapObject::Line { object: stream, .. }] =
            map.objects[&Symbol::Line(LineSymbol::SmallCrossableWatercourse)].as_slice()
        else {
            panic!("expected one stream");
        };
        assert!(cliff.is_closed());
        assert!(!stream.is_closed());
    }

    #[test]
    fn stream_skips_itself_to_merge_with_another_stream() {
        let mut map = InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S15_000, None);
        for coordinates in [
            vec![
                geo::coord! { x: 0., y: 0. },
                geo::coord! { x: 1., y: 0. },
                geo::coord! { x: 0.05, y: 0. },
            ],
            vec![geo::coord! { x: -1., y: 0. }, geo::coord! { x: 0.1, y: 0. }],
        ] {
            map.add_object(MapObject::Line {
                object: geo::LineString::new(coordinates),
                symbol: LineSymbol::SmallCrossableWatercourse,
                tags: HashMap::new(),
            });
        }

        map.merge_lines(0.2);

        assert_eq!(
            map.objects[&Symbol::Line(LineSymbol::SmallCrossableWatercourse)].len(),
            1
        );
    }
}
