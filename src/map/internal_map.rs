use std::collections::HashMap;

use proj_core::CrsDef;

use super::{LineSymbol, MapObject, PointSymbol, Symbol};
use crate::parameters::Scale;

#[cfg(test)]
use super::AreaSymbol;
#[cfg(test)]
use geo::{Area, BooleanOps, Euclidean, Length};

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
