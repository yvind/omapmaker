use geo::{BoundingRect, Distance, Euclidean};
use rstar::{AABB, Envelope, PointDistance, RTree, RTreeObject};

use super::{AreaSymbol, InternalMap, LineSymbol, MapObject, PointSymbol, Symbol};

struct Conflict<'a>(&'a MapObject);

struct BlockFootprint {
    bounds: AABB<[f64; 2]>,
    replaces_inferred_building: bool,
}

impl BlockFootprint {
    fn contains(&self, object: &MapObject) -> bool {
        replaceable_bounds(object, self.replaces_inferred_building)
            .is_some_and(|bounds| self.bounds.contains_envelope(&bounds_envelope(bounds)))
    }
}

impl RTreeObject for BlockFootprint {
    type Envelope = AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        self.bounds
    }
}

struct PlacedBoulder {
    position: [f64; 2],
    radius_m: f64,
}

impl RTreeObject for PlacedBoulder {
    type Envelope = AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_point(self.position)
    }
}

impl PointDistance for PlacedBoulder {
    fn distance_2(&self, point: &[f64; 2]) -> f64 {
        (self.position[0] - point[0]).powi(2) + (self.position[1] - point[1]).powi(2)
    }
}

impl RTreeObject for Conflict<'_> {
    type Envelope = AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        bounds_envelope(replaceable_bounds(self.0, true).expect("conflict has bounds"))
    }
}

impl InternalMap {
    /// Run after stitching tiles and converting contour loops to knolls.
    /// Priority and separation are deterministic across tile/thread order.
    pub(crate) fn filter_boulder_points(&mut self, minimum_separation_m: f64) {
        let symbols = [PointSymbol::SmallBoulder, PointSymbol::LargeBoulder];
        let mut candidates = Vec::new();
        for symbol in symbols {
            if let Some(objects) = self.objects.get_mut(&Symbol::Point(symbol)) {
                candidates.append(objects);
            }
        }
        if candidates.is_empty() {
            return;
        }
        let conflicts = RTree::bulk_load(
            self.objects
                .values()
                .flatten()
                .filter_map(|object| replaceable_bounds(object, true).map(|_| Conflict(object)))
                .collect(),
        );
        candidates.sort_unstable_by(|a, b| {
            is_large(b)
                .cmp(&is_large(a))
                .then(tag(b, "boulder_confidence").total_cmp(&tag(a, "boulder_confidence")))
                .then(tag(b, "boulder_height_m").total_cmp(&tag(a, "boulder_height_m")))
                .then_with(|| {
                    let (a, b) = (position(a), position(b));
                    a.x().total_cmp(&b.x()).then(a.y().total_cmp(&b.y()))
                })
        });
        let minimum_separation = minimum_separation_m.max(0.);
        let search_separation = minimum_separation.max(self.scale.paper_mm_to_meters(0.6));
        let clearance = self.scale.paper_mm_to_meters(0.15);
        let mut retained = Vec::new();
        let mut retained_blocks = Vec::new();
        let mut occupied = RTree::<PlacedBoulder>::new();
        for candidate in candidates {
            let point = position(&candidate);
            let block = block_footprint(&candidate);
            let paper_radius = if is_large(&candidate) { 0.3 } else { 0.2 };
            let symbol_radius = self.scale.paper_mm_to_meters(paper_radius);
            let radius =
                (tag(&candidate, "boulder_diameter_m") * 0.5).max(symbol_radius + clearance);
            let search_radius = radius + clearance;
            let envelope = AABB::from_corners(
                [point.x() - search_radius, point.y() - search_radius],
                [point.x() + search_radius, point.y() + search_radius],
            );
            let conflict = conflicts
                .locate_in_envelope_intersecting(envelope)
                .filter(|conflict| {
                    !block
                        .as_ref()
                        .is_some_and(|block| block.contains(conflict.0))
                })
                .any(|conflict| match conflict.0 {
                    MapObject::Area { object, .. } => Euclidean.distance(&point, object) <= radius,
                    MapObject::Line { object, .. } => Euclidean.distance(&point, object) <= radius,
                    MapObject::Point { object, .. } => {
                        Euclidean.distance(point, *object) <= radius + clearance
                    }
                });
            let position = [point.x(), point.y()];
            if conflict
                || occupied
                    .locate_within_distance(position, search_separation.powi(2))
                    .any(|placed| {
                        placed.distance_2(&position)
                            < minimum_separation
                                .max(symbol_radius + placed.radius_m)
                                .powi(2)
                    })
            {
                continue;
            }
            occupied.insert(PlacedBoulder {
                position,
                radius_m: symbol_radius,
            });
            if let Some(block) = block {
                retained_blocks.push(block);
            }
            retained.push(candidate);
        }
        drop(conflicts);
        // Replace geometry contained within a confirmed rock footprint.
        // A smooth dome can fool the piecewise roof fit; known class-6 returns
        // already exclude these candidates in the raw detector.
        let blocks = RTree::bulk_load(retained_blocks);
        for objects in self.objects.values_mut() {
            objects.retain(|object| {
                let Some(bounds) = replaceable_bounds(object, true) else {
                    return true;
                };
                let envelope = bounds_envelope(bounds);
                !blocks
                    .locate_in_envelope_intersecting(envelope)
                    .any(|block| {
                        (!is_building(object) || block.replaces_inferred_building)
                            && block.bounds.contains_envelope(&envelope)
                    })
            });
        }
        log::debug!("Boulder cleanup retained {} point symbols", retained.len());
        for candidate in retained {
            self.add_object(candidate);
        }
    }
}

fn block_footprint(candidate: &MapObject) -> Option<BlockFootprint> {
    if tag(candidate, "boulder_grounded_side_fraction") < 0.75 {
        return None;
    }
    let MapObject::Point { tags, .. } = candidate else {
        return None;
    };
    let coordinate = |key: &str| {
        tags.get(key)?
            .parse::<f64>()
            .ok()
            .filter(|value| value.is_finite())
    };
    let min = [
        coordinate("boulder_footprint_min_x")?,
        coordinate("boulder_footprint_min_y")?,
    ];
    let max = [
        coordinate("boulder_footprint_max_x")?,
        coordinate("boulder_footprint_max_y")?,
    ];
    if min[0] >= max[0] || min[1] >= max[1] {
        return None;
    }
    // Sobel derivatives and marching squares extend an edge beyond its source
    // cell. This tolerance follows the raster resolution, not printed size.
    let margin = 1.5 * crate::STANDARD_CELL_SIZE_METERS;
    Some(BlockFootprint {
        bounds: AABB::from_corners(
            [min[0] - margin, min[1] - margin],
            [max[0] + margin, max[1] + margin],
        ),
        replaces_inferred_building: tag(candidate, "boulder_dome_fraction") >= 0.5
            && tag(candidate, "boulder_under_surface_fraction") <= 0.05
            && tag(candidate, "boulder_nonterminal_fraction") <= 0.05,
    })
}

fn is_building(object: &MapObject) -> bool {
    matches!(
        object,
        MapObject::Area {
            symbol: AreaSymbol::Building,
            ..
        }
    )
}

fn replaceable_bounds(object: &MapObject, include_building: bool) -> Option<geo::Rect> {
    match object {
        MapObject::Area {
            object,
            symbol: AreaSymbol::Building,
            ..
        } if include_building => object.bounding_rect(),
        MapObject::Area {
            object,
            symbol: AreaSymbol::GiganticBoulder,
            ..
        } => object.bounding_rect(),
        MapObject::Line {
            object,
            symbol: LineSymbol::Cliff | LineSymbol::ImpassableCliff,
            ..
        } => object.bounding_rect(),
        MapObject::Point {
            object,
            symbol: PointSymbol::DotKnoll | PointSymbol::ElongatedDotKnoll,
            ..
        } => Some(object.bounding_rect()),
        _ => None,
    }
}

fn bounds_envelope(bounds: geo::Rect) -> AABB<[f64; 2]> {
    AABB::from_corners(
        [bounds.min().x, bounds.min().y],
        [bounds.max().x, bounds.max().y],
    )
}

fn position(object: &MapObject) -> geo::Point {
    let MapObject::Point { object, .. } = object else {
        unreachable!("boulder symbols are points")
    };
    *object
}

fn is_large(object: &MapObject) -> bool {
    matches!(
        object,
        MapObject::Point {
            symbol: PointSymbol::LargeBoulder,
            ..
        }
    )
}

fn tag(object: &MapObject, key: &str) -> f64 {
    let MapObject::Point { tags, .. } = object else {
        return 0.;
    };
    tags.get(key)
        .and_then(|value| value.parse::<f64>().ok())
        .filter(|value| value.is_finite())
        .unwrap_or(0.)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parameters::Scale;

    fn boulder(x: f64, y: f64, height: f64) -> MapObject {
        MapObject::Point {
            object: geo::Point::new(x, y),
            symbol: if height >= 2. {
                PointSymbol::LargeBoulder
            } else {
                PointSymbol::SmallBoulder
            },
            rotation: 0.,
            tags: [
                ("boulder_height_m".to_string(), height.to_string()),
                ("boulder_confidence".to_string(), "0.9".to_string()),
            ]
            .into(),
        }
    }

    fn positions(map: &InternalMap) -> Vec<(f64, f64)> {
        let mut points = map
            .objects
            .iter()
            .filter(|(symbol, _)| {
                matches!(
                    symbol,
                    Symbol::Point(PointSymbol::SmallBoulder | PointSymbol::LargeBoulder)
                )
            })
            .flat_map(|(_, objects)| {
                objects.iter().map(|object| {
                    let p = position(object);
                    (p.x(), p.y())
                })
            })
            .collect::<Vec<_>>();
        points.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)));
        points
    }

    fn confirmed_block() -> MapObject {
        let mut object = boulder(0., 0., 3.);
        let MapObject::Point { tags, .. } = &mut object else {
            unreachable!()
        };
        for (key, value) in [
            ("boulder_grounded_side_fraction", "1"),
            ("boulder_footprint_min_x", "-2"),
            ("boulder_footprint_min_y", "-2"),
            ("boulder_footprint_max_x", "2"),
            ("boulder_footprint_max_y", "2"),
        ] {
            tags.insert(key.to_string(), value.to_string());
        }
        object
    }

    fn own_cliff() -> MapObject {
        MapObject::Area {
            object: geo::Rect::new((-2.25, -2.25), (2.25, 2.25)).to_polygon(),
            symbol: AreaSymbol::GiganticBoulder,
            tags: Default::default(),
        }
    }

    #[test]
    fn confirmed_block_replaces_its_own_cliffs_and_knoll_with_one_point() {
        let mut map = InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S10_000, None);
        map.add_object(confirmed_block());
        map.add_object(own_cliff());
        map.add_object(MapObject::Line {
            object: geo::LineString::from(vec![(2., -2.), (2., 2.)]),
            symbol: LineSymbol::ImpassableCliff,
            tags: Default::default(),
        });
        map.add_object(MapObject::Point {
            object: geo::Point::new(0., 0.),
            symbol: PointSymbol::DotKnoll,
            rotation: 0.,
            tags: Default::default(),
        });
        map.add_object(MapObject::Line {
            object: geo::LineString::from(vec![(20., -10.), (20., 10.)]),
            symbol: LineSymbol::Cliff,
            tags: Default::default(),
        });
        map.filter_boulder_points(2.);
        assert_eq!(positions(&map), vec![(0., 0.)]);
        assert_eq!(map.objects.values().map(Vec::len).sum::<usize>(), 2);
        assert_eq!(map.objects[&Symbol::Line(LineSymbol::Cliff)].len(), 1);
    }

    #[test]
    fn rejected_block_does_not_remove_cliffs_and_buildings_keep_priority() {
        for obstruction in [
            MapObject::Line {
                object: geo::LineString::from(vec![(3.5, -10.), (3.5, 10.)]),
                symbol: LineSymbol::Cliff,
                tags: Default::default(),
            },
            MapObject::Area {
                object: geo::Rect::new((-3., -3.), (3., 3.)).to_polygon(),
                symbol: AreaSymbol::Building,
                tags: Default::default(),
            },
        ] {
            let mut map = InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S10_000, None);
            map.add_object(confirmed_block());
            map.add_object(own_cliff());
            map.add_object(obstruction);
            map.filter_boulder_points(2.);
            assert!(positions(&map).is_empty());
            assert_eq!(map.objects.values().map(Vec::len).sum::<usize>(), 2);
            assert_eq!(
                map.objects[&Symbol::Area(AreaSymbol::GiganticBoulder)].len(),
                1
            );
        }
    }

    #[test]
    fn cleanup_resolves_buildings_cliff_areas_cliff_lines_and_knolls() {
        let mut map = InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S10_000, None);
        for (symbol, x) in [
            (AreaSymbol::Building, 0.),
            (AreaSymbol::GiganticBoulder, 20.),
        ] {
            map.add_object(MapObject::Area {
                object: geo::Rect::new((x - 2., -2.), (x + 2., 2.)).to_polygon(),
                symbol,
                tags: Default::default(),
            });
        }
        for (symbol, x) in [(LineSymbol::Cliff, 40.), (LineSymbol::ImpassableCliff, 60.)] {
            map.add_object(MapObject::Line {
                object: geo::LineString::from(vec![(x, -2.), (x, 2.)]),
                symbol,
                tags: Default::default(),
            });
        }
        map.add_object(MapObject::Point {
            object: geo::Point::new(80., 0.),
            symbol: PointSymbol::DotKnoll,
            rotation: 0.,
            tags: Default::default(),
        });
        for x in [0., 20., 40., 60., 80., 100.] {
            map.add_object(boulder(x, 0., 2.));
        }
        // A small boulder 2.5 m from a cliff clears its physical footprint,
        // but its printed point symbol and clearance would overlap the line.
        map.add_object(boulder(42.5, 0., 2.));
        map.filter_boulder_points(2.);
        assert_eq!(positions(&map), vec![(100., 0.)]);
    }

    #[test]
    fn separation_is_scale_aware_and_deterministic_across_tile_order() {
        for reverse in [false, true] {
            let mut map = InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S15_000, None);
            let mut objects = vec![
                boulder(0., 0., 1.5),
                boulder(5., 0., 3.),
                boulder(20., 0., 1.5),
            ];
            if reverse {
                objects.reverse();
            }
            for object in objects {
                map.add_object(object);
            }
            map.filter_boulder_points(2.);
            assert_eq!(positions(&map), vec![(5., 0.), (20., 0.)]);
        }
        let mut map = InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S10_000, None);
        map.add_object(boulder(0., 0., 1.5));
        map.add_object(boulder(5., 0., 3.));
        map.filter_boulder_points(2.);
        assert_eq!(positions(&map), vec![(0., 0.), (5., 0.)]);
    }

    #[test]
    fn large_boulders_take_priority_over_smaller_conflicting_symbols() {
        for reverse in [false, true] {
            let mut small = boulder(0., 0., 1.5);
            let mut large = boulder(3., 0., 2.);
            for (object, confidence) in [(&mut small, "0.99"), (&mut large, "0.75")] {
                let MapObject::Point { tags, .. } = object else {
                    unreachable!()
                };
                tags.insert("boulder_confidence".into(), confidence.into());
            }
            let mut objects = vec![small, large];
            if reverse {
                objects.reverse();
            }
            let mut map = InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S10_000, None);
            for object in objects {
                map.add_object(object);
            }
            map.filter_boulder_points(2.);
            assert_eq!(positions(&map), vec![(3., 0.)]);
        }
    }
}
