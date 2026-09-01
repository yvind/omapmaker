use std::collections::HashMap;

use geo::{Area, BooleanOps, BoundingRect, Buffer, Intersects};
use rstar::{AABB, PointDistance, RTree, RTreeObject, primitives::GeomWithData};

use super::{
    AreaSymbol, InternalMap, LineSymbol, MapObject, Symbol,
    object::{PRESERVE_CONTOUR_GEOMETRY_TAG, STABLE_CONTOUR_SEAM_TAG},
};

struct MergeLine {
    object: geo::LineString,
    symbol: LineSymbol,
    tags: HashMap<String, String>,
}

struct MergeArea {
    object: geo::Polygon,
    symbol: AreaSymbol,
    tags: HashMap<String, String>,
}

#[derive(Clone, Copy)]
struct IndexedPolygonEnvelope {
    envelope: AABB<[f64; 2]>,
    index: usize,
}

impl RTreeObject for IndexedPolygonEnvelope {
    type Envelope = AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        self.envelope
    }
}

impl MergeLine {
    fn elevation_key(&self) -> Option<String> {
        self.tags.get("elev").cloned()
    }

    fn requires_exact_contour_merge(&self) -> bool {
        self.tags.contains_key(PRESERVE_CONTOUR_GEOMETRY_TAG)
            || self.tags.contains_key(STABLE_CONTOUR_SEAM_TAG)
    }

    fn start_point(&self) -> [f64; 2] {
        let start = self.object.0[0];
        [start.x, start.y]
    }

    fn end_point(&self) -> [f64; 2] {
        let end = self.object.0[self.object.0.len() - 1];
        [end.x, end.y]
    }

    fn into_map_object(self) -> MapObject {
        MapObject::Line {
            object: self.object,
            symbol: self.symbol,
            tags: self.tags,
        }
    }
}

impl MergeArea {
    fn signed_area(&self) -> f64 {
        self.object.signed_area()
    }

    fn abs_area(&self) -> f64 {
        self.signed_area().abs()
    }

    fn envelope(&self) -> Option<AABB<[f64; 2]>> {
        polygon_envelope(&self.object)
    }

    fn into_map_object(self) -> MapObject {
        MapObject::Area {
            object: self.object,
            symbol: self.symbol,
            tags: self.tags,
        }
    }
}

impl InternalMap {
    pub fn merge_areas(&mut self, symbol: AreaSymbol, delta: f64) -> crate::Result<()> {
        let objects = self.objects.remove(&Symbol::Area(symbol));

        let Some(objects) = objects else {
            return Ok(());
        };

        let areas = objects
            .into_iter()
            .map(|mo| match mo {
                MapObject::Area {
                    object,
                    symbol: _,
                    tags: _,
                } => Ok(object),
                MapObject::Line {
                    object: _,
                    symbol: _,
                    tags: _,
                } => anyhow::bail!("Should not be any Line objects under an Area key"),
                MapObject::Point {
                    object: _,
                    symbol: _,
                    rotation: _,
                    tags: _,
                } => anyhow::bail!("Should not be any Point objects under an Area key"),
            })
            .collect::<anyhow::Result<geo::MultiPolygon>>()?;

        let areas = areas.buffer(delta);
        let areas = geo::unary_union(&areas);
        let areas = areas.buffer(-delta);

        let objects = areas
            .into_iter()
            .map(|p| MapObject::Area {
                object: p,
                symbol,
                tags: Default::default(),
            })
            .collect::<Vec<_>>();

        self.objects.insert(Symbol::Area(symbol), objects);

        Ok(())
    }

    /// Subtract all polygons under `exclusion` from every polygon under
    /// `target`, preserving the target symbol and object tags.
    pub fn subtract_area_symbol(
        &mut self,
        target: AreaSymbol,
        exclusion: AreaSymbol,
    ) -> crate::Result<()> {
        let Some(exclusions) = self.objects.get(&Symbol::Area(exclusion)) else {
            return Ok(());
        };
        let exclusions = exclusions
            .iter()
            .map(|object| match object {
                MapObject::Area { object, .. } => Ok(object.clone()),
                _ => anyhow::bail!("Should not be a non-area object under an Area key"),
            })
            .collect::<crate::Result<geo::MultiPolygon>>()?;
        if exclusions.0.is_empty() {
            return Ok(());
        }
        let Some(targets) = self.objects.get_mut(&Symbol::Area(target)) else {
            return Ok(());
        };
        let mut clipped = Vec::new();
        for object in targets.drain(..) {
            let MapObject::Area {
                object,
                symbol,
                tags,
            } = object
            else {
                anyhow::bail!("Should not be a non-area object under an Area key");
            };
            clipped.extend(object.difference(&exclusions).into_iter().map(|object| {
                MapObject::Area {
                    object,
                    symbol,
                    tags: tags.clone(),
                }
            }));
        }
        *targets = clipped;
        Ok(())
    }

    pub fn merge_and_filter_min_size(
        &mut self,
        symbols: impl IntoIterator<Item = AreaSymbol>,
    ) -> crate::Result<()> {
        let min_areas = symbols
            .into_iter()
            .map(|a| (a, a.min_size(&self.scale)))
            .collect::<Vec<_>>();

        for (symbol, min_area) in min_areas {
            self.merge_and_filter_symbol_min_size(symbol, min_area);
        }

        Ok(())
    }

    pub fn filter_area_min_size(&mut self, symbol: AreaSymbol, minimum_area_m2: f64) {
        if minimum_area_m2.is_finite() && minimum_area_m2 > 0. {
            self.merge_and_filter_symbol_min_size(symbol, minimum_area_m2);
        }
    }

    fn merge_and_filter_symbol_min_size(&mut self, symbol: AreaSymbol, min_area: f64) {
        let Some(map_objects) = self.objects.get_mut(&Symbol::Area(symbol)) else {
            return;
        };

        let mut areas = Vec::with_capacity(map_objects.len());
        let mut others = Vec::new();

        for map_object in map_objects.drain(..) {
            if let MapObject::Area {
                object,
                symbol,
                tags,
            } = map_object
            {
                areas.push(MergeArea {
                    object,
                    symbol,
                    tags,
                });
            } else {
                others.push(map_object);
            }
        }

        merge_small_areas(&mut areas, min_area);

        map_objects.extend(
            areas
                .into_iter()
                .filter(|area| area.abs_area() >= min_area)
                .map(MergeArea::into_map_object),
        );
        map_objects.extend(others);
    }

    /// Merge line objects that are tip to tail.
    /// Line ends (directed) of the same symbol that are less than `delta`
    /// units apart are merged. Elevation tags are respected and only elements
    /// with equal elevation tags can be merged.
    pub fn merge_lines(&mut self, delta: f64) {
        self.merge_lines_with_override(delta, None);
    }

    /// Merge lines while using a different endpoint distance for one symbol.
    pub fn merge_lines_with_symbol_distance(
        &mut self,
        default_delta: f64,
        symbol: LineSymbol,
        symbol_delta: f64,
    ) {
        self.merge_lines_with_override(default_delta, Some((symbol, symbol_delta)));
    }

    fn merge_lines_with_override(
        &mut self,
        default_delta: f64,
        symbol_override: Option<(LineSymbol, f64)>,
    ) {
        for (key, map_objects) in self.objects.iter_mut() {
            if !matches!(key, Symbol::Line(_)) {
                continue;
            }
            let delta = match symbol_override {
                Some((symbol, delta)) if *key == Symbol::Line(symbol) => delta,
                _ => default_delta,
            };
            let delta = delta * delta;
            let allow_self_merge = *key != Symbol::Line(LineSymbol::SmallCrossableWatercourse);

            let mut unclosed_objects = Vec::with_capacity(map_objects.len());

            let mut i = 0;
            while i < map_objects.len() {
                if let MapObject::Line {
                    object,
                    symbol: _,
                    tags: _,
                } = &map_objects[i]
                {
                    if !object.is_closed() && object.0.len() >= 2 {
                        let MapObject::Line {
                            object,
                            symbol,
                            tags,
                        } = map_objects.swap_remove(i)
                        else {
                            unreachable!("checked line object before swap_remove");
                        };
                        unclosed_objects.push(MergeLine {
                            object,
                            symbol,
                            tags,
                        });
                    } else {
                        i += 1;
                    }
                } else {
                    i += 1;
                }
            }

            let mut unclosed_object_groups =
                HashMap::<(Option<String>, bool), Vec<MergeLine>>::new();
            for unclosed_object in unclosed_objects {
                unclosed_object_groups
                    .entry((
                        unclosed_object.elevation_key(),
                        unclosed_object.requires_exact_contour_merge(),
                    ))
                    .or_default()
                    .push(unclosed_object);
            }

            for ((_, exact_contour_merge), mut unclosed_objects) in unclosed_object_groups {
                // Preserved contour geometry may only join at effectively
                // identical endpoints. This stitches exact tile cuts without
                // bridging a deliberately pruned form-line gap.
                let merge_delta = if exact_contour_merge { 1e-16 } else { delta };
                let (line_ends, line_starts): (Vec<_>, Vec<_>) = unclosed_objects
                    .iter()
                    .enumerate()
                    .map(|(i, o)| (GeomWithData::new(o.end_point(), i), o.start_point()))
                    .collect();

                // detect the merges needed
                let end_tree = RTree::bulk_load(line_ends);

                let mut merges = Vec::with_capacity(line_starts.len());
                for (start_i, line_start) in line_starts.iter().enumerate() {
                    if let Some(nn) = end_tree
                        .nearest_neighbor_iter(*line_start)
                        .find(|candidate| allow_self_merge || start_i != candidate.data)
                        && nn.distance_2(line_start) <= merge_delta
                    {
                        merges.push((start_i, nn.data));
                    }
                }

                // start doing merges keeping track of the moved objects
                while let Some(merge) = merges.pop() {
                    if merge.0 == merge.1 {
                        let mut line = unclosed_objects.swap_remove(merge.0);
                        line.object.close();

                        map_objects.push(line.into_map_object());
                    } else {
                        // merge
                        let part2 = unclosed_objects.swap_remove(merge.0);

                        let part1 = if merge.1 >= unclosed_objects.len() {
                            &mut unclosed_objects[merge.0]
                        } else {
                            &mut unclosed_objects[merge.1]
                        };

                        let _ = part1.object.0.pop();
                        part1.object.0.extend(part2.object.0);
                    }
                    // update map
                    let mut i = 0;
                    while i < merges.len() {
                        let other_merge = &mut merges[i];

                        // find merges made impossible
                        if other_merge.1 == merge.1 || other_merge.0 == merge.0 {
                            let _ = merges.swap_remove(i);
                            continue;
                        } else {
                            i += 1;
                        }

                        // update map as merge.0 is now called merge.1
                        if other_merge.0 == merge.0 {
                            other_merge.0 = merge.1
                        }
                        if other_merge.1 == merge.0 {
                            other_merge.1 = merge.1
                        }

                        // correct map for swap remove moving object
                        if other_merge.0 >= unclosed_objects.len() {
                            other_merge.0 = merge.0;
                        }
                        if other_merge.1 >= unclosed_objects.len() {
                            other_merge.1 = merge.0;
                        }
                    }
                }
                let unclosed = unclosed_objects.into_iter().map(|mut line_object| {
                    // check if it is almost closed
                    let start = line_object.object.0[0];
                    let end = line_object.object.0[line_object.object.0.len() - 1];

                    if allow_self_merge
                        && (start.x - end.x).powi(2) + (start.y - end.y).powi(2) <= merge_delta
                    {
                        line_object.object.close();
                    }

                    line_object.into_map_object()
                });

                map_objects.extend(unclosed);
            }
        }
    }
}

fn merge_small_areas(areas: &mut Vec<MergeArea>, min_area: f64) {
    let mut active = vec![true; areas.len()];
    let mut candidate_lookup = small_area_merge_candidates(areas, min_area);

    while let Some((small_index, target_index)) =
        find_small_area_merge(areas, &active, &candidate_lookup, min_area)
    {
        let union = areas[target_index].object.union(&areas[small_index].object);
        if union.0.len() == 1 {
            areas[target_index].object = union.0.into_iter().next().expect("checked union length");
            active[small_index] = false;

            let absorbed_candidates = std::mem::take(&mut candidate_lookup[small_index]);
            candidate_lookup[target_index].extend(absorbed_candidates);
        }
    }

    let mut active = active.into_iter();
    areas.retain(|_| active.next().unwrap_or(false));
}

fn small_area_merge_candidates(areas: &[MergeArea], min_area: f64) -> Vec<Vec<usize>> {
    let indexed_polygons = areas
        .iter()
        .enumerate()
        .filter_map(|(index, area)| {
            Some(IndexedPolygonEnvelope {
                envelope: area.envelope()?,
                index,
            })
        })
        .collect::<Vec<_>>();

    let tree = RTree::bulk_load(indexed_polygons);
    let mut candidate_lookup = vec![Vec::new(); areas.len()];

    for (small_index, small_area) in areas.iter().enumerate() {
        let small_abs_area = small_area.abs_area();
        if small_abs_area >= min_area {
            continue;
        }

        let Some(envelope) = small_area.envelope() else {
            continue;
        };

        for candidate in tree.locate_in_envelope_intersecting(envelope) {
            if candidate.index == small_index {
                continue;
            }

            let candidate_area = areas[candidate.index].abs_area();
            if candidate_area < small_abs_area {
                continue;
            }

            if small_area.object.intersects(&areas[candidate.index].object)
                && small_area
                    .object
                    .union(&areas[candidate.index].object)
                    .0
                    .len()
                    == 1
            {
                candidate_lookup[small_index].push(candidate.index);
            }
        }
    }

    candidate_lookup
}

fn find_small_area_merge(
    areas: &[MergeArea],
    active: &[bool],
    candidate_lookup: &[Vec<usize>],
    min_area: f64,
) -> Option<(usize, usize)> {
    for (small_index, small_area) in areas.iter().enumerate() {
        if !active[small_index] {
            continue;
        }

        let small_abs_area = small_area.abs_area();
        if small_abs_area >= min_area {
            continue;
        }

        let mut best_target = None;
        let mut best_area = 0.;
        for &candidate_index in &candidate_lookup[small_index] {
            if !active[candidate_index] || candidate_index == small_index {
                continue;
            }

            let candidate_area = areas[candidate_index].abs_area();
            if candidate_area < small_abs_area || candidate_area <= best_area {
                continue;
            }

            if small_area.object.intersects(&areas[candidate_index].object)
                && small_area
                    .object
                    .union(&areas[candidate_index].object)
                    .0
                    .len()
                    == 1
            {
                best_area = candidate_area;
                best_target = Some(candidate_index);
            }
        }

        if let Some(target_index) = best_target {
            return Some((small_index, target_index));
        }
    }

    None
}

fn polygon_envelope(polygon: &geo::Polygon) -> Option<AABB<[f64; 2]>> {
    let rect = polygon.bounding_rect()?;
    Some(AABB::from_corners(
        [rect.min().x, rect.min().y],
        [rect.max().x, rect.max().y],
    ))
}
