use std::collections::HashMap;

use geo::{Area, BooleanOps, BoundingRect, Buffer, Distance, Euclidean, Intersects, Length};
use rstar::{AABB, PointDistance, RTree, RTreeObject, primitives::GeomWithData};

use super::{
    AreaSymbol, InternalMap, LineSymbol, MapObject, Symbol,
    object::{PRESERVE_CONTOUR_GEOMETRY_TAG, STABLE_CONTOUR_SEAM_TAG},
};

const CLIFF_MERGE_DISTANCE_M: f64 = 1.;

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
struct IndexedGeometryEnvelope {
    envelope: AABB<[f64; 2]>,
    index: usize,
}

struct CliffLine<'a> {
    object: &'a geo::LineString,
    symbol: LineSymbol,
    object_index: usize,
    length: f64,
    minimum_length: f64,
}

impl RTreeObject for IndexedGeometryEnvelope {
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
        self.merge_lines_with_override(delta, None, false);
    }

    /// Merge lines while using a different endpoint distance for one symbol.
    pub fn merge_lines_with_symbol_distance(
        &mut self,
        default_delta: f64,
        symbol: LineSymbol,
        symbol_delta: f64,
    ) {
        self.merge_lines_with_override(default_delta, Some((symbol, symbol_delta)), false);
    }

    /// Merge only cliff lines whose directed end and start are within
    /// `delta`. Small and impassable cliffs remain separate symbol classes;
    /// neither input line is reversed, preserving downhill-right orientation.
    pub fn merge_cliff_lines(&mut self, delta: f64) {
        self.merge_lines_with_override(delta, None, true);
    }

    fn merge_lines_with_override(
        &mut self,
        default_delta: f64,
        symbol_override: Option<(LineSymbol, f64)>,
        cliffs_only: bool,
    ) {
        for (key, map_objects) in self.objects.iter_mut() {
            let Symbol::Line(line_symbol) = *key else {
                continue;
            };
            if cliffs_only
                && !matches!(line_symbol, LineSymbol::Cliff | LineSymbol::ImpassableCliff)
            {
                continue;
            }
            let delta = match symbol_override {
                Some((symbol, delta)) if line_symbol == symbol => delta,
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

    /// Merge and filter cartographically short cliff lines while treating
    /// touching small and impassable segments as one connected chain.
    ///
    /// Directed fragments are first joined at up to one metre. An impassable
    /// cliff that is still individually too short is demoted to an ordinary
    /// cliff and gets another opportunity to join neighboring ordinary cliff
    /// lines. A demoted fragment without such a neighbor is exaggerated by at
    /// most half a metre at each end. The resulting geometry is then evaluated
    /// normally: a line at least as long as its symbol minimum survives on its
    /// own; a shorter line survives only when it is at least one third of that
    /// minimum and its eligible connected component reaches the full minimum.
    /// Sub-third lines cannot bridge two otherwise undersized chains.
    pub fn filter_cliff_min_size(&mut self, connection_distance: f64) {
        self.merge_cliff_lines(CLIFF_MERGE_DISTANCE_M);
        self.demote_short_impassable_cliffs();
        self.merge_cliff_lines(CLIFF_MERGE_DISTANCE_M);

        let cliff_symbols = [LineSymbol::Cliff, LineSymbol::ImpassableCliff];
        let mut cliffs = Vec::new();
        for cliff_symbol in cliff_symbols {
            let Some(objects) = self.objects.get(&Symbol::Line(cliff_symbol)) else {
                continue;
            };
            for (object_index, object) in objects.iter().enumerate() {
                let MapObject::Line { object, symbol, .. } = object else {
                    continue;
                };
                cliffs.push(CliffLine {
                    object,
                    symbol: *symbol,
                    object_index,
                    length: Euclidean.length(object),
                    minimum_length: symbol.min_length(self.scale, object.is_closed()),
                });
            }
        }

        let connection_distance = if connection_distance.is_finite() {
            connection_distance.max(0.)
        } else {
            0.
        };
        let eligible = cliffs
            .iter()
            .map(|cliff| cliff.length >= cliff.minimum_length / 3.)
            .collect::<Vec<_>>();
        let indexed = cliffs
            .iter()
            .enumerate()
            .filter(|(index, _)| eligible[*index])
            .filter_map(|(index, cliff)| {
                line_envelope(cliff.object)
                    .map(|envelope| IndexedGeometryEnvelope { envelope, index })
            })
            .collect::<Vec<_>>();
        let tree = RTree::bulk_load(indexed.clone());
        let mut parents = (0..cliffs.len()).collect::<Vec<_>>();
        for line in indexed {
            let search = expanded_envelope(line.envelope, connection_distance);
            for other in tree.locate_in_envelope_intersecting(search) {
                if other.index <= line.index
                    || !cliff_lines_connected(
                        cliffs[line.index].object,
                        cliffs[other.index].object,
                        connection_distance,
                    )
                {
                    continue;
                }
                union_components(&mut parents, line.index, other.index);
            }
        }

        let mut component_lengths = vec![0.; cliffs.len()];
        for (index, cliff) in cliffs.iter().enumerate() {
            if eligible[index] {
                let root = component_root(&mut parents, index);
                component_lengths[root] += cliff.length;
            }
        }

        let mut keep = HashMap::with_capacity(cliffs.len());
        for (index, cliff) in cliffs.iter().enumerate() {
            let component_length = if eligible[index] {
                let root = component_root(&mut parents, index);
                component_lengths[root]
            } else {
                0.
            };
            keep.insert(
                (cliff.symbol, cliff.object_index),
                cliff.length >= cliff.minimum_length
                    || eligible[index] && component_length >= cliff.minimum_length,
            );
        }
        drop(cliffs);

        for cliff_symbol in cliff_symbols {
            let Some(objects) = self.objects.get_mut(&Symbol::Line(cliff_symbol)) else {
                continue;
            };
            let mut object_index = 0;
            objects.retain(|_| {
                let retain = keep
                    .get(&(cliff_symbol, object_index))
                    .copied()
                    .unwrap_or(true);
                object_index += 1;
                retain
            });
        }
    }

    fn demote_short_impassable_cliffs(&mut self) {
        let large_key = Symbol::Line(LineSymbol::ImpassableCliff);
        let mut demoted = Vec::new();
        if let Some(large_cliffs) = self.objects.get_mut(&large_key) {
            let mut retained = Vec::with_capacity(large_cliffs.len());
            for mut object in large_cliffs.drain(..) {
                let should_demote = matches!(
                    &object,
                    MapObject::Line { object, .. }
                        if Euclidean.length(object)
                            < LineSymbol::ImpassableCliff
                                .min_length(self.scale, object.is_closed())
                );
                if should_demote {
                    if let MapObject::Line { symbol, .. } = &mut object {
                        *symbol = LineSymbol::Cliff;
                    }
                    demoted.push(object);
                } else {
                    retained.push(object);
                }
            }
            *large_cliffs = retained;
        }
        if demoted.is_empty() {
            return;
        }

        let small_key = Symbol::Line(LineSymbol::Cliff);
        let existing_small = self
            .objects
            .get(&small_key)
            .map(Vec::as_slice)
            .unwrap_or(&[]);
        let lonesome = demoted
            .iter()
            .enumerate()
            .map(|(index, candidate)| {
                !existing_small.iter().any(|other| {
                    directed_map_lines_can_merge(candidate, other, CLIFF_MERGE_DISTANCE_M)
                }) && !demoted.iter().enumerate().any(|(other_index, other)| {
                    index != other_index
                        && directed_map_lines_can_merge(candidate, other, CLIFF_MERGE_DISTANCE_M)
                })
            })
            .collect::<Vec<_>>();

        for (object, lonesome) in demoted.iter_mut().zip(lonesome) {
            if lonesome && let MapObject::Line { object, .. } = object {
                exaggerate_open_line(object, CLIFF_MERGE_DISTANCE_M);
            }
        }
        self.objects.entry(small_key).or_default().extend(demoted);
    }
}

fn directed_map_lines_can_merge(first: &MapObject, second: &MapObject, tolerance: f64) -> bool {
    let (
        MapObject::Line {
            object: first,
            tags: first_tags,
            ..
        },
        MapObject::Line {
            object: second,
            tags: second_tags,
            ..
        },
    ) = (first, second)
    else {
        return false;
    };
    if first.0.len() < 2 || second.0.len() < 2 || first_tags.get("elev") != second_tags.get("elev")
    {
        return false;
    }

    let tolerance_squared = tolerance.max(0.).powi(2);
    let first_start = first.0[0];
    let first_end = first.0[first.0.len() - 1];
    let second_start = second.0[0];
    let second_end = second.0[second.0.len() - 1];
    squared_distance(first_end, second_start) <= tolerance_squared
        || squared_distance(second_end, first_start) <= tolerance_squared
}

fn exaggerate_open_line(line: &mut geo::LineString, total_amount: f64) {
    if line.is_closed() || line.0.len() < 2 || !total_amount.is_finite() || total_amount <= 0. {
        return;
    }

    let amount_per_end = total_amount / 2.;
    let first = line.0[0];
    let last = line.0[line.0.len() - 1];
    let first_extension = line
        .0
        .iter()
        .copied()
        .skip(1)
        .find_map(|next| extension_from_neighbor(first, next, amount_per_end));
    let last_extension = line
        .0
        .iter()
        .copied()
        .rev()
        .skip(1)
        .find_map(|previous| extension_from_neighbor(last, previous, amount_per_end));
    if let Some(first) = first_extension {
        line.0[0] = first;
    }
    if let Some(last) = last_extension {
        let last_index = line.0.len() - 1;
        line.0[last_index] = last;
    }
}

fn extension_from_neighbor(
    endpoint: geo::Coord,
    neighbor: geo::Coord,
    amount: f64,
) -> Option<geo::Coord> {
    let dx = endpoint.x - neighbor.x;
    let dy = endpoint.y - neighbor.y;
    let length = dx.hypot(dy);
    (length > f64::EPSILON).then(|| {
        geo::coord! {
            x: endpoint.x + amount * dx / length,
            y: endpoint.y + amount * dy / length,
        }
    })
}

fn squared_distance(first: geo::Coord, second: geo::Coord) -> f64 {
    (first.x - second.x).powi(2) + (first.y - second.y).powi(2)
}

fn line_envelope(line: &geo::LineString) -> Option<AABB<[f64; 2]>> {
    let bounds = line.bounding_rect()?;
    Some(AABB::from_corners(
        [bounds.min().x, bounds.min().y],
        [bounds.max().x, bounds.max().y],
    ))
}

fn expanded_envelope(envelope: AABB<[f64; 2]>, amount: f64) -> AABB<[f64; 2]> {
    let lower = envelope.lower();
    let upper = envelope.upper();
    AABB::from_corners(
        [lower[0] - amount, lower[1] - amount],
        [upper[0] + amount, upper[1] + amount],
    )
}

fn cliff_lines_connected(
    first: &geo::LineString,
    second: &geo::LineString,
    tolerance: f64,
) -> bool {
    [first.0.first(), first.0.last()]
        .into_iter()
        .flatten()
        .any(|endpoint| Euclidean.distance(&geo::Point::from(*endpoint), second) <= tolerance)
        || [second.0.first(), second.0.last()]
            .into_iter()
            .flatten()
            .any(|endpoint| Euclidean.distance(&geo::Point::from(*endpoint), first) <= tolerance)
}

fn component_root(parents: &mut [usize], index: usize) -> usize {
    if parents[index] != index {
        parents[index] = component_root(parents, parents[index]);
    }
    parents[index]
}

fn union_components(parents: &mut [usize], first: usize, second: usize) {
    let first = component_root(parents, first);
    let second = component_root(parents, second);
    if first != second {
        parents[second] = first;
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
            Some(IndexedGeometryEnvelope {
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
