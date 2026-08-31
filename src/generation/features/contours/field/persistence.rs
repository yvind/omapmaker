use super::{ExtremumKind, PersistenceWork, ProtectedPersistenceFeature};
use crate::raster::{Dfm, Elevation, RasterMarker, TargetElevation};
use rayon::prelude::*;
use std::collections::HashSet;
use std::ops::Range;

const PERSISTENCE_EPSILON: f32 = 1e-6;
const NO_CELL: usize = usize::MAX;

#[derive(Clone, Debug)]
pub(super) struct PersistenceSummary {
    pub(super) requested: usize,
    pub(super) removed: usize,
    pub(super) preserved: usize,
    pub(super) unresolved: usize,
    pub(super) removed_extrema: Vec<usize>,
    pub(super) protected_features: Vec<ProtectedPersistenceFeature>,
    pub(super) work: PersistenceWork,
}

#[derive(Clone, Debug)]
struct PersistencePair {
    id: u64,
    kind: ExtremumKind,
    extremum_index: usize,
    saddle_index: usize,
    extremum_elevation: f32,
    saddle_elevation: f32,
    persistence: f32,
    terminal: bool,
    component_head: usize,
    component_tail: usize,
}

struct PersistenceDiagram {
    pairs: Vec<PersistencePair>,
    minimum_next: Vec<usize>,
    maximum_next: Vec<usize>,
}

impl PersistenceDiagram {
    fn component_next(&self, kind: ExtremumKind) -> &[usize] {
        match kind {
            ExtremumKind::Minimum => &self.minimum_next,
            ExtremumKind::Maximum => &self.maximum_next,
        }
    }

    fn has_candidates(&self, threshold: f32) -> bool {
        self.pairs.iter().any(|pair| is_candidate(pair, threshold))
    }
}

#[derive(Default)]
pub(super) struct PersistenceWorkspace {
    order: Vec<usize>,
    groups: Vec<(usize, usize)>,
    active: Vec<bool>,
    parent: Vec<usize>,
    size: Vec<usize>,
    birth: Vec<f32>,
    extremum: Vec<usize>,
    head: Vec<usize>,
    tail: Vec<usize>,
    next: Vec<usize>,
    locks: Vec<u32>,
    lock_generation: u32,
    candidate_indices: Vec<usize>,
    component: Vec<usize>,
}

impl PersistenceWorkspace {
    fn prepare_order<T: RasterMarker>(&mut self, input: &Dfm<T>) {
        let len = input.field.len();
        self.order.clear();
        self.order.extend(0..len);
        self.order.par_sort_unstable_by(|&a, &b| {
            input.field[a].total_cmp(&input.field[b]).then(a.cmp(&b))
        });
        self.groups.clear();
        let mut start = 0;
        while start < len {
            let value = input.field[self.order[start]];
            let mut end = start + 1;
            while end < len && input.field[self.order[end]].total_cmp(&value).is_eq() {
                end += 1;
            }
            self.groups.push((start, end));
            start = end;
        }
    }

    fn prepare_sweep<T: RasterMarker>(&mut self, input: &Dfm<T>) {
        let len = input.field.len();
        self.active.resize(len, false);
        self.active.fill(false);
        self.parent.clear();
        self.parent.extend(0..len);
        self.size.resize(len, 1);
        self.size.fill(1);
        self.birth.clear();
        self.birth.extend_from_slice(&input.field);
        self.extremum.clear();
        self.extremum.extend(0..len);
        self.head.clear();
        self.head.extend(0..len);
        self.tail.clear();
        self.tail.extend(0..len);
        self.next.resize(len, NO_CELL);
        self.next.fill(NO_CELL);
    }

    fn next_lock_generation(&mut self, len: usize) -> u32 {
        self.locks.resize(len, 0);
        self.lock_generation = self.lock_generation.wrapping_add(1);
        if self.lock_generation == 0 {
            self.locks.fill(0);
            self.lock_generation = 1;
        }
        self.lock_generation
    }
}

#[derive(Clone, Copy)]
struct RequestedFeature {
    id: u64,
    kind: ExtremumKind,
    extremum_index: usize,
}

struct CancellationEdit {
    target: f32,
    cells: Range<usize>,
}

#[derive(Default)]
struct CancellationPlan {
    cells: Vec<usize>,
    edits: Vec<CancellationEdit>,
}

pub(super) fn simplify_bounded<T: RasterMarker>(
    original: &Dfm<Elevation>,
    input: &Dfm<T>,
    threshold: f32,
    adjustment_bound: f32,
    workspace: &mut PersistenceWorkspace,
) -> (Dfm<TargetElevation>, PersistenceSummary) {
    original
        .grid
        .ensure_compatible(&input.grid)
        .expect("persistence cleanup requires matching grids");
    assert!(threshold.is_finite() && threshold >= 0.);
    assert!(adjustment_bound.is_finite() && adjustment_bound >= 0.);

    let mut current = input.clone();
    let mut work = PersistenceWork::default();
    if threshold <= PERSISTENCE_EPSILON {
        return finish_without_diagram(original, &current, work);
    }

    let initial_diagram = pair_persistence(input, workspace, &mut work);
    let requested = unique_requested(
        initial_diagram
            .pairs
            .iter()
            .filter(|pair| is_candidate(pair, threshold)),
    );
    if !initial_diagram.has_candidates(threshold) {
        return finish_with_diagram(
            original,
            &current,
            requested,
            &initial_diagram,
            threshold,
            work,
        );
    }

    let minimum_plan = plan_bounded_cancellations(
        original,
        &current,
        &initial_diagram,
        ExtremumKind::Minimum,
        threshold,
        adjustment_bound,
        workspace,
        &mut work,
    );
    let minimum_changed = apply_cancellation_plan(&mut current, &minimum_plan, &mut work);

    let after_minimum =
        (minimum_changed > 0).then(|| pair_persistence(&current, workspace, &mut work));
    let maximum_diagram = after_minimum.as_ref().unwrap_or(&initial_diagram);
    let maximum_plan = plan_bounded_cancellations(
        original,
        &current,
        maximum_diagram,
        ExtremumKind::Maximum,
        threshold,
        adjustment_bound,
        workspace,
        &mut work,
    );
    let maximum_changed = apply_cancellation_plan(&mut current, &maximum_plan, &mut work);

    let final_diagram =
        (maximum_changed > 0).then(|| pair_persistence(&current, workspace, &mut work));
    finish_with_diagram(
        original,
        &current,
        requested,
        final_diagram.as_ref().unwrap_or(maximum_diagram),
        threshold,
        work,
    )
}

fn finish_without_diagram<T: RasterMarker>(
    original: &Dfm<Elevation>,
    current: &Dfm<T>,
    work: PersistenceWork,
) -> (Dfm<TargetElevation>, PersistenceSummary) {
    let mut output = Dfm::<TargetElevation>::new_like(original);
    output.field.copy_from_slice(&current.field);
    (
        output,
        PersistenceSummary {
            requested: 0,
            removed: 0,
            preserved: 0,
            unresolved: 0,
            removed_extrema: Vec::new(),
            protected_features: Vec::new(),
            work,
        },
    )
}

fn finish_with_diagram<T: RasterMarker>(
    original: &Dfm<Elevation>,
    current: &Dfm<T>,
    requested: Vec<RequestedFeature>,
    final_diagram: &PersistenceDiagram,
    threshold: f32,
    work: PersistenceWork,
) -> (Dfm<TargetElevation>, PersistenceSummary) {
    let surviving_extrema = final_diagram
        .pairs
        .iter()
        .filter(|pair| pair.persistence > PERSISTENCE_EPSILON)
        .map(|pair| (pair.kind, pair.extremum_index))
        .collect::<HashSet<_>>();
    let removed = requested
        .iter()
        .filter(|feature| !surviving_extrema.contains(&(feature.kind, feature.extremum_index)))
        .collect::<Vec<_>>();
    let mut output = Dfm::<TargetElevation>::new_like(original);
    output.field.copy_from_slice(&current.field);
    let mut removed_extrema = removed
        .iter()
        .map(|feature| feature.extremum_index)
        .collect::<Vec<_>>();
    removed_extrema.sort_unstable();
    removed_extrema.dedup();

    let unresolved = final_diagram
        .pairs
        .iter()
        .filter(|pair| is_candidate(pair, threshold))
        .count();
    let protected_features = final_diagram
        .pairs
        .iter()
        .filter(|pair| pair.persistence >= threshold)
        .map(|pair| ProtectedPersistenceFeature {
            pair_id: pair.id,
            kind: pair.kind,
            extremum_index: pair.extremum_index,
            extremum: original.index2coord(
                pair.extremum_index / original.width(),
                pair.extremum_index % original.width(),
            ),
            extremum_elevation: pair.extremum_elevation,
            saddle_elevation: pair.saddle_elevation,
            persistence: pair.persistence,
        })
        .collect::<Vec<_>>();
    let summary = PersistenceSummary {
        requested: requested.len(),
        removed: removed.len(),
        preserved: protected_features.len(),
        unresolved,
        removed_extrema,
        protected_features,
        work,
    };
    (output, summary)
}

fn unique_requested<'a>(
    pairs: impl IntoIterator<Item = &'a PersistencePair>,
) -> Vec<RequestedFeature> {
    let mut requested = pairs
        .into_iter()
        .map(|pair| RequestedFeature {
            id: pair.id,
            kind: pair.kind,
            extremum_index: pair.extremum_index,
        })
        .collect::<Vec<_>>();
    requested.sort_by_key(|feature| feature.id);
    requested.dedup_by_key(|feature| feature.id);
    requested
}

fn is_candidate(pair: &PersistencePair, threshold: f32) -> bool {
    pair.persistence > PERSISTENCE_EPSILON && pair.persistence < threshold
}

#[allow(clippy::too_many_arguments)]
fn plan_bounded_cancellations<T: RasterMarker>(
    original: &Dfm<Elevation>,
    current: &Dfm<T>,
    diagram: &PersistenceDiagram,
    kind: ExtremumKind,
    threshold: f32,
    bound: f32,
    workspace: &mut PersistenceWorkspace,
    work: &mut PersistenceWork,
) -> CancellationPlan {
    work.cancellation_passes += 1;
    workspace.candidate_indices.clear();
    workspace.candidate_indices.extend(
        diagram
            .pairs
            .iter()
            .enumerate()
            .filter(|(_, pair)| pair.kind == kind && is_candidate(pair, threshold))
            .map(|(index, _)| index),
    );
    workspace.candidate_indices.sort_by(|&a, &b| {
        let a = &diagram.pairs[a];
        let b = &diagram.pairs[b];
        a.persistence
            .total_cmp(&b.persistence)
            .then(a.extremum_index.cmp(&b.extremum_index))
            .then(a.saddle_index.cmp(&b.saddle_index))
            .then(a.id.cmp(&b.id))
    });

    let generation = workspace.next_lock_generation(current.field.len());
    let next = diagram.component_next(kind);
    let mut plan = CancellationPlan::default();
    for &pair_index in &workspace.candidate_indices {
        work.candidates_considered += 1;
        let pair = &diagram.pairs[pair_index];
        workspace.component.clear();
        let mut lower = f32::NEG_INFINITY;
        let mut upper = f32::INFINITY;
        let mut locked = false;
        let mut index = pair.component_head;
        loop {
            if workspace.locks[index] == generation {
                locked = true;
                break;
            }
            workspace.component.push(index);
            lower = lower.max(original.field[index] - bound);
            upper = upper.min(original.field[index] + bound);
            if index == pair.component_tail {
                break;
            }
            index = next[index];
            debug_assert_ne!(index, NO_CELL, "component tail must remain reachable");
        }
        if locked {
            continue;
        }
        if !pair.terminal {
            let saddle = pair.saddle_index;
            if workspace.locks[saddle] == generation {
                continue;
            }
            workspace.component.push(saddle);
            lower = lower.max(original.field[saddle] - bound);
            upper = upper.min(original.field[saddle] + bound);
        }
        if lower > upper + PERSISTENCE_EPSILON {
            continue;
        }
        let target = ((f64::from(lower) + f64::from(upper)) * 0.5) as f32;
        if !workspace
            .component
            .iter()
            .any(|&cell| (current.field[cell] - target).abs() > PERSISTENCE_EPSILON)
        {
            continue;
        }

        let start = plan.cells.len();
        for &cell in &workspace.component {
            debug_assert!(
                target >= original.field[cell] - bound - PERSISTENCE_EPSILON
                    && target <= original.field[cell] + bound + PERSISTENCE_EPSILON
            );
            workspace.locks[cell] = generation;
            plan.cells.push(cell);
        }
        plan.edits.push(CancellationEdit {
            target,
            cells: start..plan.cells.len(),
        });
    }
    plan
}

fn apply_cancellation_plan<T: RasterMarker>(
    current: &mut Dfm<T>,
    plan: &CancellationPlan,
    work: &mut PersistenceWork,
) -> usize {
    let mut changed = 0;
    for edit in &plan.edits {
        let mut edit_changed = false;
        for &index in &plan.cells[edit.cells.clone()] {
            edit_changed |= (current.field[index] - edit.target).abs() > PERSISTENCE_EPSILON;
            current.field[index] = edit.target;
        }
        changed += usize::from(edit_changed);
    }
    work.cancellations_applied += changed;
    work.affected_cells_written += plan.cells.len();
    debug_assert!(plan.edits.is_empty() || changed > 0);
    changed
}

fn pair_persistence<T: RasterMarker>(
    input: &Dfm<T>,
    workspace: &mut PersistenceWorkspace,
    work: &mut PersistenceWork,
) -> PersistenceDiagram {
    work.diagram_builds += 1;
    workspace.prepare_order(input);
    let (mut pairs, minimum_next) = pair_pass(input, ExtremumKind::Minimum, workspace);
    let (maximum_pairs, maximum_next) = pair_pass(input, ExtremumKind::Maximum, workspace);
    pairs.extend(maximum_pairs);
    let mut seen = HashSet::new();
    pairs.retain(|pair| seen.insert(pair.id));
    pairs.sort_by_key(|pair| pair.id);
    PersistenceDiagram {
        pairs,
        minimum_next,
        maximum_next,
    }
}

fn pair_pass<T: RasterMarker>(
    input: &Dfm<T>,
    kind: ExtremumKind,
    workspace: &mut PersistenceWorkspace,
) -> (Vec<PersistencePair>, Vec<usize>) {
    let ascending = kind == ExtremumKind::Minimum;
    let width = input.width();
    let height = input.height();
    workspace.prepare_sweep(input);
    let mut pairs = Vec::new();
    let mut terminal_index = 0;

    if ascending {
        for position in 0..workspace.order.len() {
            let index = workspace.order[position];
            process_cell(input, kind, index, workspace, &mut pairs);
            terminal_index = index;
        }
    } else {
        for group_index in (0..workspace.groups.len()).rev() {
            let (start, end) = workspace.groups[group_index];
            for position in start..end {
                let index = workspace.order[position];
                process_cell(input, kind, index, workspace, &mut pairs);
                terminal_index = index;
            }
        }
    }

    let root = find(&mut workspace.parent, terminal_index);
    let essential_extremum = workspace.extremum[root];
    if !is_boundary(essential_extremum, width, height) {
        pairs.push(make_pair(
            input,
            kind,
            essential_extremum,
            terminal_index,
            true,
            workspace.head[root],
            workspace.tail[root],
        ));
    }
    (pairs, workspace.next.clone())
}

fn process_cell<T: RasterMarker>(
    input: &Dfm<T>,
    kind: ExtremumKind,
    index: usize,
    workspace: &mut PersistenceWorkspace,
    pairs: &mut Vec<PersistencePair>,
) {
    let ascending = kind == ExtremumKind::Minimum;
    let width = input.width();
    let height = input.height();
    workspace.active[index] = true;
    let y = index / width;
    let x = index % width;
    // Persistence uses deterministic 4-neighbour connectivity. Changing it
    // alters diagonal saddle pairing.
    let mut roots = [NO_CELL; 4];
    let mut root_count = 0;
    for neighbor in [
        (x > 0).then_some(index.wrapping_sub(1)),
        (x + 1 < width).then_some(index + 1),
        (y > 0).then_some(index.wrapping_sub(width)),
        (y + 1 < height).then_some(index + width),
    ]
    .into_iter()
    .flatten()
    {
        if workspace.active[neighbor] {
            roots[root_count] = find(&mut workspace.parent, neighbor);
            root_count += 1;
        }
    }
    roots[..root_count].sort_unstable();
    let mut unique = 0;
    for position in 0..root_count {
        if position == 0 || roots[position] != roots[position - 1] {
            roots[unique] = roots[position];
            unique += 1;
        }
    }
    root_count = unique;
    if root_count == 0 {
        return;
    }

    let survivor = *roots[..root_count]
        .iter()
        .min_by(|&&a, &&b| {
            let ordering = workspace.birth[a].total_cmp(&workspace.birth[b]);
            if ascending {
                ordering.then(workspace.extremum[a].cmp(&workspace.extremum[b]))
            } else {
                ordering
                    .reverse()
                    .then(workspace.extremum[a].cmp(&workspace.extremum[b]))
            }
        })
        .expect("nonempty neighboring components");
    let survivor_birth = workspace.birth[survivor];
    let survivor_extremum = workspace.extremum[survivor];
    for &root in &roots[..root_count] {
        if root != survivor {
            pairs.push(make_pair(
                input,
                kind,
                workspace.extremum[root],
                index,
                false,
                workspace.head[root],
                workspace.tail[root],
            ));
        }
    }

    let mut combined = roots[0];
    for &root in &roots[1..root_count] {
        combined = union_components(
            &mut workspace.parent,
            &mut workspace.size,
            &mut workspace.head,
            &mut workspace.tail,
            &mut workspace.next,
            combined,
            root,
        );
    }
    combined = union_components(
        &mut workspace.parent,
        &mut workspace.size,
        &mut workspace.head,
        &mut workspace.tail,
        &mut workspace.next,
        combined,
        index,
    );
    workspace.birth[combined] = survivor_birth;
    workspace.extremum[combined] = survivor_extremum;
}

fn make_pair<T: RasterMarker>(
    input: &Dfm<T>,
    kind: ExtremumKind,
    extremum_index: usize,
    saddle_index: usize,
    terminal: bool,
    component_head: usize,
    component_tail: usize,
) -> PersistencePair {
    let extremum_elevation = input.field[extremum_index];
    let saddle_elevation = input.field[saddle_index];
    PersistencePair {
        id: pair_id(kind, extremum_index, saddle_index),
        kind,
        extremum_index,
        saddle_index,
        extremum_elevation,
        saddle_elevation,
        persistence: (extremum_elevation - saddle_elevation).abs(),
        terminal,
        component_head,
        component_tail,
    }
}

fn pair_id(kind: ExtremumKind, extremum_index: usize, saddle_index: usize) -> u64 {
    let mut hash = 0xcbf29ce484222325_u64;
    for value in [kind as u64, extremum_index as u64, saddle_index as u64] {
        for byte in value.to_le_bytes() {
            hash ^= u64::from(byte);
            hash = hash.wrapping_mul(0x100000001b3);
        }
    }
    hash
}

fn is_boundary(index: usize, width: usize, height: usize) -> bool {
    let y = index / width;
    let x = index % width;
    x == 0 || x + 1 == width || y == 0 || y + 1 == height
}

fn find(parent: &mut [usize], mut index: usize) -> usize {
    while parent[index] != index {
        parent[index] = parent[parent[index]];
        index = parent[index];
    }
    index
}

#[allow(clippy::too_many_arguments)]
fn union_components(
    parent: &mut [usize],
    size: &mut [usize],
    head: &mut [usize],
    tail: &mut [usize],
    next: &mut [usize],
    a: usize,
    b: usize,
) -> usize {
    let mut a = find(parent, a);
    let mut b = find(parent, b);
    if a == b {
        return a;
    }
    if size[a] < size[b] {
        std::mem::swap(&mut a, &mut b);
    }
    parent[b] = a;
    size[a] += size[b];
    next[tail[a]] = head[b];
    tail[a] = tail[b];
    a
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::raster::DfmGrid;

    fn isolated_extremum(value: f32, background: f32) -> Dfm<Elevation> {
        let grid = DfmGrid::new(5, 5, 1., geo::coord! { x: 0., y: 4. }).unwrap();
        let mut source = Dfm::new(grid);
        source.field.fill(background);
        source[(2, 2)] = value;
        source
    }

    fn simplify(
        source: &Dfm<Elevation>,
        threshold: f32,
        bound: f32,
    ) -> (Dfm<TargetElevation>, PersistenceSummary) {
        simplify_bounded(
            source,
            source,
            threshold,
            bound,
            &mut PersistenceWorkspace::default(),
        )
    }

    fn diagram(input: &Dfm<Elevation>) -> PersistenceDiagram {
        pair_persistence(
            input,
            &mut PersistenceWorkspace::default(),
            &mut PersistenceWork::default(),
        )
    }

    fn within_bound(target: &Dfm<TargetElevation>, source: &Dfm<Elevation>, bound: f32) -> bool {
        target
            .field
            .iter()
            .zip(&source.field)
            .all(|(a, b)| (a - b).abs() <= bound + 1e-6)
    }

    #[test]
    fn shallow_extremum_is_removed_within_bound() {
        let source = isolated_extremum(0.2, 0.);
        let (target, summary) = simplify(&source, 0.3, 0.25);
        assert!(summary.removed > 0);
        assert!((target[(2, 2)] - target[(2, 1)]).abs() < 1e-6);
        assert!(within_bound(&target, &source, 0.25));
        assert!(summary.work.diagram_builds <= 3);
        assert_eq!(summary.work.cancellation_passes, 2);
    }

    #[test]
    fn two_sided_bound_cancels_an_isolated_maximum() {
        let source = isolated_extremum(0.4, 0.);
        let (target, summary) = simplify(&source, 0.5, 0.25);
        assert!(summary.removed > 0);
        assert_eq!(summary.unresolved, 0);
        assert!(
            target
                .field
                .windows(2)
                .all(|pair| (pair[0] - pair[1]).abs() < 1e-6)
        );
        assert!(within_bound(&target, &source, 0.25));
    }

    #[test]
    fn two_sided_bound_cancels_an_isolated_minimum() {
        let source = isolated_extremum(0., 0.4);
        let (target, summary) = simplify(&source, 0.5, 0.25);
        assert!(summary.removed > 0);
        assert_eq!(summary.unresolved, 0);
        assert!(
            target
                .field
                .windows(2)
                .all(|pair| (pair[0] - pair[1]).abs() < 1e-6)
        );
        assert!(within_bound(&target, &source, 0.25));
    }

    #[test]
    fn feature_above_twice_the_bound_is_unresolved() {
        let source = isolated_extremum(0.6, 0.);
        let (target, summary) = simplify(&source, 0.7, 0.25);
        assert_eq!(target[(2, 2)], 0.6);
        assert_eq!(summary.removed, 0);
        assert!(summary.unresolved > 0);
    }

    #[test]
    fn prominent_small_extremum_is_protected() {
        let source = isolated_extremum(1., 0.);
        let (target, summary) = simplify(&source, 0.3, 0.25);
        assert!(!summary.protected_features.is_empty());
        assert_eq!(target[(2, 2)], 1.);
    }

    #[test]
    fn persistence_pairing_is_deterministic_on_plateaus() {
        let mut source = isolated_extremum(1., 0.);
        source[(2, 3)] = 1.;
        let first = diagram(&source)
            .pairs
            .into_iter()
            .map(|pair| pair.id)
            .collect::<Vec<_>>();
        let second = diagram(&source)
            .pairs
            .into_iter()
            .map(|pair| pair.id)
            .collect::<Vec<_>>();
        assert_eq!(first, second);
    }

    #[test]
    fn cancellation_planning_is_independent_of_candidate_input_order() {
        let grid = DfmGrid::new(11, 11, 1., geo::coord! { x: 0., y: 10. }).unwrap();
        let mut source = Dfm::<Elevation>::new(grid);
        source.field.fill(0.);
        for (y, x) in [(2, 2), (2, 8), (5, 5), (8, 2), (8, 8)] {
            source[(y, x)] = 0.2;
        }
        let mut first_diagram = diagram(&source);
        let mut second_diagram = diagram(&source);
        second_diagram.pairs.reverse();
        let apply = |diagram: &PersistenceDiagram| {
            let mut current = source.clone();
            let mut workspace = PersistenceWorkspace::default();
            let mut work = PersistenceWork::default();
            let plan = plan_bounded_cancellations(
                &source,
                &current,
                diagram,
                ExtremumKind::Maximum,
                0.3,
                0.25,
                &mut workspace,
                &mut work,
            );
            apply_cancellation_plan(&mut current, &plan, &mut work);
            current.field
        };
        assert_eq!(apply(&first_diagram), apply(&second_diagram));
        first_diagram.pairs.reverse();
        assert_eq!(apply(&first_diagram), apply(&second_diagram));
    }

    #[test]
    fn nested_shallow_minimum_is_removed_without_erasing_its_hill() {
        let grid = DfmGrid::new(7, 7, 1., geo::coord! { x: 0., y: 6. }).unwrap();
        let mut source = Dfm::<Elevation>::new(grid);
        source.field.fill(0.);
        for y in 2..=4 {
            for x in 2..=4 {
                source[(y, x)] = 1.;
            }
        }
        source[(3, 3)] = 0.8;

        let (target, summary) = simplify(&source, 0.3, 0.25);
        assert!(summary.removed > 0);
        assert!(summary.protected_features.iter().any(|feature| {
            feature.kind == ExtremumKind::Maximum && feature.persistence >= 0.3
        }));
        assert!(within_bound(&target, &source, 0.25));
        let mut audited = Dfm::<Elevation>::new_like(&source);
        audited.field.copy_from_slice(&target.field);
        assert!(
            diagram(&audited)
                .pairs
                .iter()
                .all(|pair| pair.persistence >= 0.3 || pair.persistence <= PERSISTENCE_EPSILON)
        );
    }

    #[test]
    fn rough_field_has_constant_diagram_budget() {
        for size in [20, 40, 64] {
            let grid = DfmGrid::new(size, size, 1., geo::coord! { x: 0., y: size as f64 }).unwrap();
            let mut source = Dfm::<Elevation>::new(grid);
            for (index, value) in source.field.iter_mut().enumerate() {
                let mixed = (index as u64)
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .rotate_left(17);
                *value = (mixed as u32) as f32 / u32::MAX as f32;
            }
            let (_, summary) = simplify(&source, 0.3, 0.25);
            assert!(summary.work.diagram_builds <= 3);
            assert_eq!(summary.work.cancellation_passes, 2);
            assert!(summary.work.affected_cells_written <= size * size * 2);
        }
    }
}
