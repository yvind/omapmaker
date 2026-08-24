use crate::raster::Dfm;
use crate::raster::dfm::{Elevation, TargetElevation};
use rayon::prelude::*;

#[derive(Clone, Debug)]
pub(super) struct PersistenceSummary {
    pub(super) removed: usize,
    pub(super) preserved: usize,
    pub(super) removed_extrema: Vec<usize>,
    pub(super) preserved_extrema: Vec<usize>,
}

pub(super) fn simplify_bounded(
    original: &Dfm<Elevation>,
    input: &Dfm<Elevation>,
    threshold: f32,
    adjustment_bound: f32,
) -> (Dfm<TargetElevation>, PersistenceSummary) {
    original
        .grid
        .ensure_compatible(&input.grid)
        .expect("persistence cleanup requires matching grids");
    let mut output = Dfm::new_like(original);
    output.field.copy_from_slice(&input.field);
    let mut summary = PersistenceSummary {
        removed: 0,
        preserved: 0,
        removed_extrema: Vec::new(),
        preserved_extrema: Vec::new(),
    };
    simplify_pass(
        original,
        input,
        &mut output,
        threshold,
        adjustment_bound,
        true,
        &mut summary,
    );
    simplify_pass(
        original,
        input,
        &mut output,
        threshold,
        adjustment_bound,
        false,
        &mut summary,
    );
    (output, summary)
}

fn simplify_pass(
    original: &Dfm<Elevation>,
    input: &Dfm<Elevation>,
    output: &mut Dfm<TargetElevation>,
    threshold: f32,
    bound: f32,
    ascending: bool,
    summary: &mut PersistenceSummary,
) {
    let len = input.field.len();
    let width = input.width();
    let height = input.height();
    let mut order = (0..len).collect::<Vec<_>>();
    order.par_sort_unstable_by(|&a, &b| {
        let ordering = input.field[a].total_cmp(&input.field[b]);
        if ascending {
            ordering.then(a.cmp(&b))
        } else {
            ordering.reverse().then(a.cmp(&b))
        }
    });
    let mut active = vec![false; len];
    let mut parent = (0..len).collect::<Vec<_>>();
    let mut size = vec![1_usize; len];
    let mut birth = input.field.to_vec();
    let mut extremum = (0..len).collect::<Vec<_>>();
    let mut head = (0..len).collect::<Vec<_>>();
    let mut tail = head.clone();
    let mut next = vec![usize::MAX; len];

    for index in order {
        active[index] = true;
        let y = index / width;
        let x = index % width;
        let mut roots = Vec::with_capacity(4);
        if x > 0 && active[index - 1] {
            roots.push(find(&mut parent, index - 1));
        }
        if x + 1 < width && active[index + 1] {
            roots.push(find(&mut parent, index + 1));
        }
        if y > 0 && active[index - width] {
            roots.push(find(&mut parent, index - width));
        }
        if y + 1 < height && active[index + width] {
            roots.push(find(&mut parent, index + width));
        }
        roots.sort_unstable();
        roots.dedup();
        if roots.is_empty() {
            continue;
        }
        let survivor = *roots
            .iter()
            .min_by(|&&a, &&b| {
                let ordering = birth[a].total_cmp(&birth[b]);
                if ascending {
                    ordering.then(extremum[a].cmp(&extremum[b]))
                } else {
                    ordering.reverse().then(extremum[a].cmp(&extremum[b]))
                }
            })
            .expect("nonempty neighboring components");
        let survivor_birth = birth[survivor];
        let survivor_extremum = extremum[survivor];
        for &root in &roots {
            if root == survivor {
                continue;
            }
            let persistence = (input.field[index] - birth[root]).abs();
            if persistence < threshold {
                summary.removed += 1;
                summary.removed_extrema.push(extremum[root]);
                let mut cell = head[root];
                loop {
                    output.field[cell] = if ascending {
                        output.field[cell].max(input.field[index].min(original.field[cell] + bound))
                    } else {
                        output.field[cell].min(input.field[index].max(original.field[cell] - bound))
                    };
                    if next[cell] == usize::MAX {
                        break;
                    }
                    cell = next[cell];
                }
            } else {
                summary.preserved += 1;
                summary.preserved_extrema.push(extremum[root]);
            }
        }
        let mut combined = roots[0];
        for &root in &roots[1..] {
            combined = union(
                &mut parent,
                &mut size,
                &mut head,
                &mut tail,
                &mut next,
                combined,
                root,
            );
        }
        combined = union(
            &mut parent,
            &mut size,
            &mut head,
            &mut tail,
            &mut next,
            combined,
            index,
        );
        birth[combined] = survivor_birth;
        extremum[combined] = survivor_extremum;
    }
    let root = find(&mut parent, 0);
    let terminal = if ascending {
        input
            .field
            .iter()
            .copied()
            .max_by(f32::total_cmp)
            .expect("nonempty raster")
    } else {
        input
            .field
            .iter()
            .copied()
            .min_by(f32::total_cmp)
            .expect("nonempty raster")
    };
    if (terminal - birth[root]).abs() < threshold {
        summary.removed += 1;
        summary.removed_extrema.push(extremum[root]);
        let mut cell = head[root];
        loop {
            output.field[cell] = if ascending {
                output.field[cell].max(terminal.min(original.field[cell] + bound))
            } else {
                output.field[cell].min(terminal.max(original.field[cell] - bound))
            };
            if next[cell] == usize::MAX {
                break;
            }
            cell = next[cell];
        }
    } else {
        summary.preserved += 1;
        summary.preserved_extrema.push(extremum[root]);
    }
}

fn find(parent: &mut [usize], mut index: usize) -> usize {
    while parent[index] != index {
        parent[index] = parent[parent[index]];
        index = parent[index];
    }
    index
}

fn union(
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

    #[test]
    fn shallow_extremum_is_removed_within_bound() {
        let grid = DfmGrid::new(5, 5, 1., geo::coord! { x: 0., y: 4. }).unwrap();
        let mut source = Dfm::new(grid);
        source.field.fill(0.);
        source[(2, 2)] = 0.2;
        let (target, summary) = simplify_bounded(&source, &source, 0.3, 0.25);
        assert!(summary.removed > 0);
        assert!(target[(2, 2)].abs() < 1e-6);
        assert!(
            target
                .field
                .iter()
                .zip(&source.field)
                .all(|(a, b)| (a - b).abs() <= 0.25 + 1e-6)
        );
    }

    #[test]
    fn prominent_small_extremum_is_preserved() {
        let grid = DfmGrid::new(5, 5, 1., geo::coord! { x: 0., y: 4. }).unwrap();
        let mut source = Dfm::new(grid);
        source.field.fill(0.);
        source[(2, 2)] = 1.;
        let (target, summary) = simplify_bounded(&source, &source, 0.3, 0.25);
        assert!(summary.preserved > 0);
        assert_eq!(target[(2, 2)], 1.);
    }
}
