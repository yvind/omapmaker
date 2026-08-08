//! Weighted graph used to turn a polygon's medial axis into significant
//! centerline branches.

use std::collections::{HashMap, VecDeque};

use geo::{Coord, Line};

/// Distance within which two numerically near-identical Voronoi vertices are
/// treated as the same graph node.
const MERGE_TOLERANCE: f64 = 1.0e-6;
const STRAIGHTNESS_WEIGHT: f64 = 0.55;
const LENGTH_WEIGHT: f64 = 0.30;
const THICKNESS_WEIGHT: f64 = 0.15;

fn cell(coord: Coord<f64>) -> (i64, i64) {
    (
        (coord.x / MERGE_TOLERANCE).floor() as i64,
        (coord.y / MERGE_TOLERANCE).floor() as i64,
    )
}

fn normalized(value: f64, maximum: f64) -> f64 {
    if maximum > 0.0 { value / maximum } else { 0.0 }
}

fn continuation_straightness(previous: Coord<f64>, junction: Coord<f64>, next: Coord<f64>) -> f64 {
    let incoming = (junction.x - previous.x, junction.y - previous.y);
    let outgoing = (next.x - junction.x, next.y - junction.y);
    let denominator = incoming.0.hypot(incoming.1) * outgoing.0.hypot(outgoing.1);
    if denominator == 0.0 {
        return 0.0;
    }

    let cosine =
        ((incoming.0 * outgoing.0 + incoming.1 * outgoing.1) / denominator).clamp(-1.0, 1.0);
    (cosine + 1.0) / 2.0
}

struct Edge {
    a: usize,
    b: usize,
    weight: f64,
    thickness: f64,
}

impl Edge {
    fn other(&self, node: usize) -> usize {
        if self.a == node { self.b } else { self.a }
    }
}

struct Branch {
    nodes: Vec<usize>,
    length: f64,
    thickness: f64,
}

impl Branch {
    fn touches(&self, node: usize) -> bool {
        self.nodes.first() == Some(&node) || self.nodes.last() == Some(&node)
    }

    fn neighbor_from(&self, node: usize) -> Option<usize> {
        if self.nodes.first() == Some(&node) {
            self.nodes.get(1).copied()
        } else if self.nodes.last() == Some(&node) {
            self.nodes.get(self.nodes.len().checked_sub(2)?).copied()
        } else {
            None
        }
    }

    fn nodes_from(&self, node: usize) -> Option<Vec<usize>> {
        if self.nodes.first() == Some(&node) {
            Some(self.nodes.clone())
        } else if self.nodes.last() == Some(&node) {
            Some(self.nodes.iter().rev().copied().collect())
        } else {
            None
        }
    }
}

#[derive(Default)]
pub(super) struct Graph {
    coords: Vec<Coord<f64>>,
    adjacency: Vec<Vec<usize>>,
    edges: Vec<Edge>,
    cells: HashMap<(i64, i64), Vec<usize>>,
}

impl Graph {
    pub(super) fn new() -> Self {
        Self::default()
    }

    fn node(&mut self, coord: Coord<f64>) -> usize {
        let (cell_x, cell_y) = cell(coord);
        for delta_x in -1..=1 {
            for delta_y in -1..=1 {
                if let Some(ids) = self.cells.get(&(cell_x + delta_x, cell_y + delta_y)) {
                    for &id in ids {
                        let existing = self.coords[id];
                        if (existing.x - coord.x).hypot(existing.y - coord.y) <= MERGE_TOLERANCE {
                            return id;
                        }
                    }
                }
            }
        }

        let id = self.coords.len();
        self.coords.push(coord);
        self.adjacency.push(Vec::new());
        self.cells.entry((cell_x, cell_y)).or_default().push(id);
        id
    }

    /// Adds a Euclidean-distance-weighted edge. Degenerate and duplicate edges
    /// are ignored.
    pub(super) fn add_line(&mut self, line: &Line<f64>, thickness: f64) {
        let a = self.node(line.start);
        let b = self.node(line.end);
        if a == b
            || self.adjacency[a]
                .iter()
                .any(|&edge| self.edges[edge].other(a) == b)
        {
            return;
        }

        let delta = line.delta();
        let edge = self.edges.len();
        self.edges.push(Edge {
            a,
            b,
            weight: delta.x.hypot(delta.y),
            thickness: thickness.max(0.0),
        });
        self.adjacency[a].push(edge);
        self.adjacency[b].push(edge);
    }

    /// Removes short terminal spurs and returns the remaining medial axis as
    /// coherent paths, ordered with the main path first.
    ///
    /// At a junction, the continuation is selected from straightness, branch
    /// length, and branch thickness. Once the main path is removed, the same
    /// selection is repeated for the remaining significant branches. Repeated
    /// pruning is important: after short Voronoi hairs are removed, adjacent
    /// degree-two chains coalesce into the actual centerline. If no path
    /// survives, the source shape is compact rather than line-like and should
    /// not be collapsed.
    pub(super) fn significant_branches(&self, minimum_branch_length: f64) -> Vec<Vec<Coord<f64>>> {
        let Some(component) = self.largest_component() else {
            return vec![];
        };

        let mut in_component = vec![false; self.coords.len()];
        for node in component {
            in_component[node] = true;
        }

        let mut active = self
            .edges
            .iter()
            .map(|edge| in_component[edge.a] && in_component[edge.b])
            .collect::<Vec<_>>();
        self.prune_terminal_spurs(&mut active, minimum_branch_length);

        // A pure degree-two component is a closed centerline. Unlike terminal
        // paths it has no leaf from which pruning can start, so apply the same
        // significance threshold to the cycle as a whole.
        let is_cycle = (0..self.coords.len())
            .filter(|&node| self.active_degree(node, &active) > 0)
            .all(|node| self.active_degree(node, &active) == 2);
        let active_length = self
            .edges
            .iter()
            .zip(&active)
            .filter(|(_, active)| **active)
            .map(|(edge, _)| edge.weight)
            .sum::<f64>();
        if is_cycle && active_length < minimum_branch_length {
            return vec![];
        }

        self.coherent_paths(&active)
    }

    fn prune_terminal_spurs(&self, active: &mut [bool], minimum_branch_length: f64) {
        loop {
            let leaves = (0..self.coords.len())
                .filter(|&node| self.active_degree(node, active) == 1)
                .collect::<Vec<_>>();
            let mut remove = vec![false; self.edges.len()];

            for leaf in leaves {
                let (terminal_path, length) = self.terminal_path(leaf, active);
                if length < minimum_branch_length {
                    for edge in terminal_path {
                        remove[edge] = true;
                    }
                }
            }

            let mut changed = false;
            for (edge, should_remove) in active.iter_mut().zip(remove) {
                if *edge && should_remove {
                    *edge = false;
                    changed = true;
                }
            }
            if !changed {
                break;
            }
        }
    }

    /// Walks from a leaf through degree-two nodes to the next leaf or junction.
    fn terminal_path(&self, start: usize, active: &[bool]) -> (Vec<usize>, f64) {
        let mut edges = Vec::new();
        let mut length = 0.0;
        let mut current = start;
        let mut previous_edge = None;

        while let Some(edge) = self.adjacency[current]
            .iter()
            .copied()
            .find(|&edge| active[edge] && Some(edge) != previous_edge)
        {
            edges.push(edge);
            length += self.edges[edge].weight;
            current = self.edges[edge].other(current);
            previous_edge = Some(edge);

            if self.active_degree(current, active) != 2 {
                break;
            }
        }

        (edges, length)
    }

    fn coherent_paths(&self, active: &[bool]) -> Vec<Vec<Coord<f64>>> {
        let branches = self.maximal_branches(active);
        let mut remaining = vec![true; branches.len()];
        let mut paths = Vec::new();

        while remaining.iter().any(|remaining| *remaining) {
            let maximum_length = branches
                .iter()
                .zip(&remaining)
                .filter(|(_, remaining)| **remaining)
                .map(|(branch, _)| branch.length)
                .fold(0.0, f64::max);
            let maximum_thickness = branches
                .iter()
                .zip(&remaining)
                .filter(|(_, remaining)| **remaining)
                .map(|(branch, _)| branch.thickness)
                .fold(0.0, f64::max);

            let mut seed = None;
            let mut seed_score = f64::NEG_INFINITY;
            for (index, branch) in branches.iter().enumerate() {
                if !remaining[index] {
                    continue;
                }
                let score = LENGTH_WEIGHT * normalized(branch.length, maximum_length)
                    + THICKNESS_WEIGHT * normalized(branch.thickness, maximum_thickness);
                if score > seed_score {
                    seed = Some(index);
                    seed_score = score;
                }
            }

            let Some(seed) = seed else {
                break;
            };
            remaining[seed] = false;
            let mut path = branches[seed].nodes.clone();
            self.extend_path(&branches, &mut remaining, &mut path, true);
            self.extend_path(&branches, &mut remaining, &mut path, false);

            if path.len() >= 2 {
                paths.push(path.into_iter().map(|node| self.coords[node]).collect());
            }
        }

        paths
    }

    fn extend_path(
        &self,
        branches: &[Branch],
        remaining: &mut [bool],
        path: &mut Vec<usize>,
        at_front: bool,
    ) {
        loop {
            let (junction, previous) = if at_front {
                (path[0], path[1])
            } else {
                let end = path.len() - 1;
                (path[end], path[end - 1])
            };
            let candidates = branches
                .iter()
                .enumerate()
                .filter(|(index, branch)| remaining[*index] && branch.touches(junction))
                .collect::<Vec<_>>();
            if candidates.is_empty() {
                break;
            }

            let maximum_length = candidates
                .iter()
                .map(|(_, branch)| branch.length)
                .fold(0.0, f64::max);
            let maximum_thickness = candidates
                .iter()
                .map(|(_, branch)| branch.thickness)
                .fold(0.0, f64::max);
            let mut selected = None;
            let mut selected_score = f64::NEG_INFINITY;

            for (index, branch) in candidates {
                let Some(next) = branch.neighbor_from(junction) else {
                    continue;
                };
                let straightness = continuation_straightness(
                    self.coords[previous],
                    self.coords[junction],
                    self.coords[next],
                );
                let score = STRAIGHTNESS_WEIGHT * straightness
                    + LENGTH_WEIGHT * normalized(branch.length, maximum_length)
                    + THICKNESS_WEIGHT * normalized(branch.thickness, maximum_thickness);
                if score > selected_score {
                    selected = Some(index);
                    selected_score = score;
                }
            }

            let Some(selected) = selected else {
                break;
            };
            remaining[selected] = false;
            let Some(oriented) = branches[selected].nodes_from(junction) else {
                break;
            };

            if at_front {
                let mut extended = oriented.into_iter().skip(1).rev().collect::<Vec<_>>();
                extended.append(path);
                *path = extended;
            } else {
                path.extend(oriented.into_iter().skip(1));
            }
        }
    }

    fn maximal_branches(&self, active: &[bool]) -> Vec<Branch> {
        let mut visited = vec![false; self.edges.len()];
        let mut branches = Vec::new();

        // Trace every path incident to a leaf or junction first.
        for start in 0..self.coords.len() {
            let degree = self.active_degree(start, active);
            if degree == 0 || degree == 2 {
                continue;
            }

            for &edge in &self.adjacency[start] {
                if active[edge] && !visited[edge] {
                    branches.push(self.trace_branch(start, edge, active, &mut visited));
                }
            }
        }

        // Any edges not reached above form degree-two cycles, such as the
        // centerline of a narrow polygon surrounding a hole.
        for edge in 0..self.edges.len() {
            if active[edge] && !visited[edge] {
                branches.push(self.trace_branch(self.edges[edge].a, edge, active, &mut visited));
            }
        }

        branches.retain(|branch| branch.nodes.len() >= 2);
        branches
    }

    fn trace_branch(
        &self,
        start: usize,
        first_edge: usize,
        active: &[bool],
        visited: &mut [bool],
    ) -> Branch {
        let mut nodes = vec![start];
        let mut branch_edges = Vec::new();
        let mut current = start;
        let mut edge = first_edge;

        loop {
            if visited[edge] {
                break;
            }

            visited[edge] = true;
            branch_edges.push(edge);
            current = self.edges[edge].other(current);
            nodes.push(current);

            if self.active_degree(current, active) != 2 {
                break;
            }

            let Some(next_edge) = self.adjacency[current]
                .iter()
                .copied()
                .find(|&candidate| active[candidate] && !visited[candidate])
            else {
                break;
            };
            edge = next_edge;
        }

        let length = branch_edges
            .iter()
            .map(|&edge| self.edges[edge].weight)
            .sum::<f64>();
        let thickness = if length > 0.0 {
            branch_edges
                .iter()
                .map(|&edge| self.edges[edge].thickness * self.edges[edge].weight)
                .sum::<f64>()
                / length
        } else {
            0.0
        };

        Branch {
            nodes,
            length,
            thickness,
        }
    }

    fn active_degree(&self, node: usize, active: &[bool]) -> usize {
        self.adjacency[node]
            .iter()
            .filter(|&&edge| active[edge])
            .count()
    }

    fn largest_component(&self) -> Option<Vec<usize>> {
        let mut seen = vec![false; self.coords.len()];
        let mut largest = None;

        for start in 0..self.coords.len() {
            if seen[start] || self.adjacency[start].is_empty() {
                continue;
            }

            let mut queue = VecDeque::from([start]);
            let mut component = Vec::new();
            seen[start] = true;

            while let Some(node) = queue.pop_front() {
                component.push(node);
                for &edge in &self.adjacency[node] {
                    let neighbor = self.edges[edge].other(node);
                    if !seen[neighbor] {
                        seen[neighbor] = true;
                        queue.push_back(neighbor);
                    }
                }
            }

            if largest
                .as_ref()
                .is_none_or(|best: &Vec<usize>| component.len() > best.len())
            {
                largest = Some(component);
            }
        }

        largest
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo::coord;

    fn line(a: (f64, f64), b: (f64, f64)) -> Line<f64> {
        Line::new(coord! { x: a.0, y: a.1 }, coord! { x: b.0, y: b.1 })
    }

    fn add(graph: &mut Graph, a: (f64, f64), b: (f64, f64), thickness: f64) {
        graph.add_line(&line(a, b), thickness);
    }

    #[test]
    fn joins_the_straightest_branches_into_the_main_line() {
        let mut graph = Graph::new();
        add(&mut graph, (0.0, 0.0), (10.0, 0.0), 2.0);
        add(&mut graph, (10.0, 0.0), (20.0, 0.0), 2.0);
        add(&mut graph, (10.0, 0.0), (10.0, 10.0), 2.0);

        let branches = graph.significant_branches(5.0);

        assert_eq!(branches.len(), 2);
        assert_eq!(branches[0].first().unwrap().x, 0.0);
        assert_eq!(branches[0].last().unwrap().x, 20.0);
    }

    #[test]
    fn uses_branch_length_to_choose_between_equally_straight_continuations() {
        let mut graph = Graph::new();
        add(&mut graph, (-20.0, 0.0), (0.0, 0.0), 2.0);
        add(&mut graph, (0.0, 0.0), (10.0, 10.0), 2.0);
        add(&mut graph, (0.0, 0.0), (15.0, -15.0), 2.0);

        let branches = graph.significant_branches(5.0);

        assert_eq!(branches.len(), 2);
        assert!(branches[0].contains(&coord! { x: 15.0, y: -15.0 }));
        assert!(!branches[0].contains(&coord! { x: 10.0, y: 10.0 }));
    }

    #[test]
    fn uses_branch_thickness_to_break_an_equal_angle_and_length_tie() {
        let mut graph = Graph::new();
        add(&mut graph, (-20.0, 0.0), (0.0, 0.0), 2.0);
        add(&mut graph, (0.0, 0.0), (10.0, 10.0), 4.0);
        add(&mut graph, (0.0, 0.0), (10.0, -10.0), 1.0);

        let branches = graph.significant_branches(5.0);

        assert_eq!(branches.len(), 2);
        assert!(branches[0].contains(&coord! { x: 10.0, y: 10.0 }));
        assert!(!branches[0].contains(&coord! { x: 10.0, y: -10.0 }));
    }

    #[test]
    fn removes_short_spurs_and_coalesces_the_centerline() {
        let mut graph = Graph::new();
        add(&mut graph, (0.0, 0.0), (10.0, 0.0), 2.0);
        add(&mut graph, (10.0, 0.0), (20.0, 0.0), 2.0);
        add(&mut graph, (10.0, 0.0), (10.0, 2.0), 2.0);

        let branches = graph.significant_branches(5.0);

        assert_eq!(branches.len(), 1);
        let mut ends = [
            branches[0].first().unwrap().x,
            branches[0].last().unwrap().x,
        ];
        ends.sort_by(f64::total_cmp);
        assert_eq!(ends, [0.0, 20.0]);
    }

    #[test]
    fn rejects_a_graph_without_a_significant_path() {
        let mut graph = Graph::new();
        add(&mut graph, (0.0, 0.0), (4.0, 0.0), 2.0);

        assert!(graph.significant_branches(5.0).is_empty());
    }

    #[test]
    fn retains_a_degree_two_cycle_as_a_closed_centerline() {
        let mut graph = Graph::new();
        add(&mut graph, (0.0, 0.0), (10.0, 0.0), 2.0);
        add(&mut graph, (10.0, 0.0), (10.0, 10.0), 2.0);
        add(&mut graph, (10.0, 10.0), (0.0, 10.0), 2.0);
        add(&mut graph, (0.0, 10.0), (0.0, 0.0), 2.0);

        let branches = graph.significant_branches(5.0);

        assert_eq!(branches.len(), 1);
        assert_eq!(branches[0].first(), branches[0].last());
    }

    #[test]
    fn rejects_a_short_degree_two_cycle() {
        let mut graph = Graph::new();
        add(&mut graph, (0.0, 0.0), (1.0, 0.0), 2.0);
        add(&mut graph, (1.0, 0.0), (1.0, 1.0), 2.0);
        add(&mut graph, (1.0, 1.0), (0.0, 1.0), 2.0);
        add(&mut graph, (0.0, 1.0), (0.0, 0.0), 2.0);

        assert!(graph.significant_branches(5.0).is_empty());
    }
}
