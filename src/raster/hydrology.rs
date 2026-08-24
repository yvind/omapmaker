use std::{
    cmp::Ordering,
    collections::{BinaryHeap, VecDeque},
};

use crate::{CELL_SIZE_METERS, TILE_SIZE_PIXELS};

use super::{
    Dfm,
    dfm::{Elevation, FlowAccumulation, HydroCorrected},
};

const NO_FLOW: u8 = u8::MAX;

// Clockwise D8 neighbourhood, starting east. Diagonal distances are handled
// explicitly when selecting the steepest downslope receiver.
const DX: [isize; 8] = [1, 1, 0, -1, -1, -1, 0, 1];
const DY: [isize; 8] = [0, 1, 1, 1, 0, -1, -1, -1];
const D8_DISTANCE: [f32; 8] = [
    1.0,
    std::f32::consts::SQRT_2,
    1.0,
    std::f32::consts::SQRT_2,
    1.0,
    std::f32::consts::SQRT_2,
    1.0,
    std::f32::consts::SQRT_2,
];

/// A depression-free D8 flow field, contributing area, and source-DEM trench
/// evidence.
///
/// Directions are kept together with accumulation because stream vectors need
/// both: accumulation determines where a channel starts and directions connect
/// those cells into a network.
pub struct D8Flow {
    directions: Box<[u8]>,
    accumulation: Dfm<FlowAccumulation>,
    positive_cross_channel_curvature: Box<[bool]>,
}

impl D8Flow {
    /// Contributing catchment area in square metres, including each cell's own
    /// area. Exposed for optional raster export and diagnostics.
    pub fn flow_accumulation(&self) -> &Dfm<FlowAccumulation> {
        &self.accumulation
    }

    /// Convert cells meeting `minimum_catchment_area_m2` into directed stream
    /// reaches. Reaches split at confluences so every D8 edge appears once.
    pub fn stream_lines(&self, minimum_catchment_area_m2: f32) -> geo::MultiLineString {
        if !minimum_catchment_area_m2.is_finite() || minimum_catchment_area_m2 <= 0.0 {
            return geo::MultiLineString::new(Vec::new());
        }

        let stream_cells = self
            .accumulation
            .field
            .iter()
            .zip(&self.positive_cross_channel_curvature)
            .map(|(area, is_trench)| *area >= minimum_catchment_area_m2 && *is_trench)
            .collect::<Vec<_>>();
        let mut stream_inflow = vec![0_u8; stream_cells.len()];

        for index in 0..stream_cells.len() {
            if !stream_cells[index] {
                continue;
            }
            if let Some(receiver) = receiver_index(index, self.directions[index])
                && stream_cells[receiver]
            {
                stream_inflow[receiver] += 1;
            }
        }

        let mut reaches = Vec::new();
        for start in 0..stream_cells.len() {
            if !stream_cells[start] || stream_inflow[start] == 1 {
                continue;
            }

            let Some(mut receiver) = receiver_index(start, self.directions[start]) else {
                continue;
            };
            if !stream_cells[receiver] {
                continue;
            }

            let mut coordinates = vec![self.coord(start)];
            loop {
                coordinates.push(self.coord(receiver));
                if stream_inflow[receiver] != 1 {
                    break;
                }

                let Some(next) = receiver_index(receiver, self.directions[receiver]) else {
                    break;
                };
                if !stream_cells[next] {
                    break;
                }
                receiver = next;
            }

            if coordinates.len() >= 2 {
                reaches.push(geo::LineString::new(coordinates));
            }
        }

        geo::MultiLineString::new(reaches)
    }

    fn coord(&self, index: usize) -> geo::Coord {
        self.accumulation
            .index2coord(index / TILE_SIZE_PIXELS, index % TILE_SIZE_PIXELS)
    }
}

/// Recompute contributing area over a collection of overlapping DFM tiles.
///
/// Only one tile owns a cell shared by adjacent `inner` bounds (the first tile
/// in the supplied order). D8 receivers crossing an inner boundary are linked
/// to the owning tile, so upstream area continues through every downstream
/// tile instead of restarting at the seam.
pub fn accumulate_cross_tile_flow<'a>(
    flows: impl IntoIterator<Item = &'a mut D8Flow>,
) -> crate::Result<()> {
    let mut flows = flows.into_iter().collect::<Vec<_>>();
    if flows.len() < 2 {
        return Ok(());
    }

    let tile_len = TILE_SIZE_PIXELS * TILE_SIZE_PIXELS;
    let graph_len = flows.len() * tile_len;
    let no_receiver = usize::MAX;
    let mut receivers = vec![no_receiver; graph_len];
    let mut inflow_count = vec![0_u8; graph_len];
    let mut owned = vec![false; graph_len];
    let mut owned_count = 0_usize;

    for tile in 0..flows.len() {
        let inner = flows[tile].accumulation.inner;
        for y in inner.top..inner.bottom {
            for x in inner.left..inner.right {
                let local_index = y * TILE_SIZE_PIXELS + x;
                if is_inner_boundary(inner, y, x) {
                    let coordinate = flows[tile].coord(local_index);
                    if owner_of_coordinate(&flows, coordinate) != Some((tile, local_index)) {
                        continue;
                    }
                }

                let node = tile * tile_len + local_index;
                owned[node] = true;
                owned_count += 1;
            }
        }
    }

    for tile in 0..flows.len() {
        for local_index in 0..tile_len {
            let node = tile * tile_len + local_index;
            if !owned[node] {
                continue;
            }

            let direction = flows[tile].directions[local_index];
            let Some(local_receiver) = receiver_index(local_index, direction) else {
                continue;
            };
            let local_receiver_node = tile * tile_len + local_receiver;
            let receiver = if owned[local_receiver_node] {
                Some(local_receiver_node)
            } else {
                let coordinate = flows[tile].coord(local_receiver);
                owner_of_coordinate(&flows, coordinate).map(|(receiver_tile, receiver_index)| {
                    receiver_tile * tile_len + receiver_index
                })
            };

            if let Some(receiver) = receiver {
                receivers[node] = receiver;
                inflow_count[receiver] += 1;
            }
        }
    }

    for node in break_flow_cycles(&mut receivers, &owned, &inflow_count) {
        let tile = node / tile_len;
        let local_index = node % tile_len;
        flows[tile].directions[local_index] = NO_FLOW;
    }
    inflow_count.fill(0);
    for (node, receiver) in receivers.iter().copied().enumerate() {
        if owned[node] && receiver != no_receiver {
            inflow_count[receiver] += 1;
        }
    }

    for flow in &mut flows {
        // Shared inner-edge cells not owned by this tile must remain NoData so
        // merged GeoTIFF output does not average a real value with zero.
        flow.accumulation.field.fill(f32::MIN);
    }
    let cell_area = CELL_SIZE_METERS.powi(2) as f32;
    let mut queue = VecDeque::new();
    for node in 0..graph_len {
        if !owned[node] {
            continue;
        }
        let tile = node / tile_len;
        let local_index = node % tile_len;
        flows[tile].accumulation.field[local_index] = cell_area;
        if inflow_count[node] == 0 {
            queue.push_back(node);
        }
    }

    let mut processed = 0_usize;
    while let Some(node) = queue.pop_front() {
        processed += 1;
        let receiver = receivers[node];
        if receiver == no_receiver {
            continue;
        }

        let tile = node / tile_len;
        let local_index = node % tile_len;
        let receiver_tile = receiver / tile_len;
        let receiver_index = receiver % tile_len;
        let area = flows[tile].accumulation.field[local_index];
        flows[receiver_tile].accumulation.field[receiver_index] += area;
        inflow_count[receiver] -= 1;
        if inflow_count[receiver] == 0 {
            queue.push_back(receiver);
        }
    }

    if processed != owned_count {
        anyhow::bail!(
            "Cross-tile D8 graph remained cyclic after seam correction ({processed}/{owned_count} cells processed)"
        );
    }

    // Copy the two cells adjacent to every cross-tile edge into the neighbour
    // halos. This does not affect accumulation, but lets independently clipped
    // stream reaches meet at the shared boundary without a one-cell gap.
    for node in 0..graph_len {
        if !owned[node] {
            continue;
        }
        let receiver = receivers[node];
        if receiver == no_receiver {
            continue;
        }
        let tile = node / tile_len;
        let receiver_tile = receiver / tile_len;
        if tile == receiver_tile {
            continue;
        }

        let local_index = node % tile_len;
        let receiver_index_in_owner = receiver % tile_len;
        let receiver_area = flows[receiver_tile].accumulation.field[receiver_index_in_owner];
        if let Some(receiver_copy) =
            receiver_index(local_index, flows[tile].directions[local_index])
        {
            flows[tile].accumulation.field[receiver_copy] = receiver_area;
        }

        let source_coordinate = flows[tile].coord(local_index);
        if let Some(source_copy) = index_of_coordinate(flows[receiver_tile], source_coordinate) {
            let source_area = flows[tile].accumulation.field[local_index];
            flows[receiver_tile].accumulation.field[source_copy] = source_area;
        }
    }

    Ok(())
}

fn owner_of_coordinate(flows: &[&mut D8Flow], coordinate: geo::Coord) -> Option<(usize, usize)> {
    flows.iter().enumerate().find_map(|(tile, flow)| {
        let index = index_of_coordinate(flow, coordinate)?;
        let x = index % TILE_SIZE_PIXELS;
        let y = index / TILE_SIZE_PIXELS;
        let inner = flow.accumulation.inner;
        if y < inner.top || y >= inner.bottom || x < inner.left || x >= inner.right {
            return None;
        }
        Some((tile, index))
    })
}

fn index_of_coordinate(flow: &D8Flow, coordinate: geo::Coord) -> Option<usize> {
    let x = ((coordinate.x - flow.accumulation.tl_coord.x) / CELL_SIZE_METERS).round();
    let y = ((flow.accumulation.tl_coord.y - coordinate.y) / CELL_SIZE_METERS).round();
    if x < 0.0 || y < 0.0 || x >= TILE_SIZE_PIXELS as f64 || y >= TILE_SIZE_PIXELS as f64 {
        return None;
    }
    let x = x as usize;
    let y = y as usize;
    let actual = flow.accumulation.index2coord(y, x);
    let tolerance = CELL_SIZE_METERS * 1.0e-6;
    ((actual.x - coordinate.x).abs() <= tolerance && (actual.y - coordinate.y).abs() <= tolerance)
        .then_some(y * TILE_SIZE_PIXELS + x)
}

fn is_inner_boundary(inner: super::dfm::DfmPixelBounds, y: usize, x: usize) -> bool {
    y == inner.top || y + 1 == inner.bottom || x == inner.left || x + 1 == inner.right
}

fn break_flow_cycles(receivers: &mut [usize], owned: &[bool], inflow_count: &[u8]) -> Vec<usize> {
    let no_receiver = usize::MAX;
    let mut remaining_inflow = inflow_count.to_vec();
    let mut queue = VecDeque::new();
    let mut removed = vec![false; receivers.len()];
    for node in 0..receivers.len() {
        if owned[node] && remaining_inflow[node] == 0 {
            queue.push_back(node);
        }
    }
    while let Some(node) = queue.pop_front() {
        removed[node] = true;
        let receiver = receivers[node];
        if receiver != no_receiver {
            remaining_inflow[receiver] -= 1;
            if remaining_inflow[receiver] == 0 {
                queue.push_back(receiver);
            }
        }
    }

    let mut visit_generation = vec![0_usize; receivers.len()];
    let mut generation = 0_usize;
    let mut broken = Vec::new();
    for start in 0..receivers.len() {
        if !owned[start] || removed[start] {
            continue;
        }
        generation += 1;
        let mut node = start;
        while node != no_receiver && owned[node] && !removed[node] {
            if visit_generation[node] == generation {
                receivers[node] = no_receiver;
                broken.push(node);
                break;
            }
            if visit_generation[node] != 0 {
                break;
            }
            visit_generation[node] = generation;
            node = receivers[node];
        }
    }
    broken
}

impl Dfm<Elevation> {
    /// Prepare a DEM for flow analysis.
    ///
    /// This follows the correction sequence used by Whitebox hydrology tools:
    /// breach removable single-cell pits, then use an epsilon Priority-Flood to
    /// resolve every remaining depression and flat. The epsilon gradient is
    /// important because D8 analysis must not run on unresolved flats.
    pub fn hydrological_correction(&self) -> Dfm<HydroCorrected> {
        let breached = breach_single_cell_pits(self);
        priority_flood_fill(&breached)
    }

    /// Run the complete corrected-DEM flow workflow and retain only source-DEM
    /// cells with a positive cross-channel second derivative as stream
    /// candidates. Curvature is measured on `self`, before correction, so the
    /// correction process cannot create false trench evidence.
    #[cfg(test)]
    pub fn hydrological_analysis(&self) -> D8Flow {
        let corrected = self.hydrological_correction();
        self.hydrological_analysis_with_corrected(&corrected)
    }

    /// Run flow analysis with an already corrected copy of this DEM. This is
    /// useful when another feature, such as water-region growth, also needs
    /// the corrected elevations.
    pub fn hydrological_analysis_with_corrected(&self, corrected: &Dfm<HydroCorrected>) -> D8Flow {
        debug_assert_eq!(self.tl_coord, corrected.tl_coord);
        debug_assert_eq!(self.inner, corrected.inner);
        let directions = d8_directions(corrected);
        let accumulation = d8_accumulation(corrected, &directions);
        let positive_cross_channel_curvature = positive_cross_channel_curvature(self, &directions);
        D8Flow {
            directions,
            accumulation,
            positive_cross_channel_curvature,
        }
    }
}

/// Port of WhiteboxTools' BreachSingleCellPits preprocessing step. A pit is
/// connected to any lower second-order neighbour by lowering the intervening
/// cell halfway between the two elevations.
fn breach_single_cell_pits(dem: &Dfm<Elevation>) -> Dfm<Elevation> {
    const DX2: [isize; 16] = [2, 2, 2, 2, 2, 1, 0, -1, -2, -2, -2, -2, -2, -1, 0, 1];
    const DY2: [isize; 16] = [-2, -1, 0, 1, 2, 2, 2, 2, 2, 1, 0, -1, -2, -2, -2, -2];
    const BREACH_DIRECTION: [usize; 16] = [0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 0];

    let mut output = dem.clone();
    for y in 1..TILE_SIZE_PIXELS - 1 {
        for x in 1..TILE_SIZE_PIXELS - 1 {
            let elevation = dem[(y, x)];
            let has_lower_neighbour = (0..8).any(|direction| {
                let ny = (y as isize + DY[direction]) as usize;
                let nx = (x as isize + DX[direction]) as usize;
                dem[(ny, nx)] < elevation
            });
            if has_lower_neighbour {
                continue;
            }

            for second_order in 0..16 {
                let ny = y as isize + DY2[second_order];
                let nx = x as isize + DX2[second_order];
                if !in_grid(ny, nx) {
                    continue;
                }
                let lower = dem[(ny as usize, nx as usize)];
                if lower < elevation {
                    let direction = BREACH_DIRECTION[second_order];
                    let by = (y as isize + DY[direction]) as usize;
                    let bx = (x as isize + DX[direction]) as usize;
                    output[(by, bx)] = output[(by, bx)].min((elevation + lower) * 0.5);
                }
            }
        }
    }
    output
}

/// Priority-Flood depression filling with a one-ULP gradient over filled flats.
/// All raster-edge cells are potential outlets, matching Whitebox's treatment
/// of the exterior NoData region.
fn priority_flood_fill(dem: &Dfm<Elevation>) -> Dfm<HydroCorrected> {
    let mut corrected = Dfm::<HydroCorrected>::new_like(dem);
    corrected.field.clone_from_slice(&dem.field);

    let len = TILE_SIZE_PIXELS * TILE_SIZE_PIXELS;
    let mut visited = vec![false; len];
    let mut queue = BinaryHeap::new();

    for x in 0..TILE_SIZE_PIXELS {
        push_outlet(&corrected, &mut visited, &mut queue, x);
        push_outlet(
            &corrected,
            &mut visited,
            &mut queue,
            (TILE_SIZE_PIXELS - 1) * TILE_SIZE_PIXELS + x,
        );
    }
    for y in 1..TILE_SIZE_PIXELS - 1 {
        push_outlet(&corrected, &mut visited, &mut queue, y * TILE_SIZE_PIXELS);
        push_outlet(
            &corrected,
            &mut visited,
            &mut queue,
            y * TILE_SIZE_PIXELS + TILE_SIZE_PIXELS - 1,
        );
    }

    while let Some(cell) = queue.pop() {
        let y = cell.index / TILE_SIZE_PIXELS;
        let x = cell.index % TILE_SIZE_PIXELS;
        for direction in 0..8 {
            let ny = y as isize + DY[direction];
            let nx = x as isize + DX[direction];
            if !in_grid(ny, nx) {
                continue;
            }
            let neighbour = ny as usize * TILE_SIZE_PIXELS + nx as usize;
            if visited[neighbour] {
                continue;
            }
            visited[neighbour] = true;

            let original = corrected.field[neighbour];
            let elevation = if original <= cell.elevation {
                cell.elevation.next_up()
            } else {
                original
            };
            corrected.field[neighbour] = elevation;
            queue.push(HeapCell {
                elevation,
                index: neighbour,
            });
        }
    }

    corrected
}

fn push_outlet<T>(
    dem: &Dfm<T>,
    visited: &mut [bool],
    queue: &mut BinaryHeap<HeapCell>,
    index: usize,
) {
    if visited[index] {
        return;
    }
    visited[index] = true;
    queue.push(HeapCell {
        elevation: dem.field[index],
        index,
    });
}

fn d8_directions(dem: &Dfm<HydroCorrected>) -> Box<[u8]> {
    let mut directions = vec![NO_FLOW; TILE_SIZE_PIXELS * TILE_SIZE_PIXELS].into_boxed_slice();
    for y in 0..TILE_SIZE_PIXELS {
        for x in 0..TILE_SIZE_PIXELS {
            let elevation = dem[(y, x)];
            let mut best_slope = 0.0_f32;
            let mut best_direction = NO_FLOW;
            for direction in 0..8 {
                let ny = y as isize + DY[direction];
                let nx = x as isize + DX[direction];
                if !in_grid(ny, nx) {
                    continue;
                }
                let drop = elevation - dem[(ny as usize, nx as usize)];
                let slope = drop / D8_DISTANCE[direction];
                if slope > best_slope {
                    best_slope = slope;
                    best_direction = direction as u8;
                }
            }
            directions[y * TILE_SIZE_PIXELS + x] = best_direction;
        }
    }
    directions
}

/// Test for a trench in the profile perpendicular to D8 flow. For cardinal
/// flow the samples are one cell to either side; for diagonal flow they are
/// diagonal samples, so the denominator is twice the squared cell size.
fn positive_cross_channel_curvature(source: &Dfm<Elevation>, directions: &[u8]) -> Box<[bool]> {
    let mut positive = vec![false; TILE_SIZE_PIXELS * TILE_SIZE_PIXELS].into_boxed_slice();
    let cell_size_squared = CELL_SIZE_METERS.powi(2) as f32;

    for y in 1..TILE_SIZE_PIXELS - 1 {
        for x in 1..TILE_SIZE_PIXELS - 1 {
            let index = y * TILE_SIZE_PIXELS + x;
            let flow_direction = directions[index];
            if flow_direction == NO_FLOW {
                continue;
            }

            let left_direction = (flow_direction as usize + 2) % 8;
            let right_direction = (flow_direction as usize + 6) % 8;
            let left_y = (y as isize + DY[left_direction]) as usize;
            let left_x = (x as isize + DX[left_direction]) as usize;
            let right_y = (y as isize + DY[right_direction]) as usize;
            let right_x = (x as isize + DX[right_direction]) as usize;
            let sample_distance_squared = cell_size_squared * D8_DISTANCE[left_direction].powi(2);
            let second_derivative = (source[(left_y, left_x)] - 2.0 * source[(y, x)]
                + source[(right_y, right_x)])
                / sample_distance_squared;

            positive[index] = second_derivative > 0.0;
        }
    }

    positive
}

fn d8_accumulation(dem: &Dfm<HydroCorrected>, directions: &[u8]) -> Dfm<FlowAccumulation> {
    let len = TILE_SIZE_PIXELS * TILE_SIZE_PIXELS;
    let mut inflow_count = vec![0_u8; len];
    for (index, direction) in directions.iter().copied().enumerate() {
        if let Some(receiver) = receiver_index(index, direction) {
            inflow_count[receiver] += 1;
        }
    }

    let mut sources = VecDeque::new();
    for (index, count) in inflow_count.iter().copied().enumerate() {
        if count == 0 {
            sources.push_back(index);
        }
    }

    let cell_area = CELL_SIZE_METERS.powi(2) as f32;
    let mut accumulation = Dfm::<FlowAccumulation>::new_like(dem);
    accumulation.field.fill(cell_area);
    let mut processed = 0;

    while let Some(index) = sources.pop_front() {
        processed += 1;
        if let Some(receiver) = receiver_index(index, directions[index]) {
            accumulation.field[receiver] += accumulation.field[index];
            inflow_count[receiver] -= 1;
            if inflow_count[receiver] == 0 {
                sources.push_back(receiver);
            }
        }
    }

    debug_assert_eq!(processed, len, "corrected D8 field contains a flow cycle");
    accumulation
}

fn receiver_index(index: usize, direction: u8) -> Option<usize> {
    if direction == NO_FLOW {
        return None;
    }
    let y = index / TILE_SIZE_PIXELS;
    let x = index % TILE_SIZE_PIXELS;
    let ny = y as isize + DY[direction as usize];
    let nx = x as isize + DX[direction as usize];
    in_grid(ny, nx).then_some(ny as usize * TILE_SIZE_PIXELS + nx as usize)
}

#[inline]
fn in_grid(y: isize, x: isize) -> bool {
    y >= 0 && x >= 0 && y < TILE_SIZE_PIXELS as isize && x < TILE_SIZE_PIXELS as isize
}

#[derive(Clone, Copy, Debug)]
struct HeapCell {
    elevation: f32,
    index: usize,
}

impl PartialEq for HeapCell {
    fn eq(&self, other: &Self) -> bool {
        self.elevation.to_bits() == other.elevation.to_bits() && self.index == other.index
    }
}

impl Eq for HeapCell {}

impl PartialOrd for HeapCell {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for HeapCell {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse elevation and index ordering to turn BinaryHeap into a stable
        // min-heap. Stable ordering makes flat resolution deterministic.
        other
            .elevation
            .total_cmp(&self.elevation)
            .then_with(|| other.index.cmp(&self.index))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn descending_valley() -> Dfm<Elevation> {
        let mut dem = Dfm::<Elevation>::new(geo::Coord { x: 0.0, y: 0.0 });
        let centre = (TILE_SIZE_PIXELS / 2) as f32;
        for y in 0..TILE_SIZE_PIXELS {
            for x in 0..TILE_SIZE_PIXELS {
                dem[(y, x)] = (x as f32 - centre).abs() + (TILE_SIZE_PIXELS - y) as f32 * 0.01;
            }
        }
        dem
    }

    #[test]
    fn correction_resolves_pits_and_flats_before_flow_analysis() {
        let mut dem = descending_valley();
        let middle = TILE_SIZE_PIXELS / 2;
        dem[(middle, middle)] -= 100.0;
        for y in middle + 20..middle + 25 {
            for x in middle + 20..middle + 25 {
                dem[(y, x)] = 50.0;
            }
        }

        let corrected = dem.hydrological_correction();
        for y in 1..TILE_SIZE_PIXELS - 1 {
            for x in 1..TILE_SIZE_PIXELS - 1 {
                assert!((0..8).any(|direction| {
                    let ny = (y as isize + DY[direction]) as usize;
                    let nx = (x as isize + DX[direction]) as usize;
                    corrected[(ny, nx)] < corrected[(y, x)]
                }));
            }
        }
    }

    #[test]
    fn d8_accumulation_and_stream_extraction_follow_the_valley() {
        let flow = descending_valley().hydrological_analysis();
        let centre = TILE_SIZE_PIXELS / 2;
        let outlet_area = flow.accumulation[(TILE_SIZE_PIXELS - 1, centre)];
        assert!(outlet_area > 0.9 * (TILE_SIZE_PIXELS * TILE_SIZE_PIXELS) as f32 * 0.25);

        let streams = flow.stream_lines(1_000.0);
        assert!(!streams.0.is_empty());
        assert!(streams.0.iter().all(|line| line.0.len() >= 2));
        assert!(streams.0.iter().flatten().any(|coord| {
            (coord.x - centre as f64 * CELL_SIZE_METERS).abs() <= CELL_SIZE_METERS
        }));
    }

    #[test]
    fn planar_slope_is_not_a_stream_without_positive_cross_channel_curvature() {
        let mut dem = Dfm::<Elevation>::new(geo::Coord { x: 0.0, y: 0.0 });
        for y in 0..TILE_SIZE_PIXELS {
            for x in 0..TILE_SIZE_PIXELS {
                dem[(y, x)] = (TILE_SIZE_PIXELS - y) as f32;
            }
        }

        let flow = dem.hydrological_analysis();
        assert!(flow.stream_lines(10.0).0.is_empty());
    }

    #[test]
    fn invalid_stream_threshold_produces_no_network() {
        let flow = descending_valley().hydrological_analysis();
        assert!(flow.stream_lines(0.0).0.is_empty());
        assert!(flow.stream_lines(f32::NAN).0.is_empty());
    }

    #[test]
    fn cross_tile_accumulation_continues_through_the_downstream_tile() {
        const OVERLAP_CELLS: usize = 10;
        const HALF_OVERLAP: usize = OVERLAP_CELLS / 2;

        let mut upstream = Dfm::<Elevation>::new(geo::Coord { x: 0.0, y: 0.0 });
        upstream.inner.right = TILE_SIZE_PIXELS - HALF_OVERLAP;
        let downstream_left = (TILE_SIZE_PIXELS - OVERLAP_CELLS) as f64 * CELL_SIZE_METERS;
        let mut downstream = Dfm::<Elevation>::new(geo::Coord {
            x: downstream_left,
            y: 0.0,
        });
        downstream.inner.left = HALF_OVERLAP;

        for y in 0..TILE_SIZE_PIXELS {
            for x in 0..TILE_SIZE_PIXELS {
                upstream[(y, x)] = -upstream.index2coord(y, x).x as f32;
                downstream[(y, x)] = -downstream.index2coord(y, x).x as f32;
            }
        }

        let mut upstream_flow = upstream.hydrological_analysis();
        let mut downstream_flow = downstream.hydrological_analysis();
        accumulate_cross_tile_flow([&mut upstream_flow, &mut downstream_flow])
            .expect("cross-tile flow graph should be acyclic");

        let row = TILE_SIZE_PIXELS / 2;
        let downstream_outlet = downstream_flow.accumulation[(row, TILE_SIZE_PIXELS - 1)];
        let owned_cells = (TILE_SIZE_PIXELS - HALF_OVERLAP) * 2;
        let expected_area = owned_cells as f32 * CELL_SIZE_METERS.powi(2) as f32;
        assert_eq!(downstream_outlet, expected_area);
    }
}
