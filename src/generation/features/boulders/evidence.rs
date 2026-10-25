use crate::{
    geometry::{PointCloud, PointLaz},
    parameters::BoulderParameters,
    raster::{Dfm, Elevation},
};

use super::opacity::SURFACE_TOLERANCE_M;
use las::point::Classification;

/// Threshold-independent evidence. The source cloud stays on PreparedTile;
/// this compact cell index contains references into it, not copied points.
pub(crate) struct BoulderEvidence {
    layer_offsets: Box<[usize]>,
    pub(super) layer_heights: Box<[f32]>,
    layer_nonterminal_fractions: Box<[f32]>,
    layer_ground_penetration_fractions: Box<[f32]>,
    offsets: Box<[usize]>,
    point_indices: Box<[usize]>,
    pub(super) heights_above_ground: Box<[f32]>,
}

pub(crate) fn compute_boulder_evidence(
    cloud: &PointCloud,
    dem: &Dfm<Elevation>,
) -> BoulderEvidence {
    let mut offsets = vec![0; dem.field.len() + 1];
    let mut heights_above_ground = vec![f32::NAN; cloud.points.len()];
    for (index, point) in cloud.points.iter().enumerate() {
        if usable(point)
            && let Some(cell) = point_cell(point, dem)
            && let Some(ground) = ground_at(dem, geo::coord! { x: point.x(), y: point.y() })
        {
            heights_above_ground[index] = (point.0.z - f64::from(ground)) as f32;
            offsets[cell + 1] += 1;
        }
    }
    for index in 1..offsets.len() {
        offsets[index] += offsets[index - 1];
    }
    let mut cursors = offsets[..dem.field.len()].to_vec();
    let mut point_indices = vec![0; offsets[dem.field.len()]];
    for (index, point) in cloud.points.iter().enumerate() {
        if usable(point)
            && let Some(cell) = point_cell(point, dem)
            && heights_above_ground[index].is_finite()
        {
            point_indices[cursors[cell]] = index;
            cursors[cell] += 1;
        }
    }

    // Overlapping files can repeat the same observation. Deduplicate before
    // surface percentiles and opacity, not just before final point support.
    let mut unique_indices = Vec::with_capacity(point_indices.len());
    let mut unique_offsets = vec![0; offsets.len()];
    for cell in 0..dem.field.len() {
        let indices = &mut point_indices[offsets[cell]..offsets[cell + 1]];
        indices.sort_unstable_by(|&a, &b| {
            let (a, b) = (&cloud.points[a].0, &cloud.points[b].0);
            a.x.total_cmp(&b.x)
                .then(a.y.total_cmp(&b.y))
                .then(a.z.total_cmp(&b.z))
                .then(a.return_number.cmp(&b.return_number))
                .then(a.number_of_returns.cmp(&b.number_of_returns))
                .then(u8::from(a.classification).cmp(&u8::from(b.classification)))
        });
        let mut previous = None;
        for &index in indices.iter() {
            let point = &cloud.points[index].0;
            let key = (
                point.x,
                point.y,
                point.z,
                point.return_number,
                point.number_of_returns,
                point.classification,
            );
            if previous != Some(key) {
                unique_indices.push(index);
                previous = Some(key);
            }
        }
        unique_offsets[cell + 1] = unique_indices.len();
    }
    let offsets = unique_offsets;
    let point_indices = unique_indices;

    let mut layer_offsets = vec![0; dem.field.len() + 1];
    let mut layer_heights = Vec::new();
    let mut layer_nonterminal_fractions = Vec::new();
    let mut layer_ground_penetration_fractions = Vec::new();
    let mut heights = Vec::new();
    for cell in 0..dem.field.len() {
        heights.clear();
        for &index in &point_indices[offsets[cell]..offsets[cell + 1]] {
            let point = &cloud.points[index];
            // First/intermediate/invalid returns never create a surface,
            // provide object support, choose height, or choose placement.
            if is_last(point) && object_class(point.0.classification) {
                heights.push(heights_above_ground[index]);
            }
        }
        heights.sort_unstable_by(f32::total_cmp);
        // Keep vertically disconnected surfaces separate. The lower opaque
        // object must survive even when many last echoes hit canopy above it.
        let mut start = 0;
        for end in 1..=heights.len() {
            if end == heights.len() || heights[end] - heights[end - 1] >= 1. {
                let layer = &heights[start..end];
                let height = layer[((layer.len() - 1) as f64 * 0.98).round() as usize];
                let mut terminal = 0;
                let mut nonterminal = 0;
                let mut last_at_or_below = 0;
                let mut ground_below = 0;
                for &index in &point_indices[offsets[cell]..offsets[cell + 1]] {
                    let point = &cloud.points[index];
                    let residual = f64::from(heights_above_ground[index]) - f64::from(height);
                    if is_last(point) && residual <= SURFACE_TOLERANCE_M {
                        last_at_or_below += 1;
                        ground_below += usize::from(
                            residual < -SURFACE_TOLERANCE_M
                                && point.0.classification == las::point::Classification::Ground,
                        );
                    }
                    if residual.abs() <= SURFACE_TOLERANCE_M {
                        if is_last(point) && object_class(point.0.classification) {
                            terminal += 1;
                        } else {
                            nonterminal += 1;
                        }
                    }
                }
                layer_heights.push(height);
                // Ground observed beneath a crown is direct penetration
                // evidence. Remove it before it can merge into an adjacent
                // rock; non-ground side echoes are checked at object scale.
                layer_ground_penetration_fractions
                    .push(ground_below as f32 / last_at_or_below.max(1) as f32);
                // Sparse cells cannot reliably classify a crown alone. Their
                // return ratios are checked again over the whole object.
                layer_nonterminal_fractions.push(if terminal + nonterminal >= 4 {
                    nonterminal as f32 / (terminal + nonterminal) as f32
                } else {
                    0.
                });
                start = end;
            }
        }
        layer_offsets[cell + 1] = layer_heights.len();
    }
    BoulderEvidence {
        layer_offsets: layer_offsets.into_boxed_slice(),
        layer_heights: layer_heights.into_boxed_slice(),
        layer_nonterminal_fractions: layer_nonterminal_fractions.into_boxed_slice(),
        layer_ground_penetration_fractions: layer_ground_penetration_fractions.into_boxed_slice(),
        offsets: offsets.into_boxed_slice(),
        point_indices: point_indices.into_boxed_slice(),
        heights_above_ground: heights_above_ground.into_boxed_slice(),
    }
}

impl BoulderEvidence {
    pub(super) fn points_in_cell(&self, cell: usize) -> &[usize] {
        &self.point_indices[self.offsets[cell]..self.offsets[cell + 1]]
    }

    pub(super) fn filtered_surface(&self, minimum: f32, params: &BoulderParameters) -> Vec<f32> {
        self.layer_offsets
            .windows(2)
            .map(|bounds| {
                (bounds[0]..bounds[1])
                    .filter(|&layer| {
                        self.layer_nonterminal_fractions[layer]
                            <= params.maximum_nonterminal_fraction
                            && self.layer_ground_penetration_fractions[layer]
                                <= params.maximum_under_surface_fraction
                    })
                    .map(|layer| self.layer_heights[layer])
                    .find(|&height| height.is_finite() && height >= minimum)
                    .unwrap_or(0.)
            })
            .collect()
    }
}

/// Never interpret the missing-data sentinel as an extremely deep ground.
pub(super) fn ground_at(dem: &Dfm<Elevation>, coordinate: geo::Coord) -> Option<f32> {
    let x = (coordinate.x - dem.grid.top_left.x) / dem.grid.cell_size_m;
    let y = (dem.grid.top_left.y - coordinate.y) / dem.grid.cell_size_m;
    if x < 0. || y < 0. || x > (dem.width() - 1) as f64 || y > (dem.height() - 1) as f64 {
        return None;
    }
    let left = x.floor() as usize;
    let top = y.floor() as usize;
    for row in top..=(top + 1).min(dem.height() - 1) {
        for column in left..=(left + 1).min(dem.width() - 1) {
            let value = dem[(row, column)];
            if !value.is_finite() || value == f32::MIN {
                return None;
            }
        }
    }
    dem.sample_bilinear(coordinate)
}

pub(super) fn is_last(point: &PointLaz) -> bool {
    point.0.number_of_returns > 0 && point.0.return_number == point.0.number_of_returns
}

fn usable(point: &PointLaz) -> bool {
    point.x().is_finite()
        && point.y().is_finite()
        && point.0.z.is_finite()
        && !point.0.is_withheld
        && point.0.return_number > 0
        && point.0.return_number <= point.0.number_of_returns
        && !matches!(
            point.0.classification,
            Classification::LowPoint | Classification::HighNoise
        )
}

pub(super) fn object_class(class: Classification) -> bool {
    // Vegetation labels are not a physical opacity test: rocks can be
    // mislabeled. Return termination, penetration and crown shape decide.
    !matches!(
        class,
        Classification::Ground | Classification::Building | Classification::Water
    )
}

fn point_cell(point: &PointLaz, dem: &Dfm<Elevation>) -> Option<usize> {
    let x = ((point.x() - dem.grid.top_left.x) / dem.grid.cell_size_m).round() as isize;
    let y = ((dem.grid.top_left.y - point.y()) / dem.grid.cell_size_m).round() as isize;
    if x < 0 || y < 0 || x >= dem.width() as isize || y >= dem.height() as isize {
        None
    } else {
        Some(y as usize * dem.width() + x as usize)
    }
}

pub(super) fn vegetation_class(class: Classification) -> bool {
    matches!(
        class,
        Classification::LowVegetation
            | Classification::MediumVegetation
            | Classification::HighVegetation
    )
}
