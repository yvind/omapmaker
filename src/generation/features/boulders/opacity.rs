use std::collections::HashMap;

use las::point::Classification;

use crate::{
    geometry::PointCloud,
    parameters::BoulderParameters,
    raster::{Dfm, Elevation},
};

use super::evidence::{is_last, object_class};
use super::{BoulderEvidence, Rejection};

pub(super) const SURFACE_TOLERANCE_M: f64 = 0.5;

pub(super) struct Opacity {
    pub(super) under_surface_fraction: f64,
    pub(super) nonterminal_fraction: f64,
}

/// Test the selected surface, allowing any separate canopy layer above it.
/// A last echo can terminate on a rock even after earlier canopy echoes.
/// Equal cell weights keep densely sampled tops from hiding porous patches.
pub(super) fn check(
    heights: &HashMap<usize, f64>,
    evidence: &BoulderEvidence,
    cloud: &PointCloud,
    dem: &Dfm<Elevation>,
    params: &BoulderParameters,
) -> Result<Opacity, Rejection> {
    let mut nonterminal_sum = 0.;
    let mut under_surface_sum = 0.;
    let mut interior_cells = 0;
    let (left, right, top, bottom) = heights.keys().fold(
        (usize::MAX, 0, usize::MAX, 0),
        |(left, right, top, bottom), &cell| {
            let (x, y) = (cell % dem.width(), cell / dem.width());
            (left.min(x), right.max(x), top.min(y), bottom.max(y))
        },
    );
    // Check beneath the footprint interior, including sampling holes. Boundary
    // returns can lie on exposed vertical sides and are not penetration.
    for y in top..=bottom {
        for x in left..=right {
            let cell = y * dem.width() + x;
            let interior = x > left && x < right && y > top && y < bottom;
            let observed_surface = heights.get(&cell).copied();
            // Interpolate only for the penetration check. A hole contributes
            // no area or object support, but observed ground in it proves the
            // surrounding apparent surface is not solid.
            let surface = if let Some(surface) = observed_surface {
                surface
            } else if interior {
                let mut neighbors = Vec::new();
                for row in y - 1..=y + 1 {
                    for column in x - 1..=x + 1 {
                        if let Some(&height) = heights.get(&(row * dem.width() + column)) {
                            neighbors.push(height);
                        }
                    }
                }
                if neighbors.len() < 3 {
                    continue;
                }
                neighbors.sort_unstable_by(f64::total_cmp);
                neighbors[neighbors.len() / 2]
            } else {
                continue;
            };
            let mut terminal = 0;
            let mut nonterminal = 0;
            let mut below = 0;
            let mut observations = 0;
            for &index in evidence.points_in_cell(cell) {
                let point = &cloud.points[index];
                let height = f64::from(evidence.heights_above_ground[index]);
                if height > surface + SURFACE_TOLERANCE_M {
                    continue;
                }
                if matches!(
                    point.0.classification,
                    Classification::Building | Classification::Water
                ) {
                    return Err(Rejection::Context);
                }
                if (height - surface).abs() <= SURFACE_TOLERANCE_M {
                    if is_last(point) && object_class(point.0.classification) {
                        terminal += 1;
                    } else {
                        nonterminal += 1;
                    }
                }
                if is_last(point) {
                    observations += 1;
                    below += usize::from(height < surface - SURFACE_TOLERANCE_M);
                }
            }
            if observed_surface.is_some() {
                nonterminal_sum += nonterminal as f64 / (terminal + nonterminal).max(1) as f64;
            }
            if interior && observations > 0 {
                interior_cells += 1;
                under_surface_sum += below as f64 / observations as f64;
            }
        }
    }
    if interior_cells == 0 {
        return Err(Rejection::Support);
    }
    let opacity = Opacity {
        under_surface_fraction: under_surface_sum / interior_cells as f64,
        nonterminal_fraction: nonterminal_sum / heights.len() as f64,
    };
    if opacity.under_surface_fraction > f64::from(params.maximum_under_surface_fraction)
        || opacity.nonterminal_fraction > f64::from(params.maximum_nonterminal_fraction)
    {
        return Err(Rejection::Opacity);
    }
    Ok(opacity)
}
