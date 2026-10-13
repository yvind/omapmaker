use std::collections::HashMap;

use crate::raster::{Dfm, Elevation};

use super::{BoulderEvidence, Rejection};

const LAYER_THICKNESS_M: f64 = 0.5;
const LAYERS: usize = 6;

/// Keep the full vertical return profile while checking a low candidate.
/// A dense tree skirt can stop beams locally, but remains attached to foliage
/// extending upwards. A separate overhead crown with a gap is allowed.
pub(super) fn check(
    heights: &HashMap<usize, f64>,
    height: f64,
    evidence: &BoulderEvidence,
    dem: &Dfm<Elevation>,
) -> Result<(), Rejection> {
    let (left, right, top, bottom) = heights.keys().fold(
        (usize::MAX, 0, usize::MAX, 0),
        |(left, right, top, bottom), &cell| {
            let (x, y) = (cell % dem.width(), cell / dem.width());
            (left.min(x), right.max(x), top.min(y), bottom.max(y))
        },
    );
    if right <= left + 1 || bottom <= top + 1 {
        return Ok(());
    }
    let mut occupied_cells = [0_usize; LAYERS];
    let mut maximum = height;
    let mut checked_cells = 0;
    for y in top + 1..bottom {
        for x in left + 1..right {
            let cell = y * dem.width() + x;
            let mut occupied = [false; LAYERS];
            checked_cells += 1;
            for &index in evidence.points_in_cell(cell) {
                // Includes first, intermediate and last echoes, and heights
                // above the boulder ceiling. Removing these hides the tree.
                let z = f64::from(evidence.heights_above_ground[index]);
                maximum = maximum.max(z);
                let above = z - height - LAYER_THICKNESS_M;
                if above >= 0. && above < LAYERS as f64 * LAYER_THICKNESS_M {
                    occupied[(above / LAYER_THICKNESS_M) as usize] = true;
                }
            }
            for (count, occupied) in occupied_cells.iter_mut().zip(occupied) {
                *count += usize::from(occupied);
            }
        }
    }
    let required_cells = ((checked_cells as f64 * 0.25).ceil() as usize).max(1);
    let dense_layers = occupied_cells
        .iter()
        .filter(|&&count| count >= required_cells)
        .count();
    // Require foliage to start near the top and continue for several layers.
    // A handful of branches, or a crown separated by clear air, is insufficient.
    if maximum >= height + 2.5 && occupied_cells[0] >= required_cells && dense_layers >= 4 {
        return Err(Rejection::TreeStructure);
    }
    Ok(())
}
