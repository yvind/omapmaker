use std::collections::HashMap;

use crate::{
    parameters::BoulderParameters,
    raster::{Dfm, Elevation},
};

use super::Rejection;

pub(super) struct SurfaceShape {
    pub(super) top_area: f64,
    pub(super) top_fraction: f64,
    pub(super) roughness: Option<f64>,
    pub(super) dome_fraction: f64,
}

/// Recover broad, locally coherent surfaces when branches or brush have
/// joined their footprint. Thin connections and isolated upper echoes cannot
/// become a core. Use only observed cells; no holes are filled.
pub(super) fn coherent_cores(
    cells: &[usize],
    response: &[f32],
    dem: &Dfm<Elevation>,
    minimum_height: f32,
) -> Vec<Vec<usize>> {
    let footprint: std::collections::HashSet<_> = cells.iter().copied().collect();
    let mut core = std::collections::HashSet::new();
    for &cell in cells {
        if response[cell] < minimum_height {
            continue;
        }
        let (x, y) = (cell % dem.width(), cell / dem.width());
        let mut neighbors = 0;
        for row in y.saturating_sub(1)..=(y + 1).min(dem.height() - 1) {
            for column in x.saturating_sub(1)..=(x + 1).min(dem.width() - 1) {
                let other = row * dem.width() + column;
                if other != cell
                    && footprint.contains(&other)
                    && (response[other] - response[cell]).abs() <= 0.5
                {
                    neighbors += 1;
                }
            }
        }
        if neighbors >= 5 {
            core.insert(cell);
        }
    }
    let mut components = Vec::new();
    // Source order is stable, unlike HashSet iteration.
    for &seed in cells {
        if !core.remove(&seed) {
            continue;
        }
        let mut component = vec![seed];
        let mut cursor = 0;
        while cursor < component.len() {
            let cell = component[cursor];
            let (x, y) = (cell % dem.width(), cell / dem.width());
            for row in y.saturating_sub(1)..=(y + 1).min(dem.height() - 1) {
                for column in x.saturating_sub(1)..=(x + 1).min(dem.width() - 1) {
                    let other = row * dem.width() + column;
                    if core.contains(&other) && (response[other] - response[cell]).abs() <= 0.5 {
                        core.remove(&other);
                        component.push(other);
                    }
                }
            }
            cursor += 1;
        }
        if component.len() >= 4 {
            components.push(component);
        }
    }
    components
}

pub(super) fn supported_peak(heights: &HashMap<usize, f64>, dem: &Dfm<Elevation>) -> Option<f64> {
    let reach = (2. * dem.grid.cell_size_m).max(1.);
    heights
        .iter()
        .filter_map(|(&cell, &height)| {
            let coordinate = dem.index2coord(cell / dem.width(), cell % dem.width());
            heights
                .iter()
                .any(|(&neighbor, &neighbor_height)| {
                    if neighbor == cell || neighbor_height < 0.65 * height {
                        return false;
                    }
                    let p = dem.index2coord(neighbor / dem.width(), neighbor % dem.width());
                    (p.x - coordinate.x).hypot(p.y - coordinate.y) <= reach
                })
                .then_some(height)
        })
        .max_by(f64::total_cmp)
}

/// Use observed surface heights, without fitting or smoothing the boulder.
/// Broad upper support rejects an apex; changing local curvature rejects
/// foliage texture while retaining smoothly rounded, non-planar surfaces.
pub(super) fn shape(
    heights: &HashMap<usize, f64>,
    dem: &Dfm<Elevation>,
    height: f64,
    params: &BoulderParameters,
) -> Result<SurfaceShape, Rejection> {
    // Equal cell weighting prevents a densely sampled tree tip from looking
    // broad merely because it contains many echoes.
    let top_cells = heights.values().filter(|&&z| z >= 0.75 * height).count();
    let top_area = top_cells as f64 * dem.grid.cell_size_m.powi(2);
    let top_fraction = top_cells as f64 / heights.len() as f64;
    if (top_fraction as f32) < params.minimum_top_fraction {
        return Err(Rejection::Pointed);
    }
    let mut curvatures = [Vec::new(), Vec::new(), Vec::new(), Vec::new()];
    let mut checked_dome_cells = 0;
    let mut dome_cells = 0;
    // Express changes at the standard 0.5 m sampling distance even when the
    // source grid uses another resolution. Constant slope cancels directly.
    let normalization = (crate::STANDARD_CELL_SIZE_METERS / dem.grid.cell_size_m).powi(2);
    for (&cell, &z) in heights {
        if z < 0.7 * height {
            continue;
        }
        let x = cell % dem.width();
        let y = cell / dem.width();
        let horizontal = x > 0 && x + 1 < dem.width();
        let vertical = y > 0 && y + 1 < dem.height();
        if horizontal && vertical {
            let cross = [cell - 1, cell + 1, cell - dem.width(), cell + dem.width()];
            if let [Some(&west), Some(&east), Some(&north), Some(&south)] =
                cross.map(|cell| heights.get(&cell))
            {
                checked_dome_cells += 1;
                // Convex in both axes across the top distinguishes a dome
                // from roof planes or a narrow ridge. Ignore tiny noise.
                let curved_x = (2. * z - west - east) * normalization;
                let curved_y = (2. * z - north - south) * normalization;
                dome_cells += usize::from(curved_x > 0.03 && curved_y > 0.03);
            }
        }
        // Diagonal profiles retain shape evidence when sparse sampling leaves
        // holes in horizontal or vertical triples.
        for (axis, step, interior, distance_squared) in [
            (0, 1, horizontal, 1.),
            (1, dem.width(), vertical, 1.),
            (2, dem.width() + 1, horizontal && vertical, 2.),
            (3, dem.width() - 1, horizontal && vertical, 2.),
        ] {
            if !interior {
                continue;
            }
            let (Some(&a), Some(&b)) = (heights.get(&(cell - step)), heights.get(&(cell + step)))
            else {
                continue;
            };
            // Vertical runoffs belong to the side check, not top texture.
            if a >= 0.5 * height && b >= 0.5 * height {
                curvatures[axis].push((a - 2. * z + b) * normalization / distance_squared);
            }
        }
    }
    let mut deviations = Vec::new();
    for values in &mut curvatures {
        if values.is_empty() {
            continue;
        }
        values.sort_unstable_by(f64::total_cmp);
        let median = percentile(values, 0.5);
        deviations.extend(values.iter().map(|value| (value - median).abs()));
    }
    deviations.sort_unstable_by(f64::total_cmp);
    // Sampling holes can leave no three aligned surface cells. That makes
    // roughness unknown, rather than proving the observed object is foliage.
    let roughness = (!deviations.is_empty()).then(|| percentile(&deviations, 0.75));
    if roughness.is_some_and(|roughness| roughness > f64::from(params.maximum_surface_roughness_m))
    {
        return Err(Rejection::Roughness);
    }
    Ok(SurfaceShape {
        top_area,
        top_fraction,
        roughness,
        dome_fraction: if checked_dome_cells >= 4 {
            dome_cells as f64 / checked_dome_cells as f64
        } else {
            0.
        },
    })
}

fn percentile(values: &[f64], fraction: f64) -> f64 {
    values[((values.len() - 1) as f64 * fraction).round() as usize]
}
