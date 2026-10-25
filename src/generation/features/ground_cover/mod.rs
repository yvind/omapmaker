use las::point::Classification;

use super::{
    surface_features::{local_sum, point_cell_and_height},
    vegetation::{HeightBand, height_band},
};

use crate::{
    geometry::PointCloud,
    raster::{BoulderField, Dfm, Elevation, GroundRelief2m, Undergrowth},
};

/// Experimental near-ground evidence, shared by preview and export. These are
/// scores, not calibrated probabilities. Canopy returns above 2 m are excluded
/// from the denominator so rocks can survive underneath trees.
pub struct GroundCoverRasters {
    pub undergrowth: Dfm<Undergrowth>,
    pub boulder_field: Dfm<BoulderField>,
}

impl GroundCoverRasters {
    pub fn new_like(dem: &Dfm<Elevation>) -> Self {
        let mut undergrowth = Dfm::new_like(dem);
        let mut boulder_field = Dfm::new_like(dem);
        undergrowth.field.fill(0.);
        boulder_field.field.fill(0.);
        Self {
            undergrowth,
            boulder_field,
        }
    }
}

pub fn compute_ground_cover(
    cloud: &PointCloud,
    dem: &Dfm<Elevation>,
    relief: &Dfm<GroundRelief2m>,
) -> GroundCoverRasters {
    let mut result = GroundCoverRasters::new_like(dem);
    for (i, sample) in evidence(cloud, dem, relief).into_iter().enumerate() {
        let (undergrowth, rocks) = scores(sample);
        result.undergrowth.field[i] = undergrowth;
        result.boulder_field.field[i] = rocks;
    }
    result
}

fn ramp(value: f32, start: f32, end: f32) -> f32 {
    ((value - start) / (end - start)).clamp(0., 1.)
}

fn scores(s: Evidence) -> (f32, f32) {
    if s.near < 12. || s.ground < 4. || !s.roughness.is_finite() {
        return (0., 0.);
    }
    let support = (s.near / 20.).min(1.);
    let undergrowth_fraction = s.undergrowth_hits / s.near;
    let penetration_fraction =
        (s.undergrowth_nonterminal / s.undergrowth_hits.max(1.)).clamp(0., 1.);
    // Coherent terminal patches provide solid-surface evidence; broad rough
    // terrain alone (including a cliff edge) must never create a boulder field.
    let rocks = if s.roughness >= 0.05 && s.coherent >= 0.06 && undergrowth_fraction >= 0.08 {
        support
            * (0.45 * ramp(s.roughness, 0.06, 0.20) + 0.55 * ramp(s.coherent, 0.08, 0.23))
            * (1. - penetration_fraction)
    } else {
        0.
    };
    // A narrow, opaque surface is more likely rock than penetrable vegetation.
    // Retain ambiguous evidence: both scores may be high, and the cartographic
    // resolver then keeps 211 and converts 409 to a green shade.
    let solid_fraction = (s.coherent / undergrowth_fraction.max(0.01)).clamp(0., 1.);
    let nonground_undergrowth_fraction =
        (s.undergrowth_hits - s.ground_in_undergrowth_band).max(0.) / s.near;
    let undergrowth = support
        * ramp(nonground_undergrowth_fraction, 0.04, 0.24)
        * (1. - 0.9 * solid_fraction * (1. - penetration_fraction))
        * (1. - 0.85 * ramp(s.roughness, 0.14, 0.30));
    (undergrowth.clamp(0., 1.), rocks.clamp(0., 1.))
}

#[derive(Clone, Copy, Default, Debug)]
struct Evidence {
    near: f32,
    undergrowth_hits: f32,
    undergrowth_nonterminal: f32,
    #[cfg(test)]
    undergrowth_terminal: f32,
    ground_in_undergrowth_band: f32,
    #[cfg(test)]
    higher_near_ground_hits: f32,
    ground: f32,
    roughness: f32,
    elevated: f32,
    elevated_sum: f32,
    elevated_sq: f32,
    coherent: f32,
}

fn evidence(
    cloud: &PointCloud,
    dem: &Dfm<Elevation>,
    relief: &Dfm<GroundRelief2m>,
) -> Vec<Evidence> {
    let n = dem.field.len();
    let mut counts = vec![Evidence::default(); n];
    for p in &cloud.points {
        if p.0.is_withheld
            || p.0.is_synthetic
            || matches!(
                p.0.classification,
                Classification::LowPoint
                    | Classification::HighNoise
                    | Classification::Water
                    | Classification::Building
            )
        {
            continue;
        }
        let Some((i, h)) = point_cell_and_height(p, dem) else {
            continue;
        };
        let nr = p.0.number_of_returns;
        let rn = p.0.return_number;
        if nr == 0 || rn == 0 || rn > nr {
            continue;
        }
        let s = &mut counts[i];
        if (-0.2..=2.).contains(&h) {
            s.near += 1.;
        }
        if (-0.2..0.2).contains(&h) {
            s.ground += 1.;
        }
        if height_band(h) == Some(HeightBand::Undergrowth) {
            s.undergrowth_hits += 1.;
            if p.0.classification == Classification::Ground {
                s.ground_in_undergrowth_band += 1.;
            }
            if rn < nr {
                s.undergrowth_nonterminal += 1.;
            }
            #[cfg(test)]
            if rn == nr {
                s.undergrowth_terminal += 1.;
            }
        }
        #[cfg(test)]
        if (0.8..=2.).contains(&h) {
            s.higher_near_ground_hits += 1.;
        }
        if (0.2..=2.).contains(&h) && rn == nr {
            s.elevated += 1.;
            s.elevated_sum += h;
            s.elevated_sq += h * h;
        }
    }
    // Estimate the vertical scatter over a metre, then weight coherent patches
    // by their return count rather than acquisition-dependent cell density.
    let mut coherent_cells = vec![0.; n];
    let patch_cells = (1. / dem.grid.cell_size_m).round().max(1.) as usize;
    for y in (0..dem.height()).step_by(patch_cells) {
        for x in (0..dem.width()).step_by(patch_cells) {
            let mut count = 0.;
            let mut sum = 0.;
            let mut sq = 0.;
            for yy in y..(y + patch_cells).min(dem.height()) {
                for xx in x..(x + patch_cells).min(dem.width()) {
                    let s = counts[yy * dem.width() + xx];
                    count += s.elevated;
                    sum += s.elevated_sum;
                    sq += s.elevated_sq;
                }
            }
            if count < 3. || (sq / count - (sum / count).powi(2)).max(0.).sqrt() > 0.12 {
                continue;
            }
            for yy in y..(y + patch_cells).min(dem.height()) {
                for xx in x..(x + patch_cells).min(dem.width()) {
                    let i = yy * dem.width() + xx;
                    coherent_cells[i] = counts[i].elevated;
                }
            }
        }
    }
    let radius = (3. / dem.grid.cell_size_m).ceil() as usize;
    let sum = |values: Vec<f32>| local_sum(&values, dem.width(), dem.height(), radius);
    let near = sum(counts.iter().map(|s| s.near).collect());
    let undergrowth_hits = sum(counts.iter().map(|s| s.undergrowth_hits).collect());
    let nonterminal = sum(counts.iter().map(|s| s.undergrowth_nonterminal).collect());
    #[cfg(test)]
    let terminal = sum(counts.iter().map(|s| s.undergrowth_terminal).collect());
    let ground_in_undergrowth_band = sum(counts
        .iter()
        .map(|s| s.ground_in_undergrowth_band)
        .collect());
    #[cfg(test)]
    let higher_near_ground_hits = sum(counts.iter().map(|s| s.higher_near_ground_hits).collect());
    let ground = sum(counts.iter().map(|s| s.ground).collect());
    let rough = sum(relief.field.iter().map(|v| v * v).collect());
    let area = sum(vec![1.; n]);
    let coherent = sum(coherent_cells);
    (0..n)
        .map(|i| Evidence {
            near: near[i],
            undergrowth_hits: undergrowth_hits[i],
            undergrowth_nonterminal: nonterminal[i],
            #[cfg(test)]
            undergrowth_terminal: terminal[i],
            ground_in_undergrowth_band: ground_in_undergrowth_band[i],
            #[cfg(test)]
            higher_near_ground_hits: higher_near_ground_hits[i],
            ground: ground[i],
            roughness: (rough[i] / area[i]).sqrt(),
            coherent: coherent[i] / near[i].max(1.),
            ..Default::default()
        })
        .collect()
}

#[cfg(test)]
mod tests;
