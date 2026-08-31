use std::collections::VecDeque;

use crate::{
    parameters::{BuildingClassificationEvidence, BuildingParameters},
    raster::{BuildingCandidateId, BuildingProbability, Dfm},
};

use super::{BuildingDetection, BuildingSurfaceFit, MINIMUM_RANSAC_SAMPLE_SIZE};

pub fn detect_buildings(
    fit: &BuildingSurfaceFit,
    params: &BuildingParameters,
) -> BuildingDetection {
    debug_assert!(
        fit.normal_x.grid.same_layout(&fit.height_mean.grid)
            && fit.normal_y.grid.same_layout(&fit.height_mean.grid)
            && fit.normal_z.grid.same_layout(&fit.height_mean.grid),
        "building fit diagnostics must use one grid"
    );
    let width = fit.height_mean.width();
    let height = fit.height_mean.height();
    let cell_count = width * height;
    let mut probability = Dfm::<BuildingProbability>::new_like(&fit.height_mean);
    let mut candidate_id = Dfm::<BuildingCandidateId>::new_like(&fit.height_mean);
    let mut accepted_mask = Dfm::<BuildingProbability>::new_like(&fit.height_mean);
    let mut plane_rejected_mask = Dfm::<BuildingProbability>::new_like(&fit.height_mean);
    probability.field.fill(0.);
    candidate_id.field.fill(0.);
    accepted_mask.field.fill(0.);
    plane_rejected_mask.field.fill(0.);

    if !params.enabled || !valid_parameters(params) {
        return BuildingDetection {
            probability,
            candidate_id,
            accepted_mask,
            plane_rejected_mask,
        };
    }

    let mut seeds = vec![false; cell_count];
    let mut local_scores = vec![0_f32; cell_count];
    for index in 0..cell_count {
        let height_mean = fit.height_mean.field[index];
        let height_max = fit.height_max.field[index];
        let class_6 = fit.class_6_fraction[index];
        let authoritative = params.class_6_evidence
            == BuildingClassificationEvidence::Authoritative
            && class_6 >= 0.5;
        let height_ok = height_mean >= params.minimum_roof_height_m
            && height_max <= params.maximum_roof_height_m;
        let regular_seed = height_ok
            && fit.plane_residual.field[index] <= params.maximum_plane_residual_m
            && fit.planar_point_fraction.field[index] >= params.minimum_planar_point_fraction
            && fit.vegetation_fraction[index] <= params.maximum_vegetation_fraction
            && fit.normal_z.field[index] > 0.;
        if !regular_seed && !(authoritative && height_max > params.minimum_roof_height_m) {
            continue;
        }

        let height_score = ((height_mean - params.minimum_roof_height_m) / 1.).clamp(0., 1.)
            * ((params.maximum_roof_height_m - height_max) / 2.).clamp(0., 1.);
        let residual_score = (1.
            - fit.plane_residual.field[index] / params.maximum_plane_residual_m.max(1e-3))
        .clamp(0., 1.);
        let planar_score = ((fit.planar_point_fraction.field[index]
            - params.minimum_planar_point_fraction)
            / (1. - params.minimum_planar_point_fraction).max(0.05))
        .clamp(0., 1.);
        let support_score = (fit.elevated_point_count.field[index] / 2.).clamp(0., 1.);
        let mut score = 0.15 * height_score
            + 0.3 * planar_score
            + 0.25 * residual_score
            + 0.1 * support_score
            + 0.2 * (1. - fit.vegetation_fraction[index]);
        if params.class_6_evidence == BuildingClassificationEvidence::Supporting {
            score = (score + 0.2 * class_6).min(1.);
        } else if authoritative {
            score = score.max(0.98);
        }
        seeds[index] = true;
        local_scores[index] = score;
    }

    let retained_candidates = retain_candidate_components(&seeds, fit, params, width, height);
    let radius = ((params.merge_gap_m / (2. * fit.height_mean.grid.cell_size_m)).ceil() as usize)
        .max(usize::from(params.merge_gap_m > 0.));
    let mut merged = close_mask(&retained_candidates, width, height, radius);
    fill_small_holes(
        &mut merged,
        width,
        height,
        fit.height_mean.grid.cell_size_m.powi(2),
        params.maximum_candidate_hole_area_m2,
    );

    score_candidates(
        &merged,
        &local_scores,
        fit,
        params,
        &mut probability.field,
        &mut candidate_id.field,
        &mut accepted_mask.field,
    );

    for index in 0..cell_count {
        let authoritative = params.class_6_evidence
            == BuildingClassificationEvidence::Authoritative
            && fit.class_6_fraction[index] >= 0.5;
        let failed_plane_fit = fit.elevated_point_count.field[index] > 0.
            && (fit.normal_z.field[index] <= 0.
                || fit.planar_point_fraction.field[index] < params.minimum_planar_point_fraction);
        if !authoritative
            && failed_plane_fit
            && fit.vegetation_fraction[index] <= params.maximum_vegetation_fraction
        {
            plane_rejected_mask.field[index] = 1.;
        }
    }

    BuildingDetection {
        probability,
        candidate_id,
        accepted_mask,
        plane_rejected_mask,
    }
}

fn valid_parameters(params: &BuildingParameters) -> bool {
    params.minimum_roof_height_m.is_finite()
        && params.maximum_roof_height_m.is_finite()
        && params.maximum_roof_height_m > params.minimum_roof_height_m
        && params.maximum_plane_residual_m.is_finite()
        && params.maximum_plane_residual_m > 0.
        && params.ransac_iterations > 0
        && params.ransac_sample_size >= MINIMUM_RANSAC_SAMPLE_SIZE
        && params.minimum_plane_inliers >= params.ransac_sample_size
        && params.maximum_roof_planes > 0
        && params.maximum_roof_slope_degrees.is_finite()
        && (0.0..90.0).contains(&params.maximum_roof_slope_degrees)
        && (0.0..=1.0).contains(&params.minimum_planar_point_fraction)
        && params.minimum_building_area_m2.is_finite()
        && params.minimum_building_area_m2 >= 0.
        && params.maximum_candidate_hole_area_m2.is_finite()
        && params.maximum_candidate_hole_area_m2 >= 0.
        && params.merge_gap_m.is_finite()
        && params.merge_gap_m >= 0.
        && (0.0..=1.0).contains(&params.minimum_rectangularity_or_compactness)
        && (0.0..=1.0).contains(&params.maximum_vegetation_fraction)
        && (0.0..=1.0).contains(&params.confidence_threshold)
}

fn retain_candidate_components(
    seeds: &[bool],
    fit: &BuildingSurfaceFit,
    params: &BuildingParameters,
    width: usize,
    height: usize,
) -> Vec<bool> {
    let mut visited = vec![false; seeds.len()];
    let mut retained = vec![false; seeds.len()];
    let minimum_component_area = (params.minimum_building_area_m2 * 0.08).clamp(0.5, 2.);
    let cell_area = fit.height_mean.grid.cell_size_m.powi(2);
    let minimum_cells = (minimum_component_area / cell_area).ceil() as usize;
    for start in 0..seeds.len() {
        if !seeds[start] || visited[start] {
            continue;
        }
        visited[start] = true;
        let mut queue = VecDeque::from([start]);
        let mut component = Vec::new();
        while let Some(index) = queue.pop_front() {
            component.push(index);
            let y = index / width;
            let x = index % width;
            for neighbor in neighbors8(y, x, width, height) {
                if !seeds[neighbor] || visited[neighbor] {
                    continue;
                }
                visited[neighbor] = true;
                queue.push_back(neighbor);
            }
        }
        if component.len() >= minimum_cells {
            for index in component {
                retained[index] = true;
            }
        }
    }
    retained
}

pub(super) fn close_mask(mask: &[bool], width: usize, height: usize, radius: usize) -> Vec<bool> {
    if radius == 0 {
        return mask.to_vec();
    }
    let mut dilated = vec![false; mask.len()];
    for y in 0..height {
        for x in 0..width {
            let top = y.saturating_sub(radius);
            let bottom = (y + radius).min(height - 1);
            let left = x.saturating_sub(radius);
            let right = (x + radius).min(width - 1);
            dilated[y * width + x] =
                (top..=bottom).any(|yi| (left..=right).any(|xi| mask[yi * width + xi]));
        }
    }
    let mut closed = vec![false; mask.len()];
    for y in 0..height {
        for x in 0..width {
            let top = y.saturating_sub(radius);
            let bottom = (y + radius).min(height - 1);
            let left = x.saturating_sub(radius);
            let right = (x + radius).min(width - 1);
            closed[y * width + x] =
                (top..=bottom).all(|yi| (left..=right).all(|xi| dilated[yi * width + xi]));
        }
    }
    closed
}

pub(super) fn fill_small_holes(
    mask: &mut [bool],
    width: usize,
    height: usize,
    cell_area: f64,
    maximum_hole_area: f64,
) {
    if maximum_hole_area <= 0. {
        return;
    }
    let maximum_cells = (maximum_hole_area / cell_area).floor() as usize;
    let mut visited = vec![false; mask.len()];
    for start in 0..mask.len() {
        if mask[start] || visited[start] {
            continue;
        }
        visited[start] = true;
        let mut queue = VecDeque::from([start]);
        let mut component = Vec::new();
        let mut touches_edge = false;
        while let Some(index) = queue.pop_front() {
            component.push(index);
            let y = index / width;
            let x = index % width;
            touches_edge |= y == 0 || x == 0 || y + 1 == height || x + 1 == width;
            for neighbor in neighbors4(y, x, width, height) {
                if !mask[neighbor] && !visited[neighbor] {
                    visited[neighbor] = true;
                    queue.push_back(neighbor);
                }
            }
        }
        if !touches_edge && component.len() <= maximum_cells {
            for index in component {
                mask[index] = true;
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn score_candidates(
    mask: &[bool],
    local_scores: &[f32],
    fit: &BuildingSurfaceFit,
    params: &BuildingParameters,
    probability: &mut [f32],
    candidate_id: &mut [f32],
    accepted_mask: &mut [f32],
) {
    let width = fit.height_mean.width();
    let height = fit.height_mean.height();
    let cell_area = fit.height_mean.grid.cell_size_m.powi(2);
    let mut visited = vec![false; mask.len()];
    let mut next_id = 1_u32;

    for start in 0..mask.len() {
        if !mask[start] || visited[start] {
            continue;
        }
        visited[start] = true;
        let mut queue = VecDeque::from([start]);
        let mut component = Vec::new();
        let mut perimeter_edges = 0_usize;
        let mut min_x = width;
        let mut max_x = 0;
        let mut min_y = height;
        let mut max_y = 0;
        while let Some(index) = queue.pop_front() {
            component.push(index);
            let y = index / width;
            let x = index % width;
            min_x = min_x.min(x);
            max_x = max_x.max(x);
            min_y = min_y.min(y);
            max_y = max_y.max(y);
            for (dy, dx) in [(-1_isize, 0_isize), (0, 1), (1, 0), (0, -1)] {
                let ny = y as isize + dy;
                let nx = x as isize + dx;
                if ny < 0 || nx < 0 || ny >= height as isize || nx >= width as isize {
                    perimeter_edges += 1;
                    continue;
                }
                let neighbor = ny as usize * width + nx as usize;
                if !mask[neighbor] {
                    perimeter_edges += 1;
                } else if !visited[neighbor] {
                    visited[neighbor] = true;
                    queue.push_back(neighbor);
                }
            }
        }

        let area = component.len() as f64 * cell_area;
        let bounds_area = (max_x - min_x + 1) as f64 * (max_y - min_y + 1) as f64 * cell_area;
        let rectangularity = (area / bounds_area.max(cell_area)) as f32;
        let perimeter = perimeter_edges as f64 * fit.height_mean.grid.cell_size_m;
        let compactness = (4. * std::f64::consts::PI * area / perimeter.powi(2).max(cell_area))
            .clamp(0., 1.) as f32;
        let shape_score = rectangularity.max(compactness);

        let mut supported_cells = 0_u32;
        let mut local_score_sum = 0.;
        let mut vegetation_sum = 0.;
        let mut class_6_sum = 0.;
        let mut minimum_height = f32::INFINITY;
        let mut maximum_height = f32::NEG_INFINITY;
        for &index in &component {
            if fit.elevated_point_count.field[index] <= 0. {
                continue;
            }
            supported_cells += 1;
            local_score_sum += local_scores[index];
            vegetation_sum += fit.vegetation_fraction[index];
            class_6_sum += fit.class_6_fraction[index];
            minimum_height = minimum_height.min(fit.height_mean.field[index]);
            maximum_height = maximum_height.max(fit.height_mean.field[index]);
        }
        let divisor = supported_cells.max(1) as f32;
        let local_score = local_score_sum / divisor;
        let vegetation_fraction = vegetation_sum / divisor;
        let class_6_fraction = class_6_sum / divisor;
        let height_range = (maximum_height - minimum_height).max(0.);
        let height_stability = (1. - height_range / 8.).clamp(0., 1.);
        let mut candidate_score = 0.75 * local_score + 0.15 * shape_score + 0.1 * height_stability;
        if params.class_6_evidence == BuildingClassificationEvidence::Supporting {
            candidate_score = (candidate_score + 0.1 * class_6_fraction).min(1.);
        }
        let authoritative = params.class_6_evidence
            == BuildingClassificationEvidence::Authoritative
            && class_6_fraction >= 0.5;
        if authoritative {
            candidate_score = candidate_score.max(0.98);
        }
        let accepted = area >= params.minimum_building_area_m2
            && (authoritative
                || (shape_score >= params.minimum_rectangularity_or_compactness
                    && vegetation_fraction <= params.maximum_vegetation_fraction
                    && candidate_score >= params.confidence_threshold));

        for index in component {
            probability[index] = candidate_score;
            candidate_id[index] = next_id as f32;
            if accepted {
                accepted_mask[index] = 1.;
            }
        }
        next_id += 1;
    }
}

fn neighbors4(y: usize, x: usize, width: usize, height: usize) -> Vec<usize> {
    let mut output = Vec::with_capacity(4);
    if y > 0 {
        output.push((y - 1) * width + x);
    }
    if x + 1 < width {
        output.push(y * width + x + 1);
    }
    if y + 1 < height {
        output.push((y + 1) * width + x);
    }
    if x > 0 {
        output.push(y * width + x - 1);
    }
    output
}

pub(super) fn neighbors8(y: usize, x: usize, width: usize, height: usize) -> Vec<usize> {
    let mut output = Vec::with_capacity(8);
    for dy in -1_isize..=1 {
        for dx in -1_isize..=1 {
            if dy == 0 && dx == 0 {
                continue;
            }
            let ny = y as isize + dy;
            let nx = x as isize + dx;
            if ny >= 0 && nx >= 0 && ny < height as isize && nx < width as isize {
                output.push(ny as usize * width + nx as usize);
            }
        }
    }
    output
}
