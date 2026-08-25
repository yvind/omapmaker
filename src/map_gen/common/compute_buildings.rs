use std::collections::{HashMap, VecDeque};

use geo::{Area, BooleanOps, Simplify};
use las::point::Classification;

use crate::{
    geometry::{MapMultiPolygon, PointCloud},
    map_gen::egui_map::{AreaSymbol, MapObject},
    parameters::{BufferRule, BuildingClassificationEvidence, BuildingParameters},
    raster::{
        BuildingCandidateId, BuildingProbability, Dfm, ElevatedPointCount, Elevation,
        HeightAboveGroundMax, HeightAboveGroundMean, PlanarPointFraction, PlaneResidual,
        SurfaceNormalX, SurfaceNormalY, SurfaceNormalZ,
    },
};

const MINIMUM_FIT_SUPPORT: u32 = 6;
const FIT_ELEVATION_FLOOR_M: f64 = 0.5;
const FIT_ELEVATION_CEILING_M: f64 = 100.;
const PLANAR_POINT_TOLERANCE_M: f64 = 0.25;

/// Expensive point-derived products. This is cached independently of the
/// candidate thresholds so changing those thresholds does not repeat fitting.
pub struct BuildingSurfaceFit {
    pub height_mean: Dfm<HeightAboveGroundMean>,
    pub height_max: Dfm<HeightAboveGroundMax>,
    pub elevated_point_count: Dfm<ElevatedPointCount>,
    pub planar_point_fraction: Dfm<PlanarPointFraction>,
    pub plane_residual: Dfm<PlaneResidual>,
    pub normal_x: Dfm<SurfaceNormalX>,
    pub normal_y: Dfm<SurfaceNormalY>,
    pub normal_z: Dfm<SurfaceNormalZ>,
    vegetation_fraction: Box<[f32]>,
    class_6_fraction: Box<[f32]>,
}

/// Cheap threshold-dependent products. Candidate IDs are assigned in raster
/// order, making the result independent of Rayon thread count.
pub struct BuildingDetection {
    pub probability: Dfm<BuildingProbability>,
    pub candidate_id: Dfm<BuildingCandidateId>,
    accepted_mask: Dfm<BuildingProbability>,
}

impl BuildingDetection {
    /// Accepted building cells for hard exclusion in other feature detectors.
    pub fn accepted_mask(&self) -> &Dfm<BuildingProbability> {
        &self.accepted_mask
    }
}

#[derive(Clone, Copy, Default)]
struct Moments {
    count: f64,
    x: f64,
    y: f64,
    z: f64,
    xx: f64,
    xy: f64,
    yy: f64,
    xz: f64,
    yz: f64,
    zz: f64,
}

impl Moments {
    fn sample(x: f64, y: f64, z: f64) -> Self {
        Self {
            count: 1.,
            x,
            y,
            z,
            xx: x * x,
            xy: x * y,
            yy: y * y,
            xz: x * z,
            yz: y * z,
            zz: z * z,
        }
    }

    fn add(self, rhs: Self) -> Self {
        Self {
            count: self.count + rhs.count,
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
            xx: self.xx + rhs.xx,
            xy: self.xy + rhs.xy,
            yy: self.yy + rhs.yy,
            xz: self.xz + rhs.xz,
            yz: self.yz + rhs.yz,
            zz: self.zz + rhs.zz,
        }
    }

    fn sub(self, rhs: Self) -> Self {
        Self {
            count: self.count - rhs.count,
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
            xx: self.xx - rhs.xx,
            xy: self.xy - rhs.xy,
            yy: self.yy - rhs.yy,
            xz: self.xz - rhs.xz,
            yz: self.yz - rhs.yz,
            zz: self.zz - rhs.zz,
        }
    }
}

#[derive(Clone)]
struct ElevatedSample {
    x: f64,
    y: f64,
    z: f64,
    height: f64,
    cell: usize,
    vegetation_evidence: bool,
    class_6: bool,
    classification_priority: u8,
    return_number: u8,
    number_of_returns: u8,
}

/// Fits one deterministic least-squares plane around every occupied elevated
/// cell. Exact duplicate returns are removed before moments are accumulated,
/// preventing overlapping source files from increasing support artificially.
pub fn compute_building_surface_fit(
    cloud: &PointCloud,
    dem: &Dfm<Elevation>,
    plane_fit_radius_m: f64,
) -> crate::Result<BuildingSurfaceFit> {
    anyhow::ensure!(
        plane_fit_radius_m.is_finite() && plane_fit_radius_m >= dem.grid.cell_size_m,
        "building plane-fit radius must be finite and at least one raster cell"
    );

    let width = dem.width();
    let height = dem.height();
    let cell_count = width * height;
    let mut samples = Vec::new();

    for point in &cloud.points {
        let xi = ((point.x() - dem.grid.top_left.x) / dem.grid.cell_size_m).round() as isize;
        let yi = ((dem.grid.top_left.y - point.y()) / dem.grid.cell_size_m).round() as isize;
        if xi < 0 || yi < 0 || xi >= width as isize || yi >= height as isize {
            continue;
        }
        let cell = yi as usize * width + xi as usize;
        let point_height = point.0.z - f64::from(dem.field[cell]);
        if !(FIT_ELEVATION_FLOOR_M..=FIT_ELEVATION_CEILING_M).contains(&point_height) {
            continue;
        }
        let vegetation_class = matches!(
            point.0.classification,
            Classification::LowVegetation
                | Classification::MediumVegetation
                | Classification::HighVegetation
        );
        samples.push(ElevatedSample {
            x: point.x() - dem.grid.top_left.x,
            y: point.y() - dem.grid.top_left.y,
            z: point.0.z,
            height: point_height,
            cell,
            vegetation_evidence: vegetation_class || point.0.number_of_returns > 1,
            class_6: point.0.classification == Classification::Building,
            classification_priority: match point.0.classification {
                Classification::Building => 2,
                Classification::LowVegetation
                | Classification::MediumVegetation
                | Classification::HighVegetation => 1,
                _ => 0,
            },
            return_number: point.0.return_number,
            number_of_returns: point.0.number_of_returns,
        });
    }

    samples.sort_by(|a, b| {
        a.x.total_cmp(&b.x)
            .then_with(|| a.y.total_cmp(&b.y))
            .then_with(|| a.z.total_cmp(&b.z))
            .then_with(|| a.return_number.cmp(&b.return_number))
            .then_with(|| a.number_of_returns.cmp(&b.number_of_returns))
            .then_with(|| b.classification_priority.cmp(&a.classification_priority))
    });
    samples.dedup_by(|a, b| {
        a.x.to_bits() == b.x.to_bits()
            && a.y.to_bits() == b.y.to_bits()
            && a.z.to_bits() == b.z.to_bits()
            && a.return_number == b.return_number
            && a.number_of_returns == b.number_of_returns
    });

    let mut cell_moments = vec![Moments::default(); cell_count];
    let mut height_sums = vec![0.; cell_count];
    let mut height_maxima = vec![0_f64; cell_count];
    let mut point_counts = vec![0_u32; cell_count];
    let mut vegetation_counts = vec![0_u32; cell_count];
    let mut class_6_counts = vec![0_u32; cell_count];
    for sample in &samples {
        cell_moments[sample.cell] =
            cell_moments[sample.cell].add(Moments::sample(sample.x, sample.y, sample.z));
        height_sums[sample.cell] += sample.height;
        height_maxima[sample.cell] = height_maxima[sample.cell].max(sample.height);
        point_counts[sample.cell] += 1;
        vegetation_counts[sample.cell] += u32::from(sample.vegetation_evidence);
        class_6_counts[sample.cell] += u32::from(sample.class_6);
    }

    let prefix_width = width + 1;
    let mut prefix = vec![Moments::default(); prefix_width * (height + 1)];
    for y in 0..height {
        for x in 0..width {
            let destination = (y + 1) * prefix_width + x + 1;
            prefix[destination] = cell_moments[y * width + x]
                .add(prefix[y * prefix_width + x + 1])
                .add(prefix[(y + 1) * prefix_width + x])
                .sub(prefix[y * prefix_width + x]);
        }
    }
    drop(cell_moments);

    let mut height_mean = Dfm::<HeightAboveGroundMean>::new_like(dem);
    let mut height_max = Dfm::<HeightAboveGroundMax>::new_like(dem);
    let mut elevated_point_count = Dfm::<ElevatedPointCount>::new_like(dem);
    let mut planar_point_fraction = Dfm::<PlanarPointFraction>::new_like(dem);
    let mut plane_residual = Dfm::<PlaneResidual>::new_like(dem);
    let mut normal_x = Dfm::<SurfaceNormalX>::new_like(dem);
    let mut normal_y = Dfm::<SurfaceNormalY>::new_like(dem);
    let mut normal_z = Dfm::<SurfaceNormalZ>::new_like(dem);
    for raster in [
        &mut height_mean.field,
        &mut height_max.field,
        &mut elevated_point_count.field,
        &mut planar_point_fraction.field,
        &mut normal_x.field,
        &mut normal_y.field,
        &mut normal_z.field,
    ] {
        raster.fill(0.);
    }
    plane_residual.field.fill(f32::INFINITY);

    let mut vegetation_fraction = vec![0.; cell_count].into_boxed_slice();
    let mut class_6_fraction = vec![0.; cell_count].into_boxed_slice();
    let mut plane_coefficients = vec![None; cell_count];
    let radius_cells = (plane_fit_radius_m / dem.grid.cell_size_m).ceil() as usize;

    for y in 0..height {
        for x in 0..width {
            let index = y * width + x;
            let own_count = point_counts[index];
            if own_count == 0 {
                continue;
            }
            height_mean.field[index] = (height_sums[index] / f64::from(own_count)) as f32;
            height_max.field[index] = height_maxima[index] as f32;
            elevated_point_count.field[index] = own_count as f32;
            vegetation_fraction[index] = vegetation_counts[index] as f32 / own_count as f32;
            class_6_fraction[index] = class_6_counts[index] as f32 / own_count as f32;

            let top = y.saturating_sub(radius_cells);
            let bottom = (y + radius_cells + 1).min(height);
            let left = x.saturating_sub(radius_cells);
            let right = (x + radius_cells + 1).min(width);
            let neighborhood = rectangle_sum(&prefix, prefix_width, top, left, bottom, right);
            if neighborhood.count < f64::from(MINIMUM_FIT_SUPPORT) {
                continue;
            }

            let mean_x = neighborhood.x / neighborhood.count;
            let mean_y = neighborhood.y / neighborhood.count;
            let mean_z = neighborhood.z / neighborhood.count;
            let xx = neighborhood.xx - neighborhood.x * mean_x;
            let xy = neighborhood.xy - neighborhood.x * mean_y;
            let yy = neighborhood.yy - neighborhood.y * mean_y;
            let xz = neighborhood.xz - neighborhood.x * mean_z;
            let yz = neighborhood.yz - neighborhood.y * mean_z;
            let zz = (neighborhood.zz - neighborhood.z * mean_z).max(0.);
            let determinant = xx * yy - xy * xy;
            if determinant <= 1e-10 * (xx * yy).max(1.) {
                continue;
            }
            let slope_x = (xz * yy - yz * xy) / determinant;
            let slope_y = (yz * xx - xz * xy) / determinant;
            let residual_sum = (zz - slope_x * xz - slope_y * yz).max(0.);
            let residual = (residual_sum / neighborhood.count).sqrt();
            let normal_length = (slope_x * slope_x + slope_y * slope_y + 1.).sqrt();
            normal_x.field[index] = (-slope_x / normal_length) as f32;
            normal_y.field[index] = (-slope_y / normal_length) as f32;
            normal_z.field[index] = (1. / normal_length) as f32;
            plane_residual.field[index] = residual as f32;
            plane_coefficients[index] = Some((
                slope_x,
                slope_y,
                mean_z - slope_x * mean_x - slope_y * mean_y,
            ));
        }
    }
    drop(prefix);

    let mut planar_counts = vec![0_u32; cell_count];
    for sample in &samples {
        if let Some((slope_x, slope_y, intercept)) = plane_coefficients[sample.cell] {
            let predicted = slope_x * sample.x + slope_y * sample.y + intercept;
            planar_counts[sample.cell] +=
                u32::from((sample.z - predicted).abs() <= PLANAR_POINT_TOLERANCE_M);
        }
    }
    for (index, &planar_count) in planar_counts.iter().enumerate() {
        let count = elevated_point_count.field[index];
        if count > 0. {
            planar_point_fraction.field[index] = planar_count as f32 / count;
        }
    }

    Ok(BuildingSurfaceFit {
        height_mean,
        height_max,
        elevated_point_count,
        planar_point_fraction,
        plane_residual,
        normal_x,
        normal_y,
        normal_z,
        vegetation_fraction,
        class_6_fraction,
    })
}

fn rectangle_sum(
    prefix: &[Moments],
    stride: usize,
    top: usize,
    left: usize,
    bottom: usize,
    right: usize,
) -> Moments {
    prefix[bottom * stride + right]
        .sub(prefix[top * stride + right])
        .sub(prefix[bottom * stride + left])
        .add(prefix[top * stride + left])
}

pub fn detect_buildings(
    fit: &BuildingSurfaceFit,
    params: &BuildingParameters,
) -> BuildingDetection {
    let width = fit.height_mean.width();
    let height = fit.height_mean.height();
    let cell_count = width * height;
    let mut probability = Dfm::<BuildingProbability>::new_like(&fit.height_mean);
    let mut candidate_id = Dfm::<BuildingCandidateId>::new_like(&fit.height_mean);
    let mut accepted_mask = Dfm::<BuildingProbability>::new_like(&fit.height_mean);
    probability.field.fill(0.);
    candidate_id.field.fill(0.);
    accepted_mask.field.fill(0.);

    if !params.enabled || !valid_parameters(params) {
        return BuildingDetection {
            probability,
            candidate_id,
            accepted_mask,
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
        if !regular_seed && !(authoritative && height_max >= FIT_ELEVATION_FLOOR_M as f32) {
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

    let retained_facets = retain_roof_facets(&seeds, fit, params, width, height);
    let radius = ((params.merge_gap_m / (2. * fit.height_mean.grid.cell_size_m)).ceil() as usize)
        .max(usize::from(params.merge_gap_m > 0.));
    let mut merged = close_mask(&retained_facets, width, height, radius);
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

    BuildingDetection {
        probability,
        candidate_id,
        accepted_mask,
    }
}

fn valid_parameters(params: &BuildingParameters) -> bool {
    params.minimum_roof_height_m.is_finite()
        && params.maximum_roof_height_m.is_finite()
        && params.maximum_roof_height_m > params.minimum_roof_height_m
        && params.maximum_plane_residual_m.is_finite()
        && params.maximum_plane_residual_m > 0.
        && (0.0..=1.0).contains(&params.minimum_planar_point_fraction)
        && params
            .maximum_neighboring_normal_difference_degrees
            .is_finite()
        && params.maximum_facet_height_discontinuity_m.is_finite()
        && params.maximum_facet_height_discontinuity_m >= 0.
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

fn retain_roof_facets(
    seeds: &[bool],
    fit: &BuildingSurfaceFit,
    params: &BuildingParameters,
    width: usize,
    height: usize,
) -> Vec<bool> {
    let mut visited = vec![false; seeds.len()];
    let mut retained = vec![false; seeds.len()];
    let minimum_facet_area = (params.minimum_building_area_m2 * 0.08).clamp(0.5, 2.);
    let cell_area = fit.height_mean.grid.cell_size_m.powi(2);
    let minimum_cells = (minimum_facet_area / cell_area).ceil() as usize;
    let maximum_angle = params
        .maximum_neighboring_normal_difference_degrees
        .to_radians()
        .cos();

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
                if (fit.height_mean.field[index] - fit.height_mean.field[neighbor]).abs()
                    > params.maximum_facet_height_discontinuity_m
                {
                    continue;
                }
                let dot = fit.normal_x.field[index] * fit.normal_x.field[neighbor]
                    + fit.normal_y.field[index] * fit.normal_y.field[neighbor]
                    + fit.normal_z.field[index] * fit.normal_z.field[neighbor];
                if dot < maximum_angle {
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

fn close_mask(mask: &[bool], width: usize, height: usize, radius: usize) -> Vec<bool> {
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
    for y in radius..height.saturating_sub(radius) {
        for x in radius..width.saturating_sub(radius) {
            closed[y * width + x] = (y - radius..=y + radius)
                .all(|yi| (x - radius..=x + radius).all(|xi| dilated[yi * width + xi]));
        }
    }
    closed
}

fn fill_small_holes(
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

fn neighbors8(y: usize, x: usize, width: usize, height: usize) -> Vec<usize> {
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

pub fn building_objects(
    detection: &BuildingDetection,
    convex_hull: &geo::Polygon,
    cut_overlay: &geo::Polygon,
    buffer_rules: &[BufferRule],
) -> Vec<MapObject> {
    debug_assert!(
        detection
            .candidate_id
            .grid
            .same_layout(&detection.accepted_mask.grid),
        "building diagnostics must use one grid"
    );
    let contours = detection.accepted_mask.marching_squares(0.5);
    let mut polygons = geo::MultiPolygon::from_contours(contours, convex_hull, false)
        .simplify(crate::SIMPLIFICATION_DIST.max(detection.accepted_mask.grid.cell_size_m / 5.));
    for rule in buffer_rules {
        polygons = polygons.apply_buffer_rule(rule);
    }
    polygons = cut_overlay.intersection(&polygons);
    polygons
        .into_iter()
        .filter(|polygon| polygon.unsigned_area() > 0.)
        .map(|polygon| MapObject::Area {
            object: polygon,
            symbol: AreaSymbol::Building,
            tags: HashMap::new(),
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::raster::DfmGrid;
    use las::{Bounds, Vector};

    fn bounds() -> Bounds {
        Bounds {
            min: Vector {
                x: 0.,
                y: 0.,
                z: 0.,
            },
            max: Vector {
                x: 30.,
                y: 30.,
                z: 20.,
            },
        }
    }

    fn synthetic_roof(reverse: bool) -> (Dfm<Elevation>, PointCloud) {
        let grid = DfmGrid::new(31, 31, 1., geo::coord! { x: 0., y: 30. }).unwrap();
        let mut dem = Dfm::<Elevation>::new(grid);
        dem.field.fill(0.);
        let mut points = Vec::new();
        for y in 8..=18 {
            for x in 5..=16 {
                for offset in [-0.2, 0.2] {
                    let z = 5. + 0.08 * x as f64 + 0.03 * y as f64;
                    points.push(crate::geometry::PointLaz::new(
                        x as f64 + offset,
                        30. - y as f64,
                        z,
                    ));
                }
            }
        }
        // A similarly elevated but volume-like, multi-return tree patch.
        for y in 8..=14 {
            for x in 22..=26 {
                for level in [3., 6., 9.] {
                    let mut point = crate::geometry::PointLaz::new(x as f64, 30. - y as f64, level);
                    point.0.number_of_returns = 3;
                    point.0.return_number = (level / 3.) as u8;
                    points.push(point);
                }
            }
        }
        if reverse {
            points.reverse();
        }
        (dem, PointCloud::new(points, bounds()))
    }

    #[test]
    fn plane_fit_recovers_a_sloped_roof_and_rejects_volume_returns() {
        let (dem, cloud) = synthetic_roof(false);
        let fit = compute_building_surface_fit(&cloud, &dem, 2.5).unwrap();
        let roof = 12 * dem.width() + 10;
        let tree = 11 * dem.width() + 24;
        assert!(fit.plane_residual.field[roof] < 0.05);
        assert!(fit.normal_z.field[roof] > 0.98);
        assert!(fit.plane_residual.field[tree] > 1.);
        assert!(fit.vegetation_fraction[tree] > 0.9);
    }

    #[test]
    fn detector_emits_one_building_and_is_input_order_deterministic() {
        let (dem, cloud) = synthetic_roof(false);
        let (_, reversed) = synthetic_roof(true);
        let fit = compute_building_surface_fit(&cloud, &dem, 2.5).unwrap();
        let reversed_fit = compute_building_surface_fit(&reversed, &dem, 2.5).unwrap();
        let mut params = crate::parameters::MapParameters::default();
        params.building.minimum_building_area_m2 = 20.;
        params.building.confidence_threshold = 0.6;
        let detection = detect_buildings(&fit, &params.building);
        let reversed_detection = detect_buildings(&reversed_fit, &params.building);
        assert_eq!(
            detection.candidate_id.field,
            reversed_detection.candidate_id.field
        );
        assert_eq!(
            detection.accepted_mask.field,
            reversed_detection.accepted_mask.field
        );

        let hull = geo::Rect::new(
            geo::coord! { x: -1., y: -1. },
            geo::coord! { x: 31., y: 31. },
        )
        .to_polygon();
        let objects = building_objects(
            &detection,
            &hull,
            &hull,
            &params.geometry.buildings.buffer_rules,
        );
        assert_eq!(objects.len(), 1);
        assert!(matches!(
            objects[0],
            MapObject::Area {
                symbol: AreaSymbol::Building,
                ..
            }
        ));
    }

    #[test]
    fn authoritative_class_6_accepts_a_nonplanar_component() {
        let (dem, mut cloud) = synthetic_roof(false);
        for point in &mut cloud.points {
            if point.x() >= 22. {
                point.0.classification = Classification::Building;
            }
        }
        let fit = compute_building_surface_fit(&cloud, &dem, 2.5).unwrap();
        let ignored_params = BuildingParameters {
            minimum_building_area_m2: 10.,
            class_6_evidence: BuildingClassificationEvidence::Ignore,
            ..Default::default()
        };
        let ignored = detect_buildings(&fit, &ignored_params);
        let authoritative_params = BuildingParameters {
            class_6_evidence: BuildingClassificationEvidence::Authoritative,
            ..ignored_params
        };
        let authoritative = detect_buildings(&fit, &authoritative_params);

        let tree = 11 * dem.width() + 24;
        assert_eq!(ignored.accepted_mask.field[tree], 0.);
        assert_eq!(authoritative.accepted_mask.field[tree], 1.);
    }
}
