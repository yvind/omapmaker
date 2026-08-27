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

const FIT_ELEVATION_CEILING_M: f64 = 100.;
const MINIMUM_RANSAC_SAMPLE_SIZE: usize = 3;

/// Candidate-local RANSAC diagnostics. The expensive fit cache tracks only
/// candidate discovery and plane-model parameters, not acceptance scoring.
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
    /// Non-vegetation elevated candidates that lacked enough planar support.
    /// The independent cliff detector can confirm the rocky subset as boulders.
    pub plane_rejected_mask: Dfm<BuildingProbability>,
    accepted_mask: Dfm<BuildingProbability>,
}

impl BuildingDetection {
    /// Accepted building cells for hard exclusion in other feature detectors.
    pub fn accepted_mask(&self) -> &Dfm<BuildingProbability> {
        &self.accepted_mask
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

#[derive(Clone, Copy)]
struct Plane {
    // ax + by + cz + d = 0; the normal is always unit length and points up.
    a: f64,
    b: f64,
    c: f64,
    d: f64,
}

struct CandidatePlaneModel {
    plane: Plane,
    inliers: Vec<usize>,
    rmse: f64,
}

struct CandidateComponent {
    cells: Vec<usize>,
    samples: Vec<ElevatedSample>,
}

/// Finds candidates in a single-return DSM-minus-DEM raster, then fits one or
/// more planes to the last returns around each candidate using RANSAC.
pub fn compute_building_surface_fit(
    cloud: &PointCloud,
    dem: &Dfm<Elevation>,
    params: &BuildingParameters,
) -> crate::Result<BuildingSurfaceFit> {
    anyhow::ensure!(
        valid_ransac_parameters(params, dem.grid.cell_size_m),
        "invalid building candidate or RANSAC parameters"
    );

    let width = dem.width();
    let height = dem.height();
    let cell_count = width * height;
    let mut single_return_dsm_minus_dem = vec![0_f32; cell_count];
    let mut authoritative_class_6 = vec![false; cell_count];

    // Candidate discovery is deliberately raster-only and linear in the point
    // count. An exact cell maximum is more appropriate for a DSM than the
    // power-mean canopy raster used by the vegetation pipeline.
    for point in &cloud.points {
        let Some(cell) = point_cell(point.x(), point.y(), dem) else {
            continue;
        };
        let point_height = point.0.z - f64::from(dem.field[cell]);
        if !point_height.is_finite() || !(0.0..=FIT_ELEVATION_CEILING_M).contains(&point_height) {
            continue;
        }
        if point.0.return_number == 1 && point.0.number_of_returns == 1 {
            single_return_dsm_minus_dem[cell] =
                single_return_dsm_minus_dem[cell].max(point_height as f32);
        }
        if params.class_6_evidence == BuildingClassificationEvidence::Authoritative
            && point.0.classification == Classification::Building
            && point_height > f64::from(params.minimum_roof_height_m)
            && point_height <= f64::from(params.maximum_roof_height_m)
        {
            authoritative_class_6[cell] = true;
        }
    }

    let mut candidate_mask = single_return_dsm_minus_dem
        .iter()
        .zip(authoritative_class_6)
        .map(|(&height, class_6)| {
            class_6
                || (height > params.minimum_roof_height_m && height <= params.maximum_roof_height_m)
        })
        .collect::<Vec<_>>();
    let merge_radius = ((params.merge_gap_m / (2. * dem.grid.cell_size_m)).ceil() as usize)
        .max(usize::from(params.merge_gap_m > 0.));
    candidate_mask = close_mask(&candidate_mask, width, height, merge_radius);
    fill_small_holes(
        &mut candidate_mask,
        width,
        height,
        dem.grid.cell_size_m.powi(2),
        params.maximum_candidate_hole_area_m2,
    );
    let (candidate_ids, candidate_cells) =
        label_candidate_components(&candidate_mask, width, height);
    let mut candidates = candidate_cells
        .into_iter()
        .map(|cells| CandidateComponent {
            cells,
            samples: Vec::new(),
        })
        .collect::<Vec<_>>();

    // RANSAC sees only last/only returns strictly above the configured roof
    // floor. Points just outside a raster component are assigned to the
    // nearest candidate within the search radius, which fills sampling holes
    // without ever fitting the rest of the tile.
    let mut samples = Vec::new();
    for point in &cloud.points {
        if point.0.number_of_returns == 0 || point.0.return_number != point.0.number_of_returns {
            continue;
        }
        let Some(cell) = point_cell(point.x(), point.y(), dem) else {
            continue;
        };
        let point_height = point.0.z - f64::from(dem.field[cell]);
        if point_height <= f64::from(params.minimum_roof_height_m)
            || point_height > f64::from(params.maximum_roof_height_m)
            || point_height > FIT_ELEVATION_CEILING_M
        {
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
    let assignment_radius = (params.plane_fit_radius_m / dem.grid.cell_size_m).ceil() as usize;
    for sample in samples {
        let Some(candidate_id) = nearest_candidate_id(
            sample.cell,
            &candidate_ids,
            width,
            height,
            assignment_radius,
        ) else {
            continue;
        };
        candidates[candidate_id as usize - 1].samples.push(sample);
    }

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

    for (candidate_index, candidate) in candidates.into_iter().enumerate() {
        let point_count = candidate.samples.len();
        if point_count == 0 {
            continue;
        }
        let divisor = point_count as f64;
        let mean_height = (candidate
            .samples
            .iter()
            .map(|point| point.height)
            .sum::<f64>()
            / divisor) as f32;
        let maximum_height = candidate
            .samples
            .iter()
            .map(|point| point.height)
            .fold(0_f64, f64::max) as f32;
        let vegetation = candidate
            .samples
            .iter()
            .filter(|point| point.vegetation_evidence)
            .count() as f32
            / point_count as f32;
        let class_6 = candidate
            .samples
            .iter()
            .filter(|point| point.class_6)
            .count() as f32
            / point_count as f32;
        let models = fit_candidate_planes(&candidate.samples, params, candidate_index as u64 + 1);
        let inlier_count = models
            .iter()
            .map(|model| model.inliers.len())
            .sum::<usize>();
        let planarity = inlier_count as f32 / point_count as f32;
        let residual = if inlier_count == 0 {
            f32::INFINITY
        } else {
            (models
                .iter()
                .map(|model| model.rmse.powi(2) * model.inliers.len() as f64)
                .sum::<f64>()
                / inlier_count as f64)
                .sqrt() as f32
        };
        let dominant_plane = models
            .iter()
            .max_by_key(|model| model.inliers.len())
            .map(|model| model.plane);

        for cell in candidate.cells {
            height_mean.field[cell] = mean_height;
            height_max.field[cell] = maximum_height;
            elevated_point_count.field[cell] = point_count as f32;
            planar_point_fraction.field[cell] = planarity;
            plane_residual.field[cell] = residual;
            vegetation_fraction[cell] = vegetation;
            class_6_fraction[cell] = class_6;
            if let Some(plane) = dominant_plane {
                normal_x.field[cell] = plane.a as f32;
                normal_y.field[cell] = plane.b as f32;
                normal_z.field[cell] = plane.c as f32;
            }
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

fn point_cell<T: crate::raster::RasterMarker>(x: f64, y: f64, raster: &Dfm<T>) -> Option<usize> {
    let xi = ((x - raster.grid.top_left.x) / raster.grid.cell_size_m).round() as isize;
    let yi = ((raster.grid.top_left.y - y) / raster.grid.cell_size_m).round() as isize;
    (xi >= 0 && yi >= 0 && xi < raster.width() as isize && yi < raster.height() as isize)
        .then_some(yi as usize * raster.width() + xi as usize)
}

fn label_candidate_components(
    mask: &[bool],
    width: usize,
    height: usize,
) -> (Vec<u32>, Vec<Vec<usize>>) {
    let mut ids = vec![0_u32; mask.len()];
    let mut components = Vec::new();
    for start in 0..mask.len() {
        if !mask[start] || ids[start] != 0 {
            continue;
        }
        let id = components.len() as u32 + 1;
        ids[start] = id;
        let mut queue = VecDeque::from([start]);
        let mut cells = Vec::new();
        while let Some(cell) = queue.pop_front() {
            cells.push(cell);
            let y = cell / width;
            let x = cell % width;
            for neighbor in neighbors8(y, x, width, height) {
                if mask[neighbor] && ids[neighbor] == 0 {
                    ids[neighbor] = id;
                    queue.push_back(neighbor);
                }
            }
        }
        components.push(cells);
    }
    (ids, components)
}

fn nearest_candidate_id(
    cell: usize,
    candidate_ids: &[u32],
    width: usize,
    height: usize,
    radius: usize,
) -> Option<u32> {
    if candidate_ids[cell] != 0 {
        return Some(candidate_ids[cell]);
    }
    let y = cell / width;
    let x = cell % width;
    let top = y.saturating_sub(radius);
    let bottom = (y + radius).min(height - 1);
    let left = x.saturating_sub(radius);
    let right = (x + radius).min(width - 1);
    let mut best = None;
    let mut best_distance = usize::MAX;
    for yi in top..=bottom {
        for xi in left..=right {
            let id = candidate_ids[yi * width + xi];
            if id == 0 {
                continue;
            }
            let distance = yi.abs_diff(y).pow(2) + xi.abs_diff(x).pow(2);
            if distance <= radius.pow(2)
                && (distance < best_distance || (distance == best_distance && Some(id) < best))
            {
                best = Some(id);
                best_distance = distance;
            }
        }
    }
    best
}

fn fit_candidate_planes(
    points: &[ElevatedSample],
    params: &BuildingParameters,
    seed: u64,
) -> Vec<CandidatePlaneModel> {
    let mut remaining = (0..points.len()).collect::<Vec<_>>();
    let mut models = Vec::new();
    let mut rng = DeterministicRng::new(seed);
    while models.len() < params.maximum_roof_planes {
        let Some(model) = fit_ransac_plane(points, &remaining, params, &mut rng) else {
            break;
        };
        let mut inlier = vec![false; points.len()];
        for &index in &model.inliers {
            inlier[index] = true;
        }
        remaining.retain(|&index| !inlier[index]);
        models.push(model);
        if remaining.len() < params.minimum_plane_inliers.max(params.ransac_sample_size) {
            break;
        }
    }
    models
}

fn fit_ransac_plane(
    points: &[ElevatedSample],
    remaining: &[usize],
    params: &BuildingParameters,
    rng: &mut DeterministicRng,
) -> Option<CandidatePlaneModel> {
    let sample_size = params.ransac_sample_size.max(MINIMUM_RANSAC_SAMPLE_SIZE);
    let minimum_inliers = params.minimum_plane_inliers.max(sample_size);
    if remaining.len() < minimum_inliers {
        return None;
    }
    let mut best: Option<CandidatePlaneModel> = None;
    for _ in 0..params.ransac_iterations {
        let sample = sample_without_replacement(remaining, sample_size, rng);
        let Some(model) = Plane::from_points(points, &sample) else {
            continue;
        };
        if model.slope_degrees() > f64::from(params.maximum_roof_slope_degrees) {
            continue;
        }
        let inliers = plane_inliers(model, points, remaining, params.maximum_plane_residual_m);
        if inliers.len() < minimum_inliers {
            continue;
        }
        let Some(refined) = Plane::from_points(points, &inliers) else {
            continue;
        };
        if refined.slope_degrees() > f64::from(params.maximum_roof_slope_degrees) {
            continue;
        }
        let refined_inliers =
            plane_inliers(refined, points, remaining, params.maximum_plane_residual_m);
        if refined_inliers.len() < minimum_inliers {
            continue;
        }
        let rmse = refined.rmse(points, &refined_inliers);
        let replace = best.as_ref().is_none_or(|current| {
            refined_inliers.len() > current.inliers.len()
                || (refined_inliers.len() == current.inliers.len() && rmse < current.rmse)
        });
        if replace {
            best = Some(CandidatePlaneModel {
                plane: refined,
                inliers: refined_inliers,
                rmse,
            });
        }
    }
    best
}

fn plane_inliers(
    plane: Plane,
    points: &[ElevatedSample],
    candidates: &[usize],
    threshold: f32,
) -> Vec<usize> {
    candidates
        .iter()
        .copied()
        .filter(|&index| plane.residual(&points[index]) <= f64::from(threshold))
        .collect()
}

impl Plane {
    /// Orthogonal least-squares plane using the best-conditioned covariance
    /// minor, adapted from WhiteboxTools' `LidarRansacPlanes` implementation.
    fn from_points(points: &[ElevatedSample], indices: &[usize]) -> Option<Self> {
        if indices.len() < MINIMUM_RANSAC_SAMPLE_SIZE {
            return None;
        }
        let count = indices.len() as f64;
        let mut centroid = [0.; 3];
        for &index in indices {
            centroid[0] += points[index].x;
            centroid[1] += points[index].y;
            centroid[2] += points[index].z;
        }
        for value in &mut centroid {
            *value /= count;
        }
        let (mut xx, mut xy, mut xz, mut yy, mut yz, mut zz) = (0., 0., 0., 0., 0., 0.);
        for &index in indices {
            let x = points[index].x - centroid[0];
            let y = points[index].y - centroid[1];
            let z = points[index].z - centroid[2];
            xx += x * x;
            xy += x * y;
            xz += x * z;
            yy += y * y;
            yz += y * z;
            zz += z * z;
        }
        let det_x = yy * zz - yz * yz;
        let det_y = xx * zz - xz * xz;
        let det_z = xx * yy - xy * xy;
        let det_max = det_x.max(det_y).max(det_z);
        if !det_max.is_finite() || det_max <= 1e-12 {
            return None;
        }
        let (mut a, mut b, mut c) = if det_max == det_x {
            (1., (xz * yz - xy * zz) / det_x, (xy * yz - xz * yy) / det_x)
        } else if det_max == det_y {
            ((yz * xz - xy * zz) / det_y, 1., (xy * xz - yz * xx) / det_y)
        } else {
            ((yz * xy - xz * yy) / det_z, (xz * xy - yz * xx) / det_z, 1.)
        };
        let norm = (a * a + b * b + c * c).sqrt();
        if !norm.is_finite() || norm <= f64::EPSILON {
            return None;
        }
        a /= norm;
        b /= norm;
        c /= norm;
        if c < 0. {
            a = -a;
            b = -b;
            c = -c;
        }
        let d = -a * centroid[0] - b * centroid[1] - c * centroid[2];
        Some(Self { a, b, c, d })
    }

    fn residual(self, point: &ElevatedSample) -> f64 {
        // The normal is normalized in `from_points`, so this is an orthogonal
        // point-to-plane distance rather than a vertical z residual.
        (self.a * point.x + self.b * point.y + self.c * point.z + self.d).abs()
    }

    fn rmse(self, points: &[ElevatedSample], indices: &[usize]) -> f64 {
        (indices
            .iter()
            .map(|&index| self.residual(&points[index]).powi(2))
            .sum::<f64>()
            / indices.len().max(1) as f64)
            .sqrt()
    }

    fn slope_degrees(self) -> f64 {
        self.c.abs().clamp(0., 1.).acos().to_degrees()
    }
}

struct DeterministicRng(u64);

impl DeterministicRng {
    fn new(seed: u64) -> Self {
        Self(seed.wrapping_mul(0x9e37_79b9_7f4a_7c15).max(1))
    }

    fn index(&mut self, upper_bound: usize) -> usize {
        let mut value = self.0;
        value ^= value << 13;
        value ^= value >> 7;
        value ^= value << 17;
        self.0 = value;
        ((u128::from(value) * upper_bound as u128) >> 64) as usize
    }
}

fn sample_without_replacement(
    population: &[usize],
    sample_size: usize,
    rng: &mut DeterministicRng,
) -> Vec<usize> {
    let mut positions = Vec::with_capacity(sample_size);
    for current in population.len() - sample_size..population.len() {
        let draw = rng.index(current + 1);
        positions.push(if positions.contains(&draw) {
            current
        } else {
            draw
        });
    }
    positions
        .into_iter()
        .map(|position| population[position])
        .collect()
}

fn valid_ransac_parameters(params: &BuildingParameters, cell_size_m: f64) -> bool {
    params.minimum_roof_height_m.is_finite()
        && params.maximum_roof_height_m.is_finite()
        && params.maximum_roof_height_m > params.minimum_roof_height_m
        && params.plane_fit_radius_m.is_finite()
        && params.plane_fit_radius_m >= cell_size_m
        && params.maximum_plane_residual_m.is_finite()
        && params.maximum_plane_residual_m > 0.
        && params.ransac_iterations > 0
        && params.ransac_sample_size >= MINIMUM_RANSAC_SAMPLE_SIZE
        && params.minimum_plane_inliers >= params.ransac_sample_size
        && params.maximum_roof_planes > 0
        && params.maximum_roof_slope_degrees.is_finite()
        && (0.0..90.0).contains(&params.maximum_roof_slope_degrees)
        && params.maximum_candidate_hole_area_m2.is_finite()
        && params.maximum_candidate_hole_area_m2 >= 0.
        && params.merge_gap_m.is_finite()
        && params.merge_gap_m >= 0.
}

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
    parameters: &BuildingParameters,
    buffer_rules: &[BufferRule],
) -> Vec<MapObject> {
    debug_assert!(
        detection
            .candidate_id
            .grid
            .same_layout(&detection.accepted_mask.grid)
            && detection
                .plane_rejected_mask
                .grid
                .same_layout(&detection.accepted_mask.grid),
        "building diagnostics must use one grid"
    );
    let contours = detection.accepted_mask.marching_squares(0.5);
    let mut traced = geo::MultiPolygon::from_contours(contours, convex_hull, false);
    for rule in buffer_rules {
        traced = traced.apply_buffer_rule(rule);
    }
    let mut polygons = geo::MultiPolygon::new(
        traced
            .into_iter()
            .map(|polygon| {
                super::building_regularization::regularize_building_footprint(&polygon, parameters)
            })
            .collect(),
    )
    .simplify(crate::SIMPLIFICATION_DIST);
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
                    let mut point =
                        crate::geometry::PointLaz::new(x as f64 + offset, 30. - y as f64, z);
                    point.0.return_number = 1;
                    point.0.number_of_returns = 1;
                    points.push(point);
                }
            }
        }
        // A similarly elevated but volume-like, multi-return tree patch.
        for y in 8..=14 {
            for x in 22..=26 {
                let canopy_top = 7. + ((x * 11 + y * 7) % 9) as f64 * 0.45;
                for (return_number, level) in [(1, 3.), (2, 6.), (3, canopy_top)] {
                    let mut point = crate::geometry::PointLaz::new(x as f64, 30. - y as f64, level);
                    point.0.number_of_returns = 3;
                    point.0.return_number = return_number;
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
        let params = BuildingParameters {
            minimum_building_area_m2: 10.,
            ..Default::default()
        };
        let fit = compute_building_surface_fit(&cloud, &dem, &params).unwrap();
        let roof = 12 * dem.width() + 10;
        let tree = 11 * dem.width() + 24;
        assert!(fit.plane_residual.field[roof] < 0.05);
        assert!(fit.normal_z.field[roof] > 0.98);
        assert_eq!(fit.elevated_point_count.field[tree], 0.);
    }

    #[test]
    fn detector_emits_one_building_and_is_input_order_deterministic() {
        let (dem, cloud) = synthetic_roof(false);
        let (_, reversed) = synthetic_roof(true);
        let mut params = crate::parameters::MapParameters::default();
        params.building.minimum_building_area_m2 = 20.;
        params.building.confidence_threshold = 0.6;
        let fit = compute_building_surface_fit(&cloud, &dem, &params.building).unwrap();
        let reversed_fit = compute_building_surface_fit(&reversed, &dem, &params.building).unwrap();
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
            &params.building,
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
        let ignored_params = BuildingParameters {
            minimum_building_area_m2: 10.,
            class_6_evidence: BuildingClassificationEvidence::Ignore,
            ..Default::default()
        };
        let fit = compute_building_surface_fit(&cloud, &dem, &ignored_params).unwrap();
        let ignored = detect_buildings(&fit, &ignored_params);
        let authoritative_params = BuildingParameters {
            class_6_evidence: BuildingClassificationEvidence::Authoritative,
            ..ignored_params
        };
        let authoritative_fit =
            compute_building_surface_fit(&cloud, &dem, &authoritative_params).unwrap();
        let authoritative = detect_buildings(&authoritative_fit, &authoritative_params);

        let tree = 11 * dem.width() + 24;
        assert_eq!(ignored.accepted_mask.field[tree], 0.);
        assert_eq!(authoritative.accepted_mask.field[tree], 1.);
    }

    #[test]
    fn ransac_accepts_a_two_plane_roof_and_ignores_non_last_returns() {
        let grid = DfmGrid::new(31, 31, 1., geo::coord! { x: 0., y: 30. }).unwrap();
        let mut dem = Dfm::<Elevation>::new(grid);
        dem.field.fill(0.);
        let mut points = Vec::new();
        for y in 8..=18 {
            for x in 5..=19 {
                for offset in [-0.2, 0.2] {
                    let ridge_distance = (x as f64 - 12.).abs();
                    let mut roof = crate::geometry::PointLaz::new(
                        x as f64 + offset,
                        30. - y as f64,
                        7. - 0.18 * ridge_distance,
                    );
                    roof.0.return_number = 1;
                    roof.0.number_of_returns = 1;
                    points.push(roof);
                }

                // Neither point may influence the fit: the first is not a last
                // return and the last lies below the roof-height threshold.
                let mut canopy_first = crate::geometry::PointLaz::new(
                    x as f64,
                    30. - y as f64,
                    12. + ((x + y) % 5) as f64,
                );
                canopy_first.0.return_number = 1;
                canopy_first.0.number_of_returns = 2;
                points.push(canopy_first);
                let mut canopy_last = crate::geometry::PointLaz::new(x as f64, 30. - y as f64, 1.);
                canopy_last.0.return_number = 2;
                canopy_last.0.number_of_returns = 2;
                points.push(canopy_last);
            }
        }
        let cloud = PointCloud::new(points, bounds());
        let params = BuildingParameters {
            maximum_plane_residual_m: 0.05,
            minimum_planar_point_fraction: 0.9,
            minimum_building_area_m2: 40.,
            confidence_threshold: 0.55,
            ..Default::default()
        };
        let fit = compute_building_surface_fit(&cloud, &dem, &params).unwrap();
        let center = 12 * dem.width() + 12;
        assert!(fit.planar_point_fraction.field[center] > 0.95);
        assert!(
            fit.plane_residual.field[center] < 0.03,
            "residual was {}",
            fit.plane_residual.field[center]
        );
        let detection = detect_buildings(&fit, &params);
        assert_eq!(detection.accepted_mask.field[center], 1.);
    }

    #[test]
    fn nonplanar_candidate_is_exposed_for_boulder_review_but_trees_are_not() {
        let grid = DfmGrid::new(31, 31, 1., geo::coord! { x: 0., y: 30. }).unwrap();
        let mut dem = Dfm::<Elevation>::new(grid);
        dem.field.fill(0.);
        let make_cloud = |vegetation: bool| {
            let mut points = Vec::new();
            for y in 8..=16 {
                for x in 8..=16 {
                    let z = 4. + ((x * 17 + y * 31) % 11) as f64 * 0.23;
                    let mut point = crate::geometry::PointLaz::new(x as f64, 30. - y as f64, z);
                    point.0.return_number = 1;
                    point.0.number_of_returns = 1;
                    if vegetation {
                        point.0.classification = Classification::HighVegetation;
                    }
                    points.push(point);
                }
            }
            PointCloud::new(points, bounds())
        };
        let params = BuildingParameters {
            maximum_plane_residual_m: 0.02,
            minimum_planar_point_fraction: 0.8,
            minimum_building_area_m2: 20.,
            minimum_plane_inliers: 10,
            maximum_roof_planes: 1,
            ..Default::default()
        };
        let center = 12 * dem.width() + 12;

        let fit = compute_building_surface_fit(&make_cloud(false), &dem, &params).unwrap();
        let detection = detect_buildings(&fit, &params);
        assert_eq!(detection.accepted_mask.field[center], 0.);
        assert_eq!(detection.plane_rejected_mask.field[center], 1.);

        let tree_fit = compute_building_surface_fit(&make_cloud(true), &dem, &params).unwrap();
        let tree_detection = detect_buildings(&tree_fit, &params);
        assert_eq!(tree_detection.accepted_mask.field[center], 0.);
        assert_eq!(tree_detection.plane_rejected_mask.field[center], 0.);
    }

    #[test]
    fn raster_stair_steps_regularize_to_the_roof_direction() {
        let grid = DfmGrid::new(61, 61, 0.5, geo::coord! { x: 0., y: 15. }).unwrap();
        let mut probability = Dfm::<BuildingProbability>::new(grid.clone());
        let mut candidate_id = Dfm::<BuildingCandidateId>::new(grid.clone());
        let mut accepted_mask = Dfm::<BuildingProbability>::new(grid);
        probability.field.fill(0.);
        candidate_id.field.fill(0.);
        accepted_mask.field.fill(0.);

        let roof_angle = 17_f64.to_radians();
        for y in 0..accepted_mask.height() {
            for x in 0..accepted_mask.width() {
                let coordinate = accepted_mask.index2coord(y, x);
                let dx = coordinate.x - 15.;
                let dy = coordinate.y;
                let along = dx * roof_angle.cos() + dy * roof_angle.sin();
                let across = -dx * roof_angle.sin() + dy * roof_angle.cos();
                if along.abs() <= 6. && across.abs() <= 3. {
                    let index = y * accepted_mask.width() + x;
                    probability.field[index] = 1.;
                    candidate_id.field[index] = 1.;
                    accepted_mask.field[index] = 1.;
                }
            }
        }
        let mut plane_rejected_mask = Dfm::<BuildingProbability>::new_like(&accepted_mask);
        plane_rejected_mask.field.fill(0.);
        let detection = BuildingDetection {
            probability,
            candidate_id,
            accepted_mask,
            plane_rejected_mask,
        };
        let hull = geo::Rect::new(
            geo::coord! { x: -1., y: -16. },
            geo::coord! { x: 31., y: 16. },
        )
        .to_polygon();
        let parameters = BuildingParameters {
            regularization_simplification_tolerance_m: 0.6,
            regularization_maximum_boundary_displacement_m: 1.,
            regularization_minimum_iou: 0.75,
            ..Default::default()
        };

        let objects = building_objects(&detection, &hull, &hull, &parameters, &[]);
        assert_eq!(objects.len(), 1);
        let MapObject::Area { object, .. } = &objects[0] else {
            panic!("building detector emitted a non-area object");
        };
        let longest_edge = object
            .exterior()
            .lines()
            .max_by(|first, second| {
                first
                    .dx()
                    .hypot(first.dy())
                    .total_cmp(&second.dx().hypot(second.dy()))
            })
            .unwrap();
        let regularized_direction = longest_edge
            .dy()
            .atan2(longest_edge.dx())
            .rem_euclid(std::f64::consts::FRAC_PI_2);
        assert!((regularized_direction - roof_angle).abs().to_degrees() < 2.);
    }
}
