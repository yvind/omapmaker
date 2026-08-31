use std::collections::VecDeque;

use las::point::Classification;

use crate::{
    geometry::PointCloud,
    parameters::{BuildingClassificationEvidence, BuildingParameters},
    raster::{
        Dfm, ElevatedPointCount, Elevation, HeightAboveGroundMax, HeightAboveGroundMean,
        PlanarPointFraction, PlaneResidual, SurfaceNormalX, SurfaceNormalY, SurfaceNormalZ,
    },
};

use super::detection::{close_mask, fill_small_holes, neighbors8};
use super::{BuildingSurfaceFit, FIT_ELEVATION_CEILING_M, MINIMUM_RANSAC_SAMPLE_SIZE};

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
