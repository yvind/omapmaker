use crate::{
    CELL_SIZE_METERS, TILE_SIZE_PIXELS,
    geometry::PointCloud,
    raster::{
        Dfm,
        dfm::{Elevation, Water},
    },
    statistics::LidarStats,
};

// A 3 m neighborhood is large enough to estimate a plane even where water
// returns are sparse, while still preserving the banks of small ponds.
const WATER_RADIUS_METERS: f64 = 3.;
const MIN_PLANE_RETURNS: f64 = 6.;
const SPARSE_DENSITY_PERCENT_OF_AVERAGE: f64 = 0.25;
const PLANAR_RMSE_METERS: f64 = 0.10;
// Water surfaces are level. A slope of 0.015 is approximately a 0.86 degree
// incline and already receives a strong penalty.
const LEVEL_PLANE_SLOPE: f64 = 0.015;

/// Estimate the probability of water in every raster cell.
///
/// Only single returns are used. The estimate combines the three properties
/// expected of water returns: low spatial density, weak intensity relative to
/// the data set, and a level locally fitted plane with a small residual.
pub fn compute_water_probability(
    all_point_cloud: &PointCloud,
    dem: &Dfm<Elevation>,
    stats: &LidarStats,
) -> Dfm<Water> {
    let side = TILE_SIZE_PIXELS;
    let len = side * side;
    let mut count = vec![0.; len];
    let mut intensity = vec![0.; len];
    let mut x = vec![0.; len];
    let mut y = vec![0.; len];
    let mut z = vec![0.; len];
    let mut xx = vec![0.; len];
    let mut xy = vec![0.; len];
    let mut yy = vec![0.; len];
    let mut xz = vec![0.; len];
    let mut yz = vec![0.; len];
    let mut zz = vec![0.; len];

    for point in all_point_cloud
        .points
        .iter()
        .filter(|point| point.0.return_number == 1 && point.0.number_of_returns == 1)
    {
        let xi = ((point.x() - dem.tl_coord.x) / CELL_SIZE_METERS).round() as isize;
        let yi = ((dem.tl_coord.y - point.y()) / CELL_SIZE_METERS).round() as isize;
        if xi < 0 || yi < 0 || xi >= side as isize || yi >= side as isize {
            continue;
        }

        let index = yi as usize * side + xi as usize;
        // Work in tile-local coordinates to keep the plane-fit covariance
        // numerically stable even when the source CRS has large coordinates.
        let px = point.x() - dem.tl_coord.x;
        let py = dem.tl_coord.y - point.y();
        let pz = point.0.z;
        count[index] += 1.;
        intensity[index] += f64::from(point.0.intensity);
        x[index] += px;
        y[index] += py;
        z[index] += pz;
        xx[index] += px * px;
        xy[index] += px * py;
        yy[index] += py * py;
        xz[index] += px * pz;
        yz[index] += py * pz;
        zz[index] += pz * pz;
    }

    let fields =
        [count, intensity, x, y, z, xx, xy, yy, xz, yz, zz].map(|field| summed_area_table(&field));
    let radius = (WATER_RADIUS_METERS / CELL_SIZE_METERS).ceil() as usize;
    let mut water = Dfm::<Water>::new_like(dem);

    for yi in 0..side {
        let top = yi.saturating_sub(radius);
        let bottom = (yi + radius + 1).min(side);
        for xi in 0..side {
            let left = xi.saturating_sub(radius);
            let right = (xi + radius + 1).min(side);
            let sums = fields
                .each_ref()
                .map(|field| rectangle_sum(field, top, bottom, left, right));
            let [
                n,
                sum_i,
                sx,
                sy,
                sz,
                sxx_raw,
                sxy_raw,
                syy_raw,
                sxz_raw,
                syz_raw,
                szz_raw,
            ] = sums;

            if n < MIN_PLANE_RETURNS {
                water[(yi, xi)] = 0.;
                continue;
            }

            let sxx = sxx_raw - sx * sx / n;
            let sxy = sxy_raw - sx * sy / n;
            let syy = syy_raw - sy * sy / n;
            let sxz = sxz_raw - sx * sz / n;
            let syz = syz_raw - sy * sz / n;
            let szz = (szz_raw - sz * sz / n).max(0.);
            let determinant = sxx * syy - sxy * sxy;
            if determinant <= f64::EPSILON {
                water[(yi, xi)] = 0.;
                continue;
            }

            let plane_x = (sxz * syy - syz * sxy) / determinant;
            let plane_y = (syz * sxx - sxz * sxy) / determinant;
            let residual_sum = (szz - plane_x * sxz - plane_y * syz).max(0.);
            let plane_rmse = (residual_sum / n).sqrt();
            let plane_slope = plane_x.hypot(plane_y);
            let area = ((bottom - top) * (right - left)) as f64 * CELL_SIZE_METERS.powi(2);
            let density = n / area;
            let mean_intensity = sum_i / n;

            water[(yi, xi)] =
                water_likelihood(density, mean_intensity, plane_rmse, plane_slope, n, stats);
        }
    }

    water
}

fn water_likelihood(
    density: f64,
    mean_intensity: f64,
    plane_rmse: f64,
    plane_slope: f64,
    returns: f64,
    stats: &LidarStats,
) -> f64 {
    let sparse_density =
        (stats.average_density * SPARSE_DENSITY_PERCENT_OF_AVERAGE).max(f64::EPSILON);
    let density_score = 1. / (1. + (density / sparse_density).powi(2));
    let planarity_score = (-(plane_rmse / PLANAR_RMSE_METERS).powi(2)).exp();
    let level_score = (-(plane_slope / LEVEL_PLANE_SLOPE).powi(2)).exp();
    let flatness_score = planarity_score * level_score;

    let intensity_scale = stats.intensity.std_dev.max(1.);
    let weak_boundary = stats.intensity.mean - 0.5 * intensity_scale;
    let intensity_score = 1. / (1. + ((mean_intensity - weak_boundary) / intensity_scale).exp());

    let evidence = 1. - (-(returns - MIN_PLANE_RETURNS + 1.) / MIN_PLANE_RETURNS).exp();
    // Plane residual and, especially, a level fitted plane dominate the
    // result. Density and intensity are supporting evidence rather than
    // enough to classify a sloping surface as water.
    (flatness_score.powf(0.70) * density_score.powf(0.15) * intensity_score.powf(0.15))
        .mul_add(evidence, 0.)
        .clamp(0., 1.)
}

fn summed_area_table(values: &[f64]) -> Vec<f64> {
    let stride = TILE_SIZE_PIXELS + 1;
    let mut table = vec![0.; stride * stride];
    for y in 0..TILE_SIZE_PIXELS {
        let mut row_sum = 0.;
        for x in 0..TILE_SIZE_PIXELS {
            row_sum += values[y * TILE_SIZE_PIXELS + x];
            table[(y + 1) * stride + x + 1] = table[y * stride + x + 1] + row_sum;
        }
    }
    table
}

fn rectangle_sum(table: &[f64], top: usize, bottom: usize, left: usize, right: usize) -> f64 {
    let stride = TILE_SIZE_PIXELS + 1;
    table[bottom * stride + right] + table[top * stride + left]
        - table[top * stride + right]
        - table[bottom * stride + left]
}
