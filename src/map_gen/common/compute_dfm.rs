use crate::geometry::{PointCloud, PointLaz};
use crate::parameters::VegetationWeights;
use crate::raster::Dfm;
use crate::raster::dfm::{
    Elevation, Ground, HeightAboveGround, HighVegetation, Intensity, LastReturn, LowVegetation,
    MediumVegetation, Ndvd, Returns, SurfaceObjects, Water,
};
use crate::statistics::LidarStats;
use crate::{CELL_SIZE_METERS, TILE_SIZE_PIXELS};

use spade::DelaunayTriangulation;

const CHM_SPIKINESS: f64 = 8.;
const CHM_HILLSHADE_SUN_ANGLE: f64 = 3. * std::f64::consts::FRAC_PI_4;
const VEGETATION_DENSITY_RADIUS_METERS: f64 = 2.;
const GROUND_MAX_HEIGHT_METERS: f64 = 0.2;
const LOW_VEGETATION_MAX_HEIGHT_METERS: f64 = 1.5;
const MEDIUM_VEGETATION_MAX_HEIGHT_METERS: f64 = 3.;

pub struct ComputedDfms {
    pub dem: Dfm<Elevation>,
    pub return_number: Dfm<Returns>,
    pub intensity: Dfm<Intensity>,
    pub last_return: Dfm<LastReturn>,
    pub ground_vegetation: Dfm<Ground>,
    pub low_vegetation: Dfm<LowVegetation>,
    pub medium_vegetation: Dfm<MediumVegetation>,
    pub high_vegetation: Dfm<HighVegetation>,
    pub surface_objects: Dfm<SurfaceObjects>,
    pub water: Dfm<Water>,
    pub canopy_height: Dfm<HeightAboveGround>,
    pub z_range: (f64, f64),
}

pub fn compute_dfms(
    ground_cloud: PointCloud,
    stats: &LidarStats,
    all_point_cloud: &PointCloud,
    cut_bounds: geo::Rect,
) -> crate::Result<ComputedDfms> {
    let dem_bounds = ground_cloud.get_dfm_dimensions();
    let tl = geo::Coord {
        x: dem_bounds.min.x,
        y: dem_bounds.max.y,
    };

    let mut dem = Dfm::<Elevation>::with_cut_bounds(tl, cut_bounds);
    let mut drm = Dfm::<Returns>::new_like(&dem);
    let mut dim = Dfm::<Intensity>::new_like(&dem);

    // Because the z_bounds in the header gets wrecked by noise points
    let mut z_range = (f64::MAX, f64::MIN);
    for p in ground_cloud.points.iter() {
        if p.0.z > z_range.1 {
            z_range.1 = p.0.z;
        }
        if p.0.z < z_range.0 {
            z_range.0 = p.0.z;
        }
    }

    let dt = DelaunayTriangulation::<PointLaz>::bulk_load_stable(ground_cloud.points)?;
    let nn = dt.natural_neighbor();

    for y_index in 0..TILE_SIZE_PIXELS {
        for x_index in 0..TILE_SIZE_PIXELS {
            let coords = dem.index2spade(y_index, x_index);

            // all points inside the point cloud's convex hull gets interpolated
            // this is problematic if the pc has a hole on a corner, fixed by adding points to the corners of the dem through IDW extrapolation
            if let Some(elev) = nn.interpolate(|p| p.data().0.z, coords) {
                dem[(y_index, x_index)] = elev;
            } else {
                anyhow::bail!(
                    "Interpolation point ({x_index}, {y_index}) is outside of the point cloud hull"
                );
            }
            if let Some(rn) = nn.interpolate(|p| p.data().0.return_number as f64, coords) {
                drm[(y_index, x_index)] = rn;
            }
            if let Some(int) = nn.interpolate(|p| p.data().0.intensity as f64, coords) {
                dim[(y_index, x_index)] = int;
            }
        }
    }

    // some slight smoothing to remove artifacts
    dem = dem.smoothen(15., 7, 5);
    dim = dim.smoothen(15., 7, 5);
    drm = drm.smoothen(15., 7, 5);

    // normalize the return numbers
    for r in drm.field.iter_mut() {
        *r = (*r - stats.return_number.min) / stats.return_number.max;
    }

    // normalize the intensity
    for i in dim.field.iter_mut() {
        *i = (*i - stats.intensity.min) / stats.intensity.max
    }

    let canopy_height = all_point_cloud.canopy_height_model(&dem, CHM_SPIKINESS);

    let last_return_cloud = filter_last_returns(all_point_cloud);
    let last_return = last_return_cloud
        .canopy_height_model(&dem, CHM_SPIKINESS)
        .hillshade_as::<LastReturn>(CHM_HILLSHADE_SUN_ANGLE);

    let surface_object_cloud =
        filter_height_above_ground(all_point_cloud, &dem, 0., LOW_VEGETATION_MAX_HEIGHT_METERS);
    let surface_objects = surface_object_cloud
        .canopy_height_model(&dem, CHM_SPIKINESS)
        .hillshade_as::<SurfaceObjects>(CHM_HILLSHADE_SUN_ANGLE);

    let vegetation_density =
        compute_vegetation_density_dfms(all_point_cloud, &dem, VEGETATION_DENSITY_RADIUS_METERS);
    let water = super::compute_water_probability(all_point_cloud, &dem, stats);

    Ok(ComputedDfms {
        dem,
        return_number: drm,
        intensity: dim,
        last_return,
        ground_vegetation: vegetation_density.ground,
        low_vegetation: vegetation_density.low,
        medium_vegetation: vegetation_density.medium,
        high_vegetation: vegetation_density.high,
        surface_objects,
        water,
        canopy_height,
        z_range,
    })
}

struct VegetationDensityDfms {
    ground: Dfm<Ground>,
    low: Dfm<LowVegetation>,
    medium: Dfm<MediumVegetation>,
    high: Dfm<HighVegetation>,
}

fn filter_last_returns(point_cloud: &PointCloud) -> PointCloud {
    PointCloud::new(
        point_cloud
            .points
            .iter()
            .filter(|point| {
                point.0.number_of_returns > 0 && point.0.return_number == point.0.number_of_returns
            })
            .cloned()
            .collect(),
        point_cloud.bounds,
    )
}

fn filter_height_above_ground(
    point_cloud: &PointCloud,
    dem: &Dfm<Elevation>,
    min_height: f64,
    max_height: f64,
) -> PointCloud {
    PointCloud::new(
        point_cloud
            .points
            .iter()
            .filter(|point| {
                height_above_ground(point, dem)
                    .is_some_and(|height| height >= min_height && height <= max_height)
            })
            .cloned()
            .collect(),
        point_cloud.bounds,
    )
}

fn compute_vegetation_density_dfms(
    point_cloud: &PointCloud,
    dem: &Dfm<Elevation>,
    radius_meters: f64,
) -> VegetationDensityDfms {
    let mut ground_sums = vec![0.; TILE_SIZE_PIXELS * TILE_SIZE_PIXELS];
    let mut low_sums = vec![0.; TILE_SIZE_PIXELS * TILE_SIZE_PIXELS];
    let mut medium_sums = vec![0.; TILE_SIZE_PIXELS * TILE_SIZE_PIXELS];
    let mut high_sums = vec![0.; TILE_SIZE_PIXELS * TILE_SIZE_PIXELS];
    let mut total_sums = vec![0.; TILE_SIZE_PIXELS * TILE_SIZE_PIXELS];

    let radius_cells = (radius_meters / CELL_SIZE_METERS).ceil() as isize;
    let radius2 = radius_meters.powi(2);
    let sigma = (radius_meters / 2.).max(CELL_SIZE_METERS);
    let two_sigma2 = 2. * sigma.powi(2);

    for point in point_cloud.points.iter() {
        let x_center = ((point.x() - dem.tl_coord.x) / CELL_SIZE_METERS).round() as isize;
        let y_center = ((dem.tl_coord.y - point.y()) / CELL_SIZE_METERS).round() as isize;

        if x_center < -radius_cells
            || y_center < -radius_cells
            || x_center >= TILE_SIZE_PIXELS as isize + radius_cells
            || y_center >= TILE_SIZE_PIXELS as isize + radius_cells
        {
            continue;
        }

        let dem_x = x_center.clamp(0, TILE_SIZE_PIXELS as isize - 1) as usize;
        let dem_y = y_center.clamp(0, TILE_SIZE_PIXELS as isize - 1) as usize;
        let height_above_ground = point.0.z - dem[(dem_y, dem_x)];

        let y_min = (y_center - radius_cells).max(0) as usize;
        let y_max = (y_center + radius_cells).min(TILE_SIZE_PIXELS as isize - 1) as usize;
        let x_min = (x_center - radius_cells).max(0) as usize;
        let x_max = (x_center + radius_cells).min(TILE_SIZE_PIXELS as isize - 1) as usize;

        for yi in y_min..=y_max {
            for xi in x_min..=x_max {
                let cell_coord = dem.index2coord(yi, xi);
                let dist2 = (point.x() - cell_coord.x).powi(2) + (point.y() - cell_coord.y).powi(2);
                if dist2 > radius2 {
                    continue;
                }

                let weight = (-dist2 / two_sigma2).exp();
                let index = yi * TILE_SIZE_PIXELS + xi;
                total_sums[index] += weight;

                if height_above_ground < GROUND_MAX_HEIGHT_METERS {
                    ground_sums[index] += weight;
                } else if height_above_ground < LOW_VEGETATION_MAX_HEIGHT_METERS {
                    low_sums[index] += weight;
                } else if height_above_ground < MEDIUM_VEGETATION_MAX_HEIGHT_METERS {
                    medium_sums[index] += weight;
                } else {
                    high_sums[index] += weight;
                }
            }
        }
    }

    VegetationDensityDfms {
        ground: normalize_density_dfm(dem, &ground_sums, &total_sums),
        low: normalize_density_dfm(dem, &low_sums, &total_sums),
        medium: normalize_density_dfm(dem, &medium_sums, &total_sums),
        high: normalize_density_dfm(dem, &high_sums, &total_sums),
    }
}

fn normalize_density_dfm<T: Clone, U>(
    source: &Dfm<U>,
    band_sums: &[f64],
    total_sums: &[f64],
) -> Dfm<T> {
    let mut dfm = Dfm::<T>::new_like(source);

    for ((value, band), total) in dfm
        .field
        .iter_mut()
        .zip(band_sums.iter())
        .zip(total_sums.iter())
    {
        *value = if *total > f64::EPSILON {
            *band / *total
        } else {
            0.
        };
    }

    dfm
}

pub fn compute_ndvd(
    ground: &Dfm<Ground>,
    low: &Dfm<LowVegetation>,
    medium: &Dfm<MediumVegetation>,
    high: &Dfm<HighVegetation>,
    weights: VegetationWeights,
) -> Dfm<Ndvd> {
    let mut ndvd = Dfm::<Ndvd>::new_like(ground);

    for index in 0..ndvd.field.len() {
        let g = ground.field[index];
        let v = weights.low * low.field[index]
            + weights.medium * medium.field[index]
            + weights.high * high.field[index];
        let denominator = v + g;
        ndvd.field[index] = if denominator > f64::EPSILON {
            (((v - g) / denominator) + 1.) / 2.
        } else {
            0.
        };
    }

    ndvd
}

fn height_above_ground(point: &PointLaz, dem: &Dfm<Elevation>) -> Option<f64> {
    let x_index = ((point.x() - dem.tl_coord.x) / CELL_SIZE_METERS).round() as isize;
    let y_index = ((dem.tl_coord.y - point.y()) / CELL_SIZE_METERS).round() as isize;

    if x_index < 0
        || y_index < 0
        || x_index >= TILE_SIZE_PIXELS as isize
        || y_index >= TILE_SIZE_PIXELS as isize
    {
        return None;
    }

    Some(point.0.z - dem[(y_index as usize, x_index as usize)])
}
