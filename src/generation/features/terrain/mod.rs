use crate::TILE_SIZE_PIXELS;
use crate::geometry::{PointCloud, PointLaz};
use crate::lidar::LidarStats;
use crate::parameters::VegetationWeights;
use crate::raster::{
    Dfm, Elevation, FilteredSurface, Ground, GroundPointDensity, GroundRelief2m, GroundRelief5m,
    HardObjectConfidence, HardObjectHeight, HeightAboveGround, HighVegetation, Intensity,
    LastReturn, LowVegetation, MediumVegetation, Ndvd, PointDensity, RasterMarker, Returns,
    SurfaceObjects, VegetationLikelihood, Water,
};

use super::{
    surface_features::point_cell_and_height,
    vegetation::{HeightBand, height_band},
};
use las::point::Classification;
use spade::DelaunayTriangulation;

const CHM_SPIKINESS: f64 = 8.;
const CHM_HILLSHADE_SUN_ANGLE: f64 = 3. * std::f64::consts::FRAC_PI_4;
const VEGETATION_DENSITY_RADIUS_METERS: f64 = 2.;

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
    pub ground_relief_2m: Dfm<GroundRelief2m>,
    pub ground_relief_5m: Dfm<GroundRelief5m>,
    pub hard_object_height: Dfm<HardObjectHeight>,
    pub hard_object_confidence: Dfm<HardObjectConfidence>,
    pub vegetation_likelihood: Dfm<VegetationLikelihood>,
    pub filtered_surface: Dfm<FilteredSurface>,
    pub water: Dfm<Water>,
    pub canopy_height: Dfm<HeightAboveGround>,
    pub point_density: Dfm<PointDensity>,
    pub ground_point_density: Dfm<GroundPointDensity>,
    pub z_range: (f32, f32),
    pub ground_cover: super::GroundCoverRasters,
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
    // Preserve direct observation support before the ground points are moved
    // into the interpolation triangulation. This raster is never smoothed in
    // place; feature detectors derive their own metre-scale support products.
    let ground_density = ground_cloud.observed_point_density(&dem);
    let mut ground_point_density = Dfm::<GroundPointDensity>::new_like(&dem);
    ground_point_density
        .field
        .copy_from_slice(&ground_density.field);

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
    let z_range = (z_range.0 as f32, z_range.1 as f32);

    let dt = DelaunayTriangulation::<PointLaz>::bulk_load_stable(ground_cloud.points)?;
    let nn = dt.natural_neighbor();

    for y_index in 0..TILE_SIZE_PIXELS {
        for x_index in 0..TILE_SIZE_PIXELS {
            let coords = dem.index2spade(y_index, x_index);

            // all points inside the point cloud's convex hull gets interpolated
            // this is problematic if the pc has a hole on a corner, fixed by adding points to the corners of the dem through IDW extrapolation
            if let Some(elev) = nn.interpolate(|p| p.data().0.z, coords) {
                dem[(y_index, x_index)] = elev as f32;
            } else {
                anyhow::bail!(
                    "Interpolation point ({x_index}, {y_index}) is outside of the point cloud hull"
                );
            }
            if let Some(rn) = nn.interpolate(|p| p.data().0.return_number as f64, coords) {
                drm[(y_index, x_index)] = rn as f32;
            }
            if let Some(int) = nn.interpolate(|p| p.data().0.intensity as f64, coords) {
                dim[(y_index, x_index)] = int as f32;
            }
        }
    }

    // Keep these canonical rasters unsmoothed. Terrain and scalar filtering
    // belong to their feature-specific derived products.

    // normalize the return numbers
    for r in drm.field.iter_mut() {
        *r = stats.return_number.normalize(*r);
    }

    // normalize the intensity
    for i in dim.field.iter_mut() {
        *i = stats.intensity.normalize(*i);
    }

    let canopy_height = all_point_cloud.canopy_height_model(&dem, CHM_SPIKINESS);
    let point_density = all_point_cloud.point_density(&dem);

    let last_return_cloud = filter_last_returns(all_point_cloud);
    let last_return = last_return_cloud
        .canopy_height_model(&dem, CHM_SPIKINESS)
        .hillshade_as::<LastReturn>(CHM_HILLSHADE_SUN_ANGLE);

    let super::SurfaceFeatureRasters {
        ground_relief_2m,
        ground_relief_5m,
        hard_object_height,
        hard_object_confidence,
        vegetation_likelihood,
        filtered_surface,
    } = super::compute_surface_features(all_point_cloud, &dem);
    let ground_cover = super::compute_ground_cover(all_point_cloud, &dem, &ground_relief_2m);
    // Preserve the legacy display product, now rendered from the actual
    // vegetation-filtered elevation surface instead of a mean of low last
    // returns. The raw metric products above remain available independently.
    let surface_objects = filtered_surface.hillshade_as::<SurfaceObjects>(CHM_HILLSHADE_SUN_ANGLE);

    let vegetation_density =
        compute_vegetation_density_dfms(all_point_cloud, &dem, VEGETATION_DENSITY_RADIUS_METERS);
    let water = super::compute_water_probability(all_point_cloud, &dem, &dim, &canopy_height);

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
        ground_relief_2m,
        ground_relief_5m,
        hard_object_height,
        hard_object_confidence,
        vegetation_likelihood,
        filtered_surface,
        water,
        canopy_height,
        point_density,
        ground_point_density,
        z_range,
        ground_cover,
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

fn compute_vegetation_density_dfms(
    point_cloud: &PointCloud,
    dem: &Dfm<Elevation>,
    radius_meters: f64,
) -> VegetationDensityDfms {
    let mut ground_sums = vec![0_f64; dem.field.len()];
    let mut low_sums = vec![0_f64; dem.field.len()];
    let mut medium_sums = vec![0_f64; dem.field.len()];
    let mut high_sums = vec![0_f64; dem.field.len()];
    let mut total_sums = vec![0_f64; dem.field.len()];

    let radius_cells = (radius_meters / dem.grid.cell_size_m).ceil() as isize;
    let radius2 = radius_meters.powi(2);
    let sigma = (radius_meters / 2.).max(dem.grid.cell_size_m);
    let two_sigma2 = 2. * sigma.powi(2);

    for point in point_cloud.points.iter() {
        if point.0.is_withheld
            || matches!(
                point.0.classification,
                Classification::LowPoint
                    | Classification::HighNoise
                    | Classification::Water
                    | Classification::Building
            )
        {
            continue;
        }
        // Share both the ground sample and height bands with 409, including
        // on slopes where a nearest-cell DEM would give a different height.
        let Some((_, height)) = point_cell_and_height(point, dem) else {
            continue;
        };
        let band = height_band(height);
        let x_center = ((point.x() - dem.grid.top_left.x) / dem.grid.cell_size_m).round() as isize;
        let y_center = ((dem.grid.top_left.y - point.y()) / dem.grid.cell_size_m).round() as isize;

        if x_center < -radius_cells
            || y_center < -radius_cells
            || x_center >= dem.width() as isize + radius_cells
            || y_center >= dem.height() as isize + radius_cells
        {
            continue;
        }

        let y_min = (y_center - radius_cells).max(0) as usize;
        let y_max = (y_center + radius_cells).min(dem.height() as isize - 1) as usize;
        let x_min = (x_center - radius_cells).max(0) as usize;
        let x_max = (x_center + radius_cells).min(dem.width() as isize - 1) as usize;

        for yi in y_min..=y_max {
            for xi in x_min..=x_max {
                let cell_coord = dem.index2coord(yi, xi);
                let dist2 = (point.x() - cell_coord.x).powi(2) + (point.y() - cell_coord.y).powi(2);
                if dist2 > radius2 {
                    continue;
                }

                let weight = (-dist2 / two_sigma2).exp();
                let index = yi * dem.width() + xi;
                total_sums[index] += weight;

                match band {
                    Some(HeightBand::Ground) => ground_sums[index] += weight,
                    Some(HeightBand::Low) => low_sums[index] += weight,
                    Some(HeightBand::Medium) => medium_sums[index] += weight,
                    Some(HeightBand::High) => high_sums[index] += weight,
                    // 409 is scored separately, then resolved against green
                    // once. Its hits must not also increase the green score.
                    Some(HeightBand::Undergrowth) | None => (),
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

fn normalize_density_dfm<T: RasterMarker, U: RasterMarker>(
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
            (*band / *total) as f32
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
        let g = f64::from(ground.field[index]);
        let v = f64::from(weights.low) * f64::from(low.field[index])
            + f64::from(weights.medium) * f64::from(medium.field[index])
            + f64::from(weights.high) * f64::from(high.field[index]);
        let denominator = v + g;
        ndvd.field[index] = if denominator > f64::EPSILON {
            ((((v - g) / denominator) + 1.) / 2.) as f32
        } else {
            0.
        };
    }

    ndvd
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::raster::DfmGrid;

    fn sample(height: f64, slope: f32) -> (PointCloud, Dfm<Elevation>) {
        let mut dem =
            Dfm::<Elevation>::new(DfmGrid::new(21, 21, 0.5, geo::coord! {x:0.,y:10.}).unwrap());
        for y in 0..dem.height() {
            for x in 0..dem.width() {
                dem[(y, x)] = slope * x as f32;
            }
        }
        let mut point = PointLaz::new(5.2, 4.8, 0.);
        point.0.z = f64::from(
            dem.sample_bilinear(geo::coord! {x:point.x(),y:point.y()})
                .unwrap(),
        ) + height;
        point.0.classification = Classification::Unclassified;
        point.0.is_synthetic = false;
        (PointCloud::new(vec![point], las::Bounds::default()), dem)
    }

    #[test]
    fn undergrowth_hits_do_not_feed_the_green_bands_even_on_a_slope() {
        for slope in [0., 0.7] {
            for height in [0.2, 0.4, 0.79] {
                let (cloud, dem) = sample(height, slope);
                let bands = compute_vegetation_density_dfms(&cloud, &dem, 2.);
                assert!(
                    bands.low.field.iter().all(|&v| v == 0.),
                    "height {height}, slope {slope}"
                );
                assert!(bands.medium.field.iter().all(|&v| v == 0.));
                assert!(bands.high.field.iter().all(|&v| v == 0.));
                assert!(
                    compute_ndvd(
                        &bands.ground,
                        &bands.low,
                        &bands.medium,
                        &bands.high,
                        VegetationWeights::default()
                    )
                    .field
                    .iter()
                    .all(|&v| v == 0.)
                );
            }
        }
        for height in [0.8, 1.5, 1.799] {
            let (cloud, dem) = sample(height, 0.);
            let bands = compute_vegetation_density_dfms(&cloud, &dem, 2.);
            assert!(bands.low.field.iter().any(|&v| v > 0.), "height {height}");
            assert!(bands.medium.field.iter().all(|&v| v == 0.));
        }
        let (cloud, dem) = sample(1.8, 0.);
        let bands = compute_vegetation_density_dfms(&cloud, &dem, 2.);
        assert!(bands.low.field.iter().all(|&v| v == 0.));
        assert!(bands.medium.field.iter().any(|&v| v > 0.));
    }

    #[test]
    fn low_vegetation_uses_heights_and_ignores_untrusted_labels() {
        let (base, dem) = sample(1., 0.);
        let expected = compute_vegetation_density_dfms(&base, &dem, 2.);
        for label in [
            Classification::Unclassified,
            Classification::LowVegetation,
            Classification::MediumVegetation,
            Classification::HighVegetation,
        ] {
            let mut cloud = base.clone();
            cloud.points[0].0.classification = label;
            let actual = compute_vegetation_density_dfms(&cloud, &dem, 2.);
            assert_eq!(actual.low.field, expected.low.field);
        }
        for label in [
            Classification::LowPoint,
            Classification::HighNoise,
            Classification::Water,
            Classification::Building,
        ] {
            let mut cloud = base.clone();
            cloud.points[0].0.classification = label;
            let actual = compute_vegetation_density_dfms(&cloud, &dem, 2.);
            assert!(actual.low.field.iter().all(|&v| v == 0.));
        }
    }
}
