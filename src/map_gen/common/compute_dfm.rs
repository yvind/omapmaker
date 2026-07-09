use crate::TILE_SIZE_PIXELS;
use crate::geometry::{PointCloud, PointLaz};
use crate::raster::Dfm;
use crate::raster::dfm::{Elevation, Intensity, Returns};
use crate::statistics::LidarStats;

use spade::DelaunayTriangulation;

pub fn compute_dfms(
    ground_cloud: PointCloud,
    stats: &LidarStats,
) -> crate::Result<(Dfm<Elevation>, Dfm<Returns>, Dfm<Intensity>, (f64, f64))> {
    let dem_bounds = ground_cloud.get_dfm_dimensions();
    let tl = geo::Coord {
        x: dem_bounds.min.x,
        y: dem_bounds.max.y,
    };

    let mut dem = Dfm::<Elevation>::new(tl);
    let mut drm = Dfm::<Returns>::new(tl);
    let mut dim = Dfm::<Intensity>::new(tl);

    // Because the z_bounds in the header gets wrecked by noise points
    let mut z_range = (f64::MAX, f64::MIN);
    for p in ground_cloud.points.iter() {
        if p.0.z > z_range.1 {
            z_range.1 = p.0.z;
        } else if p.0.z < z_range.0 {
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

    Ok((dem, drm, dim, z_range))
}
