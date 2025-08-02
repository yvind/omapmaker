use crate::geometry::{PointCloud, PointLaz};
use crate::raster::Dfm;
use crate::statistics::LidarStats;
use crate::SIDE_LENGTH;

use geo::Coord;
use spade::DelaunayTriangulation;

pub fn compute_dfms(ground_cloud: PointCloud, stats: &LidarStats) -> (Dfm, Dfm, Dfm, (f64, f64)) {
    let dem_bounds = ground_cloud.get_dfm_dimensions();
    let tl = Coord {
        x: dem_bounds.min.x,
        y: dem_bounds.max.y,
    };

    let mut dem = Dfm::new(tl);
    let mut drm = Dfm::new(tl);
    let mut dim = Dfm::new(tl);

    // Because the z_bounds in the header gets wrecked by noise points
    let mut z_range = (f64::MAX, f64::MIN);
    for p in ground_cloud.points.iter() {
        if p.0.z > z_range.1 {
            z_range.1 = p.0.z;
        } else if p.0.z < z_range.0 {
            z_range.0 = p.0.z;
        }
    }

    let dt = DelaunayTriangulation::<PointLaz>::bulk_load_stable(ground_cloud.points).unwrap();
    let nn = dt.natural_neighbor();

    for y_index in 0..SIDE_LENGTH {
        for x_index in 0..SIDE_LENGTH {
            let coords = dem.index2spade(y_index, x_index);

            // all points inside the point cloud's convex hull gets interpolated
            // this is problematic if the pc has a hole on a corner, fixed by adding points to the corners of the dem through IDW extrapolation
            if let Some(elev) = nn.interpolate(|p| p.data().0.z, coords) {
                dem[(y_index, x_index)] = elev;
            } else {
                panic!("Interpolation point outside of point cloud hull!");
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

    (dem, drm, dim, z_range)
}
