use crate::geometry::{PointCloud, PointLaz};
use crate::raster::Dfm;
use crate::SIDE_LENGTH;

use geo::Coord;
use spade::DelaunayTriangulation;

pub fn compute_dfms(ground_cloud: PointCloud) -> (Dfm, Dfm, Dfm, (f64, f64)) {
    let dem_bounds = ground_cloud.get_dfm_dimensions();
    let tl = Coord {
        x: dem_bounds.min.x,
        y: dem_bounds.max.y,
    };

    let mut dem = Dfm::new(tl);
    let mut drm = Dfm::new(tl);
    let mut dim = Dfm::new(tl);

    let mut z_range = (f64::MAX, f64::MIN);
    let mut i_range = (u16::MAX, u16::MIN);
    let mut r_range = (u8::MAX, u8::MIN);
    for p in ground_cloud.points.iter() {
        if p.0.z > z_range.1 {
            z_range.1 = p.0.z;
        } else if p.0.z < z_range.0 {
            z_range.0 = p.0.z;
        }

        if p.0.intensity > i_range.1 {
            i_range.1 = p.0.intensity;
        } else if p.0.intensity < i_range.0 {
            i_range.0 = p.0.intensity;
        }

        if p.0.return_number > r_range.1 {
            r_range.1 = p.0.return_number;
        } else if p.0.return_number < r_range.0 {
            r_range.0 = p.0.return_number;
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

    // normalize the return numbers
    let r_range = (r_range.0 as f64, r_range.1 as f64);
    for r in drm.field.iter_mut() {
        *r = (*r - r_range.0) / r_range.1;
    }

    // normalize the intensity
    let i_range = (i_range.0 as f64, i_range.1 as f64);
    for i in dim.field.iter_mut() {
        *i = (*i - i_range.0) / i_range.1;
    }

    (dem, drm, dim, z_range)
}
