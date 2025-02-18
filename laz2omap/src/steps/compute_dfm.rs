use crate::geometry::{PointCloud, PointLaz};
use crate::raster::Dfm;
use crate::SIDE_LENGTH;

use geo::Coord;
use spade::DelaunayTriangulation;

pub fn compute_dfms(ground_cloud: PointCloud, tl: Coord) -> (Dfm, Dfm) {
    let mut dem = Dfm::new(tl);
    let mut drm = Dfm::new(tl);

    let dt = DelaunayTriangulation::<PointLaz>::bulk_load_stable(ground_cloud.points).unwrap();
    let nn = dt.natural_neighbor();

    for y_index in 0..SIDE_LENGTH {
        for x_index in 0..SIDE_LENGTH {
            let coords = dem.index2spade(y_index, x_index);

            // all points inside the point cloud's convex hull gets interpolated
            // this is problematic if the pc has a hole on a corner
            if let Some(elev) = nn.interpolate(|p| p.data().0.z, coords) {
                dem[(y_index, x_index)] = elev;
            }
            if let Some(rn) = nn.interpolate(|p| p.data().0.return_number as f64, coords) {
                drm[(y_index, x_index)] = (rn - 1.) / 5.; // want a range between 0-1 for veg and this basic algo does not do that;
            }
        }
    }
    (dem, drm)
}
