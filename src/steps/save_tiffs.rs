use crate::{dfm::Dfm, geometry::Point2D};

use std::{ffi::OsString, path::Path, sync::Arc};

pub fn save_tiffs(
    dem: Arc<Dfm>,
    grad_dem: Arc<Dfm>,
    dim: Arc<Dfm>,
    drm: Arc<Dfm>,
    ref_point: &Point2D,
    file_stem: &OsString,
    output_directory: &Path,
) {
    let mut dem_name = OsString::from("dem_");
    dem_name.push(file_stem);

    let mut grad_dem_name = OsString::from("slope_");
    grad_dem_name.push(file_stem);

    let mut drm_name = OsString::from("drm_");
    drm_name.push(file_stem);

    let mut dim_name = OsString::from("dim_");
    dim_name.push(file_stem);

    dem.write_to_tiff(&dem_name, output_directory, ref_point);
    grad_dem.write_to_tiff(&grad_dem_name, output_directory, ref_point);
    dim.write_to_tiff(&dim_name, output_directory, ref_point);
    drm.write_to_tiff(&drm_name, output_directory, ref_point);
}
