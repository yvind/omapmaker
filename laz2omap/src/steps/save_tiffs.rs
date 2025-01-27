use crate::raster::Dfm;

use geo::Coord;

use std::{ffi::OsStr, ffi::OsString, path::Path};

pub fn save_tiffs(
    dem: Dfm,
    grad_dem: Dfm,
    dim: Dfm,
    drm: Dfm,
    ref_point: &Coord,
    file_stem: &OsStr,
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
