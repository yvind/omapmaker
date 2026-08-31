use crate::{
    Result,
    progress::{ProgressReporter, ProgressUpdate},
};

use std::path::PathBuf;

use las::Reader;

pub(crate) struct CrsAnalysis {
    pub(crate) paths: Vec<PathBuf>,
    pub(crate) crs_defs: Vec<Option<proj_core::CrsDef>>,
    pub(crate) crs_less: usize,
    pub(crate) removed_unreadable: bool,
}

pub fn parse_crs(reporter: &dyn ProgressReporter, mut paths: Vec<PathBuf>) -> Result<CrsAnalysis> {
    reporter.log("Detecting CRS of all provided files...".to_string());
    reporter.progress(ProgressUpdate::Start);

    let mut crs_defs = vec![];

    let mut num_crs_less = 0;

    let inc_size = 1. / paths.len() as f32;

    let mut unreadable_path = false;

    let mut i = 0;
    while i < paths.len() {
        let reader = match Reader::from_path(&paths[i]) {
            Ok(r) => r,
            Err(_) => {
                paths.swap_remove(i);
                unreadable_path = true;
                reporter.progress(ProgressUpdate::Advance(inc_size));
                continue;
            }
        };

        let mut crs_def = None;
        if let Some(wkt) = reader.header().get_wkt_crs_bytes() {
            crs_def = str::from_utf8(wkt)
                .ok()
                .and_then(|s| proj_wkt::parse_crs(s).ok());
        }
        if crs_def.is_none()
            && let Some(geotiff) = reader.header().get_geotiff_crs().ok().flatten()
        {
            let horizontal = geotiff.get_projected_crs_geo_key_value();

            if let Some(epsg) = horizontal {
                crs_def = proj_wkt::parse_crs(&epsg.to_string()).ok();
            }
        }

        if crs_def.is_none() {
            num_crs_less += 1;
        }
        crs_defs.push(crs_def);

        reporter.progress(ProgressUpdate::Advance(inc_size));

        i += 1;
    }
    reporter.progress(ProgressUpdate::Finish);

    let num_files = paths.len();

    if paths.is_empty() {
        anyhow::bail!("None of the given files were readable as lidar files");
    }

    reporter.log(format!(
        "Successfully detected a CRS for {} out of {num_files} lidar files",
        num_files - num_crs_less
    ));

    Ok(CrsAnalysis {
        paths,
        crs_defs,
        crs_less: num_crs_less,
        removed_unreadable: unreadable_path,
    })
}
