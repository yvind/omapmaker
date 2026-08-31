use std::sync::{Arc, Mutex};

use anyhow::Context;

use crate::{
    Result,
    parameters::FileParameters,
    progress::ProgressReporter,
    raster::{Dfm, RasterMarker},
};

pub(super) fn push_saved_raster<T: RasterMarker>(
    saved_rasters: &Arc<Mutex<Vec<Dfm<T>>>>,
    raster: Dfm<T>,
    label: &str,
    reporter: &dyn ProgressReporter,
) -> bool {
    if let Ok(mut rasters) = saved_rasters.lock() {
        rasters.push(raster);
        true
    } else {
        reporter.error(format!("{label} raster mutex was poisoned"));
        false
    }
}

pub(super) fn write_saved_rasters<T: RasterMarker>(
    reporter: &dyn ProgressReporter,
    saved_rasters: Option<Arc<Mutex<Vec<Dfm<T>>>>>,
    naming: (&str, &str),
    file_params: &FileParameters,
    ref_point: geo::Coord,
    crs: Option<&proj_core::CrsDef>,
) -> Result<()> {
    let (label, suffix) = naming;
    let Some(saved_rasters) = saved_rasters else {
        return Ok(());
    };

    let rasters = Arc::<Mutex<Vec<Dfm<T>>>>::into_inner(saved_rasters)
        .with_context(|| {
            format!("Could not get saved {label} rasters; a worker still holds a reference")
        })?
        .into_inner()
        .map_err(|_| anyhow::anyhow!("{label} raster mutex was poisoned during generation"))?;

    if rasters.is_empty() {
        return Ok(());
    }

    let raw_values = file_params.write_raw_raster_values;
    let encoding = if raw_values {
        "raw-valued float32"
    } else {
        "viewer-scaled 8-bit"
    };
    reporter.log(format!("Writing {label} {encoding} GeoTIFF..."));
    let path = if raw_values {
        crate::raster::geotiff::write_merged_dfm_geotiff_f32(
            &file_params.save_location,
            suffix,
            &rasters,
            ref_point,
            crs,
        )?
    } else {
        crate::raster::geotiff::write_merged_dfm_geotiff(
            &file_params.save_location,
            suffix,
            &rasters,
            ref_point,
            crs,
        )?
    };
    reporter.log(format!("Wrote {label} raster to {}", path.display()));

    Ok(())
}

/// Display-only rasters deliberately bypass the raw-value toggle. Their
/// samples are render products or categorical diagnostics rather than a
/// physical/numeric field that downstream GIS analysis should consume.
pub(super) fn write_saved_viewer_rasters<T: RasterMarker>(
    reporter: &dyn ProgressReporter,
    saved_rasters: Option<Arc<Mutex<Vec<Dfm<T>>>>>,
    naming: (&str, &str),
    file_params: &FileParameters,
    ref_point: geo::Coord,
    crs: Option<&proj_core::CrsDef>,
) -> Result<()> {
    let (label, suffix) = naming;
    let Some(saved_rasters) = saved_rasters else {
        return Ok(());
    };

    let rasters = Arc::<Mutex<Vec<Dfm<T>>>>::into_inner(saved_rasters)
        .with_context(|| {
            format!("Could not get saved {label} rasters; a worker still holds a reference")
        })?
        .into_inner()
        .map_err(|_| anyhow::anyhow!("{label} raster mutex was poisoned during generation"))?;

    if rasters.is_empty() {
        return Ok(());
    }

    reporter.log(format!("Writing {label} GeoTIFF..."));
    let path = crate::raster::geotiff::write_merged_dfm_geotiff(
        &file_params.save_location,
        suffix,
        &rasters,
        ref_point,
        crs,
    )?;
    reporter.log(format!("Wrote {label} raster to {}", path.display()));

    Ok(())
}
