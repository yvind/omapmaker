use crate::{
    Result,
    lidar::LidarStats,
    progress::{ProgressReporter, ProgressUpdate},
};
use anyhow::{Context, bail};
use std::{
    path::{Path, PathBuf},
    vec,
};

use copc_converter::{NodeStorage, Pipeline, PipelineConfig, TempCompression};
use geo::Intersects;
use las::CopcReader;
use proj_core::CrsDef;

pub(crate) enum CopcConversionOutcome {
    Converted {
        paths: Vec<PathBuf>,
        stats: LidarStats,
        single_copc_path: Option<PathBuf>,
    },
    NoIntersection,
}

#[allow(clippy::too_many_arguments)]
pub fn convert_copc(
    reporter: &dyn ProgressReporter,
    paths: Vec<PathBuf>,
    input_crs: Vec<Option<CrsDef>>,
    output_crs: Option<CrsDef>,
    save_location: PathBuf,
    boundaries: Vec<[geo::Coord; 4]>,
    polygon_filter: geo::LineString,
    write_single_copc: bool,
    memory_budget: u8,
) -> Result<CopcConversionOutcome> {
    try_convert_copc(
        reporter,
        paths,
        input_crs,
        output_crs,
        save_location,
        boundaries,
        polygon_filter,
        write_single_copc,
        memory_budget,
    )
}

#[allow(clippy::too_many_arguments)]
fn try_convert_copc(
    reporter: &dyn ProgressReporter,
    paths: Vec<PathBuf>,
    input_crs: Vec<Option<CrsDef>>,
    output_crs: Option<CrsDef>,
    save_location: PathBuf,
    boundaries: Vec<[geo::Coord; 4]>,
    polygon_filter: geo::LineString,
    write_single_copc: bool,
    memory_budget: u8,
) -> Result<CopcConversionOutcome> {
    let mut new_paths = paths.clone();
    let mut relevant_paths = Vec::new();

    let memory_budget = memory_budget as u64 * 1024 * 1024 * 1024;

    reporter.log("Gathering statistics and converting files...".to_string());
    reporter.progress(ProgressUpdate::Start);

    let polygon = geo::Polygon::new(polygon_filter, vec![]);

    let mut stats = Vec::new();
    let inc_size = 1. / paths.len() as f32;
    for (pi, path) in paths.iter().cloned().enumerate() {
        // first check if the file is relevant i.e overlaps with the polygon
        let bounds = boundaries[pi];

        let relevant =
            polygon.exterior().0.is_empty() || polygon.intersects(&boundary_polygon(bounds));

        if relevant {
            let transform_needed =
                if let (Some(input), Some(output)) = (&input_crs[pi], &output_crs) {
                    input.epsg() != output.epsg()
                } else {
                    false
                };

            let conversion_needed = CopcReader::from_path(&path).is_err();

            new_paths[pi] = if !conversion_needed && !transform_needed {
                // the lidar file is both a COPC and in the correct CRS
                path
            } else if transform_needed && !conversion_needed {
                // the lidar file needs to be transformed into another CRS but is already a copc
                transform_file(
                    path,
                    input_crs[pi].clone(),
                    output_crs
                        .clone()
                        .context("Output CRS is required when transforming COPC files")?,
                )?
            } else if conversion_needed && !transform_needed {
                // the lidar file needs to be converted to copc
                convert_file(path, input_crs[pi].clone(), memory_budget)?
            } else {
                // the lidar file needs both to be transformed into another CRS and written to COPC
                convert_and_transform_file(
                    path,
                    input_crs[pi].clone(),
                    output_crs
                        .clone()
                        .context("Output CRS is required when converting and transforming files")?,
                )?
            };

            if write_single_copc {
                relevant_paths.push(new_paths[pi].clone());
            }

            stats.push(
                LidarStats::calculate_statistics(&new_paths[pi]).with_context(|| {
                    format!("Failed to calculate statistics for {:?}", new_paths[pi])
                })?,
            );
        }

        reporter.progress(ProgressUpdate::Advance(inc_size));
    }

    if stats.is_empty() {
        reporter.progress(ProgressUpdate::Finish);
        return Ok(CopcConversionOutcome::NoIntersection);
    }

    let single_copc_path = if write_single_copc {
        let mut merged_path = save_location;
        merged_path.set_extension("copc.laz");

        reporter.log(format!(
            "Writing {} relevant lidar files to {:?}",
            relevant_paths.len(),
            merged_path
        ));

        run_copc_converter(&relevant_paths, &merged_path, memory_budget)
            .with_context(|| format!("Failed to write merged COPC to {merged_path:?}"))?;

        Some(merged_path)
    } else {
        None
    };

    let stats = stats
        .into_iter()
        .reduce(LidarStats::combine_stats)
        .context("No lidar statistics were produced")?;

    reporter.progress(ProgressUpdate::Finish);

    Ok(CopcConversionOutcome::Converted {
        paths: new_paths,
        stats,
        single_copc_path,
    })
}

fn boundary_polygon(bounds: [geo::Coord; 4]) -> geo::Polygon {
    geo::Polygon::new(
        geo::LineString::new(vec![bounds[0], bounds[1], bounds[2], bounds[3], bounds[0]]),
        vec![],
    )
}

fn transform_file(
    _path: PathBuf,
    _current_crs: Option<CrsDef>,
    _out_crs: CrsDef,
) -> Result<PathBuf> {
    bail!("Transforming CRS is not yet supported");
}

fn convert_file(
    mut path: PathBuf,
    _current_crs: Option<CrsDef>,
    memory_budget: u64,
) -> Result<PathBuf> {
    let raw_path = path.clone();
    path.set_extension("copc.laz");

    run_copc_converter(&[raw_path], &path, memory_budget)
        .with_context(|| format!("Failed to convert lidar file to COPC at {path:?}"))?;
    Ok(path)
}

fn run_copc_converter(
    input_files: &[PathBuf],
    output_path: &Path,
    budget: u64,
) -> copc_converter::Result<()> {
    let config = PipelineConfig {
        memory_budget: budget,
        temp_dir: None,
        temporal_index: None,
        progress: None,
        chunk_target_override: None,
        temp_compression: TempCompression::None,
        node_storage: NodeStorage::Files,
    };

    Pipeline::scan(input_files, config)?
        .validate()?
        .distribute()?
        .build()?
        .write(output_path)
}

fn convert_and_transform_file(
    _path: PathBuf,
    _current_crs: Option<CrsDef>,
    _out_crs: CrsDef,
) -> Result<PathBuf> {
    bail!("Transforming CRS is not yet supported");
}
