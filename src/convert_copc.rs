use crate::{Result, comms::messages::*, statistics::LidarStats};
use anyhow::{Context, bail};
use std::{
    path::{Path, PathBuf},
    sync::mpsc::Sender,
    vec,
};

use copc_converter::{NodeStorage, Pipeline, PipelineConfig, TempCompression};
use copc_rs::CopcReader;
use geo::Intersects;
use proj_core::CrsDef;

const DEFAULT_COPC_MEMORY_BUDGET: u64 = 8_u64 * 1024 * 1024 * 1024;

// should be multithreaded
pub fn convert_copc(
    sender: Sender<FrontendTask>,
    paths: Vec<PathBuf>,
    input_crs: Vec<Option<CrsDef>>,
    output_crs: Option<CrsDef>,
    save_location: PathBuf,
    boundaries: Vec<[walkers::Position; 4]>,
    polygon_filter: geo::LineString,
    write_single_copc: bool,
) {
    if let Err(e) = try_convert_copc(
        sender.clone(),
        paths,
        input_crs,
        output_crs,
        save_location,
        boundaries,
        polygon_filter,
        write_single_copc,
    ) {
        let _ = sender.send(FrontendTask::ProgressBar(ProgressBar::Finish));
        let _ = sender.send(FrontendTask::Error(e.to_string(), true));
    }
}

fn try_convert_copc(
    sender: Sender<FrontendTask>,
    paths: Vec<PathBuf>,
    input_crs: Vec<Option<CrsDef>>,
    output_crs: Option<CrsDef>,
    save_location: PathBuf,
    boundaries: Vec<[walkers::Position; 4]>,
    polygon_filter: geo::LineString,
    write_single_copc: bool,
) -> Result<()> {
    let mut new_paths = paths.clone();
    let mut relevant_paths = Vec::new();

    let _ = sender.send(FrontendTask::Log(
        "Gathering statistics and Converting files...".to_string(),
    ));
    let _ = sender.send(FrontendTask::ProgressBar(ProgressBar::Start));

    let polygon = geo::Polygon::new(polygon_filter, vec![]);

    let mut stats = Vec::new();
    let inc_size = 1. / paths.len() as f32;
    for (pi, path) in paths.iter().cloned().enumerate() {
        // first check if the file is relevant i.e overlaps with the polygon
        let bounds = boundaries[pi];

        let relevant =
            polygon.exterior().0.is_empty() || polygon.intersects(&boundary_polygon(bounds));

        if relevant {
            stats.push(
                LidarStats::calculate_statistics(&path)
                    .with_context(|| format!("Failed to calculate statistics for {path:?}"))?,
            );

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
                convert_file(path, input_crs[pi].clone(), sender.clone())?
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
        }

        let _ = sender.send(FrontendTask::ProgressBar(ProgressBar::Inc(inc_size)));
    }

    if stats.is_empty() {
        let _ = sender.send(FrontendTask::ProgressBar(ProgressBar::Finish));
        let _ = sender.send(FrontendTask::Error(
            "The chosen polygon filter does not intersect the lidar files".to_string(),
            false,
        ));
        return Ok(());
    }

    if write_single_copc {
        let mut merged_path = save_location;
        merged_path.set_extension("copc.laz");

        let _ = sender.send(FrontendTask::Log(format!(
            "Writing {} relevant lidar files to {:?}",
            relevant_paths.len(),
            merged_path
        )));

        run_copc_converter(&relevant_paths, &merged_path)
            .with_context(|| format!("Failed to write merged COPC to {merged_path:?}"))?;

        let _ = sender.send(FrontendTask::UpdateVariable(Variable::SingleCopcPath(
            merged_path,
        )));
    }

    let stats = stats
        .into_iter()
        .reduce(LidarStats::combine_stats)
        .context("No lidar statistics were produced")?;

    let _ = sender.send(FrontendTask::ProgressBar(ProgressBar::Finish));

    let _ = sender.send(FrontendTask::UpdateVariable(Variable::Stats(stats)));

    let _ = sender.send(FrontendTask::UpdateVariable(Variable::Paths(new_paths)));

    let _ = sender.send(FrontendTask::TaskComplete(TaskDone::ConvertCopc));

    Ok(())
}

fn boundary_polygon(bounds: [walkers::Position; 4]) -> geo::Polygon {
    geo::Polygon::new(
        geo::LineString::new(vec![
            bounds[0].0,
            bounds[1].0,
            bounds[2].0,
            bounds[3].0,
            bounds[0].0,
        ]),
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
    _sender: Sender<FrontendTask>,
) -> Result<PathBuf> {
    let raw_path = path.clone();
    path.set_extension("copc.laz");

    run_copc_converter(&[raw_path], &path)
        .with_context(|| format!("Failed to convert lidar file to COPC at {path:?}"))?;
    Ok(path)
}

fn run_copc_converter(input_files: &[PathBuf], output_path: &Path) -> copc_converter::Result<()> {
    let config = PipelineConfig {
        memory_budget: DEFAULT_COPC_MEMORY_BUDGET,
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
