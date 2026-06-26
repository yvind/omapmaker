use crate::{comms::messages::*, statistics::LidarStats};
use std::{
    path::{Path, PathBuf},
    sync::mpsc::Sender,
    vec,
};

use copc_converter::{NodeStorage, Pipeline, PipelineConfig, TempCompression};
use copc_rs::CopcReader;
use geo::Contains;
use proj_core::CrsDef;

const DEFAULT_COPC_MEMORY_BUDGET: u64 = 8_u64 * 1024 * 1024 * 1024;

// should be multithreaded
pub fn convert_copc(
    sender: Sender<FrontendTask>,
    paths: Vec<PathBuf>,
    input_crs: Vec<Option<CrsDef>>,
    output_crs: Option<CrsDef>,
    save_location: PathBuf,
    selected_file: usize,
    boundaries: Vec<[walkers::Position; 4]>,
    polygon_filter: geo::LineString,
    write_single_copc: bool,
) {
    let mut new_paths = paths.clone();
    let mut relevant_paths = Vec::new();

    sender
        .send(FrontendTask::Log(
            "Gathering statistics and Converting files...".to_string(),
        ))
        .unwrap();
    sender
        .send(FrontendTask::ProgressBar(ProgressBar::Start))
        .unwrap();

    let polygon = geo::Polygon::new(polygon_filter, vec![]);

    let mut stats = Vec::new();
    let inc_size = 1. / paths.len() as f32;
    for (pi, path) in paths.iter().cloned().enumerate() {
        // first check if the file is relevant i.e overlaps with the polygon or is the selected file
        let bounds = boundaries[pi];

        // ugly
        let mut relevant = true;
        if pi != selected_file && !polygon.exterior().0.is_empty() {
            relevant = polygon.contains(&bounds[0])
                || polygon.contains(&bounds[1])
                || polygon.contains(&bounds[2])
                || polygon.contains(&bounds[3]);
        }

        if relevant {
            stats.push(LidarStats::calculate_statistics(&path).unwrap());
            relevant_paths.push(path.clone());

            let transform_needed =
                if let (Some(input), Some(output)) = (&input_crs[pi], &output_crs) {
                    input.epsg() != output.epsg()
                } else {
                    false
                };

            let conversion_needed = CopcReader::from_path(&path).is_err();

            if write_single_copc {
                if transform_needed && !conversion_needed {
                    // the lidar file needs to be transformed into another CRS but is already a copc
                    transform_file(path, input_crs[pi].clone(), output_crs.clone().unwrap());
                } else if transform_needed {
                    // the lidar file needs both to be transformed into another CRS and written to COPC
                    convert_and_transform_file(
                        path,
                        input_crs[pi].clone(),
                        output_crs.clone().unwrap(),
                    );
                } else if pi == selected_file && conversion_needed {
                    // Keep the selected file readable for the map preview stage.
                    new_paths[pi] = convert_file(path, input_crs[pi].clone(), sender.clone());
                }
            } else {
                new_paths[pi] = if !conversion_needed && !transform_needed {
                    // the lidar file is both a COPC and in the correct CRS
                    path
                } else if transform_needed && !conversion_needed {
                    // the lidar file needs to be transformed into another CRS but is already a copc
                    transform_file(path, input_crs[pi].clone(), output_crs.clone().unwrap())
                } else if conversion_needed && !transform_needed {
                    // the lidar file needs to be converted to copc
                    convert_file(path, input_crs[pi].clone(), sender.clone())
                } else {
                    // the lidar file needs both to be transformed into another CRS and written to COPC
                    convert_and_transform_file(
                        path,
                        input_crs[pi].clone(),
                        output_crs.clone().unwrap(),
                    )
                };
            }
        }

        sender
            .send(FrontendTask::ProgressBar(ProgressBar::Inc(inc_size)))
            .unwrap()
    }

    if write_single_copc {
        let mut merged_path = save_location;
        merged_path.set_extension("copc.laz");

        sender
            .send(FrontendTask::Log(format!(
                "Writing {} relevant lidar files to {:?}",
                relevant_paths.len(),
                merged_path
            )))
            .unwrap();

        run_copc_converter(&relevant_paths, &merged_path).unwrap();

        sender
            .send(FrontendTask::UpdateVariable(Variable::SingleCopcPath(
                merged_path,
            )))
            .unwrap();
    }

    let stats = stats.into_iter().reduce(LidarStats::combine_stats).unwrap();

    sender
        .send(FrontendTask::ProgressBar(ProgressBar::Finish))
        .unwrap();

    sender
        .send(FrontendTask::UpdateVariable(Variable::Stats(stats)))
        .unwrap();

    sender
        .send(FrontendTask::UpdateVariable(Variable::Paths(new_paths)))
        .unwrap();

    sender
        .send(FrontendTask::TaskComplete(TaskDone::ConvertCopc))
        .unwrap();
}

fn transform_file(_path: PathBuf, _current_crs: Option<CrsDef>, _out_crs: CrsDef) -> PathBuf {
    unimplemented!("Transforming CRS not yet supported");
}

fn convert_file(
    mut path: PathBuf,
    _current_crs: Option<CrsDef>,
    _sender: Sender<FrontendTask>,
) -> PathBuf {
    let raw_path = path.clone();
    path.set_extension("copc.laz");

    run_copc_converter(&[raw_path], &path).unwrap();
    path
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
) -> PathBuf {
    unimplemented!("Transforming CRS not yet supported");
}
