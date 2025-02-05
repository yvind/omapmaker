use crate::comms::messages::*;
use std::{path::PathBuf, sync::mpsc::Sender, vec};

use copc_rs::{CopcReader, CopcWriter};
use geo::Contains;
use las::{Builder, Reader};
use proj4rs::{transform::transform, Proj};

const LOCAL_OUT_EPSG: &str = "LOCAL_CS[\"Undefined\"]";

// should be multithreaded
pub fn convert_copc(
    sender: Sender<FrontendTask>,
    paths: Vec<PathBuf>,
    input_epsg: Vec<u16>,
    output_epsg: Option<u16>,
    selected_file: usize,
    boundaries: Vec<[walkers::Position; 4]>,
    polygon_filter: geo::LineString,
) {
    let mut new_paths = Vec::with_capacity(paths.len());

    sender
        .send(FrontendTask::Log(
            "Converting and transforming files".to_string(),
        ))
        .unwrap();
    sender.send(FrontendTask::StartProgressBar).unwrap();

    let polygon = geo::Polygon::new(polygon_filter, vec![]);

    let inc_size = 1. / paths.len() as f32;
    for (pi, path) in paths.into_iter().enumerate() {
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
            let transform_needed = if let Some(o_epsg) = output_epsg {
                input_epsg[pi] != o_epsg
            } else {
                false
            };

            let conversion_needed = CopcReader::from_path(&path).is_err();

            let new_path = if !conversion_needed && !transform_needed {
                // the lidar file is both a COPC and in the correct CRS
                path
            } else if transform_needed && !conversion_needed {
                // the lidar file needs to be transformed into another CRS but is already a copc
                transform_file(path, input_epsg[pi], output_epsg.unwrap())
            } else if conversion_needed && !transform_needed {
                // the lidar file needs both to be converted to copc
                convert_file(path, input_epsg[pi])
            } else {
                // the lidar file needs both to be transformed into another CRS and written to COPC
                convert_and_transform_file(path, input_epsg[pi], output_epsg.unwrap())
            };

            new_paths.push(new_path);
        } else {
            new_paths.push(path);
        }

        sender
            .send(FrontendTask::IncrementProgressBar(inc_size))
            .unwrap()
    }
    sender.send(FrontendTask::FinishProgrssBar).unwrap();

    sender
        .send(FrontendTask::SetVariable(Variable::Paths(new_paths)))
        .unwrap();

    sender
        .send(FrontendTask::TaskComplete(TaskDone::ConvertCopc))
        .unwrap();
}

fn transform_file(path: PathBuf, current_epsg: u16, out_epsg: u16) -> PathBuf {
    unimplemented!("Transforming CRS not yet supported");
}

fn convert_file(mut path: PathBuf, current_epsg: u16) -> PathBuf {
    let mut las_reader = Reader::from_path(&path).unwrap();

    path.set_extension("copc.laz");

    let mut header = las_reader.header().clone();

    if current_epsg == u16::MAX {
        // Local coords => remove all crs vlrs from the header (should not be any) and add our own
        // Needs to be done because copc demands a crs vlr to exist
        let mut raw_head = header.clone().into_raw().unwrap();

        raw_head.global_encoding |= 0b10000; // set the wkt crs bit

        let mut builder = Builder::new(raw_head).unwrap();

        for evlr in header.evlrs() {
            match (evlr.user_id.to_lowercase().as_str(), evlr.record_id) {
                // not forwarding these vlrs
                ("lasf_projection", 2112 | 34735..=34737) => (),
                _ => builder.evlrs.push(evlr.clone()),
            }
        }
        for vlr in header.vlrs() {
            match (vlr.user_id.to_lowercase().as_str(), vlr.record_id) {
                // not forwarding these vlrs
                ("lasf_projection", 2112 | 34735..=34737) => (),
                _ => builder.vlrs.push(vlr.clone()),
            }
        }
        let mut user_id = [0; 16];
        for (i, byte) in "LASF_Projection".as_bytes().iter().enumerate() {
            user_id[i] = *byte;
        }

        let data = LOCAL_OUT_EPSG.as_bytes().to_vec();

        let local_vlr = las::Vlr::new(las::raw::Vlr {
            reserved: 0,
            user_id,
            record_id: 2112,
            record_length_after_header: las::raw::vlr::RecordLength::Vlr(data.len() as u16),
            description: [0; 32],
            data,
        });

        builder.vlrs.push(local_vlr);

        header = builder.into_header().unwrap();
    } else if las_crs::parse_las_crs(&header).is_err() {
        // check if a crs exists if not we must add our own
        let data = crs_definitions::from_code(current_epsg)
            .unwrap()
            .wkt
            .as_bytes()
            .to_vec();

        let mut user_id = [0; 16];
        for (i, byte) in "LASF_Projection".as_bytes().iter().enumerate() {
            user_id[i] = *byte;
        }

        let crs_vlr = las::Vlr::new(las::raw::Vlr {
            reserved: 0,
            user_id,
            record_id: 2112,
            record_length_after_header: las::raw::vlr::RecordLength::Vlr(data.len() as u16),
            description: [0; 32],
            data,
        });

        let mut raw_head = header.clone().into_raw().unwrap();

        raw_head.global_encoding |= 0b10000; // set the wkt crs bit

        let mut builder = Builder::new(raw_head).unwrap();

        for evlr in header.evlrs() {
            match (evlr.user_id.to_lowercase().as_str(), evlr.record_id) {
                // not forwarding these vlrs
                ("lasf_projection", 2112 | 34735..=34737) => (),
                _ => builder.evlrs.push(evlr.clone()),
            }
        }
        for vlr in header.vlrs() {
            match (vlr.user_id.to_lowercase().as_str(), vlr.record_id) {
                // not forwarding these vlrs
                ("lasf_projection", 2112 | 34735..=34737) => (),
                _ => builder.vlrs.push(vlr.clone()),
            }
        }

        builder.vlrs.push(crs_vlr);

        header = builder.into_header().unwrap();
    }
    // now the header is guaranteed to contain a crs vlr

    let num_points = header.number_of_points() as i32;

    let mut copc_writer = CopcWriter::from_path(&path, header, -1, -1).unwrap();

    let points = las_reader.points().filter_map(las::Result::ok);

    copc_writer.write(points, num_points).unwrap();

    path
}

fn convert_and_transform_file(path: PathBuf, current_epsg: u16, out_epsg: u16) -> PathBuf {
    unimplemented!("Transforming CRS not yet supported");
}
