use crate::comms::messages::*;

use std::{path::PathBuf, sync::mpsc};

use las::Reader;

pub fn parse_crs(sender: mpsc::Sender<FrontendTask>, mut paths: Vec<PathBuf>) {
    sender
        .send(FrontendTask::Log(
            "Detecting CRS of all provided files...".to_string(),
        ))
        .unwrap();
    sender.send(FrontendTask::StartProgressBar).unwrap();

    let mut crs_epsg = vec![];

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
                sender
                    .send(FrontendTask::IncrementProgressBar(inc_size))
                    .unwrap();
                continue;
            }
        };
        let crs_res = las_crs::parse_las_crs(reader.header());

        if let Ok(epsg) = crs_res {
            crs_epsg.push(epsg.0);
        } else {
            crs_epsg.push(u16::MAX);
            num_crs_less += 1;
        }
        sender
            .send(FrontendTask::IncrementProgressBar(inc_size))
            .unwrap();

        i += 1;
    }
    sender.send(FrontendTask::FinishProgrssBar).unwrap();

    let num_files = paths.len();

    if paths.is_empty() {
        sender
            .send(FrontendTask::BackendError(
                "None of the given files were readable as lidar files".to_string(),
                true,
            ))
            .unwrap();
        return;
    } else if unreadable_path {
        sender
            .send(FrontendTask::BackendError(
                "Some paths were not readable as lidar files and have been removed".to_string(),
                false,
            ))
            .unwrap();
        sender
            .send(FrontendTask::SetVariable(Variable::Paths(paths)))
            .unwrap();
    }

    sender
        .send(FrontendTask::Log(format!(
            "Successfully detected a CRS for {} out of {num_files} lidar files",
            num_files - num_crs_less
        )))
        .unwrap();

    sender
        .send(FrontendTask::SetVariable(Variable::CrsEPSG(crs_epsg)))
        .unwrap();
    sender
        .send(FrontendTask::SetVariable(Variable::CrsLessString(
            num_crs_less,
        )))
        .unwrap();
    sender
        .send(FrontendTask::SetVariable(Variable::CrsLessCheckBox(
            num_crs_less,
        )))
        .unwrap();

    if num_crs_less == 0 {
        sender
            .send(FrontendTask::TaskComplete(TaskDone::ParseCrs(SetCrs::Auto)))
            .unwrap();
    } else {
        sender.send(FrontendTask::CrsModal).unwrap();
    }
}
