use crate::{comms::messages::*, gui::modals::OmapModal};

use std::{path::PathBuf, sync::mpsc};

use las::Reader;

pub fn parse_crs(sender: mpsc::Sender<FrontendTask>, mut paths: Vec<PathBuf>) {
    sender
        .send(FrontendTask::Log(
            "Detecting CRS of all provided files...".to_string(),
        ))
        .unwrap();
    sender
        .send(FrontendTask::ProgressBar(ProgressBar::Start))
        .unwrap();

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
                sender
                    .send(FrontendTask::ProgressBar(ProgressBar::Inc(inc_size)))
                    .unwrap();
                continue;
            }
        };

        let mut crs_def = None;
        if let Some(wkt) = reader.header().get_wkt_crs_bytes() {
            let wkt_string = str::from_utf8(wkt).unwrap();

            crs_def = proj_wkt::parse_crs(wkt_string).ok();
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

        sender
            .send(FrontendTask::ProgressBar(ProgressBar::Inc(inc_size)))
            .unwrap();

        i += 1;
    }
    sender
        .send(FrontendTask::ProgressBar(ProgressBar::Finish))
        .unwrap();

    let num_files = paths.len();

    if paths.is_empty() {
        sender
            .send(FrontendTask::Error(
                "None of the given files were readable as lidar files".to_string(),
                true,
            ))
            .unwrap();
        return;
    } else if unreadable_path {
        sender
            .send(FrontendTask::Error(
                "Some paths were not readable as lidar files and have been removed".to_string(),
                false,
            ))
            .unwrap();
        sender
            .send(FrontendTask::UpdateVariable(Variable::Paths(paths)))
            .unwrap();
    }

    sender
        .send(FrontendTask::Log(format!(
            "Successfully detected a CRS for {} out of {num_files} lidar files",
            num_files - num_crs_less
        )))
        .unwrap();

    sender
        .send(FrontendTask::UpdateVariable(Variable::CrsDefs(crs_defs)))
        .unwrap();
    sender
        .send(FrontendTask::UpdateVariable(Variable::CrsLessString(
            num_crs_less,
        )))
        .unwrap();
    sender
        .send(FrontendTask::UpdateVariable(Variable::CrsLessCheckBox(
            num_crs_less,
        )))
        .unwrap();

    if num_crs_less == 0 {
        sender
            .send(FrontendTask::TaskComplete(TaskDone::ParseCrs(SetCrs::Auto)))
            .unwrap();
    } else {
        sender
            .send(FrontendTask::OpenModal(OmapModal::ManualSetCRS))
            .unwrap();
    }
}
