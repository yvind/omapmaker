use crate::comms::{messages::*, OmapComms};
use crate::params::{FileParams, MapParams};

use las::Reader;
use proj4rs::{proj::Proj, transform::transform};
use std::{path::PathBuf, time::Duration};

pub struct OmapGenerator {
    comms: OmapComms<FrontEndTask, BackendTask>,
    file_params: FileParams,
    map_params: MapParams,
}

impl OmapGenerator {
    pub fn boot(comms: OmapComms<FrontEndTask, BackendTask>) {
        std::thread::spawn(move || {
            let mut backend = OmapGenerator {
                comms,
                map_params: Default::default(),
                file_params: Default::default(),
            };

            backend.run();
        });
    }

    fn run(&mut self) {
        loop {
            if let Ok(task) = self.comms.try_recv() {
                match task {
                    BackendTask::ParseCrs(paths) => {
                        self.parse_crs(paths);
                    }
                    BackendTask::ConnectedComponentAnalysis(paths, crs) => {
                        self.connected_components(paths, crs);
                    }
                    BackendTask::ConvertCopc(_epsg) => {
                        std::thread::sleep(std::time::Duration::from_secs(5));
                        self.comms
                            .send(FrontEndTask::TaskComplete(TaskDone::ConvertCopc))
                            .unwrap();
                    }
                    BackendTask::Reset => {
                        self.comms
                            .send(FrontEndTask::TaskComplete(TaskDone::Reset))
                            .unwrap();
                    }
                    BackendTask::RegenerateMap(params) => {
                        self.comms
                            .send(FrontEndTask::TaskComplete(TaskDone::RegenerateMap))
                            .unwrap();
                    }
                    BackendTask::MakeMap(_) => {
                        std::thread::sleep(std::time::Duration::from_secs(5));
                        self.comms
                            .send(FrontEndTask::TaskComplete(TaskDone::MakeMap))
                            .unwrap();
                    }
                    BackendTask::HeartBeat => (),
                }
            }
            std::thread::sleep(Duration::from_millis(100));
        }
    }

    fn connected_components(&mut self, paths: Vec<PathBuf>, crs_epsg: Option<Vec<u16>>) {
        self.comms
            .send(FrontEndTask::Log(
                "Building Lidar Neighbour Graph".to_string(),
            ))
            .unwrap();
        let (boundaries, mid_point) = read_boundaries(paths, crs_epsg);

        self.comms
            .send(FrontEndTask::SetVariable(Variable::Boundaries(
                boundaries.clone(),
            )))
            .unwrap();

        self.comms
            .send(FrontEndTask::SetVariable(Variable::Home(mid_point)))
            .unwrap();

        let parts = connected_component_analysis(boundaries);

        self.comms
            .send(FrontEndTask::SetVariable(Variable::ConnectedComponents(
                parts,
            )))
            .unwrap();
        self.comms
            .send(FrontEndTask::TaskComplete(
                TaskDone::ConnectedComponentAnalysis,
            ))
            .unwrap();
    }

    fn parse_crs(&mut self, paths: Vec<PathBuf>) {
        self.comms
            .send(FrontEndTask::Log(
                "Detecting CRS of all provided files...".to_string(),
            ))
            .unwrap();
        self.comms.send(FrontEndTask::StartProgressBar).unwrap();

        let mut crs_epsg = vec![];

        let mut crs_less = 0;

        let inc_size = 1. / paths.len() as f32;
        for path in paths.iter() {
            let reader = Reader::from_path(path).unwrap();
            let crs_res = las_crs::parse_las_crs(reader.header());

            if let Ok(epsg) = crs_res {
                crs_epsg.push(epsg.0);
            } else {
                crs_epsg.push(u16::MAX);
                crs_less += 1;
            }
            self.comms
                .send(FrontEndTask::IncrementProgressBar(inc_size))
                .unwrap();
        }
        self.comms.send(FrontEndTask::FinishProgrssBar).unwrap();

        self.comms
            .send(FrontEndTask::Log(format!(
                "{crs_less} out of {} lidar files have no CRS detected",
                paths.len()
            )))
            .unwrap();

        self.comms
            .send(FrontEndTask::SetVariable(Variable::CrsEPSG(crs_epsg)))
            .unwrap();
        self.comms
            .send(FrontEndTask::SetVariable(Variable::CrsLessString(crs_less)))
            .unwrap();
        self.comms
            .send(FrontEndTask::SetVariable(Variable::CrsLessCheckBox(
                crs_less,
            )))
            .unwrap();

        if crs_less == 0 {
            self.comms
                .send(FrontEndTask::TaskComplete(TaskDone::ParseCrs(SetCrs::Auto)))
                .unwrap();
        } else {
            self.comms.send(FrontEndTask::CrsModal).unwrap();
        }
    }
}

fn connected_component_analysis(boundaries: Vec<[walkers::Position; 4]>) -> Vec<Vec<usize>> {
    vec![(0..boundaries.len()).collect()]
}

fn read_boundaries(
    paths: Vec<PathBuf>,
    crs_epsg: Option<Vec<u16>>,
) -> (Vec<[walkers::Position; 4]>, walkers::Position) {
    let mut boundaries = vec![];

    let mut all_lidar_bounds = [(f64::MAX, f64::MIN), (f64::MIN, f64::MAX)];

    let global_coords = crs_epsg.is_some();

    for (i, path) in paths.iter().enumerate() {
        let reader = Reader::from_path(path).unwrap();
        let bounds = reader.header().bounds();

        let mut points = [
            (bounds.min.x, bounds.max.y),
            (bounds.min.x, bounds.min.y),
            (bounds.max.x, bounds.min.y),
            (bounds.max.x, bounds.max.y),
        ];

        if global_coords {
            // transform bounds to lat lon
            let to = Proj::from_user_string("WGS84").unwrap();
            let from = Proj::from_epsg_code(crs_epsg.as_ref().unwrap()[i]).unwrap();

            transform(&from, &to, points.as_mut_slice()).unwrap();

            for (x, y) in points.iter_mut() {
                *x = x.to_degrees();
                *y = y.to_degrees();
            }
        }

        boundaries.push([
            walkers::pos_from_lon_lat(points[0].0, points[0].1),
            walkers::pos_from_lon_lat(points[1].0, points[1].1),
            walkers::pos_from_lon_lat(points[2].0, points[2].1),
            walkers::pos_from_lon_lat(points[3].0, points[3].1),
        ]);

        if all_lidar_bounds[0].0 > points[0].0 {
            all_lidar_bounds[0].0 = points[0].0;
        }
        if all_lidar_bounds[0].1 < points[0].1 {
            all_lidar_bounds[0].1 = points[0].1;
        }
        if all_lidar_bounds[1].0 < points[2].0 {
            all_lidar_bounds[1].0 = points[2].0;
        }
        if all_lidar_bounds[1].1 > points[2].1 {
            all_lidar_bounds[1].1 = points[2].1;
        }
    }
    let mid_point = walkers::pos_from_lon_lat(
        (all_lidar_bounds[0].0 + all_lidar_bounds[1].0) / 2.,
        (all_lidar_bounds[0].1 + all_lidar_bounds[1].1) / 2.,
    );
    (boundaries, mid_point)
}
