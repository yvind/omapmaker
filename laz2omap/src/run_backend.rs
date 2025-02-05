use proj4rs::transform::transform;
use proj4rs::Proj;

use crate::comms::{messages::*, OmapComms};
use crate::raster::Dfm;

use std::time::Duration;

pub struct OmapGenerator {
    comms: OmapComms<FrontendTask, BackendTask>,

    // for iterating the params
    map_tile_dem: Option<Dfm>,
    map_tile_grad_dem: Option<Dfm>,
    convex_hull: Option<geo::LineString>,
    ref_point: geo::Coord,
    z_range: (f64, f64),
}

impl OmapGenerator {
    pub fn boot(comms: OmapComms<FrontendTask, BackendTask>) {
        std::thread::Builder::new()
            .stack_size(crate::STACK_SIZE * 1024 * 1024) // needs to increase thread stack size as dfms are kept on the stack
            .spawn(move || {
                let mut backend = OmapGenerator {
                    comms,
                    map_tile_dem: None,
                    map_tile_grad_dem: None,
                    convex_hull: None,
                    ref_point: geo::Coord { x: 0., y: 0. },
                    z_range: (0., 0.),
                };

                backend.run();
            })
            .unwrap();
    }

    fn run(&mut self) {
        loop {
            if let Ok(task) = self.comms.try_recv() {
                match task {
                    BackendTask::ParseCrs(paths) => {
                        crate::steps::parse_crs(self.comms.clone_sender(), paths);
                    }
                    BackendTask::MapSpatialLidarRelations(paths, crs) => {
                        crate::steps::map_laz(self.comms.clone_sender(), paths, crs);
                    }
                    BackendTask::ConvertCopc(
                        paths,
                        in_epsg,
                        out_epsg,
                        selected_file,
                        bounds,
                        polygon,
                    ) => {
                        crate::steps::convert_copc(
                            self.comms.clone_sender(),
                            paths,
                            in_epsg,
                            out_epsg,
                            selected_file,
                            bounds,
                            polygon,
                        );
                    }
                    BackendTask::InitializeMapTile(path) => {
                        let (dem, gdem, hull, ref_point, z_range) =
                            crate::steps::initialize_map_tile(self.comms.clone_sender(), path);
                        self.map_tile_dem = Some(dem);
                        self.map_tile_grad_dem = Some(gdem);
                        self.convex_hull = Some(hull);
                        self.ref_point = ref_point;
                        self.z_range = z_range;
                    }
                    BackendTask::RegenerateMap(params) => {
                        assert!(self.map_tile_dem.is_some());
                        crate::steps::regenerate_map_tile(
                            self.comms.clone_sender(),
                            self.map_tile_dem.as_ref().unwrap(),
                            self.map_tile_grad_dem.as_ref().unwrap(),
                            self.convex_hull.as_ref().unwrap(),
                            self.ref_point,
                            self.z_range,
                            *params,
                        );
                    }
                    BackendTask::MakeMap(map_params, file_params, polygon_filter) => {
                        // transform the linestring to output coords
                        let local_polygon_filter =
                            transform_polygon(map_params.output_epsg, polygon_filter);

                        crate::make_map(
                            self.comms.clone_sender(),
                            *map_params,
                            *file_params,
                            local_polygon_filter,
                        );
                    }
                    BackendTask::Reset => {
                        self.map_tile_dem = None;
                        self.map_tile_grad_dem = None;
                        self.convex_hull = None;
                        self.ref_point = geo::Coord { x: 0., y: 0. };
                        self.comms
                            .send(FrontendTask::TaskComplete(TaskDone::Reset))
                            .unwrap();
                    }
                    BackendTask::HeartBeat => (), // to check if the backend's receiver has hung up, i.e. backend has panicked and must be restarted
                }
            }
            std::thread::sleep(Duration::from_millis(100));
        }
    }
}

fn transform_polygon(epsg: Option<u16>, line: geo::LineString) -> Option<geo::Polygon> {
    if line.0.is_empty() {
        return None;
    }
    if epsg.is_none() {
        return Some(geo::Polygon::new(line, vec![]));
    }
    let epsg = epsg.unwrap();

    let global_proj = Proj::from_epsg_code(4326).unwrap();
    let local_proj = Proj::from_epsg_code(epsg).unwrap();

    let mut points: Vec<(f64, f64)> = line.0.into_iter().map(|c| c.x_y()).collect();

    transform(&global_proj, &local_proj, points.as_mut_slice()).unwrap();

    let line = geo::LineString::new(
        points
            .into_iter()
            .map(|t| geo::Coord { x: t.0, y: t.1 })
            .collect(),
    );

    Some(geo::Polygon::new(line, vec![]))
}
