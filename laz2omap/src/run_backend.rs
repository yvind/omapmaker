use eframe::egui;
use las::Reader;

use crate::comms::{messages::*, OmapComms};
use crate::geometry::MapRect;
use crate::params::MapParams;
use crate::raster::Dfm;

use std::time::Duration;

pub struct OmapGenerator {
    comms: OmapComms<FrontendTask, BackendTask>,
    ctx: egui::Context,
    // store the params used for generating a map tile
    // on the next call to only generate the
    // objects corresponding to the changed parameters
    map_params: Option<MapParams>,

    // for iterating the params
    map_tile_dem: Vec<Dfm>,
    map_tile_grad_dem: Vec<Dfm>,
    map_tile_drm: Vec<Dfm>,
    cut_bounds: Vec<geo::Polygon>,
    hull: geo::Polygon,
    ref_point: geo::Coord,
    z_range: (f64, f64),
}

impl OmapGenerator {
    pub fn boot(comms: OmapComms<FrontendTask, BackendTask>, ctx: egui::Context) {
        std::thread::Builder::new()
            .stack_size(crate::STACK_SIZE * 1024 * 1024)
            .spawn(move || {
                let mut backend = OmapGenerator {
                    comms,
                    ctx,
                    map_params: None,
                    map_tile_dem: Vec::with_capacity(9),
                    map_tile_grad_dem: Vec::with_capacity(9),
                    map_tile_drm: Vec::with_capacity(9),
                    cut_bounds: Vec::with_capacity(9),
                    hull: geo::Polygon::new(geo::LineString::new(vec![]), vec![]),
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
                    BackendTask::ClearParams => {
                        self.map_params = None;
                    }
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
                    BackendTask::InitializeMapTile(path, tiles) => {
                        let (dem, gdem, drm, cut_bounds, hull, ref_point, z_range) =
                            crate::steps::initialize_map_tile(
                                self.comms.clone_sender(),
                                path,
                                tiles,
                            );
                        self.map_tile_dem = dem;
                        self.map_tile_grad_dem = gdem;
                        self.map_tile_drm = drm;
                        self.cut_bounds = cut_bounds;
                        self.hull = hull;
                        self.ref_point = ref_point;
                        self.z_range = z_range;
                    }
                    BackendTask::RegenerateMap(params) => {
                        assert!(!self.map_tile_dem.is_empty());
                        crate::steps::regenerate_map_tile(
                            self.comms.clone_sender(),
                            &self.map_tile_dem,
                            &self.map_tile_grad_dem,
                            &self.map_tile_drm,
                            &self.cut_bounds,
                            &self.hull,
                            self.ref_point,
                            self.z_range,
                            *params.clone(),
                            self.map_params.clone(),
                        );

                        self.map_params = Some(*params);
                        // to force to update function to run
                        self.ctx.request_repaint();
                    }
                    BackendTask::MakeMap(map_params, file_params, polygon_filter) => {
                        // transform the linestring to output coords
                        let local_polygon_filter = crate::project::polygon::from_walkers_map_coords(
                            map_params.output_epsg,
                            polygon_filter,
                        );

                        // we are not going back here so can clear the dems to free some memory
                        self.reset();

                        crate::steps::make_map(
                            self.comms.clone_sender(),
                            *map_params,
                            *file_params,
                            local_polygon_filter,
                        );
                    }
                    BackendTask::Reset => {
                        self.reset();
                        self.comms
                            .send(FrontendTask::TaskComplete(TaskDone::Reset))
                            .unwrap();
                    }
                    BackendTask::TileSelectedFile(path, epsg) => {
                        let bounds = Reader::from_path(&path).unwrap().header().bounds();
                        let rect = geo::Rect::from_bounds(bounds);

                        let (_, cb, n_x, n_y) = crate::steps::retile_bounds(
                            &rect,
                            &geo::Rect::new(
                                geo::Coord { x: 0., y: 0. },
                                geo::Coord { x: 0., y: 0. },
                            ),
                        );
                        let neighbours = crate::steps::neighbours_on_grid(n_x, n_y);

                        let cb = crate::project::rectangles::to_walkers_map_coords(epsg, &cb);

                        self.comms
                            .send(FrontendTask::UpdateVariable(Variable::TileBounds(cb)))
                            .unwrap();
                        self.comms
                            .send(FrontendTask::UpdateVariable(Variable::TileNeighbours(
                                neighbours,
                            )))
                            .unwrap();
                        self.comms
                            .send(FrontendTask::TaskComplete(TaskDone::TileSelectedFile))
                            .unwrap();
                    }
                }
            }
            std::thread::sleep(Duration::from_millis(100));
        }
    }

    fn reset(&mut self) {
        self.map_params = None;
        self.map_tile_dem.clear();
        self.map_tile_grad_dem.clear();
        self.map_tile_drm.clear();
        self.cut_bounds.clear();
        self.hull.exterior_mut(|l| l.0.clear());
        self.z_range = (0., 0.);
        self.ref_point = geo::Coord { x: 0., y: 0. };
    }
}
