use eframe::egui;
use las::Reader;
use proj4rs::transform::transform;
use proj4rs::Proj;

use crate::comms::{messages::*, OmapComms};
use crate::geometry::MapRect;
use crate::raster::Dfm;

use std::time::Duration;

pub struct OmapGenerator {
    comms: OmapComms<FrontendTask, BackendTask>,
    ctx: egui::Context,

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
            .stack_size(crate::STACK_SIZE * 1024 * 1024) // needs to increase thread stack size as dfms are kept on the stack
            .spawn(move || {
                let mut backend = OmapGenerator {
                    comms,
                    ctx,
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
                            *params,
                        );
                        // to force to update function to run
                        self.ctx.request_repaint();
                    }
                    BackendTask::MakeMap(map_params, file_params, polygon_filter) => {
                        // transform the linestring to output coords
                        let local_polygon_filter =
                            transform_polygon(map_params.output_epsg, polygon_filter);

                        // we are not going back here so can clear the dems to free some memory
                        self.reset();

                        crate::make_map(
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
                        let neighbours = get_neighbours(n_x, n_y);

                        let cb = transform_rects(epsg, &cb);

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
        self.map_tile_dem.clear();
        self.map_tile_grad_dem.clear();
        self.map_tile_drm.clear();
        self.cut_bounds.clear();
        self.ref_point = geo::Coord { x: 0., y: 0. };
    }
}

fn transform_rects(epsg: Option<u16>, rects: &Vec<geo::Rect>) -> Vec<[walkers::Position; 4]> {
    let mut out = Vec::with_capacity(rects.len());
    if epsg.is_none() {
        for rect in rects {
            out.push([
                geo::Coord {
                    x: rect.min().x,
                    y: rect.max().y,
                },
                rect.min(),
                geo::Coord {
                    x: rect.max().x,
                    y: rect.min().y,
                },
                rect.max(),
            ]);
        }
    } else if epsg.is_some() {
        let epsg = epsg.unwrap();

        let global_proj = Proj::from_epsg_code(4326).unwrap();
        let local_proj = Proj::from_epsg_code(epsg).unwrap();

        for rect in rects {
            let mut points = [
                (rect.min().x, rect.max().y),
                rect.min().x_y(),
                (rect.max().x, rect.min().y),
                rect.max().x_y(),
            ];

            transform(&local_proj, &global_proj, points.as_mut_slice()).unwrap();

            out.push([
                geo::Coord {
                    x: points[0].0.to_degrees(),
                    y: points[0].1.to_degrees(),
                },
                geo::Coord {
                    x: points[1].0.to_degrees(),
                    y: points[1].1.to_degrees(),
                },
                geo::Coord {
                    x: points[2].0.to_degrees(),
                    y: points[2].1.to_degrees(),
                },
                geo::Coord {
                    x: points[3].0.to_degrees(),
                    y: points[3].1.to_degrees(),
                },
            ]);
        }
    }
    out
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

    // proj4rs uses radians, but walkers uses degrees. Conversion needed
    let mut points: Vec<(f64, f64)> = line
        .0
        .into_iter()
        .map(|c| (c.x.to_radians(), c.y.to_radians()))
        .collect();

    transform(&global_proj, &local_proj, points.as_mut_slice()).unwrap();

    let line = geo::LineString::new(
        points
            .into_iter()
            .map(|t| geo::Coord { x: t.0, y: t.1 })
            .collect(),
    );

    Some(geo::Polygon::new(line, vec![]))
}

fn get_neighbours(nx: usize, ny: usize) -> Vec<[Option<usize>; 9]> {
    let mut neighbours = Vec::with_capacity(nx * ny);

    for yi in 0..ny {
        for xi in 0..nx {
            if xi == 0 && yi == 0 {
                //no neighbours to the left or top
                neighbours.push([
                    Some(yi * nx + xi),
                    None,
                    None,
                    None,
                    Some(yi * nx + xi + 1),
                    Some(yi * nx + xi + 1 + nx),
                    Some(yi * nx + xi + nx),
                    None,
                    None,
                ]);
            } else if xi == nx - 1 && yi == 0 {
                // no neighbours to the right or top
                neighbours.push([
                    Some(yi * nx + xi),
                    None,
                    None,
                    None,
                    None,
                    None,
                    Some(yi * nx + xi + nx),
                    Some(yi * nx + xi + nx - 1),
                    Some(yi * nx + xi - 1),
                ]);
            } else if xi == 0 && yi == ny - 1 {
                // no neighbours to the left or bottom
                neighbours.push([
                    Some(yi * nx + xi),
                    None,
                    Some(yi * nx + xi - nx),
                    Some(yi * nx + xi - nx + 1),
                    Some(yi * nx + xi + 1),
                    None,
                    None,
                    None,
                    None,
                ]);
            } else if xi == nx - 1 && yi == ny - 1 {
                // no neighbours to the right or bottom
                neighbours.push([
                    Some(yi * nx + xi),
                    Some(yi * nx + xi - 1 - nx),
                    Some(yi * nx + xi - nx),
                    None,
                    None,
                    None,
                    None,
                    None,
                    Some(yi * nx + xi - 1),
                ]);
            } else if xi == 0 {
                // no neighbours to the left
                neighbours.push([
                    Some(yi * nx + xi),
                    None,
                    Some(yi * nx + xi - nx),
                    Some(yi * nx + xi - nx + 1),
                    Some(yi * nx + xi + 1),
                    Some(yi * nx + xi + nx + 1),
                    Some(yi * nx + xi + nx),
                    None,
                    None,
                ]);
            } else if xi == nx - 1 {
                neighbours.push([
                    Some(yi * nx + xi),
                    Some(yi * nx + xi - 1 - nx),
                    Some(yi * nx + xi - nx),
                    None,
                    None,
                    None,
                    Some(yi * nx + xi + nx),
                    Some(yi * nx + xi + nx - 1),
                    Some(yi * nx + xi - 1),
                ]);
            } else if yi == 0 {
                neighbours.push([
                    Some(yi * nx + xi),
                    None,
                    None,
                    None,
                    Some(yi * nx + xi + 1),
                    Some(yi * nx + xi + nx + 1),
                    Some(yi * nx + xi + nx),
                    Some(yi * nx + xi + nx - 1),
                    Some(yi * nx + xi - 1),
                ]);
            } else if yi == ny - 1 {
                neighbours.push([
                    Some(yi * nx + xi),
                    Some(yi * nx + xi - 1 - nx),
                    Some(yi * nx + xi - nx),
                    Some(yi * nx + xi - nx + 1),
                    Some(yi * nx + xi + 1),
                    None,
                    None,
                    None,
                    Some(yi * nx + xi - 1),
                ]);
            } else {
                neighbours.push([
                    Some(yi * nx + xi),
                    Some(yi * nx + xi - 1 - nx),
                    Some(yi * nx + xi - nx),
                    Some(yi * nx + xi - nx + 1),
                    Some(yi * nx + xi + 1),
                    Some(yi * nx + xi + nx + 1),
                    Some(yi * nx + xi + nx),
                    Some(yi * nx + xi + nx - 1),
                    Some(yi * nx + xi - 1),
                ]);
            }
        }
    }
    neighbours
}
