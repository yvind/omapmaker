use anyhow::Context;
use eframe::egui;
use las::Reader;

use crate::comms::{FrontendSender, OmapComms, messages::*};
use crate::geometry::MapRect;
use crate::map_gen;
use crate::map_gen::pipeline::PreparedTile;
use crate::neighbors::{self, Neighborhood};
use crate::parameters::MapParameters;
use crate::project;

use rayon::{ThreadPool, ThreadPoolBuilder};

pub struct Backend {
    comms: OmapComms<FrontendTask, BackendTask>,
    ctx: egui::Context,
    // store the params used for generating a map tile
    // so the next call only generates the
    // objects corresponding to the changed parameters
    map_params: Option<MapParameters>,

    // for iterating the params
    map_tiles: Vec<PreparedTile>,
    hull: geo::Polygon,
    ref_point: geo::Coord,
    thread_pool: ThreadPool,
    worker_threads: usize,
}

impl Backend {
    pub fn boot(comms: OmapComms<FrontendTask, BackendTask>, ctx: egui::Context) {
        if let Err(e) = std::thread::Builder::new()
            .stack_size(crate::STACK_SIZE * 1024 * 1024)
            .spawn(move || {
                let worker_threads = default_worker_threads();
                let thread_pool = match build_thread_pool(worker_threads) {
                    Ok(pool) => pool,
                    Err(e) => {
                        eprintln!("Failed to create backend Rayon thread pool: {e}");
                        return;
                    }
                };

                let mut backend = Backend {
                    comms,
                    ctx,
                    map_params: None,
                    map_tiles: Vec::with_capacity(9),
                    hull: geo::Polygon::new(geo::LineString::new(vec![]), vec![]),
                    ref_point: geo::Coord { x: 0., y: 0. },
                    thread_pool,
                    worker_threads,
                };

                backend.run();
            })
        {
            eprintln!("Failed to spawn backend thread: {e}");
        }
    }

    fn run(&mut self) {
        while let Ok(task) = self.comms.recv() {
            match task {
                BackendTask::ClearParams => {
                    self.map_params = None;
                }
                BackendTask::SetWorkerThreads(worker_threads) => {
                    if let Err(e) = self.set_worker_threads(worker_threads) {
                        self.send_event(FrontendTask::Error(e.to_string(), false));
                    }
                }
                BackendTask::ParseCrs(paths) => {
                    crate::parse_crs::parse_crs(self.events(), paths);
                }
                BackendTask::MapSpatialLidarRelations(paths, crs) => {
                    map_gen::egui_map::map_laz(self.events(), paths, crs);
                }
                BackendTask::ConvertCopc(
                    paths,
                    in_epsg,
                    out_epsg,
                    save_location,
                    bounds,
                    polygon,
                    write_single_copc,
                ) => {
                    crate::convert_copc::convert_copc(
                        self.events(),
                        paths,
                        in_epsg,
                        out_epsg,
                        save_location,
                        bounds,
                        polygon,
                        write_single_copc,
                    );
                }

                BackendTask::InitializeMapTile(path, tiles, stats) => {
                    match map_gen::egui_map::initialize_map_tile(self.events(), path, tiles, stats)
                    {
                        Ok(initialized) => {
                            self.map_tiles = initialized.tiles;
                            self.hull = initialized.hull;
                            self.ref_point = initialized.ref_point;
                        }
                        Err(e) => {
                            self.send_event(FrontendTask::ProgressBar(ProgressBar::Finish));
                            self.send_event(FrontendTask::Error(e.to_string(), true));
                        }
                    }
                }

                BackendTask::RegenerateMap(job_id, params, scope) => {
                    assert!(!self.map_tiles.is_empty());
                    map_gen::egui_map::regenerate_map_tile(
                        self.events(),
                        job_id,
                        &self.thread_pool,
                        &self.map_tiles,
                        &self.hull,
                        self.ref_point,
                        &params,
                        &self.map_params,
                        scope,
                    );

                    self.map_params = Some(*params);
                }

                BackendTask::MakeMap(map_params, file_params, polygon_filter, stats) => {
                    // transform the linestring to output coords
                    let local_polygon_filter = match project::polygon::from_walkers_map_coords(
                        map_params.output.crs.clone(),
                        polygon_filter,
                    ) {
                        Ok(polygon) => polygon,
                        Err(e) => {
                            self.send_event(FrontendTask::Error(e.to_string(), true));
                            continue;
                        }
                    };

                    // we are not going back here so can clear the DEMs to free some memory
                    self.reset();

                    let _ = match map_gen::final_map::make_map(
                        self.events(),
                        &self.thread_pool,
                        *map_params,
                        *file_params,
                        local_polygon_filter,
                        stats,
                    ) {
                        Ok(_) => self
                            .events()
                            .send(FrontendTask::TaskComplete(TaskDone::MakeMap)),
                        Err(e) => self.events().send(FrontendTask::Error(e.to_string(), true)),
                    };
                }
                BackendTask::Reset => {
                    self.reset();
                    self.send_event(FrontendTask::TaskComplete(TaskDone::Reset));
                }
                BackendTask::TileSelectedFile(path, epsg) => {
                    if let Err(e) = self.tile_selected_file(path, epsg) {
                        self.send_event(FrontendTask::Error(e.to_string(), false));
                    }
                }
            }
        }
    }

    fn reset(&mut self) {
        self.map_params = None;

        // removing the allocated memory also, not justing clearing
        self.map_tiles = Vec::new();

        self.hull.exterior_mut(|l| l.0.clear());
        self.ref_point = geo::Coord { x: 0., y: 0. };
    }

    fn set_worker_threads(&mut self, worker_threads: usize) -> crate::Result<()> {
        let worker_threads = worker_threads.max(1);
        if worker_threads == self.worker_threads {
            return Ok(());
        }

        self.thread_pool = build_thread_pool(worker_threads)?;
        self.worker_threads = worker_threads;
        self.send_event(FrontendTask::Log(format!(
            "Backend worker pool set to {worker_threads} threads"
        )));
        Ok(())
    }

    fn tile_selected_file(
        &self,
        path: std::path::PathBuf,
        epsg: Option<proj_core::CrsDef>,
    ) -> crate::Result<()> {
        let bounds = Reader::from_path(&path)
            .with_context(|| format!("Failed to read selected lidar file {path:?}"))?
            .header()
            .bounds();
        let rect = geo::Rect::from_bounds(bounds);

        let (_, cb, n_x, n_y) = map_gen::common::retile_bounds(&rect, &Neighborhood::new(0));
        let neighbors = neighbors::neighbors_on_grid(n_x, n_y);

        let cb = project::rectangles::to_walkers_map_points(epsg, &cb)?;

        self.send_event(FrontendTask::UpdateVariable(Variable::TileBounds(cb)));
        self.send_event(FrontendTask::UpdateVariable(Variable::TileNeighbors(
            neighbors,
        )));
        self.send_event(FrontendTask::TaskComplete(TaskDone::TileSelectedFile));

        Ok(())
    }

    fn events(&self) -> FrontendSender {
        self.comms.frontend_sender(&self.ctx)
    }

    fn send_event(&self, event: FrontendTask) {
        let _ = self.events().send(event);
    }
}

fn default_worker_threads() -> usize {
    std::thread::available_parallelism()
        .map(|threads| threads.get())
        .unwrap_or(8)
        .max(1)
}

fn build_thread_pool(worker_threads: usize) -> crate::Result<ThreadPool> {
    ThreadPoolBuilder::new()
        .num_threads(worker_threads.max(1))
        .stack_size(crate::STACK_SIZE * 1024 * 1024)
        .build()
        .map_err(|e| anyhow::anyhow!("Failed to create backend Rayon thread pool: {e}"))
}
