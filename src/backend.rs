use crate::comms::{OmapComms, messages::*};
use crate::map_gen;
use crate::map_gen::pipeline::PreparedTile;
use crate::parameters::MapParameters;
use crate::project;

use rayon::{ThreadPool, ThreadPoolBuilder};

pub struct Backend {
    comms: OmapComms<FrontendTask, BackendTask>,
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
    pub fn boot(comms: OmapComms<FrontendTask, BackendTask>) -> crate::Result<()> {
        std::thread::Builder::new().spawn(move || -> crate::Result<()> {
            let worker_threads = std::thread::available_parallelism()
                .map(|threads| threads.get())
                .unwrap_or(8)
                .max(1);
            let thread_pool = ThreadPoolBuilder::new()
                .num_threads(worker_threads.max(1))
                .build()
                .map_err(|e| anyhow::anyhow!("Failed to create backend Rayon thread pool: {e}"))?;

            let mut backend = Backend {
                comms,
                map_params: None,
                map_tiles: Vec::with_capacity(9),
                hull: geo::Polygon::new(geo::LineString::new(vec![]), vec![]),
                ref_point: geo::Coord { x: 0., y: 0. },
                thread_pool,
                worker_threads,
            };

            backend.run();
            Ok(())
        })?;
        Ok(())
    }

    fn run(&mut self) {
        while let Ok(task) = self.comms.recv() {
            match task {
                BackendTask::ClearParams => {
                    self.map_params = None;
                }
                BackendTask::SetWorkerThreads(worker_threads) => {
                    if let Err(e) = self.set_worker_threads(worker_threads) {
                        let _ = self.comms.send(FrontendTask::Error(e.to_string(), false));
                    }
                }
                BackendTask::ParseCrs(paths) => {
                    crate::parse_crs::parse_crs(self.comms.sender(), paths);
                }
                BackendTask::MapSpatialLidarRelations(paths, crs) => {
                    map_gen::egui_map::map_laz(self.comms.sender(), paths, crs);
                }
                BackendTask::ConvertCopc(task) => {
                    let ConvertCopcTask {
                        paths,
                        in_epsg,
                        out_epsg,
                        save_location,
                        bounds,
                        polygon,
                        write_single_copc,
                    } = *task;

                    crate::convert_copc::convert_copc(
                        self.comms.sender(),
                        paths,
                        in_epsg,
                        out_epsg,
                        save_location,
                        bounds,
                        polygon,
                        write_single_copc,
                    );
                }

                BackendTask::InitializeMapTile(task) => {
                    let InitializeMapTileTask {
                        paths,
                        test_area,
                        stats,
                    } = *task;

                    match map_gen::egui_map::initialize_map_tile(
                        self.comms.sender(),
                        paths,
                        test_area,
                        stats,
                    ) {
                        Ok(initialized) => {
                            self.map_tiles = initialized.tiles;
                            self.hull = initialized.hull;
                            self.ref_point = initialized.ref_point;
                        }
                        Err(e) => {
                            let _ = self
                                .comms
                                .send(FrontendTask::ProgressBar(ProgressBar::Finish));
                            let _ = self.comms.send(FrontendTask::Error(e.to_string(), true));
                        }
                    }
                }

                BackendTask::RegenerateMap(job_id, params, scope) => {
                    assert!(!self.map_tiles.is_empty());
                    map_gen::egui_map::regenerate_map_tile(
                        &self.comms,
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

                BackendTask::MakeMap(task) => {
                    let MakeMapTask {
                        map_params,
                        file_params,
                        polygon_filter,
                        stats,
                    } = *task;

                    // transform the linestring to output coords
                    let local_polygon_filter = match project::polygon::from_walkers_map_coords(
                        map_params.output.crs.clone(),
                        polygon_filter,
                    ) {
                        Ok(polygon) => polygon,
                        Err(e) => {
                            let _ = self.comms.send(FrontendTask::Error(e.to_string(), true));
                            continue;
                        }
                    };

                    // we are not going back here so can clear the DEMs to free some memory
                    self.reset();

                    let _ = match map_gen::final_map::make_map(
                        self.comms.sender(),
                        &self.thread_pool,
                        map_params,
                        file_params,
                        local_polygon_filter,
                        stats,
                    ) {
                        Ok(_) => self
                            .comms
                            .send(FrontendTask::TaskComplete(TaskDone::MakeMap)),
                        Err(e) => self.comms.send(FrontendTask::Error(e.to_string(), true)),
                    };
                }
                BackendTask::Reset => {
                    self.reset();
                    let _ = self.comms.send(FrontendTask::TaskComplete(TaskDone::Reset));
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

        self.thread_pool = ThreadPoolBuilder::new()
            .num_threads(worker_threads.max(1))
            .build()
            .map_err(|e| anyhow::anyhow!("Failed to create backend Rayon thread pool: {e}"))?;

        self.worker_threads = worker_threads;
        self.comms.send(FrontendTask::Log(format!(
            "Backend worker pool set to {worker_threads} threads"
        )))?;
        Ok(())
    }
}
