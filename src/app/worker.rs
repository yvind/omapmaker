use crate::app::{
    DrawableOmap, OmapComms,
    protocol::{
        AppEvent, ConvertCopcTask, GenerateMapRequest, InitializeMapTileTask, MapPreviewSection,
        ProgressBar, RegenerationScope, TaskComplete, Variable, WorkerCommand,
    },
};
use crate::generation;
use crate::generation::pipeline::PreparedTile;
use crate::parameters::MapParameters;
use crate::project;

use rayon::{ThreadPool, ThreadPoolBuilder};

pub struct Worker {
    comms: OmapComms<AppEvent, WorkerCommand>,
    // store the params used for generating a map tile
    // so the next call only generates the
    // objects corresponding to the changed parameters
    map_params: Option<MapParameters>,
    // Later preview features must not be computed before their adjustment
    // section has been reached. Keep the furthest reached section separately
    // from the current frontend state so going back does not hide valid work.
    preview_section_reached: Option<MapPreviewSection>,

    // for iterating the params
    map_tiles: Vec<PreparedTile>,
    hull: geo::Polygon,
    ref_point: geo::Coord,
    thread_pool: ThreadPool,
    worker_threads: usize,
}

impl Worker {
    pub fn boot(comms: OmapComms<AppEvent, WorkerCommand>) -> crate::Result<()> {
        std::thread::Builder::new().spawn(move || -> crate::Result<()> {
            let worker_threads = std::thread::available_parallelism()
                .map(|threads| threads.get())
                .unwrap_or(8)
                .max(1);
            let thread_pool = ThreadPoolBuilder::new()
                .num_threads(worker_threads.max(1))
                .build()
                .map_err(|e| anyhow::anyhow!("Failed to create backend Rayon thread pool: {e}"))?;

            let mut backend = Worker {
                comms,
                map_params: None,
                preview_section_reached: None,
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
                WorkerCommand::ClearParams => {
                    self.map_params = None;
                    self.preview_section_reached = None;
                }
                WorkerCommand::SetWorkerThreads(worker_threads) => {
                    if let Err(e) = self.set_worker_threads(worker_threads) {
                        let _ = self.comms.send(AppEvent::Error(e.to_string(), false));
                    }
                }
                WorkerCommand::ParseCrs(paths) => {
                    match crate::lidar::parse_crs(&self.comms.sender(), paths) {
                        Ok(crate::lidar::CrsAnalysis {
                            paths,
                            crs_defs,
                            crs_less,
                            removed_unreadable,
                        }) => {
                            if removed_unreadable {
                                let _ = self.comms.send(AppEvent::Error(
                                    "Some paths were not readable as lidar files and have been removed"
                                        .to_string(),
                                    false,
                                ));
                                let _ = self
                                    .comms
                                    .send(AppEvent::UpdateVariable(Variable::Paths(paths)));
                            }
                            let _ = self
                                .comms
                                .send(AppEvent::UpdateVariable(Variable::CrsDefs(crs_defs)));
                            let _ = self
                                .comms
                                .send(AppEvent::UpdateVariable(Variable::CrsLessString(crs_less)));
                            let _ = self.comms.send(AppEvent::UpdateVariable(
                                Variable::CrsLessCheckBox(crs_less),
                            ));
                            let _ = self
                                .comms
                                .send(AppEvent::TaskComplete(TaskComplete::ParseCrs));
                        }
                        Err(error) => {
                            let _ = self.comms.send(AppEvent::Error(error.to_string(), true));
                        }
                    }
                }
                WorkerCommand::MapSpatialLidarRelations(paths, crs) => {
                    match crate::lidar::map_spatial_relations(paths, crs) {
                        Ok(crate::lidar::SpatialRelations {
                            boundaries,
                            boundary_areas,
                            home,
                            components,
                        }) => {
                            let boundaries = boundaries
                                .into_iter()
                                .map(|boundary| {
                                    boundary.map(|point| walkers::lon_lat(point.x, point.y))
                                })
                                .collect();
                            let _ = self
                                .comms
                                .send(AppEvent::UpdateVariable(Variable::Boundaries(boundaries)));
                            let _ =
                                self.comms
                                    .send(AppEvent::UpdateVariable(Variable::BoundaryAreas(
                                        boundary_areas,
                                    )));
                            let _ = self.comms.send(AppEvent::UpdateVariable(Variable::Home(
                                walkers::lon_lat(home.x, home.y),
                            )));
                            let _ = self.comms.send(AppEvent::UpdateVariable(
                                Variable::ConnectedComponents(components),
                            ));
                            let _ = self.comms.send(AppEvent::TaskComplete(
                                TaskComplete::MapSpatialLidarRelations,
                            ));
                        }
                        Err(error) => {
                            let _ = self.comms.send(AppEvent::Error(error.to_string(), true));
                        }
                    }
                }
                WorkerCommand::ConvertCopc(task) => {
                    let ConvertCopcTask {
                        paths,
                        in_crs,
                        out_crs,
                        save_location,
                        bounds,
                        polygon,
                        write_single_copc,
                        budget_gb,
                    } = *task;

                    match crate::lidar::convert_copc(
                        &self.comms.sender(),
                        paths,
                        in_crs,
                        out_crs,
                        save_location,
                        bounds
                            .into_iter()
                            .map(|boundary| boundary.map(|point| point.0))
                            .collect(),
                        polygon,
                        write_single_copc,
                        budget_gb,
                    ) {
                        Ok(crate::lidar::CopcConversionOutcome::Converted {
                            paths,
                            stats,
                            single_copc_path,
                        }) => {
                            if let Some(path) = single_copc_path {
                                let _ = self
                                    .comms
                                    .send(AppEvent::UpdateVariable(Variable::SingleCopcPath(path)));
                            }
                            let _ = self
                                .comms
                                .send(AppEvent::UpdateVariable(Variable::Stats(Box::new(stats))));
                            let _ = self
                                .comms
                                .send(AppEvent::UpdateVariable(Variable::Paths(paths)));
                            let _ = self
                                .comms
                                .send(AppEvent::TaskComplete(TaskComplete::ConvertCopc));
                        }
                        Ok(crate::lidar::CopcConversionOutcome::NoIntersection) => {
                            let _ = self.comms.send(AppEvent::Error(
                                "The chosen polygon filter does not intersect the lidar files"
                                    .to_string(),
                                false,
                            ));
                        }
                        Err(error) => {
                            let _ = self.comms.send(AppEvent::ProgressBar(ProgressBar::Finish));
                            let _ = self.comms.send(AppEvent::Error(error.to_string(), true));
                        }
                    }
                }

                WorkerCommand::InitializeMapTile(task) => {
                    self.preview_section_reached = None;
                    let InitializeMapTileTask {
                        paths,
                        test_area,
                        stats,
                    } = *task;

                    match generation::preview::initialize_map_tile(
                        &self.comms.sender(),
                        paths,
                        test_area,
                        stats,
                    ) {
                        Ok(initialized) => {
                            self.map_tiles = initialized.tiles;
                            self.hull = initialized.hull;
                            self.ref_point = initialized.ref_point;
                            let _ = self
                                .comms
                                .send(AppEvent::TaskComplete(TaskComplete::InitializeMapTile));
                        }
                        Err(e) => {
                            let _ = self.comms.send(AppEvent::ProgressBar(ProgressBar::Finish));
                            let _ = self.comms.send(AppEvent::Error(e.to_string(), true));
                        }
                    }
                }

                WorkerCommand::RegenerateMap(job_id, params, scope, cancellation) => {
                    assert!(!self.map_tiles.is_empty());
                    if let RegenerationScope::Section(section) = scope {
                        self.preview_section_reached = Some(
                            self.preview_section_reached
                                .map_or(section, |reached| reached.max(section)),
                        );
                    }
                    let completed = match generation::preview::regenerate_map_tile(
                        &self.thread_pool,
                        &self.map_tiles,
                        self.ref_point,
                        &params,
                        &self.map_params,
                        scope,
                        self.preview_section_reached,
                        &cancellation,
                    ) {
                        Ok(Some(update)) => match DrawableOmap::from_temp_map(
                            update.document,
                            self.hull.exterior().clone(),
                            &params.geometry,
                        ) {
                            Ok(map) => {
                                if let Some(score) = update.contour_score {
                                    let _ = self.comms.send(AppEvent::UpdateVariable(
                                        Variable::ContourScore(job_id, score),
                                    ));
                                }
                                let _ = self.comms.send(AppEvent::UpdateVariable(
                                    Variable::MapTile(job_id, Box::new(map)),
                                ));
                                let _ = self.comms.send(AppEvent::TaskComplete(
                                    TaskComplete::RegenerateMap(job_id),
                                ));
                                true
                            }
                            Err(error) => {
                                let _ = self.comms.send(AppEvent::Error(error.to_string(), true));
                                false
                            }
                        },
                        Ok(None) => false,
                        Err(error) => {
                            let _ = self.comms.send(AppEvent::Error(error.to_string(), true));
                            false
                        }
                    };

                    if completed && !cancellation.is_cancelled() {
                        self.map_params = Some(*params);
                    }
                }

                WorkerCommand::GenerateMap(task) => {
                    let GenerateMapRequest {
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
                            let _ = self.comms.send(AppEvent::Error(e.to_string(), true));
                            continue;
                        }
                    };

                    // we are not going back here so can clear the DEMs to free some memory
                    self.reset();

                    let _ = match generation::export::export_map(
                        &self.comms.sender(),
                        &self.thread_pool,
                        map_params,
                        file_params,
                        local_polygon_filter,
                        stats,
                    ) {
                        Ok(_) => self
                            .comms
                            .send(AppEvent::TaskComplete(TaskComplete::GenerateMap)),
                        Err(e) => self.comms.send(AppEvent::Error(e.to_string(), true)),
                    };
                }
                WorkerCommand::Reset => {
                    self.reset();
                    let _ = self.comms.send(AppEvent::TaskComplete(TaskComplete::Reset));
                }
            }
        }
    }

    fn reset(&mut self) {
        self.map_params = None;
        self.preview_section_reached = None;

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
        self.comms.send(AppEvent::Log(format!(
            "Worker worker pool set to {worker_threads} threads"
        )))?;
        Ok(())
    }
}
