use super::{
    AppState, OmapComms, OmapModal, ProcessStage,
    protocol::{
        AppAction, AppEvent, CancellationToken, ConvertCopcTask, GenerateMapRequest,
        InitializeMapTileTask, JobId, MapPreviewSection, ProgressBar, RegenerationScope, SetCrs,
        TaskComplete, Variable, WorkerCommand,
    },
    tile_sources,
    worker::Worker,
};
use eframe::egui;
use walkers::{HttpTiles, MapMemory, MercatorProjection};

pub const HOME_LON_LAT: (f64, f64) = (10.6134, 59.9594);

pub struct OmapMaker {
    // background osm and otm tiles
    pub http_tiles: (
        HttpTiles<MercatorProjection>,
        HttpTiles<MercatorProjection>,
        HttpTiles<MercatorProjection>,
    ),
    pub map_memory: MapMemory,
    pub home: walkers::Position,
    pub home_zoom: f64,

    // variables controlling what to show
    pub gui_variables: AppState,

    // modals
    pub open_modal: OmapModal,

    // app state
    pub state: ProcessStage,

    // app context
    ctx: egui::Context,

    // backend communication
    comms: OmapComms<WorkerCommand, AppEvent>,
    active_preview_job_id: Option<JobId>,
    active_preview_cancellation: Option<CancellationToken>,
    next_preview_job_id: JobId,
}

impl eframe::App for OmapMaker {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        // register all backend events that has occurred and monitor for panic
        loop {
            match self.comms.try_recv() {
                Ok(event) => self.dispatch_action(AppAction::AppEvent(event)),
                Err(recv_err) => match recv_err {
                    // message buffer empty i.e. all pending messages have been dealt with
                    std::sync::mpsc::TryRecvError::Empty => break,
                    // backend has hung up i.e. has panicked
                    std::sync::mpsc::TryRecvError::Disconnected => {
                        self.dispatch_action(AppAction::AppEvent(AppEvent::Error(
                            "The backend panicked. Starting over".to_string(),
                            true,
                        )));
                    }
                },
            }
        }

        // render correct side panel
        egui::Panel::left("Guide Panel")
            .exact_size(400.)
            .show(ui, |ui| {
                ui.style_mut().spacing.scroll = egui::style::ScrollStyle::solid();
                match self.state {
                    ProcessStage::Welcome => self.render_welcome_panel(ui),
                    ProcessStage::CheckLidar => self.render_checking_lidar_panel(ui),
                    ProcessStage::ShowComponents => self.render_show_components_panel(ui),
                    ProcessStage::ConvertingCOPC => self.render_copc_panel(ui),
                    ProcessStage::ChooseSquare => self.render_choose_test_area_panel(ui),
                    ProcessStage::DrawPolygon => self.render_draw_polygon_panel(ui),
                    ProcessStage::PrepareMapPreview => self.render_prepare_map_preview_panel(ui),
                    ProcessStage::AdjustContours
                    | ProcessStage::AdjustOpenness
                    | ProcessStage::AdjustVegetation
                    | ProcessStage::AdjustBuildings
                    | ProcessStage::AdjustCliffs
                    | ProcessStage::AdjustWater
                    | ProcessStage::AdjustMarsh
                    | ProcessStage::AdjustStreams
                    | ProcessStage::AdjustIntensity => self.render_adjust_slider_panel(ui),
                    ProcessStage::GenerateMap => self.render_generating_map_panel(ui),
                    ProcessStage::ExportDone => self.render_done_panel(ui),
                }
            });

        // render correct main panel
        egui::CentralPanel::default()
            .frame(egui::Frame::default().fill(ui.style().visuals.panel_fill))
            .show(ui, |ui| match self.state {
                ProcessStage::Welcome
                | ProcessStage::ChooseSquare
                | ProcessStage::ExportDone
                | ProcessStage::DrawPolygon
                | ProcessStage::AdjustContours
                | ProcessStage::AdjustOpenness
                | ProcessStage::AdjustVegetation
                | ProcessStage::AdjustBuildings
                | ProcessStage::AdjustCliffs
                | ProcessStage::AdjustWater
                | ProcessStage::AdjustMarsh
                | ProcessStage::AdjustStreams
                | ProcessStage::AdjustIntensity
                | ProcessStage::ShowComponents => {
                    self.render_map(ui);
                }
                ProcessStage::CheckLidar
                | ProcessStage::ConvertingCOPC
                | ProcessStage::PrepareMapPreview
                | ProcessStage::GenerateMap => self.render_console(ui),
            });

        // render the open modal
        let ctx = ui.ctx();
        match &self.open_modal {
            OmapModal::None => (),
            OmapModal::OutputCRS(crs) => self.output_crs_modal(ctx, *crs.clone()),
            OmapModal::ManualSetCRS => self.manual_set_crs_modal(ctx),
            OmapModal::SetOneCrsForAll => self.set_one_crs_for_all_modal(ctx),
            OmapModal::SetOneCrsForEach => self.set_one_crs_for_each_modal(ctx),
            OmapModal::ConfirmDropAll => self.confirm_drop_all_modal(ctx),
            OmapModal::ConfirmStartOver => self.confirm_start_over_modal(ctx),
            OmapModal::ConfirmGenerateMap => self.confirm_generate_map_modal(ctx),
            OmapModal::MultipleGraphComponents => self.multiple_graph_components_modal(ctx),
            OmapModal::ErrorModal(s) => self.error_modal(ctx, s.clone()),
            OmapModal::WaiverModal => self.waiver_modal(ctx),
        }
    }
}

// public functions
impl OmapMaker {
    pub fn new(ctx: egui::Context) -> Self {
        let (app_comms, worker_comms) = OmapComms::new(&ctx);

        // starts the computation thread
        Worker::boot(worker_comms).expect("Could not boot the worker threads");

        Self {
            http_tiles: tile_sources::get_tile_sources(&ctx),
            map_memory: Default::default(),
            state: ProcessStage::Welcome,
            ctx,
            comms: app_comms,
            active_preview_job_id: None,
            active_preview_cancellation: None,
            next_preview_job_id: 0,
            open_modal: OmapModal::None,
            home: walkers::lon_lat(HOME_LON_LAT.0, HOME_LON_LAT.1),
            home_zoom: 16.,
            gui_variables: Default::default(),
        }
    }
}

// private functions
impl OmapMaker {
    fn apply_event(&mut self, event: AppEvent) {
        match event {
            AppEvent::Log(s) => {
                self.gui_variables.log_terminal.println(&*s);
            }
            AppEvent::UpdateVariable(variable) => self.on_update_variable(variable),
            AppEvent::TaskComplete(task) => self.on_task_complete(task),
            AppEvent::Error(s, fatal) => {
                if fatal {
                    self.dispatch_action(AppAction::Reset);
                }
                self.dispatch_action(AppAction::OpenModal(OmapModal::ErrorModal(s)));
            }
            AppEvent::ProgressBar(p) => match p {
                ProgressBar::Start => self.gui_variables.log_terminal.start_progress_bar(40),
                ProgressBar::Inc(delta) => self.gui_variables.log_terminal.inc_progress_bar(delta),
                ProgressBar::Finish => self.gui_variables.log_terminal.finish_progress_bar(),
            },
        }
    }

    fn on_update_variable(&mut self, variable: Variable) {
        match variable {
            Variable::Paths(p) => self.gui_variables.project.paths = p,
            Variable::Boundaries(vec) => self.gui_variables.lidar.boundaries = vec,
            Variable::BoundaryAreas(vec) => self.gui_variables.lidar.boundary_areas = vec,
            Variable::Home(position) => self.home = position,
            Variable::CrsDefs(vec) => {
                self.gui_variables.project.crses = vec;
                self.gui_variables.update_unique_crs();
            }
            Variable::CrsLessString(num) => {
                self.gui_variables.lidar.crs_less_search_strings = vec!["".to_string(); num]
            }
            Variable::CrsLessCheckBox(num) => {
                self.gui_variables.lidar.drop_checkboxes = vec![false; num]
            }
            Variable::ConnectedComponents(vec) => {
                self.gui_variables.lidar.connected_components = vec
            }
            Variable::MapTile(job_id, drawable_omap) => {
                if self.active_preview_job_id == Some(job_id) {
                    self.gui_variables.update_map(*drawable_omap);
                }
            }
            Variable::ContourScore(job_id, score) => {
                if self.active_preview_job_id == Some(job_id) {
                    self.gui_variables.preview.contour_score = score;
                }
            }
            Variable::Stats(lidar_stats) => self.gui_variables.lidar.stats = Some(*lidar_stats),
            Variable::SingleCopcPath(path) => {
                self.gui_variables.project.single_copc_path = Some(path)
            }
        }
    }

    pub(crate) fn dispatch_action(&mut self, task: AppAction) {
        match task {
            AppAction::AppEvent(event) => self.apply_event(event),
            AppAction::SetWorkerThreads => {
                let _ = self.comms.send(WorkerCommand::SetWorkerThreads(
                    self.gui_variables.project.worker_threads,
                ));
            }
            AppAction::ParseCrs(paths) => {
                let _ = self.comms.send(WorkerCommand::ParseCrs(paths));
            }
            AppAction::SetCrs(choice) => {
                let use_local_coordinates = matches!(choice, SetCrs::Local);
                if self.update_crs(choice) {
                    self.dispatch_action(if use_local_coordinates {
                        AppAction::DoConnectedComponentAnalysis
                    } else {
                        AppAction::GetOutputCRS
                    });
                }
            }
            AppAction::GetOutputCRS => {
                if let Some(majority) = self.gui_variables.get_most_popular_crs() {
                    self.dispatch_action(AppAction::OpenModal(OmapModal::OutputCRS(Box::new(
                        majority,
                    ))));
                } else {
                    self.dispatch_action(AppAction::OpenModal(OmapModal::ManualSetCRS));
                }
            }
            AppAction::OutputCrsSelected => {
                self.open_modal = OmapModal::None;
                self.dispatch_action(AppAction::DoConnectedComponentAnalysis);
            }
            AppAction::DoConnectedComponentAnalysis => {
                let crses = self
                    .gui_variables
                    .generation
                    .params
                    .output
                    .crs
                    .as_ref()
                    .map(|_| self.gui_variables.project.crses.clone());
                let _ = self.comms.send(WorkerCommand::MapSpatialLidarRelations(
                    self.gui_variables.project.paths.clone(),
                    crses,
                ));
            }
            AppAction::QueryDropComponents => {
                self.gui_variables.log_terminal.println(
                    format!(
                        "The lidar files are not all connected and form {} parts",
                        self.gui_variables.lidar.connected_components.len()
                    )
                    .as_str(),
                );
                self.dispatch_action(AppAction::OpenModal(OmapModal::MultipleGraphComponents));
            }
            AppAction::ShowComponents => {
                self.open_modal = OmapModal::None;
                self.state = ProcessStage::ShowComponents;
            }
            AppAction::DropComponents => {
                self.open_modal = OmapModal::None;
                self.home = self.gui_variables.drop_small_graph_components();
                self.dispatch_action(AppAction::NextState);
            }
            AppAction::ConvertCopc => {
                let ready = match self.gui_variables.validate_copc_conversion() {
                    Ok(ready) => ready,
                    Err(error) => {
                        self.dispatch_action(AppAction::AppEvent(AppEvent::Error(
                            error.to_string(),
                            false,
                        )));
                        return;
                    }
                };
                if let Err(error) = self.gui_variables.prepare_test_area() {
                    self.dispatch_action(AppAction::AppEvent(AppEvent::Error(
                        error.to_string(),
                        false,
                    )));
                    return;
                }
                self.state.next();
                self.gui_variables.project.single_copc_path = None;
                let _ = self
                    .comms
                    .send(WorkerCommand::ConvertCopc(Box::new(ConvertCopcTask {
                        paths: ready.file_params.paths,
                        in_crs: ready.file_params.crs_epsg,
                        out_crs: ready.output_crs,
                        save_location: ready.save_location,
                        bounds: ready.boundaries,
                        polygon: ready.polygon_filter,
                        write_single_copc: ready.write_single_copc,
                        budget_gb: ready.memory_budget_gb,
                    })));
            }
            AppAction::InitializeMapTile => {
                let ready = match self.gui_variables.validate_map_preview() {
                    Ok(ready) => ready,
                    Err(error) => {
                        self.dispatch_action(AppAction::AppEvent(AppEvent::Error(
                            error.to_string(),
                            false,
                        )));
                        return;
                    }
                };
                self.gui_variables.preview.generating_map_tile = true;
                self.state.next();
                let _ = self.comms.send(WorkerCommand::InitializeMapTile(Box::new(
                    InitializeMapTileTask {
                        paths: ready.paths,
                        test_area: ready.test_area,
                        stats: ready.stats,
                    },
                )));
            }
            AppAction::RegenerateMap(scope) => self.regenerate_map(scope),
            AppAction::GenerateMap => {
                let ready = match self.gui_variables.validate_final_map() {
                    Ok(ready) => ready,
                    Err(error) => {
                        self.dispatch_action(AppAction::AppEvent(AppEvent::Error(
                            error.to_string(),
                            false,
                        )));
                        return;
                    }
                };
                self.state.next();
                let _ = self
                    .comms
                    .send(WorkerCommand::GenerateMap(Box::new(GenerateMapRequest {
                        map_params: ready.map_params,
                        file_params: ready.file_params,
                        polygon_filter: ready.polygon_filter,
                        stats: ready.stats,
                    })));
            }
            AppAction::OpenModal(modal) => self.open_modal = modal,
            AppAction::NextState => self.next_state(),
            AppAction::PrevState => self.prev_state(),
            AppAction::Reset => self.reset(),
        }
    }

    fn on_task_complete(&mut self, task: TaskComplete) {
        match task {
            TaskComplete::ParseCrs => {
                if self.gui_variables.project.crses.iter().any(Option::is_none) {
                    self.dispatch_action(AppAction::OpenModal(OmapModal::ManualSetCRS));
                } else {
                    self.dispatch_action(AppAction::GetOutputCRS);
                }
            }
            TaskComplete::MapSpatialLidarRelations => {
                if self.gui_variables.lidar.connected_components.len() == 1 {
                    self.gui_variables
                        .log_terminal
                        .println("The lidar files are all connected.");
                    self.dispatch_action(AppAction::NextState);
                } else {
                    self.gui_variables
                        .log_terminal
                        .println("The remaining lidar files are all connected.");
                    self.dispatch_action(AppAction::QueryDropComponents);
                }
            }
            TaskComplete::ConvertCopc => self.dispatch_action(AppAction::NextState),
            TaskComplete::RegenerateMap(job_id) => {
                if self.active_preview_job_id == Some(job_id) {
                    self.gui_variables.preview.generating_map_tile = false;
                    self.active_preview_job_id = None;
                    self.active_preview_cancellation = None;
                }
            }
            TaskComplete::GenerateMap => self.dispatch_action(AppAction::NextState),
            TaskComplete::Reset => (),
            TaskComplete::InitializeMapTile => {
                self.gui_variables.preview.generating_map_tile = false;
                self.dispatch_action(AppAction::NextState);
            }
        }
    }

    fn next_state(&mut self) {
        match self.state {
            ProcessStage::Welcome => {
                let ready = match self.gui_variables.project.validate_welcome() {
                    Ok(ready) => ready,
                    Err(error) => {
                        self.dispatch_action(AppAction::AppEvent(AppEvent::Error(
                            error.to_string(),
                            false,
                        )));
                        return;
                    }
                };
                self.state.next();
                self.gui_variables.project.selected_file = None;
                self.dispatch_action(AppAction::SetWorkerThreads);
                self.dispatch_action(AppAction::ParseCrs(ready.paths));
            }
            ProcessStage::CheckLidar => {
                self.state.next();
                self.map_memory.follow_my_position();
                if self.gui_variables.project.paths.len() == 1 {
                    self.gui_variables.project.selected_file = Some(0);
                }
            }
            ProcessStage::DrawPolygon => self.dispatch_action(AppAction::ConvertCopc),
            ProcessStage::ConvertingCOPC => {
                self.state.next();
                self.map_memory.follow_my_position();
            }
            ProcessStage::ChooseSquare => self.dispatch_action(AppAction::InitializeMapTile),
            state if state.is_adjustment() && state != ProcessStage::AdjustIntensity => {
                self.state.next();
                let section = match self.state {
                    ProcessStage::AdjustContours => MapPreviewSection::Contours,
                    ProcessStage::AdjustOpenness => MapPreviewSection::Openness,
                    ProcessStage::AdjustVegetation => MapPreviewSection::Vegetation,
                    ProcessStage::AdjustBuildings => MapPreviewSection::Buildings,
                    ProcessStage::AdjustCliffs => MapPreviewSection::Cliffs,
                    ProcessStage::AdjustWater => MapPreviewSection::Water,
                    ProcessStage::AdjustMarsh => MapPreviewSection::Marsh,
                    ProcessStage::AdjustStreams => MapPreviewSection::Streams,
                    ProcessStage::AdjustIntensity => MapPreviewSection::Intensity,
                    _ => unreachable!("The next adjustment stage must be adjustable"),
                };
                self.dispatch_action(AppAction::RegenerateMap(RegenerationScope::Section(
                    section,
                )));
            }
            ProcessStage::AdjustIntensity => self.dispatch_action(AppAction::GenerateMap),
            ProcessStage::GenerateMap => {
                self.state.next();
                self.dispatch_action(AppAction::OpenModal(OmapModal::WaiverModal));
            }
            ProcessStage::PrepareMapPreview => {
                self.state.next();
                self.dispatch_action(AppAction::RegenerateMap(RegenerationScope::Section(
                    MapPreviewSection::Contours,
                )));
            }
            _ => unreachable!(
                "Should not call next on state for {:?} variant.",
                self.state
            ),
        }
    }

    fn prev_state(&mut self) {
        match self.state {
            ProcessStage::AdjustContours => {
                self.gui_variables.preview.map_tile = None;
                self.gui_variables.tile.selected_square = None;
                self.gui_variables.tile.selected_square_boundary = None;
                let _ = self.comms.send(WorkerCommand::ClearParams);
            }
            state if state.is_adjustment() => (),
            ProcessStage::ShowComponents => {
                self.gui_variables.project.selected_file = None;
                self.open_modal = OmapModal::MultipleGraphComponents;
            }
            _ => return,
        }

        self.state.prev();
    }

    fn update_crs(&mut self, message: SetCrs) -> bool {
        match message {
            SetCrs::SetAllEpsg => {
                let Ok(a) = self.gui_variables.lidar.crs_less_search_strings[0].parse::<u16>()
                else {
                    self.dispatch_action(AppAction::AppEvent(AppEvent::Error(
                        "Could not parse EPSG code".to_string(),
                        false,
                    )));
                    return false;
                };

                let Ok(parsed_crs) = proj_wkt::parse_crs(&a.to_string()) else {
                    self.dispatch_action(AppAction::AppEvent(AppEvent::Error(
                        "Could not create a CRS from the given EPSG code".to_string(),
                        false,
                    )));
                    return false;
                };

                for crs in self.gui_variables.project.crses.iter_mut() {
                    if crs.is_none() {
                        *crs = Some(parsed_crs.clone());
                    }
                }
            }
            SetCrs::SetEachCrs => {
                let mut drop_list = vec![];
                let mut crs_less_indecies = vec![];
                for (i, crs) in self.gui_variables.project.crses.iter().enumerate() {
                    if crs.is_none() {
                        crs_less_indecies.push(i);
                    }
                }
                for (i, s) in self
                    .gui_variables
                    .lidar
                    .crs_less_search_strings
                    .iter()
                    .enumerate()
                {
                    if self.gui_variables.lidar.drop_checkboxes[i] {
                        drop_list.push(crs_less_indecies[i]);
                    } else {
                        let crs = match proj_wkt::parse_crs(s) {
                            Ok(crs) => Some(crs),
                            Err(_) => {
                                self.dispatch_action(AppAction::AppEvent(AppEvent::Error(
                                    "Could not create a CRS from one of the provided codes"
                                        .to_string(),
                                    false,
                                )));
                                return false;
                            }
                        };
                        self.gui_variables.project.crses[crs_less_indecies[i]] = crs;
                    }
                }
                drop_list.sort_by(|a: &usize, b: &usize| b.cmp(a));
                for i in drop_list {
                    self.gui_variables.project.paths.remove(i);
                    self.gui_variables.project.crses.remove(i);
                }
            }
            SetCrs::Default => {
                let mut default_crs = None;
                for a in self.gui_variables.project.crses.iter() {
                    if a.is_some() {
                        default_crs = a.clone();
                        break;
                    }
                }
                assert!(
                    default_crs.is_some(),
                    "Default crs button available but should not have been"
                );

                self.gui_variables.project.crses =
                    vec![default_crs; self.gui_variables.project.paths.len()];
            }
            SetCrs::DropAll => {
                let mut drop_list = vec![];
                for (i, crs) in self.gui_variables.project.crses.iter().enumerate() {
                    if crs.is_none() {
                        drop_list.push(i);
                    }
                }
                drop_list.sort_by(|a: &usize, b: &usize| b.cmp(a));
                for i in drop_list {
                    self.gui_variables.project.paths.remove(i);
                    self.gui_variables.project.crses.remove(i);
                }
            }
            SetCrs::Local => (),
        }

        assert!(self.gui_variables.project.paths.len() == self.gui_variables.project.crses.len());

        if self.gui_variables.project.paths.is_empty() {
            self.dispatch_action(AppAction::AppEvent(AppEvent::Error(
                "All Lidar files were dropped.".to_string(),
                true,
            )));
            false
        } else {
            self.gui_variables.lidar.crs_less_search_strings.clear();
            self.gui_variables.lidar.drop_checkboxes.clear();
            self.open_modal = OmapModal::None;
            true
        }
    }

    fn regenerate_map(&mut self, scope: RegenerationScope) {
        if let Some(cancellation) = self.active_preview_cancellation.take() {
            cancellation.cancel();
        }
        self.next_preview_job_id = self.next_preview_job_id.wrapping_add(1);
        let job_id = self.next_preview_job_id;
        let cancellation = CancellationToken::default();
        self.active_preview_job_id = Some(job_id);
        self.active_preview_cancellation = Some(cancellation.clone());
        self.gui_variables.preview.generating_map_tile = true;
        let _ = self.comms.send(WorkerCommand::RegenerateMap(
            job_id,
            Box::new(self.gui_variables.generation.params.clone()),
            scope,
            cancellation,
        ));
    }

    fn reset(&mut self) {
        if let Some(cancellation) = self.active_preview_cancellation.take() {
            cancellation.cancel();
        }
        self.home = walkers::lon_lat(HOME_LON_LAT.0, HOME_LON_LAT.1);
        self.gui_variables = Default::default();
        self.active_preview_job_id = None;
        self.next_preview_job_id = 0;
        self.open_modal = OmapModal::None;
        self.home_zoom = 16.;
        let _ = self.map_memory.set_zoom(self.home_zoom);
        self.map_memory.follow_my_position();

        match self.comms.send(WorkerCommand::Reset) {
            Ok(_) => (),
            Err(_) => self.restart_worker(),
        }

        self.state = ProcessStage::Welcome;
    }

    fn restart_worker(&mut self) {
        // start backend thread
        let (app_comms, worker_comms) = OmapComms::new(&self.ctx);

        // starts the backend on its own thread
        Worker::boot(worker_comms).expect("Could not restart the background threads");
        self.comms = app_comms;
    }
}
