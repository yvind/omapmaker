use crate::backend::Backend;
use crate::comms::{OmapComms, messages::*};
use crate::gui::{GuiVariables, ProcessStage, modals::OmapModal};
use eframe::egui;
use walkers::{HttpTiles, MapMemory, MercatorProjection, Position, sources};

pub const HOME_LON_LAT: (f64, f64) = (10.6134, 59.9594);

pub struct OmapMaker {
    // background osm and otm tiles
    pub http_tiles: (
        HttpTiles<MercatorProjection>,
        HttpTiles<MercatorProjection>,
        HttpTiles<MercatorProjection>,
    ),
    pub map_memory: MapMemory,
    pub home: Position,
    pub home_zoom: f64,

    // variables controlling what to show
    pub gui_variables: GuiVariables,

    // modals
    pub open_modal: OmapModal,

    // app state
    pub state: ProcessStage,

    // app context
    ctx: egui::Context,

    // backend communication
    comms: OmapComms<BackendTask, FrontendTask>,
    active_preview_job_id: Option<JobId>,
    next_preview_job_id: JobId,
}

impl eframe::App for OmapMaker {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        // register all events that has occurred and monitor for backend panic
        loop {
            match self.comms.try_recv() {
                Ok(event) => self.on_frontend_task(event),
                Err(recv_err) => match recv_err {
                    // message buffer empty i.e. all pending messages have been dealt with
                    std::sync::mpsc::TryRecvError::Empty => break,
                    // backend has hung up i.e. has panicked
                    std::sync::mpsc::TryRecvError::Disconnected => {
                        self.on_frontend_task(FrontendTask::Error(
                            "The backend panicked. Starting over".to_string(),
                            true,
                        ))
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
                    ProcessStage::ChooseSquare => self.render_choose_lidar_panel(ui, true),
                    ProcessStage::ChooseSubTile => self.render_choose_tile_panel(ui),
                    ProcessStage::DrawPolygon => self.render_draw_polygon_panel(ui),
                    ProcessStage::PrepareMapPreview => self.render_prepare_map_preview_panel(ui),
                    ProcessStage::AdjustContours
                    | ProcessStage::AdjustOpenness
                    | ProcessStage::AdjustVegetation
                    | ProcessStage::AdjustCliffs
                    | ProcessStage::AdjustIntensity => self.render_adjust_slider_panel(ui),
                    ProcessStage::MakeMap => self.render_generating_map_panel(ui),
                    ProcessStage::ExportDone => self.render_done_panel(ui),
                }
            });

        // render correct main panel
        egui::CentralPanel::default()
            .frame(egui::Frame::default().fill(ui.style().visuals.panel_fill))
            .show(ui, |ui| match self.state {
                ProcessStage::Welcome
                | ProcessStage::ChooseSquare
                | ProcessStage::ChooseSubTile
                | ProcessStage::ExportDone
                | ProcessStage::DrawPolygon
                | ProcessStage::AdjustContours
                | ProcessStage::AdjustOpenness
                | ProcessStage::AdjustVegetation
                | ProcessStage::AdjustCliffs
                | ProcessStage::AdjustIntensity
                | ProcessStage::ShowComponents => {
                    self.render_map(ui);
                }
                ProcessStage::CheckLidar
                | ProcessStage::ConvertingCOPC
                | ProcessStage::PrepareMapPreview
                | ProcessStage::MakeMap => self.render_console(ui),
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
            OmapModal::ConfirmMakeMap => self.confirm_make_map_modal(ctx),
            OmapModal::MultipleGraphComponents => self.multiple_graph_components_modal(ctx),
            OmapModal::ErrorModal(s) => self.error_modal(ctx, s.clone()),
            OmapModal::WaiverModal => self.waiver_modal(ctx),
        }
    }
}

pub struct ArcGisSource;

impl walkers::sources::TileSource for ArcGisSource {
    type Projection = walkers::MercatorProjection;

    fn tile_url(&self, tile_id: walkers::TileId) -> String {
        format!(
            "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{}/{}/{}",
            tile_id.zoom, tile_id.y, tile_id.x
        )
    }

    fn attribution(&self) -> sources::Attribution {
        sources::Attribution {
            text: "nope",
            url: "lol",
            logo_light: None,
            logo_dark: None,
        }
    }

    fn projection(&self) -> Self::Projection {
        MercatorProjection
    }
}

// public functions
impl OmapMaker {
    pub fn new(ctx: egui::Context) -> Self {
        let (frontend_comms, backend_comms) = OmapComms::new(&ctx);

        // starts the computation thread
        Backend::boot(backend_comms).expect("Could not boot the worker threads");

        let http_tiles = (
            HttpTiles::new(sources::OpenStreetMap, ctx.clone()),
            HttpTiles::new(
                sources::OpenTopoMap(sources::OpenTopoServer::C),
                ctx.clone(),
            ),
            HttpTiles::new(ArcGisSource, ctx.clone()),
        );

        Self {
            http_tiles,
            map_memory: Default::default(),
            state: ProcessStage::Welcome,
            ctx,
            comms: frontend_comms,
            active_preview_job_id: None,
            next_preview_job_id: 0,
            open_modal: OmapModal::None,
            home: walkers::lon_lat(HOME_LON_LAT.0, HOME_LON_LAT.1),
            home_zoom: 16.,
            gui_variables: Default::default(),
        }
    }

    pub fn on_frontend_task(&mut self, event: FrontendTask) {
        match event {
            FrontendTask::Log(s) => {
                self.gui_variables.log_terminal.println(&*s);
            }
            FrontendTask::UpdateVariable(variable) => self.on_update_variable(variable),
            FrontendTask::TaskComplete(task) => self.on_task_complete(task),
            FrontendTask::OpenModal(modal) => self.open_modal = modal,
            FrontendTask::DelegateTask(task) => self.start_task(task),
            FrontendTask::NextState => self.next_state(),
            FrontendTask::PrevState => self.prev_state(),
            FrontendTask::Error(s, fatal) => {
                if fatal {
                    self.reset();
                }
                self.open_modal = OmapModal::ErrorModal(s.clone());
            }
            FrontendTask::ProgressBar(p) => match p {
                ProgressBar::Start => self.gui_variables.log_terminal.start_progress_bar(40),
                ProgressBar::Inc(delta) => self.gui_variables.log_terminal.inc_progress_bar(delta),
                ProgressBar::Finish => self.gui_variables.log_terminal.finish_progress_bar(),
            },
        }
    }
}

// private functions
impl OmapMaker {
    fn on_update_variable(&mut self, variable: Variable) {
        match variable {
            Variable::Paths(p) => self.gui_variables.project.paths = p,
            Variable::Boundaries(vec) => self.gui_variables.lidar.boundaries = vec,
            Variable::BoundaryAreas(vec) => self.gui_variables.lidar.boundary_areas = vec,
            Variable::Home(position) => self.home = position,
            Variable::CrsDefs(vec) => {
                self.gui_variables.project.crs_epsg = vec;
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
            Variable::TileBounds(tb) => self.gui_variables.tile.subtile_boundaries = tb,
            Variable::TileNeighbors(tn) => self.gui_variables.tile.subtile_neighbors = tn,
            Variable::ContourScore(job_id, score) => {
                if self.active_preview_job_id == Some(job_id) {
                    self.gui_variables.preview.contour_score = score;
                }
            }
            Variable::Stats(lidar_stats) => self.gui_variables.lidar.stats = Some(lidar_stats),
            Variable::SingleCopcPath(path) => {
                self.gui_variables.project.single_copc_path = Some(path)
            }
        }
    }

    fn start_task(&mut self, task: Task) {
        match task {
            Task::RegenerateMap => {
                self.regenerate_map(RegenerationScope::Changed);
            }
            Task::Reset => self.reset(),
            Task::SetCrs(s) => self.update_crs(s),
            Task::ShowComponents => self.state = ProcessStage::ShowComponents,
            Task::DropComponents => {
                let new_home = self.gui_variables.drop_small_graph_components();

                self.home = new_home;
                self.on_task_complete(TaskDone::DropComponents);
            }
            Task::GetOutputCRS => {
                if let Some(majority) = self.gui_variables.get_most_popular_crs() {
                    self.open_modal = OmapModal::OutputCRS(Box::new(majority));
                } else {
                    self.open_modal = OmapModal::ManualSetCRS;
                }
            }
            Task::QueryDropComponents => {
                self.gui_variables.log_terminal.println(
                    format!(
                        "The lidar files are not all connected and form {} parts",
                        self.gui_variables.lidar.connected_components.len()
                    )
                    .as_str(),
                );
                self.open_modal = OmapModal::MultipleGraphComponents;
            }
            Task::DoConnectedComponentAnalysis => {
                if self.gui_variables.generation.params.output.crs.is_some() {
                    let _ = self.comms.send(BackendTask::MapSpatialLidarRelations(
                        self.gui_variables.project.paths.clone(),
                        Some(self.gui_variables.project.crs_epsg.clone()),
                    ));
                } else {
                    let _ = self.comms.send(BackendTask::MapSpatialLidarRelations(
                        self.gui_variables.project.paths.clone(),
                        None,
                    ));
                }
            }
        }
    }

    fn on_task_complete(&mut self, task: TaskDone) {
        match task {
            TaskDone::TileSelectedFile => {
                if self.gui_variables.tile.subtile_boundaries.len() <= 9 {
                    self.gui_variables.tile.selected_tile =
                        Some(self.gui_variables.tile.subtile_boundaries.len() / 2);
                    self.next_state();
                }
            }
            TaskDone::ParseCrs(m) => {
                if let SetCrs::Local = m {
                    self.on_frontend_task(FrontendTask::TaskComplete(TaskDone::OutputCrs));
                } else {
                    self.on_frontend_task(FrontendTask::DelegateTask(Task::GetOutputCRS));
                }
            }
            TaskDone::MapSpatialLidarRelations => {
                if self.gui_variables.lidar.connected_components.len() == 1 {
                    self.on_frontend_task(FrontendTask::TaskComplete(TaskDone::DropComponents));
                    self.gui_variables
                        .log_terminal
                        .println("The lidar files are all connected.");
                } else {
                    self.on_frontend_task(FrontendTask::DelegateTask(Task::QueryDropComponents));
                    self.gui_variables
                        .log_terminal
                        .println("The remaining lidar files are all connected.");
                }
            }
            TaskDone::DropComponents => self.next_state(),
            TaskDone::OutputCrs => {
                self.on_frontend_task(FrontendTask::DelegateTask(
                    Task::DoConnectedComponentAnalysis,
                ));
            }
            TaskDone::ConvertCopc => {
                self.next_state();
            }
            TaskDone::RegenerateMap(job_id) => {
                if self.active_preview_job_id == Some(job_id) {
                    self.gui_variables.preview.generating_map_tile = false;
                    self.active_preview_job_id = None;
                }
            }
            TaskDone::MakeMap => self.next_state(),
            TaskDone::Reset => (),
            TaskDone::InitializeMapTile => {
                self.gui_variables.preview.generating_map_tile = false;
                self.state.next();
                self.start_task(Task::RegenerateMap);
            }
        }
    }

    fn next_state(&mut self) {
        match self.state {
            ProcessStage::Welcome => {
                let ready = match self.gui_variables.project.validate_welcome() {
                    Ok(ready) => ready,
                    Err(error) => {
                        self.on_frontend_task(FrontendTask::Error(error.to_string(), false));
                        return;
                    }
                };
                self.state.next();
                self.gui_variables.project.selected_file = None;
                let _ = self.comms.send(BackendTask::SetWorkerThreads(
                    self.gui_variables.project.worker_threads,
                ));
                let _ = self.comms.send(BackendTask::ParseCrs(ready.paths));
            }
            ProcessStage::CheckLidar => {
                self.state.next();
                self.map_memory.follow_my_position();
                if self.gui_variables.project.paths.len() == 1 {
                    self.gui_variables.project.selected_file = Some(0);
                }
            }
            ProcessStage::DrawPolygon => {
                let ready = match self.gui_variables.validate_copc_conversion() {
                    Ok(ready) => ready,
                    Err(error) => {
                        self.on_frontend_task(FrontendTask::Error(error.to_string(), false));
                        return;
                    }
                };
                self.state.next();
                self.gui_variables.project.single_copc_path = None;
                let _ = self.comms.send(BackendTask::ConvertCopc(
                    ready.file_params.paths,
                    ready.file_params.crs_epsg,
                    ready.output_crs,
                    ready.save_location,
                    ready.boundaries,
                    ready.polygon_filter,
                    ready.write_single_copc,
                ));
            }
            ProcessStage::ConvertingCOPC => {
                self.state.next();
                self.map_memory.follow_my_position();
            }
            ProcessStage::ChooseSquare => {
                let ready = match self.gui_variables.project.validate_selected_file() {
                    Ok(ready) => ready,
                    Err(error) => {
                        self.on_frontend_task(FrontendTask::Error(error.to_string(), false));
                        return;
                    }
                };
                self.state.next();
                let _ = self
                    .comms
                    .send(BackendTask::TileSelectedFile(ready.path, ready.crs));
            }
            ProcessStage::ChooseSubTile => {
                if let Err(error) = self.gui_variables.tile.validate_selected_tile() {
                    self.on_frontend_task(FrontendTask::Error(error.to_string(), false));
                    return;
                }
                let ready = match self.gui_variables.validate_map_preview() {
                    Ok(ready) => ready,
                    Err(error) => {
                        self.on_frontend_task(FrontendTask::Error(error.to_string(), false));
                        return;
                    }
                };
                self.gui_variables.preview.generating_map_tile = true;
                self.state.next();
                let _ = self.comms.send(BackendTask::InitializeMapTile(
                    ready.path,
                    ready.tile,
                    ready.stats,
                ));
            }
            state if state.is_adjustment() && state != ProcessStage::AdjustIntensity => {
                self.state.next();
                self.regenerate_current_adjustment_section();
            }
            ProcessStage::AdjustIntensity => {
                let ready = match self.gui_variables.validate_final_map() {
                    Ok(ready) => ready,
                    Err(error) => {
                        self.on_frontend_task(FrontendTask::Error(error.to_string(), false));
                        return;
                    }
                };
                self.state.next();
                let _ = self.comms.send(BackendTask::MakeMap(
                    Box::new(ready.map_params),
                    Box::new(ready.file_params),
                    ready.polygon_filter,
                    ready.stats,
                ));
            }
            ProcessStage::MakeMap => {
                self.state.next();
                self.open_modal = OmapModal::WaiverModal;
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
                self.gui_variables.tile.selected_tile = None;
                let _ = self.comms.send(BackendTask::ClearParams);
            }
            state if state.is_adjustment() => (),
            ProcessStage::ShowComponents => {
                self.gui_variables.project.selected_file = None;
                self.open_modal = OmapModal::MultipleGraphComponents;
            }
            ProcessStage::ChooseSubTile => {
                self.gui_variables.tile.selected_tile = None;
            }
            _ => return,
        }

        self.state.prev();
    }

    fn update_crs(&mut self, message: SetCrs) {
        match message {
            SetCrs::SetAllEpsg => {
                let Ok(a) = self.gui_variables.lidar.crs_less_search_strings[0].parse::<u16>()
                else {
                    self.on_frontend_task(FrontendTask::Error(
                        "Could not parse EPSG code".to_string(),
                        false,
                    ));
                    return;
                };

                let Ok(parsed_crs) = proj_wkt::parse_crs(&a.to_string()) else {
                    self.on_frontend_task(FrontendTask::Error(
                        "Could not create a CRS from the given EPSG code".to_string(),
                        false,
                    ));
                    return;
                };

                for crs in self.gui_variables.project.crs_epsg.iter_mut() {
                    if crs.is_none() {
                        *crs = Some(parsed_crs.clone());
                    }
                }
            }
            SetCrs::SetEachCrs => {
                let mut drop_list = vec![];
                let mut crs_less_indecies = vec![];
                for (i, crs) in self.gui_variables.project.crs_epsg.iter().enumerate() {
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
                                self.on_frontend_task(FrontendTask::Error(
                                    "Could not create a CRS from one of the provided codes"
                                        .to_string(),
                                    false,
                                ));
                                return;
                            }
                        };
                        self.gui_variables.project.crs_epsg[crs_less_indecies[i]] = crs;
                    }
                }
                drop_list.sort_by(|a: &usize, b: &usize| b.cmp(a));
                for i in drop_list {
                    self.gui_variables.project.paths.remove(i);
                    self.gui_variables.project.crs_epsg.remove(i);
                }
            }
            SetCrs::Default => {
                let mut default_crs = None;
                for a in self.gui_variables.project.crs_epsg.iter() {
                    if a.is_some() {
                        default_crs = a.clone();
                        break;
                    }
                }
                assert!(
                    default_crs.is_some(),
                    "Default crs button available but should not have been"
                );

                self.gui_variables.project.crs_epsg =
                    vec![default_crs; self.gui_variables.project.paths.len()];
            }
            SetCrs::DropAll => {
                let mut drop_list = vec![];
                for (i, crs) in self.gui_variables.project.crs_epsg.iter().enumerate() {
                    if crs.is_none() {
                        drop_list.push(i);
                    }
                }
                drop_list.sort_by(|a: &usize, b: &usize| b.cmp(a));
                for i in drop_list {
                    self.gui_variables.project.paths.remove(i);
                    self.gui_variables.project.crs_epsg.remove(i);
                }
            }
            _ => (),
        }

        assert!(
            self.gui_variables.project.paths.len() == self.gui_variables.project.crs_epsg.len()
        );

        if self.gui_variables.project.paths.is_empty() {
            self.on_frontend_task(FrontendTask::Error(
                "All Lidar files were dropped.".to_string(),
                true,
            ));
        } else {
            self.gui_variables.lidar.crs_less_search_strings.clear();
            self.gui_variables.lidar.drop_checkboxes.clear();
            self.on_task_complete(TaskDone::ParseCrs(message));
        }
    }

    fn regenerate_map(&mut self, scope: RegenerationScope) {
        self.next_preview_job_id = self.next_preview_job_id.wrapping_add(1);
        let job_id = self.next_preview_job_id;
        self.active_preview_job_id = Some(job_id);
        self.gui_variables.preview.generating_map_tile = true;
        let _ = self.comms.send(BackendTask::RegenerateMap(
            job_id,
            Box::new(self.gui_variables.generation.params.clone()),
            scope,
        ));
    }

    fn regenerate_current_adjustment_section(&mut self) {
        let section = match self.state {
            ProcessStage::AdjustOpenness => MapPreviewSection::Openness,
            ProcessStage::AdjustVegetation => MapPreviewSection::Vegetation,
            ProcessStage::AdjustCliffs => MapPreviewSection::Cliffs,
            ProcessStage::AdjustIntensity => MapPreviewSection::Intensity,
            _ => return,
        };

        self.regenerate_map(RegenerationScope::Section(section));
    }

    fn reset(&mut self) {
        self.home = walkers::lon_lat(HOME_LON_LAT.0, HOME_LON_LAT.1);
        self.gui_variables = Default::default();
        self.active_preview_job_id = None;
        self.next_preview_job_id = 0;
        self.open_modal = OmapModal::None;
        self.home_zoom = 16.;
        let _ = self.map_memory.set_zoom(self.home_zoom);
        self.map_memory.follow_my_position();

        match self.comms.send(BackendTask::Reset) {
            Ok(_) => (),
            Err(_) => self.restart_backend(),
        }

        self.state = ProcessStage::Welcome;
    }

    fn restart_backend(&mut self) {
        // start backend thread
        let (frontend_comms, backend_comms) = OmapComms::new(&self.ctx);

        // starts the backend on its own thread
        Backend::boot(backend_comms).expect("Could not restart the background threads");
        self.comms = frontend_comms;
    }
}
