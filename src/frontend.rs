use crate::backend::Backend;
use crate::comms::{messages::*, OmapComms};
use crate::gui::{modals::OmapModal, GuiVariables, ProcessStage};
use eframe::egui;
use walkers::{sources, HttpTiles, MapMemory, Position};

pub const HOME_LON_LAT: (f64, f64) = (10.6134, 59.9594);

pub struct OmapMaker {
    // background osm and otm tiles
    pub http_tiles: (HttpTiles, HttpTiles),
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
}

impl eframe::App for OmapMaker {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
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
        egui::SidePanel::left("Guide Panel")
            .exact_width(400.)
            .show(ctx, |ui| {
                ui.style_mut().spacing.scroll = egui::style::ScrollStyle::solid();
                match self.state {
                    ProcessStage::Welcome => self.render_welcome_panel(ui),
                    ProcessStage::CheckLidar => self.render_checking_lidar_panel(ui),
                    ProcessStage::ShowComponents => self.render_show_components_panel(ui),
                    ProcessStage::ConvertingCOPC => self.render_copc_panel(ui),
                    ProcessStage::ChooseSquare => self.render_choose_lidar_panel(ui, true),
                    ProcessStage::ChooseSubTile => self.render_choose_tile_panel(ui),
                    ProcessStage::DrawPolygon => self.render_choose_lidar_panel(ui, false),
                    ProcessStage::AdjustSliders => self.render_adjust_slider_panel(ui),
                    ProcessStage::MakeMap => self.render_generating_map_panel(ui),
                    ProcessStage::ExportDone => self.render_done_panel(ui),
                }
            });

        // render correct main panel
        egui::CentralPanel::default()
            .frame(egui::Frame::default().fill(ctx.style().visuals.panel_fill))
            .show(ctx, |ui| match self.state {
                ProcessStage::Welcome
                | ProcessStage::AdjustSliders
                | ProcessStage::ChooseSquare
                | ProcessStage::ChooseSubTile
                | ProcessStage::ExportDone
                | ProcessStage::DrawPolygon
                | ProcessStage::ShowComponents => {
                    self.render_map(ui);
                }
                ProcessStage::CheckLidar | ProcessStage::ConvertingCOPC | ProcessStage::MakeMap => {
                    self.render_console(ui)
                }
            });

        // render the open modal
        match &self.open_modal {
            OmapModal::None => (),
            OmapModal::OutputCRS(epsg) => self.output_crs_modal(ctx, *epsg),
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

// public functions
impl OmapMaker {
    pub fn new(ctx: egui::Context) -> Self {
        let (frontend_comms, backend_comms) = OmapComms::new();

        // starts the computation thread
        Backend::boot(backend_comms, ctx.clone());

        let rand_server = fastrand::choice([
            sources::OpenTopoServer::A,
            sources::OpenTopoServer::B,
            sources::OpenTopoServer::C,
        ])
        .unwrap();
        let http_tiles = (
            HttpTiles::new(sources::OpenStreetMap, ctx.clone()),
            HttpTiles::new(sources::OpenTopoMap(rand_server), ctx.clone()),
        );

        Self {
            http_tiles,
            map_memory: Default::default(),
            state: ProcessStage::Welcome,
            ctx,
            comms: frontend_comms,
            open_modal: OmapModal::None,
            home: walkers::pos_from_lon_lat(HOME_LON_LAT.0, HOME_LON_LAT.1),
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
            FrontendTask::ProgressBar(p) => {
                self.update_progress_bar(p);
            }
        }
    }
}

// private functions
impl OmapMaker {
    fn on_update_variable(&mut self, variable: Variable) {
        match variable {
            Variable::Paths(p) => self.gui_variables.file_params.paths = p,
            Variable::Boundaries(vec) => self.gui_variables.boundaries = vec,
            Variable::Home(position) => self.home = position,
            Variable::CrsEPSG(vec) => {
                self.gui_variables.file_params.crs_epsg = vec;
                self.gui_variables.update_unique_crs();
            }
            Variable::CrsLessString(num) => {
                self.gui_variables.crs_less_search_strings = vec!["".to_string(); num]
            }
            Variable::CrsLessCheckBox(num) => self.gui_variables.drop_checkboxes = vec![false; num],
            Variable::ConnectedComponents(vec) => self.gui_variables.connected_components = vec,
            Variable::MapTile(drawable_omap) => self.gui_variables.update_map(*drawable_omap),
            Variable::TileBounds(tb) => self.gui_variables.subtile_boundaries = tb,
            Variable::TileNeighbors(tn) => self.gui_variables.subtile_neighbors = tn,
            Variable::ContourScore(score) => self.gui_variables.contour_score = score,
            Variable::Stats(lidar_stats) => self.gui_variables.lidar_stats = Some(lidar_stats),
        }
    }

    fn start_task(&mut self, task: Task) {
        match task {
            Task::RegenerateMap => {
                self.gui_variables.generating_map_tile = true;
                self.comms
                    .send(BackendTask::RegenerateMap(Box::new(
                        self.gui_variables.map_params.clone(),
                    )))
                    .unwrap();
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
                let majority = self.gui_variables.get_most_popular_crs().unwrap();
                self.open_modal = OmapModal::OutputCRS(majority);
            }
            Task::QueryDropComponents => {
                self.gui_variables.log_terminal.println(
                    format!(
                        "The lidar files are not all connected and form {} parts",
                        self.gui_variables.connected_components.len()
                    )
                    .as_str(),
                );
                self.open_modal = OmapModal::MultipleGraphComponents;
            }
            Task::DoConnectedComponentAnalysis => {
                if self.gui_variables.map_params.output_epsg.is_some() {
                    self.comms
                        .send(BackendTask::MapSpatialLidarRelations(
                            self.gui_variables.file_params.paths.clone(),
                            Some(self.gui_variables.file_params.crs_epsg.clone()),
                        ))
                        .unwrap();
                } else {
                    self.comms
                        .send(BackendTask::MapSpatialLidarRelations(
                            self.gui_variables.file_params.paths.clone(),
                            None,
                        ))
                        .unwrap();
                }
            }
        }
    }

    fn on_task_complete(&mut self, task: TaskDone) {
        match task {
            TaskDone::TileSelectedFile => {
                if self.gui_variables.subtile_boundaries.len() <= 9 {
                    self.gui_variables.selected_tile =
                        Some(self.gui_variables.subtile_boundaries.len() / 2);
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
                if self.gui_variables.connected_components.len() == 1 {
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
                self.gui_variables.generating_map_tile = true;
                self.comms
                    .send(BackendTask::InitializeMapTile(
                        self.gui_variables.file_params.paths
                            [self.gui_variables.file_params.selected_file.unwrap_or(0)]
                        .clone(),
                        self.gui_variables.subtile_neighbors
                            [self.gui_variables.selected_tile.unwrap_or(0)]
                        .clone(),
                        self.gui_variables.lidar_stats.clone().unwrap(),
                    ))
                    .unwrap();
            }
            TaskDone::RegenerateMap => {
                self.gui_variables.generating_map_tile = false;
            }
            TaskDone::MakeMap => self.next_state(),
            TaskDone::Reset => (),
            TaskDone::InitializeMapTile => {
                self.gui_variables.generating_map_tile = false;
                self.next_state();
            }
        }
    }

    fn next_state(&mut self) {
        self.state.next();

        match self.state {
            ProcessStage::CheckLidar => {
                self.gui_variables.file_params.selected_file = None;
                self.comms
                    .send(BackendTask::ParseCrs(
                        self.gui_variables.file_params.paths.clone(),
                    ))
                    .unwrap();
            }
            ProcessStage::ConvertingCOPC => {
                self.comms
                    .send(BackendTask::ConvertCopc(
                        self.gui_variables.file_params.paths.clone(),
                        self.gui_variables.file_params.crs_epsg.clone(),
                        self.gui_variables.map_params.output_epsg,
                        self.gui_variables.file_params.selected_file.unwrap_or(0),
                        self.gui_variables.boundaries.clone(),
                        self.gui_variables.polygon_filter.clone(),
                    ))
                    .unwrap();
            }
            ProcessStage::MakeMap => {
                self.comms
                    .send(BackendTask::MakeMap(
                        Box::new(self.gui_variables.map_params.clone()),
                        Box::new(self.gui_variables.file_params.clone()),
                        self.gui_variables.polygon_filter.clone(),
                        self.gui_variables.lidar_stats.clone().unwrap(),
                    ))
                    .unwrap();
            }
            ProcessStage::ExportDone => self.open_modal = OmapModal::WaiverModal,
            ProcessStage::ChooseSquare => self.map_memory.follow_my_position(),
            ProcessStage::AdjustSliders => {
                self.gui_variables.generating_map_tile = true;
                self.comms
                    .send(BackendTask::RegenerateMap(Box::new(
                        self.gui_variables.map_params.clone(),
                    )))
                    .unwrap();
            }
            ProcessStage::ChooseSubTile => {
                self.comms
                    .send(BackendTask::TileSelectedFile(
                        self.gui_variables.file_params.paths
                            [self.gui_variables.file_params.selected_file.unwrap()]
                        .clone(),
                        self.gui_variables.map_params.output_epsg,
                    ))
                    .unwrap();
            }
            _ => (),
        }
    }

    fn prev_state(&mut self) {
        match self.state {
            ProcessStage::AdjustSliders => {
                self.gui_variables.map_tile = None;
                self.gui_variables.selected_tile = None;
                self.comms.send(BackendTask::ClearParams).unwrap()
            }
            ProcessStage::ShowComponents => {
                self.gui_variables.file_params.selected_file = None;
                self.open_modal = OmapModal::MultipleGraphComponents;
            }
            ProcessStage::ChooseSubTile => {
                self.gui_variables.selected_tile = None;
            }
            _ => unimplemented!("Should not have been called on this state"),
        }

        self.state.prev();
    }

    fn update_progress_bar(&mut self, p: ProgressBar) {
        match p {
            ProgressBar::Start => self.gui_variables.log_terminal.start_progress_bar(40),
            ProgressBar::Inc(delta) => self.gui_variables.log_terminal.inc_progress_bar(delta),
            ProgressBar::Finish => self.gui_variables.log_terminal.finish_progress_bar(),
        }
    }

    fn update_crs(&mut self, message: SetCrs) {
        match message {
            SetCrs::SetAllEpsg => {
                let a = self.gui_variables.crs_less_search_strings[0]
                    .parse::<u16>()
                    .unwrap();

                for crs in self.gui_variables.file_params.crs_epsg.iter_mut() {
                    if *crs == u16::MAX {
                        *crs = a;
                    }
                }
            }
            SetCrs::SetEachCrs => {
                let mut drop_list = vec![];
                let mut crs_less_indecies = vec![];
                for (i, crs) in self.gui_variables.file_params.crs_epsg.iter().enumerate() {
                    if crs == &u16::MAX {
                        crs_less_indecies.push(i);
                    }
                }
                for (i, s) in self
                    .gui_variables
                    .crs_less_search_strings
                    .iter()
                    .enumerate()
                {
                    if self.gui_variables.drop_checkboxes[i] {
                        drop_list.push(crs_less_indecies[i]);
                    } else {
                        let crs = s.parse::<u16>().unwrap();
                        self.gui_variables.file_params.crs_epsg[crs_less_indecies[i]] = crs;
                    }
                }
                drop_list.sort_by(|a: &usize, b: &usize| b.cmp(a));
                for i in drop_list {
                    self.gui_variables.file_params.paths.remove(i);
                    self.gui_variables.file_params.crs_epsg.remove(i);
                }
            }
            SetCrs::Default => {
                let mut default_crs = u16::MAX;
                for a in self.gui_variables.file_params.crs_epsg.iter() {
                    if a != &u16::MAX {
                        default_crs = *a;
                        break;
                    }
                }
                assert!(
                    default_crs != u16::MAX,
                    "Default crs button available but should not have been"
                );

                self.gui_variables.file_params.crs_epsg =
                    vec![default_crs; self.gui_variables.file_params.paths.len()];
            }
            SetCrs::DropAll => {
                let mut drop_list = vec![];
                for (i, crs) in self.gui_variables.file_params.crs_epsg.iter().enumerate() {
                    if crs == &u16::MAX {
                        drop_list.push(i);
                    }
                }
                drop_list.sort_by(|a: &usize, b: &usize| b.cmp(a));
                for i in drop_list {
                    self.gui_variables.file_params.paths.remove(i);
                    self.gui_variables.file_params.crs_epsg.remove(i);
                }
            }
            _ => (),
        }

        assert!(
            self.gui_variables.file_params.paths.len()
                == self.gui_variables.file_params.crs_epsg.len()
        );

        if self.gui_variables.file_params.paths.is_empty() {
            self.on_frontend_task(FrontendTask::Error(
                "All Lidar files were dropped.".to_string(),
                true,
            ));
        } else {
            self.gui_variables.crs_less_search_strings.clear();
            self.gui_variables.drop_checkboxes.clear();
            self.on_task_complete(TaskDone::ParseCrs(message));
        }
    }

    fn reset(&mut self) {
        self.home = walkers::pos_from_lon_lat(HOME_LON_LAT.0, HOME_LON_LAT.1);
        self.gui_variables = Default::default();
        self.open_modal = OmapModal::None;
        self.home_zoom = 16.;
        self.map_memory.set_zoom(self.home_zoom).unwrap();
        self.map_memory.follow_my_position();

        match self.comms.send(BackendTask::Reset) {
            Ok(_) => (),
            Err(_) => self.restart_backend(),
        }

        self.state = ProcessStage::Welcome;
    }

    fn restart_backend(&mut self) {
        // start backend thread
        let (frontend_comms, backend_comms) = OmapComms::new();

        // starts the backend on its own thread
        Backend::boot(backend_comms, self.ctx.clone());
        self.comms = frontend_comms;
    }
}
