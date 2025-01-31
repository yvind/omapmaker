use super::{modals::OmapModal, ProcessStage};
use eframe::egui;
use laz2omap::{
    comms::{messages::*, OmapComms},
    drawing::GuiVariables,
    OmapGenerator,
};
use walkers::{sources, HttpOptions, HttpTiles, MapMemory, Position};

use std::sync::mpsc;

pub const HOME_LON_LAT: (f64, f64) = (10.6134, 59.9594);

pub struct OmapMaker {
    // background osm map
    pub http_tiles: HttpTiles,
    pub map_memory: MapMemory,
    pub home: Position,
    pub home_zoom: f64,

    // variables controlling what to show
    pub gui_variables: GuiVariables,

    // modals
    pub open_modal: OmapModal,

    // app state
    pub state: ProcessStage,

    // backend
    comms: OmapComms<BackendTask, FrontEndTask>,
}

impl eframe::App for OmapMaker {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // register all events that has occured
        while let Ok(event) = self.comms.try_recv() {
            self.on_frontend_task(event);
        }

        // render correct side panel
        egui::SidePanel::left("Guide")
            .exact_width(400.)
            .show(ctx, |ui| match self.state {
                ProcessStage::Welcome => self.render_welcome_panel(ui),
                ProcessStage::CheckLidar => self.render_checking_lidar_panel(ui),
                ProcessStage::ShowComponents => self.render_show_components_panel(ui),
                ProcessStage::ConvertingCOPC => self.render_copc_panel(ui),
                ProcessStage::ChooseSquare => self.render_choose_lidar_panel(ui, true),
                ProcessStage::DrawPolygon => self.render_choose_lidar_panel(ui, false),
                ProcessStage::AdjustSliders => self.render_adjust_slider_panel(ui),
                ProcessStage::MakeMap => self.render_generating_map_panel(ui),
                ProcessStage::ExportDone => self.render_done_panel(ui),
            });

        // render correct main panel
        egui::CentralPanel::default()
            .frame(egui::Frame::default().fill(ctx.style().visuals.panel_fill))
            .show(ctx, |ui| match self.state {
                ProcessStage::Welcome
                | ProcessStage::AdjustSliders
                | ProcessStage::ChooseSquare
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
    pub fn new(egui_ctx: egui::Context) -> Self {
        // start backend thread
        let (to_frontend, from_backend) = mpsc::channel();
        let (to_backend, from_frontend) = mpsc::channel();

        let backend_comms = OmapComms::new(to_frontend, from_frontend);
        let frontend_comms = OmapComms::new(to_backend, from_backend);

        // starts the backend on its own thread
        OmapGenerator::boot(backend_comms);

        let http_tiles = HttpTiles::with_options(
            sources::OpenStreetMap,
            HttpOptions {
                cache: Some(".cache".into()),
                ..Default::default()
            },
            egui_ctx,
        );

        Self {
            http_tiles,
            map_memory: Default::default(),
            state: ProcessStage::Welcome,
            comms: frontend_comms,
            open_modal: OmapModal::None,
            home: Position::from_lon_lat(HOME_LON_LAT.0, HOME_LON_LAT.1),
            home_zoom: 16.,
            gui_variables: Default::default(),
        }
    }

    pub fn on_frontend_task(&mut self, event: FrontEndTask) {
        match event {
            FrontEndTask::Log(s) => {
                self.gui_variables.log_string.push('\n');
                self.gui_variables.log_string.push_str(s.as_str());
            }
            FrontEndTask::SetVariable(variable) => self.on_update_variable(variable),
            FrontEndTask::TaskComplete(task) => self.on_task_complete(task),
            FrontEndTask::CrsModal => self.open_modal = OmapModal::ManualSetCRS,
            FrontEndTask::DelegateTask(task) => self.start_task(task),
            FrontEndTask::NextState => self.next_state(),
            FrontEndTask::PrevState => self.prev_state(),
            FrontEndTask::BackendError(s) => {
                self.restart_backend();
                self.reset();
                self.open_modal = OmapModal::ErrorModal(s.clone());
            }
            FrontEndTask::UpdateMap(drawable_omap) => todo!(),
        }
    }
}

// private functions
impl OmapMaker {
    fn on_update_variable(&mut self, variable: Variable) {
        match variable {
            Variable::Boundaries(vec) => self.gui_variables.boundaries = vec,
            Variable::Home(position) => self.home = position,
            Variable::CrsEPSG(vec) => {
                self.gui_variables.crs_epsg = vec;
                self.gui_variables.update_unique_crs();
            }
            Variable::CrsLessString(num) => {
                self.gui_variables.crs_less_search_strings = vec!["".to_string(); num]
            }
            Variable::CrsLessCheckBox(num) => self.gui_variables.drop_checkboxes = vec![false; num],
            Variable::ConnectedComponents(vec) => self.gui_variables.connected_components = vec,
        }
    }

    fn start_task(&mut self, task: Task) {
        match task {
            Task::RegenerateMap => {
                self.comms
                    .send(BackendTask::RegenerateMap(Box::new(
                        self.gui_variables.clone(),
                    )))
                    .unwrap();
            }
            Task::Reset => self.reset(),
            Task::SetCrs(s) => self.update_crs(s),
            Task::ShowComponents => self.state = ProcessStage::ShowComponents,
            Task::Error(s) => {
                self.reset();
                self.open_modal = OmapModal::ErrorModal(s.clone());
            }
            Task::DropComponents => {
                self.gui_variables.drop_small_graph_components();
                self.on_task_complete(TaskDone::DropComponents);
            }
            Task::GetOutputCRS => {
                let majority = self.gui_variables.get_most_popular_crs().unwrap();
                self.open_modal = OmapModal::OutputCRS(majority);
            }
            Task::QueryDropComponents => {
                self.gui_variables.log_string.push_str(
                    format!(
                        "\nThe lidar files are not all connected and form {} parts",
                        self.gui_variables.connected_components.len()
                    )
                    .as_str(),
                );
                self.open_modal = OmapModal::MultipleGraphComponents;
            }
            Task::DoConnectedComponentAnalysis => {
                if self.gui_variables.output_epsg.is_some() {
                    self.comms
                        .send(BackendTask::ConnectedComponentAnalysis(
                            self.gui_variables.paths.clone(),
                            Some(self.gui_variables.crs_epsg.clone()),
                        ))
                        .unwrap();
                } else {
                    self.comms
                        .send(BackendTask::ConnectedComponentAnalysis(
                            self.gui_variables.paths.clone(),
                            None,
                        ))
                        .unwrap();
                }
            }
        }
    }

    fn on_task_complete(&mut self, task: TaskDone) {
        match task {
            TaskDone::ParseCrs(m) => {
                if let SetCrs::Local = m {
                    self.on_frontend_task(FrontEndTask::TaskComplete(TaskDone::OutputCrs));
                } else {
                    self.on_frontend_task(FrontEndTask::DelegateTask(Task::GetOutputCRS));
                }
            }
            TaskDone::ConnectedComponentAnalysis => {
                if self.gui_variables.connected_components.len() == 1 {
                    self.on_frontend_task(FrontEndTask::TaskComplete(TaskDone::DropComponents));
                } else {
                    self.on_frontend_task(FrontEndTask::DelegateTask(Task::QueryDropComponents));
                }
            }
            TaskDone::DropComponents => {
                self.gui_variables
                    .log_string
                    .push_str("\nThe remaining lidar files are all connected.");
                self.next_state();
            }
            TaskDone::OutputCrs => {
                self.on_frontend_task(FrontEndTask::DelegateTask(
                    Task::DoConnectedComponentAnalysis,
                ));
            }
            TaskDone::ConvertCopc => self.next_state(),
            TaskDone::RegenerateMap => (),
            TaskDone::MakeMap => self.next_state(),
            TaskDone::Reset => (),
        }
    }

    fn next_state(&mut self) {
        self.state.next();

        match self.state {
            ProcessStage::CheckLidar => {
                self.gui_variables.selected_file = None;
                self.comms
                    .send(BackendTask::ParseCrs(self.gui_variables.paths.clone()))
                    .unwrap();
            }
            ProcessStage::ConvertingCOPC => {
                self.comms
                    .send(BackendTask::ConvertCopc(self.gui_variables.output_epsg))
                    .unwrap();
            }
            ProcessStage::MakeMap => {
                self.comms
                    .send(BackendTask::MakeMap(Box::new(self.gui_variables.clone())))
                    .unwrap();
            }
            ProcessStage::ExportDone => self.open_modal = OmapModal::WaiverModal,
            ProcessStage::ChooseSquare => self.map_memory.follow_my_position(),
            _ => (),
        }
    }

    fn prev_state(&mut self) {
        match self.state {
            ProcessStage::AdjustSliders => (),
            ProcessStage::ShowComponents => {
                self.gui_variables.selected_file = None;
                self.open_modal = OmapModal::MultipleGraphComponents;
            }
            _ => unimplemented!("Should not have been called on this state"),
        }

        self.state.prev();
    }

    fn reset(&mut self) {
        self.comms.send(BackendTask::Reset).unwrap();
        self.state = ProcessStage::Welcome;
        self.home = Position::from_lon_lat(HOME_LON_LAT.0, HOME_LON_LAT.1);
        self.gui_variables = Default::default();
        self.open_modal = OmapModal::None;
        self.home_zoom = 16.;
        self.map_memory.set_zoom(self.home_zoom).unwrap();
        self.map_memory.follow_my_position();

        // Wait for current backend tasks to finish and then continue
        loop {
            std::thread::sleep(std::time::Duration::from_millis(100));

            match self.comms.try_recv() {
                Ok(e) => {
                    if let FrontEndTask::TaskComplete(TaskDone::Reset) = e {
                        break;
                    }
                }
                Err(_) => panic!("The comms channel has collapsed!"),
            }
        }
    }

    fn update_crs(&mut self, message: SetCrs) {
        match message {
            SetCrs::SetAllEpsg => {
                let a = self.gui_variables.crs_less_search_strings[0]
                    .parse::<u16>()
                    .unwrap();
                self.gui_variables.crs_epsg = vec![a; self.gui_variables.paths.len()];
            }
            SetCrs::SetEachCrs => {
                let mut drop_list = vec![];
                let mut crs_less_indecies = vec![];
                for (i, crs) in self.gui_variables.crs_epsg.iter().enumerate() {
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
                        self.gui_variables.crs_epsg[crs_less_indecies[i]] = crs;
                    }
                }
                drop_list.sort_by(|a, b| b.cmp(a));
                for i in drop_list {
                    self.gui_variables.paths.remove(i);
                    self.gui_variables.crs_epsg.remove(i);
                }
            }
            SetCrs::Default => {
                let mut default_crs = u16::MAX;
                for a in self.gui_variables.crs_epsg.iter() {
                    if a != &u16::MAX {
                        default_crs = *a;
                    }
                }
                assert!(
                    default_crs != u16::MAX,
                    "Defult crs button available but should not have been"
                );

                self.gui_variables.crs_epsg = vec![default_crs; self.gui_variables.paths.len()];
            }
            SetCrs::DropAll => {
                let mut drop_list = vec![];
                for (i, crs) in self.gui_variables.crs_epsg.iter().enumerate() {
                    if crs == &u16::MAX {
                        drop_list.push(i);
                    }
                }
                drop_list.sort_by(|a, b| b.cmp(a));
                for i in drop_list {
                    self.gui_variables.paths.remove(i);
                    self.gui_variables.crs_epsg.remove(i);
                }
            }
            _ => (),
        }

        assert!(self.gui_variables.paths.len() == self.gui_variables.crs_epsg.len());

        if self.gui_variables.paths.is_empty() {
            self.start_task(Task::Error("All Lidar files were dropped.".to_string()));
        } else {
            self.gui_variables.crs_less_search_strings.clear();
            self.gui_variables.drop_checkboxes.clear();
            self.on_task_complete(TaskDone::ParseCrs(message));
        }
    }

    fn restart_backend(&mut self) {
        if self.comms.send(BackendTask::HeartBeat).is_ok() {
            eprintln!("Tried to restart backend, but backend is still alive");
        } else {
            // start backend thread
            let (to_frontend, from_backend) = mpsc::channel();
            let (to_backend, from_frontend) = mpsc::channel();

            let backend_comms = OmapComms::new(to_frontend, from_frontend);
            let frontend_comms = OmapComms::new(to_backend, from_backend);

            // starts the backend on its own thread
            OmapGenerator::boot(backend_comms);
            self.comms = frontend_comms;
        }
    }
}
