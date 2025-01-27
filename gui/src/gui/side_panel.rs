use laz2omap::comms::messages::*;

use super::{modals::OmapModal, OmapMaker};
use eframe::egui;

impl OmapMaker {
    pub fn render_welcome_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("Welcome to OmapMaker");
        ui.add_space(20.);
        ui.label(
            "Let's start your new map. Click the \"Add Lidar\" \
        button below and choose your files.",
        );
        ui.add_space(10.);
        ui.label(
            "Only .las and .laz files are accepted.\n\
        .copc.laz files are strongly recommended.\n\
        If normal las or laz files are provided they will be written to copc.laz.",
        );

        ui.horizontal(|ui| {
            if ui.button("Add Lidar").clicked() {
                let files = rfd::FileDialog::new()
                    .add_filter("Lidar Files (*.las, *.laz)", &["las", "laz"])
                    .pick_files();
                if let Some(f) = files {
                    for file in f {
                        if let Some(ext) = file.extension() {
                            if (ext.to_ascii_lowercase().to_string_lossy() == "laz"
                                || ext.to_ascii_lowercase().to_string_lossy() == "las")
                                && !self.gui_variables.paths.contains(&file)
                            {
                                self.gui_variables.paths.push(file);
                            }
                        }
                    }
                }
            }
            if ui.button("Clear Lidar").clicked() {
                self.gui_variables.paths.clear();
                self.gui_variables.selected_file = None;
            }
            if ui.button("Remove selected").clicked() {
                if let Some(i) = self.gui_variables.selected_file {
                    self.gui_variables.paths.remove(i);
                    if self.gui_variables.paths.is_empty() {
                        self.gui_variables.selected_file = None;
                    } else if self.gui_variables.paths.len() <= i {
                        self.gui_variables.selected_file = Some(i - 1);
                    }
                }
            }
        });

        ui.label("Selected files:");

        ui.style_mut().spacing.scroll = egui::style::ScrollStyle::thin();

        egui::ScrollArea::both()
            .max_height(ui.available_height() / 3.)
            .show(ui, |ui| {
                for (index, p) in self.gui_variables.paths.iter().enumerate() {
                    if ui
                        .selectable_label(
                            self.gui_variables.selected_file == Some(index),
                            p.file_name().unwrap().to_str().unwrap(),
                        )
                        .clicked()
                    {
                        if Some(index) == self.gui_variables.selected_file {
                            self.gui_variables.selected_file = None;
                        } else {
                            self.gui_variables.selected_file = Some(index);
                        }
                    }
                }
            });
        ui.label(format!(
            "Number of files: {}",
            self.gui_variables.paths.len()
        ));

        ui.add_space(20.);

        ui.label("Choose where to save the resulting omap-file.");
        if ui.button("Choose save location and name").clicked() {
            self.gui_variables.save_location = rfd::FileDialog::new()
                .add_filter("OpenOrienteering Mapper (*.omap)", &["omap"])
                .save_file();
        }
        let text = if self.gui_variables.save_location.is_some() {
            self.gui_variables
                .save_location
                .as_ref()
                .unwrap()
                .to_str()
                .unwrap()
        } else {
            ""
        };
        ui.label(text);

        egui::CollapsingHeader::new("Advanced settings").show(ui, |ui| {
            ui.checkbox(
                &mut self.gui_variables.save_tiffs,
                "Save tiff-images generated during lidar-processing.",
            );
            ui.add_enabled_ui(self.gui_variables.save_tiffs, |ui| {
                ui.label("Choose where to save the resulting tiff-files.");
                if ui.button("Choose which folder to save tiffs to").clicked() {
                    self.gui_variables.tiff_location = rfd::FileDialog::new().pick_folder();
                }
                let text = if self.gui_variables.tiff_location.is_some() {
                    self.gui_variables
                        .tiff_location
                        .as_ref()
                        .unwrap()
                        .to_str()
                        .unwrap()
                } else {
                    ""
                };
                ui.label(text);
            });
        });

        if ui
            .add_enabled(
                !(self.gui_variables.paths.is_empty()
                    || self.gui_variables.save_location.is_none()),
                egui::Button::new("Next step"),
            )
            .clicked()
        {
            self.on_frontend_task(FrontEndTask::NextState);
        }

        egui::Window::new("text size")
            .anchor(egui::Align2::LEFT_BOTTOM, [10., -10.])
            .collapsible(false)
            .resizable(false)
            .title_bar(false)
            .interactable(false)
            .frame(egui::Frame::default().fill(egui::Color32::TRANSPARENT))
            .show(ui.ctx(), |ui| {
                ui.heading("press 'ctrl +' to enlarge the UI.");
                ui.heading("press 'ctrl -' to shrink the UI.");
            });
    }

    pub fn render_checking_lidar_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("Checking validity of Lidar files");

        ui.add_space(20.);
        ui.label(
            "Checking readabilty of the files, coordinate refrence systems \
            and doing connected component analysis on the lidar-neighbour-graph.",
        );

        ui.add_space(10.);
        ui.label(
            "First each file's CRS is read. If one or more file lacks \
        a CRS some options for CRS assigment will be presented. \
        Hover over the different buttons to see what they do.",
        );

        ui.add_space(10.);
        ui.label(
            "From all the files a graph is constructed where each lidar file \
        is a node and bordering files are connected by edges. \
        This assumes that the files belong to a grid-like structure. \
        If the graph has more than one connected component the user gets \
        to choose wether to keep the biggest connected component (by node count) or start over.",
        );

        ui.add_space(10.);
        ui.label("Then the the user will be prompted to choose a CRS for the final output. \
        Every file not in the chosen CRS will at a later stage be transformed to that CRS. \
        It is recommended to choose the CRS which results in the fewest transformed files. \
        Though choosing a different CRS makes sense in some cases. Such as when the \
        lidar files are given in a CRS with imperial units, but is generally discouraged as it is time-consuming.");
    }

    pub fn render_show_components_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("Connected components");
        ui.label(
            "All given lidar files are displayed on the map to the right. \
        The different connected components of the lidar neighbour graph is colored differently.",
        );
        ui.add_space(10.);
        ui.label("Clicking a file in the list will center the map at that file's location.");
        egui::ScrollArea::both()
            .max_height(ui.available_height() / 2.)
            .show(ui, |ui| {
                for (index, p) in self.gui_variables.paths.iter().enumerate() {
                    if ui
                        .selectable_label(
                            self.gui_variables.selected_file == Some(index),
                            p.file_name().unwrap().to_str().unwrap(),
                        )
                        .clicked()
                    {
                        if Some(index) == self.gui_variables.selected_file {
                            self.gui_variables.selected_file = None;
                        } else {
                            self.gui_variables.selected_file = Some(index);
                            let center = walkers::Position::from_lat_lon(
                                (self.gui_variables.boundaries[index][0].lat()
                                    + self.gui_variables.boundaries[index][1].lat())
                                    / 2.,
                                (self.gui_variables.boundaries[index][0].lon()
                                    + self.gui_variables.boundaries[index][1].lon())
                                    / 2.,
                            );
                            self.map_memory.center_at(center);
                        }
                    }
                }
            });

        if ui.button("Go back").clicked() {
            self.on_frontend_task(FrontEndTask::PrevState);
        }
    }

    pub fn render_copc_panel(&mut self, ui: &mut egui::Ui) {
        if self.gui_variables.output_epsg.is_some() {
            ui.heading("Writing files to COPC and transforming CRS");
        } else {
            ui.heading("Writing files to COPC");
        }

        ui.add_space(20.);
        ui.label("This might take some time");

        ui.add_space(10.);
        ui.label(".copc.laz is a .laz file (compressed .las file) where the points internally are structered in an octree.\
        This makes for logarithmic-time spatial queries and the possibility to efficiently add resolution restrictions, at a trade off for slightly larger files. \
        This step is performed on all files not alreday in the .copc.laz format and is non-destructive. \
        Any modern lidar-reader can read points from .copc.laz files, but specialized readers are needed to utilize the octree structure.");

        if self.gui_variables.output_epsg.is_some() {
            ui.add_space(20.);
            ui.label("Any file not given in the previously chosen CRS is transformed to the chosen CRS during writing. \
            If the file is transformed \"_EPSG_*\" is appended to the filename. \
            Where the star is replaced with the code of the CRS.");
        }
        ui.label("The resulting .copc.laz files are stored next to their parent.");
    }

    pub fn render_choose_lidar_panel(&mut self, ui: &mut egui::Ui, enabled: bool) {
        ui.heading("Select test file");
        ui.add_space(20.);
        ui.label(
            "Select a Lidar file either on the map or in the list. \
            This Lidar file will be used for adjusting parameter values.",
        );
        egui::ScrollArea::both()
            .max_height(ui.available_height() / 2.)
            .show(ui, |ui| {
                for (index, p) in self.gui_variables.paths.iter().enumerate() {
                    if ui
                        .selectable_label(
                            self.gui_variables.selected_file == Some(index),
                            p.file_name().unwrap().to_str().unwrap(),
                        )
                        .clicked()
                    {
                        if Some(index) == self.gui_variables.selected_file {
                            self.gui_variables.selected_file = None;
                        } else {
                            self.gui_variables.selected_file = Some(index);
                        }
                    }
                }
            });

        ui.add_space(20.);
        ui.label(
            "If you only need to map part of the area that your Lidar files cover, \
                click the \"Draw Polygon\" button and click around the area you want to keep. \
                Double click to close the polygon.\n\
                If no polygon is drawn the whole area that the Lidar files cover will be mapped.",
        );

        ui.add_space(20.);
        ui.horizontal(|ui| {
            if ui.button("Start over").clicked() {
                self.open_modal = OmapModal::ConfirmStartOver;
            }
            if ui
                .add_enabled(
                    self.gui_variables.selected_file.is_some() && enabled,
                    egui::Button::new("Next step"),
                )
                .clicked()
            {
                self.on_frontend_task(FrontEndTask::NextState);
            }
        });
    }

    pub fn render_adjust_slider_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("Adjust the parameters for the map");
        ui.add_space(20.);
        ui.label("Adjust each value untill you're happy and press the \"next step\" button below.");

        ui.add_space(20.);
        ui.label(egui::RichText::new("Contour parameters").strong());
        ui.checkbox(
            &mut self.gui_variables.formlines,
            "Add formlines to the map.",
        );
        ui.horizontal(|ui| {
            ui.label("Contour interval: ");
            ui.add(
                egui::widgets::DragValue::new(&mut self.gui_variables.contour_interval)
                    .fixed_decimals(1)
                    .range(1.0..=20.),
            );
        });
        ui.checkbox(
            &mut self.gui_variables.basemap_contour,
            "Add basemap contours to the map.",
        );
        ui.add_enabled_ui(self.gui_variables.basemap_contour, |ui| {
            ui.label("Basemap interval: ");
            ui.add(
                egui::widgets::DragValue::new(&mut self.gui_variables.basemap_interval)
                    .fixed_decimals(1)
                    .range(0.1..=self.gui_variables.contour_interval),
            );
        });

        ui.add_space(20.);

        ui.label(egui::RichText::new("Vegetation parameters").strong());
        ui.label("Yellow threshold");
        ui.add(
            egui::Slider::new(&mut self.gui_variables.yellow, 0.0..=1.0)
                .text("Yellow 403")
                .show_value(true),
        );
        ui.add_space(20.);
        ui.label("Green thresholds");
        ui.add(
            egui::Slider::new(&mut self.gui_variables.green.0, 0.0..=1.0)
                .text("Green 406")
                .show_value(true),
        );
        ui.add(
            egui::Slider::new(&mut self.gui_variables.green.1, 0.0..=1.0)
                .text("Green 408")
                .show_value(true),
        );
        ui.add(
            egui::Slider::new(&mut self.gui_variables.green.2, 0.0..=1.0)
                .text("Green 410")
                .show_value(true),
        );

        // clamp the greens to the correct order
        self.gui_variables.green.0 = self
            .gui_variables
            .green
            .0
            .min(self.gui_variables.green.1)
            .min(self.gui_variables.green.2);
        self.gui_variables.green.1 = self
            .gui_variables
            .green
            .0
            .max(self.gui_variables.green.1)
            .min(self.gui_variables.green.2);
        self.gui_variables.green.2 = self
            .gui_variables
            .green
            .0
            .max(self.gui_variables.green.1)
            .max(self.gui_variables.green.2);

        ui.add_space(20.);
        ui.label(egui::RichText::new("Geometry simplification parameters").strong());
        ui.checkbox(
            &mut self.gui_variables.bezier_bool,
            "Output map geometries in bezier curves.",
        );

        if self.gui_variables.bezier_bool {
            ui.label("Bezier simplification error\n(smaller value gives less simplification, but larger files):");
            ui.add(
                egui::Slider::new(&mut self.gui_variables.bezier_error, 0.1..=5.0)
                    .fixed_decimals(1)
                    .show_value(true),
            );
        } else {
            ui.label("Polyline simplification distance\n(smaller value gives less simplification, but larger files):");
            ui.add(
                egui::Slider::new(&mut self.gui_variables.simplification_distance, 0.1..=2.0)
                    .fixed_decimals(1)
                    .show_value(true),
            );
        }
        ui.add_space(20.);

        egui::CollapsingHeader::new("Contour Algo Debug Params").show(ui, |ui| {
            ui.label("Number of steps to perform in Contour Algo");
            ui.add(
                egui::Slider::new(&mut self.gui_variables.contour_algo_steps, 0..=20)
                    .show_value(true),
            );
            ui.label("Contour Algo Regularization.\nBigger number punishes squiggly lines more.");
            ui.add(
                egui::Slider::new(&mut self.gui_variables.contour_algo_lambda, 0.0..=10.)
                    .show_value(true),
            );
        });

        ui.add_space(20.);
        if ui.button("Re-generate map").clicked() {
            self.on_frontend_task(FrontEndTask::DelegateTask(Task::RegenerateMap));
        }

        ui.add_space(20.);

        ui.horizontal(|ui| {
            if ui.button("Prev step").clicked() {
                self.on_frontend_task(FrontEndTask::PrevState);
            }
            if ui.button("Next step").clicked() {
                self.open_modal = OmapModal::ConfirmMakeMap;
            }
        });
    }

    pub fn render_generating_map_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("Generating the rest of the map.");
        ui.add_space(20.);
        ui.label("This might take some time.");
    }

    pub fn render_done_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("All done!");
        ui.label("The map is saved at: ");
        ui.label(format!(
            "{:?}.",
            self.gui_variables.save_location.as_ref().unwrap()
        ));
        ui.label("The map can be opened in OpenOrienteering Mapper for editing.");

        ui.add_space(20.);
        ui.label("If you like this application. Please star the project on Github:)");
        ui.hyperlink_to("OmapMaker on Github", "https://github.com/oyhj1801/")
            .on_hover_text("https://github.com/oyhj1801/");

        ui.add_space(20.);
        if ui.button("Start a new map").clicked() {
            self.on_frontend_task(FrontEndTask::DelegateTask(Task::Reset));
        }
    }
}
