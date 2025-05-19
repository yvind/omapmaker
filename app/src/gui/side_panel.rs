use laz2omap::{comms::messages::*, drawable::DrawOrder, parameters::ContourAlgo};

use super::modals::OmapModal;
use crate::OmapMaker;
use eframe::egui;
use egui_double_slider::DoubleSlider;

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
                                && !self.gui_variables.file_params.paths.contains(&file)
                            {
                                self.gui_variables.file_params.paths.push(file);
                            }
                        }
                    }
                }
            }
            if ui.button("Clear Lidar").clicked() {
                self.gui_variables.file_params.paths.clear();
                self.gui_variables.file_params.selected_file = None;
            }
            if ui.button("Remove selected").clicked() {
                if let Some(i) = self.gui_variables.file_params.selected_file {
                    self.gui_variables.file_params.paths.remove(i);
                    if self.gui_variables.file_params.paths.is_empty() {
                        self.gui_variables.file_params.selected_file = None;
                    } else if self.gui_variables.file_params.paths.len() <= i {
                        self.gui_variables.file_params.selected_file = Some(i - 1);
                    }
                }
            }
        });

        ui.label("Selected files:");

        egui::ScrollArea::both()
            .max_height(ui.available_height() - 300.)
            .auto_shrink(false)
            .max_width(f32::INFINITY)
            .show(ui, |ui| {
                for (index, p) in self.gui_variables.file_params.paths.iter().enumerate() {
                    if ui
                        .selectable_label(
                            self.gui_variables.file_params.selected_file == Some(index),
                            p.file_name().unwrap().to_str().unwrap(),
                        )
                        .clicked()
                    {
                        if Some(index) == self.gui_variables.file_params.selected_file {
                            self.gui_variables.file_params.selected_file = None;
                        } else {
                            self.gui_variables.file_params.selected_file = Some(index);
                        }
                    }
                }
            });
        ui.label(format!(
            "Number of files: {}",
            self.gui_variables.file_params.paths.len()
        ));

        ui.add_space(20.);

        if ui.button("Choose save location and name").clicked() {
            if let Some(path) = rfd::FileDialog::new()
                .add_filter("OpenOrienteering Mapper (*.omap)", &["omap"])
                .save_file()
            {
                self.gui_variables.file_params.save_location = path;
            };
        }

        let text = self
            .gui_variables
            .file_params
            .save_location
            .to_str()
            .unwrap();
        if text.is_empty() {
            ui.label("Choose where to save the resulting omap-file.");
        } else {
            ui.label(text);
        }

        if ui
            .add_enabled(
                !(self.gui_variables.file_params.paths.is_empty()
                    || self
                        .gui_variables
                        .file_params
                        .save_location
                        .as_os_str()
                        .is_empty()),
                egui::Button::new("Next step"),
            )
            .clicked()
        {
            self.on_frontend_task(FrontendTask::NextState);
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
            .auto_shrink(false)
            .max_width(f32::INFINITY)
            .max_height(ui.available_height() / 2.)
            .show(ui, |ui| {
                for (index, p) in self.gui_variables.file_params.paths.iter().enumerate() {
                    if ui
                        .selectable_label(
                            self.gui_variables.file_params.selected_file == Some(index),
                            p.file_name().unwrap().to_str().unwrap(),
                        )
                        .clicked()
                    {
                        if Some(index) == self.gui_variables.file_params.selected_file {
                            self.gui_variables.file_params.selected_file = None;
                        } else {
                            self.gui_variables.file_params.selected_file = Some(index);
                            let center = walkers::pos_from_lat_lon(
                                (self.gui_variables.boundaries[index][0].y
                                    + self.gui_variables.boundaries[index][1].y)
                                    / 2.,
                                (self.gui_variables.boundaries[index][0].x
                                    + self.gui_variables.boundaries[index][1].x)
                                    / 2.,
                            );
                            self.map_memory.center_at(center);
                        }
                    }
                }
            });

        if ui.button("Go back").clicked() {
            self.on_frontend_task(FrontendTask::PrevState);
        }
    }

    pub fn render_copc_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("Preparing files for map generation");

        ui.add_space(20.);
        ui.label(
            "This includes writing all relevant .las or .laz files to .copc.laz \
        and transforming any relevant lidar file not given in the output CRS to that CRS.\n\
        This might take some time",
        );

        ui.add_space(10.);
        ui.label(
            "A file is deemed relevant if it overlaps with the map area or is the selected file.",
        );

        ui.add_space(10.);
        ui.label(".copc.laz is a .laz file (compressed .las file) where the points internally are structered in an octree. \
        This makes for logarithmic-time spatial queries and the possibility to efficiently add resolution restrictions, at a trade off for slightly larger files. \
        This step is performed on all relevant files not alreday in the .copc.laz format and is non-destructive. \
        Any modern lidar-reader can read points from .copc.laz files, but specialized readers are needed to utilize the octree structure.");

        ui.add_space(20.);
        ui.label("Any relevant file not given in the previously chosen CRS is transformed to the chosen CRS during writing. \
        If the file is transformed \"_EPSG_*\" is appended to the filename. \
        Where the star is replaced with the code of the CRS.");

        ui.label("The resulting files are stored next to their parent.");

        ui.add_space(20.);
        ui.label("Finally the chosen area for adjusting parameters is prepared.");
    }

    pub fn render_choose_lidar_panel(&mut self, ui: &mut egui::Ui, enabled: bool) {
        ui.heading("Select test file");
        ui.add_space(20.);
        ui.label(
            "Select a Lidar file either on the map or in the list. \
            This Lidar file will be used for adjusting parameter values.",
        );
        egui::ScrollArea::both()
            .auto_shrink(false)
            .max_width(f32::INFINITY)
            .max_height(ui.available_height() / 2.)
            .show(ui, |ui| {
                for (index, p) in self.gui_variables.file_params.paths.iter().enumerate() {
                    if ui
                        .selectable_label(
                            self.gui_variables.file_params.selected_file == Some(index),
                            p.file_name().unwrap().to_str().unwrap(),
                        )
                        .clicked()
                    {
                        if Some(index) == self.gui_variables.file_params.selected_file {
                            self.gui_variables.file_params.selected_file = None;
                        } else {
                            self.gui_variables.file_params.selected_file = Some(index);
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
                    self.gui_variables.file_params.selected_file.is_some() && enabled,
                    egui::Button::new("Next step"),
                )
                .clicked()
            {
                self.on_frontend_task(FrontendTask::NextState);
            }
        });
    }

    pub fn render_adjust_slider_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("Adjust the parameters for the map");
        ui.add_space(20.);
        ui.label("Adjust each value untill you're happy and press the \"next step\" button below.");

        egui::ScrollArea::both()
            .auto_shrink(false)
            .max_height(ui.available_height() / 1.2)
            .max_width(f32::INFINITY)
            .show(ui, |ui| {
                ui.label(egui::RichText::new("Map Scale").strong());
                ui.horizontal(|ui| {
                    if ui
                        .selectable_label(
                            self.gui_variables.map_params.scale == omap::Scale::S15_000,
                            "1:15 000",
                        )
                        .clicked()
                    {
                        self.gui_variables.map_params.scale = omap::Scale::S15_000;
                    };
                    ui.separator();
                    if ui
                        .selectable_label(
                            self.gui_variables.map_params.scale == omap::Scale::S10_000,
                            "1:10 000",
                        )
                        .clicked()
                    {
                        self.gui_variables.map_params.scale = omap::Scale::S10_000;
                    };
                });
                ui.add_space(20.);
                ui.label(egui::RichText::new("Contour algorithm parameters:").strong());
                ui.horizontal(|ui| {
                    ui.label("Contour algorithm:");
                    egui::ComboBox::from_id_salt("Contour algo")
                        .selected_text(format!(
                            "{}",
                            self.gui_variables.map_params.contour_algorithm
                        ))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(
                                &mut self.gui_variables.map_params.contour_algorithm,
                                ContourAlgo::AI,
                                "AI contours (slowest)",
                            );
                            ui.selectable_value(
                                &mut self.gui_variables.map_params.contour_algorithm,
                                ContourAlgo::NaiveIterations,
                                "Naive interpolation error correction (slow)",
                            );
                            ui.selectable_value(
                                &mut self.gui_variables.map_params.contour_algorithm,
                                ContourAlgo::NormalFieldSmoothing,
                                "Normal field smoothing (fast)",
                            );
                            ui.selectable_value(
                                &mut self.gui_variables.map_params.contour_algorithm,
                                ContourAlgo::Raw,
                                "Raw contours (fastest)",
                            );
                        });
                });

                if self.gui_variables.map_params.contour_algorithm != ContourAlgo::Raw {
                    if self.gui_variables.map_params.contour_algorithm
                        == ContourAlgo::NormalFieldSmoothing
                    {
                        ui.label("Number of smoothing iterations");
                    } else {
                        ui.label("Number of error correction iterations");
                    }
                    ui.add(
                        egui::Slider::new(
                            &mut self.gui_variables.map_params.contour_algo_steps,
                            1..=20,
                        )
                        .show_value(true),
                    );
                }
                if self.gui_variables.map_params.contour_algorithm == ContourAlgo::AI {
                    ui.label(
                        "Contour Algo Regularization.\nBigger number punishes squiggly lines more.",
                    );
                    ui.add(
                        egui::Slider::new(
                            &mut self.gui_variables.map_params.contour_algo_lambda,
                            0.0..=1.,
                        )
                        .show_value(true),
                    );
                }
                ui.add_space(10.);

                ui.label(egui::RichText::new("Contour parameters:").strong());
                ui.horizontal(|ui| {
                    ui.label("Contour interval: ");
                    ui.add(
                        egui::widgets::DragValue::new(
                            &mut self.gui_variables.map_params.contour_interval,
                        )
                        .fixed_decimals(1)
                        .range(1.0..=20.),
                    );
                });
                ui.checkbox(
                    &mut self.gui_variables.map_params.form_lines,
                    "Add formlines to the map.",
                );
                ui.add_enabled_ui(self.gui_variables.map_params.form_lines, |ui| {
                    ui.label("Formline pruning parameter. \nBigger number gives more formlines.");
                    ui.add(
                        egui::Slider::new(
                            &mut self.gui_variables.map_params.form_line_prune,
                            0.0..=1.,
                        )
                        .show_value(true),
                    );
                });

                ui.checkbox(
                    &mut self.gui_variables.map_params.basemap_contour,
                    "Add basemap contours to the map.",
                );
                ui.add_enabled_ui(self.gui_variables.map_params.basemap_contour, |ui| {
                    ui.label("Basemap interval: ");
                    ui.add(
                        egui::widgets::DragValue::new(
                            &mut self.gui_variables.map_params.basemap_interval,
                        )
                        .fixed_decimals(2)
                        .range(0.1..=self.gui_variables.map_params.contour_interval),
                    );
                });

                ui.add_space(20.);

                ui.label(egui::RichText::new("Vegetation parameters").strong());
                ui.label("Yellow threshold");
                ui.add(
                    egui::Slider::new(&mut self.gui_variables.map_params.yellow, 0.0..=1.0)
                        .text("Yellow 403")
                        .show_value(true),
                );
                ui.add_space(20.);
                ui.label("Green thresholds");
                ui.add(
                    egui::Slider::new(&mut self.gui_variables.map_params.green.0, 0.0..=1.0)
                        .text("Green 406")
                        .show_value(true),
                );
                ui.add(
                    egui::Slider::new(&mut self.gui_variables.map_params.green.1, 0.0..=1.0)
                        .text("Green 408")
                        .show_value(true),
                );
                ui.add(
                    egui::Slider::new(&mut self.gui_variables.map_params.green.2, 0.0..=1.0)
                        .text("Green 410")
                        .show_value(true),
                );

                ui.add_space(20.);

                ui.label(egui::RichText::new("Cliff parameters").strong());
                ui.add(
                    egui::Slider::new(&mut self.gui_variables.map_params.cliff, 0.2..=2.0)
                        .text("Cliff")
                        .show_value(true),
                );

                // clamp the greens to the correct order
                clamp_greens(&mut self.gui_variables.map_params.green);

                ui.add_space(20.);
                ui.label(egui::RichText::new("Geometry simplification parameters").strong());
                ui.checkbox(
                    &mut self.gui_variables.map_params.bezier_bool,
                    "Output map geometries in bezier curves.",
                );

                ui.add_enabled_ui(self.gui_variables.map_params.bezier_bool, |ui| {
                    ui.label("Permitted error in Bezier simplification:");
                    ui.add(
                        egui::Slider::new(
                            &mut self.gui_variables.map_params.bezier_error,
                            0.01..=1.0,
                        )
                        .fixed_decimals(2)
                        .show_value(true),
                    );
                });

                ui.add_space(20.);

                ui.label(egui::RichText::new("Lidar Intensity filters").strong());
                for (i, intensity_filter) in self
                    .gui_variables
                    .map_params
                    .intensity_filters
                    .iter_mut()
                    .enumerate()
                {
                    ui.horizontal(|ui| {
                        ui.add(egui::DragValue::new(&mut intensity_filter.low).range(0.0..=1.0));
                        ui.add(
                            DoubleSlider::new(
                                &mut intensity_filter.low,
                                &mut intensity_filter.high,
                                0.0..=1.0,
                            )
                            .separation_distance(0.01),
                        );
                        ui.add(egui::DragValue::new(&mut intensity_filter.high).range(0.0..=1.0));
                        egui::ComboBox::from_id_salt(format!("Intensity filter {}", i + 1))
                            .selected_text(format!("{:?}", intensity_filter.symbol))
                            .show_ui(ui, |ui| {
                                for area_symbol in omap::symbols::AreaSymbol::draw_order() {
                                    ui.selectable_value(
                                        &mut intensity_filter.symbol,
                                        area_symbol,
                                        format!("{:?}", area_symbol),
                                    );
                                }
                            });
                    });
                }
                ui.horizontal(|ui| {
                    if ui.button("Add filter").clicked() {
                        self.gui_variables
                            .map_params
                            .intensity_filters
                            .push(Default::default());
                    }
                    if ui
                        .add_enabled(
                            !self.gui_variables.map_params.intensity_filters.is_empty(),
                            egui::Button::new("Remove filter"),
                        )
                        .clicked()
                    {
                        self.gui_variables.map_params.intensity_filters.pop();
                    }
                });
            });

        ui.add_space(20.);

        let button_txt = if self.gui_variables.generating_map_tile {
            "Generating map..."
        } else {
            "Re-generate map"
        };
        if ui
            .add_enabled(
                !self.gui_variables.generating_map_tile,
                egui::Button::new(button_txt),
            )
            .clicked()
        {
            self.on_frontend_task(FrontendTask::DelegateTask(Task::RegenerateMap));
        }

        ui.add_space(20.);

        ui.add_enabled_ui(!self.gui_variables.generating_map_tile, |ui| {
            ui.horizontal(|ui| {
                if ui.button("Prev step").clicked() {
                    self.on_frontend_task(FrontendTask::PrevState);
                }
                if ui.button("Next step").clicked() {
                    self.open_modal = OmapModal::ConfirmMakeMap;
                }
            });
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
            self.gui_variables.file_params.save_location
        ));
        ui.label("The map can be opened in OpenOrienteering Mapper for editing.");

        ui.add_space(20.);
        ui.label("If you like this application. Please star the project on Github:)");
        ui.hyperlink_to("OmapMaker on Github", "https://github.com/oyhj1801/")
            .on_hover_text("https://github.com/oyhj1801/");

        ui.add_space(20.);
        if ui.button("Start a new map").clicked() {
            self.on_frontend_task(FrontendTask::DelegateTask(Task::Reset));
        }
    }

    pub fn render_choose_tile_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("Select a sub-tile");
        ui.add_space(20.);
        ui.label("The chosen Lidar file is too large to use the entire file for adjusting parameters.\
        Select a sub-tile to use for parameter adjusting. Both the selected tile and all its neighbours will be used.\
        Select by clicking on the tile in the map.");

        ui.add_space(20.);

        ui.horizontal(|ui| {
            if ui.button("Prev step").clicked() {
                self.on_frontend_task(FrontendTask::PrevState);
            }
            if ui
                .add_enabled(
                    self.gui_variables.selected_tile.is_some(),
                    egui::Button::new("Next step"),
                )
                .clicked()
            {
                self.on_frontend_task(FrontendTask::NextState);
            }
        });
    }
}

fn clamp_greens(greens: &mut (f64, f64, f64)) {
    greens.0 = greens.0.clamp(0., greens.1);
    greens.2 = greens.2.clamp(greens.1, 1.);
    greens.1 = greens.1.clamp(greens.0, greens.2);
}
