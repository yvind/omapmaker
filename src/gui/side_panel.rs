use crate::{
    comms::messages::*,
    drawable::DrawOrder,
    map_gen::egui_map::AreaSymbol,
    parameters::{BezierParameters, BufferDirection, BufferRule, ContourAlgo, Scale},
};

use super::{ProcessStage, modals::OmapModal};
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
                        if let Some(ext) = file.extension()
                            && (ext.to_ascii_lowercase().to_string_lossy() == "laz"
                                || ext.to_ascii_lowercase().to_string_lossy() == "las")
                            && !self.gui_variables.project.paths.contains(&file)
                        {
                            self.gui_variables.project.paths.push(file);
                        }
                    }
                }
            }
            if ui.button("Clear Lidar").clicked() {
                self.gui_variables.project.paths.clear();
                self.gui_variables.project.selected_file = None;
            }
            if ui.button("Remove selected").clicked()
                && let Some(i) = self.gui_variables.project.selected_file
            {
                self.gui_variables.project.paths.remove(i);
                if self.gui_variables.project.paths.is_empty() {
                    self.gui_variables.project.selected_file = None;
                } else if self.gui_variables.project.paths.len() <= i {
                    self.gui_variables.project.selected_file = Some(i - 1);
                }
            }
        });

        ui.label("Selected files:");

        egui::ScrollArea::both()
            .max_height(ui.available_height() - 400.)
            .auto_shrink(false)
            .max_width(f32::INFINITY)
            .show(ui, |ui| {
                for (index, p) in self.gui_variables.project.paths.iter().enumerate() {
                    if ui
                        .selectable_label(
                            self.gui_variables.project.selected_file == Some(index),
                            p.file_name()
                                .and_then(|name| name.to_str())
                                .unwrap_or_else(|| p.to_str().unwrap_or("<invalid path>")),
                        )
                        .clicked()
                    {
                        if Some(index) == self.gui_variables.project.selected_file {
                            self.gui_variables.project.selected_file = None;
                        } else {
                            self.gui_variables.project.selected_file = Some(index);
                        }
                    }
                }
            });
        ui.label(format!(
            "Number of files: {}",
            self.gui_variables.project.paths.len()
        ));

        ui.add_space(10.);
        egui::CollapsingHeader::new("Advanced options")
            .id_salt("welcome_advanced_options")
            .show(ui, |ui| {
                let max_threads = std::thread::available_parallelism()
                    .map(|threads| threads.get())
                    .unwrap_or(8)
                    .max(self.gui_variables.project.worker_threads)
                    .max(1);

                ui.add(
                    egui::Slider::new(
                        &mut self.gui_variables.project.worker_threads,
                        1..=max_threads,
                    )
                    .text("Backend threads"),
                )
                .on_hover_text("Number of worker threads used by the backend Rayon thread pool.");

                ui.checkbox(
                    &mut self.gui_variables.project.write_single_copc,
                    "Write all relevant lidar files to one COPC file",
                )
                .on_hover_text(
                    "The final map generation will read one merged .copc.laz file instead of one COPC file per relevant input tile.",
                );

                ui.checkbox(
                    &mut self.gui_variables.project.save_rasters,
                    "Save rasters",
                )
                .on_hover_text(
                    "Write selected generated rasters as merged GeoTIFF files next to the .omap output.",
                );

                if !self.gui_variables.project.save_rasters {
                    self.gui_variables.project.save_slope_raster = false;
                    self.gui_variables.project.save_hillshade_raster = false;
                    self.gui_variables.project.save_last_return_raster = false;
                    self.gui_variables.project.save_canopy_height_raster = false;
                    self.gui_variables.project.save_surface_objects_raster = false;
                    self.gui_variables.project.save_ndvd_raster = false;
                }

                ui.indent("indented raster checkboxes", |ui| {
                    ui.add_enabled(
                        self.gui_variables.project.save_rasters,
                        egui::Checkbox::new(
                            &mut self.gui_variables.project.save_slope_raster,
                            "Save slope raster",
                        ),
                    );

                    ui.add_enabled(
                        self.gui_variables.project.save_rasters,
                        egui::Checkbox::new(
                            &mut self.gui_variables.project.save_hillshade_raster,
                            "Save hillshade raster",
                        ),
                    );

                    ui.add_enabled(
                        self.gui_variables.project.save_rasters,
                        egui::Checkbox::new(
                            &mut self.gui_variables.project.save_last_return_raster,
                            "Save last-return raster",
                        ),
                    );

                    ui.add_enabled(
                        self.gui_variables.project.save_rasters,
                        egui::Checkbox::new(
                            &mut self.gui_variables.project.save_canopy_height_raster,
                            "Save canopy height raster",
                        ),
                    );

                    ui.add_enabled(
                        self.gui_variables.project.save_rasters,
                        egui::Checkbox::new(
                            &mut self.gui_variables.project.save_surface_objects_raster,
                            "Save surface objects raster",
                        ),
                    );

                    ui.add_enabled(
                        self.gui_variables.project.save_rasters,
                        egui::Checkbox::new(
                            &mut self.gui_variables.project.save_ndvd_raster,
                            "Save NDVD raster",
                        ),
                    );
                });
            });

        ui.add_space(20.);

        if ui.button("Choose save location and name").clicked()
            && let Some(mut path) = rfd::FileDialog::new()
                .add_filter("OpenOrienteering Mapper (*.omap)", &["omap"])
                .save_file()
        {
            path.set_extension("omap");
            self.gui_variables.project.save_location = path;
        };

        if self
            .gui_variables
            .project
            .save_location
            .as_os_str()
            .is_empty()
        {
            ui.label("Choose where to save the resulting omap-file.");
        } else {
            ui.label(format!(
                "{}",
                self.gui_variables.project.save_location.display()
            ));
        }

        if ui
            .add_enabled(
                !(self.gui_variables.project.paths.is_empty()
                    || self
                        .gui_variables
                        .project
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
            and doing connected component analysis on the lidar-neighbor-graph.",
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
        The different connected components of the lidar neighbor graph is colored differently.",
        );
        ui.add_space(10.);
        ui.label("Clicking a file in the list will center the map at that file's location.");
        egui::ScrollArea::both()
            .auto_shrink(false)
            .max_width(f32::INFINITY)
            .max_height(ui.available_height() / 2.)
            .show(ui, |ui| {
                for (index, p) in self.gui_variables.project.paths.iter().enumerate() {
                    if ui
                        .selectable_label(
                            self.gui_variables.project.selected_file == Some(index),
                            p.file_name()
                                .and_then(|name| name.to_str())
                                .unwrap_or_else(|| p.to_str().unwrap_or("<invalid path>")),
                        )
                        .clicked()
                    {
                        if Some(index) == self.gui_variables.project.selected_file {
                            self.gui_variables.project.selected_file = None;
                        } else {
                            self.gui_variables.project.selected_file = Some(index);
                            let center = walkers::lat_lon(
                                (self.gui_variables.lidar.boundaries[index][0].y()
                                    + self.gui_variables.lidar.boundaries[index][2].y())
                                    / 2.,
                                (self.gui_variables.lidar.boundaries[index][0].x()
                                    + self.gui_variables.lidar.boundaries[index][2].x())
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
        ui.label("A file is deemed relevant if it overlaps with the chosen map area.");

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
        ui.label("After conversion you will choose the lidar tile used for adjusting parameters.");
    }

    pub fn render_prepare_map_preview_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("Preparing test tile");
        ui.add_space(20.);
        ui.label(
            "The selected sub-tile and its neighbors are being read and prepared for parameter adjustment.",
        );
        ui.add_space(10.);
        ui.label("This calculates the raster data used by the contour and vegetation preview.");
    }

    pub fn render_draw_polygon_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("Choose map area");
        ui.add_space(20.);
        ui.label(
            "All lidar bounds are shown on the map. Draw a polygon around the area to keep, \
            or continue without one to use the full lidar coverage.",
        );
        ui.add_space(10.);
        ui.label("Click Draw polygon, then click around the area and double click to close it.");

        ui.add_space(20.);
        ui.label(format!(
            "Lidar bounds area: {:.2} km²",
            self.gui_variables.lidar.boundary_areas.iter().sum::<f64>() / 1_000_000.
        ));
        if !self.gui_variables.area.polygon_filter.0.is_empty() {
            ui.label(format!(
                "Polygon area: {:.2} km²",
                self.gui_variables.polygon_area().unwrap_or(0.) / 1_000_000.
            ));
        }

        ui.add_space(20.);
        if self.gui_variables.area.drawing_polygon {
            if ui.button("Cancel drawing").clicked() {
                self.gui_variables.area.polygon_filter.0.clear();
                self.gui_variables.area.drawing_polygon = false;
            }
            ui.label("Click the map to draw.");
        } else if self.gui_variables.area.polygon_filter.0.is_empty() {
            if ui.button("Draw polygon").clicked() {
                self.gui_variables.area.drawing_polygon = true;
            }
        } else if ui.button("Clear polygon").clicked() {
            self.gui_variables.area.polygon_filter.0.clear();
            self.gui_variables.area.drawing_polygon = false;
        }

        if self.gui_variables.area.drawing_polygon
            && !self.gui_variables.area.polygon_filter.0.is_empty()
            && !self.gui_variables.area.polygon_filter.is_closed()
        {
            ui.add_enabled(false, egui::Button::new("Double click to end polygon"));
        }

        ui.add_space(20.);
        ui.horizontal(|ui| {
            if ui.button("Start over").clicked() {
                self.open_modal = OmapModal::ConfirmStartOver;
            }
            let polygon_ready = !self.gui_variables.area.drawing_polygon
                && (self.gui_variables.area.polygon_filter.0.is_empty()
                    || self.gui_variables.area.polygon_filter.is_closed());
            if ui
                .add_enabled(polygon_ready, egui::Button::new("Next step"))
                .clicked()
            {
                self.on_frontend_task(FrontendTask::NextState);
            }
        });
    }

    pub fn render_choose_test_area_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("Select test area");
        ui.add_space(20.);
        ui.label(
            "Select a square test area on the map. At least half of the square must overlap the available lidar area.",
        );

        ui.add_space(20.);
        ui.horizontal(|ui| {
            if ui.button("Start over").clicked() {
                self.open_modal = OmapModal::ConfirmStartOver;
            }
            if ui
                .add_enabled(
                    self.gui_variables.tile.selected_square.is_some(),
                    egui::Button::new("Next step"),
                )
                .clicked()
            {
                self.on_frontend_task(FrontendTask::NextState);
            }
        });
    }

    pub fn render_adjust_slider_panel(&mut self, ui: &mut egui::Ui) {
        let (heading, help_text) = match self.state {
            ProcessStage::AdjustContours => (
                "Adjust contour settings",
                "Tune contour generation and contour geometry.",
            ),
            ProcessStage::AdjustOpenness => (
                "Adjust openness settings",
                "Tune the yellow/open-land layer and its polygon geometry.",
            ),
            ProcessStage::AdjustVegetation => (
                "Adjust vegetation settings",
                "Tune the green vegetation layers and their polygon geometry.",
            ),
            ProcessStage::AdjustCliffs => (
                "Adjust cliff settings",
                "Tune cliff detection and cliff geometry.",
            ),
            ProcessStage::AdjustIntensity => (
                "Adjust lidar intensity settings",
                "Tune lidar intensity filters and their polygon geometry.",
            ),
            _ => unreachable!("Should not render adjustment panel for {:?}", self.state),
        };

        ui.heading(heading);
        ui.add_space(20.);
        ui.label(help_text);

        egui::ScrollArea::both()
            .auto_shrink(false)
            .max_height(ui.available_height() / 1.2)
            .max_width(f32::INFINITY)
            .show(ui, |ui| match self.state {
                ProcessStage::AdjustContours => {
                    self.render_contour_adjustments(ui);
                    ui.add_space(20.);
                    ui.label(egui::RichText::new("Contour Bezier simplification").strong());
                    Self::render_bezier_parameters(
                        ui,
                        &mut self.gui_variables.generation.params.geometry.contours,
                    );
                }
                ProcessStage::AdjustOpenness => {
                    ui.label(egui::RichText::new("Openness threshold").strong());
                    ui.add(
                        egui::Slider::new(
                            &mut self.gui_variables.generation.params.vegetation.yellow,
                            0.0..=1.0,
                        )
                        .text("Yellow 403")
                        .show_value(true),
                    );
                    ui.add_space(20.);
                    ui.label(egui::RichText::new("Openness Bezier simplification").strong());
                    Self::render_bezier_parameters(
                        ui,
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .openness
                            .bezier,
                    );
                    ui.checkbox(
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .openness
                            .min_size_filter,
                        "Filter polygons by minimum symbol size.",
                    );
                    ui.add_space(20.);
                    Self::render_buffer_rules(
                        ui,
                        "openness_buffer_rule",
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .openness
                            .buffer_rules,
                    );
                }
                ProcessStage::AdjustVegetation => {
                    self.render_vegetation_adjustments(ui);
                    ui.add_space(20.);
                    ui.label(egui::RichText::new("Vegetation Bezier simplification").strong());
                    Self::render_bezier_parameters(
                        ui,
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .vegetation
                            .bezier,
                    );
                    ui.checkbox(
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .vegetation
                            .min_size_filter,
                        "Filter polygons by minimum symbol size.",
                    );
                    ui.add_space(20.);
                    Self::render_buffer_rules(
                        ui,
                        "vegetation_buffer_rule",
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .vegetation
                            .buffer_rules,
                    );
                }
                ProcessStage::AdjustCliffs => {
                    ui.label(egui::RichText::new("Cliff threshold").strong());
                    ui.add(
                        egui::Slider::new(
                            &mut self.gui_variables.generation.params.cliff.cliff,
                            0.2..=5.0,
                        )
                        .text("Cliff")
                        .show_value(true),
                    );
                    ui.add_space(20.);
                    ui.label(egui::RichText::new("Cliff Bezier simplification").strong());
                    Self::render_bezier_parameters(
                        ui,
                        &mut self.gui_variables.generation.params.geometry.cliffs.bezier,
                    );
                    ui.checkbox(
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .cliffs
                            .min_size_filter,
                        "Filter polygons by minimum symbol size.",
                    );
                    ui.add_space(20.);
                    Self::render_buffer_rules(
                        ui,
                        "cliffs_buffer_rule",
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .cliffs
                            .buffer_rules,
                    );
                }
                ProcessStage::AdjustIntensity => {
                    self.render_intensity_adjustments(ui);
                    ui.add_space(20.);
                    ui.label(egui::RichText::new("Lidar intensity Bezier simplification").strong());
                    Self::render_bezier_parameters(
                        ui,
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .intensity
                            .bezier,
                    );
                    ui.checkbox(
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .intensity
                            .min_size_filter,
                        "Filter polygons by minimum symbol size.",
                    );
                    ui.add_space(20.);
                    Self::render_buffer_rules(
                        ui,
                        "intensity_buffer_rule",
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .intensity
                            .buffer_rules,
                    );
                }
                _ => unreachable!("Should not render adjustment panel for {:?}", self.state),
            });

        ui.add_space(20.);

        let button_txt = if self.gui_variables.preview.generating_map_tile {
            "Generating map..."
        } else {
            "Re-generate map"
        };
        if ui
            .add_enabled(
                !self.gui_variables.preview.generating_map_tile,
                egui::Button::new(button_txt),
            )
            .clicked()
        {
            self.on_frontend_task(FrontendTask::DelegateTask(Task::RegenerateMap));
        }

        ui.add_space(20.);

        ui.add_enabled_ui(!self.gui_variables.preview.generating_map_tile, |ui| {
            ui.horizontal(|ui| {
                if ui.button("Prev step").clicked() {
                    self.on_frontend_task(FrontendTask::PrevState);
                }
                if ui.button("Next step").clicked() {
                    if self.state == ProcessStage::AdjustIntensity {
                        self.open_modal = OmapModal::ConfirmMakeMap;
                    } else {
                        self.on_frontend_task(FrontendTask::NextState);
                    }
                }
            });
        });
    }

    fn render_contour_adjustments(&mut self, ui: &mut egui::Ui) {
        ui.label(egui::RichText::new("Map Scale").strong());
        ui.horizontal(|ui| {
            if ui
                .selectable_label(
                    self.gui_variables.generation.params.scale == Scale::S15_000,
                    "1:15 000",
                )
                .clicked()
            {
                self.gui_variables.generation.params.scale = Scale::S15_000;
            };
            ui.separator();
            if ui
                .selectable_label(
                    self.gui_variables.generation.params.scale == Scale::S10_000,
                    "1:10 000",
                )
                .clicked()
            {
                self.gui_variables.generation.params.scale = Scale::S10_000;
            };
        });
        ui.add_space(20.);

        ui.label(egui::RichText::new("Contour algorithm parameters").strong());
        ui.horizontal(|ui| {
            ui.label("Contour algorithm:");
            egui::ComboBox::from_id_salt("Contour algo")
                .selected_text(format!(
                    "{}",
                    self.gui_variables.generation.params.contour.algorithm
                ))
                .show_ui(ui, |ui| {
                    ui.selectable_value(
                        &mut self.gui_variables.generation.params.contour.algorithm,
                        ContourAlgo::NaiveIterations,
                        "Naive interpolation error correction (slow)",
                    );
                    ui.selectable_value(
                        &mut self.gui_variables.generation.params.contour.algorithm,
                        ContourAlgo::NormalFieldSmoothing,
                        "Normal field smoothing (fast)",
                    );
                    ui.selectable_value(
                        &mut self.gui_variables.generation.params.contour.algorithm,
                        ContourAlgo::Raw,
                        "Raw contours (fastest)",
                    );
                });
        });

        if self.gui_variables.generation.params.contour.algorithm != ContourAlgo::Raw {
            if self.gui_variables.generation.params.contour.algorithm
                == ContourAlgo::NormalFieldSmoothing
            {
                ui.label("Number of smoothing iterations (usual range 5-15)");
            } else {
                ui.label("Number of error correction iterations (usual range 1-3)");
            }
            ui.add(
                egui::Slider::new(
                    &mut self.gui_variables.generation.params.contour.algo_steps,
                    1..=20,
                )
                .show_value(true),
            );
        }
        ui.add_space(10.);

        ui.label(egui::RichText::new("Contour parameters").strong());
        ui.horizontal(|ui| {
            ui.label("Contour interval: ");
            ui.add(
                egui::widgets::DragValue::new(
                    &mut self.gui_variables.generation.params.contour.interval,
                )
                .fixed_decimals(1)
                .range(1.0..=20.),
            );
        });
        ui.checkbox(
            &mut self.gui_variables.generation.params.contour.form_lines,
            "Add form lines to the map.",
        );
        ui.label("Area filter for marking small knolls as dotknolls:");
        ui.horizontal(|ui| {
            ui.add(
                egui::DragValue::new(
                    &mut self
                        .gui_variables
                        .generation
                        .params
                        .contour
                        .dot_knoll_area
                        .0,
                )
                .range(0.0..=225.0),
            );
            ui.add(
                DoubleSlider::new(
                    &mut self
                        .gui_variables
                        .generation
                        .params
                        .contour
                        .dot_knoll_area
                        .0,
                    &mut self
                        .gui_variables
                        .generation
                        .params
                        .contour
                        .dot_knoll_area
                        .1,
                    0.0..=225.0,
                )
                .separation_distance(0.),
            );
            ui.add(
                egui::DragValue::new(
                    &mut self
                        .gui_variables
                        .generation
                        .params
                        .contour
                        .dot_knoll_area
                        .1,
                )
                .range(0.0..=225.0),
            );
        });

        ui.checkbox(
            &mut self.gui_variables.generation.params.contour.basemap_contour,
            "Add basemap contours to the map.",
        );
        ui.add_enabled_ui(
            self.gui_variables.generation.params.contour.basemap_contour,
            |ui| {
                ui.label("Basemap interval: ");
                ui.add(
                    egui::widgets::DragValue::new(
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .contour
                            .basemap_interval,
                    )
                    .fixed_decimals(2)
                    .range(0.1..=self.gui_variables.generation.params.contour.interval),
                );
            },
        );
    }

    fn render_vegetation_adjustments(&mut self, ui: &mut egui::Ui) {
        ui.label(egui::RichText::new("Vegetation weighting").strong());
        let weights = &mut self.gui_variables.generation.params.vegetation.weights;
        ui.add(
            egui::Slider::new(&mut weights.low, 0.0..=1.0)
                .text("Low vegetation")
                .show_value(true),
        );
        ui.add(
            egui::Slider::new(&mut weights.medium, 0.0..=1.0)
                .text("Medium vegetation")
                .show_value(true),
        );
        ui.add(
            egui::Slider::new(&mut weights.high, 0.0..=1.0)
                .text("High vegetation")
                .show_value(true),
        );
        ui.add_space(20.);

        ui.label(egui::RichText::new("Green thresholds").strong());
        ui.add(
            egui::Slider::new(
                &mut self.gui_variables.generation.params.vegetation.green.0,
                0.0..=1.0,
            )
            .text("Green 406")
            .show_value(true),
        );
        ui.add(
            egui::Slider::new(
                &mut self.gui_variables.generation.params.vegetation.green.1,
                0.0..=1.0,
            )
            .text("Green 408")
            .show_value(true),
        );
        ui.add(
            egui::Slider::new(
                &mut self.gui_variables.generation.params.vegetation.green.2,
                0.0..=1.0,
            )
            .text("Green 410")
            .show_value(true),
        );

        let greens = &mut self.gui_variables.generation.params.vegetation.green;
        greens.0 = greens.0.clamp(0., greens.1);
        greens.2 = greens.2.clamp(greens.1, 1.);
        greens.1 = greens.1.clamp(greens.0, greens.2);
    }

    fn render_intensity_adjustments(&mut self, ui: &mut egui::Ui) {
        ui.label(egui::RichText::new("Lidar Intensity filters").strong());
        for (i, intensity_filter) in self
            .gui_variables
            .generation
            .params
            .intensity
            .filters
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
                        for area_symbol in AreaSymbol::draw_order() {
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
                    .generation
                    .params
                    .intensity
                    .filters
                    .push(Default::default());
            }
            if ui
                .add_enabled(
                    !self
                        .gui_variables
                        .generation
                        .params
                        .intensity
                        .filters
                        .is_empty(),
                    egui::Button::new("Remove filter"),
                )
                .clicked()
            {
                self.gui_variables.generation.params.intensity.filters.pop();
            }
        });
    }

    fn render_bezier_parameters(ui: &mut egui::Ui, bezier: &mut BezierParameters) {
        ui.checkbox(&mut bezier.enabled, "Output this process in Bezier curves.");
        ui.add_enabled_ui(bezier.enabled, |ui| {
            ui.label("Permitted error in Bezier simplification:");
            ui.add(
                egui::Slider::new(&mut bezier.error, 0.5..=5.0)
                    .fixed_decimals(2)
                    .show_value(true),
            );
        });
    }

    fn render_buffer_rules(ui: &mut egui::Ui, id_prefix: &str, buffer_rules: &mut Vec<BufferRule>) {
        ui.label("Add buffer rules for polygons. Rules are applied in order");
        for (i, buffer_rule) in buffer_rules.iter_mut().enumerate() {
            ui.horizontal(|ui| {
                egui::ComboBox::from_id_salt(format!("{id_prefix}_{i}"))
                    .selected_text(format!("{:?}", buffer_rule.direction))
                    .show_ui(ui, |ui| {
                        ui.selectable_value(
                            &mut buffer_rule.direction,
                            BufferDirection::Grow,
                            format!("{:?}", BufferDirection::Grow),
                        );
                        ui.selectable_value(
                            &mut buffer_rule.direction,
                            BufferDirection::Shrink,
                            format!("{:?}", BufferDirection::Shrink),
                        );
                    });
                ui.label("Distance: ");
                ui.add(egui::DragValue::new(&mut buffer_rule.amount).range(1.0..=50.0));
            });
        }
        ui.horizontal(|ui| {
            if ui.button("Add rule").clicked() {
                buffer_rules.push(Default::default());
            }
            if ui
                .add_enabled(!buffer_rules.is_empty(), egui::Button::new("Remove rule"))
                .clicked()
            {
                buffer_rules.pop();
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
            "{}.",
            self.gui_variables.project.save_location.display()
        ));
        ui.label("The map can be opened in OpenOrienteering Mapper for editing.");

        ui.add_space(20.);
        ui.label("If you like this application. Please star the project on Github:)");
        ui.hyperlink_to("OmapMaker on Github", "https://github.com/yvind/")
            .on_hover_text("https://github.com/yvind/");

        ui.add_space(20.);
        if ui.button("Start a new map").clicked() {
            self.on_frontend_task(FrontendTask::DelegateTask(Task::Reset));
        }
    }
}
