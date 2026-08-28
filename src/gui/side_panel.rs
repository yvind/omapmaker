use crate::{
    comms::messages::*,
    drawable::DrawOrder,
    map_gen::egui_map::AreaSymbol,
    parameters::{
        BezierParameters, BufferDirection, BufferRule, BuildingClassificationEvidence,
        CliffAlgorithm, ContourAlgo, FormlinePruneAlgo, Scale, StreamAlgorithm,
    },
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
        If normal las or laz files are provided, they will be written to copc.laz.",
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
            .max_height(ui.available_height() - 500.)
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

                ui.add(
                    egui::Slider::new(
                        &mut self.gui_variables.project.memory_budget_gb,
                        1..=64
                    ).text("COPC memory budget")
                ).on_hover_text(
                    "All non-COPC lidar files are converted into COPC. This is a memory intensive process. Adjust the max allowed memory usage (in GB) with this slider."
                );

                ui.label("Rasters:");
                ui.checkbox(
                    &mut self.gui_variables.project.write_raw_raster_values,
                    "Write raw numeric raster values",
                )
                .on_hover_text(
                    "Preserve metres, densities, probabilities, and other numeric samples as float32. When disabled, numeric rasters are scaled to 8-bit for ordinary image viewers. Hillshades, masks, and reason-code rasters are always viewer-scaled.",
                );
                egui::ScrollArea::vertical().max_height(200.).show(ui, |ui| {
                    ui.checkbox(&mut self.gui_variables.project.save_dem_raster, "Save DEM raster");
                    ui.checkbox(&mut self.gui_variables.project.save_slope_raster,"Save slope raster");
                    ui.checkbox(&mut self.gui_variables.project.save_hillshade_raster,"Save hillshade raster");
                    ui.checkbox(&mut self.gui_variables.project.save_intensity_raster,"Save intensity raster");
                    ui.checkbox(&mut self.gui_variables.project.save_last_return_raster,"Save last-return raster");
                    ui.checkbox(&mut self.gui_variables.project.save_canopy_height_raster,"Save canopy height raster");
                    ui.checkbox(&mut self.gui_variables.project.save_surface_objects_raster,"Save surface objects raster");
                    ui.checkbox(&mut self.gui_variables.project.save_ground_relief_2m_raster,"Save 2 m ground-relief raster");
                    ui.checkbox(&mut self.gui_variables.project.save_ground_relief_5m_raster,"Save 5 m ground-relief raster");
                    ui.checkbox(&mut self.gui_variables.project.save_hard_object_height_raster,"Save filtered hard-object height raster");
                    ui.checkbox(&mut self.gui_variables.project.save_hard_object_confidence_raster,"Save hard-object confidence raster");
                    ui.checkbox(&mut self.gui_variables.project.save_vegetation_likelihood_raster,"Save vegetation-likelihood raster");
                    ui.checkbox(&mut self.gui_variables.project.save_filtered_surface_raster,"Save vegetation-filtered surface raster");
                    ui.checkbox(&mut self.gui_variables.project.save_ndvd_raster,"Save NDVD raster");
                    ui.checkbox(&mut self.gui_variables.project.save_point_density_raster,"Save lidar point-density raster");
                    ui.checkbox(&mut self.gui_variables.project.save_flow_accumulation_raster, "Save flow accumulation raster");
                    ui.checkbox(&mut self.gui_variables.project.save_building_height_raster,"Save building height diagnostic");
                    ui.checkbox(&mut self.gui_variables.project.save_building_planarity_raster,"Save building planarity diagnostic");
                    ui.checkbox(&mut self.gui_variables.project.save_building_residual_raster,"Save building plane-residual diagnostic");
                    ui.checkbox(&mut self.gui_variables.project.save_building_probability_raster,"Save building probability diagnostic");
                    ui.checkbox(&mut self.gui_variables.project.save_building_plane_rejected_raster,"Save rejected building-plane mask");
                    ui.checkbox(&mut self.gui_variables.project.save_marsh_probability_raster,"Save marsh probability diagnostic");
                    ui.checkbox(&mut self.gui_variables.project.save_marsh_support_raster,"Save marsh observation-support diagnostic");
                    ui.checkbox(&mut self.gui_variables.project.save_marsh_wetness_raster,"Save marsh wetness diagnostic");
                    ui.checkbox(&mut self.gui_variables.project.save_marsh_reason_raster,"Save marsh reason-code diagnostic")
                    .on_hover_text(
                        "Codes: 1 insufficient support, 2 open water, 3 building, 4 non-planar surface, 5 edge-dependent drainage, 10 terrain planarity, 11 hydrology.",
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
            self.start_task(Task::NextState);
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
            self.start_task(Task::PrevState);
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
                self.start_task(Task::OpenModal(OmapModal::ConfirmStartOver));
            }
            let polygon_ready = !self.gui_variables.area.drawing_polygon
                && (self.gui_variables.area.polygon_filter.0.is_empty()
                    || self.gui_variables.area.polygon_filter.is_closed());
            if ui
                .add_enabled(polygon_ready, egui::Button::new("Next step"))
                .clicked()
            {
                self.start_task(Task::NextState);
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
                self.start_task(Task::OpenModal(OmapModal::ConfirmStartOver));
            }
            if ui
                .add_enabled(
                    self.gui_variables.tile.selected_square.is_some(),
                    egui::Button::new("Next step"),
                )
                .clicked()
            {
                self.start_task(Task::NextState);
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
            ProcessStage::AdjustBuildings => (
                "Adjust building detection",
                "Tune conservative roof-surface detection and building footprint geometry.",
            ),
            ProcessStage::AdjustCliffs => (
                "Adjust cliff settings",
                "Tune cliff detection and cliff geometry.",
            ),
            ProcessStage::AdjustWater => (
                "Adjust water settings",
                "Tune water seeds, flood-fill behavior, and the resulting polygon geometry.",
            ),
            ProcessStage::AdjustMarsh => (
                "Adjust marsh settings",
                "Tune marsh detection and the resulting polygon geometry.",
            ),
            ProcessStage::AdjustStreams => (
                "Adjust stream settings",
                "Choose and tune stream detection and line geometry.",
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
                ProcessStage::AdjustBuildings => {
                    self.render_building_adjustments(ui);
                    ui.add_space(20.);
                    ui.label(egui::RichText::new("Building Bezier simplification").strong());
                    Self::render_bezier_parameters(
                        ui,
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .buildings
                            .bezier,
                    );
                    ui.checkbox(
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .buildings
                            .min_size_filter,
                        "Filter polygons by minimum symbol size.",
                    );
                    ui.add_space(20.);
                    Self::render_buffer_rules(
                        ui,
                        "building_buffer_rule",
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .buildings
                            .buffer_rules,
                    );
                }
                ProcessStage::AdjustCliffs => {
                    ui.horizontal(|ui| {
                        ui.label("Cliff algorithm:");
                        egui::ComboBox::from_id_salt("Cliff algorithm")
                            .selected_text(
                                self.gui_variables.generation.params.cliff.algorithm.to_string(),
                            )
                            .show_ui(ui, |ui| {
                                ui.selectable_value(
                                    &mut self
                                        .gui_variables
                                        .generation
                                        .params
                                        .cliff
                                        .algorithm,
                                    CliffAlgorithm::PolynomialFit,
                                    "Polynomial fit (adaptive)",
                                );
                                ui.selectable_value(
                                    &mut self
                                        .gui_variables
                                        .generation
                                        .params
                                        .cliff
                                        .algorithm,
                                    CliffAlgorithm::SobelSlope,
                                    "Sobel slope (legacy)",
                                );
                            });
                    });
                    ui.label(egui::RichText::new("Cliff threshold").strong());
                    ui.add(
                        egui::Slider::new(
                            &mut self.gui_variables.generation.params.cliff.cliff,
                            0.2..=5.0,
                        )
                        .text("Cliff")
                        .show_value(true),
                    );
                    ui.add_space(10.);
                    ui.checkbox(&mut self.gui_variables.generation.params.cliff.collapse, "Convert linear polygons to line objects");
                    ui.add_enabled_ui(self.gui_variables.generation.params.cliff.collapse, |ui| {
                        ui.add(egui::Slider::new(
                            &mut self.gui_variables.generation.params.cliff.collapse_amount_small_cliff,
                            0.1..=5.0
                        ).text("Collapse amount").show_value(true));
                        ui.add(egui::Slider::new(
                            &mut self.gui_variables.generation.params.cliff.collapse_amount_large_cliff,
                            0.1..=5.0
                        ).text("Collapse amount").show_value(true));
                        ui.add(
                            egui::Slider::new(
                                &mut self.gui_variables.generation.params.cliff.collapse_linearity,
                                0.0..=10.0,
                            )
                            .text("Linearity threshold")
                            .show_value(true),
                        )
                        .on_hover_text(
                            "Minimum significant centerline branch length as a multiple of the collapse amount. Smaller values also convert more compact polygons. Polygons below the minimum symbol area bypass this threshold.",
                        );
                    });
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
                        "Filter lines/polygons by minimum symbol size.",
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
                ProcessStage::AdjustWater => {
                    ui.label(egui::RichText::new("Water seed detection").strong());
                    ui.add(
                        egui::Slider::new(
                            &mut self.gui_variables.generation.params.water.threshold,
                            0.0..=1.0,
                        )
                        .text("Seed probability")
                        .show_value(true),
                    )
                    .on_hover_text(
                        "Cells at or above this probability seed water regions. Higher values require stronger lidar evidence, while the elevation flood fill determines the final extent.",
                    );
                    ui.add_space(12.);
                    ui.label(egui::RichText::new("Seed buffers before flood fill").strong());
                    Self::render_buffer_rules(
                        ui,
                        "water_seed_buffer_rule",
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .water
                            .seed_buffer_rules,
                    );
                    ui.add_space(12.);
                    ui.label(egui::RichText::new("Flood fill").strong());
                    ui.add(
                        egui::Slider::new(
                            &mut self
                                .gui_variables
                                .generation
                                .params
                                .water
                                .elevation_tolerance_m,
                            0.0..=1.0,
                        )
                        .text("Level tolerance (m)")
                        .show_value(true),
                    )
                    .on_hover_text(
                        "Maximum elevation difference from a water seed included by the flood fill on the hydro-corrected elevation model.",
                    );
                    ui.checkbox(
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .water
                            .allow_downhill_flow,
                        "Allow water to follow D8 flow paths downhill.",
                    )
                    .on_hover_text(
                        "Extends filled water cells only through their hydrologically corrected D8 receivers, avoiding the lateral leakage caused by accepting every lower neighboring cell.",
                    );
                    ui.add_space(20.);
                    ui.label(egui::RichText::new("Water Bezier simplification").strong());
                    Self::render_bezier_parameters(
                        ui,
                        &mut self.gui_variables.generation.params.geometry.water.bezier,
                    );
                    ui.checkbox(
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .water
                            .min_size_filter,
                        "Filter polygons by minimum symbol size.",
                    );
                    ui.add_space(20.);
                    ui.label(egui::RichText::new("Buffers after flood fill").strong());
                    Self::render_buffer_rules(
                        ui,
                        "water_output_buffer_rule",
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .water
                            .buffer_rules,
                    );
                }
                ProcessStage::AdjustStreams => {
                    #[cfg(feature = "deep-learning")]
                    ui.horizontal(|ui| {
                        ui.label("Stream algorithm:");
                        egui::ComboBox::from_id_salt("Stream algorithm")
                            .selected_text(
                                self.gui_variables
                                    .generation
                                    .params
                                    .streams
                                    .algorithm
                                    .to_string(),
                            )
                            .show_ui(ui, |ui| {
                                ui.selectable_value(
                                    &mut self
                                        .gui_variables
                                        .generation
                                        .params
                                        .streams
                                        .algorithm,
                                    StreamAlgorithm::Hydrological,
                                    "Hydrological flow accumulation",
                                );
                                ui.selectable_value(
                                    &mut self
                                        .gui_variables
                                        .generation
                                        .params
                                        .streams
                                        .algorithm,
                                    StreamAlgorithm::DitchesStreamsSvfSlope,
                                    "ONNX sky-view factor and slope",
                                );
                            });
                    });

                    if self.gui_variables.generation.params.streams.algorithm
                        == StreamAlgorithm::Hydrological
                    {
                        ui.label(egui::RichText::new("Stream initiation").strong());
                        ui.add(
                            egui::Slider::new(
                                &mut self
                                    .gui_variables
                                    .generation
                                    .params
                                    .streams
                                    .minimum_catchment_area_m2,
                                100.0..=50_000.0,
                            )
                            .logarithmic(true)
                            .text("Minimum catchment area (m²)")
                            .show_value(true),
                        )
                        .on_hover_text(
                            "Minimum upstream area needed to map a small crossable watercourse. Candidates must also have positive cross-channel curvature in the original elevation model.",
                        );
                    }
                    #[cfg(feature = "deep-learning")]
                    if self.gui_variables.generation.params.streams.algorithm
                        == StreamAlgorithm::DitchesStreamsSvfSlope
                    {
                        ui.label(
                            "Embedded WGPU model: Ditches and streams from sky-view factor and slope",
                        );
                        ui.label(egui::RichText::new("ONNX raster-to-vector extraction").strong());
                        let vectorization = &mut self
                            .gui_variables
                            .generation
                            .params
                            .streams
                            .onnx_vectorization;
                        ui.add(
                            egui::Slider::new(&mut vectorization.confidence_threshold, 0.0..=1.0)
                                .text("Confidence threshold")
                                .show_value(true),
                        )
                        .on_hover_text(
                            "The winning ditch or stream class must reach this probability and beat background. Zero preserves pure highest-probability-class extraction.",
                        );
                        ui.add(
                            egui::Slider::new(&mut vectorization.polygon_buffer_m, -3.0..=3.0)
                                .text("Prediction polygon buffer (m)")
                                .show_value(true),
                        )
                        .on_hover_text(
                            "Grow positive raster prediction polygons or shrink them with a negative value before their centerlines are extracted.",
                        );
                        ui.add(
                            egui::Slider::new(
                                &mut vectorization.endpoint_merge_distance_m,
                                0.0..=25.0,
                            )
                            .text("Endpoint merge distance (m)")
                            .show_value(true),
                        )
                        .on_hover_text(
                            "Join adjacent extracted stream lines when one line end is within this distance of the next line start.",
                        );
                        egui::CollapsingHeader::new("Advanced ONNX vector geometry")
                            .default_open(false)
                            .show(ui, |ui| {
                                ui.add(
                                    egui::Slider::new(
                                        &mut vectorization.centerline_sampling_distance_m,
                                        0.1..=5.0,
                                    )
                                    .logarithmic(true)
                                    .text("Centerline sampling distance (m)"),
                                )
                                .on_hover_text(
                                    "Boundary sampling distance for medial-axis construction. Smaller values retain more detail but cost more to process.",
                                );
                                ui.add(
                                    egui::Slider::new(
                                        &mut vectorization.minimum_branch_length_m,
                                        0.0..=20.0,
                                    )
                                    .text("Minimum branch length (m)"),
                                )
                                .on_hover_text(
                                    "Remove shorter terminal branches from the extracted medial axis.",
                                );
                                ui.add(
                                    egui::Slider::new(
                                        &mut vectorization.branch_length_exemption_area_m2,
                                        0.0..=100.0,
                                    )
                                    .text("Branch exemption area (m²)"),
                                )
                                .on_hover_text(
                                    "Small prediction components below this area bypass minimum-branch-length pruning.",
                                );
                                ui.add(
                                    egui::Slider::new(
                                        &mut vectorization.simplification_tolerance_m,
                                        0.0..=2.0,
                                    )
                                    .text("Line simplification (m)"),
                                )
                                .on_hover_text(
                                    "Douglas-Peucker tolerance applied after centerline extraction. Zero disables simplification.",
                                );
                            });
                    }
                    ui.add_space(20.);
                    ui.label(egui::RichText::new("Stream Bezier simplification").strong());
                    Self::render_bezier_parameters(
                        ui,
                        &mut self.gui_variables.generation.params.geometry.streams,
                    );
                }
                ProcessStage::AdjustMarsh => {
                    ui.label(egui::RichText::new("Marsh detection").strong());
                    ui.checkbox(
                        &mut self.gui_variables.generation.params.marsh.enabled,
                        "Generate marsh polygons",
                    );
                    ui.add(
                        egui::Slider::new(
                            &mut self.gui_variables.generation.params.marsh.sensitivity,
                            0.0..=1.0,
                        )
                        .text("Sensitivity")
                        .show_value(true),
                    )
                    .on_hover_text(
                        "Higher sensitivity lowers both seed and growth thresholds while retaining the expert evidence weights.",
                    );
                    ui.add(
                        egui::Slider::new(
                            &mut self
                                .gui_variables
                                .generation
                                .params
                                .marsh
                                .minimum_polygon_area_m2,
                            1.0..=2_000.0,
                        )
                        .logarithmic(true)
                        .text("Minimum marsh area (m²)")
                        .show_value(true),
                    );
                    egui::CollapsingHeader::new("Advanced marsh evidence")
                        .default_open(false)
                        .show(ui, |ui| {
                            let marsh = &mut self.gui_variables.generation.params.marsh;
                            ui.add(
                                egui::Slider::new(&mut marsh.planarity_radius_m, 0.5..=10.0)
                                    .text("Local planarity radius (m)"),
                            );
                            ui.add(
                                egui::Slider::new(
                                    &mut marsh.maximum_planarity_rmse_m,
                                    0.01..=0.5,
                                )
                                .text("Maximum plane residual (m)"),
                            );
                            ui.add(
                                egui::Slider::new(
                                    &mut marsh.drainage_initiation_area_m2,
                                    100.0..=50_000.0,
                                )
                                .logarithmic(true)
                                .text("Drainage initiation (m²)"),
                            );
                            ui.add(
                                egui::Slider::new(
                                    &mut marsh.maximum_height_above_drainage_m,
                                    0.1..=5.0,
                                )
                                .text("Maximum HAND (m)"),
                            );
                            ui.add(
                                egui::Slider::new(
                                    &mut marsh.maximum_downslope_distance_m,
                                    2.0..=150.0,
                                )
                                .text("Maximum drainage distance (m)"),
                            );
                            ui.add(
                                egui::Slider::new(
                                    &mut marsh.preferred_depression_depth_m,
                                    0.05..=1.5,
                                )
                                .text("Preferred depression depth (m)"),
                            );
                            ui.add(
                                egui::Slider::new(
                                    &mut marsh.minimum_wetness_score,
                                    0.0..=1.0,
                                )
                                .text("Minimum wetness"),
                            );
                            ui.add(
                                egui::Slider::new(&mut marsh.seed_threshold, 0.05..=1.0)
                                    .text("Seed threshold"),
                            );
                            ui.add(
                                egui::Slider::new(&mut marsh.growth_threshold, 0.0..=0.95)
                                    .text("Growth threshold"),
                            );
                            ui.add(
                                egui::Slider::new(&mut marsh.closing_radius_m, 0.0..=10.0)
                                    .text("Closing radius (m)"),
                            );
                            ui.add(
                                egui::Slider::new(&mut marsh.opening_radius_m, 0.0..=10.0)
                                    .text("Opening radius (m)"),
                            );
                            ui.add(
                                egui::Slider::new(
                                    &mut marsh.maximum_hole_area_m2,
                                    0.0..=500.0,
                                )
                                .text("Fill holes up to (m²)"),
                            );
                            ui.separator();
                            ui.label("Relative evidence weights");
                            ui.add(
                                egui::Slider::new(&mut marsh.weights.terrain, 0.0..=1.0)
                                    .text("Terrain"),
                            );
                            ui.add(
                                egui::Slider::new(&mut marsh.weights.hydrology, 0.0..=1.0)
                                    .text("Hydrology"),
                            );
                        });
                    ui.add_space(12.);
                    ui.label(egui::RichText::new("Marsh Bezier simplification").strong());
                    Self::render_bezier_parameters(
                        ui,
                        &mut self.gui_variables.generation.params.geometry.marsh.bezier,
                    );
                    ui.checkbox(
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .marsh
                            .min_size_filter,
                        "Filter marsh polygons by minimum symbol size.",
                    );
                    Self::render_buffer_rules(
                        ui,
                        "marsh_output_buffer_rule",
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .marsh
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
            self.start_task(Task::RegenerateMap(RegenerationScope::Changed));
        }

        ui.add_space(20.);

        ui.add_enabled_ui(!self.gui_variables.preview.generating_map_tile, |ui| {
            ui.horizontal(|ui| {
                if ui.button("Prev step").clicked() {
                    self.start_task(Task::PrevState);
                }
                if ui.button("Next step").clicked() {
                    if self.state == ProcessStage::AdjustIntensity {
                        self.start_task(Task::OpenModal(OmapModal::ConfirmMakeMap));
                    } else {
                        self.start_task(Task::NextState);
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

        ui.add_space(10.);

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
                        ContourAlgo::WeightedScalarField,
                        "Weighted scalar field",
                    );
                    ui.selectable_value(
                        &mut self.gui_variables.generation.params.contour.algorithm,
                        ContourAlgo::Raw,
                        "Raw contours (fastest)",
                    );
                });
        });

        if self.gui_variables.generation.params.contour.algorithm
            == ContourAlgo::WeightedScalarField
        {
            ui.label("Maximum optimization iterations");
            ui.add(
                egui::Slider::new(
                    &mut self
                        .gui_variables
                        .generation
                        .params
                        .contour
                        .contour_field
                        .max_iterations,
                    20..=400,
                )
                .show_value(true),
            );
            ui.horizontal(|ui| {
                use crate::parameters::ContourGeneralization;
                ui.label("Terrain attraction/generalization:");
                egui::ComboBox::from_id_salt("Contour generalization")
                    .selected_text(
                        self.gui_variables
                            .generation
                            .params
                            .contour
                            .contour_field
                            .generalization
                            .to_string(),
                    )
                    .show_ui(ui, |ui| {
                        for value in [
                            ContourGeneralization::Light,
                            ContourGeneralization::Balanced,
                            ContourGeneralization::Strong,
                        ] {
                            ui.selectable_value(
                                &mut self
                                    .gui_variables
                                    .generation
                                    .params
                                    .contour
                                    .contour_field
                                    .generalization,
                                value,
                                value.to_string(),
                            );
                        }
                    });
            });
            ui.horizontal(|ui| {
                ui.label("Persistence threshold:");
                ui.add(
                    egui::widgets::DragValue::new(
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .contour
                            .contour_field
                            .persistence_threshold_fraction,
                    )
                    .speed(0.01)
                    .range(0.0..=0.5),
                );
            });
        } else if self.gui_variables.generation.params.contour.algorithm != ContourAlgo::Raw {
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

        ui.label(egui::RichText::new("Formline algorithm parameters").strong());
        ui.checkbox(
            &mut self.gui_variables.generation.params.contour.form_lines,
            "Add form lines to the map.",
        );
        ui.add_enabled_ui(
            self.gui_variables.generation.params.contour.form_lines,
            |ui| {
                ui.horizontal(|ui| {
                    ui.label("Form-line pruning algorithm:");
                    egui::ComboBox::from_id_salt("Form-line pruning algorithm")
                        .selected_text(format!(
                            "{}",
                            self
                                .gui_variables
                                .generation
                                .params
                                .contour
                                .form_line_prune_algorithm
                        ))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(
                                &mut self
                                    .gui_variables
                                    .generation
                                    .params
                                    .contour
                                    .form_line_prune_algorithm,
                                FormlinePruneAlgo::None,
                                "None",
                            );
                            ui.selectable_value(
                                &mut self
                                    .gui_variables
                                    .generation
                                    .params
                                    .contour
                                    .form_line_prune_algorithm,
                                FormlinePruneAlgo::TerrainChange,
                                "Terrain change",
                            );
                            ui.selectable_value(
                                &mut self
                                    .gui_variables
                                    .generation
                                    .params
                                    .contour
                                    .form_line_prune_algorithm,
                                FormlinePruneAlgo::InterpolationError,
                                "Contour interpolation error",
                            );
                        });
                });

                match self
                    .gui_variables
                    .generation
                    .params
                    .contour
                    .form_line_prune_algorithm
                {
                    FormlinePruneAlgo::TerrainChange => {
                        ui.add(
                            egui::Slider::new(
                                &mut self
                                    .gui_variables
                                    .generation
                                    .params
                                    .contour
                                    .form_line_prune_threshold,
                                0.05..=5.0,
                            )
                            .logarithmic(true)
                            .text("Terrain-change threshold")
                            .show_value(true),
                        )
                        .on_hover_text(
                            "Higher values keep form lines only where slope or elevation curvature is stronger.",
                        );
                    }
                    FormlinePruneAlgo::InterpolationError => {
                        ui.add(
                            egui::Slider::new(
                                &mut self
                                    .gui_variables
                                    .generation
                                    .params
                                    .contour
                                    .form_line_error_threshold,
                                0.01..=5.0,
                            )
                            .logarithmic(true)
                            .text("Error improvement threshold (m)")
                            .show_value(true),
                        )
                        .on_hover_text(
                            "Minimum local reduction in elevation reconstruction error required to retain a form line.",
                        );
                    }
                    FormlinePruneAlgo::None => {
                        ui.label(
                            "All form lines are important; shared length and ring rules still apply.",
                        );
                    }
                }
                ui.horizontal(|ui| {
                    ui.label("Minimum open/closed length (m, 0 = symbol default):");
                    ui.add(
                        egui::DragValue::new(
                            &mut self
                                .gui_variables
                                .generation
                                .params
                                .contour
                                .form_line_geometry
                                .minimum_open_length_m,
                        )
                        .speed(0.5)
                        .range(0.0..=100.0),
                    );
                    ui.add(
                        egui::DragValue::new(
                            &mut self
                                .gui_variables
                                .generation
                                .params
                                .contour
                                .form_line_geometry
                                .minimum_closed_length_m,
                        )
                        .speed(0.5)
                        .range(0.0..=100.0),
                    );
                });
                ui.horizontal(|ui| {
                    ui.label("Reconnect culled gaps up to (m):");
                    ui.add(
                        egui::DragValue::new(
                            &mut self
                                .gui_variables
                                .generation
                                .params
                                .contour
                                .form_line_geometry
                                .reconnect_gap_m,
                        )
                        .speed(0.25)
                        .range(0.0..=20.0),
                    );
                });
            },
        );

        ui.add_space(10.);
        ui.label(egui::RichText::new("Basemap parameters").strong());
        ui.checkbox(
            &mut self.gui_variables.generation.params.contour.basemap_contour,
            "Add basemap contours to the map.",
        );
        ui.add_enabled_ui(
            self.gui_variables.generation.params.contour.basemap_contour,
            |ui| {
                ui.horizontal(|ui| {
                    ui.label("Basemap interval:");
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
                });
            },
        );

        ui.add_space(10.);
        ui.label(egui::RichText::new("Dotknoll filter").strong());

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
    }

    fn render_building_adjustments(&mut self, ui: &mut egui::Ui) {
        let parameters = &mut self.gui_variables.generation.params.building;
        ui.checkbox(&mut parameters.enabled, "Detect buildings from LiDAR");
        ui.add_enabled_ui(parameters.enabled, |ui| {
            ui.label(egui::RichText::new("Roof elevation and fit").strong());
            ui.add(
                egui::Slider::new(&mut parameters.minimum_roof_height_m, 0.5..=10.)
                    .text("Minimum roof height (m)"),
            );
            ui.add(
                egui::Slider::new(&mut parameters.maximum_roof_height_m, 5.0..=100.)
                    .text("Maximum roof height (m)"),
            );
            ui.add(
                egui::Slider::new(&mut parameters.plane_fit_radius_m, 0.5..=8.)
                    .text("Candidate point radius (m)"),
            );
            ui.add(
                egui::Slider::new(&mut parameters.maximum_plane_residual_m, 0.02..=1.)
                    .text("RANSAC inlier distance (m)"),
            );
            ui.add(
                egui::Slider::new(&mut parameters.minimum_planar_point_fraction, 0.0..=1.)
                    .text("Minimum planar fraction"),
            );
            ui.add(
                egui::Slider::new(&mut parameters.maximum_roof_slope_degrees, 1.0..=89.)
                    .text("Maximum roof slope (°)"),
            );
            ui.add(
                egui::Slider::new(&mut parameters.ransac_iterations, 10..=300)
                    .text("RANSAC iterations"),
            );
            ui.add(
                egui::Slider::new(&mut parameters.ransac_sample_size, 3..=8)
                    .text("RANSAC sample size"),
            );
            if parameters.minimum_plane_inliers < parameters.ransac_sample_size {
                parameters.minimum_plane_inliers = parameters.ransac_sample_size;
            }
            ui.add(
                egui::Slider::new(&mut parameters.minimum_plane_inliers, 3..=100)
                    .text("Minimum plane inliers"),
            );
            ui.add(
                egui::Slider::new(&mut parameters.maximum_roof_planes, 1..=20)
                    .text("Maximum roof planes"),
            );

            ui.add_space(12.);
            ui.label(egui::RichText::new("Candidate assembly").strong());
            ui.add(egui::Slider::new(&mut parameters.merge_gap_m, 0.0..=4.).text("Merge gap (m)"));
            ui.add(
                egui::Slider::new(&mut parameters.maximum_candidate_hole_area_m2, 0.0..=50.)
                    .text("Maximum hole area (m²)"),
            );

            ui.add_space(12.);
            ui.label(egui::RichText::new("Candidate acceptance").strong());
            ui.add(
                egui::Slider::new(&mut parameters.minimum_building_area_m2, 1.0..=500.)
                    .logarithmic(true)
                    .text("Minimum area (m²)"),
            );
            ui.add(
                egui::Slider::new(
                    &mut parameters.minimum_rectangularity_or_compactness,
                    0.0..=1.,
                )
                .text("Minimum shape score"),
            );
            ui.add(
                egui::Slider::new(&mut parameters.maximum_vegetation_fraction, 0.0..=1.)
                    .text("Maximum vegetation evidence"),
            );
            ui.add(
                egui::Slider::new(&mut parameters.confidence_threshold, 0.0..=1.)
                    .text("Confidence threshold"),
            );
            ui.horizontal(|ui| {
                ui.label("LAS class 6:");
                egui::ComboBox::from_id_salt("building_class_6_evidence")
                    .selected_text(parameters.class_6_evidence.to_string())
                    .show_ui(ui, |ui| {
                        ui.selectable_value(
                            &mut parameters.class_6_evidence,
                            BuildingClassificationEvidence::Authoritative,
                            "Authoritative",
                        );
                        ui.selectable_value(
                            &mut parameters.class_6_evidence,
                            BuildingClassificationEvidence::Supporting,
                            "Supporting evidence",
                        );
                        ui.selectable_value(
                            &mut parameters.class_6_evidence,
                            BuildingClassificationEvidence::Ignore,
                            "Ignore",
                        );
                    });
            });

            ui.add_space(12.);
            ui.label(egui::RichText::new("Footprint regularization").strong());
            ui.checkbox(
                &mut parameters.regularize_footprints,
                "Align edges to a dominant building direction",
            );
            ui.add_enabled_ui(parameters.regularize_footprints, |ui| {
                ui.add(
                    egui::Slider::new(
                        &mut parameters.regularization_simplification_tolerance_m,
                        0.0..=2.0,
                    )
                    .text("Pre-simplification (m)"),
                );
                ui.add(
                    egui::Slider::new(
                        &mut parameters.regularization_parallel_threshold_m,
                        0.0..=2.0,
                    )
                    .text("Parallel-edge merge (m)"),
                );
                ui.add(
                    egui::Slider::new(
                        &mut parameters.regularization_maximum_boundary_displacement_m,
                        0.1..=3.0,
                    )
                    .text("Maximum boundary movement (m)"),
                );
                ui.add(
                    egui::Slider::new(
                        &mut parameters.regularization_maximum_angle_deviation_degrees,
                        1.0..=45.0,
                    )
                    .text("Maximum snap angle (°)"),
                );
                ui.add(
                    egui::Slider::new(
                        &mut parameters.regularization_minimum_supported_edge_fraction,
                        0.0..=1.0,
                    )
                    .text("Minimum supported edge fraction"),
                );
                ui.add(
                    egui::Slider::new(&mut parameters.regularization_minimum_iou, 0.1..=1.0)
                        .text("Minimum footprint IoU"),
                );
                ui.checkbox(
                    &mut parameters.regularization_allow_45_degree_edges,
                    "Allow 45° edges",
                );
                ui.add_enabled(
                    parameters.regularization_allow_45_degree_edges,
                    egui::Slider::new(
                        &mut parameters.regularization_diagonal_bias_degrees,
                        0.0..=22.5,
                    )
                    .text("Diagonal snap bias (°)"),
                );
            });
        });
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
        ui.label("Rules are applied in order.");
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
                ui.add(egui::DragValue::new(&mut buffer_rule.amount).range(0.1..=25.0));
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
            self.start_task(Task::Reset);
        }
    }
}
