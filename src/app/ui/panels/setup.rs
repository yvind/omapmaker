use crate::app::{OmapMaker, OmapModal, protocol::AppAction};
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
                    .text("Worker threads"),
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
            self.dispatch_action(AppAction::NextState);
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
            self.dispatch_action(AppAction::PrevState);
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
                self.dispatch_action(AppAction::OpenModal(OmapModal::ConfirmStartOver));
            }
            let polygon_ready = !self.gui_variables.area.drawing_polygon
                && (self.gui_variables.area.polygon_filter.0.is_empty()
                    || self.gui_variables.area.polygon_filter.is_closed());
            if ui
                .add_enabled(polygon_ready, egui::Button::new("Next step"))
                .clicked()
            {
                self.dispatch_action(AppAction::NextState);
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
                self.dispatch_action(AppAction::OpenModal(OmapModal::ConfirmStartOver));
            }
            if ui
                .add_enabled(
                    self.gui_variables.tile.selected_square.is_some(),
                    egui::Button::new("Next step"),
                )
                .clicked()
            {
                self.dispatch_action(AppAction::NextState);
            }
        });
    }
}
