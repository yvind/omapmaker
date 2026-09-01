use crate::app::{OmapMaker, protocol::AppAction};
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
}
