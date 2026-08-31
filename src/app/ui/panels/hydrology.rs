use crate::{
    app::OmapMaker, generation::features::streams::available_algorithms,
    parameters::StreamAlgorithm,
};
use eframe::egui;

impl OmapMaker {
    pub(super) fn render_water_adjustments(&mut self, ui: &mut egui::Ui) {
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
            &mut self.gui_variables.generation.params.water.seed_buffer_rules,
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

    pub(super) fn render_stream_adjustments(&mut self, ui: &mut egui::Ui) {
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
                    for &algorithm in available_algorithms() {
                        ui.selectable_value(
                            &mut self.gui_variables.generation.params.streams.algorithm,
                            algorithm,
                            algorithm.to_string(),
                        );
                    }
                });
        });

        if self.gui_variables.generation.params.streams.algorithm == StreamAlgorithm::Hydrological {
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
        if self.gui_variables.generation.params.streams.algorithm
            == StreamAlgorithm::DitchesStreamsSvfSlope
        {
            ui.label("Embedded WGPU model: Ditches and streams from sky-view factor and slope");
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

    pub(super) fn render_marsh_adjustments(&mut self, ui: &mut egui::Ui) {
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
                    egui::Slider::new(&mut marsh.maximum_planarity_rmse_m, 0.01..=0.5)
                        .text("Maximum plane residual (m)"),
                );
                ui.add(
                    egui::Slider::new(&mut marsh.drainage_initiation_area_m2, 100.0..=50_000.0)
                        .logarithmic(true)
                        .text("Drainage initiation (m²)"),
                );
                ui.add(
                    egui::Slider::new(&mut marsh.maximum_height_above_drainage_m, 0.1..=5.0)
                        .text("Maximum HAND (m)"),
                );
                ui.add(
                    egui::Slider::new(&mut marsh.maximum_downslope_distance_m, 2.0..=150.0)
                        .text("Maximum drainage distance (m)"),
                );
                ui.add(
                    egui::Slider::new(&mut marsh.preferred_depression_depth_m, 0.05..=1.5)
                        .text("Preferred depression depth (m)"),
                );
                ui.add(
                    egui::Slider::new(&mut marsh.minimum_wetness_score, 0.0..=1.0)
                        .text("Minimum wetness"),
                );
                ui.add(
                    egui::Slider::new(&mut marsh.seed_threshold, 0.05..=1.0).text("Seed threshold"),
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
                    egui::Slider::new(&mut marsh.maximum_hole_area_m2, 0.0..=500.0)
                        .text("Fill holes up to (m²)"),
                );
                ui.separator();
                ui.label("Relative evidence weights");
                ui.add(egui::Slider::new(&mut marsh.weights.terrain, 0.0..=1.0).text("Terrain"));
                ui.add(
                    egui::Slider::new(&mut marsh.weights.hydrology, 0.0..=1.0).text("Hydrology"),
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
}
