use crate::{app::OmapMaker, parameters::BuildingClassificationEvidence};
use eframe::egui;

impl OmapMaker {
    pub(super) fn render_building_adjustments(&mut self, ui: &mut egui::Ui) {
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
}
