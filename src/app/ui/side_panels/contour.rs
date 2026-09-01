use crate::{
    app::OmapMaker,
    parameters::{ContourAlgo, FormlinePruneAlgo, Scale},
};
use eframe::egui;
use egui_double_slider::DoubleSlider;

impl OmapMaker {
    pub(super) fn render_contour_adjustments(&mut self, ui: &mut egui::Ui) {
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
}
