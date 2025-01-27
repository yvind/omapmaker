use laz2omap::comms::messages::*;

use super::OmapMaker;
use eframe::egui;
use egui::Modal;

#[derive(Clone)]
pub enum OmapModal {
    None,
    OutputCRS(u16),
    ManualSetCRS,
    SetOneCrsForAll,
    SetOneCrsForEach,
    ConfirmDropAll,
    ConfirmStartOver,
    ConfirmMakeMap,
    MultipleGraphComponents,
    ErrorModal(String),
    WaiverModal,
}

impl OmapMaker {
    pub fn confirm_drop_all_modal(&mut self, ctx: &egui::Context) {
        let drop_modal = Modal::new(egui::Id::new("Drop all?"));
        drop_modal.show(ctx, |ui| {
            ui.heading("Drop all non-georefrenced lidar files");
            ui.separator();
            ui.label("Do you want to remove all the lidar files no CRS could be associated with?");
            ui.separator();
            egui::Sides::new().show(
                ui,
                |_ui| {},
                |ui| {
                    if ui.button("No, go back!").clicked() {
                        self.open_modal = OmapModal::ManualSetCRS;
                    };
                    if ui.button("Yes, drop the files!").clicked() {
                        self.open_modal = OmapModal::None;
                        self.on_frontend_task(FrontEndTask::DelegateTask(Task::SetCrs(
                            SetCrs::DropAll,
                        )));
                    }
                },
            );
        });
    }

    pub fn set_one_crs_for_all_modal(&mut self, ctx: &egui::Context) {
        let mut crs_set = false;

        let single_epsg_modal = Modal::new(egui::Id::new("set single epsg"));
        single_epsg_modal.show(ctx, |ui| {
            ui.heading("Choose CRS by EPSG code");
            ui.separator();
            ui.label(
                "Select a CRS by EPSG code from the list.\n\
            The wkt-definition of the chosen CRS will be displayed below.",
            );

            ui.horizontal(|ui| {
                ui.label("Choose CRS by EPSG code (4 or 5 digits)");
                ui.add(
                    egui_autocomplete::AutoCompleteTextEdit::new(
                        &mut self.gui_variables.crs_less_search_strings[0],
                        crate::epsg_list::EPSG_LIST,
                    )
                    .highlight_matches(true)
                    .set_text_edit_properties(|t| t.char_limit(5)),
                );
            });
            if let Ok(code) = self.gui_variables.crs_less_search_strings[0].parse::<u16>() {
                if let Some(proj) = crs_definitions::from_code(code) {
                    ui.label(proj.wkt);
                    crs_set = true;
                } else {
                    ui.label("Input CRS not recognized");
                }
            } else {
                ui.label("Could not parse input CRS code");
            }
            ui.separator();
            egui::Sides::new().show(
                ui,
                |_ui| {},
                |ui| {
                    if ui
                        .add_enabled(crs_set, egui::Button::new("Save choice of CRS"))
                        .clicked()
                    {
                        self.open_modal = OmapModal::None;
                        self.on_frontend_task(FrontEndTask::DelegateTask(Task::SetCrs(
                            SetCrs::SetAllEpsg,
                        )));
                    };
                    if ui.button("Go back").clicked() {
                        self.open_modal = OmapModal::ManualSetCRS;
                    }
                },
            );
        });
    }

    pub fn set_one_crs_for_each_modal(&mut self, ctx: &egui::Context) {
        let mut crs_less_files = vec![];
        for (i, crs) in self.gui_variables.crs_epsg.iter().enumerate() {
            if *crs == u16::MAX {
                crs_less_files.push(i);
            }
        }
        assert!(crs_less_files.len() == self.gui_variables.drop_checkboxes.len());

        let mut crs_set = true;
        let all_epsg_modal = Modal::new(egui::Id::new("set all epsg"));
        all_epsg_modal.show(ctx, |ui| {
            ui.heading("Choose CRS by EPSG-code or drop for all non-CRS files");
            ui.separator();
            ui.label("Filename");
            egui::ScrollArea::both().show(ui, |ui| {
                for (i, crs_less) in crs_less_files.iter().enumerate() {
                    let disabled = self.gui_variables.drop_checkboxes[i];
                    ui.horizontal(|ui| {
                        ui.label(format!(
                            "{:?}",
                            self.gui_variables.paths[*crs_less].file_name().unwrap()
                        ));
                        ui.vertical(|ui| {
                            ui.checkbox(
                                &mut self.gui_variables.drop_checkboxes[i],
                                "Drop this file",
                            );
                            ui.horizontal(|ui| {
                                ui.label("EPSG: ");
                                ui.add_enabled(
                                    !disabled,
                                    egui_autocomplete::AutoCompleteTextEdit::new(
                                        &mut self.gui_variables.crs_less_search_strings[i],
                                        crate::epsg_list::EPSG_LIST,
                                    )
                                    .highlight_matches(true)
                                    .max_suggestions(10)
                                    .set_text_edit_properties(|t| t.char_limit(5)),
                                );
                            });
                            if !disabled {
                                let crs_str = &self.gui_variables.crs_less_search_strings[i];
                                if let Ok(code) = crs_str.parse::<u16>() {
                                    if crs_definitions::from_code(code).is_none() {
                                        crs_set = false;
                                        ui.label("Invalid EPSG code");
                                    }
                                } else {
                                    crs_set = false;
                                    ui.label("Unable to parse EPSG code");
                                }
                            }
                        });
                    });
                }
            });
            ui.separator();
            egui::Sides::new().show(
                ui,
                |_ui| {},
                |ui| {
                    if ui.add_enabled(crs_set, egui::Button::new("Done")).clicked() {
                        self.open_modal = OmapModal::None;
                        self.on_frontend_task(FrontEndTask::DelegateTask(Task::SetCrs(
                            SetCrs::SetEachCrs,
                        )));
                    }
                    if ui.button("Go back").clicked() {
                        self.open_modal = OmapModal::ManualSetCRS;
                    }
                },
            );
        });
    }

    pub fn manual_set_crs_modal(&mut self, ctx: &egui::Context) {
        let no_crs_dialog = Modal::new(egui::Id::new("CRS-less lidar"));
        no_crs_dialog.show(ctx, |ui| {

            ui.heading("Choose CRS for files without detected CRS");
            ui.separator();
            ui.label(
                "No CRS was detected for one or more files.\n\
            For georeferencing purposes all files need to be associated with a CRS.\n\
            Assign CRS or drop files by clicking the buttons.\n\n\
            Hover over the different buttons for an explanation of what they do. \n\n\
            Depending on the number of unique CRS's detected among the files, different options are presented.",
            );
            ui.separator();

            ui.horizontal(|ui|{
                if self.gui_variables.unique_crs.is_empty() {
                    if ui.button("Use \"Local Coordinates\"")
                        .on_hover_text("This option is only available if no CRS has been detected among the lidar files. \
                        This button assumes they all are in same CRS without caring about which. \
                        The output map will not be georefrenced, but everything should work fine regardless.")
                        .clicked()
                        {
                            self.open_modal = OmapModal::None;
                            self.on_frontend_task(FrontEndTask::DelegateTask(Task::SetCrs(SetCrs::Local)));
                        }
                } else if self.gui_variables.unique_crs.len() == 1 {
                    #[allow(clippy::collapsible_if)]
                    if ui.button("Use default CRS")
                        .on_hover_text("This option is only available if only one unique CRS has been detected among the lidar files. \
                        This button assosciates all CRS-less files with that unique CRS.")
                        .clicked()
                    {
                        self.open_modal = OmapModal::None;
                        self.on_frontend_task(FrontEndTask::DelegateTask(Task::SetCrs(SetCrs::Default)));
                    }
                }

                if ui.button("Choose one EPSG code for all").on_hover_text("Choose a CRS by EPSG code and associate all CRS-less files with that CRS.")
                .clicked() {
                    self.open_modal = OmapModal::SetOneCrsForAll;
                }
                if ui
                    .button("Choose EPSG code or drop file for each")
                    .on_hover_text("Choose wether to set a CRS by EPSG code or to drop the file for each CRS-less file.")
                    .clicked()
                {
                    self.open_modal = OmapModal::SetOneCrsForEach;
                }
                if ui.button("Drop all non-CRS files")
                .on_hover_text("Remove all CRS-less files from the list of lidar files.")
                .clicked() {
                    self.open_modal = OmapModal::ConfirmDropAll;
                }
            });
        });
    }

    pub fn output_crs_modal(&mut self, ctx: &egui::Context, majority_epsg: u16) {
        let mut transform_crs = None;
        let transform_modal = Modal::new(egui::Id::new("output CRS modal"));
        transform_modal.show(ctx, |ui| {
            ui.heading("Choose output CRS");
            ui.separator();
            ui.label("Choose the output CRS of the map. \
            As every relevant Lidar file gets converted into this CRS it is recommended to click the \"Majority Vote\" button. \
            This will lead to the fewest (maybe none) time consuming file transformations. \
            It makes sense to choose another CRS if your files are in imperial units, but it's discouraged otherwise. \
            Only files not already in the output CRS will be transformed. \
            New files will be written and so any transform will not affect the origin file. \
            The transformations are done at a later stage.");
            ui.label("Choose new CRS by EPSG code:");
            ui.horizontal(|ui| {
                ui.label("EPSG: ");
                ui.add(egui_autocomplete::AutoCompleteTextEdit::new(
                    &mut self.gui_variables.output_crs_string,
                    crate::epsg_list::EPSG_LIST).highlight_matches(true).max_suggestions(10)
                    .set_text_edit_properties(|t| {
                        t.char_limit(5)
                    }));
            });

            if let Ok(code) = self.gui_variables.output_crs_string.parse::<u16>() {
                if let Some(def) = crs_definitions::from_code(code) {
                    transform_crs = Some(code);
                    ui.label(def.wkt);
                } else {
                    ui.label("Invalid EPSG code");
                }
            } else {
                ui.label("Could not parse EPSG code");
            }
            ui.separator();
            egui::Sides::new().show(
                ui,
                |_ui| {},
                |ui| {
                if ui.button(format!("Majority Vote (EPSG: {majority_epsg})")).clicked() {
                    self.gui_variables.output_epsg = Some(majority_epsg);
                    self.open_modal = OmapModal::None;
                    self.on_frontend_task(FrontEndTask::TaskComplete(TaskDone::OutputCrs));
                }
                if ui.add_enabled(transform_crs.is_some(), egui::Button::new("Select the given CRS")).clicked() {
                    self.gui_variables.output_epsg = transform_crs;
                    self.open_modal = OmapModal::None;
                    self.on_frontend_task(FrontEndTask::TaskComplete(TaskDone::OutputCrs));
                }
            });
        });
    }

    pub fn multiple_graph_components_modal(&mut self, ctx: &egui::Context) {
        let mgc_modal = Modal::new(egui::Id::new("multiple graph parts"));
        mgc_modal.show(ctx, |ui| {
            ui.heading("Multiple Graph Components Detected");
            ui.separator();
            if self.gui_variables.connected_components.len() > 9 {
                ui.label(
                    "The Lidar neighbour graph forms too many components (more than 9).\
                \nPlease start over.",
                );
            } else {
                ui.label(
                    "Multiple graph components have been detected in the lidar neighbour graph. \
                Only the largest will be kept.",
                );
                ui.vertical_centered(|ui| {
                    if ui.button("Show components").clicked() {
                        self.open_modal = OmapModal::None;
                        self.on_frontend_task(FrontEndTask::DelegateTask(Task::ShowComponents));
                    }
                });
            }
            ui.separator();
            egui::Sides::new().show(
                ui,
                |_ui| {},
                |ui| {
                    if ui.button("Start over").clicked() {
                        self.open_modal = OmapModal::None;
                        self.on_frontend_task(FrontEndTask::DelegateTask(Task::Reset));
                    };
                    if ui
                        .add_enabled(
                            self.gui_variables.connected_components.len() < 9,
                            egui::Button::new("Drop all files not in the largest component"),
                        )
                        .clicked()
                    {
                        self.open_modal = OmapModal::None;
                        self.on_frontend_task(FrontEndTask::DelegateTask(Task::DropComponents));
                    }
                },
            );
        });
    }

    pub fn confirm_make_map_modal(&mut self, ctx: &egui::Context) {
        let continue_dialog = Modal::new(egui::Id::new("Make map?"));
        continue_dialog.show(ctx, |ui| {
            ui.heading("Continue to map generation");
            ui.separator();
            ui.label("The next step is the map generation, which may take a little while. There is no way of going back once the generation starts. \
            Are you happy with your parameter settings and want to continue?");
            ui.separator();
            egui::Sides::new().show(
                ui,
                |_ui| {},
                |ui| {
                if ui.button("No, adjust some more!").clicked() {
                    self.open_modal = OmapModal::None;
                };
                if ui.button("Yes, let's make that map!").clicked() {
                    self.open_modal = OmapModal::None;
                    self.on_frontend_task(FrontEndTask::NextState);
                }
            });
        });
    }

    pub fn confirm_start_over_modal(&mut self, ctx: &egui::Context) {
        let start_over_dialog = Modal::new(egui::Id::new("Start Over?"));
        start_over_dialog.show(ctx, |ui| {
            ui.heading("Are you sure?");
            ui.separator();
            ui.label("Do you want to start over?");
            ui.separator();
            egui::Sides::new().show(
                ui,
                |_ui| {},
                |ui| {
                    if ui.button("No, continue!").clicked() {
                        self.open_modal = OmapModal::None;
                    };
                    if ui.button("Yes, start over!").clicked() {
                        self.open_modal = OmapModal::None;
                        self.on_frontend_task(FrontEndTask::DelegateTask(Task::Reset));
                    }
                },
            );
        });
    }

    pub fn error_modal(&mut self, ctx: &egui::Context, cause: String) {
        let error_dialog = Modal::new(egui::Id::new("Error, start over"));
        error_dialog.show(ctx, |ui| {
            ui.heading("An error ocurred");
            ui.separator();
            ui.label(cause);
            ui.separator();
            egui::Sides::new().show(
                ui,
                |_ui| {},
                |ui| {
                    if ui.button("Ok!").clicked() {
                        self.open_modal = OmapModal::None;
                    };
                },
            );
        });
    }

    pub fn waiver_modal(&mut self, ctx: &egui::Context) {
        let waiver_modal = Modal::new(egui::Id::new("Error, start over"));
        waiver_modal.show(ctx, |ui| {
            ui.heading("User's liability waiver");
            ui.separator();
            ui.label(
                "Possession of a map does not grant you permission to access the land on the map. \
            Please check with the rules for your area or the landowners. \n\
            The auto-generated map does not depict what is private and what is open to the public. \
            Do the necessary preperations so you do not accidentally trespass.",
            );
            ui.separator();
            egui::Sides::new().show(
                ui,
                |_ui| {},
                |ui| {
                    if ui.button("Ok").clicked() {
                        self.open_modal = OmapModal::None;
                    };
                },
            );
        });
    }
}
