use std::{
    path::PathBuf,
    sync::mpsc::{self, Receiver, TryRecvError},
};

use eframe::egui;

pub(super) enum FilePickerResult {
    Lidar(Option<Vec<PathBuf>>),
    SaveLocation(Option<PathBuf>),
}

#[derive(Default)]
pub(super) struct FilePicker {
    receiver: Option<Receiver<FilePickerResult>>,
    error: Option<String>,
}

impl FilePicker {
    pub(super) fn is_open(&self) -> bool {
        self.receiver.is_some()
    }

    pub(super) fn error(&self) -> Option<&str> {
        self.error.as_deref()
    }

    pub(super) fn pick_lidar(&mut self, ctx: &egui::Context) {
        self.start(ctx, "lidar-file-picker", || {
            FilePickerResult::Lidar(
                rfd::FileDialog::new()
                    .add_filter("Lidar Files (*.las, *.laz)", &["las", "laz"])
                    .pick_files(),
            )
        });
    }

    pub(super) fn pick_save_location(&mut self, ctx: &egui::Context) {
        self.start(ctx, "omap-save-file-picker", || {
            FilePickerResult::SaveLocation(
                rfd::FileDialog::new()
                    .add_filter("OpenOrienteering Mapper (*.omap)", &["omap"])
                    .save_file(),
            )
        });
    }

    pub(super) fn try_take_result(&mut self) -> Option<FilePickerResult> {
        let result = match self.receiver.as_ref()?.try_recv() {
            Ok(result) => Some(result),
            Err(TryRecvError::Empty) => return None,
            Err(TryRecvError::Disconnected) => {
                self.error = Some("The file picker closed unexpectedly.".to_owned());
                None
            }
        };

        self.receiver = None;
        result
    }

    fn start(
        &mut self,
        ctx: &egui::Context,
        thread_name: &str,
        open_dialog: impl FnOnce() -> FilePickerResult + Send + 'static,
    ) {
        if self.is_open() {
            return;
        }

        self.error = None;
        let (sender, receiver) = mpsc::channel();
        let repaint_ctx = ctx.clone();
        let spawn_result = std::thread::Builder::new()
            .name(thread_name.to_owned())
            .spawn(move || {
                let _repaint_on_exit = RepaintOnDrop(repaint_ctx);
                let _ = sender.send(open_dialog());
            });

        match spawn_result {
            Ok(_) => self.receiver = Some(receiver),
            Err(error) => {
                self.error = Some(format!("Could not open the file picker: {error}"));
            }
        }
    }
}

struct RepaintOnDrop(egui::Context);

impl Drop for RepaintOnDrop {
    fn drop(&mut self) {
        self.0.request_repaint();
    }
}
