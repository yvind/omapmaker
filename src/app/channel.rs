use std::sync::mpsc;

use eframe::egui;

use super::protocol::{AppEvent, WorkerCommand};
use crate::progress::{ProgressReporter, ProgressUpdate};

// Multiple Producer Single Consumer, i.e. a sender is cloneable but the receiver not
// A dual message passing channel for the frontend and backend
// OmapComms are created in pairs, one for the backend and one for the frontend
// so they can both both send and receive AppEvent/WorkerCommand
pub struct OmapComms<T, S> {
    sender: mpsc::Sender<T>,
    receiver: mpsc::Receiver<S>,
    ctx: egui::Context,
}

impl OmapComms<WorkerCommand, AppEvent> {
    pub fn send(&self, t: WorkerCommand) -> Result<(), mpsc::SendError<WorkerCommand>> {
        self.sender.send(t)
    }

    pub fn try_recv(&self) -> Result<AppEvent, mpsc::TryRecvError> {
        self.receiver.try_recv()
    }
}

impl OmapComms<AppEvent, WorkerCommand> {
    pub fn sender(&self) -> AppSender {
        AppSender {
            sender: self.sender.clone(),
            ctx: self.ctx.clone(),
        }
    }

    pub fn send(&self, t: AppEvent) -> Result<(), mpsc::SendError<AppEvent>> {
        let result = self.sender.send(t);
        self.ctx.request_repaint();
        result
    }

    pub fn recv(&self) -> Result<WorkerCommand, mpsc::RecvError> {
        self.receiver.recv()
    }
}

impl OmapComms<AppEvent, WorkerCommand> {
    pub fn new(
        ctx: &egui::Context,
    ) -> (
        OmapComms<WorkerCommand, AppEvent>,
        OmapComms<AppEvent, WorkerCommand>,
    ) {
        let (to_frontend, from_backend) = mpsc::channel();
        let (to_backend, from_frontend) = mpsc::channel();

        let worker_comms = OmapComms {
            sender: to_frontend,
            receiver: from_frontend,
            ctx: ctx.clone(),
        };
        let app_comms = OmapComms {
            sender: to_backend,
            receiver: from_backend,
            ctx: ctx.clone(),
        };

        (app_comms, worker_comms)
    }
}

#[derive(Clone)]
pub struct AppSender {
    sender: mpsc::Sender<AppEvent>,
    ctx: egui::Context,
}

impl AppSender {
    pub fn send(&self, t: AppEvent) -> Result<(), mpsc::SendError<AppEvent>> {
        let result = self.sender.send(t);
        self.ctx.request_repaint();
        result
    }
}

impl ProgressReporter for AppSender {
    fn log(&self, message: String) {
        let _ = self.send(AppEvent::Log(message));
    }

    fn progress(&self, update: ProgressUpdate) {
        let progress = match update {
            ProgressUpdate::Start => super::protocol::ProgressBar::Start,
            ProgressUpdate::Advance(amount) => super::protocol::ProgressBar::Inc(amount),
            ProgressUpdate::Finish => super::protocol::ProgressBar::Finish,
        };
        let _ = self.send(AppEvent::ProgressBar(progress));
    }

    fn error(&self, message: String) {
        let _ = self.send(AppEvent::Error(message, true));
    }
}
