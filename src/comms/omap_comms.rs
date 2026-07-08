use std::sync::mpsc;

use eframe::egui;

use super::messages::{BackendTask, FrontendTask};

// Multiple Producer Single Consumer, i.e. a sender is cloneable but the receiver not
// A dual message passing channel for the frontend and backend
// OmapComms are created in pairs, one for the backend and one for the frontend
// so they can both both send and receive FrontendTask/BackendTask
pub struct OmapComms<T, S> {
    sender: mpsc::Sender<T>,
    receiver: mpsc::Receiver<S>,
    ctx: egui::Context,
}

impl OmapComms<BackendTask, FrontendTask> {
    pub fn send(&self, t: BackendTask) -> Result<(), mpsc::SendError<BackendTask>> {
        self.sender.send(t)
    }

    pub fn try_recv(&self) -> Result<FrontendTask, mpsc::TryRecvError> {
        self.receiver.try_recv()
    }
}

impl OmapComms<FrontendTask, BackendTask> {
    pub fn sender(&self) -> FrontendSender {
        FrontendSender {
            sender: self.sender.clone(),
            ctx: self.ctx.clone(),
        }
    }

    pub fn send(&self, t: FrontendTask) -> Result<(), mpsc::SendError<FrontendTask>> {
        let result = self.sender.send(t);
        self.ctx.request_repaint();
        result
    }

    pub fn recv(&self) -> Result<BackendTask, mpsc::RecvError> {
        self.receiver.recv()
    }
}

impl OmapComms<FrontendTask, BackendTask> {
    pub fn new(
        ctx: &egui::Context,
    ) -> (
        OmapComms<BackendTask, FrontendTask>,
        OmapComms<FrontendTask, BackendTask>,
    ) {
        let (to_frontend, from_backend) = mpsc::channel();
        let (to_backend, from_frontend) = mpsc::channel();

        let backend_comms = OmapComms {
            sender: to_frontend,
            receiver: from_frontend,
            ctx: ctx.clone(),
        };
        let frontend_comms = OmapComms {
            sender: to_backend,
            receiver: from_backend,
            ctx: ctx.clone(),
        };

        (frontend_comms, backend_comms)
    }
}

#[derive(Clone)]
pub struct FrontendSender {
    sender: mpsc::Sender<FrontendTask>,
    ctx: egui::Context,
}

impl FrontendSender {
    pub fn send(&self, t: FrontendTask) -> Result<(), mpsc::SendError<FrontendTask>> {
        let result = self.sender.send(t);
        self.ctx.request_repaint();
        result
    }
}
