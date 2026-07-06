use std::sync::mpsc;

use eframe::egui;

use super::messages::{BackendTask, FrontendTask};

#[derive(Clone)]
pub struct FrontendSender {
    sender: mpsc::Sender<FrontendTask>,
    ctx: egui::Context,
}

impl FrontendSender {
    pub fn send(&self, task: FrontendTask) -> Result<(), mpsc::SendError<FrontendTask>> {
        let result = self.sender.send(task);
        self.ctx.request_repaint();
        result
    }
}

// Multiple Producer Single Consumer, i.e. a sender is cloneable but the receiver not
// A dual message passing channel for the frontend and backend
// OmapComms are created in pairs, one for the backend and one for the frontend
// so they can both both send and receive FrontendTask/BackendTask
pub struct OmapComms<T, S> {
    sender: mpsc::Sender<T>,
    receiver: mpsc::Receiver<S>,
}

impl<T, S> OmapComms<T, S> {
    pub fn send(&self, t: T) -> Result<(), mpsc::SendError<T>> {
        self.sender.send(t)
    }

    pub fn try_recv(&self) -> Result<S, mpsc::TryRecvError> {
        self.receiver.try_recv()
    }

    pub fn recv(&self) -> Result<S, mpsc::RecvError> {
        self.receiver.recv()
    }
}

impl OmapComms<FrontendTask, BackendTask> {
    pub fn frontend_sender(&self, ctx: &egui::Context) -> FrontendSender {
        FrontendSender {
            sender: self.sender.clone(),
            ctx: ctx.clone(),
        }
    }
}

// the generics does not really matter here
impl OmapComms<FrontendTask, BackendTask> {
    pub fn new() -> (
        OmapComms<BackendTask, FrontendTask>,
        OmapComms<FrontendTask, BackendTask>,
    ) {
        let (to_frontend, from_backend) = mpsc::channel();
        let (to_backend, from_frontend) = mpsc::channel();

        let backend_comms = OmapComms {
            sender: to_frontend,
            receiver: from_frontend,
        };
        let frontend_comms = OmapComms {
            sender: to_backend,
            receiver: from_backend,
        };

        (frontend_comms, backend_comms)
    }
}
