use std::sync::mpsc;

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

    pub fn new(sender: mpsc::Sender<T>, receiver: mpsc::Receiver<S>) -> Self {
        OmapComms { sender, receiver }
    }
}
