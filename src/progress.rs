#[derive(Clone, Copy, Debug)]
pub(crate) enum ProgressUpdate {
    Start,
    Advance(f32),
    Finish,
}

pub(crate) trait ProgressReporter: Sync {
    fn log(&self, message: String);
    fn progress(&self, update: ProgressUpdate);
    fn error(&self, message: String);
}
