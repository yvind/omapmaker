use std::sync::{Arc, Mutex};

use eframe::egui::TextBuffer;
use log::Log;

// must be used with a monospace font for the progress bar to look ok
#[derive(Clone)]
pub struct TerminalLike {
    string_logger: StringLogger,
    progress_bar: Option<ProgressBar>,
}

impl Default for TerminalLike {
    fn default() -> Self {
        Self {
            string_logger: StringLogger::new("PROGRESS LOG\n", log::Level::Warn),
            progress_bar: Default::default(),
        }
    }
}

impl<'a> TerminalLike {
    pub fn println(&mut self, s: impl Into<&'a str>) {
        let mut log = self.string_logger.log_output.lock().unwrap();

        // needs to check for active progress bar

        log.push('\n');
        log.push_str(s.into());
    }

    pub fn inc_progress_bar(&mut self, delta: f32) {
        if let Some(pb) = &mut self.progress_bar {
            let mut log = self.string_logger.log_output.lock().unwrap();

            let len = log.len();

            log.delete_char_range(pb.start_char_pos..len);
            pb.inc(delta);
            pb.draw_to_string(&mut log);
        }
    }

    pub fn finish_progress_bar(&mut self) {
        if let Some(pb) = &mut self.progress_bar {
            let mut log = self.string_logger.log_output.lock().unwrap();

            let len = log.len();

            log.delete_char_range(pb.start_char_pos..len);
            pb.inc(1.);
            pb.draw_to_string(&mut log);
            self.progress_bar = None;
            log.push('\n'); // we want space after the progress bar
        }
    }

    pub fn start_progress_bar(&mut self, width: u32) {
        let mut log = self.string_logger.log_output.lock().unwrap();

        log.push_str("\n\n");

        let pb = ProgressBar::new(log.len(), width);
        pb.draw_to_string(&mut log);

        self.progress_bar = Some(pb);
    }
}

impl TextBuffer for TerminalLike {
    fn is_mutable(&self) -> bool {
        true
    }

    fn as_str(&self) -> &str {
        self.string_logger.log_output.as_ref()
    }

    fn insert_text(&mut self, text: &str, char_index: usize) -> usize {
        let mut log = self.string_logger.log_output.lock().unwrap();
        log.insert_text(text, char_index)
    }

    fn delete_char_range(&mut self, char_range: std::ops::Range<usize>) {
        let mut log = self.string_logger.log_output.lock().unwrap();
        log.delete_char_range(char_range);
    }

    fn clear(&mut self) {
        let mut log = self.string_logger.log_output.lock().unwrap();
        log.clear();
        self.progress_bar = None;
    }
}

#[derive(Debug, Clone)]
struct StringLogger {
    log_output: Arc<Mutex<String>>,
    level_filter: log::Level,
}

impl StringLogger {
    pub fn new(title: &str, level_filter: log::Level) -> StringLogger {
        let log_output = Arc::new(Mutex::new(String::from(title)));
        StringLogger {
            log_output,
            level_filter,
        }
    }
}

impl Log for StringLogger {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        metadata.level() <= self.level_filter
    }

    fn log(&self, record: &log::Record) {
        if self.enabled(record.metadata()) {
            let mut log_string = self.log_output.lock().unwrap();
            log_string.push_str(&format!("[{}] {}\n", record.level(), record.args()));
        }
    }

    fn flush(&self) {}
}

#[derive(Clone)]
struct ProgressBar {
    pub(crate) start_char_pos: usize,
    progress: f32,
    total_char_width: u32,
}

impl ProgressBar {
    fn new(pos: usize, width: u32) -> ProgressBar {
        ProgressBar {
            start_char_pos: pos,
            progress: 0.,
            total_char_width: width,
        }
    }

    fn draw_to_string(&self, str: &mut String) {
        str.push_str("\t[");

        let num_done = (self.total_char_width as f32 * self.progress) as u32;
        let mut rest = self.total_char_width - num_done;

        for _ in 0..num_done {
            str.push('#');
        }
        if rest != 0 {
            str.push('>');
            rest -= 1;
        }
        for _ in 0..rest {
            str.push(' ');
        }
        str.push_str(format!("] {:>3}%", (self.progress * 100.).round() as u32).as_str());
    }

    fn inc(&mut self, delta: f32) {
        // clamp progress between 0 and 1
        self.progress = 1.0_f32.min(self.progress + delta).max(0.0_f32);
    }
}
