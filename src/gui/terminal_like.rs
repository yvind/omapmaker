use eframe::egui::TextBuffer;

// must be used with a monospace font for the progress bar to look ok
#[derive(Clone)]
pub struct TerminalLike {
    progress_bar: Option<ProgressBar>,
    string: String,
}

impl Default for TerminalLike {
    fn default() -> Self {
        let str = "PROGRESS LOG\n\n";

        Self {
            progress_bar: Default::default(),
            string: String::from(str),
        }
    }
}

impl<'a> TerminalLike {
    pub fn println(&mut self, s: impl Into<&'a str>) {
        if let Some(pb) = &self.progress_bar {
            let len = self.string.len();
            self.string.delete_char_range(pb.start_char_pos..len);
            self.string.push_str(s.into());
            pb.draw_to_string(&mut self.string);
        } else {
            self.string.push_str(s.into());
        }
        self.string.push('\n');
    }

    pub fn inc_progress_bar(&mut self, delta: f32) {
        if let Some(pb) = &mut self.progress_bar {
            self.string
                .delete_char_range(pb.start_char_pos..self.string.len());
            pb.inc(delta);
            pb.draw_to_string(&mut self.string);
            self.string.push('\n');
        }
    }

    pub fn finish_progress_bar(&mut self) {
        if let Some(pb) = &mut self.progress_bar {
            self.string
                .delete_char_range(pb.start_char_pos..self.string.len());
            pb.inc(1.);
            pb.draw_to_string(&mut self.string);
            self.progress_bar = None;
            self.string.push('\n'); // we want space after the progress bar
        }
    }

    pub fn start_progress_bar(&mut self, width: u32) {
        self.string.push('\n');

        let pb = ProgressBar::new(self.string.len(), width);
        pb.draw_to_string(&mut self.string);

        self.progress_bar = Some(pb);
        self.string.push('\n');
    }
}

impl TextBuffer for TerminalLike {
    fn is_mutable(&self) -> bool {
        true
    }

    fn as_str(&self) -> &str {
        self.string.as_str()
    }

    fn insert_text(&mut self, text: &str, char_index: usize) -> usize {
        self.string.insert_text(text, char_index)
    }

    fn delete_char_range(&mut self, char_range: std::ops::Range<usize>) {
        self.string.delete_char_range(char_range);
    }

    fn clear(&mut self) {
        self.string.clear();
        self.progress_bar = None;
    }

    fn type_id(&self) -> std::any::TypeId {
        self.string.type_id()
    }
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
        self.progress = (self.progress + delta).clamp(0., 1.);
    }
}
