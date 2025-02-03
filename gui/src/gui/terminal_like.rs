use eframe::egui::TextBuffer;

#[derive(Clone)]
pub struct TerminalLike {
    string: String,
    progress_bar: Option<ProgressBar>,
}

impl Default for TerminalLike {
    fn default() -> Self {
        Self {
            string: String::from("PROGRESS LOG\n"),
            progress_bar: Default::default(),
        }
    }
}

impl<'a> TerminalLike {
    pub fn print(&mut self, s: impl Into<&'a str>) {
        self.string.push_str(s.into());
    }

    pub fn println(&mut self, s: impl Into<&'a str>) {
        self.string.push('\n');
        self.string.push_str(s.into());
    }

    pub fn inc_progress_bar(&mut self, delta: f32) {
        if let Some(pb) = &mut self.progress_bar {
            self.string
                .delete_char_range(pb.start_char_pos..self.string.len());
            pb.inc(delta);
            pb.draw_to_string(&mut self.string);
        }
    }

    pub fn finish_progress_bar(&mut self) {
        if let Some(pb) = &mut self.progress_bar {
            self.string
                .delete_char_range(pb.start_char_pos..self.string.len());
            pb.inc(1.);
            pb.draw_to_string(&mut self.string);
            self.progress_bar = None;
        }
    }

    pub fn start_progress_bar(&mut self, width: u32) {
        self.string.push('\n');
        self.progress_bar = Some(ProgressBar::new(self.string.len(), width));
    }

    pub fn progress_bar_is_active(&self) -> bool {
        self.progress_bar.is_some()
    }
}

impl TextBuffer for TerminalLike {
    fn is_mutable(&self) -> bool {
        true
    }

    fn as_str(&self) -> &str {
        self.string.as_ref()
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
        self.progress = 1.0_f32.min(self.progress + delta);
    }
}
