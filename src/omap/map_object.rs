use std::{fs::File, io::BufWriter};

pub trait MapObject: 'static {
    fn add_tag(&mut self, k: &str, v: &str);

    fn add_auto_tag(&mut self) {
        self.add_tag("generator", "laz2omap");
    }

    fn write_to_map(&self, f: &mut BufWriter<File>);

    fn write_coords(&self, f: &mut BufWriter<File>);

    fn write_tags(&self, f: &mut BufWriter<File>);
}
