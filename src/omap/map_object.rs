use std::{fs::File, io::BufWriter};

pub trait MapObject {
    fn add_tag(&self, k: &str, v: &str);

    fn add_auto_tag(&self) {
        self.add_tag("generator", "laz2omap");
    }

    fn write_to_map(&self, f: &BufWriter<File>);

    fn write_coords(&self, f: &BufWriter<File>);

    fn write_tags(&self, f: &BufWriter<File>);
}
