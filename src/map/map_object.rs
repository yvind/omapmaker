use crate::geometry::Rectangle;
use std::{fs::File, io::BufWriter};

pub trait MapObject: 'static + Sync + Send {
    fn add_tag(&mut self, k: &str, v: &str);

    fn add_auto_tag(&mut self) {
        self.add_tag("generator", "laz2omap");
    }

    fn write_to_map(&self, f: &mut BufWriter<File>, as_bezier: bool);

    fn write_coords(&self, f: &mut BufWriter<File>, as_bezier: bool);

    fn write_tags(&self, f: &mut BufWriter<File>);
}
