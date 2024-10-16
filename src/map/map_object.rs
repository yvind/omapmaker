#![allow(dead_code)]

use std::{fs::File, io::BufWriter};

use crate::geometry::{Point2D, Rectangle};

pub trait MapObject: 'static + Sync + Send {
    fn add_tag(&mut self, k: &str, v: &str);

    fn add_auto_tag(&mut self) {
        self.add_tag("generator", "laz2omap");
    }

    fn bounding_box(&self) -> Rectangle;

    fn write_to_map(&self, f: &mut BufWriter<File>);

    fn write_coords(&self, f: &mut BufWriter<File>);

    fn write_tags(&self, f: &mut BufWriter<File>);

    //fn cut(self, bounds: &Rectangle) -> Vec<Box<dyn MapObject>>;
}
