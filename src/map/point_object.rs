#![allow(dead_code)]

use super::{MapObject, Symbol, Tag};
use crate::geometry::{MapCoord, Point, Rectangle};
use std::{
    fs::File,
    io::{BufWriter, Write},
};

pub struct PointObject {
    symbol: Symbol,
    coordinates: Point,
    rotation: f64,
    tags: Vec<Tag>,
}

impl PointObject {
    fn from_point(coordinates: Point, symbol: Symbol, rotation: f64) -> Self {
        Self {
            symbol,
            coordinates,
            rotation,
            tags: vec![],
        }
    }
}

impl MapObject for PointObject {
    fn bounding_box(&self) -> Rectangle {
        Rectangle::new(self.coordinates.0, self.coordinates.0)
    }

    fn add_tag(&mut self, k: &str, v: &str) {
        self.tags.push(Tag::new(k, v));
    }

    fn write_to_map(&self, f: &mut BufWriter<File>) {
        f.write_all(
            format!(
                "<object type=\"0\" symbol=\"{}\" rotation=\"{}\">",
                self.symbol, self.rotation
            )
            .as_bytes(),
        )
        .expect("Could not write to map file");
        self.write_tags(f);
        self.write_coords(f);
        f.write_all(b"</object>\n")
            .expect("Could not write to map file");
    }

    fn write_coords(&self, f: &mut BufWriter<File>) {
        let c = self.coordinates.0.to_map_coordinates().unwrap();
        f.write_all(format!("<coords count=\"1\">{} {};</coords>", c.0, c.1).as_bytes())
            .expect("Could not write to map file");
    }

    fn write_tags(&self, f: &mut BufWriter<File>) {
        if self.tags.is_empty() {
            return;
        }

        f.write_all(b"<tags>").expect("Could not write to map file");
        for tag in self.tags.iter() {
            f.write_all(tag.to_string().as_bytes())
                .expect("Could not write to map file");
        }
        f.write_all(b"</tags>")
            .expect("Could not write to map file");
    }
}
