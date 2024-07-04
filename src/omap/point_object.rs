use super::{MapObject, Symbol, Tag};
use crate::geometry::Point2D;
use std::{
    fs::File,
    io::{BufWriter, Write},
};

pub struct PointObject {
    symbol: Symbol,
    coordinates: Point2D,
    rotation: f64,
    tags: Vec<Tag>,
}

impl PointObject {
    fn from_point(coordinates: Point2D, symbol: Symbol, rotation: f64) -> Self {
        Self {
            symbol,
            coordinates,
            rotation,
            tags: vec![],
        }
    }
}

impl MapObject for PointObject {
    fn add_tag(&self, k: &str, v: &str) {
        self.tags.push(Tag::new(k, v));
    }

    fn write_to_map(&self, f: &BufWriter<File>) {
        f.write(
            format!(
                "<object type=\"0\" symbol=\"{}\" rotation=\"{}\">",
                self.symbol, self.rotation
            )
            .as_bytes(),
        );
        self.write_tags(f);
        self.write_coords(f);
        f.write(b"</object>\n");
    }

    fn write_coords(&self, f: &BufWriter<File>) {
        let c = self.coordinates.to_map_coordinates().unwrap();
        f.write(format!("<coords count=\"1\">{} {};</coords>", c.0, c.1).as_bytes());
    }

    fn write_tags(&self, f: &BufWriter<File>) {
        if self.tags.is_empty() {
            return;
        }

        f.write(b"<tags>");
        for tag in self.tags {
            f.write(tag.to_string().as_bytes());
        }
        f.write(b"</tags>");
    }
}
