use super::{MapObject, Tag};
use crate::geometry::Contour;

use std::{
    fs::File,
    io::{BufWriter, Write},
};

pub struct LineObject {
    symbol: u16,
    coordinates: Contour,
    tags: Vec<Tag>,
}

impl LineObject {
    pub fn from_line(line: Contour, symbol: u16) -> Self {
        Self {
            symbol: symbol,
            coordinates: line,
            tags: vec![],
        }
    }
}

impl MapObject for LineObject {
    fn add_tag(&self, k: &str, v: &str) {
        self.tags.push(Tag::new(k, v));
    }

    fn write_to_map(&self, f: &BufWriter<File>) {
        f.write(format!("<object type=\"1\" symbol=\"{}\">", self.symbol).as_bytes());
        self.write_tags(f);
        self.write_coords(f);
        f.write(b"</object>\n");
    }

    fn write_coords(&self, f: &BufWriter<File>) {
        let num_coords = self.coordinates.len();

        f.write(format!("<coords count=\"{num_coords}\">").as_bytes());
        for (i, coord) in self.coordinates.vertices.iter().enumerate() {
            let c = coord.to_map_coordinates();

            if i == num_coords - 1 && self.coordinates.is_closed() {
                f.write(format!("{} {} 18;", c.0, c.1).as_bytes());
            } else {
                f.write(format!("{} {};", c.0, c.1).as_bytes());
            }
        }
        f.write(b"</coords>");
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
