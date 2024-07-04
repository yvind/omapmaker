use super::{MapObject, Symbol, Tag};
use crate::geometry::Line;

use std::{
    fs::File,
    io::{BufWriter, Write},
};

pub struct LineObject {
    symbol: Symbol,
    coordinates: Line,
    tags: Vec<Tag>,
}

impl LineObject {
    pub fn from_line(line: Line, symbol: Symbol) -> Self {
        Self {
            symbol: symbol,
            coordinates: line,
            tags: vec![],
        }
    }
}

impl MapObject for LineObject {
    fn add_tag(&mut self, k: &str, v: &str) {
        self.tags.push(Tag::new(k, v));
    }

    fn write_to_map(&self, f: &mut BufWriter<File>) {
        f.write(format!("<object type=\"1\" symbol=\"{}\">", self.symbol).as_bytes())
            .expect("Could not write to map file");
        self.write_tags(f);
        self.write_coords(f);
        f.write(b"</object>\n")
            .expect("Could not write to map file");
    }

    fn write_coords(&self, f: &mut BufWriter<File>) {
        let num_coords = self.coordinates.len();

        f.write(format!("<coords count=\"{num_coords}\">").as_bytes())
            .expect("Could not write to map file");
        for (i, coord) in self.coordinates.vertices.iter().enumerate() {
            let c = coord.to_map_coordinates().unwrap();

            if i == num_coords - 1 && self.coordinates.is_closed() {
                f.write(format!("{} {} 18;", c.0, c.1).as_bytes())
                    .expect("Could not write to map file");
            } else {
                f.write(format!("{} {};", c.0, c.1).as_bytes())
                    .expect("Could not write to map file");
            }
        }
        f.write(b"</coords>").expect("Could not write to map file");
    }

    fn write_tags(&self, f: &mut BufWriter<File>) {
        if self.tags.is_empty() {
            return;
        }

        f.write(b"<tags>").expect("Could not write to map file");
        for tag in self.tags.iter() {
            f.write(tag.to_string().as_bytes())
                .expect("Could not write to map file");
        }
        f.write(b"</tags>").expect("Could not write to map file");
    }
}
