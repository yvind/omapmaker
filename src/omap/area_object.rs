use super::{MapObject, Symbol, Tag};
use crate::geometry::Polygon;
use std::{
    fs::File,
    io::{BufWriter, Write},
};

pub struct AreaObject {
    symbol: Symbol,
    coordinates: Polygon,
    tags: Vec<Tag>,
}

impl AreaObject {
    pub fn from_polygon(polygon: Polygon, symbol: Symbol) -> Self {
        Self {
            symbol: symbol,
            coordinates: polygon,
            tags: vec![],
        }
    }
}

impl MapObject for AreaObject {
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
        let mut num_coords = self.coordinates.boundary.len();
        let boundary_length = num_coords;

        for hole in self.coordinates.holes {
            num_coords += hole.len();
        }

        f.write(format!("<coords count=\"{}\">", num_coords).as_bytes());

        for (i, coord) in self.coordinates.boundary.vertices.iter().enumerate() {
            let c = coord.to_map_coordinates().unwrap();

            if i == boundary_length - 1 {
                f.write(format!("{} {} 18;", c.0, c.1).as_bytes());
            } else {
                f.write(format!("{} {};", c.0, c.1).as_bytes());
            }
        }
        for hole in self.coordinates.holes {
            let hole_length = hole.len();

            for (i, coord) in hole.vertices.iter().enumerate() {
                let c = coord.to_map_coordinates().unwrap();

                if i == hole_length - 1 {
                    f.write(format!("{} {} 18;", c.0, c.1).as_bytes());
                } else {
                    f.write(format!("{} {};", c.0, c.1).as_bytes());
                }
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
