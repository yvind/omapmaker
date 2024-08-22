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
            symbol,
            coordinates: polygon,
            tags: vec![],
        }
    }
}

impl MapObject for AreaObject {
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
        let mut num_coords = self.coordinates.boundary.len();
        let boundary_length = num_coords;

        for hole in self.coordinates.holes.iter() {
            num_coords += hole.len();
        }

        f.write(format!("<coords count=\"{}\">", num_coords).as_bytes())
            .expect("Could not write to map file");

        for (i, coord) in self.coordinates.boundary.vertices.iter().enumerate() {
            let c = coord.to_map_coordinates().unwrap();

            if i == boundary_length - 1 {
                f.write(format!("{} {} 18;", c.0, c.1).as_bytes())
                    .expect("Could not write to map file");
            } else {
                f.write(format!("{} {};", c.0, c.1).as_bytes())
                    .expect("Could not write to map file");
            }
        }
        for hole in self.coordinates.holes.iter() {
            let hole_length = hole.len();

            for (i, coord) in hole.vertices.iter().enumerate() {
                let c = coord.to_map_coordinates().unwrap();

                if i == hole_length - 1 {
                    f.write(format!("{} {} 18;", c.0, c.1).as_bytes())
                        .expect("Could not write to map file");
                } else {
                    f.write(format!("{} {};", c.0, c.1).as_bytes())
                        .expect("Could not write to map file");
                }
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
