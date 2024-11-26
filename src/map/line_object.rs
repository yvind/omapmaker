use super::{MapObject, Symbol, Tag};
use crate::geometry::{BezierSegmentType, BezierString, LineString, MapCoord, Rectangle};
use crate::BEZIER_ERROR;

use std::{
    fs::File,
    io::{BufWriter, Write},
};

pub struct LineObject {
    symbol: Symbol,
    coordinates: LineString,
    tags: Vec<Tag>,
}

impl LineObject {
    pub fn from_line_string(line: LineString, symbol: Symbol) -> Self {
        Self {
            symbol,
            coordinates: line,
            tags: vec![],
        }
    }

    fn write_polyline(&self, f: &mut BufWriter<File>) {
        let num_coords = self.coordinates.0.len();

        f.write_all(format!("<coords count=\"{num_coords}\">").as_bytes())
            .expect("Could not write to map file");

        let mut coord_iter = self.coordinates.coords();
        let mut i = 0;
        while i < num_coords - 1 {
            let c = coord_iter.next().unwrap().to_map_coordinates().unwrap();
            f.write_all(format!("{} {};", c.0, c.1).as_bytes())
                .expect("Could not write to map file");

            i += 1;
        }
        let c = coord_iter.next().unwrap().to_map_coordinates().unwrap();
        if self.coordinates.is_closed() {
            f.write_all(format!("{} {} 18;", c.0, c.1).as_bytes())
                .expect("Could not write to map file");
        } else {
            f.write_all(format!("{} {};", c.0, c.1).as_bytes())
                .expect("Could not write to map file");
        }

        f.write_all(b"</coords>")
            .expect("Could not write to map file");
    }

    fn write_bezier(&self, f: &mut BufWriter<File>) {
        let bezier = BezierString::from_polyline(&self.coordinates, BEZIER_ERROR);

        let num_coords = bezier.num_points();
        let num_segments = bezier.0.len();
        f.write_all(format!("<coords count=\"{num_coords}\">").as_bytes())
            .expect("Could not write to map file");

        let mut bez_iterator = bezier.0.into_iter();
        let mut i = 0;
        while i < num_segments - 1 {
            let segment = bez_iterator.next().unwrap();
            match segment.line_type() {
                BezierSegmentType::Polyline => {
                    let c = segment.0 .0.to_map_coordinates().unwrap();

                    f.write_all(format!("{} {};", c.0, c.1).as_bytes())
                        .expect("Could not write to map file");
                }
                BezierSegmentType::Bezier => {
                    let c = segment.0 .0.to_map_coordinates().unwrap();
                    let h1 = segment.0 .1.unwrap().to_map_coordinates().unwrap();
                    let h2 = segment.0 .2.unwrap().to_map_coordinates().unwrap();
                    f.write_all(
                        format!("{} {} 1; {} {}; {} {}", c.0, c.1, h1.0, h1.1, h2.0, h2.1)
                            .as_bytes(),
                    )
                    .expect("Could not write to map file");
                }
            }
            i += 1;
        }
        // finish with the last segment of the curve
        let final_segment = bez_iterator.next().unwrap();
        match final_segment.line_type() {
            BezierSegmentType::Polyline => {
                let c1 = final_segment.0 .0.to_map_coordinates().unwrap();
                let c2 = final_segment.0 .3.to_map_coordinates().unwrap();

                if self.coordinates.is_closed() {
                    f.write_all(format!("{} {}; {} {} 18;", c1.0, c1.1, c2.0, c2.0).as_bytes())
                        .expect("Could not write to map file");
                } else {
                    f.write_all(format!("{} {}; {} {};", c1.0, c1.1, c2.0, c2.0).as_bytes())
                        .expect("Could not write to map file");
                }
            }
            BezierSegmentType::Bezier => {
                let c1 = final_segment.0 .0.to_map_coordinates().unwrap();
                let h1 = final_segment.0 .1.unwrap().to_map_coordinates().unwrap();
                let h2 = final_segment.0 .2.unwrap().to_map_coordinates().unwrap();
                let c2 = final_segment.0 .3.to_map_coordinates().unwrap();

                if self.coordinates.is_closed() {
                    f.write_all(
                        format!(
                            "{} {} 1; {} {}; {} {}; {} {} 18;",
                            c1.0, c1.1, h1.0, h1.1, h2.0, h2.1, c2.0, c2.1
                        )
                        .as_bytes(),
                    )
                    .expect("Could not write to map file");
                } else {
                    f.write_all(
                        format!(
                            "{} {} 1; {} {}; {} {}; {} {};",
                            c1.0, c1.1, h1.0, h1.1, h2.0, h2.1, c2.0, c2.1
                        )
                        .as_bytes(),
                    )
                    .expect("Could not write to map file");
                }
            }
        }

        f.write_all(b"</coords>")
            .expect("Could not write to map file");
    }
}

impl MapObject for LineObject {
    fn add_tag(&mut self, k: &str, v: &str) {
        self.tags.push(Tag::new(k, v));
    }

    fn write_to_map(&self, f: &mut BufWriter<File>, as_bezier: bool) {
        f.write_all(format!("<object type=\"1\" symbol=\"{}\">", self.symbol).as_bytes())
            .expect("Could not write to map file");
        self.write_tags(f);
        self.write_coords(f, as_bezier);
        f.write_all(b"</object>\n")
            .expect("Could not write to map file");
    }

    fn write_coords(&self, f: &mut BufWriter<File>, as_bezier: bool) {
        if as_bezier {
            self.write_bezier(f);
        } else {
            self.write_polyline(f);
        }
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
