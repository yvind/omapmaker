use super::{MapObject, Symbol, Tag};
use crate::geometry::{BezierSegmentType, BezierString, MapCoord, Polygon, Rectangle};
use crate::BEZIER_ERROR;

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

    fn write_polyline(&self, f: &mut BufWriter<File>) {
        let mut num_coords = self.coordinates.exterior().0.len();
        let boundary_length = num_coords;

        for hole in self.coordinates.interiors().iter() {
            num_coords += hole.0.len();
        }

        f.write_all(format!("<coords count=\"{}\">", num_coords).as_bytes())
            .expect("Could not write to map file");

        let mut ext_iter = self.coordinates.exterior().coords();
        let mut i = 0;

        while i < boundary_length - 1 {
            let c = ext_iter.next().unwrap().to_map_coordinates().unwrap();
            f.write_all(format!("{} {};", c.0, c.1).as_bytes())
                .expect("Could not write to map file");
            i += 1;
        }
        let c = ext_iter.next().unwrap().to_map_coordinates().unwrap();
        f.write_all(format!("{} {} 18;", c.0, c.1).as_bytes())
            .expect("Could not write to map file");

        for hole in self.coordinates.interiors().iter() {
            let hole_length = hole.0.len();

            let mut int_iter = hole.coords();
            let mut i = 0;

            while i < hole_length - 1 {
                let c = int_iter.next().unwrap().to_map_coordinates().unwrap();
                f.write_all(format!("{} {};", c.0, c.1).as_bytes())
                    .expect("Could not write to map file");

                i += 1;
            }
            let c = int_iter.next().unwrap().to_map_coordinates().unwrap();
            f.write_all(format!("{} {} 18;", c.0, c.1).as_bytes())
                .expect("Could not write to map file");
        }
        f.write_all(b"</coords>")
            .expect("Could not write to map file");
    }

    fn write_bezier(&self, f: &mut BufWriter<File>) {
        let mut beziers = Vec::with_capacity(self.coordinates.num_rings());
        beziers.push(BezierString::from_polyline(
            self.coordinates.exterior(),
            BEZIER_ERROR,
        ));
        for hole in self.coordinates.interiors() {
            beziers.push(BezierString::from_polyline(hole, BEZIER_ERROR));
        }
        let mut num_coords = 0;
        for b in beziers.iter() {
            num_coords += b.num_points();
        }

        f.write_all(format!("<coords count=\"{num_coords}\">").as_bytes())
            .expect("Could not write to map file");

        for bezier in beziers {
            let num_segments = bezier.0.len();

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

                    f.write_all(format!("{} {}; {} {} 18;", c1.0, c1.1, c2.0, c2.0).as_bytes())
                        .expect("Could not write to map file");
                }
                BezierSegmentType::Bezier => {
                    let c1 = final_segment.0 .0.to_map_coordinates().unwrap();
                    let h1 = final_segment.0 .1.unwrap().to_map_coordinates().unwrap();
                    let h2 = final_segment.0 .2.unwrap().to_map_coordinates().unwrap();
                    let c2 = final_segment.0 .3.to_map_coordinates().unwrap();

                    f.write_all(
                        format!(
                            "{} {} 1; {} {}; {} {}; {} {} 18;",
                            c1.0, c1.1, h1.0, h1.1, h2.0, h2.1, c2.0, c2.1
                        )
                        .as_bytes(),
                    )
                    .expect("Could not write to map file");
                }
            }
        }
    }
}

impl MapObject for AreaObject {
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
