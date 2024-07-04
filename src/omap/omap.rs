#![allow(dead_code)]

use super::MapObject;
use crate::geometry::Point2D;

use std::io::{BufWriter, Write};
use std::{fs::File, path::PathBuf};

pub struct Omap {
    filepath: PathBuf,
    ref_point: Point2D,
    objects: Vec<Box<dyn MapObject>>,
}

impl Omap {
    pub fn new(filename: &str, output_dir: &str, georef_point: Point2D) -> Self {
        Omap {
            filepath: PathBuf::from(format!("{}/{}.omap", output_dir, filename)),
            ref_point: georef_point,
            objects: vec![],
        }
    }

    pub fn add_object<T: MapObject>(&mut self, obj: T) {
        self.objects.push(Box::new(obj));
    }

    pub fn write_to_file(&self) {
        let f = File::create(&self.filepath).expect("Unable to create omap file");
        let mut f = BufWriter::new(f);

        self.write_header(&mut f);
        self.write_colors_symbols(&mut f);
        self.write_objects(&mut f);
        self.write_end_of_file(&mut f);
    }

    fn write_header(&self, f: &mut BufWriter<File>) {
        f.write(b"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<map xmlns=\"http://openorienteering.org/apps/mapper/xml/v2\" version=\"9\">\n<notes></notes>\n").expect("Could not write to map file");
        f.write(format!("<georeferencing scale=\"15000\"><projected_crs id=\"Local\"><ref_point x=\"{}\" y=\"{}\"/></projected_crs></georeferencing>\n", self.ref_point.x, self.ref_point.y).as_bytes()).expect("Could not write to map file");
    }

    fn write_colors_symbols(&self, f: &mut BufWriter<File>) {
        f.write(include_str!("colors_and_symbols_omap.txt").as_bytes())
            .expect("Could not write to map file");
    }

    fn write_objects(&self, f: &mut BufWriter<File>) {
        f.write(
            format!(
                "<parts count=\"1\" current=\"0\">\n<part name=\"map\"><objects count=\"{}\">\n",
                self.objects.len()
            )
            .as_bytes(),
        )
        .expect("Could not write to map file");

        for object in self.objects.iter() {
            object.write_to_map(f);
        }

        f.write(b"</objects></part>\n</parts>\n")
            .expect("Could not write to map file");
    }

    fn write_end_of_file(&self, f: &mut BufWriter<File>) {
        f.write(b"<templates count=\"0\" first_front_template=\"0\">\n<defaults use_meters_per_pixel=\"true\" meters_per_pixel=\"0\" dpi=\"0\" scale=\"0\"/></templates>\n<view>\n").expect("Could not write to map file");
        f.write(b"<grid color=\"#646464\" display=\"0\" alignment=\"0\" additional_rotation=\"0\" unit=\"1\" h_spacing=\"500\" v_spacing=\"500\" h_offset=\"0\" v_offset=\"0\" snapping_enabled=\"true\"/>\n").expect("Could not write to map file");
        f.write(b"<map_view zoom=\"1\" position_x=\"0\" position_y=\"0\"><map opacity=\"1\" visible=\"true\"/><templates count=\"0\"/></map_view>\n</view>\n</barrier>\n</map>").expect("Could not write to map file");
    }
}
