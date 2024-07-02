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

    pub fn add_object<T: MapObject>(&self, obj: T) {
        self.objects.push(Box::new(obj));
    }

    pub fn write_to_file(&self) {
        let f = File::create(&self.filepath).expect("Unable to create omap file");
        let mut f = BufWriter::new(f);

        self.write_header(&f);
        self.write_colors_symbols(&f);
        self.write_objects(&f);
        self.write_end_of_file(&f);
    }

    fn write_header(&self, f: &BufWriter<File>) {
        f.write(b"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<map xmlns=\"http://openorienteering.org/apps/mapper/xml/v2\" version=\"9\">\n<notes></notes>\n");
        f.write(format!("<georeferencing scale=\"15000\"><projected_crs id=\"Local\"><ref_point x=\"{}\" y=\"{}\"/></projected_crs></georeferencing>\n", self.ref_point.x, self.ref_point.y).as_bytes());
    }

    fn write_colors_symbols(&self, f: &BufWriter<File>) {
        f.write(include_str!("colors_and_symbols_omap.txt").as_bytes());
    }

    fn write_objects(&self, f: &BufWriter<File>) {
        f.write(
            format!(
                "<parts count=\"1\" current=\"0\">\n<part name=\"map\"><objects count=\"{}\">\n",
                self.objects.len()
            )
            .as_bytes(),
        );

        for object in self.objects {
            object.write_to_map(f);
        }
    }

    fn write_end_of_file(&self, f: &BufWriter<File>) {
        f.write(b"<templates count=\"0\" first_front_template=\"0\">\n<defaults use_meters_per_pixel=\"true\" meters_per_pixel=\"0\" dpi=\"0\" scale=\"0\"/></templates>\n<view>\n");
        f.write(b"<grid color=\"#646464\" display=\"0\" alignment=\"0\" additional_rotation=\"0\" unit=\"1\" h_spacing=\"500\" v_spacing=\"500\" h_offset=\"0\" v_offset=\"0\" snapping_enabled=\"true\"/>\n");
        f.write(b"<map_view zoom=\"1\" position_x=\"0\" position_y=\"0\"><map opacity=\"1\" visible=\"true\"/><templates count=\"0\"/></map_view></view></barrier></map>");
    }
}
