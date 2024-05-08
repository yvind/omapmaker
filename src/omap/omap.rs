#![allow(dead_code)]

use crate::contour::{Polygon, Contour, Vertex};

use std::io::{BufWriter, Write};
use std::{fs, fs::File, io::BufWriter, path::Path};

pub struct OmapFile{
    f: BufWriter<File>,
}

impl OmapFile{
    pub fn new(&self, filename: String, georef_point: &Point2D) -> OmapFile{
        let bf = File::create(&Path::new(&filename)).expect("Unable to create omap file, try changing the name of the input lidar file and make sure no file named [lidar-file-name].omap exists");
        let map_file = OmapFile{ f = }
    }

}