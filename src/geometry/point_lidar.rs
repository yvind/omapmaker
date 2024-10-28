#![allow(dead_code)]

use super::{Line, Point};

pub use las::Point as PointLaz;

impl Point for PointLaz {
    fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            x,
            y,
            z,
            intensity: 0,
            return_number: 1,
            number_of_returns: 1,
            scan_direction: las::point::ScanDirection::LeftToRight,
            is_edge_of_flight_line: false,
            classification: las::point::Classification::Ground,
            is_synthetic: true,
            is_key_point: false,
            is_withheld: false,
            is_overlap: false,
            scanner_channel: 0,
            scan_angle: 0.,
            user_data: 0,
            point_source_id: 0,
            gps_time: None,
            color: None,
            waveform: None,
            nir: None,
            extra_bytes: vec![],
        }
    }

    fn get_x(&self) -> f64 {
        self.x
    }

    fn get_y(&self) -> f64 {
        self.y
    }

    fn get_z(&self) -> f64 {
        self.z
    }

    fn translate(&mut self, dx: f64, dy: f64, dz: f64) {
        self.x += dx;
        self.y += dy;
        self.z += dz;
    }

    fn closest_point_on_line_segment(&self, line: &Line) -> Self {
        let mut diff = self.clone();
        diff.x = line.end.x - line.start.x;
        diff.y = line.end.y - line.start.y;
        let len = diff.length();
        diff.norm();

        let mut s = self.clone();
        s.translate(-line.start.x, -line.start.y, 0.);

        let image = s.dot(&diff).max(0.).min(len);

        let mut out = self.clone();
        out.x = line.start.x + diff.x * image;
        out.y = line.start.y + diff.y * image;
        out
    }

    fn squared_euclidean_distance(&self, b: &impl Point) -> f64 {
        (self.x - b.get_x()).powi(2) + (self.y - b.get_y()).powi(2)
    }

    fn dist_to_line_segment_squared(&self, line: &Line) -> f64 {
        self.squared_euclidean_distance(&self.closest_point_on_line_segment(line))
    }

    fn normal(&self) -> Self {
        let mut out = self.clone();

        out.x = self.y;
        out.y = -self.x;

        out
    }

    fn scale(&mut self, l: f64) {
        self.x *= l;
        self.y *= l;
    }
}
