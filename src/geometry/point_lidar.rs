#![allow(dead_code)]

use super::Point;

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

    fn closest_point_on_line_segment(&self, a: &impl Point, b: &impl Point) -> PointLaz {
        let mut diff = self.clone();
        diff.x = b.get_x() - a.get_x();
        diff.y = b.get_y() - a.get_y();
        let len = diff.length();
        diff.norm();

        let mut s = self.clone();
        s.translate(-a.get_x(), -a.get_y(), 0.);

        let image = s.dot(&diff).max(0.).min(len);

        let mut out = self.clone();
        out.x = a.get_x() + diff.get_x() * image;
        out.y = a.get_y() + diff.get_y() * image;
        out
    }

    fn squared_euclidean_distance(&self, b: &impl Point) -> f64 {
        (self.x - b.get_x()).powi(2) + (self.y - b.get_y()).powi(2)
    }

    fn consecutive_orientation(&self, a: &impl Point, b: &impl Point) -> f64 {
        (a.get_x() - self.x) * (b.get_y() - self.y) - (a.get_y() - self.y) * (b.get_x() - self.x)
    }

    fn cross_product(&self, other: &impl Point) -> f64 {
        self.x * other.get_y() - other.get_x() * self.y
    }

    fn dist_to_line_segment_squared(&self, a: &impl Point, b: &impl Point) -> f64 {
        self.squared_euclidean_distance(&self.closest_point_on_line_segment(a, b))
    }

    fn dot(&self, other: &impl Point) -> f64 {
        self.x * other.get_x() + self.y * other.get_y()
    }

    fn norm(&mut self) {
        let l = self.length();

        self.scale(1. / l);
    }

    fn length(&self) -> f64 {
        (self.x * self.x + self.y * self.y).sqrt()
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
