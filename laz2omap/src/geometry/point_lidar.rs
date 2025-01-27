#![allow(dead_code)]

use geo::{Coord, Point};
pub use las::Point as PointLaz;

impl PointTrait for PointLaz {
    fn new(x: f64, y: f64, z: f64) -> PointLaz {
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

    fn squared_euclidean_distance(&self, b: &PointLaz) -> f64 {
        (self.x - b.x).powi(2) + (self.y - b.y).powi(2)
    }

    fn flatten(self) -> Point {
        Point::new(self.x, self.y)
    }

    fn coords(&self) -> Coord {
        Coord {
            x: self.x,
            y: self.y,
        }
    }

    fn consecutive_orientation(&self, a: &PointLaz, b: &PointLaz) -> f64 {
        (a.x - self.x) * (b.y - self.y) - (a.y - self.y) * (b.x - self.x)
    }
}

pub trait PointTrait {
    fn new(x: f64, y: f64, z: f64) -> Self;

    fn consecutive_orientation(&self, a: &PointLaz, b: &PointLaz) -> f64;

    fn squared_euclidean_distance(&self, other: &PointLaz) -> f64;

    fn flatten(self) -> Point;

    fn coords(&self) -> Coord;
}
