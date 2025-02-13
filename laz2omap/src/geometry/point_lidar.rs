#![allow(dead_code)]

use geo::{Coord, Point};
use spade::{HasPosition, Point2};

#[derive(Clone)]
pub struct PointLaz(pub las::Point);

impl PointLaz {
    pub fn new(x: f64, y: f64, z: f64) -> PointLaz {
        PointLaz(las::Point {
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
        })
    }

    #[inline]
    pub fn squared_euclidean_distance(&self, b: &PointLaz) -> f64 {
        (self.0.x - b.0.x).powi(2) + (self.0.y - b.0.y).powi(2)
    }

    #[inline]
    pub fn flatten(self) -> Point {
        Point::new(self.0.x, self.0.y)
    }

    #[inline]
    pub fn coords(&self) -> Coord {
        Coord {
            x: self.0.x,
            y: self.0.y,
        }
    }

    #[inline]
    pub fn consecutive_orientation(&self, a: &PointLaz, b: &PointLaz) -> f64 {
        (a.0.x - self.0.x) * (b.0.y - self.0.y) - (a.0.y - self.0.y) * (b.0.x - self.0.x)
    }

    #[inline]
    pub fn x(&self) -> f64 {
        self.0.x
    }

    #[inline]
    pub fn y(&self) -> f64 {
        self.0.y
    }
}

impl HasPosition for PointLaz {
    type Scalar = f64;

    fn position(&self) -> Point2<Self::Scalar> {
        Point2::new(self.0.x, self.0.y)
    }
}
