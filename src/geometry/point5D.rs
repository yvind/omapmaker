use super::Point;

use std::ops::{Add, Sub};

#[derive(Copy, Clone, Debug)]
pub struct Point5D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub r: u8,
    pub i: u32,
}

impl Point5D {
    pub fn new(x: f64, y: f64, z: f64, r: u8, i: u32) -> Self {
        Point5D { x, y, z, r, i }
    }
}

impl Point for Point5D {
    fn squared_euclidean_distance(&self, b: &Point5D) -> f64 {
        (self.x - b.x).powi(2) + (self.y - b.y).powi(2)
    }

    fn consecutive_orientation(&self, a: &Point5D, b: &Point5D) -> f64 {
        (a.x - self.x) * (b.y - self.y) - (a.y - self.y) * (b.x - self.x)
    }

    fn cross_product(&self, other: &Self) -> f64 {
        self.x * other.y - other.x * self.y
    }
}

impl Add for Point5D {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
            r: self.r,
            i: self.i,
        }
    }
}

impl Sub for Point5D {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
            r: self.r,
            i: self.i,
        }
    }
}
