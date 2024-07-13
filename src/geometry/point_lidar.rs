use super::Point;

use std::ops::{Add, Sub};

#[derive(Copy, Clone, Debug)]
pub struct PointLaz {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub r: u8,
    pub i: u32,
    pub c: u8,
    pub n: u8,
}

impl PointLaz {
    pub fn new(x: f64, y: f64, z: f64, r: u8, i: u32, c: u8, n: u8) -> Self {
        PointLaz {
            x,
            y,
            z,
            r,
            i,
            c,
            n,
        }
    }
}

impl Point for PointLaz {
    fn squared_euclidean_distance(&self, b: &PointLaz) -> f64 {
        (self.x - b.x).powi(2) + (self.y - b.y).powi(2)
    }

    fn consecutive_orientation(&self, a: &PointLaz, b: &PointLaz) -> f64 {
        (a.x - self.x) * (b.y - self.y) - (a.y - self.y) * (b.x - self.x)
    }

    fn cross_product(&self, other: &Self) -> f64 {
        self.x * other.y - other.x * self.y
    }

    fn dist_to_line_squared(&self, a: &Self, b: &Self) -> f64 {
        let diff = *b - *a;

        (self.cross_product(&diff) + b.cross_product(a))
            .abs()
            .powi(2)
            / b.squared_euclidean_distance(a)
    }

    fn dot(&self, other: &PointLaz) -> f64 {
        self.x * other.x + self.y * other.y
    }

    fn norm(self) -> Self {
        let l = self.length();
        Self {
            x: self.x / l,
            y: self.y / l,
            z: self.z,
            r: self.r,
            i: self.i,
            c: self.c,
            n: self.n,
        }
    }

    fn length(&self) -> f64 {
        (self.x * self.x + self.y * self.y).sqrt()
    }
}

impl Add for PointLaz {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
            r: self.r,
            i: self.i,
            c: self.c,
            n: self.n,
        }
    }
}

impl Sub for PointLaz {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
            r: self.r,
            i: self.i,
            c: self.c,
            n: self.n,
        }
    }
}
