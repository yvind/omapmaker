#![allow(dead_code)]
pub struct Vector6 {
    pub data: [f64; 6],
}

impl Vector6 {
    pub fn new(vector: [f64; 6]) -> Vector6 {
        Vector6 { data: vector }
    }

    pub fn zeros() -> Vector6 {
        Vector6 { data: [0.; 6] }
    }

    // treats self as a row vector and other as a column vector
    pub fn dot(&self, other: &Vector6) -> f64 {
        let v1 = &self.data;
        let v2 = &other.data;

        v1[0] * v2[0]
            + v1[1] * v2[1]
            + v1[2] * v2[2]
            + v1[3] * v2[3]
            + v1[4] * v2[4]
            + v1[5] * v2[5]
    }
}
