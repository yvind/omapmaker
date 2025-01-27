#![allow(dead_code)]
pub struct Vector32{
    pub data: [f64; 32],
}

impl Vector32{
    pub fn new(vector: [f64; 32]) -> Vector32{
        Vector32{
            data: vector,
        }
    }

    pub fn zeros() -> Vector32{
        Vector32{
            data: [0.; 32],
        }
    }
}