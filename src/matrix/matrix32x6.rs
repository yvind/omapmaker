#![allow(dead_code)]
use crate::matrix::vector6::Vector6;
use crate::matrix::vector32::Vector32;
use crate::matrix::matrix6x6::Matrix6x6;

#[derive(Clone, Debug)]
pub struct Matrix32x6{
    pub data: [[f64; 6]; 32],
}

impl Matrix32x6{
    pub fn new(array: [[f64; 6]; 32]) -> Matrix32x6{
        Matrix32x6{
            data: array,
        }
    }

    pub fn zeros() -> Matrix32x6{
        Matrix32x6{
            data: [[0.; 6]; 32],
        }
    }

    pub fn insert_row(&mut self, row: [f64; 6], row_index: usize){
        self.data[row_index][0] = row[0];
        self.data[row_index][1] = row[1];
        self.data[row_index][2] = row[2];
        self.data[row_index][3] = row[3];
        self.data[row_index][4] = row[4];
        self.data[row_index][5] = row[5];
    }

    pub fn tdot_vec(&self, vec: &Vector32) -> Vector6{
        let mut result = Vector6::zeros();
        for i in 0..32{
            result.data[0] += mat.data[i][0]*vec.data[i];
            result.data[1] += mat.data[i][1]*vec.data[i];
            result.data[2] += mat.data[i][2]*vec.data[i];
            result.data[3] += mat.data[i][3]*vec.data[i];
            result.data[4] += mat.data[i][4]*vec.data[i];
            result.data[5] += mat.data[i][5]*vec.data[i];
        }
        return result;
    }

    pub fn tdot_self(&self) -> Matrix6x6{
        let mut result = Matrix6x6::zeros();
        for i in 0..6{
            for j in i..6{
                let mut sum = 0.0;
                for k in 0..32{
                    sum += self.data[k][i] * self.data[k][j];
                }
                result.data[i][j] = sum;
                result.data[j][i] = sum;
            }
        }
        return result;
    }
}