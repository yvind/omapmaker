use std::ops::{Add, Mul, Neg, Sub};

use super::Matrix6x6;

pub struct Matrix3x3 {
    pub data: [[f64; 3]; 3],
}

impl Matrix3x3 {
    pub fn from_6x6(mat: &Matrix6x6, tl_index: (usize, usize)) -> Matrix3x3 {
        Matrix3x3 {
            data: [
                [
                    mat.data[tl_index.0][tl_index.1],
                    mat.data[tl_index.0][1 + tl_index.1],
                    mat.data[tl_index.0][2 + tl_index.1],
                ],
                [
                    mat.data[1 + tl_index.0][tl_index.1],
                    mat.data[1 + tl_index.0][1 + tl_index.1],
                    mat.data[1 + tl_index.0][2 + tl_index.1],
                ],
                [
                    mat.data[2 + tl_index.0][tl_index.1],
                    mat.data[2 + tl_index.0][1 + tl_index.1],
                    mat.data[2 + tl_index.0][2 + tl_index.1],
                ],
            ],
        }
    }

    pub fn zeros() -> Matrix3x3 {
        Matrix3x3 { data: [[0.; 3]; 3] }
    }

    pub fn inv_sym(&self) -> Matrix3x3 {
        // 21 multiplications
        // 1 division

        let mat = &self.data;

        let c0 = mat[1][1] * mat[2][2] - mat[1][2] * mat[1][2];
        let c1 = mat[1][2] * mat[0][2] - mat[0][1] * mat[2][2];
        let c2 = mat[0][1] * mat[1][2] - mat[1][1] * mat[0][2];

        let det = mat[0][0] * c0 + mat[0][1] * c1 + mat[0][2] * c2;
        let inv_det = 1.0 / det;

        let nc01 = c1 * inv_det;
        let nc02 = c2 * inv_det;
        let nc12 = (mat[0][1] * mat[0][2] - mat[0][0] * mat[1][2]) * inv_det;

        Matrix3x3 {
            data: [
                [c0 * inv_det, nc01, nc02],
                [
                    nc01,
                    (mat[0][0] * mat[2][2] - mat[0][2] * mat[0][2]) * inv_det,
                    nc12,
                ],
                [
                    nc02,
                    nc12,
                    (mat[0][0] * mat[1][1] - mat[0][1] * mat[0][1]) * inv_det,
                ],
            ],
        }
    }

    pub fn t(&self) -> Matrix3x3 {
        Matrix3x3 {
            data: [
                [self.data[0][0], self.data[1][0], self.data[2][0]],
                [self.data[0][1], self.data[1][1], self.data[2][1]],
                [self.data[0][2], self.data[1][2], self.data[2][2]],
            ],
        }
    }
}

impl Mul for &Matrix3x3 {
    type Output = Matrix3x3;

    fn mul(self, rhs: Self) -> Self::Output {
        // 27 multiplications
        Matrix3x3 {
            data: [
                [
                    self.data[0][0] * rhs.data[0][0]
                        + self.data[0][1] * rhs.data[1][0]
                        + self.data[0][2] * rhs.data[2][0],
                    self.data[0][0] * rhs.data[0][1]
                        + self.data[0][1] * rhs.data[1][1]
                        + self.data[0][2] * rhs.data[2][1],
                    self.data[0][0] * rhs.data[0][2]
                        + self.data[0][1] * rhs.data[1][2]
                        + self.data[0][2] * rhs.data[2][2],
                ],
                [
                    self.data[1][0] * rhs.data[0][0]
                        + self.data[1][1] * rhs.data[1][0]
                        + self.data[1][2] * rhs.data[2][0],
                    self.data[1][0] * rhs.data[0][1]
                        + self.data[1][1] * rhs.data[1][1]
                        + self.data[1][2] * rhs.data[2][1],
                    self.data[1][0] * rhs.data[0][2]
                        + self.data[1][1] * rhs.data[1][2]
                        + self.data[1][2] * rhs.data[2][2],
                ],
                [
                    self.data[2][0] * rhs.data[0][0]
                        + self.data[2][1] * rhs.data[1][0]
                        + self.data[2][2] * rhs.data[2][0],
                    self.data[2][0] * rhs.data[0][1]
                        + self.data[2][1] * rhs.data[1][1]
                        + self.data[2][2] * rhs.data[2][1],
                    self.data[2][0] * rhs.data[0][2]
                        + self.data[2][1] * rhs.data[1][2]
                        + self.data[2][2] * rhs.data[2][2],
                ],
            ],
        }
    }
}

impl Sub for &Matrix3x3 {
    type Output = Matrix3x3;

    fn sub(self, rhs: Self) -> Self::Output {
        Matrix3x3 {
            data: [
                [
                    self.data[0][0] - rhs.data[0][0],
                    self.data[0][1] - rhs.data[0][1],
                    self.data[0][2] - rhs.data[0][2],
                ],
                [
                    self.data[1][0] - rhs.data[1][0],
                    self.data[1][1] - rhs.data[1][1],
                    self.data[1][2] - rhs.data[1][2],
                ],
                [
                    self.data[2][0] - rhs.data[2][0],
                    self.data[2][1] - rhs.data[2][1],
                    self.data[2][2] - rhs.data[2][2],
                ],
            ],
        }
    }
}

impl Add for &Matrix3x3 {
    type Output = Matrix3x3;

    fn add(self, rhs: Self) -> Self::Output {
        Matrix3x3 {
            data: [
                [
                    self.data[0][0] + rhs.data[0][0],
                    self.data[0][1] + rhs.data[0][1],
                    self.data[0][2] + rhs.data[0][2],
                ],
                [
                    self.data[1][0] + rhs.data[1][0],
                    self.data[1][1] + rhs.data[1][1],
                    self.data[1][2] + rhs.data[1][2],
                ],
                [
                    self.data[2][0] + rhs.data[2][0],
                    self.data[2][1] + rhs.data[2][1],
                    self.data[2][2] + rhs.data[2][2],
                ],
            ],
        }
    }
}

impl Neg for &Matrix3x3 {
    type Output = Matrix3x3;

    fn neg(self) -> Self::Output {
        Matrix3x3 {
            data: [
                [-self.data[0][0], -self.data[0][1], -self.data[0][2]],
                [-self.data[1][0], -self.data[1][1], -self.data[1][2]],
                [-self.data[2][0], -self.data[2][1], -self.data[2][2]],
            ],
        }
    }
}
