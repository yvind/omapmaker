#![allow(dead_code)]

use std::simd::{f64x4, Simd, SimdFloat};

use crate::matrix::vector6::Vector6;

#[derive(Clone, Debug)]
pub struct Matrix6x6 {
    pub data: [[f64; 6]; 6],
}

impl Matrix6x6 {
    pub fn zeros() -> Matrix6x6 {
        Matrix6x6 {
            data: [[0.0; 6]; 6],
        }
    }

    pub fn add_to_diag(&self, d: f64) -> Matrix6x6 {
        let mut diag = self.clone();
        diag.data[0][0] += d;
        diag.data[1][1] += d;
        diag.data[2][2] += d;
        diag.data[3][3] += d;
        diag.data[4][4] += d;
        diag.data[5][5] += d;

        return diag;
    }

    pub fn dot_vec(&self, other: Vector6) -> Vector6 {
        let mut res: Vector6 = Vector6::zeros();
        let mat = &self.data;
        let vec = &other.data;

        res.data[0] = mat[0][0] * vec[0]
            + mat[0][1] * vec[1]
            + mat[0][2] * vec[2]
            + mat[0][3] * vec[3]
            + mat[0][4] * vec[4]
            + mat[0][5] * vec[5];
        res.data[1] = mat[1][0] * vec[0]
            + mat[1][1] * vec[1]
            + mat[1][2] * vec[2]
            + mat[1][3] * vec[3]
            + mat[1][4] * vec[4]
            + mat[1][5] * vec[5];
        res.data[2] = mat[2][0] * vec[0]
            + mat[2][1] * vec[1]
            + mat[2][2] * vec[2]
            + mat[2][3] * vec[3]
            + mat[2][4] * vec[4]
            + mat[2][5] * vec[5];
        res.data[3] = mat[3][0] * vec[0]
            + mat[3][1] * vec[1]
            + mat[3][2] * vec[2]
            + mat[3][3] * vec[3]
            + mat[3][4] * vec[4]
            + mat[3][5] * vec[5];
        res.data[4] = mat[4][0] * vec[0]
            + mat[4][1] * vec[1]
            + mat[4][2] * vec[2]
            + mat[4][3] * vec[3]
            + mat[4][4] * vec[4]
            + mat[4][5] * vec[5];
        res.data[5] = mat[5][0] * vec[0]
            + mat[5][1] * vec[1]
            + mat[5][2] * vec[2]
            + mat[5][3] * vec[3]
            + mat[5][4] * vec[4]
            + mat[5][5] * vec[5];

        return res;
    }

    // faster inverse of symmetric positive definite matrix
    // adapted from DOOM 3 SDK
    pub fn inverse_spd(&self) -> Matrix6x6 {
        //  192 multiplications
        //    2 divisions

        let mat = &self.data;

        let mut r0: [[f64; 3]; 3] = [[0.; 3]; 3];
        let mut r1: [[f64; 3]; 3] = [[0.; 3]; 3];
        let mut r2: [[f64; 3]; 3] = [[0.; 3]; 3];
        let mut r3: [[f64; 3]; 3] = [[0.; 3]; 3];

        // r0 = m0.Inverse(); r0 is spd
        let c0 = mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1];
        let c1 = mat[1][2] * mat[2][0] - mat[1][0] * mat[2][2];
        let c2 = mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0];

        let mut det = mat[0][0] * c0 + mat[0][1] * c1 + mat[0][2] * c2;
        let mut inv_det = 1.0 / det;

        r0[0][0] = c0 * inv_det;
        r0[0][1] = (mat[0][2] * mat[2][1] - mat[0][1] * mat[2][2]) * inv_det;
        r0[0][2] = (mat[0][1] * mat[1][2] - mat[0][2] * mat[1][1]) * inv_det;
        r0[1][0] = c1 * inv_det;
        r0[1][1] = (mat[0][0] * mat[2][2] - mat[0][2] * mat[2][0]) * inv_det;
        r0[1][2] = (mat[0][2] * mat[1][0] - mat[0][0] * mat[1][2]) * inv_det;
        r0[2][0] = c2 * inv_det;
        r0[2][1] = (mat[0][1] * mat[2][0] - mat[0][0] * mat[2][1]) * inv_det;
        r0[2][2] = (mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0]) * inv_det;

        // r1 = r0 * m1;
        r1[0][0] = r0[0][0] * mat[0][3] + r0[0][1] * mat[1][3] + r0[0][2] * mat[2][3];
        r1[0][1] = r0[0][0] * mat[0][4] + r0[0][1] * mat[1][4] + r0[0][2] * mat[2][4];
        r1[0][2] = r0[0][0] * mat[0][5] + r0[0][1] * mat[1][5] + r0[0][2] * mat[2][5];
        r1[1][0] = r0[1][0] * mat[0][3] + r0[1][1] * mat[1][3] + r0[1][2] * mat[2][3];
        r1[1][1] = r0[1][0] * mat[0][4] + r0[1][1] * mat[1][4] + r0[1][2] * mat[2][4];
        r1[1][2] = r0[1][0] * mat[0][5] + r0[1][1] * mat[1][5] + r0[1][2] * mat[2][5];
        r1[2][0] = r0[2][0] * mat[0][3] + r0[2][1] * mat[1][3] + r0[2][2] * mat[2][3];
        r1[2][1] = r0[2][0] * mat[0][4] + r0[2][1] * mat[1][4] + r0[2][2] * mat[2][4];
        r1[2][2] = r0[2][0] * mat[0][5] + r0[2][1] * mat[1][5] + r0[2][2] * mat[2][5];

        // r2 = m2 * r1;
        r2[0][0] = mat[3][0] * r1[0][0] + mat[3][1] * r1[1][0] + mat[3][2] * r1[2][0];
        r2[0][1] = mat[3][0] * r1[0][1] + mat[3][1] * r1[1][1] + mat[3][2] * r1[2][1];
        r2[0][2] = mat[3][0] * r1[0][2] + mat[3][1] * r1[1][2] + mat[3][2] * r1[2][2];
        r2[1][0] = mat[4][0] * r1[0][0] + mat[4][1] * r1[1][0] + mat[4][2] * r1[2][0];
        r2[1][1] = mat[4][0] * r1[0][1] + mat[4][1] * r1[1][1] + mat[4][2] * r1[2][1];
        r2[1][2] = mat[4][0] * r1[0][2] + mat[4][1] * r1[1][2] + mat[4][2] * r1[2][2];
        r2[2][0] = mat[5][0] * r1[0][0] + mat[5][1] * r1[1][0] + mat[5][2] * r1[2][0];
        r2[2][1] = mat[5][0] * r1[0][1] + mat[5][1] * r1[1][1] + mat[5][2] * r1[2][1];
        r2[2][2] = mat[5][0] * r1[0][2] + mat[5][1] * r1[1][2] + mat[5][2] * r1[2][2];

        // r3 = r2 - m3;
        r3[0][0] = r2[0][0] - mat[3][3];
        r3[0][1] = r2[0][1] - mat[3][4];
        r3[0][2] = r2[0][2] - mat[3][5];
        r3[1][0] = r2[1][0] - mat[4][3];
        r3[1][1] = r2[1][1] - mat[4][4];
        r3[1][2] = r2[1][2] - mat[4][5];
        r3[2][0] = r2[2][0] - mat[5][3];
        r3[2][1] = r2[2][1] - mat[5][4];
        r3[2][2] = r2[2][2] - mat[5][5];

        // r3 = r3.Inverse();
        // uses r2 as temporary storage of intermediate values
        r2[0][0] = r3[1][1] * r3[2][2] - r3[1][2] * r3[2][1];
        r2[1][0] = r3[1][2] * r3[2][0] - r3[1][0] * r3[2][2];
        r2[2][0] = r3[1][0] * r3[2][1] - r3[1][1] * r3[2][0];

        det = r3[0][0] * r2[0][0] + r3[0][1] * r2[1][0] + r3[0][2] * r2[2][0];
        inv_det = 1.0 / det;

        r2[0][1] = r3[0][2] * r3[2][1] - r3[0][1] * r3[2][2];
        r2[0][2] = r3[0][1] * r3[1][2] - r3[0][2] * r3[1][1];
        r2[1][1] = r3[0][0] * r3[2][2] - r3[0][2] * r3[2][0];
        r2[1][2] = r3[0][2] * r3[1][0] - r3[0][0] * r3[1][2];
        r2[2][1] = r3[0][1] * r3[2][0] - r3[0][0] * r3[2][1];
        r2[2][2] = r3[0][0] * r3[1][1] - r3[0][1] * r3[1][0];

        r3[0][0] = r2[0][0] * inv_det;
        r3[0][1] = r2[0][1] * inv_det;
        r3[0][2] = r2[0][2] * inv_det;
        r3[1][0] = r2[1][0] * inv_det;
        r3[1][1] = r2[1][1] * inv_det;
        r3[1][2] = r2[1][2] * inv_det;
        r3[2][0] = r2[2][0] * inv_det;
        r3[2][1] = r2[2][1] * inv_det;
        r3[2][2] = r2[2][2] * inv_det;

        // r2 = m2 * r0;
        r2[0][0] = mat[3][0] * r0[0][0] + mat[3][1] * r0[1][0] + mat[3][2] * r0[2][0];
        r2[0][1] = mat[3][0] * r0[0][1] + mat[3][1] * r0[1][1] + mat[3][2] * r0[2][1];
        r2[0][2] = mat[3][0] * r0[0][2] + mat[3][1] * r0[1][2] + mat[3][2] * r0[2][2];
        r2[1][0] = mat[4][0] * r0[0][0] + mat[4][1] * r0[1][0] + mat[4][2] * r0[2][0];
        r2[1][1] = mat[4][0] * r0[0][1] + mat[4][1] * r0[1][1] + mat[4][2] * r0[2][1];
        r2[1][2] = mat[4][0] * r0[0][2] + mat[4][1] * r0[1][2] + mat[4][2] * r0[2][2];
        r2[2][0] = mat[5][0] * r0[0][0] + mat[5][1] * r0[1][0] + mat[5][2] * r0[2][0];
        r2[2][1] = mat[5][0] * r0[0][1] + mat[5][1] * r0[1][1] + mat[5][2] * r0[2][1];
        r2[2][2] = mat[5][0] * r0[0][2] + mat[5][1] * r0[1][2] + mat[5][2] * r0[2][2];

        let mut result = Matrix6x6::zeros();

        // m2 = r3 * r2;
        result.data[3][0] = r3[0][0] * r2[0][0] + r3[0][1] * r2[1][0] + r3[0][2] * r2[2][0];
        result.data[3][1] = r3[0][0] * r2[0][1] + r3[0][1] * r2[1][1] + r3[0][2] * r2[2][1];
        result.data[3][2] = r3[0][0] * r2[0][2] + r3[0][1] * r2[1][2] + r3[0][2] * r2[2][2];
        result.data[4][0] = r3[1][0] * r2[0][0] + r3[1][1] * r2[1][0] + r3[1][2] * r2[2][0];
        result.data[4][1] = r3[1][0] * r2[0][1] + r3[1][1] * r2[1][1] + r3[1][2] * r2[2][1];
        result.data[4][2] = r3[1][0] * r2[0][2] + r3[1][1] * r2[1][2] + r3[1][2] * r2[2][2];
        result.data[5][0] = r3[2][0] * r2[0][0] + r3[2][1] * r2[1][0] + r3[2][2] * r2[2][0];
        result.data[5][1] = r3[2][0] * r2[0][1] + r3[2][1] * r2[1][1] + r3[2][2] * r2[2][1];
        result.data[5][2] = r3[2][0] * r2[0][2] + r3[2][1] * r2[1][2] + r3[2][2] * r2[2][2];

        // m0 = r0 - r1 * m2;
        result.data[0][0] = r0[0][0]
            - r1[0][0] * result.data[3][0]
            - r1[0][1] * result.data[4][0]
            - r1[0][2] * result.data[5][0];
        result.data[0][1] = r0[0][1]
            - r1[0][0] * result.data[3][1]
            - r1[0][1] * result.data[4][1]
            - r1[0][2] * result.data[5][1];
        result.data[0][2] = r0[0][2]
            - r1[0][0] * result.data[3][2]
            - r1[0][1] * result.data[4][2]
            - r1[0][2] * result.data[5][2];
        result.data[1][0] = result.data[0][1];
        result.data[1][1] = r0[1][1]
            - r1[1][0] * result.data[3][1]
            - r1[1][1] * result.data[4][1]
            - r1[1][2] * result.data[5][1];
        result.data[1][2] = r0[1][2]
            - r1[1][0] * result.data[3][2]
            - r1[1][1] * result.data[4][2]
            - r1[1][2] * result.data[5][2];
        result.data[2][0] = result.data[0][2];
        result.data[2][1] = result.data[1][2];
        result.data[2][2] = r0[2][2]
            - r1[2][0] * result.data[3][2]
            - r1[2][1] * result.data[4][2]
            - r1[2][2] * result.data[5][2];

        // m1 = r1 * r3; and symmetric
        result.data[0][3] = result.data[3][0];
        result.data[0][4] = result.data[4][0];
        result.data[0][5] = result.data[5][0];
        result.data[1][3] = result.data[3][1];
        result.data[1][4] = result.data[4][1];
        result.data[1][5] = result.data[5][1];
        result.data[2][3] = result.data[3][2];
        result.data[2][4] = result.data[4][2];
        result.data[2][5] = result.data[5][2];

        // m3 = -r3;
        result.data[3][3] = -r3[0][0];
        result.data[3][4] = -r3[0][1];
        result.data[3][5] = -r3[0][2];
        result.data[4][3] = result.data[3][4];
        result.data[4][4] = -r3[1][1];
        result.data[4][5] = -r3[1][2];
        result.data[5][3] = result.data[3][5];
        result.data[5][4] = result.data[4][5];
        result.data[5][5] = -r3[2][2];

        return result;
    }

    // same algorithm as inverse_spd just with simd intrinsics (should still be portable)
    pub fn inverse_spd_simd(&self) -> Matrix6x6 {
        let mat = &self.data;

        let row0 = f64x4::from_array([mat[0][0], mat[0][1], mat[0][2], 0.0]);
        let row1 = f64x4::from_array([mat[1][0], mat[1][1], mat[1][2], 0.0]);
        let row2 = f64x4::from_array([mat[2][0], mat[2][1], mat[2][2], 0.0]);

        let c0 = row1.shuffle::<1, 2, 0, 3>() * row2.shuffle::<2, 0, 1, 3>();
        let c1 = row1.shuffle::<2, 0, 1, 3>() * row2.shuffle::<1, 2, 0, 3>();
        let c = c0 - c1;
        let det = (row0 * c).reduce_sum();

        let inv_det = Simd::splat(1.0 / det);

        let r0 = c * inv_det;
        let r1 = (row0.shuffle::<2, 0, 1, 3>() * row2.shuffle::<1, 2, 0, 3>()
            - row0.shuffle::<1, 2, 0, 3>() * row2.shuffle::<2, 0, 1, 3>())
            * inv_det;
        let r2 = (row0.shuffle::<1, 2, 0, 3>() * row1.shuffle::<2, 0, 1, 3>()
            - row0.shuffle::<2, 0, 1, 3>() * row1.shuffle::<1, 2, 0, 3>())
            * inv_det;

        let mut result = Matrix6x6::zeros();
        result
    }
}
