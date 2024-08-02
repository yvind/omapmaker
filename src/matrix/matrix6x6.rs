#![allow(dead_code)]

use std::simd::{f64x4, num::SimdFloat, simd_swizzle, Simd, StdFloat};

use crate::matrix::vector6::Vector6;

use super::Matrix3x3;

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

    pub fn from_blocks(
        tl: &Matrix3x3,
        tr: &Matrix3x3,
        bl: &Matrix3x3,
        br: &Matrix3x3,
    ) -> Matrix6x6 {
        Matrix6x6 {
            data: [
                [
                    tl.data[0][0],
                    tl.data[0][1],
                    tl.data[0][2],
                    tr.data[0][0],
                    tr.data[0][1],
                    tr.data[0][2],
                ],
                [
                    tl.data[1][0],
                    tl.data[1][1],
                    tl.data[1][2],
                    tr.data[1][0],
                    tr.data[1][1],
                    tr.data[1][2],
                ],
                [
                    tl.data[2][0],
                    tl.data[2][1],
                    tl.data[2][2],
                    tr.data[2][0],
                    tr.data[2][1],
                    tr.data[2][2],
                ],
                [
                    bl.data[0][0],
                    bl.data[0][1],
                    bl.data[0][2],
                    br.data[0][0],
                    br.data[0][1],
                    br.data[0][2],
                ],
                [
                    bl.data[1][0],
                    bl.data[1][1],
                    bl.data[1][2],
                    br.data[1][0],
                    br.data[1][1],
                    br.data[1][2],
                ],
                [
                    bl.data[2][0],
                    bl.data[2][1],
                    bl.data[2][2],
                    br.data[2][0],
                    br.data[2][1],
                    br.data[2][2],
                ],
            ],
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

        for i in 0..6 {
            for j in 0..6 {
                res.data[i] += mat[i][j] * vec[j];
            }
        }
        res
    }

    // faster inverse of symmetric positive definite matrix
    pub fn inverse_spd(&self) -> Matrix6x6 {
        // ~145 multiplications
        // 2 divisions

        let m1 = Matrix3x3::from_6x6(self, (0, 3));
        let m1t = Matrix3x3::from_6x6(self, (3, 0));
        let m2 = Matrix3x3::from_6x6(self, (3, 3));
        let m0inv = Matrix3x3::from_6x6(self, (0, 0)).inv_sym();

        let a = &m1t * &m0inv;
        let b = (&m2 - &(&a * &m1)).inv_sym();

        let tr = (&(-&a).t()) * &b;
        let tl = &m0inv - &(&tr * &a);

        Matrix6x6::from_blocks(&tl, &tr, &tr.t(), &b)
    }

    // same algorithm as inverse_spd just with simd intrinsics (should still be portable)
    pub fn inverse_spd_simd(&self) -> Matrix6x6 {
        let mat = &self.data;

        // m0
        let row0 = f64x4::from_array([mat[0][0], mat[0][1], mat[0][2], 0.0]);
        let row1 = f64x4::from_array([mat[1][0], mat[1][1], mat[1][2], 0.0]);
        let row2 = f64x4::from_array([mat[2][0], mat[2][1], mat[2][2], 0.0]);

        let c0 = simd_swizzle!(row1, [1, 2, 0, 3]) * simd_swizzle!(row2, [2, 0, 1, 3]);
        let c1 = simd_swizzle!(row1, [2, 0, 1, 3]) * simd_swizzle!(row2, [1, 2, 0, 3]);
        let c = c0 - c1;
        let det = (row0 * c).reduce_sum();
        let inv_det = Simd::splat(1.0 / det);

        // m0_inv
        let r0 = c * inv_det;
        let r1 = (simd_swizzle!(row0, [2, 0, 1, 3]) * simd_swizzle!(row2, [1, 2, 0, 3])
            - simd_swizzle!(row0, [1, 2, 0, 3]) * simd_swizzle!(row2, [2, 0, 1, 3]))
            * inv_det;

        let r2 = (simd_swizzle!(row0, [1, 2, 0, 3]) * simd_swizzle!(row1, [2, 0, 1, 3])
            - simd_swizzle!(row0, [2, 0, 1, 3]) * simd_swizzle!(row1, [1, 2, 0, 3]))
            * inv_det;

        // calculate a = m1'm0-1
        let m1_row0 = f64x4::from_array([mat[0][3], mat[1][3], mat[2][3], 0.0]);
        let m1_row1 = f64x4::from_array([mat[0][4], mat[1][4], mat[2][4], 0.0]);
        let m1_row2 = f64x4::from_array([mat[0][5], mat[1][5], mat[2][5], 0.0]);

        let a0 = (m1_row0 * r0).reduce_sum();
        let a1 = (m1_row0 * r1).reduce_sum();
        let a2 = (m1_row0 * r2).reduce_sum();
        let a3 = (m1_row1 * r0).reduce_sum();
        let a4 = (m1_row1 * r1).reduce_sum();
        let a5 = (m1_row1 * r2).reduce_sum();
        let a6 = (m1_row2 * r0).reduce_sum();
        let a7 = (m1_row2 * r1).reduce_sum();
        let a8 = (m1_row2 * r2).reduce_sum();

        // calculate b
        let b00 = mat[3][3] - (a0 * mat[0][3] + a1 * mat[1][3] + a2 * mat[2][3]);
        let b01 = mat[3][4] - (a0 * mat[0][4] + a1 * mat[1][4] + a2 * mat[2][4]);
        let b02 = mat[3][5] - (a0 * mat[0][5] + a1 * mat[1][5] + a2 * mat[2][5]);
        let b11 = mat[4][4] - (a3 * mat[0][4] + a4 * mat[1][4] + a5 * mat[2][4]);
        let b12 = mat[4][5] - (a3 * mat[0][5] + a4 * mat[1][5] + a5 * mat[2][5]);
        let b22 = mat[5][5] - (a6 * mat[0][5] + a7 * mat[1][5] + a8 * mat[2][5]);

        // invert b
        let b_row0 = f64x4::from_array([b00, b01, b02, 0.0]);
        let b_row1 = f64x4::from_array([b01, b11, b12, 0.0]);
        let b_row2 = f64x4::from_array([b02, b12, b22, 0.0]);

        let b_c0 = simd_swizzle!(b_row1, [1, 2, 0, 3]) * simd_swizzle!(b_row2, [2, 0, 1, 3]);
        let b_c1 = simd_swizzle!(b_row1, [2, 0, 1, 3]) * simd_swizzle!(b_row2, [1, 2, 0, 3]);
        let b_c = b_c0 - b_c1;
        let b_det = (b_row0 * b_c).reduce_sum();
        let b_inv_det = Simd::splat(1.0 / b_det);

        let b_r0 = b_c * b_inv_det;
        let b_r1 = (simd_swizzle!(b_row0, [2, 0, 1, 3]) * simd_swizzle!(b_row2, [1, 2, 0, 3])
            - simd_swizzle!(b_row0, [1, 2, 0, 3]) * simd_swizzle!(b_row2, [2, 0, 1, 3]))
            * b_inv_det;
        let b_r2 = (simd_swizzle!(b_row0, [1, 2, 0, 3]) * simd_swizzle!(b_row1, [2, 0, 1, 3])
            - simd_swizzle!(b_row0, [2, 0, 1, 3]) * simd_swizzle!(b_row1, [1, 2, 0, 3]))
            * b_inv_det;

        // calculate a'b-1
        let atb0 = f64x4::from_array([a0, a3, a6, 0.0]);
        let atb1 = f64x4::from_array([a1, a4, a7, 0.0]);
        let atb2 = f64x4::from_array([a2, a5, a8, 0.0]);

        let atbinv00 = (atb0 * b_r0).reduce_sum();
        let atbinv01 = (atb0 * b_r1).reduce_sum();
        let atbinv02 = (atb0 * b_r2).reduce_sum();
        let atbinv10 = (atb1 * b_r0).reduce_sum();
        let atbinv11 = (atb1 * b_r1).reduce_sum();
        let atbinv12 = (atb1 * b_r2).reduce_sum();
        let atbinv20 = (atb2 * b_r0).reduce_sum();
        let atbinv21 = (atb2 * b_r1).reduce_sum();
        let atbinv22 = (atb2 * b_r2).reduce_sum();

        // build the result
        // build the result
        let tl00 = r0[0] + (atbinv00 * a0 + atbinv01 * a3 + atbinv02 * a6);
        let tl01 = r0[1] + atbinv00 * a1 + atbinv01 * a4 + atbinv02 * a7;
        let tl02 = r0[2] + atbinv00 * a2 + atbinv01 * a5 + atbinv02 * a8;
        let tl11 = r1[1] + (atbinv10 * a1 + atbinv11 * a4 + atbinv12 * a7);
        let tl12 = atbinv10 * a2 + atbinv11 * a5 + atbinv12 * a8;
        let tl22 = r2[2] + (atbinv20 * a2 + atbinv21 * a5 + atbinv22 * a8);

        Matrix6x6 {
            data: [
                [tl00, tl01, tl02, -atbinv00, -atbinv01, -atbinv02],
                [tl01, tl11, tl12, -atbinv10, -atbinv11, -atbinv12],
                [tl02, tl12, tl22, -atbinv20, -atbinv21, -atbinv22],
                [-atbinv00, -atbinv10, -atbinv20, b_r0[0], b_r0[1], b_r0[2]],
                [-atbinv01, -atbinv11, -atbinv21, b_r1[0], b_r1[1], b_r1[2]],
                [-atbinv02, -atbinv12, -atbinv22, b_r2[0], b_r2[1], b_r2[2]],
            ],
        }
    }

    pub fn inverse_spd_simd2(&self) -> Matrix6x6 {
        use std::simd::f64x4;

        let mat = &self.data;

        // Load upper-left 3x3 block (m0)
        let row0 = f64x4::from_array([mat[0][0], mat[0][1], mat[0][2], 0.0]);
        let row1 = f64x4::from_array([mat[1][0], mat[1][1], mat[1][2], 0.0]);
        let row2 = f64x4::from_array([mat[2][0], mat[2][1], mat[2][2], 0.0]);

        // Calculate m0 inverse
        let c0 = row1.rotate_elements_right::<1>() * row2.rotate_elements_right::<2>();
        let c1 = row1.rotate_elements_right::<2>() * row2.rotate_elements_right::<1>();
        let c = c0 - c1;
        let det = (row0 * c).reduce_sum();
        let inv_det = f64x4::splat(1.0 / det);

        let m0_inv0 = c * inv_det;
        let m0_inv1 = (row0.rotate_elements_right::<2>() * row2.rotate_elements_right::<1>()
            - row0.rotate_elements_right::<1>() * row2.rotate_elements_right::<2>())
            * inv_det;
        let m0_inv2 = (row0.rotate_elements_right::<1>() * row1.rotate_elements_right::<2>()
            - row0.rotate_elements_right::<2>() * row1.rotate_elements_right::<1>())
            * inv_det;

        // Load m1 (upper-right 3x3 block)
        let m1_row0 = f64x4::from_array([mat[0][3], mat[1][3], mat[2][3], 0.0]);
        let m1_row1 = f64x4::from_array([mat[0][4], mat[1][4], mat[2][4], 0.0]);
        let m1_row2 = f64x4::from_array([mat[0][5], mat[1][5], mat[2][5], 0.0]);

        // Calculate a = m1' * m0_inv
        let a0 = f64x4::from_array([
            (m1_row0 * m0_inv0).reduce_sum(),
            (m1_row1 * m0_inv0).reduce_sum(),
            (m1_row2 * m0_inv0).reduce_sum(),
            0.0,
        ]);
        let a1 = f64x4::from_array([
            (m1_row0 * m0_inv1).reduce_sum(),
            (m1_row1 * m0_inv1).reduce_sum(),
            (m1_row2 * m0_inv1).reduce_sum(),
            0.0,
        ]);
        let a2 = f64x4::from_array([
            (m1_row0 * m0_inv2).reduce_sum(),
            (m1_row1 * m0_inv2).reduce_sum(),
            (m1_row2 * m0_inv2).reduce_sum(),
            0.0,
        ]);

        // Calculate b = m2 - a * m1
        let m2_row0 = f64x4::from_array([mat[3][3], mat[3][4], mat[3][5], 0.0]);
        let m2_row1 = f64x4::from_array([mat[4][3], mat[4][4], mat[4][5], 0.0]);
        let m2_row2 = f64x4::from_array([mat[5][3], mat[5][4], mat[5][5], 0.0]);

        let b0 = m2_row0 - (a0 * m1_row0 + a1 * m1_row1 + a2 * m1_row2);
        let b1 = m2_row1
            - (a0.rotate_elements_right::<1>() * m1_row0
                + a1.rotate_elements_right::<1>() * m1_row1
                + a2.rotate_elements_right::<1>() * m1_row2);
        let b2 = m2_row2
            - (a0.rotate_elements_right::<2>() * m1_row0
                + a1.rotate_elements_right::<2>() * m1_row1
                + a2.rotate_elements_right::<2>() * m1_row2);

        // Calculate b inverse
        let b_c0 = b1.rotate_elements_right::<1>() * b2.rotate_elements_right::<2>();
        let b_c1 = b1.rotate_elements_right::<2>() * b2.rotate_elements_right::<1>();
        let b_c = b_c0 - b_c1;
        let b_det = (b0 * b_c).reduce_sum();
        let b_inv_det = f64x4::splat(1.0 / b_det);

        let b_inv0 = b_c * b_inv_det;
        let b_inv1 = (b0.rotate_elements_right::<2>() * b2.rotate_elements_right::<1>()
            - b0.rotate_elements_right::<1>() * b2.rotate_elements_right::<2>())
            * b_inv_det;
        let b_inv2 = (b0.rotate_elements_right::<1>() * b1.rotate_elements_right::<2>()
            - b0.rotate_elements_right::<2>() * b1.rotate_elements_right::<1>())
            * b_inv_det;

        // Calculate final blocks
        let neg_a_b_inv0 = -(a0 * b_inv0 + a1 * b_inv1 + a2 * b_inv2);
        let neg_a_b_inv1 = -(a0.rotate_elements_right::<1>() * b_inv0
            + a1.rotate_elements_right::<1>() * b_inv1
            + a2.rotate_elements_right::<1>() * b_inv2);
        let neg_a_b_inv2 = -(a0.rotate_elements_right::<2>() * b_inv0
            + a1.rotate_elements_right::<2>() * b_inv1
            + a2.rotate_elements_right::<2>() * b_inv2);

        let tl0 = m0_inv0 - (neg_a_b_inv0 * a0 + neg_a_b_inv1 * a1 + neg_a_b_inv2 * a2);
        let tl1 = m0_inv1
            - (neg_a_b_inv0.rotate_elements_right::<1>() * a0
                + neg_a_b_inv1.rotate_elements_right::<1>() * a1
                + neg_a_b_inv2.rotate_elements_right::<1>() * a2);
        let tl2 = m0_inv2
            - (neg_a_b_inv0.rotate_elements_right::<2>() * a0
                + neg_a_b_inv1.rotate_elements_right::<2>() * a1
                + neg_a_b_inv2.rotate_elements_right::<2>() * a2);

        // Construct the result matrix
        Matrix6x6 {
            data: [
                [
                    tl0[0],
                    tl0[1],
                    tl0[2],
                    neg_a_b_inv0[0],
                    neg_a_b_inv0[1],
                    neg_a_b_inv0[2],
                ],
                [
                    tl0[1],
                    tl1[1],
                    tl1[2],
                    neg_a_b_inv1[0],
                    neg_a_b_inv1[1],
                    neg_a_b_inv1[2],
                ],
                [
                    tl0[2],
                    tl1[2],
                    tl2[2],
                    neg_a_b_inv2[0],
                    neg_a_b_inv2[1],
                    neg_a_b_inv2[2],
                ],
                [
                    neg_a_b_inv0[0],
                    neg_a_b_inv1[0],
                    neg_a_b_inv2[0],
                    b_inv0[0],
                    b_inv0[1],
                    b_inv0[2],
                ],
                [
                    neg_a_b_inv0[1],
                    neg_a_b_inv1[1],
                    neg_a_b_inv2[1],
                    b_inv0[1],
                    b_inv1[1],
                    b_inv1[2],
                ],
                [
                    neg_a_b_inv0[2],
                    neg_a_b_inv1[2],
                    neg_a_b_inv2[2],
                    b_inv0[2],
                    b_inv1[2],
                    b_inv2[2],
                ],
            ],
        }
    }
}
