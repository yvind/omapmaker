use crate::geometry::contour_set::ContourPoint;
use crate::{CELL_SIZE, SIDE_LENGTH};
use geo::{Coord, LineString, MultiLineString};

use std::ops::{Index, IndexMut};

#[derive(Clone, Debug)]
pub struct Dfm {
    pub field: [f64; SIDE_LENGTH * SIDE_LENGTH],
    pub tl_coord: Coord,
}

impl Dfm {
    pub fn new(tl_coord: Coord) -> Dfm {
        Dfm {
            field: [f64::MIN; SIDE_LENGTH * SIDE_LENGTH],
            tl_coord,
        }
    }

    pub fn create_ghost_points(&self) -> [ContourPoint; 4] {
        const GRAD_CELLS: usize = 5;
        const GRAD_LENGTH: f64 = GRAD_CELLS as f64 * CELL_SIZE;

        let top_left = ContourPoint {
            pos: self.index2spade(0, 0),
            z: self[(0, 0)],
            grad: [
                (self[(0, GRAD_CELLS)] - self[(0, 0)]) / GRAD_LENGTH,
                (self[(0, 0)] - self[(GRAD_CELLS, 0)]) / GRAD_LENGTH,
            ],
        };

        let top_right = ContourPoint {
            pos: self.index2spade(0, SIDE_LENGTH - 1),
            z: self[(0, SIDE_LENGTH - 1)],
            grad: [
                (self[(0, SIDE_LENGTH - 1)] - self[(0, SIDE_LENGTH - 1 - GRAD_CELLS)])
                    / GRAD_LENGTH,
                (self[(0, SIDE_LENGTH - 1)] - self[(GRAD_CELLS, SIDE_LENGTH - 1)]) / GRAD_LENGTH,
            ],
        };

        let bottom_right = ContourPoint {
            pos: self.index2spade(SIDE_LENGTH - 1, SIDE_LENGTH - 1),
            z: self[(SIDE_LENGTH - 1, SIDE_LENGTH - 1)],
            grad: [
                (self[(SIDE_LENGTH - 1, SIDE_LENGTH - 1)]
                    - self[(SIDE_LENGTH - 1, SIDE_LENGTH - 1 - GRAD_CELLS)])
                    / GRAD_LENGTH,
                (self[(SIDE_LENGTH - 1 - GRAD_CELLS, SIDE_LENGTH - 1)]
                    - self[(SIDE_LENGTH - 1, SIDE_LENGTH - 1)])
                    / GRAD_LENGTH,
            ],
        };

        let bottom_left = ContourPoint {
            pos: self.index2spade(SIDE_LENGTH - 1, 0),
            z: self[(SIDE_LENGTH - 1, 0)],
            grad: [
                (self[(SIDE_LENGTH - 1, GRAD_CELLS)] - self[(SIDE_LENGTH - 1, 0)]) / GRAD_LENGTH,
                (self[(SIDE_LENGTH - 1 - GRAD_CELLS, 0)] - self[(SIDE_LENGTH - 1, 0)])
                    / GRAD_LENGTH,
            ],
        };

        [top_left, top_right, bottom_left, bottom_right]
    }

    pub fn error(&self, other: &Dfm) -> f64 {
        let mut square_diff = 0.;
        for y in 0..SIDE_LENGTH {
            for x in 0..SIDE_LENGTH {
                square_diff += (self[(y, x)] - other[(y, x)]).powi(2);
            }
        }
        square_diff / (SIDE_LENGTH * SIDE_LENGTH) as f64
    }

    pub fn difference(&self, other: &Dfm) -> Dfm {
        let mut diff = self.clone();
        for y in 0..SIDE_LENGTH {
            for x in 0..SIDE_LENGTH {
                diff[(y, x)] -= other[(y, x)];
            }
        }
        diff
    }

    #[inline]
    pub fn index2coord(&self, yi: usize, xi: usize) -> Coord {
        Coord {
            x: (xi as f64) * CELL_SIZE + self.tl_coord.x,
            y: self.tl_coord.y - (yi as f64) * CELL_SIZE,
        }
    }

    #[inline]
    pub fn index2spade(&self, yi: usize, xi: usize) -> spade::Point2<f64> {
        spade::Point2 {
            x: (xi as f64) * CELL_SIZE + self.tl_coord.x,
            y: self.tl_coord.y - (yi as f64) * CELL_SIZE,
        }
    }

    pub fn adjust(
        &mut self,
        truth: &Dfm,
        interpolated: &Dfm,
        filter_half_size: usize,
        amplitude: f64,
    ) {
        let diff = truth.difference(interpolated);
        for yi in 0..SIDE_LENGTH {
            let top_i = yi.saturating_sub(filter_half_size);
            let bottom_i = (yi + filter_half_size).min(SIDE_LENGTH - 1);
            for xi in 0..SIDE_LENGTH {
                let left_i = xi.saturating_sub(filter_half_size);
                let right_i = (xi + filter_half_size).min(SIDE_LENGTH - 1);

                let mut adjustment = 0.;
                for yj in top_i..=bottom_i {
                    for xj in left_i..=right_i {
                        adjustment += diff[(yj, xj)];
                    }
                }

                self[(yi, xi)] += amplitude * adjustment
                    / ((bottom_i - top_i + 1) * (right_i - left_i + 1)) as f64;
            }
        }
    }

    pub fn slope(&self, filter_half_size: usize) -> Dfm {
        let mut slope = Dfm::new(self.tl_coord);

        for yi in 0..SIDE_LENGTH {
            let top_i = yi.saturating_sub(filter_half_size);
            let bottom_i = (yi + filter_half_size).min(SIDE_LENGTH - 1);
            for xi in 0..SIDE_LENGTH {
                let left_i = xi.saturating_sub(filter_half_size);
                let right_i = (xi + filter_half_size).min(SIDE_LENGTH - 1);

                slope[(yi, xi)] = (((self[(top_i, xi)] - self[(bottom_i, xi)])
                    / ((bottom_i - top_i + 1) as f64 * CELL_SIZE))
                    .powi(2)
                    + ((self[(yi, left_i)] - self[(yi, right_i)])
                        / ((right_i - left_i + 1) as f64 * CELL_SIZE))
                        .powi(2))
                .sqrt();
            }
        }

        slope
    }

    // marching squares algorithm for extracting contours
    pub fn marching_squares(&self, level: f64) -> MultiLineString {
        // should preallocate some memory, but how much? How many contours can be expected to be created?
        let mut contours: Vec<LineString> = Vec::with_capacity(8);

        // maps from cell edges to the contour passing that edge in contours-vec
        // including edges added due to padding
        //
        // **_*_*_*_*_*_*_*_*_*_**
        // *|*******************|*
        // *|*******************|*
        // *|************-------|-
        // -|------------|      |
        //  |                   |
        //  |                   |
        //  |___________________|
        //
        // only along the exposed edge can a segment be added
        // the length of the exposed edge is SIDE_LENGTH+2
        // (SIDE_LENGTH-1 horizontal inner segments + 2 paddding + 1 vertical)
        // horizontal segments have indecies 0..=SIDE_LENGTH
        // and the vertical segment has index SIDE_LENGTH+1
        let mut contour_map = [usize::MAX; SIDE_LENGTH + 2];

        //   0       1
        //   *-------*   index into the lut based on the sum of (c > level)*2^i for the corner value c at all corner indecies i
        //   |       |   the lut gives which directed edge that should be crossed by the contour as corner indecies of the start and end corner
        //   |       |   performs linear interpolation based on the corner values of the crossed edges
        //   *-------*
        //   3       2
        //
        // 5s are only filler values, need four spaces for the special cases 5 and 10
        let lut = [
            [5, 5, 5, 5], // nothing
            [3, 0, 5, 5], // merge
            [0, 1, 5, 5], // append,
            [3, 1, 5, 5], // append
            [1, 2, 5, 5], // new
            [1, 0, 3, 2], // prepend and append
            [0, 2, 5, 5], // append
            [3, 2, 5, 5], // append
            [2, 3, 5, 5], // prepend
            [2, 0, 5, 5], // prepend
            [0, 1, 2, 3], // append and prepend
            [2, 1, 5, 5], // new
            [1, 3, 5, 5], // prepend
            [1, 0, 5, 5], // prepend
            [0, 3, 5, 5], // merge
            [5, 5, 5, 5], // nothing
        ];

        // make a f64::MIN-padded proxy of self to avoid edge problems and close all contours
        let padded = DfmPaddedProxy::new(self);

        for yi in 0..SIDE_LENGTH + 1 {
            let ys = [yi, yi, yi + 1, yi + 1];
            for xi in 0..SIDE_LENGTH + 1 {
                let xs = [xi, xi + 1, xi + 1, xi];
                let map_address_lut = [xi, SIDE_LENGTH + 1, xi, SIDE_LENGTH + 1];

                let index = (padded[(ys[0], xs[0])] >= level) as usize
                    + 2 * (padded[(ys[1], xs[1])] >= level) as usize
                    + 4 * (padded[(ys[2], xs[2])] >= level) as usize
                    + 8 * (padded[(ys[3], xs[3])] >= level) as usize;

                let edge_indices = lut[index];

                match index {
                    0 | 15.. => (),
                    4 | 11 => {
                        // new
                        let contour = LineString::new(vec![
                            padded.vertex_interpolate(edge_indices[0], &xs, &ys, level),
                            padded.vertex_interpolate(edge_indices[1], &xs, &ys, level),
                        ]);
                        contours.push(contour);
                        // update map
                        contour_map[map_address_lut[edge_indices[0]]] = contours.len() - 1;
                        contour_map[map_address_lut[edge_indices[1]]] = contours.len() - 1;
                    }
                    2 | 3 | 6 | 7 => {
                        // append
                        let ci = contour_map[map_address_lut[edge_indices[0]]];
                        contours[ci].0.push(padded.vertex_interpolate(
                            edge_indices[1],
                            &xs,
                            &ys,
                            level,
                        ));
                        // update map
                        contour_map[map_address_lut[edge_indices[1]]] = ci;
                    }
                    8 | 9 | 12 | 13 => {
                        // prepend
                        let ci = contour_map[map_address_lut[edge_indices[1]]];
                        contours[ci].0.insert(
                            0,
                            padded.vertex_interpolate(edge_indices[0], &xs, &ys, level),
                        );
                        // update map
                        contour_map[map_address_lut[edge_indices[0]]] = ci;
                    }
                    5 => {
                        // prepend + append

                        // prepend
                        let ci1 = contour_map[map_address_lut[edge_indices[1]]];
                        contours[ci1].0.insert(
                            0,
                            padded.vertex_interpolate(edge_indices[0], &xs, &ys, level),
                        );

                        // append
                        let ci2 = contour_map[map_address_lut[edge_indices[2]]];
                        contours[ci2].0.push(padded.vertex_interpolate(
                            edge_indices[3],
                            &xs,
                            &ys,
                            level,
                        ));
                        // update map
                        contour_map[map_address_lut[edge_indices[0]]] = ci1;
                        contour_map[map_address_lut[edge_indices[3]]] = ci2;
                    }
                    10 => {
                        // append + prepend

                        // append
                        let ci1 = contour_map[map_address_lut[edge_indices[0]]];
                        contours[ci1].0.push(padded.vertex_interpolate(
                            edge_indices[1],
                            &xs,
                            &ys,
                            level,
                        ));

                        // prepend
                        let ci2 = contour_map[map_address_lut[edge_indices[3]]];
                        contours[ci2].0.insert(
                            0,
                            padded.vertex_interpolate(edge_indices[2], &xs, &ys, level),
                        );
                        // update map
                        contour_map[map_address_lut[edge_indices[1]]] = ci1;
                        contour_map[map_address_lut[edge_indices[2]]] = ci2;
                    }
                    1 | 14 => {
                        // merge
                        let mut part1_key = contour_map[map_address_lut[edge_indices[0]]];
                        let part2_key = contour_map[map_address_lut[edge_indices[1]]];

                        if part1_key == part2_key {
                            // close a contour
                            contours[part1_key].close();
                        } else {
                            // merge two different contours
                            let part2 = contours.swap_remove(part2_key);

                            // if part1_key was the last element it's new position
                            // is now part2_key after the swap_remove
                            if part1_key == contours.len() {
                                part1_key = part2_key;
                            }
                            // append the contour to the contour at part1_key
                            contours[part1_key].0.extend(part2);

                            // update the map
                            for key in contour_map.iter_mut() {
                                if key == &part2_key {
                                    // update the map for the merged contour
                                    *key = part1_key;
                                } else if key == &contours.len() {
                                    // update the map for the collateral contour
                                    // the keys that pointed to the last element
                                    // should point to part2_key after the swap_remove
                                    *key = part2_key;
                                }
                            }
                        }
                    }
                }
            }
        }
        MultiLineString::new(contours)
    }

    // feature preserving smoothing of a DEM by normal vector smoothing
    //
    // LiDAR DEM Smoothing and the Preservation of Drainage Features
    // J.B.Lindsay (2019)
    //
    // `max_norm_diff` -
    // cells with an angle between their normal vectors greater
    // than this are not used in smoothing each other
    //
    // `filter_size` -
    // pixel size of the smoothing filter, must be odd and min 3
    //
    // `num_iter` -
    // number of smoothing iterations, min 1
    pub fn smoothen(
        &self,
        mut max_norm_diff: f64,
        mut filter_size: usize,
        mut num_iter: usize,
    ) -> Dfm {
        if filter_size % 2 == 0 {
            filter_size += 1;
        }
        filter_size = filter_size.max(3);

        num_iter = num_iter.max(1);
        max_norm_diff = max_norm_diff.abs().min(60.);

        // faster to work with the cosine of the angle instead of getting the actual angles
        let threshold = max_norm_diff.to_radians().cos();

        // calculate normal vectors
        let mut normal_vecs = [(0., 0.); SIDE_LENGTH * SIDE_LENGTH];
        for y in 0..SIDE_LENGTH {
            let y_min_1 = y.saturating_sub(1);
            let y_plus_1 = (y + 1).min(SIDE_LENGTH - 1);

            let ys = [
                y_min_1, y, y_plus_1, y_plus_1, y_plus_1, y, y_min_1, y_min_1,
            ];

            let mut z_vals = [0.; 8];
            for x in 0..SIDE_LENGTH {
                let x_min_1 = x.saturating_sub(1);
                let x_plus_1 = (x + 1).min(SIDE_LENGTH - 1);

                let xs = [
                    x_plus_1, x_plus_1, x_plus_1, x, x_min_1, x_min_1, x_min_1, x,
                ];

                for i in 0..8 {
                    z_vals[i] = self[(ys[i], xs[i])];
                }

                let dzdx = -(z_vals[2] - z_vals[4] + 2. * (z_vals[1] - z_vals[5]) + z_vals[0]
                    - z_vals[6])
                    / (CELL_SIZE * 8.);
                let dzdy = -(z_vals[6] - z_vals[4] + 2. * (z_vals[7] - z_vals[3]) + z_vals[0]
                    - z_vals[2])
                    / (CELL_SIZE * 8.);

                normal_vecs[y * SIDE_LENGTH + x] = (dzdx, dzdy);
            }
        }

        // Smooth normal vectors
        let mut smooth_normal_vecs = [(0., 0.); SIDE_LENGTH * SIDE_LENGTH];

        let mut dx = vec![0; filter_size * filter_size];
        let mut dy = vec![0; filter_size * filter_size];

        // fill the filter d_x and d_y values and the distance-weights
        let half_size = (filter_size as f64 / 2f64).floor() as isize;
        let mut a = 0;
        for y in 0..filter_size {
            for x in 0..filter_size {
                dx[a] = x as isize - half_size;
                dy[a] = y as isize - half_size;
                a += 1;
            }
        }

        for y in 0..SIDE_LENGTH {
            for x in 0..SIDE_LENGTH {
                let mut sum_weights = 0.;
                let mut a = 0.;
                let mut b = 0.;
                for n in 0..filter_size * filter_size {
                    let x_neighbour =
                        (x as isize + dx[n]).clamp(0, SIDE_LENGTH as isize - 1) as usize;
                    let y_neighbour =
                        (y as isize + dy[n]).clamp(0, SIDE_LENGTH as isize - 1) as usize;
                    let neighbour_normal = normal_vecs[y_neighbour * SIDE_LENGTH + x_neighbour];
                    let diff =
                        cos_angle_between(normal_vecs[y * SIDE_LENGTH + x], neighbour_normal);
                    if diff > threshold {
                        let weight = (diff - threshold).powi(2);
                        sum_weights += weight;
                        a += neighbour_normal.0 * weight;
                        b += neighbour_normal.1 * weight;
                    }
                }

                a /= sum_weights;
                b /= sum_weights;

                smooth_normal_vecs[y * SIDE_LENGTH + x] = (a, b);
            }
        }

        // Update the DEM based on the smoothed normal vectors
        let x = [
            -CELL_SIZE, -CELL_SIZE, -CELL_SIZE, 0., CELL_SIZE, CELL_SIZE, CELL_SIZE, 0.,
        ];
        let y = [
            -CELL_SIZE, 0., CELL_SIZE, CELL_SIZE, CELL_SIZE, 0., -CELL_SIZE, -CELL_SIZE,
        ];

        let mut output = self.clone();

        for _ in 0..num_iter {
            for yi in 0..SIDE_LENGTH {
                let y_min_1 = yi.saturating_sub(1);
                let y_plus_1 = (yi + 1).min(SIDE_LENGTH - 1);

                let ys = [
                    y_min_1, yi, y_plus_1, y_plus_1, y_plus_1, yi, y_min_1, y_min_1,
                ];
                for xi in 0..SIDE_LENGTH {
                    let x_min_1 = xi.saturating_sub(1);
                    let x_plus_1 = (xi + 1).min(SIDE_LENGTH - 1);

                    let xs = [
                        x_plus_1, x_plus_1, x_plus_1, xi, x_min_1, x_min_1, x_min_1, xi,
                    ];

                    let mut sum_weight = 0.;
                    let mut z = 0.;
                    for n in 0..8 {
                        let x_neighbour = xs[n];
                        let y_neighbour = ys[n];

                        let smooth_neighbour_normal =
                            smooth_normal_vecs[y_neighbour * SIDE_LENGTH + x_neighbour];
                        let diff = cos_angle_between(
                            smooth_normal_vecs[yi * SIDE_LENGTH + xi],
                            smooth_neighbour_normal,
                        );
                        if diff > threshold {
                            let weight = (diff - threshold).powi(2);
                            sum_weight += weight;
                            z += -(smooth_neighbour_normal.0 * x[n]
                                + smooth_neighbour_normal.1 * y[n]
                                - output[(y_neighbour, x_neighbour)])
                                * weight;
                        }
                    }
                    if sum_weight > f64::EPSILON {
                        output[(yi, xi)] = z / sum_weight;
                    }
                }
            }
        }

        output
    }
}

fn cos_angle_between(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 * b.0 + a.1 * b.1 + 1.)
        / ((a.0 * a.0 + a.1 * a.1 + 1.) * (b.0 * b.0 + b.1 * b.1 + 1.)).sqrt()
}

impl Index<(usize, usize)> for Dfm {
    type Output = f64;

    fn index(&self, index: (usize, usize)) -> &Self::Output {
        &self.field[index.0 * SIDE_LENGTH + index.1]
    }
}

impl IndexMut<(usize, usize)> for Dfm {
    fn index_mut(&mut self, index: (usize, usize)) -> &mut Self::Output {
        &mut self.field[index.0 * SIDE_LENGTH + index.1]
    }
}

struct DfmPaddedProxy<'a> {
    inner: &'a Dfm,
}

impl<'a> DfmPaddedProxy<'a> {
    fn new(inner: &'a Dfm) -> DfmPaddedProxy<'a> {
        DfmPaddedProxy { inner }
    }

    #[inline]
    fn index2coord(&self, yi: usize, xi: usize) -> Coord {
        Coord {
            x: self.inner.tl_coord.x - CELL_SIZE + (xi as f64) * CELL_SIZE,
            y: self.inner.tl_coord.y + CELL_SIZE - (yi as f64) * CELL_SIZE,
        }
    }

    #[inline]
    fn vertex_interpolate(&self, e: usize, xs: &[usize; 4], ys: &[usize; 4], level: f64) -> Coord {
        let a = self[(ys[e], xs[e])];
        let b = self[(ys[(e + 1) % 4], xs[(e + 1) % 4])];

        let a_coord = self.index2coord(ys[e], xs[e]);

        Coord {
            x: a_coord.x
                + CELL_SIZE * (xs[(e + 1) % 4] as i32 - xs[e] as i32) as f64 * (level - a)
                    / (b - a),
            y: a_coord.y
                + CELL_SIZE * (ys[e] as i32 - ys[(e + 1) % 4] as i32) as f64 * (level - a)
                    / (b - a),
        }
    }
}

impl Index<(usize, usize)> for DfmPaddedProxy<'_> {
    type Output = f64;

    fn index(&self, index: (usize, usize)) -> &Self::Output {
        if index.0 == 0 || index.0 == SIDE_LENGTH + 1 || index.1 == 0 || index.1 == SIDE_LENGTH + 1
        {
            &Self::Output::MIN
        } else {
            &self.inner[(index.0 - 1, index.1 - 1)]
        }
    }
}
