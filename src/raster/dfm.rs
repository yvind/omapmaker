use crate::geometry::contour_set::ContourPoint;
use crate::{CELL_SIZE_METERS, TILE_SIZE_PIXELS};

use std::marker::PhantomData;
use std::ops::{Index, IndexMut};

#[derive(Clone, Copy, Debug)]
pub struct Elevation;
#[derive(Clone, Copy, Debug)]
pub struct Slope;
#[derive(Clone, Copy, Debug)]
pub struct Hillshade;
#[derive(Clone, Copy, Debug)]
pub struct Returns;
#[derive(Clone, Copy, Debug)]
pub struct Intensity;
#[derive(Clone, Copy, Debug)]
pub struct HeightAboveGround;
#[derive(Clone, Copy, Debug)]
pub struct LastReturn;
#[derive(Clone, Copy, Debug)]
pub struct Ground;
#[derive(Clone, Copy, Debug)]
pub struct LowVegetation;
#[derive(Clone, Copy, Debug)]
pub struct MediumVegetation;
#[derive(Clone, Copy, Debug)]
pub struct HighVegetation;
#[derive(Clone, Copy, Debug)]
pub struct SurfaceObjects;
#[derive(Clone, Copy, Debug)]
pub struct Ndvd;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DfmPixelBounds {
    pub top: usize,
    pub bottom: usize,
    pub left: usize,
    pub right: usize,
}

impl DfmPixelBounds {
    fn full() -> Self {
        Self {
            top: 0,
            bottom: TILE_SIZE_PIXELS,
            left: 0,
            right: TILE_SIZE_PIXELS,
        }
    }

    pub fn is_empty(self) -> bool {
        self.top >= self.bottom || self.left >= self.right
    }
}

#[derive(Clone, Debug)]
pub struct Dfm<T> {
    pub field: Box<[f64]>,
    pub tl_coord: geo::Coord,
    pub inner: DfmPixelBounds,
    _t: PhantomData<T>,
}

impl<T> Dfm<T> {
    #[inline]
    pub fn index2coord(&self, yi: usize, xi: usize) -> geo::Coord {
        geo::Coord {
            x: (xi as f64) * CELL_SIZE_METERS + self.tl_coord.x,
            y: self.tl_coord.y - (yi as f64) * CELL_SIZE_METERS,
        }
    }

    #[inline]
    pub fn index2spade(&self, yi: usize, xi: usize) -> spade::Point2<f64> {
        spade::Point2 {
            x: (xi as f64) * CELL_SIZE_METERS + self.tl_coord.x,
            y: self.tl_coord.y - (yi as f64) * CELL_SIZE_METERS,
        }
    }
}

impl<T: Clone> Dfm<T> {
    pub fn new(tl_coord: geo::Coord) -> Dfm<T> {
        Dfm {
            field: vec![f64::MIN; TILE_SIZE_PIXELS * TILE_SIZE_PIXELS].into_boxed_slice(),
            tl_coord,
            inner: DfmPixelBounds::full(),
            _t: PhantomData,
        }
    }

    pub fn with_cut_bounds(tl_coord: geo::Coord, cut_bounds: geo::Rect) -> Dfm<T> {
        let mut dfm = Self::new(tl_coord);
        dfm.inner = dfm.pixel_bounds(cut_bounds);
        dfm
    }

    pub fn new_like<U>(other: &Dfm<U>) -> Dfm<T> {
        let mut dfm = Self::new(other.tl_coord);
        dfm.inner = other.inner;
        dfm
    }

    fn pixel_bounds(&self, cut_bounds: geo::Rect) -> DfmPixelBounds {
        let left = (0..TILE_SIZE_PIXELS)
            .find(|&x| self.index2coord(0, x).x >= cut_bounds.min().x)
            .unwrap_or(TILE_SIZE_PIXELS);
        let right = (left..TILE_SIZE_PIXELS)
            .find(|&x| self.index2coord(0, x).x > cut_bounds.max().x)
            .unwrap_or(TILE_SIZE_PIXELS);
        let top = (0..TILE_SIZE_PIXELS)
            .find(|&y| self.index2coord(y, 0).y <= cut_bounds.max().y)
            .unwrap_or(TILE_SIZE_PIXELS);
        let bottom = (top..TILE_SIZE_PIXELS)
            .find(|&y| self.index2coord(y, 0).y < cut_bounds.min().y)
            .unwrap_or(TILE_SIZE_PIXELS);

        DfmPixelBounds {
            top,
            bottom,
            left,
            right,
        }
    }

    pub fn error(&self, other: &Dfm<T>) -> f64 {
        let mut square_diff = 0.;
        for y in 0..TILE_SIZE_PIXELS {
            for x in 0..TILE_SIZE_PIXELS {
                square_diff += (self[(y, x)] - other[(y, x)]).powi(2);
            }
        }
        square_diff / (TILE_SIZE_PIXELS * TILE_SIZE_PIXELS) as f64
    }

    pub fn difference(&self, other: &Dfm<T>) -> Dfm<T> {
        let mut diff = self.clone();
        for y in 0..TILE_SIZE_PIXELS {
            for x in 0..TILE_SIZE_PIXELS {
                diff[(y, x)] -= other[(y, x)];
            }
        }
        diff
    }

    pub fn adjust(
        &mut self,
        truth: &Dfm<T>,
        interpolated: &Dfm<T>,
        filter_half_size: usize,
        amplitude: f64,
    ) {
        let diff = truth.difference(interpolated);
        for yi in 0..TILE_SIZE_PIXELS {
            let top_i = yi.saturating_sub(filter_half_size);
            let bottom_i = (yi + filter_half_size).min(TILE_SIZE_PIXELS - 1);
            for xi in 0..TILE_SIZE_PIXELS {
                let left_i = xi.saturating_sub(filter_half_size);
                let right_i = (xi + filter_half_size).min(TILE_SIZE_PIXELS - 1);

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
}

impl Dfm<Elevation> {
    pub fn create_ghost_points(&self) -> [ContourPoint; 4] {
        const GRAD_CELLS: usize = 5;
        const GRAD_LENGTH: f64 = GRAD_CELLS as f64 * CELL_SIZE_METERS;

        let top_left = ContourPoint {
            pos: self.index2spade(0, 0),
            z: self[(0, 0)],
            grad: [
                (self[(0, GRAD_CELLS)] - self[(0, 0)]) / GRAD_LENGTH,
                (self[(0, 0)] - self[(GRAD_CELLS, 0)]) / GRAD_LENGTH,
            ],
        };

        let top_right = ContourPoint {
            pos: self.index2spade(0, TILE_SIZE_PIXELS - 1),
            z: self[(0, TILE_SIZE_PIXELS - 1)],
            grad: [
                (self[(0, TILE_SIZE_PIXELS - 1)] - self[(0, TILE_SIZE_PIXELS - 1 - GRAD_CELLS)])
                    / GRAD_LENGTH,
                (self[(0, TILE_SIZE_PIXELS - 1)] - self[(GRAD_CELLS, TILE_SIZE_PIXELS - 1)])
                    / GRAD_LENGTH,
            ],
        };

        let bottom_right = ContourPoint {
            pos: self.index2spade(TILE_SIZE_PIXELS - 1, TILE_SIZE_PIXELS - 1),
            z: self[(TILE_SIZE_PIXELS - 1, TILE_SIZE_PIXELS - 1)],
            grad: [
                (self[(TILE_SIZE_PIXELS - 1, TILE_SIZE_PIXELS - 1)]
                    - self[(TILE_SIZE_PIXELS - 1, TILE_SIZE_PIXELS - 1 - GRAD_CELLS)])
                    / GRAD_LENGTH,
                (self[(TILE_SIZE_PIXELS - 1 - GRAD_CELLS, TILE_SIZE_PIXELS - 1)]
                    - self[(TILE_SIZE_PIXELS - 1, TILE_SIZE_PIXELS - 1)])
                    / GRAD_LENGTH,
            ],
        };

        let bottom_left = ContourPoint {
            pos: self.index2spade(TILE_SIZE_PIXELS - 1, 0),
            z: self[(TILE_SIZE_PIXELS - 1, 0)],
            grad: [
                (self[(TILE_SIZE_PIXELS - 1, GRAD_CELLS)] - self[(TILE_SIZE_PIXELS - 1, 0)])
                    / GRAD_LENGTH,
                (self[(TILE_SIZE_PIXELS - 1 - GRAD_CELLS, 0)] - self[(TILE_SIZE_PIXELS - 1, 0)])
                    / GRAD_LENGTH,
            ],
        };

        [top_left, top_right, bottom_left, bottom_right]
    }

    /// Sobel filter gradient estimation
    pub fn slope(&self) -> Dfm<Slope> {
        let mut slope = Dfm::new_like(self);

        for yi in 0..TILE_SIZE_PIXELS {
            for xi in 0..TILE_SIZE_PIXELS {
                let (v, h) = self.sobel_gradient(yi, xi);

                slope[(yi, xi)] = (v.powi(2) + h.powi(2)).sqrt() / 2_f64.sqrt();
            }
        }
        slope
    }

    /// Hill shade from a Sobel-estimated surface normal.
    ///
    /// `sun_angle` is an azimuth in radians, measured counter-clockwise from
    /// the positive x-axis. The sun elevation is fixed at 45 degrees.
    pub fn hillshade(&self, sun_angle: f64) -> Dfm<Hillshade> {
        self.hillshade_as(sun_angle)
    }
}

impl<T: Clone> Dfm<T> {
    pub fn hillshade_as<U: Clone>(&self, sun_angle: f64) -> Dfm<U> {
        let mut hillshade = Dfm::new_like(self);

        let sun_elevation = std::f64::consts::FRAC_PI_4;
        let light_x = sun_angle.cos() * sun_elevation.cos();
        let light_y = sun_angle.sin() * sun_elevation.cos();
        let light_z = sun_elevation.sin();

        for yi in 0..TILE_SIZE_PIXELS {
            for xi in 0..TILE_SIZE_PIXELS {
                let (v, h) = self.sobel_gradient(yi, xi);
                let normal_x = v;
                let normal_y = -h;
                let normal_z = 1.;
                let normal_length =
                    (normal_x.powi(2) + normal_y.powi(2) + normal_z * normal_z).sqrt();

                hillshade[(yi, xi)] =
                    ((normal_x * light_x + normal_y * light_y + normal_z * light_z)
                        / normal_length)
                        .max(0.);
            }
        }

        hillshade
    }

    #[inline]
    fn sobel_gradient(&self, yi: usize, xi: usize) -> (f64, f64) {
        let top_i = yi.saturating_sub(1);
        let bottom_i = (yi + 1).min(TILE_SIZE_PIXELS - 1);
        let left_i = xi.saturating_sub(1);
        let right_i = (xi + 1).min(TILE_SIZE_PIXELS - 1);

        let v = (self[(top_i, left_i)] - self[(top_i, right_i)] + 2. * self[(yi, left_i)]
            - 2. * self[(yi, right_i)]
            + self[(bottom_i, left_i)]
            - self[(bottom_i, right_i)])
            / (2. * CELL_SIZE_METERS);

        let h = (self[(top_i, left_i)] - self[(bottom_i, left_i)] + 2. * self[(top_i, xi)]
            - 2. * self[(bottom_i, xi)]
            + self[(top_i, right_i)]
            - self[(bottom_i, right_i)])
            / (2. * CELL_SIZE_METERS);

        (v, h)
    }

    // marching squares algorithm for extracting contours
    pub fn marching_squares(&self, level: f64) -> geo::MultiLineString {
        // should preallocate some memory, but how much? How many contours can be expected to be created?
        let mut contours: Vec<geo::LineString> = Vec::with_capacity(8);

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
        let mut contour_map = [usize::MAX; TILE_SIZE_PIXELS + 2];

        //   0       1
        //   *-------*   index into the lut based on the sum of (c > level)*2^i for the corner value c at all corner indecies i
        //   |       |   the lut gives which directed edge that should be crossed by the contour as corner indecies of the start and end corner
        //   |       |   performs linear interpolation based on the corner values of the crossed edges
        //   *-------*
        //   3       2
        //
        // 5s are only filler values, need four spaces for the special cases 5 and 10
        const LUT: [[usize; 4]; 16] = [
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

        for yi in 0..TILE_SIZE_PIXELS + 1 {
            let ys = [yi, yi, yi + 1, yi + 1];
            for xi in 0..TILE_SIZE_PIXELS + 1 {
                let xs = [xi, xi + 1, xi + 1, xi];
                let map_address_lut = [xi, TILE_SIZE_PIXELS + 1, xi, TILE_SIZE_PIXELS + 1];

                let index = (padded[(ys[0], xs[0])] >= level) as usize
                    + 2 * (padded[(ys[1], xs[1])] >= level) as usize
                    + 4 * (padded[(ys[2], xs[2])] >= level) as usize
                    + 8 * (padded[(ys[3], xs[3])] >= level) as usize;

                let edge_indices = LUT[index];

                match index {
                    0 | 15.. => (),
                    4 | 11 => {
                        // new
                        let contour = geo::LineString::new(vec![
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
        geo::MultiLineString::new(contours)
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
    ) -> Dfm<T> {
        if filter_size.is_multiple_of(2) {
            filter_size += 1;
        }
        filter_size = filter_size.max(3);

        num_iter = num_iter.max(1);
        max_norm_diff = max_norm_diff.abs().min(60.);

        // faster to work with the cosine of the angle instead of getting the actual angles
        let threshold = max_norm_diff.to_radians().cos();

        // calculate normal vectors
        let mut normal_vecs = vec![(0., 0.); TILE_SIZE_PIXELS * TILE_SIZE_PIXELS];
        for y in 0..TILE_SIZE_PIXELS {
            let y_min_1 = y.saturating_sub(1);
            let y_plus_1 = (y + 1).min(TILE_SIZE_PIXELS - 1);

            let ys = [
                y_min_1, y, y_plus_1, y_plus_1, y_plus_1, y, y_min_1, y_min_1,
            ];

            let mut z_vals = [0.; 8];
            for x in 0..TILE_SIZE_PIXELS {
                let x_min_1 = x.saturating_sub(1);
                let x_plus_1 = (x + 1).min(TILE_SIZE_PIXELS - 1);

                let xs = [
                    x_plus_1, x_plus_1, x_plus_1, x, x_min_1, x_min_1, x_min_1, x,
                ];

                for i in 0..8 {
                    z_vals[i] = self[(ys[i], xs[i])];
                }

                let dzdx = -(z_vals[2] - z_vals[4] + 2. * (z_vals[1] - z_vals[5]) + z_vals[0]
                    - z_vals[6])
                    / (CELL_SIZE_METERS * 8.);
                let dzdy = -(z_vals[6] - z_vals[4] + 2. * (z_vals[7] - z_vals[3]) + z_vals[0]
                    - z_vals[2])
                    / (CELL_SIZE_METERS * 8.);

                normal_vecs[y * TILE_SIZE_PIXELS + x] = (dzdx, dzdy);
            }
        }

        // Smooth normal vectors
        let mut smooth_normal_vecs = vec![(0., 0.); TILE_SIZE_PIXELS * TILE_SIZE_PIXELS];

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

        for y in 0..TILE_SIZE_PIXELS {
            for x in 0..TILE_SIZE_PIXELS {
                let mut sum_weights = 0.;
                let mut a = 0.;
                let mut b = 0.;
                for n in 0..filter_size * filter_size {
                    let x_neighbor =
                        (x as isize + dx[n]).clamp(0, TILE_SIZE_PIXELS as isize - 1) as usize;
                    let y_neighbor =
                        (y as isize + dy[n]).clamp(0, TILE_SIZE_PIXELS as isize - 1) as usize;
                    let neighbor_normal = normal_vecs[y_neighbor * TILE_SIZE_PIXELS + x_neighbor];
                    let diff =
                        cos_angle_between(normal_vecs[y * TILE_SIZE_PIXELS + x], neighbor_normal);
                    if diff > threshold {
                        let weight = (diff - threshold).powi(2);
                        sum_weights += weight;
                        a += neighbor_normal.0 * weight;
                        b += neighbor_normal.1 * weight;
                    }
                }

                a /= sum_weights;
                b /= sum_weights;

                smooth_normal_vecs[y * TILE_SIZE_PIXELS + x] = (a, b);
            }
        }

        // Update the DEM based on the smoothed normal vectors
        let x = [
            -CELL_SIZE_METERS,
            -CELL_SIZE_METERS,
            -CELL_SIZE_METERS,
            0.,
            CELL_SIZE_METERS,
            CELL_SIZE_METERS,
            CELL_SIZE_METERS,
            0.,
        ];
        let y = [
            -CELL_SIZE_METERS,
            0.,
            CELL_SIZE_METERS,
            CELL_SIZE_METERS,
            CELL_SIZE_METERS,
            0.,
            -CELL_SIZE_METERS,
            -CELL_SIZE_METERS,
        ];

        let mut output = self.clone();

        for _ in 0..num_iter {
            for yi in 0..TILE_SIZE_PIXELS {
                let y_min_1 = yi.saturating_sub(1);
                let y_plus_1 = (yi + 1).min(TILE_SIZE_PIXELS - 1);

                let ys = [
                    y_min_1, yi, y_plus_1, y_plus_1, y_plus_1, yi, y_min_1, y_min_1,
                ];
                for xi in 0..TILE_SIZE_PIXELS {
                    let x_min_1 = xi.saturating_sub(1);
                    let x_plus_1 = (xi + 1).min(TILE_SIZE_PIXELS - 1);

                    let xs = [
                        x_plus_1, x_plus_1, x_plus_1, xi, x_min_1, x_min_1, x_min_1, xi,
                    ];

                    let mut sum_weight = 0.;
                    let mut z = 0.;
                    for n in 0..8 {
                        let x_neighbor = xs[n];
                        let y_neighbor = ys[n];

                        let smooth_neighbor_normal =
                            smooth_normal_vecs[y_neighbor * TILE_SIZE_PIXELS + x_neighbor];
                        let diff = cos_angle_between(
                            smooth_normal_vecs[yi * TILE_SIZE_PIXELS + xi],
                            smooth_neighbor_normal,
                        );
                        if diff > threshold {
                            let weight = (diff - threshold).powi(2);
                            sum_weight += weight;
                            z += -(smooth_neighbor_normal.0 * x[n]
                                + smooth_neighbor_normal.1 * y[n]
                                - output[(y_neighbor, x_neighbor)])
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

impl<T> Index<(usize, usize)> for Dfm<T> {
    type Output = f64;

    fn index(&self, index: (usize, usize)) -> &Self::Output {
        &self.field[index.0 * TILE_SIZE_PIXELS + index.1]
    }
}

impl<T> IndexMut<(usize, usize)> for Dfm<T> {
    fn index_mut(&mut self, index: (usize, usize)) -> &mut Self::Output {
        &mut self.field[index.0 * TILE_SIZE_PIXELS + index.1]
    }
}

struct DfmPaddedProxy<'a, T> {
    inner: &'a Dfm<T>,
}

impl<'a, T> DfmPaddedProxy<'a, T> {
    fn new(inner: &'a Dfm<T>) -> DfmPaddedProxy<'a, T> {
        DfmPaddedProxy { inner }
    }

    #[inline]
    fn index2coord(&self, yi: usize, xi: usize) -> geo::Coord {
        geo::Coord {
            x: self.inner.tl_coord.x - CELL_SIZE_METERS + (xi as f64) * CELL_SIZE_METERS,
            y: self.inner.tl_coord.y + CELL_SIZE_METERS - (yi as f64) * CELL_SIZE_METERS,
        }
    }

    #[inline]
    fn vertex_interpolate(
        &self,
        e: usize,
        xs: &[usize; 4],
        ys: &[usize; 4],
        level: f64,
    ) -> geo::Coord {
        let a = self[(ys[e], xs[e])];
        let b = self[(ys[(e + 1) % 4], xs[(e + 1) % 4])];

        let a_coord = self.index2coord(ys[e], xs[e]);

        geo::Coord {
            x: a_coord.x
                + CELL_SIZE_METERS * (xs[(e + 1) % 4] as i32 - xs[e] as i32) as f64 * (level - a)
                    / (b - a),
            y: a_coord.y
                + CELL_SIZE_METERS * (ys[e] as i32 - ys[(e + 1) % 4] as i32) as f64 * (level - a)
                    / (b - a),
        }
    }
}

impl<T> Index<(usize, usize)> for DfmPaddedProxy<'_, T> {
    type Output = f64;

    fn index(&self, index: (usize, usize)) -> &Self::Output {
        if index.0 == 0
            || index.0 == TILE_SIZE_PIXELS + 1
            || index.1 == 0
            || index.1 == TILE_SIZE_PIXELS + 1
        {
            &Self::Output::MIN
        } else {
            &self.inner[(index.0 - 1, index.1 - 1)]
        }
    }
}
