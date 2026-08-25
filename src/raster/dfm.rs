use crate::geometry::contour_set::ContourPoint;
use crate::raster::{
    DfmGrid, Elevation, FloodFill, Hillshade, HydroCorrected, InterpolationErrorImprovement,
    RasterMarker, Slope, TerrainChange, TerrainRasterMarker,
};

use std::collections::VecDeque;
use std::marker::PhantomData;
use std::ops::{Index, IndexMut};

/// Physical parameters for normal-aware, feature-preserving terrain smoothing.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TerrainSmoothing {
    /// Maximum normal-to-normal angle included in the local estimate.
    pub max_normal_difference_degrees: f32,
    /// Radius of the normal-estimation neighbourhood, in map metres.
    pub radius_m: f64,
    /// Number of elevation update passes.
    pub iterations: usize,
    /// Absolute vertical displacement allowed from the input terrain, in metres.
    pub max_elevation_change_m: f32,
}

#[derive(Clone, Debug)]
pub struct Dfm<T: RasterMarker> {
    pub field: Box<[f32]>,
    pub grid: DfmGrid,
    _marker: PhantomData<T>,
}

impl<T: RasterMarker> Dfm<T> {
    pub fn new(grid: DfmGrid) -> Self {
        Self {
            field: vec![f32::MIN; grid.width * grid.height].into_boxed_slice(),
            grid,
            _marker: PhantomData::<T>,
        }
    }

    #[allow(dead_code)]
    pub fn standard(top_left: geo::Coord) -> Self {
        Self::new(DfmGrid::standard(top_left))
    }

    pub fn with_cut_bounds(top_left: geo::Coord, cut_bounds: geo::Rect) -> Self {
        let mut grid = DfmGrid::standard(top_left);
        grid.inner = grid.pixel_bounds(cut_bounds);
        Self::new(grid)
    }

    pub fn new_like<U: RasterMarker>(other: &Dfm<U>) -> Self {
        Self::new(other.grid.clone())
    }

    #[inline]
    pub fn width(&self) -> usize {
        self.grid.width
    }

    #[inline]
    pub fn height(&self) -> usize {
        self.grid.height
    }

    #[inline]
    pub fn index2coord(&self, row: usize, column: usize) -> geo::Coord {
        self.grid.coord(row, column)
    }

    #[inline]
    pub fn index2spade(&self, row: usize, column: usize) -> spade::Point2<f64> {
        let point = self.index2coord(row, column);
        spade::Point2 {
            x: point.x,
            y: point.y,
        }
    }

    pub fn create_ghost_points(&self) -> [ContourPoint; 4] {
        let step = 5.min(self.width().min(self.height()) - 1);
        let length = step as f64 * self.grid.cell_size_m;
        let last_x = self.width() - 1;
        let last_y = self.height() - 1;
        let point = |y: usize, x: usize, x0: usize, x1: usize, y0: usize, y1: usize| ContourPoint {
            pos: self.index2spade(y, x),
            z: self[(y, x)],
            grad: [
                ((f64::from(self[(y, x1)]) - f64::from(self[(y, x0)])) / length) as f32,
                ((f64::from(self[(y0, x)]) - f64::from(self[(y1, x)])) / length) as f32,
            ],
        };
        [
            point(0, 0, 0, step, 0, step),
            point(0, last_x, last_x - step, last_x, 0, step),
            point(last_y, 0, 0, step, last_y - step, last_y),
            point(last_y, last_x, last_x - step, last_x, last_y - step, last_y),
        ]
    }

    #[allow(dead_code)]
    pub fn sample_bilinear(&self, coordinate: geo::Coord) -> Option<f32> {
        let x = (coordinate.x - self.grid.top_left.x) / self.grid.cell_size_m;
        let y = (self.grid.top_left.y - coordinate.y) / self.grid.cell_size_m;
        if x < 0. || y < 0. || x > (self.width() - 1) as f64 || y > (self.height() - 1) as f64 {
            return None;
        }
        let left = x.floor() as usize;
        let top = y.floor() as usize;
        let right = (left + 1).min(self.width() - 1);
        let bottom = (top + 1).min(self.height() - 1);
        let tx = (x - left as f64) as f32;
        let ty = (y - top as f64) as f32;
        Some(
            self[(top, left)] * (1. - tx) * (1. - ty)
                + self[(top, right)] * tx * (1. - ty)
                + self[(bottom, left)] * (1. - tx) * ty
                + self[(bottom, right)] * tx * ty,
        )
    }

    pub fn error(&self, other: &Self) -> f64 {
        self.grid
            .ensure_compatible(&other.grid)
            .expect("cellwise DFM operation requires matching grids");
        self.field
            .iter()
            .zip(&other.field)
            .map(|(&a, &b)| (f64::from(a) - f64::from(b)).powi(2))
            .sum::<f64>()
            / self.field.len() as f64
    }

    pub fn difference(&self, other: &Self) -> Self {
        self.grid
            .ensure_compatible(&other.grid)
            .expect("cellwise DFM operation requires matching grids");
        let mut output = self.clone();
        output
            .field
            .iter_mut()
            .zip(&other.field)
            .for_each(|(value, other)| *value -= other);
        output
    }

    pub fn adjust(
        &mut self,
        truth: &Self,
        interpolated: &Self,
        filter_half_size: usize,
        amplitude: f32,
    ) {
        let difference = truth.difference(interpolated);
        for y in 0..self.height() {
            let top = y.saturating_sub(filter_half_size);
            let bottom = (y + filter_half_size).min(self.height() - 1);
            for x in 0..self.width() {
                let left = x.saturating_sub(filter_half_size);
                let right = (x + filter_half_size).min(self.width() - 1);
                let mut adjustment = 0.;
                for row in top..=bottom {
                    for column in left..=right {
                        adjustment += f64::from(difference[(row, column)]);
                    }
                }
                self[(y, x)] += amplitude
                    * (adjustment / ((bottom - top + 1) * (right - left + 1)) as f64) as f32;
            }
        }
    }

    pub fn hillshade_as<U: RasterMarker>(&self, sun_angle: f64) -> Dfm<U> {
        let mut output = Dfm::new_like(self);
        let sun_elevation = std::f64::consts::FRAC_PI_4;
        let light = (
            sun_angle.cos() * sun_elevation.cos(),
            sun_angle.sin() * sun_elevation.cos(),
            sun_elevation.sin(),
        );
        for y in 0..self.height() {
            for x in 0..self.width() {
                let (normal_x, normal_y) = self.surface_normal_xy(y, x);
                let normal = (normal_x, normal_y, 1.);
                let length = (normal.0.powi(2) + normal.1.powi(2) + 1.).sqrt();
                output[(y, x)] =
                    ((normal.0 * light.0 + normal.1 * light.1 + light.2) / length).max(0.) as f32;
            }
        }
        output
    }

    /// Sobel-estimated elevation gradient in map coordinates.
    ///
    /// The first component is `dz/dx` for east-positive x and the second is
    /// `dz/dy` for north-positive y. Raster rows increase in the opposite
    /// direction to map y, which accounts for the different stencil signs.
    #[inline]
    fn sobel_gradient(&self, y: usize, x: usize) -> (f64, f64) {
        let top = y.saturating_sub(1);
        let bottom = (y + 1).min(self.height() - 1);
        let left = x.saturating_sub(1);
        let right = (x + 1).min(self.width() - 1);
        let cell = self.grid.cell_size_m;
        (
            (f64::from(self[(top, right)]) - f64::from(self[(top, left)])
                + 2. * f64::from(self[(y, right)])
                - 2. * f64::from(self[(y, left)])
                + f64::from(self[(bottom, right)])
                - f64::from(self[(bottom, left)]))
                / (8. * cell),
            (f64::from(self[(top, left)]) - f64::from(self[(bottom, left)])
                + 2. * f64::from(self[(top, x)])
                - 2. * f64::from(self[(bottom, x)])
                + f64::from(self[(top, right)])
                - f64::from(self[(bottom, right)]))
                / (8. * cell),
        )
    }

    #[inline]
    fn surface_normal_xy(&self, y: usize, x: usize) -> (f64, f64) {
        let (gradient_x, gradient_y) = self.sobel_gradient(y, x);
        (-gradient_x, -gradient_y)
    }
}

impl<T: TerrainRasterMarker> Dfm<T> {
    /// Smooth an elevation surface while retaining discontinuities between
    /// sufficiently different surface normals.
    ///
    /// The neighbourhood and vertical bound use physical units so the result
    /// does not depend on raster resolution. `U` names the derived terrain
    /// product and makes accidental reuse as the canonical DEM visible in the
    /// type system.
    pub fn feature_preserving_smooth_as<U: TerrainRasterMarker>(
        &self,
        parameters: TerrainSmoothing,
    ) -> Dfm<U> {
        let iterations = parameters.iterations.max(1);
        let max_normal_difference = if parameters.max_normal_difference_degrees.is_finite() {
            parameters.max_normal_difference_degrees.abs().min(60.)
        } else {
            15.
        };
        let threshold = f64::from(max_normal_difference).to_radians().cos();
        let width = self.width();
        let height = self.height();
        let cell = self.grid.cell_size_m;
        let radius_m = if parameters.radius_m.is_finite() {
            parameters.radius_m.abs()
        } else {
            cell
        };
        let radius = (radius_m / cell).ceil().max(1.) as isize;
        let max_elevation_change = if parameters.max_elevation_change_m.is_finite() {
            parameters.max_elevation_change_m.abs()
        } else {
            f32::MAX
        };
        let mut normals = vec![(0., 0.); width * height];
        for y in 0..height {
            for x in 0..width {
                normals[y * width + x] = self.surface_normal_xy(y, x);
            }
        }

        let mut smooth = vec![(0., 0.); normals.len()];
        for y in 0..height {
            for x in 0..width {
                let center = normals[y * width + x];
                let mut sum = (0., 0., 0.);
                for dy in -radius..=radius {
                    let row = (y as isize + dy).clamp(0, height as isize - 1) as usize;
                    for dx in -radius..=radius {
                        let column = (x as isize + dx).clamp(0, width as isize - 1) as usize;
                        let neighbor = normals[row * width + column];
                        let similarity = cosine_between_normals(center, neighbor);
                        if similarity > threshold {
                            let weight = (similarity - threshold).powi(2);
                            sum.0 += neighbor.0 * weight;
                            sum.1 += neighbor.1 * weight;
                            sum.2 += weight;
                        }
                    }
                }
                smooth[y * width + x] = if sum.2 > f64::EPSILON {
                    (sum.0 / sum.2, sum.1 / sum.2)
                } else {
                    center
                };
            }
        }

        let neighbors = [
            (-1, -1),
            (-1, 0),
            (-1, 1),
            (0, 1),
            (1, 1),
            (1, 0),
            (1, -1),
            (0, -1),
        ];
        let mut output = Dfm::<U>::new_like(self);
        output.field.copy_from_slice(&self.field);
        let mut next = output.clone();
        for _ in 0..iterations {
            for y in 0..height {
                for x in 0..width {
                    let center = smooth[y * width + x];
                    let mut weighted_height = 0.;
                    let mut weight_sum = 0.;
                    for &(dy, dx) in &neighbors {
                        let row = (y as isize + dy).clamp(0, height as isize - 1) as usize;
                        let column = (x as isize + dx).clamp(0, width as isize - 1) as usize;
                        let neighbor = smooth[row * width + column];
                        let similarity = cosine_between_normals(center, neighbor);
                        if similarity > threshold {
                            let weight = (similarity - threshold).powi(2);
                            let offset_x = dx as f64 * cell;
                            let offset_y = -dy as f64 * cell;
                            weighted_height += (f64::from(output[(row, column)])
                                + neighbor.0 * offset_x
                                + neighbor.1 * offset_y)
                                * weight;
                            weight_sum += weight;
                        }
                    }
                    if weight_sum > f64::EPSILON {
                        let source = self[(y, x)];
                        next[(y, x)] = ((weighted_height / weight_sum) as f32)
                            .clamp(source - max_elevation_change, source + max_elevation_change);
                    } else {
                        next[(y, x)] = output[(y, x)];
                    }
                }
            }
            std::mem::swap(&mut output, &mut next);
        }
        output
    }
}

impl Dfm<Elevation> {
    pub fn slope(&self) -> Dfm<Slope> {
        let mut output = Dfm::new_like(self);
        for y in 0..self.height() {
            for x in 0..self.width() {
                let (vertical, horizontal) = self.sobel_gradient(y, x);
                output[(y, x)] = (vertical.hypot(horizontal) / 2_f64.sqrt()) as f32;
            }
        }
        output
    }

    pub fn terrain_change(&self, contour_interval: f32) -> Dfm<TerrainChange> {
        let mut output = Dfm::new_like(self);
        let cell = self.grid.cell_size_m;
        let cell_squared = cell * cell;
        for y in 0..self.height() {
            let top = y.saturating_sub(1);
            let bottom = (y + 1).min(self.height() - 1);
            for x in 0..self.width() {
                let left = x.saturating_sub(1);
                let right = (x + 1).min(self.width() - 1);
                let center = f64::from(self[(y, x)]);
                let dx = (f64::from(self[(y, right)]) - f64::from(self[(y, left)])) / (2. * cell);
                let dy = (f64::from(self[(top, x)]) - f64::from(self[(bottom, x)])) / (2. * cell);
                let dxx = (f64::from(self[(y, right)]) - 2. * center + f64::from(self[(y, left)]))
                    / cell_squared;
                let dyy = (f64::from(self[(top, x)]) - 2. * center + f64::from(self[(bottom, x)]))
                    / cell_squared;
                let dxy = (f64::from(self[(top, right)])
                    - f64::from(self[(top, left)])
                    - f64::from(self[(bottom, right)])
                    + f64::from(self[(bottom, left)]))
                    / (4. * cell_squared);
                output[(y, x)] = dx.hypot(dy).hypot(
                    f64::from(contour_interval.abs())
                        * (dxx.powi(2) + 2. * dxy.powi(2) + dyy.powi(2)).sqrt(),
                ) as f32;
            }
        }
        output
    }

    pub fn interpolation_error_improvement(
        &self,
        with_formlines: &Self,
        without_formlines: &Self,
    ) -> Dfm<InterpolationErrorImprovement> {
        self.grid
            .ensure_compatible(&with_formlines.grid)
            .and_then(|_| self.grid.ensure_compatible(&without_formlines.grid))
            .expect("interpolation error requires matching grids");
        let mut output = Dfm::new_like(self);
        for i in 0..self.field.len() {
            output.field[i] = ((self.field[i] - without_formlines.field[i]).abs()
                - (self.field[i] - with_formlines.field[i]).abs())
            .max(0.);
        }
        output
    }

    pub fn hillshade(&self, sun_angle: f64) -> Dfm<Hillshade> {
        self.hillshade_as(sun_angle)
    }
}

impl<T: RasterMarker> Dfm<T> {
    // marching squares algorithm for extracting contours
    pub fn marching_squares(&self, level: f32) -> geo::MultiLineString {
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
        let mut contour_map = vec![usize::MAX; self.width() + 2].into_boxed_slice();

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

        // make an f32::MIN-padded proxy of self to avoid edge problems and close all contours
        let padded = DfmPaddedProxy::new(self);

        for yi in 0..self.height() + 1 {
            let ys = [yi, yi, yi + 1, yi + 1];
            for xi in 0..self.width() + 1 {
                let xs = [xi, xi + 1, xi + 1, xi];
                let map_address_lut = [xi, self.width() + 1, xi, self.width() + 1];

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
}

impl Dfm<HydroCorrected> {
    /// Grow level regions from seed coordinates on a hydrologically corrected
    /// elevation model.
    ///
    /// With `allow_water_fall` disabled, a cell belongs to a seed's region
    /// when its elevation differs from the seed elevation by at most
    /// `threshold`. With it enabled, all cells below the seed elevation plus
    /// the threshold are accepted. Eight-neighbour connectivity is used.
    pub fn flood_fill(
        &self,
        generators: Vec<geo::Coord>,
        threshold: f32,
        allow_water_fall: bool,
    ) -> Dfm<FloodFill> {
        let mut water_mask = Dfm::new_like(self);

        if !threshold.is_finite() || threshold < 0. {
            water_mask.field.fill(0.);
            return water_mask;
        }

        let mut visited_generation = vec![0_u32; self.field.len()];
        let mut generation = 0_u32;

        for generator in generators {
            let x = ((generator.x - self.grid.top_left.x) / self.grid.cell_size_m).round();
            let y = ((self.grid.top_left.y - generator.y) / self.grid.cell_size_m).round();
            if !x.is_finite()
                || !y.is_finite()
                || x < 0.
                || y < 0.
                || x >= self.width() as f64
                || y >= self.height() as f64
            {
                continue;
            }

            let seed_index = y as usize * self.width() + x as usize;
            // A seed already covered by an earlier seed has the same completed
            // region, so avoid traversing that region again for every seed
            // cell produced by the likelihood filter.
            if water_mask.field[seed_index] == 1. {
                continue;
            }

            let generator_value = self.field[seed_index];
            if !generator_value.is_finite() || generator_value == f32::MIN {
                continue;
            }

            generation = generation.wrapping_add(1);
            if generation == 0 {
                visited_generation.fill(0);
                generation = 1;
            }

            let mut queue = VecDeque::from([seed_index]);
            visited_generation[seed_index] = generation;

            while let Some(index) = queue.pop_front() {
                let elevation = self.field[index];
                if !elevation.is_finite() || elevation == f32::MIN {
                    continue;
                }

                let accepted = if allow_water_fall {
                    elevation <= generator_value + threshold
                } else {
                    (elevation - generator_value).abs() <= threshold
                };
                if !accepted {
                    continue;
                }

                water_mask.field[index] = 1.;
                let cell_y = index / self.width();
                let cell_x = index % self.height();
                let top = cell_y.saturating_sub(1);
                let bottom = (cell_y + 1).min(self.height() - 1);
                let left = cell_x.saturating_sub(1);
                let right = (cell_x + 1).min(self.width() - 1);

                for neighbour_y in top..=bottom {
                    for neighbour_x in left..=right {
                        let neighbour = neighbour_y * self.width() + neighbour_x;
                        if visited_generation[neighbour] != generation {
                            visited_generation[neighbour] = generation;
                            queue.push_back(neighbour);
                        }
                    }
                }
            }
        }

        for v in water_mask.field.iter_mut() {
            if *v == f32::MIN {
                *v = 0.;
            }
        }
        water_mask
    }
}

//fn cos_angle_between(a: (f64, f64), b: (f64, f64)) -> f64 {
fn cosine_between_normals(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 * b.0 + a.1 * b.1 + 1.)
        / ((a.0 * a.0 + a.1 * a.1 + 1.) * (b.0 * b.0 + b.1 * b.1 + 1.)).sqrt()
}

impl<T: RasterMarker> Index<(usize, usize)> for Dfm<T> {
    type Output = f32;

    #[inline]
    fn index(&self, (row, column): (usize, usize)) -> &Self::Output {
        &self.field[row * self.grid.width + column]
    }
}

impl<T: RasterMarker> IndexMut<(usize, usize)> for Dfm<T> {
    #[inline]
    fn index_mut(&mut self, (row, column): (usize, usize)) -> &mut Self::Output {
        &mut self.field[row * self.grid.width + column]
    }
}

struct DfmPaddedProxy<'a, T: RasterMarker> {
    inner: &'a Dfm<T>,
}

impl<'a, T: RasterMarker> DfmPaddedProxy<'a, T> {
    fn new(inner: &'a Dfm<T>) -> Self {
        Self { inner }
    }

    #[inline]
    fn index2coord(&self, row: usize, column: usize) -> geo::Coord {
        let cell_size = self.inner.grid.cell_size_m;
        geo::Coord {
            x: self.inner.grid.top_left.x - cell_size + column as f64 * cell_size,
            y: self.inner.grid.top_left.y + cell_size - row as f64 * cell_size,
        }
    }

    #[inline]
    fn vertex_interpolate(
        &self,
        edge: usize,
        xs: &[usize; 4],
        ys: &[usize; 4],
        level: f32,
    ) -> geo::Coord {
        let next = (edge + 1) % 4;
        let a = self[(ys[edge], xs[edge])];
        let b = self[(ys[next], xs[next])];
        let fraction = (f64::from(level) - f64::from(a)) / (f64::from(b) - f64::from(a));
        let a_coord = self.index2coord(ys[edge], xs[edge]);
        let cell = self.inner.grid.cell_size_m;

        geo::Coord {
            x: a_coord.x + cell * (xs[next] as isize - xs[edge] as isize) as f64 * fraction,
            y: a_coord.y + cell * (ys[edge] as isize - ys[next] as isize) as f64 * fraction,
        }
    }
}

impl<T: RasterMarker> Index<(usize, usize)> for DfmPaddedProxy<'_, T> {
    type Output = f32;

    fn index(&self, (row, column): (usize, usize)) -> &Self::Output {
        if row == 0
            || row == self.inner.height() + 1
            || column == 0
            || column == self.inner.width() + 1
        {
            &Self::Output::MIN
        } else {
            &self.inner[(row - 1, column - 1)]
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::MapLineString;
    use crate::raster::{ContourTerrain, DfmPixelBounds};
    use geo::BoundingRect;

    fn plane(grid: DfmGrid) -> Dfm<Elevation> {
        let mut raster = Dfm::new(grid);
        for y in 0..raster.height() {
            for x in 0..raster.width() {
                let coordinate = raster.index2coord(y, x);
                raster[(y, x)] = (2. * coordinate.x - 3. * coordinate.y + 7.) as f32;
            }
        }
        raster
    }

    #[test]
    fn terrain_smoothing_is_bounded_and_produces_a_named_derivative() {
        let grid = DfmGrid::new(9, 9, 0.5, geo::coord! { x: 0., y: 4. }).unwrap();
        let mut source = Dfm::<Elevation>::new(grid);
        for y in 0..source.height() {
            for x in 0..source.width() {
                source[(y, x)] = if (x + y) % 2 == 0 { 10.05 } else { 9.95 };
            }
        }

        let smoothed: Dfm<ContourTerrain> = source.feature_preserving_smooth_as(TerrainSmoothing {
            max_normal_difference_degrees: 60.,
            radius_m: 1.5,
            iterations: 3,
            max_elevation_change_m: 0.02,
        });

        assert!(
            smoothed
                .field
                .iter()
                .zip(&source.field)
                .any(|(&actual, &raw)| (actual - raw).abs() > 1e-4)
        );
        assert!(
            smoothed
                .field
                .iter()
                .zip(&source.field)
                .all(|(&actual, &raw)| (actual - raw).abs() <= 0.020_001)
        );
        assert_eq!(smoothed.grid, source.grid);
    }

    #[test]
    fn restriction_and_prolongation_preserve_a_plane() {
        let mut grid = DfmGrid::new(8, 8, 0.5, geo::coord! { x: 0., y: 4. }).unwrap();
        grid.inner = DfmPixelBounds {
            top: 2,
            bottom: 6,
            left: 2,
            right: 6,
        };
        let source = plane(grid.clone());
        let coarse_grid = grid.aligned_coarsened(1.).unwrap();
        let coarse = source.restrict_to(&coarse_grid).unwrap();
        assert_eq!(
            coarse.grid.inner,
            DfmPixelBounds {
                top: 1,
                bottom: 3,
                left: 1,
                right: 3,
            }
        );
        let restored = coarse.prolong_to(&grid).unwrap();
        for (&actual, &expected) in restored.field.iter().zip(&source.field) {
            assert!((actual - expected).abs() < 1e-5);
        }
        assert_eq!(restored.grid.inner, source.grid.inner);
    }

    #[test]
    fn marching_squares_closes_an_interior_ring() {
        let grid = DfmGrid::new(5, 5, 1., geo::coord! { x: 0., y: 4. }).unwrap();
        let mut raster = Dfm::<Elevation>::new(grid);
        raster.field.fill(0.);
        raster[(2, 2)] = 2.;
        let contours = raster.marching_squares(1.);
        assert_eq!(contours.0.len(), 1);
        assert!(contours.0[0].is_closed());
        assert_eq!(
            contours.0[0].coords().next(),
            contours.0[0].coords().next_back()
        );
    }

    #[test]
    fn marching_squares_preserves_extremum_winding() {
        let grid = DfmGrid::new(5, 5, 1., geo::coord! { x: 0., y: 4. }).unwrap();
        let mut hill = Dfm::<Elevation>::new(grid.clone());
        hill.field.fill(0.);
        hill[(2, 2)] = 2.;
        assert!(
            hill.marching_squares(1.).0[0]
                .line_string_signed_area()
                .is_some_and(|area| area > 0.)
        );

        let mut depression = Dfm::<Elevation>::new(grid);
        depression.field.fill(2.);
        depression[(2, 2)] = 0.;
        assert!(
            depression
                .marching_squares(1.)
                .iter()
                .any(|line| line.line_string_signed_area().is_some_and(|area| area < 0.))
        );
    }

    #[test]
    fn bilinear_sampling_recovers_plane_values() {
        let raster = plane(DfmGrid::new(4, 3, 1., geo::coord! { x: 5., y: 8. }).unwrap());
        let point = geo::coord! { x: 6.25, y: 7.5 };
        assert!(
            (raster.sample_bilinear(point).unwrap() - (2. * 6.25 - 3. * 7.5 + 7.) as f32).abs()
                < 1e-5
        );
    }

    #[test]
    fn sobel_gradient_and_surface_normal_use_map_coordinate_signs() {
        let raster = plane(DfmGrid::new(5, 5, 1., geo::coord! { x: 0., y: 4. }).unwrap());

        let gradient = raster.sobel_gradient(2, 2);
        assert!((gradient.0 - 2.).abs() < 1e-9);
        assert!((gradient.1 + 3.).abs() < 1e-9);

        let normal = raster.surface_normal_xy(2, 2);
        assert!((normal.0 + 2.).abs() < 1e-9);
        assert!((normal.1 - 3.).abs() < 1e-9);
    }

    #[test]
    fn rectangular_indexing_uses_grid_width() {
        let mut raster =
            Dfm::<Elevation>::new(DfmGrid::new(4, 3, 2., geo::coord! { x: 0., y: 0. }).unwrap());
        raster[(2, 3)] = 9.;
        assert_eq!(raster.field[11], 9.);
    }

    #[test]
    fn contour_coordinates_follow_rectangular_grid_and_cell_size() {
        let mut raster =
            Dfm::<Elevation>::new(DfmGrid::new(5, 3, 2., geo::coord! { x: 10., y: 20. }).unwrap());
        raster.field.fill(0.);
        raster[(1, 2)] = 2.;
        let bounds = raster.marching_squares(1.).bounding_rect().unwrap();
        assert_eq!(
            bounds,
            geo::Rect::new(
                geo::coord! { x: 13., y: 17. },
                geo::coord! { x: 15., y: 19. }
            )
        );
    }

    fn corrected_dtm(elevation: f32) -> Dfm<HydroCorrected> {
        let mut dtm = Dfm::new(DfmGrid::standard(geo::Coord { x: 100., y: 200. }));
        dtm.field.fill(elevation);
        dtm
    }

    #[test]
    fn flood_fill_grows_only_the_level_seed_region() {
        let mut dtm = corrected_dtm(10.);
        for y in 20..=22 {
            for x in 30..=32 {
                dtm[(y, x)] = 2. + (x - 30) as f32 * 0.02;
            }
        }

        let water = dtm.flood_fill(vec![dtm.index2coord(21, 31)], 0.05, false);

        assert_eq!(water.field.iter().filter(|value| **value == 1.).count(), 9);
        assert_eq!(water[(20, 30)], 1.);
        assert_eq!(water[(22, 32)], 1.);
        assert_eq!(water[(19, 31)], 0.);
    }

    #[test]
    fn rejected_cell_can_still_act_as_a_later_seed() {
        let mut dtm = corrected_dtm(10.);
        dtm[(20, 20)] = 2.;
        dtm[(20, 21)] = 5.;

        let water = dtm.flood_fill(
            vec![dtm.index2coord(20, 20), dtm.index2coord(20, 21)],
            0.1,
            false,
        );

        assert_eq!(water[(20, 20)], 1.);
        assert_eq!(water[(20, 21)], 1.);
    }

    #[test]
    fn waterfall_mode_accepts_connected_lower_ground() {
        let mut dtm = corrected_dtm(10.);
        dtm[(20, 20)] = 5.;
        dtm[(20, 21)] = 4.;
        dtm[(20, 22)] = 3.;

        let level_water = dtm.flood_fill(vec![dtm.index2coord(20, 20)], 0.1, false);
        let falling_water = dtm.flood_fill(vec![dtm.index2coord(20, 20)], 0.1, true);

        assert_eq!(level_water[(20, 22)], 0.);
        assert_eq!(falling_water[(20, 22)], 1.);
    }

    #[test]
    fn flood_fill_ignores_invalid_generators_and_tolerances() {
        let dtm = corrected_dtm(10.);
        let outside = geo::Coord { x: -10., y: -10. };

        let outside_water = dtm.flood_fill(vec![outside], 0.1, false);
        let invalid_tolerance = dtm.flood_fill(vec![dtm.index2coord(10, 10)], -0.1, false);

        assert!(outside_water.field.iter().all(|value| *value == 0.));
        assert!(invalid_tolerance.field.iter().all(|value| *value == 0.));
    }

    #[test]
    fn contour_coordinates_support_portrait_grids_and_common_resolutions() {
        for &(width, height) in &[(7, 5), (5, 7)] {
            for &cell_size in &[0.5, 1., 2.] {
                let mut raster = Dfm::<Elevation>::new(
                    DfmGrid::new(width, height, cell_size, geo::coord! { x: 100., y: 200. })
                        .unwrap(),
                );
                raster.field.fill(0.);
                let (row, column) = (height / 2, width / 2);
                raster[(row, column)] = 2.;
                let center = raster.index2coord(row, column);
                let bounds = raster.marching_squares(1.).bounding_rect().unwrap();
                assert!(
                    (bounds.min().x - (center.x - cell_size / 2.)).abs() < 1e-9
                        && (bounds.max().x - (center.x + cell_size / 2.)).abs() < 1e-9
                        && (bounds.min().y - (center.y - cell_size / 2.)).abs() < 1e-9
                        && (bounds.max().y - (center.y + cell_size / 2.)).abs() < 1e-9
                );
            }
        }
    }

    #[test]
    fn padding_closes_boundary_touching_contours() {
        let grid = DfmGrid::new(5, 4, 1., geo::coord! { x: 0., y: 3. }).unwrap();
        let mut raster = Dfm::<Elevation>::new(grid);
        raster.field.fill(0.);
        raster[(0, 0)] = 2.;
        raster[(0, 1)] = 2.;
        let contours = raster.marching_squares(1.);
        assert!(!contours.0.is_empty());
        assert!(contours.iter().all(geo::LineString::is_closed));
    }
}
