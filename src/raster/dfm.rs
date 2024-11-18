#![allow(dead_code)]

use crate::geometry::{Coord, LineString, MultiLineString};
use crate::{CELL_SIZE, INV_CELL_SIZE_USIZE, TILE_SIZE_USIZE};

const SIDE_LENGTH: usize = INV_CELL_SIZE_USIZE * TILE_SIZE_USIZE;

use std::ops::{Index, IndexMut};
use std::{
    ffi::OsString,
    fs::File,
    io::{BufWriter, Write},
    path::{Path, PathBuf},
};
use tiff::encoder::{colortype::Gray32Float, TiffEncoder};

#[derive(Clone, Debug)]
pub struct Dfm {
    pub field: [f64; SIDE_LENGTH * SIDE_LENGTH],
    pub tl_coord: Coord,
}

impl Dfm {
    pub fn hint_value(&self) -> Option<&f64> {
        if self.field[self.field.len() / 2] > f64::MIN {
            return Some(&self.field[self.field.len() / 2]);
        }
        self.field.iter().find(|&f| f > &f64::MIN)
    }

    pub fn new(tl_coord: Coord) -> Dfm {
        Dfm {
            field: [f64::MIN; SIDE_LENGTH * SIDE_LENGTH],
            tl_coord,
        }
    }

    pub fn difference(&self, other: &Dfm) -> Result<Dfm, &'static str> {
        let mut diff = Dfm::new(self.tl_coord);
        for y in 0..SIDE_LENGTH {
            for x in 0..SIDE_LENGTH {
                diff[(y, x)] = self[(y, x)] - other[(y, x)];
            }
        }
        Ok(diff)
    }

    #[inline]
    pub fn index2coord(&self, xi: usize, yi: usize) -> Coord {
        Coord {
            x: (xi as f64) * CELL_SIZE + self.tl_coord.x,
            y: self.tl_coord.y - (yi as f64) * CELL_SIZE,
        }
    }

    pub fn adjust(
        &mut self,
        truth: &Dfm,
        interpolated: &Dfm,
        weigth: f64,
    ) -> Result<(), &'static str> {
        let diff = truth.difference(interpolated)?;
        for y in 0..SIDE_LENGTH {
            for x in 0..SIDE_LENGTH {
                self[(y, x)] += diff[(y, x)] * weigth;
            }
        }
        Ok(())
    }

    pub fn marching_squares(&self, level: f64) -> MultiLineString {
        // should preallocate some memory, but how much? How many contours can be expected to be created?
        let mut contours: Vec<LineString> = Vec::with_capacity(32);

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

        // make a padded proxy of self to avoid edge problems and close all contours
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

    pub fn write_to_tiff(self, filename: &OsString, output_directory: &Path, ref_point: &Coord) {
        let mut tiff_path = PathBuf::from(output_directory);
        tiff_path.push(filename);
        tiff_path.set_extension("tiff");

        let mut tfw_path = PathBuf::from(output_directory);
        tfw_path.push(filename);
        tfw_path.set_extension("tfw");

        let mut tiff = File::create(tiff_path).expect("Unable to create tiff-file");
        let mut tiff = TiffEncoder::new(&mut tiff).unwrap();

        let data = self.field.map(|d| d as f32);

        tiff.write_image::<Gray32Float>(SIDE_LENGTH as u32, SIDE_LENGTH as u32, &data)
            .expect("Cannot write tiff-file");

        let tfw = File::create(tfw_path).expect("Unable to create tfw-file");
        let mut tfw = BufWriter::new(tfw);
        tfw.write_all(
            format!(
                "{}\n0\n0\n-{}\n{}\n{}",
                CELL_SIZE,
                CELL_SIZE,
                self.tl_coord.x + ref_point.x,
                self.tl_coord.y + ref_point.y
            )
            .as_bytes(),
        )
        .expect("Cannot write tfw-file");
    }
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
    fn index2coord(&self, xi: usize, yi: usize) -> Coord {
        Coord {
            x: self.inner.tl_coord.x - CELL_SIZE + (xi as f64) * CELL_SIZE,
            y: self.inner.tl_coord.y + CELL_SIZE - (yi as f64) * CELL_SIZE,
        }
    }

    #[inline]
    fn vertex_interpolate(&self, e: usize, xs: &[usize; 4], ys: &[usize; 4], level: f64) -> Coord {
        let a = self[(ys[e], xs[e])];
        let b = self[(ys[(e + 1) % 4], xs[(e + 1) % 4])];

        let a_coord = self.index2coord(xs[e], ys[e]);

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

impl<'a> Index<(usize, usize)> for DfmPaddedProxy<'a> {
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
