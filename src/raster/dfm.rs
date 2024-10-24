#![allow(dead_code)]

use crate::geometry::{Line, Point2D};
use crate::{CELL_SIZE, INV_CELL_SIZE_USIZE, TILE_SIZE, TILE_SIZE_USIZE};

const SIDE_LENGTH: usize = INV_CELL_SIZE_USIZE * TILE_SIZE_USIZE;
const NUM_EDGES: usize = 2 * SIDE_LENGTH * (SIDE_LENGTH + 1);

use std::ops::{Index, IndexMut};
use std::{
    ffi::OsString,
    fs::File,
    io::{BufWriter, Write},
    path::{Path, PathBuf},
};
use tiff::encoder::{colortype::Gray32Float, TiffEncoder};

pub enum Edges {
    Top = 0,
    Right = 1,
    Bottom = 2,
    Left = 3,
}

#[derive(Clone, Debug)]
pub struct Dfm {
    pub field: [f64; SIDE_LENGTH * SIDE_LENGTH],
    pub tl_coord: Point2D,
}

impl Dfm {
    pub fn new(tl_coord: Point2D) -> Dfm {
        Dfm {
            field: [f64::NAN; SIDE_LENGTH * SIDE_LENGTH],
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

    pub fn index2coord(&self, xi: usize, yi: usize) -> Result<Point2D, &'static str> {
        assert!(xi < SIDE_LENGTH);
        assert!(yi < SIDE_LENGTH);

        Ok(Point2D {
            x: (xi as f64 + 0.5) * CELL_SIZE + self.tl_coord.x,
            y: self.tl_coord.y - (yi as f64 + 0.5) * CELL_SIZE,
        })
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

    fn get_edge_index(&self, point: &Point2D) -> Result<usize, ()> {
        let p0 = Point2D::new(point.x - self.tl_coord.x, self.tl_coord.y - point.y);

        assert!(p0.x >= 0.);
        assert!(p0.y >= 0.);
        assert!(p0.x <= TILE_SIZE);
        assert!(p0.y <= TILE_SIZE);

        let mut dx = p0.x / CELL_SIZE;
        let mut dy = p0.y / CELL_SIZE;

        let x = dx.trunc();
        let y = dy.trunc();

        let xi = x as usize;
        let yi = y as usize;

        dx -= x + 0.5;
        dy -= y + 0.5;

        let ei;
        if dx > 0. && dy > 0. {
            // right or top edge
            if dx > dy {
                ei = Edges::Right;
            } else {
                ei = Edges::Top;
            }
        } else if dx > 0. {
            // right or bottom edge
            if dx > dy.abs() {
                ei = Edges::Right;
            } else {
                ei = Edges::Bottom;
            }
        } else if dy > 0. {
            // top or left edge
            if dy > dx.abs() {
                ei = Edges::Top;
            } else {
                ei = Edges::Left;
            }
        } else {
            // bottom or left edge
            if dy.abs() > dx.abs() {
                ei = Edges::Bottom;
            } else {
                ei = Edges::Left;
            }
        }

        match ei {
            Edges::Top => Ok(yi * (2 * SIDE_LENGTH + 1) + xi),
            Edges::Right => Ok(yi * (2 * SIDE_LENGTH + 1) + (xi + 1) + SIDE_LENGTH),
            Edges::Bottom => Ok((yi + 1) * (2 * SIDE_LENGTH + 1) + xi),
            Edges::Left => Ok(yi * (2 * SIDE_LENGTH + 1) + xi + SIDE_LENGTH),
        }
    }

    pub fn marching_squares(&self, level: f64) -> Result<Vec<Line>, &'static str> {
        /*
            0       1
            *-------*   index into the lut based on the sum of (c > level)*2^i for the corner value c at all corner indecies i
            |       |   the lut gives which directed edge that should be crossed by the contour as corner indecies of the start and end corner
            |       |   performs linear interpolation based on the corner values of the crossed edges
            *-------*   [0, 0] is a special case corresponding to either no edge crossing or two edges should be crossed (handled seperately)
            3       2
        */

        // should preallocate some memory, but how much? How many contours can be expected to be created?
        let mut contours: Vec<Line> = Vec::new(); //with_capacity(32);

        // maps from edges to contour passing that edge in contours-vec, avoids hashmap overhead
        // is 1 MiB for SIDE_LENGTH = 256, needs to increase thread stack size
        let mut contour_map = [usize::MAX; NUM_EDGES];

        let lut: [[usize; 2]; 16] = [
            [0, 0],
            [3, 0],
            [0, 1],
            [3, 1],
            [1, 2],
            [0, 0],
            [0, 2],
            [3, 2],
            [2, 3],
            [2, 0],
            [0, 0],
            [2, 1],
            [1, 3],
            [1, 0],
            [0, 3],
            [0, 0],
        ];

        for yi in 0..SIDE_LENGTH - 1 {
            let ys = [yi, yi, yi + 1, yi + 1];

            for xi in 0..SIDE_LENGTH - 1 {
                let xs = [xi, xi + 1, xi + 1, xi];

                if self[(ys[0], xs[0])].is_nan()
                    || self[(ys[1], xs[1])].is_nan()
                    || self[(ys[2], xs[2])].is_nan()
                    || self[(ys[3], xs[3])].is_nan()
                {
                    continue;
                }
                let index = (self[(ys[0], xs[0])] >= level) as usize
                    + 2 * (self[(ys[1], xs[1])] >= level) as usize
                    + 4 * (self[(ys[2], xs[2])] >= level) as usize
                    + 8 * (self[(ys[3], xs[3])] >= level) as usize;

                let edge_indices: Vec<usize>;
                if index == 0 || index == 15 {
                    continue;
                } else if index == 5 {
                    let dr = (self[(ys[0], xs[0])] + self[(ys[2], xs[2])]) / 2.; // above
                    let dl = (self[(ys[1], xs[1])] + self[(ys[3], xs[3])]) / 2.; // below

                    if (dr - level).abs() > (dl - level).abs() {
                        edge_indices = vec![3, 0, 1, 2];
                    } else {
                        edge_indices = vec![1, 0, 3, 2];
                    }
                } else if index == 10 {
                    let dr = (self[(ys[0], xs[0])] + self[(ys[2], xs[2])]) / 2.; // below
                    let dl = (self[(ys[1], xs[1])] + self[(ys[3], xs[3])]) / 2.; // above

                    if (dr - level).abs() > (dl - level).abs() {
                        edge_indices = vec![0, 3, 2, 1];
                    } else {
                        edge_indices = vec![0, 1, 2, 3];
                    }
                } else {
                    edge_indices = lut[index].to_vec();
                }

                let mut vertex_coordinates: [Point2D; 2] = [Point2D { x: 0.0, y: 0.0 }; 2];
                for (i, e) in edge_indices.iter().enumerate() {
                    let a = self[(ys[*e], xs[*e])];
                    let b = self[(ys[(*e + 1) % 4], xs[(*e + 1) % 4])];

                    let a_coord = self.index2coord(xs[*e], ys[*e])?;

                    vertex_coordinates[i % 2].x = a_coord.x
                        + CELL_SIZE
                            * (xs[(*e + 1) % 4] as i32 - xs[*e] as i32) as f64
                            * (level - a)
                            / (b - a);
                    vertex_coordinates[i % 2].y = a_coord.y
                        + CELL_SIZE
                            * (ys[*e] as i32 - ys[(*e + 1) % 4] as i32) as f64
                            * (level - a)
                            / (b - a);

                    if i % 2 == 1 {
                        let vertex1 = vertex_coordinates[0];
                        let vertex2 = vertex_coordinates[1];

                        let key1 = self.get_edge_index(&vertex1).unwrap();
                        let key2 = self.get_edge_index(&vertex2).unwrap();

                        let mut end_contour_index = contour_map[key1];
                        let start_contour_index = contour_map[key2];

                        if end_contour_index != usize::MAX && start_contour_index != usize::MAX {
                            // join two existing contours
                            if end_contour_index == start_contour_index {
                                // close the contour (joining a contour with itself)
                                contours[end_contour_index].close();
                            } else {
                                // join two different contours
                                // do a swap remove on the start_contour_index and update map
                                // append the contour to the contour at end_contour_index
                                let contour = contours.swap_remove(start_contour_index);

                                // if end_contour_index was the last element it's new position
                                // is now start_contour_index after the swap_remove
                                if contours.len() == end_contour_index {
                                    end_contour_index = start_contour_index;
                                }
                                contours[end_contour_index].append(contour);

                                // get the index of the positions in the map that needs updating
                                // only first and last edge indecies of the contour needs updating
                                // as the "inner" edges of a contour should never be encountered again
                                let end_key = self
                                    .get_edge_index(contours[end_contour_index].last_vertex())
                                    .unwrap();
                                let start_key = self
                                    .get_edge_index(contours[end_contour_index].first_vertex())
                                    .unwrap();

                                contour_map[end_key] = end_contour_index;
                                contour_map[start_key] = end_contour_index;

                                // if the last element was removed no other update needs to be done
                                if contours.len() == start_contour_index {
                                    continue;
                                }
                                // get the index of the other affect contour in the map that needs updating
                                // only first and last edge indecies of the contour needs updating
                                // as the "inner" edges of a contour should never be encountered again

                                let end_key = self
                                    .get_edge_index(contours[start_contour_index].last_vertex())
                                    .unwrap();
                                let start_key = self
                                    .get_edge_index(contours[start_contour_index].first_vertex())
                                    .unwrap();

                                contour_map[end_key] = start_contour_index;
                                contour_map[start_key] = start_contour_index;
                            }
                        } else if end_contour_index != usize::MAX {
                            // append to an existing contour
                            contours[end_contour_index].push(vertex2);
                            // update map
                            contour_map[key2] = end_contour_index;
                        } else if start_contour_index != usize::MAX {
                            // prepend to an existing contour
                            contours[start_contour_index].prepend(vertex1);
                            // update map
                            contour_map[key1] = start_contour_index;
                        } else {
                            // start a new contour
                            let contour: Line = Line::new(vertex1, vertex2);
                            contours.push(contour);

                            contour_map[key1] = contours.len() - 1;
                            contour_map[key2] = contours.len() - 1;
                        }
                    }
                }
            }
        }
        Ok(contours)
    }

    pub fn write_to_tiff(self, filename: &OsString, output_directory: &Path, ref_point: &Point2D) {
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
        &self.field[index.0 * INV_CELL_SIZE_USIZE * TILE_SIZE_USIZE + index.1]
    }
}

impl IndexMut<(usize, usize)> for Dfm {
    fn index_mut(&mut self, index: (usize, usize)) -> &mut Self::Output {
        &mut self.field[index.0 * INV_CELL_SIZE_USIZE * TILE_SIZE_USIZE + index.1]
    }
}
