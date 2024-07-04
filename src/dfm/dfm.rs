use crate::geometry::{Line, Point2D};

use rustc_hash::FxHashMap as HashMap;
use std::{fs::File, io::BufWriter, io::Write};
use tiff::encoder::{colortype::Gray64Float, TiffEncoder};

#[derive(Clone, Debug)]
pub struct Dfm {
    pub field: Vec<Vec<f64>>,
    pub height: usize,
    pub width: usize,
    pub tl_coord: Point2D,
    pub cell_size: f64,
}

impl Dfm {
    pub fn new(width: usize, height: usize, tl_coord: Point2D, cell_size: f64) -> Dfm {
        return Dfm {
            field: vec![vec![f64::NAN; width]; height],
            height,
            width,
            tl_coord,
            cell_size,
        };
    }

    pub fn difference(&self, other: &Dfm) -> Result<Dfm, &'static str> {
        if self.height != other.height || self.width != other.width {
            return Err("DFM dimensions don't match!");
        }
        let mut diff = Dfm::new(self.width, self.height, self.tl_coord, self.cell_size);
        for y in 0..self.height {
            for x in 0..self.width {
                diff.field[y][x] = self.field[y][x] - other.field[y][x];
            }
        }
        return Ok(diff);
    }

    pub fn index2coord(&self, xi: usize, yi: usize) -> Result<Point2D, &'static str> {
        if xi >= self.width || yi >= self.height {
            return Err("Index out of bounds for DFM coordinate");
        }
        return Ok(Point2D {
            x: xi as f64 * self.cell_size + self.tl_coord.x,
            y: self.tl_coord.y - yi as f64 * self.cell_size,
        });
    }

    pub fn adjust(
        &mut self,
        truth: &Dfm,
        interpolated: &Dfm,
        weigth: f64,
    ) -> Result<(), &'static str> {
        let diff = truth.difference(interpolated)?;
        for y in 0..self.height {
            for x in 0..self.width {
                self.field[y][x] += diff.field[y][x] * weigth;
            }
        }
        return Ok(());
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
        let dem = &self.field;
        let mut contour_by_end: HashMap<Point2D, Line> = HashMap::default();
        let mut contour_by_start: HashMap<Point2D, Line> = HashMap::default();
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

        for yi in 0..self.height - 1 {
            let ys = [yi, yi, yi + 1, yi + 1];

            for xi in 0..self.width - 1 {
                let xs = [xi, xi + 1, xi + 1, xi];

                if dem[ys[0]][xs[0]].is_nan()
                    || dem[ys[1]][xs[1]].is_nan()
                    || dem[ys[1]][xs[1]].is_nan()
                    || dem[ys[1]][xs[1]].is_nan()
                {
                    continue;
                }
                let index = (dem[ys[0]][xs[0]] >= level) as usize
                    + 2 * (dem[ys[1]][xs[1]] >= level) as usize
                    + 4 * (dem[ys[2]][xs[2]] >= level) as usize
                    + 8 * (dem[ys[3]][xs[3]] >= level) as usize;

                let edge_indices: Vec<usize>;
                if index == 0 || index == 15 {
                    continue;
                } else if index == 5 {
                    let dr = (dem[ys[0]][xs[0]] + dem[ys[2]][xs[2]]) / 2.; // above
                    let dl = (dem[ys[1]][xs[1]] + dem[ys[3]][xs[3]]) / 2.; // below

                    if (dr - level).abs() > (dl - level).abs() {
                        edge_indices = vec![3, 0, 1, 2];
                    } else {
                        edge_indices = vec![1, 0, 3, 2];
                    }
                } else if index == 10 {
                    let dr = (dem[ys[0]][xs[0]] + dem[ys[2]][xs[2]]) / 2.; // below
                    let dl = (dem[ys[1]][xs[1]] + dem[ys[3]][xs[3]]) / 2.; // above

                    if (dr - level).abs() > (dl - level).abs() {
                        edge_indices = vec![0, 3, 2, 1];
                    } else {
                        edge_indices = vec![0, 1, 2, 3];
                    }
                } else {
                    edge_indices = lut[index].to_vec();
                }

                let mut coordinates: [Point2D; 2] = [Point2D { x: 0.0, y: 0.0 }; 2];
                for (i, e) in edge_indices.iter().enumerate() {
                    let a = dem[ys[*e]][xs[*e]];
                    let b = dem[ys[(*e + 1) % 4]][xs[(*e + 1) % 4]];

                    let xy: Point2D = self.index2coord(xs[*e], ys[*e])?;

                    coordinates[i % 2].x = xy.x
                        + self.cell_size
                            * (xs[(*e + 1) % 4] as i32 - xs[*e] as i32) as f64
                            * (level - a)
                            / (b - a);
                    coordinates[i % 2].y = xy.y
                        + self.cell_size
                            * (ys[*e] as i32 - ys[(*e + 1) % 4] as i32) as f64
                            * (level - a)
                            / (b - a);

                    if i % 2 == 1 {
                        let vertex1 = coordinates[0];
                        let vertex2 = coordinates[1];

                        if contour_by_end.contains_key(&vertex1)
                            && contour_by_start.contains_key(&vertex2)
                        {
                            // join two existing contours

                            let mut contour: Line = contour_by_end.remove(&vertex1).unwrap();
                            let mut contour2: Line = contour_by_start.remove(&vertex2).unwrap();

                            if contour == contour2 {
                                // close a contour (joining a contour with it self)
                                contour.close();
                                contour_by_end.insert(vertex2, contour);
                            } else {
                                // join two different contours
                                contour.append(contour2);

                                let end_vertex = contour.last_vertex();
                                let start_vertex = contour.first_vertex();

                                contour_by_end.remove(end_vertex).unwrap(); // unwrapping to cause a panic if logic fails
                                contour_by_start.remove(start_vertex).unwrap();

                                contour_by_end.insert(*end_vertex, contour.clone());
                                contour_by_start.insert(*start_vertex, contour);
                            }
                        } else if let Some(mut contour) = contour_by_end.remove(&vertex1) {
                            // append to an existing contour
                            contour.push(vertex2.clone());

                            let start_vertex = contour.first_vertex();
                            contour_by_start.remove(start_vertex).unwrap();

                            contour_by_end.insert(vertex2, contour.clone());
                            contour_by_start.insert(*start_vertex, contour);
                        } else if let Some(mut contour) = contour_by_start.remove(&vertex2) {
                            // prepend to an existing contour
                            contour.prepend(vertex1.clone());

                            let end_vertex = contour.last_vertex();
                            contour_by_end.remove(end_vertex).unwrap();

                            contour_by_start.insert(vertex1, contour.clone());
                            contour_by_end.insert(*end_vertex, contour);
                        } else if !contour_by_end.contains_key(&vertex1)
                            && !contour_by_start.contains_key(&vertex2)
                        {
                            // start a new contour
                            let contour: Line = Line::new(vertex1.clone(), vertex2.clone());

                            contour_by_end.insert(vertex2, contour.clone());
                            contour_by_start.insert(vertex1, contour);
                        } else {
                            panic!("Contour generation failed. Logic error...");
                        }
                    }
                }
            }
        }
        return Ok(contour_by_end.into_values().collect());
    }

    pub fn write_to_tiff(&self, filename: String, output_directory: &str) {
        let tiff_path = format!("{}/{}.tiff", output_directory, filename);
        let tfw_path = format!("{}/{}.tfw", output_directory, filename);

        let mut tiff = File::create(tiff_path).expect("Unable to create tiff-file");
        let mut tiff = TiffEncoder::new(&mut tiff).unwrap();
        tiff.write_image::<Gray64Float>(
            self.width as u32,
            self.height as u32,
            &self
                .field
                .clone()
                .into_iter()
                .flatten()
                .collect::<Vec<f64>>(),
        )
        .expect("Cannot write tiff-file");

        let tfw = File::create(tfw_path).expect("Unable to create tfw-file");
        let mut tfw = BufWriter::new(tfw);
        tfw.write(
            format!(
                "{}\n0\n0\n-{}\n{}\n{}",
                self.cell_size, self.cell_size, self.tl_coord.x, self.tl_coord.y
            )
            .as_bytes(),
        )
        .expect("Cannot write tfw-file");
    }
}
