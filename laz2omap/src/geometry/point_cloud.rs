use super::{PointLaz, PointTrait};
use crate::matrix::{Matrix32x6, Vector32, Vector6};
use crate::raster::FieldType;

use crate::{CELL_SIZE, TILE_SIZE};

use geo::{Coord, LineString, Simplify};
use las::point::Classification;
use las::{Bounds, Vector};
use std::cmp::Ordering;

#[derive(Clone)]
pub struct PointCloud {
    pub points: Vec<PointLaz>,
    pub bounds: Bounds,
}

impl PointCloud {
    pub fn new(v: Vec<PointLaz>, b: Bounds) -> Self {
        Self {
            points: v,
            bounds: b,
        }
    }

    pub fn add(&mut self, v: Vec<PointLaz>) {
        self.points.extend(v);
    }

    pub fn to_2d_slice(&self) -> Vec<[f64; 2]> {
        self.points.iter().map(|p| [p.x, p.y]).collect()
    }

    pub fn get_dfm_dimensions(&self) -> Bounds {
        let dx = self.bounds.max.x - self.bounds.min.x;
        let dy = self.bounds.max.y - self.bounds.min.y;

        // small but non-zero for some odd reason
        // stretch or shrink the bounds to fit
        // to TILESIZE exactly
        let stretch_x = (TILE_SIZE - dx) / 2.;
        let stretch_y = (TILE_SIZE - dy) / 2.;

        // because the top-left corner of every cell is queried
        // shift the dem over so top left corner of the first and last
        // cell in both dimensions are equally far from self.bounds
        // i.e shift by half the cell size
        // positive in x as left is min_x -> need to increase to shift
        // negative in y as top is max_y -> need to decrease to shift
        let offset_x = CELL_SIZE / 2.;
        let offset_y = -CELL_SIZE / 2.;

        Bounds {
            min: Vector {
                x: self.bounds.min.x - stretch_x + offset_x,
                y: self.bounds.min.y - stretch_y + offset_y,
                z: self.bounds.min.z,
            },
            max: Vector {
                x: self.bounds.max.x + stretch_x + offset_x,
                y: self.bounds.max.y + stretch_y + offset_y,
                z: self.bounds.max.z,
            },
        }
    }

    pub fn bounded_convex_hull(&mut self, dfm_bounds: &Bounds, epsilon: f64) -> LineString {
        let convex_hull = self.convex_hull();
        let mut hull_contour: LineString = LineString::new(vec![]);

        for mut point in convex_hull {
            if (dfm_bounds.min.x - point.x).abs() <= epsilon {
                point.x = dfm_bounds.min.x;
            } else if (dfm_bounds.max.x - point.x).abs() <= epsilon {
                point.x = dfm_bounds.max.x;
            }
            if (dfm_bounds.min.y - point.y).abs() <= epsilon {
                point.y = dfm_bounds.min.y;
            } else if (dfm_bounds.max.y - point.y).abs() <= epsilon {
                point.y = dfm_bounds.max.y;
            }

            hull_contour.0.push(point.flatten().into());
        }
        hull_contour.close();

        hull_contour.simplify(&epsilon)
    }

    fn convex_hull(&mut self) -> Vec<PointLaz> {
        let mut gp_iter = self
            .points
            .iter()
            .filter(|p| p.classification == Classification::Ground);

        let mut bottom_point = gp_iter.next().unwrap().clone();
        for point in gp_iter {
            if point.y < bottom_point.y || (point.y == bottom_point.y && point.x < bottom_point.x) {
                bottom_point = point.clone();
            }
        }

        let point_compare_angle = |a: &PointLaz, b: &PointLaz| -> Ordering {
            let orientation = bottom_point.consecutive_orientation(a, b);
            if orientation < 0.0 {
                Ordering::Greater
            } else if orientation > 0.0 {
                Ordering::Less
            } else {
                let a_dist = bottom_point.squared_euclidean_distance(a);
                let b_dist = bottom_point.squared_euclidean_distance(b);
                b_dist.partial_cmp(&a_dist).unwrap_or(Ordering::Equal)
            }
        };
        self.points.sort_by(point_compare_angle);

        let mut convex_hull: Vec<PointLaz> = vec![];

        convex_hull.push(bottom_point.clone());

        let mut gp_iter = self
            .points
            .iter()
            .skip(1)
            .filter(|p| p.classification == Classification::Ground);
        convex_hull.push(gp_iter.next().unwrap().clone());

        for point in gp_iter {
            if bottom_point.consecutive_orientation(point, &convex_hull[convex_hull.len() - 1])
                == 0.0
            {
                continue;
            }
            while convex_hull.len() > 2 {
                // If segment(i, i+1) turns right relative to segment(i-1, i), point(i) is not part of the convex hull.
                let orientation = convex_hull[convex_hull.len() - 2]
                    .consecutive_orientation(&convex_hull[convex_hull.len() - 1], point);
                if orientation <= 0.0 {
                    convex_hull.pop();
                } else {
                    break;
                }
            }
            convex_hull.push(point.clone());
        }
        convex_hull
    }

    pub fn interpolate_field(
        &self,
        field: FieldType,
        neighbours: &Vec<usize>,
        point: &Coord,
        smoothing: f64,
    ) -> (f64, f64) {
        let nrows = neighbours.len();

        let mut mean: [f64; 3] = [0., 0., 0.];
        for n in neighbours {
            mean[0] += self.points[*n].x;
            mean[1] += self.points[*n].y;

            match field {
                FieldType::Elevation => mean[2] += self.points[*n].z,
                FieldType::ReturnNumber => mean[2] += self.points[*n].return_number as f64,
                FieldType::Intensity => mean[2] += self.points[*n].intensity as f64,
            }
        }
        mean = [
            mean[0] / nrows as f64,
            mean[1] / nrows as f64,
            mean[2] / nrows as f64,
        ];

        let mut std: [f64; 3] = [0., 0., 0.];
        for n in neighbours {
            std[0] += (self.points[*n].x - mean[0]).powi(2);
            std[1] += (self.points[*n].y - mean[1]).powi(2);

            match field {
                FieldType::Elevation => std[2] += (self.points[*n].z - mean[2]).powi(2),
                FieldType::ReturnNumber => {
                    std[2] += (self.points[*n].return_number as f64 - mean[2]).powi(2)
                }
                FieldType::Intensity => {
                    std[2] += (self.points[*n].intensity as f64 - mean[2]).powi(2)
                }
            }
        }
        std = [
            (std[0] / nrows as f64).sqrt(),
            (std[1] / nrows as f64).sqrt(),
            (std[2] / nrows as f64).sqrt(),
        ];

        if std[2] < 0.01 {
            return (mean[2], 0.0);
        }

        let mut xy: Matrix32x6 = Matrix32x6::zeros();
        let mut z: Vector32 = Vector32::zeros();
        for (i, n) in neighbours.iter().enumerate() {
            let x = (self.points[*n].x - mean[0]) / std[0];
            let y = (self.points[*n].y - mean[1]) / std[1];

            xy.insert_row([1.0, x, y, x * x, y * y, x * y], i);

            match field {
                FieldType::Elevation => z.data[i] = (self.points[*n].z - mean[2]) / std[2],
                FieldType::ReturnNumber => {
                    z.data[i] = (self.points[*n].return_number as f64 - mean[2]) / std[2]
                }
                FieldType::Intensity => {
                    z.data[i] = (self.points[*n].intensity as f64 - mean[2]) / std[2]
                }
            }
        }

        // slow matrix inversion
        let beta: Vector6 = (xy.tdot_self().add_to_diag(smoothing))
            .inverse_spd()
            .dot_vec(xy.tdot_vec(&z));

        let nx = (point.x - mean[0]) / std[0];
        let ny = (point.y - mean[1]) / std[1];

        let x0 = Vector6::new([1.0, nx, ny, nx * nx, ny * ny, nx * ny]);
        let dx = Vector6::new([0.0, 1.0, 0.0, 2.0 * nx, 0.0, ny]);
        let dy = Vector6::new([0.0, 0.0, 1.0, 0.0, 2.0 * ny, nx]);

        let value = x0.dot(&beta) * std[2] + mean[2];
        let gradient_x = dx.dot(&beta) * std[2] / std[0];
        let gradient_y = dy.dot(&beta) * std[2] / std[1];
        let gradient_size = (gradient_x.powi(2) + gradient_y.powi(2)).sqrt();

        (value, gradient_size)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_dfm_dimensions() {
        let bounds = Bounds {
            min: Vector {
                x: 0.,
                y: 0.,
                z: 0.,
            },
            max: Vector {
                x: TILE_SIZE - 0.01,
                y: TILE_SIZE + 0.01,
                z: 0.,
            },
        };

        let pc = PointCloud::new(vec![], bounds);

        let dfm_bounds = pc.get_dfm_dimensions();

        let expected = Bounds {
            min: Vector {
                x: CELL_SIZE / 2.,
                y: -CELL_SIZE / 2.,
                z: 0.,
            },
            max: Vector {
                x: TILE_SIZE + CELL_SIZE / 2.,
                y: TILE_SIZE - CELL_SIZE / 2.,
                z: 0.,
            },
        };

        assert!(
            ((dfm_bounds.max.x - expected.max.x).powi(2)
                + (dfm_bounds.min.x - expected.min.x).powi(2))
            .abs()
                < 0.01
        );
        assert!(
            ((dfm_bounds.max.y - expected.max.y).powi(2)
                + (dfm_bounds.min.y - expected.min.y).powi(2))
            .abs()
                < 0.01
        );
    }
}
