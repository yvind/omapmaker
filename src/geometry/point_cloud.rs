use super::{Line, Point, Point2D, PointLaz};
use crate::dfm::FieldType;
use crate::matrix::{Matrix32x6, Vector32, Vector6};

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

    pub fn to_2d_slice(&self) -> Vec<[f64; 2]> {
        self.points.iter().map(|p| [p.x, p.y]).collect()
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn get_dfm_dimensions(&self, cell_size: f64) -> (usize, usize, Bounds) {
        let dx = self.bounds.max.x - self.bounds.min.x;
        let dy = self.bounds.max.y - self.bounds.min.y;

        let width = (dx / cell_size).trunc() + 1.;
        let height = (dy / cell_size).trunc() + 1.;

        let offset_x = (width * cell_size - dx) / 2.;
        let offset_y = (height * cell_size - dy) / 2.;

        let dfm_bounds = Bounds {
            min: Vector {
                x: self.bounds.min.x - offset_x,
                y: self.bounds.min.y - offset_y,
                z: self.bounds.min.z,
            },
            max: Vector {
                x: self.bounds.max.x + offset_x,
                y: self.bounds.max.y + offset_y,
                z: self.bounds.max.z,
            },
        };
        (width as usize + 1, height as usize + 1, dfm_bounds)
    }

    pub fn bounded_convex_hull(
        &mut self,
        cell_size: f64,
        dfm_bounds: &Bounds,
        epsilon: f64,
    ) -> Line {
        let convex_hull = self.convex_hull();
        let mut hull_contour: Line = Line { vertices: vec![] };

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

            hull_contour.push(point.into())
        }
        hull_contour.close();

        hull_contour.simplify(cell_size);
        hull_contour
    }

    fn convex_hull(&mut self) -> Vec<PointLaz> {
        let point_compare_position = |a: &PointLaz, b: &PointLaz| -> Ordering {
            if a.y == b.y {
                a.x.partial_cmp(&b.x).unwrap_or(Ordering::Equal)
            } else {
                a.y.partial_cmp(&b.y).unwrap_or(Ordering::Equal)
            }
        };

        self.points.sort_by(point_compare_position);

        let mut most_south_west_point = PointLaz::new(0., 0., 0., 0, 0, 0, 0);
        for point in self.points.iter() {
            if point.c == 2 {
                most_south_west_point = point.clone();
                break;
            }
        }
        if most_south_west_point.n == 0 {
            panic!("No ground points in the pointcloud");
        }

        let point_compare_angle = |a: &PointLaz, b: &PointLaz| -> Ordering {
            let orientation = most_south_west_point.consecutive_orientation(a, b);
            if orientation < 0.0 {
                Ordering::Greater
            } else if orientation > 0.0 {
                Ordering::Less
            } else {
                let a_dist = most_south_west_point.squared_euclidean_distance(a);
                let b_dist = most_south_west_point.squared_euclidean_distance(b);
                b_dist.partial_cmp(&a_dist).unwrap_or(Ordering::Equal)
            }
        };
        self.points.sort_by(point_compare_angle);

        let mut convex_hull: Vec<PointLaz> = vec![];

        convex_hull.push(most_south_west_point.clone());

        let mut skip_to = 1;
        for (i, point) in self.points.iter().skip(0).enumerate() {
            if point.c == 2 {
                convex_hull.push(point.clone());
                skip_to = i;
                break;
            }
        }

        let mut hull_head = 1;
        for point in self.points.iter().skip(skip_to) {
            if point.c != 2 {
                continue;
            }
            if most_south_west_point.consecutive_orientation(point, &convex_hull[hull_head]) == 0.0
            {
                continue;
            }
            while hull_head > 1 {
                // If segment(i, i+1) turns right relative to segment(i-1, i), point(i) is not part of the convex hull.
                let orientation = convex_hull[hull_head - 1]
                    .consecutive_orientation(&convex_hull[hull_head], point);
                if orientation <= 0.0 {
                    hull_head -= 1;
                    convex_hull.pop();
                } else {
                    break;
                }
            }
            convex_hull.push(point.clone());
            hull_head += 1;
        }
        convex_hull
    }

    pub fn interpolate_field(
        &self,
        field: FieldType,
        neighbours: &Vec<usize>,
        point: &Point2D,
        smoothing: f64,
    ) -> (f64, f64) {
        let nrows = neighbours.len();

        let mut mean: [f64; 3] = [0., 0., 0.];
        for n in neighbours {
            mean[0] += self.points[*n].x;
            mean[1] += self.points[*n].y;

            match field {
                FieldType::Elevation => mean[2] += self.points[*n].z,
                FieldType::ReturnNumber => mean[2] += self.points[*n].r as f64,
                FieldType::Intensity => mean[2] += self.points[*n].i as f64,
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
                FieldType::ReturnNumber => std[2] += (self.points[*n].r as f64 - mean[2]).powi(2),
                FieldType::Intensity => std[2] += (self.points[*n].i as f64 - mean[2]).powi(2),
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
                    z.data[i] = (self.points[*n].r as f64 - mean[2]) / std[2]
                }
                FieldType::Intensity => z.data[i] = (self.points[*n].i as f64 - mean[2]) / std[2],
            }
        }

        // slow matrix inversion
        let beta: Vector6 = (xy.tdot_self().add_to_diag(smoothing))
            .inverse_spd_simd2()
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
mod test {
    use super::*;

    fn setup() -> PointCloud {
        let b = Bounds {
            min: Vector {
                x: -2.0,
                y: -2.0,
                z: -1.0,
            },
            max: Vector {
                x: 1.99,
                y: 1.99,
                z: 0.99,
            },
        };

        let v = vec![
            PointLaz::new(-2.0, -1.23, 0., 1, 0, 2, 1),
            PointLaz::new(-1.0, 1.6, 0., 1, 0, 2, 1),
            PointLaz::new(-1.7, 0.2, 0., 1, 0, 2, 1),
            PointLaz::new(-1.3, -2.0, 0., 1, 0, 2, 1),
            PointLaz::new(0.6, 1.96, 0., 1, 0, 2, 1),
            PointLaz::new(0.2, -0.5, 0., 1, 0, 2, 1),
            PointLaz::new(0.8, -1.0, 0., 1, 0, 2, 1),
            PointLaz::new(1.1, 1.23, 0., 1, 0, 2, 1),
            PointLaz::new(1.6, -0.73, 0., 1, 0, 2, 1),
            PointLaz::new(1.9, 1.9, 0., 1, 0, 2, 1),
            PointLaz::new(1.91, -1.9, 0., 1, 0, 2, 1),
            PointLaz::new(-1.1, -2.0, 0., 1, 0, 2, 1),
        ];

        PointCloud::new(v, b)
    }

    #[test]
    fn dfm_dimensions() {
        let pc = setup();

        let (w, h, b) = pc.get_dfm_dimensions(0.1);

        let true_b = Bounds {
            min: Vector {
                x: -2.005,
                y: -2.005,
                z: -1.,
            },
            max: Vector {
                x: 1.994999,
                y: 1.994999,
                z: 0.99,
            },
        };

        let diff_abs = (b.min.x - true_b.min.x).abs()
            + (b.min.y - true_b.min.y).abs()
            + (b.max.x - true_b.max.x).abs()
            + (b.max.y - true_b.max.y).abs();

        assert_eq!(w, 41);
        assert_eq!(h, 41);
        assert!(diff_abs < 0.1);
    }

    #[test]
    fn create_convex_hull() {
        let mut pc = setup();
        let cs = 0.1;

        let (_, _, b) = pc.get_dfm_dimensions(cs);

        let hull = pc.bounded_convex_hull(cs, &b, 0.05);

        assert_eq!(hull.vertices[0], Point2D::new(-1.3, -2.005));
        assert_eq!(hull.vertices[0], hull.vertices[hull.len() - 1]);

        assert_eq!(hull.vertices[1], Point2D::new(1.91, -1.9));

        assert_eq!(hull.len(), 8);
    }
}
