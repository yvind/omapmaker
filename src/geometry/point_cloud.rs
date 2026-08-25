use super::PointLaz;

use crate::{
    CELL_SIZE_METERS, TILE_SIZE_METERS,
    raster::{Dfm, Elevation, HeightAboveGround, PointDensity, RasterMarker},
};
use anyhow::{Context, bail};

use geo::Simplify;
use las::{Bounds, Vector, point::Classification};
use std::{cmp::Ordering, collections::HashSet, ops::Index};

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
        self.points.iter().map(|p| [p.x(), p.y()]).collect()
    }

    /// `spikiness` is the exponent of a power mean. A value of 1.0 gives the
    /// arithmetic mean height in each cell; larger values move toward the
    /// maximum return and preserve sharper canopy peaks.
    pub fn canopy_height_model(
        &self,
        dem: &Dfm<Elevation>,
        spikiness: f64,
    ) -> Dfm<HeightAboveGround> {
        let mut chm = Dfm::<HeightAboveGround>::new_like(dem);
        let mut sums = vec![0.; dem.width() * dem.height()];
        let mut counts = vec![0_u32; dem.width() * dem.height()];
        let power = if spikiness.is_finite() {
            spikiness.clamp(1., 64.)
        } else {
            1.
        };

        for point in self.points.iter() {
            let x_index =
                ((point.x() - dem.grid.top_left.x) / dem.grid.cell_size_m).round() as isize;
            let y_index =
                ((dem.grid.top_left.y - point.y()) / dem.grid.cell_size_m).round() as isize;

            if x_index < 0
                || y_index < 0
                || x_index >= dem.width() as isize
                || y_index >= dem.height() as isize
            {
                continue;
            }

            let x_index = x_index as usize;
            let y_index = y_index as usize;
            let index = y_index * dem.width() + x_index;
            let height_above_ground = (point.0.z - f64::from(dem[(y_index, x_index)])).max(0.);

            sums[index] += height_above_ground.powf(power);
            counts[index] += 1;
        }

        for (index, value) in chm.field.iter_mut().enumerate() {
            let count = counts[index];
            *value = if count == 0 {
                0.
            } else {
                (sums[index] / f64::from(count)).powf(1. / power) as f32
            };
        }

        chm
    }

    pub fn point_density<T: RasterMarker>(&self, grid: &Dfm<T>) -> Dfm<PointDensity> {
        self.point_density_impl(grid, false)
    }

    /// Point density excluding synthetic interpolation support points.
    pub fn observed_point_density<T: RasterMarker>(&self, grid: &Dfm<T>) -> Dfm<PointDensity> {
        self.point_density_impl(grid, true)
    }

    fn point_density_impl<T: RasterMarker>(
        &self,
        grid: &Dfm<T>,
        exclude_synthetic: bool,
    ) -> Dfm<PointDensity> {
        let mut density = Dfm::<PointDensity>::new_like(grid);
        let mut counts = vec![0_u32; grid.width() * grid.height()];
        let mut observed_returns = HashSet::with_capacity(self.points.len());

        for point in &self.points {
            if exclude_synthetic && point.0.is_synthetic {
                continue;
            }
            // Exact duplicate records are common where adjacent source files
            // overlap. Count them once so overlap cannot masquerade as better
            // observation support. Distinct returns at the same XY remain
            // distinct through Z and return metadata.
            if !observed_returns.insert((
                point.0.x.to_bits(),
                point.0.y.to_bits(),
                point.0.z.to_bits(),
                point.0.return_number,
                point.0.number_of_returns,
                point.0.intensity,
            )) {
                continue;
            }
            let x_index =
                ((point.x() - grid.grid.top_left.x) / grid.grid.cell_size_m).round() as isize;
            let y_index =
                ((grid.grid.top_left.y - point.y()) / grid.grid.cell_size_m).round() as isize;

            if x_index < 0
                || y_index < 0
                || x_index >= grid.width() as isize
                || y_index >= grid.height() as isize
            {
                continue;
            }

            let index = y_index as usize * grid.width() + x_index as usize;
            counts[index] += 1;
        }

        let cell_area = grid.grid.cell_size_m.powi(2);
        for (value, count) in density.field.iter_mut().zip(counts) {
            *value = (f64::from(count) / cell_area) as f32;
        }

        density
    }

    pub fn get_dfm_dimensions(&self) -> Bounds {
        let dx = self.bounds.max.x - self.bounds.min.x;
        let dy = self.bounds.max.y - self.bounds.min.y;

        // small but non-zero for some odd reason
        // stretch or shrink the bounds to fit
        // to TILE_SIZE exactly
        let stretch_x = (TILE_SIZE_METERS - dx) / 2.;
        let stretch_y = (TILE_SIZE_METERS - dy) / 2.;

        // because the top-left corner of every cell is queried
        // shift the dem over so top left corner of the first and last
        // cell in both dimensions are equally far from self.bounds
        // i.e shift by half the cell size
        // positive in x as left is min_x -> need to increase to shift
        // negative in y as top is max_y -> need to decrease to shift
        let offset_x = CELL_SIZE_METERS / 2.;
        let offset_y = -CELL_SIZE_METERS / 2.;

        Bounds {
            min: Vector {
                x: self.bounds.min.x - stretch_x + offset_x,
                y: self.bounds.min.y - stretch_y + offset_y,
                z: (i32::MIN / 1000) as f64,
            },
            max: Vector {
                x: self.bounds.max.x + stretch_x + offset_x,
                y: self.bounds.max.y + stretch_y + offset_y,
                z: (i32::MAX / 1000) as f64,
            },
        }
    }

    pub fn bounded_convex_hull(
        &mut self,
        dfm_bounds: &Bounds,
        epsilon: f64,
    ) -> crate::Result<geo::Polygon> {
        let convex_hull = self.convex_hull()?;
        let mut hull_contour = geo::LineString::new(vec![]);

        for mut point in convex_hull {
            if (dfm_bounds.min.x - point.x()).abs() <= epsilon {
                point.0.x = dfm_bounds.min.x;
            } else if (dfm_bounds.max.x - point.x()).abs() <= epsilon {
                point.0.x = dfm_bounds.max.x;
            }
            if (dfm_bounds.min.y - point.y()).abs() <= epsilon {
                point.0.y = dfm_bounds.min.y;
            } else if (dfm_bounds.max.y - point.y()).abs() <= epsilon {
                point.0.y = dfm_bounds.max.y;
            }

            hull_contour.0.push(point.flatten().into());
        }
        hull_contour.close();

        Ok(geo::Polygon::new(hull_contour.simplify(epsilon), vec![]))
    }

    fn convex_hull(&mut self) -> crate::Result<Vec<PointLaz>> {
        let mut gp_iter = self
            .points
            .iter()
            .filter(|p| p.0.classification == Classification::Ground);

        let mut bottom_point = gp_iter
            .next()
            .context("Cannot build a convex hull without ground points")?
            .clone();
        for point in gp_iter {
            if point.y() < bottom_point.y()
                || (point.y() == bottom_point.y() && point.x() < bottom_point.x())
            {
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
            .filter(|p| p.0.classification == Classification::Ground);
        let Some(second_point) = gp_iter.next() else {
            bail!("Cannot build a convex hull with fewer than two ground points");
        };
        convex_hull.push(second_point.clone());

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
        Ok(convex_hull)
    }
}

impl Index<usize> for PointCloud {
    type Output = PointLaz;

    fn index(&self, index: usize) -> &Self::Output {
        &self.points[index]
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
                x: TILE_SIZE_METERS - 0.01,
                y: TILE_SIZE_METERS + 0.01,
                z: 0.,
            },
        };

        let pc = PointCloud::new(vec![], bounds);

        let dfm_bounds = pc.get_dfm_dimensions();

        let expected = Bounds {
            min: Vector {
                x: CELL_SIZE_METERS / 2.,
                y: -CELL_SIZE_METERS / 2.,
                z: 0.,
            },
            max: Vector {
                x: TILE_SIZE_METERS + CELL_SIZE_METERS / 2.,
                y: TILE_SIZE_METERS - CELL_SIZE_METERS / 2.,
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

    #[test]
    fn point_density_is_point_count_divided_by_cell_area() {
        let bounds = Bounds {
            min: Vector {
                x: 0.,
                y: 0.,
                z: 0.,
            },
            max: Vector {
                x: TILE_SIZE_METERS,
                y: TILE_SIZE_METERS,
                z: 0.,
            },
        };
        let grid = Dfm::<Elevation>::standard(geo::Coord { x: 0., y: 10. });
        let points = vec![
            PointLaz::new(0.1, 9.9, 0.),
            PointLaz::new(-0.1, 10.1, 0.),
            PointLaz::new(CELL_SIZE_METERS, 10., 0.),
            PointLaz::new(-CELL_SIZE_METERS, 10., 0.),
        ];

        let density = PointCloud::new(points, bounds).point_density(&grid);

        assert_eq!(density[(0, 0)], (2. / CELL_SIZE_METERS.powi(2)) as f32);
        assert_eq!(density[(0, 1)], (1. / CELL_SIZE_METERS.powi(2)) as f32);
        assert_eq!(density[(1, 0)], 0.);
    }

    #[test]
    fn point_density_does_not_double_exact_overlap_returns() {
        let bounds = Bounds {
            min: Vector {
                x: 0.,
                y: 0.,
                z: 0.,
            },
            max: Vector {
                x: TILE_SIZE_METERS,
                y: TILE_SIZE_METERS,
                z: 0.,
            },
        };
        let grid = Dfm::<Elevation>::standard(geo::Coord { x: 0., y: 10. });
        let point = PointLaz::new(0.1, 9.9, 0.);
        let density = PointCloud::new(vec![point.clone(), point], bounds).point_density(&grid);
        assert_eq!(density[(0, 0)], (1. / CELL_SIZE_METERS.powi(2)) as f32);
    }
}
