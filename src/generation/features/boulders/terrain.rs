#[cfg(test)]
use crate::geometry::PointLaz;
use crate::raster::{Dfm, Elevation};

pub(super) struct Plane {
    elevation: f64,
    slope_x: f64,
    slope_y: f64,
    pub(super) roughness: f64,
}

impl Plane {
    #[cfg(test)]
    pub(super) fn elevation_at(&self, coordinate: geo::Coord, center: geo::Coord) -> f64 {
        self.elevation
            + self.slope_x * (coordinate.x - center.x)
            + self.slope_y * (coordinate.y - center.y)
    }
    #[cfg(test)]
    pub(super) fn height(&self, point: &PointLaz, center: geo::Coord) -> f64 {
        point.0.z - self.elevation_at(geo::coord! { x: point.x(), y: point.y() }, center)
    }
}

/// Fit surrounding canonical terrain, excluding the whole footprint and a
/// one-cell moat. Trim outlying context samples, never the object's own DEM.
pub(super) fn context_plane(
    dem: &Dfm<Elevation>,
    bounds: (usize, usize, usize, usize),
    center: geo::Coord,
) -> Option<Plane> {
    let (left, right, top, bottom) = bounds;
    let margin = (3. / dem.grid.cell_size_m).ceil() as usize;
    if left < margin
        || top < margin
        || right + margin >= dem.width()
        || bottom + margin >= dem.height()
    {
        return None;
    }
    let mut samples = Vec::new();
    for y in top - margin..=bottom + margin {
        for x in left - margin..=right + margin {
            if x >= left.saturating_sub(1)
                && x <= right + 1
                && y >= top.saturating_sub(1)
                && y <= bottom + 1
            {
                continue;
            }
            let z = f64::from(dem[(y, x)]);
            if z.is_finite() && dem[(y, x)] != f32::MIN {
                let p = dem.index2coord(y, x);
                samples.push([p.x - center.x, p.y - center.y, z]);
            }
        }
    }
    if samples.len() < 12 {
        return None;
    }
    let mut retained = samples.clone();
    let mut plane = fit(&retained)?;
    for _ in 0..2 {
        retained.sort_unstable_by(|a, b| error(a, &plane).abs().total_cmp(&error(b, &plane).abs()));
        retained.truncate((retained.len() * 4 / 5).max(12));
        plane = fit(&retained)?;
    }
    let mut errors = samples
        .iter()
        .map(|s| error(s, &plane).abs())
        .collect::<Vec<_>>();
    errors.sort_unstable_by(f64::total_cmp);
    plane.roughness = errors[errors.len() / 2];
    Some(plane)
}

fn error(sample: &[f64; 3], plane: &Plane) -> f64 {
    sample[2] - plane.elevation - plane.slope_x * sample[0] - plane.slope_y * sample[1]
}

fn fit(samples: &[[f64; 3]]) -> Option<Plane> {
    let n = samples.len() as f64;
    let mean = samples.iter().fold([0.; 3], |mut sum, p| {
        for i in 0..3 {
            sum[i] += p[i] / n;
        }
        sum
    });
    let (mut xx, mut yy, mut xy, mut xz, mut yz) = (0., 0., 0., 0., 0.);
    for p in samples {
        let (x, y, z) = (p[0] - mean[0], p[1] - mean[1], p[2] - mean[2]);
        xx += x * x;
        yy += y * y;
        xy += x * y;
        xz += x * z;
        yz += y * z;
    }
    let determinant = xx * yy - xy * xy;
    if determinant.abs() < 1e-12 {
        return None;
    }
    let slope_x = (xz * yy - yz * xy) / determinant;
    let slope_y = (yz * xx - xz * xy) / determinant;
    Some(Plane {
        elevation: mean[2] - slope_x * mean[0] - slope_y * mean[1],
        slope_x,
        slope_y,
        roughness: 0.,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::raster::DfmGrid;

    #[test]
    fn surrounding_ground_height_is_invariant_to_a_planar_slope() {
        let grid = DfmGrid::new(41, 41, 0.5, geo::coord! { x: 0., y: 20. }).unwrap();
        let mut dem = Dfm::<Elevation>::new(grid);
        for slope in [0., 0.125, -0.2] {
            for cell in 0..dem.field.len() {
                let p = dem.index2coord(cell / dem.width(), cell % dem.width());
                dem.field[cell] = (slope * p.x) as f32;
            }
            let center = geo::coord! { x: 10., y: 10. };
            let ground = context_plane(&dem, (18, 22, 18, 22), center).unwrap();
            let point = PointLaz::new(10.08, 10., 3. + slope * 10.08);
            assert!((ground.height(&point, center) - 3.).abs() < 1e-6);
            assert!(ground.roughness < 1e-6);
        }
    }

    #[test]
    fn missing_or_incomplete_surrounding_terrain_is_rejected() {
        let grid = DfmGrid::new(41, 41, 0.5, geo::coord! { x: 0., y: 20. }).unwrap();
        let mut dem = Dfm::<Elevation>::new(grid);
        dem.field.fill(0.);
        let center = geo::coord! { x: 10., y: 10. };
        assert!(context_plane(&dem, (18, 22, 18, 22), center).is_some());
        assert!(context_plane(&dem, (0, 4, 0, 4), center).is_none());
        dem.field.fill(f32::MIN);
        assert!(context_plane(&dem, (18, 22, 18, 22), center).is_none());
    }
}
