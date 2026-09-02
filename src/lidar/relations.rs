#![allow(clippy::too_many_arguments)]
#![allow(clippy::type_complexity)]

use anyhow::Context;
use las::Reader;
use proj_core::{CrsDef, Transform};

use std::path::PathBuf;

use crate::Result;
use crate::geometry::MapRect;

pub(crate) struct SpatialRelations {
    pub(crate) boundaries: Vec<[geo::Coord; 4]>,
    pub(crate) boundary_areas: Vec<f64>,
    pub(crate) home: geo::Coord,
    pub(crate) components: Vec<Vec<usize>>,
}

pub(crate) fn map_spatial_relations(
    paths: Vec<PathBuf>,
    crs_defs: Option<Vec<Option<CrsDef>>>,
) -> Result<SpatialRelations> {
    let (boundaries, boundary_areas, home, components) = read_boundaries(paths, crs_defs)?;
    Ok(SpatialRelations {
        boundaries,
        boundary_areas,
        home,
        components,
    })
}

fn read_boundaries(
    paths: Vec<PathBuf>,
    crs_defs: Option<Vec<Option<CrsDef>>>,
) -> Result<(Vec<[geo::Coord; 4]>, Vec<f64>, geo::Coord, Vec<Vec<usize>>)> {
    let (bounds, components) = spatial_laz_analysis(&paths);

    let mut all_lidar_bounds = [(f64::MAX, f64::MIN), (f64::MIN, f64::MAX)];

    let mut geographic_boundaries = Vec::with_capacity(bounds.len());
    let mut boundary_areas = Vec::with_capacity(bounds.len());

    for (i, bound) in bounds.iter().enumerate() {
        boundary_areas.push((bound.max().x - bound.min().x) * (bound.max().y - bound.min().y));

        let mut points = [
            (bound.min().x, bound.max().y),
            (bound.min().x, bound.min().y),
            (bound.max().x, bound.min().y),
            (bound.max().x, bound.max().y),
        ];

        if let Some(crs_defs) = &crs_defs
            && let Some(crs) = &crs_defs[i]
        {
            // transform bounds to lat lon
            let transform =
                Transform::from_horizontal_components(crs, &crate::projection::get_global_crs())
                    .with_context(|| format!("Failed to create transform from {:?}", crs))?;

            points[0] = transform.convert(points[0])?;
            points[1] = transform.convert(points[1])?;
            points[2] = transform.convert(points[2])?;
            points[3] = transform.convert(points[3])?;
        }

        geographic_boundaries.push([
            geo::coord! { x: points[0].0, y: points[0].1 },
            geo::coord! { x: points[1].0, y: points[1].1 },
            geo::coord! { x: points[2].0, y: points[2].1 },
            geo::coord! { x: points[3].0, y: points[3].1 },
        ]);

        if all_lidar_bounds[0].0 > points[0].0 {
            all_lidar_bounds[0].0 = points[0].0;
        }
        if all_lidar_bounds[0].1 < points[0].1 {
            all_lidar_bounds[0].1 = points[0].1;
        }
        if all_lidar_bounds[1].0 < points[2].0 {
            all_lidar_bounds[1].0 = points[2].0;
        }
        if all_lidar_bounds[1].1 > points[2].1 {
            all_lidar_bounds[1].1 = points[2].1;
        }
    }
    let mid_point = geo::coord! {
        x: (all_lidar_bounds[0].0 + all_lidar_bounds[1].0) / 2.,
        y: (all_lidar_bounds[0].1 + all_lidar_bounds[1].1) / 2.,
    };
    Ok((geographic_boundaries, boundary_areas, mid_point, components))
}

fn spatial_laz_analysis(paths: &[PathBuf]) -> (Vec<geo::Rect>, Vec<Vec<usize>>) {
    let mut tile_bounds = Vec::with_capacity(paths.len());

    for las_path in paths {
        if let Ok(las_reader) = Reader::from_path(las_path) {
            let b = las_reader.header().bounds();
            tile_bounds.push(geo::Rect::from_bounds(b));
        }
    }

    let components = connected_components(&tile_bounds);

    (tile_bounds, components)
}

fn connected_components(bounds: &[geo::Rect]) -> Vec<Vec<usize>> {
    if bounds.is_empty() {
        return Vec::new();
    }

    let average_size = bounds
        .iter()
        .map(|bounds| bounds.width() + bounds.height())
        .sum::<f64>()
        / (2 * bounds.len()) as f64;
    let connection_margin = 0.1 * average_size;
    let mut visited = vec![false; bounds.len()];
    let mut components = Vec::new();

    for start in 0..bounds.len() {
        if visited[start] {
            continue;
        }

        visited[start] = true;
        let mut pending = vec![start];
        let mut component = Vec::new();
        while let Some(current) = pending.pop() {
            component.push(current);
            for candidate in 0..bounds.len() {
                if !visited[candidate]
                    && bounds[current].touch_margin(&bounds[candidate], connection_margin)
                {
                    visited[candidate] = true;
                    pending.push(candidate);
                }
            }
        }
        component.sort_unstable();
        components.push(component);
    }

    components
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bounds_components_support_non_rectangular_and_overlapping_layouts() {
        let bounds = [
            geo::Rect::new((0., 0.), (10., 10.)),
            geo::Rect::new((10., 0.), (20., 10.)),
            geo::Rect::new((0., 10.), (10., 20.)),
            geo::Rect::new((8., 8.), (12., 12.)),
            geo::Rect::new((100., 100.), (110., 110.)),
        ];

        assert_eq!(
            connected_components(&bounds),
            vec![vec![0, 1, 2, 3], vec![4]]
        );
    }
}
