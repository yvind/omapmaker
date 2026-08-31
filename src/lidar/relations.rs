#![allow(clippy::too_many_arguments)]
#![allow(clippy::type_complexity)]

use anyhow::Context;
use las::Reader;
use proj_core::{CrsDef, Transform};

use std::{collections::HashSet, path::PathBuf};

use crate::Result;
use crate::geometry::MapRect;
use crate::geometry::neighbors::Neighborhood;

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
                Transform::from_horizontal_components(crs, &crate::project::get_global_crs())
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

fn spatial_laz_analysis(paths: &Vec<PathBuf>) -> (Vec<geo::Rect>, Vec<Vec<usize>>) {
    let mut tile_centers = Vec::with_capacity(paths.len());
    let mut tile_bounds = Vec::with_capacity(paths.len());

    for las_path in paths {
        if let Ok(las_reader) = Reader::from_path(las_path) {
            let b = las_reader.header().bounds();
            tile_centers.push([(b.min.x + b.max.x) / 2., (b.min.y + b.max.y) / 2.]);
            tile_bounds.push(geo::Rect::from_bounds(b));
        }
    }

    if tile_centers.len() == 1 {
        return (tile_bounds, vec![vec![0]]);
    }

    let neighbors = Neighborhood::neighboring_tiles(&tile_centers, &tile_bounds);
    let components = connected_components(&neighbors);

    (tile_bounds, components)
}

fn connected_components(graph: &Vec<Neighborhood>) -> Vec<Vec<usize>> {
    let mut cc: Vec<HashSet<usize>> = vec![];

    for node in graph {
        let mut belongs_to = usize::MAX;

        for (i, component) in cc.iter().enumerate() {
            if component.contains(&node.center) {
                belongs_to = i;
                break;
            }
        }

        if belongs_to != usize::MAX {
            // the main node belongs to a component and so all
            // of its neighbors also belong to that component
            for ni in node.neighbor_indices() {
                let _ = cc[belongs_to].insert(ni);
            }
        } else {
            // the main node does not belong to a component
            // create a new component and add it and all of its neighbors to that component
            let mut new_component = HashSet::new();

            for ni in node.all_indices() {
                let _ = new_component.insert(ni);
            }
            cc.push(new_component);
        }

        // check for overlaps, i.e. that some node exists in
        // multiple components if so merge those components
        let mut i = 0;
        while i < cc.len() {
            // the components that should be merged to component i
            let mut merge = vec![];
            for j in i + 1..cc.len() {
                if !cc[i].is_disjoint(&cc[j]) {
                    // component i and j are connected
                    // mark them for merging
                    merge.push(j);
                }
            }

            // walk through backwards to not affect the marked indices with the swap_remove
            for j in merge.iter().rev() {
                let com = cc.swap_remove(*j);

                cc[i].extend(com);
            }
            i += 1;
        }
    }
    cc.into_iter().map(|mut h| h.drain().collect()).collect()
}
