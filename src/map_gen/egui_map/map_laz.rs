#![allow(clippy::too_many_arguments)]
#![allow(clippy::type_complexity)]

use geo::Rect;
use las::Reader;
use proj4rs::{transform::transform, Proj};

use std::{collections::HashSet, path::PathBuf, sync::mpsc::Sender};

use crate::comms::messages::*;
use crate::geometry::MapRect;
use crate::neighbors::Neighborhood;
use crate::Result;

pub fn map_laz(sender: Sender<FrontendTask>, paths: Vec<PathBuf>, crs_epsg: Option<Vec<u16>>) {
    let (boundaries, mid_point, components) = match read_boundaries(paths, crs_epsg) {
        Ok(bm) => bm,
        Err(e) => {
            sender
                .send(FrontendTask::Error(e.to_string(), true))
                .unwrap();
            return;
        }
    };

    sender
        .send(FrontendTask::UpdateVariable(Variable::Boundaries(
            boundaries.clone(),
        )))
        .unwrap();

    sender
        .send(FrontendTask::UpdateVariable(Variable::Home(mid_point)))
        .unwrap();

    sender
        .send(FrontendTask::UpdateVariable(Variable::ConnectedComponents(
            components,
        )))
        .unwrap();
    sender
        .send(FrontendTask::TaskComplete(
            TaskDone::MapSpatialLidarRelations,
        ))
        .unwrap();
}

fn read_boundaries(
    paths: Vec<PathBuf>,
    crs_epsg: Option<Vec<u16>>,
) -> Result<(
    Vec<[walkers::Position; 4]>,
    walkers::Position,
    Vec<Vec<usize>>,
)> {
    let (bounds, components) = spatial_laz_analysis(&paths);

    let mut all_lidar_bounds = [(f64::MAX, f64::MIN), (f64::MIN, f64::MAX)];

    let mut walkers_boundaries = Vec::with_capacity(bounds.len());

    for (i, bound) in bounds.iter().enumerate() {
        let mut points = [
            (bound.min().x, bound.max().y),
            (bound.min().x, bound.min().y),
            (bound.max().x, bound.min().y),
            (bound.max().x, bound.max().y),
        ];

        if crs_epsg.is_some() {
            // transform bounds to lat lon
            let to = Proj::from_user_string("WGS84").unwrap();
            let from = Proj::from_epsg_code(crs_epsg.as_ref().unwrap()[i])?;

            transform(&from, &to, points.as_mut_slice())?;

            for (x, y) in points.iter_mut() {
                *x = x.to_degrees();
                *y = y.to_degrees();
            }
        }

        walkers_boundaries.push([
            walkers::pos_from_lon_lat(points[0].0, points[0].1),
            walkers::pos_from_lon_lat(points[1].0, points[1].1),
            walkers::pos_from_lon_lat(points[2].0, points[2].1),
            walkers::pos_from_lon_lat(points[3].0, points[3].1),
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
    let mid_point = walkers::pos_from_lon_lat(
        (all_lidar_bounds[0].0 + all_lidar_bounds[1].0) / 2.,
        (all_lidar_bounds[0].1 + all_lidar_bounds[1].1) / 2.,
    );
    Ok((walkers_boundaries, mid_point, components))
}

fn spatial_laz_analysis(paths: &Vec<PathBuf>) -> (Vec<Rect>, Vec<Vec<usize>>) {
    let mut tile_centers = Vec::with_capacity(paths.len());
    let mut tile_bounds = Vec::with_capacity(paths.len());

    for las_path in paths {
        if let Ok(las_reader) = Reader::from_path(las_path) {
            let b = las_reader.header().bounds();
            tile_centers.push([(b.min.x + b.max.x) / 2., (b.min.y + b.max.y) / 2.]);
            tile_bounds.push(Rect::from_bounds(b));
        }
    }

    if tile_centers.len() == 1 {
        return (tile_bounds, vec![vec![0]]);
    }

    let neighbours = Neighborhood::neighboring_tiles(&tile_centers, &tile_bounds);
    let components = connected_components(&neighbours);

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
