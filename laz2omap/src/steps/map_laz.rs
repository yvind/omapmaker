use geo::{Coord, Rect};
use kiddo::{immutable::float::kdtree::ImmutableKdTree, SquaredEuclidean};
use las::Reader;
use proj4rs::{transform::transform, Proj};

use std::{collections::HashSet, path::PathBuf, sync::mpsc::Sender};

use crate::comms::messages::*;
use crate::geometry::MapRect;
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
    let (_neighbour_graph, bounds, _ref_point, components) = spatial_laz_analysis(&paths);

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

fn spatial_laz_analysis(
    paths: &Vec<PathBuf>,
) -> (Vec<[Option<usize>; 9]>, Vec<Rect>, Coord, Vec<Vec<usize>>) {
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
        let mut center_point = tile_centers.swap_remove(0); // round ref_point to nearest 10m
        center_point[0] = (center_point[0] / 10.).round() * 10.;
        center_point[1] = (center_point[1] / 10.).round() * 10.;

        return (
            vec![[Some(0), None, None, None, None, None, None, None, None]],
            tile_bounds,
            Coord::from(center_point),
            vec![vec![0]],
        );
    }

    let neighbours = neighbouring_tiles(&tile_centers, &tile_bounds);

    let components = connected_components(&neighbours);

    let mut ref_point: Coord<f64> = Coord::default();
    tile_centers.iter().for_each(|tc| {
        ref_point.x += tc[0];
        ref_point.y += tc[1]
    });
    ref_point.x = (ref_point.x / (10 * tile_centers.len()) as f64).round() * 10.;
    ref_point.y = (ref_point.y / (10 * tile_centers.len()) as f64).round() * 10.;

    (neighbours, tile_bounds, ref_point, components)
}

fn neighbouring_tiles(tile_centers: &[[f64; 2]], tile_bounds: &[Rect]) -> Vec<[Option<usize>; 9]> {
    let tree: ImmutableKdTree<f64, usize, 2, 32> = ImmutableKdTree::new_from_slice(tile_centers);

    let mut avg_tile_size = 0.;
    tile_bounds
        .iter()
        .for_each(|r| avg_tile_size += r.max().x - r.min().x + r.max().y - r.min().y);
    avg_tile_size /= (2 * tile_bounds.len()) as f64;

    let margin = 0.1 * avg_tile_size;

    let mut tile_neighbours = Vec::with_capacity(tile_centers.len());
    for (i, point) in tile_centers.iter().enumerate() {
        let bounds = &tile_bounds[i];

        let nn = tree.nearest_n::<SquaredEuclidean>(point, 9);
        let mut neighbours_index: Vec<usize> = nn.iter().map(|n| n.item).collect();

        neighbours_index.retain(|&e| tile_bounds[i].touch_margin(&tile_bounds[e], margin));

        let mut orderd_neighbours = [Some(i), None, None, None, None, None, None, None, None];
        for ni in neighbours_index.iter().skip(1) {
            if let Some(j) = get_neighbour_side(bounds, tile_centers[*ni]) {
                orderd_neighbours[j] = Some(*ni)
            }
        }

        tile_neighbours.push(orderd_neighbours);
    }
    tile_neighbours
}

pub fn neighbours_on_grid(nx: usize, ny: usize) -> Vec<[Option<usize>; 9]> {
    let mut neighbours = Vec::with_capacity(nx * ny);

    for yi in 0..ny {
        for xi in 0..nx {
            if xi == 0 && yi == 0 {
                //no neighbours to the left or top
                neighbours.push([
                    Some(yi * nx + xi),
                    None,
                    None,
                    None,
                    Some(yi * nx + xi + 1),
                    Some(yi * nx + xi + 1 + nx),
                    Some(yi * nx + xi + nx),
                    None,
                    None,
                ]);
            } else if xi == nx - 1 && yi == 0 {
                // no neighbours to the right or top
                neighbours.push([
                    Some(yi * nx + xi),
                    None,
                    None,
                    None,
                    None,
                    None,
                    Some(yi * nx + xi + nx),
                    Some(yi * nx + xi + nx - 1),
                    Some(yi * nx + xi - 1),
                ]);
            } else if xi == 0 && yi == ny - 1 {
                // no neighbours to the left or bottom
                neighbours.push([
                    Some(yi * nx + xi),
                    None,
                    Some(yi * nx + xi - nx),
                    Some(yi * nx + xi - nx + 1),
                    Some(yi * nx + xi + 1),
                    None,
                    None,
                    None,
                    None,
                ]);
            } else if xi == nx - 1 && yi == ny - 1 {
                // no neighbours to the right or bottom
                neighbours.push([
                    Some(yi * nx + xi),
                    Some(yi * nx + xi - 1 - nx),
                    Some(yi * nx + xi - nx),
                    None,
                    None,
                    None,
                    None,
                    None,
                    Some(yi * nx + xi - 1),
                ]);
            } else if xi == 0 {
                // no neighbours to the left
                neighbours.push([
                    Some(yi * nx + xi),
                    None,
                    Some(yi * nx + xi - nx),
                    Some(yi * nx + xi - nx + 1),
                    Some(yi * nx + xi + 1),
                    Some(yi * nx + xi + nx + 1),
                    Some(yi * nx + xi + nx),
                    None,
                    None,
                ]);
            } else if xi == nx - 1 {
                neighbours.push([
                    Some(yi * nx + xi),
                    Some(yi * nx + xi - 1 - nx),
                    Some(yi * nx + xi - nx),
                    None,
                    None,
                    None,
                    Some(yi * nx + xi + nx),
                    Some(yi * nx + xi + nx - 1),
                    Some(yi * nx + xi - 1),
                ]);
            } else if yi == 0 {
                neighbours.push([
                    Some(yi * nx + xi),
                    None,
                    None,
                    None,
                    Some(yi * nx + xi + 1),
                    Some(yi * nx + xi + nx + 1),
                    Some(yi * nx + xi + nx),
                    Some(yi * nx + xi + nx - 1),
                    Some(yi * nx + xi - 1),
                ]);
            } else if yi == ny - 1 {
                neighbours.push([
                    Some(yi * nx + xi),
                    Some(yi * nx + xi - 1 - nx),
                    Some(yi * nx + xi - nx),
                    Some(yi * nx + xi - nx + 1),
                    Some(yi * nx + xi + 1),
                    None,
                    None,
                    None,
                    Some(yi * nx + xi - 1),
                ]);
            } else {
                neighbours.push([
                    Some(yi * nx + xi),
                    Some(yi * nx + xi - 1 - nx),
                    Some(yi * nx + xi - nx),
                    Some(yi * nx + xi - nx + 1),
                    Some(yi * nx + xi + 1),
                    Some(yi * nx + xi + nx + 1),
                    Some(yi * nx + xi + nx),
                    Some(yi * nx + xi + nx - 1),
                    Some(yi * nx + xi - 1),
                ]);
            }
        }
    }
    neighbours
}

fn connected_components(graph: &Vec<[Option<usize>; 9]>) -> Vec<Vec<usize>> {
    let mut cc: Vec<HashSet<usize>> = vec![];

    for node in graph {
        let middle = node[0].unwrap();
        let mut belongs_to = usize::MAX;

        for (i, component) in cc.iter().enumerate() {
            if component.contains(&middle) {
                belongs_to = i;
                break;
            }
        }

        if belongs_to != usize::MAX {
            // the main node belongs to a component and so all
            // of its neighbours also belong to that component
            for ni in node.iter().skip(1).flatten() {
                let _ = cc[belongs_to].insert(*ni);
            }
        } else {
            // the main node does not belong to a component
            // create a new component and add it and all of its neighbours to that component
            let mut new_component = HashSet::new();

            for ni in node.iter().flatten() {
                let _ = new_component.insert(*ni);
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
                    // mark them for mergeing
                    merge.push(j);
                }
            }

            // walk through backwards to not affect the marked indecies with the swap_remove
            for j in merge.iter().rev() {
                let com = cc.swap_remove(*j);

                cc[i].extend(com);
            }
            i += 1;
        }
    }
    cc.into_iter().map(|mut h| h.drain().collect()).collect()
}

fn get_neighbour_side(bounds: &Rect, tile_center: [f64; 2]) -> Option<usize> {
    if tile_center[0] < bounds.min().x && tile_center[1] > bounds.max().y {
        return Some(1);
    }
    if tile_center[0] > bounds.max().x && tile_center[1] > bounds.max().y {
        return Some(3);
    }
    if tile_center[0] > bounds.max().x && tile_center[1] < bounds.min().y {
        return Some(5);
    }
    if tile_center[0] < bounds.min().x && tile_center[1] < bounds.min().y {
        return Some(7);
    }
    if tile_center[1] > bounds.max().y {
        return Some(2);
    }
    if tile_center[0] > bounds.max().x {
        return Some(4);
    }
    if tile_center[1] < bounds.min().y {
        return Some(6);
    }
    if tile_center[0] < bounds.min().x {
        return Some(8);
    }
    None
}
