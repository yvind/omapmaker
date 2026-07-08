use crate::{
    Error, Result,
    geometry::{MapRect, PointCloud, PointLaz},
    neighbors::{NeighborSide, Neighborhood},
};

use copc_rs::CopcReader;
use las::point::Classification;
use rstar::{PointDistance, RTree, primitives::GeomWithData};

use std::path::PathBuf;

// Arbitrary (but often used) sine-hash multiplier used to derive stable fractional-mm jitter from coordinates
const JITTER_HASH_MULTIPLIER: f64 = 43_758.545_312_3;

// Add a deterministic sub-millimeter XY jitter before shifting points into the local coordinate frame.
// This breaks exact duplicate/collinear grid-aligned inputs that can make the hull and Delaunay triangulation degenerate.
fn jitter_point(point: &mut las::Point, ref_point: geo::Coord) {
    let jitter = |value: f64| (value.sin() * JITTER_HASH_MULTIPLIER).rem_euclid(1.0) / 1_000.;
    point.x += jitter(point.x) - 0.0005 - ref_point.x;
    point.y += jitter(point.y) - 0.0005 - ref_point.y;
}

pub fn read_laz(
    las_paths: &[PathBuf],
    neighbor_map: &Neighborhood,
    tile_bounds: geo::Rect,
    edge_tile: NeighborSide,
    ref_point: geo::Coord,
) -> Result<(PointCloud, geo::Polygon)> {
    let mut las_reader = CopcReader::from_path(&las_paths[neighbor_map.center])?;

    let header = las_reader.header();

    let query_bounds = tile_bounds.into_bounds(header.bounds().min.z, header.bounds().max.z);
    let mut rel_bounds = query_bounds;
    rel_bounds.max.x -= ref_point.x;
    rel_bounds.min.x -= ref_point.x;
    rel_bounds.max.y -= ref_point.y;
    rel_bounds.min.y -= ref_point.y;

    let mut point_cloud = PointCloud::new(
        las_reader
            .points(
                copc_rs::LodSelection::All,
                copc_rs::BoundsSelection::Within(query_bounds),
            )?
            .filter_map(|mut p| {
                (p.classification == Classification::Ground && !p.is_withheld).then(|| {
                    jitter_point(&mut p, ref_point);
                    PointLaz(p)
                })
            })
            .collect(),
        rel_bounds,
    );

    // skip this tile if there is almost no ground points
    if point_cloud.points.len() < 4 {
        return Err(Error::NoGroundPoints.into());
    }

    // get the indices for neighboring laz file if edge tile
    let edge_paths_index = match edge_tile {
        NeighborSide::TopLeft => [neighbor_map.left, neighbor_map.top_left, neighbor_map.top]
            .into_iter()
            .flatten()
            .collect(),
        NeighborSide::Top => [neighbor_map.top].into_iter().flatten().collect(),
        NeighborSide::TopRight => [neighbor_map.right, neighbor_map.top_right, neighbor_map.top]
            .into_iter()
            .flatten()
            .collect(),
        NeighborSide::Right => [neighbor_map.right].into_iter().flatten().collect(),
        NeighborSide::BottomRight => [
            neighbor_map.right,
            neighbor_map.bottom_right,
            neighbor_map.bottom,
        ]
        .into_iter()
        .flatten()
        .collect(),
        NeighborSide::Bottom => [neighbor_map.bottom].into_iter().flatten().collect(),
        NeighborSide::BottomLeft => [
            neighbor_map.bottom,
            neighbor_map.bottom_left,
            neighbor_map.left,
        ]
        .into_iter()
        .flatten()
        .collect(),
        NeighborSide::Left => [neighbor_map.left].into_iter().flatten().collect(),
        _ => vec![],
    };

    for ei in edge_paths_index.iter() {
        let mut edge_reader = CopcReader::from_path(&las_paths[*ei])?;

        point_cloud.add(
            edge_reader
                .points(
                    copc_rs::LodSelection::All,
                    copc_rs::BoundsSelection::Within(query_bounds),
                )?
                .filter_map(|mut p| {
                    (p.classification == Classification::Ground && !p.is_withheld).then(|| {
                        jitter_point(&mut p, ref_point);
                        PointLaz(p)
                    })
                })
                .collect(),
        );
    }

    let map_bounds = point_cloud.get_dfm_dimensions();

    let convex_hull = point_cloud.bounded_convex_hull(&map_bounds, 2. * crate::CELL_SIZE)?;

    // add the water points to the ground cloud
    let water_points = las_reader
        .points(
            copc_rs::LodSelection::All,
            copc_rs::BoundsSelection::Within(query_bounds),
        )?
        .filter_map(|mut p| {
            (p.classification == Classification::Water && !p.is_withheld).then(|| {
                jitter_point(&mut p, ref_point);
                PointLaz(p)
            })
        })
        .collect();
    point_cloud.add(water_points);

    for ei in edge_paths_index.iter() {
        let mut edge_reader = CopcReader::from_path(&las_paths[*ei])?;

        point_cloud.add(
            edge_reader
                .points(
                    copc_rs::LodSelection::All,
                    copc_rs::BoundsSelection::Within(query_bounds),
                )?
                .filter_map(|mut p| {
                    (p.classification == Classification::Water && !p.is_withheld).then(|| {
                        jitter_point(&mut p, ref_point);
                        PointLaz(p)
                    })
                })
                .collect(),
        );
    }

    // add ghost points at the corners of the bounds to make the entire dem interpolate-able
    // IDW interpolating the ghost points from the 4 closest real points
    let query_points = [
        [rel_bounds.min.x, rel_bounds.max.y],
        [rel_bounds.min.x, rel_bounds.min.y],
        [rel_bounds.max.x, rel_bounds.min.y],
        [rel_bounds.max.x, rel_bounds.max.y],
    ];
    let mut zs = [0.; 4];

    let pt = RTree::bulk_load(
        point_cloud
            .to_2d_slice()
            .into_iter()
            .enumerate()
            .map(|(index, point)| GeomWithData::new(point, index))
            .collect(),
    );
    for (i, qp) in query_points.iter().enumerate() {
        let neighbors = pt.nearest_neighbor_iter(*qp).take(4).collect::<Vec<_>>();
        let tot_weight = neighbors
            .iter()
            .fold(0., |acc, n| acc + 1. / n.distance_2(qp).max(f64::EPSILON));

        zs[i] = neighbors.iter().fold(0., |acc, n| {
            acc + point_cloud[n.data].0.z / n.distance_2(qp).max(f64::EPSILON)
        }) / tot_weight;
    }

    point_cloud.add(vec![
        PointLaz::new(query_points[0][0], query_points[0][1], zs[0]),
        PointLaz::new(query_points[1][0], query_points[1][1], zs[1]),
        PointLaz::new(query_points[2][0], query_points[2][1], zs[2]),
        PointLaz::new(query_points[3][0], query_points[3][1], zs[3]),
    ]);

    Ok((point_cloud, convex_hull))
}
