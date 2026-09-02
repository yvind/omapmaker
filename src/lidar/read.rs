use crate::{
    Error, Result,
    geometry::{MapRect, PointCloud, PointLaz},
    lidar::LidarSourceIndex,
};

use anyhow::Context;
use las::CopcReader;
use las::point::Classification;
use rstar::{PointDistance, RTree, primitives::GeomWithData};

use std::collections::HashMap;

// Arbitrary (but often used) sine-hash multiplier used to derive stable fractional-mm jitter from coordinates
const JITTER_HASH_MULTIPLIER: f64 = 43_758.545_312_3;

// Add a deterministic sub-millimeter XY jitter before shifting points into the local coordinate frame.
// This breaks exact duplicate/collinear grid-aligned inputs that can make the hull and Delaunay triangulation degenerate.
fn jitter_point(point: &mut las::Point, ref_point: geo::Coord) {
    let jitter = |value: f64| (value.sin() * JITTER_HASH_MULTIPLIER).rem_euclid(1.0) / 1_000.;
    point.x += jitter(point.x) - 0.0005 - ref_point.x;
    point.y += jitter(point.y) - 0.0005 - ref_point.y;
}

#[derive(Debug, Hash, PartialEq, Eq)]
struct ExactPointKey {
    x: u64,
    y: u64,
    z: u64,
    intensity: u16,
    return_number: u8,
    number_of_returns: u8,
    classification: u8,
    point_source_id: u16,
    gps_time: Option<u64>,
}

impl From<&las::Point> for ExactPointKey {
    fn from(point: &las::Point) -> Self {
        Self {
            x: point.x.to_bits(),
            y: point.y.to_bits(),
            z: point.z.to_bits(),
            intensity: point.intensity,
            return_number: point.return_number,
            number_of_returns: point.number_of_returns,
            classification: point.classification.into(),
            point_source_id: point.point_source_id,
            gps_time: point.gps_time.map(f64::to_bits),
        }
    }
}

fn candidate_is_better(candidate: &las::Point, current: &las::Point) -> bool {
    (!candidate.is_overlap, -candidate.scan_angle.abs())
        > (!current.is_overlap, -current.scan_angle.abs())
}

fn insert_exact_point(
    unique_indices: &mut HashMap<ExactPointKey, usize>,
    selected_points: &mut Vec<las::Point>,
    point: las::Point,
) {
    let key = ExactPointKey::from(&point);
    if let Some(index) = unique_indices.get(&key).copied() {
        if candidate_is_better(&point, &selected_points[index]) {
            selected_points[index] = point;
        }
    } else {
        unique_indices.insert(key, selected_points.len());
        selected_points.push(point);
    }
}

pub fn read_laz(
    source_index: &LidarSourceIndex,
    tile_bounds: geo::Rect,
    ref_point: geo::Coord,
) -> Result<(PointCloud, PointCloud, geo::Polygon)> {
    let query_bounds = tile_bounds.into_bounds();
    let mut rel_bounds = query_bounds;
    rel_bounds.max.x -= ref_point.x;
    rel_bounds.min.x -= ref_point.x;
    rel_bounds.max.y -= ref_point.y;
    rel_bounds.min.y -= ref_point.y;

    // `tile_bounds` is the full processing halo, not the smaller output-owned
    // cut bounds. At a source-file edge this intentionally selects and queries
    // every adjacent or overlapping COPC needed to interpolate across the seam.
    let sources = source_index.sources_intersecting(tile_bounds);
    let mut unique_indices = HashMap::new();
    let mut selected_points = Vec::<las::Point>::new();

    for source in sources {
        let mut reader = CopcReader::from_path(source.path()).with_context(|| {
            format!(
                "Failed to open intersecting COPC source {}",
                source.path().display()
            )
        })?;
        for point in reader
            .query(
                las::LodSelection::All,
                las::BoundsSelection::Within(query_bounds),
            )?
            .points()
            .filter_map(std::result::Result::ok)
            .filter(|point| !point.is_withheld)
        {
            insert_exact_point(&mut unique_indices, &mut selected_points, point);
        }
    }

    let all_points = selected_points
        .into_iter()
        .map(|mut point| {
            jitter_point(&mut point, ref_point);
            PointLaz(point)
        })
        .collect::<Vec<_>>();
    let ground_points = all_points
        .iter()
        .filter(|point| point.0.classification == Classification::Ground)
        .cloned()
        .collect::<Vec<_>>();
    let mut point_cloud = PointCloud::new(ground_points, rel_bounds);

    // skip this tile if there is almost no ground points
    if point_cloud.points.len() < 4 {
        return Err(Error::NoGroundPoints.into());
    }

    let map_bounds = point_cloud.get_dfm_dimensions();

    let convex_hull = point_cloud.bounded_convex_hull(&map_bounds, 2. * crate::CELL_SIZE_METERS)?;

    // add the water points to the ground cloud
    point_cloud.add(
        all_points
            .iter()
            .filter(|p| p.0.classification == Classification::Water)
            .cloned()
            .collect(),
    );

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

    let all_point_cloud = PointCloud::new(all_points, rel_bounds);

    Ok((point_cloud, all_point_cloud, convex_hull))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point() -> las::Point {
        PointLaz::new(1., 2., 3.).0
    }

    #[test]
    fn exact_duplicates_keep_one_non_overlap_representative() {
        let mut overlap = point();
        overlap.is_overlap = true;
        overlap.scan_angle = 20.;
        let mut preferred = overlap.clone();
        preferred.is_overlap = false;
        preferred.scan_angle = 2.;
        let mut indices = HashMap::new();
        let mut selected = Vec::new();

        insert_exact_point(&mut indices, &mut selected, overlap);
        insert_exact_point(&mut indices, &mut selected, preferred);

        assert_eq!(selected.len(), 1);
        assert!(!selected[0].is_overlap);
        assert_eq!(selected[0].scan_angle, 2.);
    }

    #[test]
    fn distinct_acquisition_times_are_not_exact_duplicates() {
        let mut first = point();
        first.gps_time = Some(1.);
        let mut second = first.clone();
        second.gps_time = Some(2.);
        let mut indices = HashMap::new();
        let mut selected = Vec::new();

        insert_exact_point(&mut indices, &mut selected, first);
        insert_exact_point(&mut indices, &mut selected, second);

        assert_eq!(selected.len(), 2);
    }
}
