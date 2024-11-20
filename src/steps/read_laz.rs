use crate::geometry::{Coord, LineString, PointCloud};

use fastrand::f64 as random;
use las::{point::Classification, Reader};
use std::path::PathBuf;

pub fn read_laz(las_path: &PathBuf, ref_point: Coord) -> (PointCloud, LineString, Coord) {
    let mut las_reader = Reader::from_path(las_path).unwrap_or_else(|_| {
        panic!(
            "Could not read given laz/las file with path: {}",
            las_path.to_string_lossy()
        )
    });

    let header = las_reader.header();
    let mut las_bounds = header.bounds();

    las_bounds.max.x -= ref_point.x;
    las_bounds.min.x -= ref_point.x;
    las_bounds.max.y -= ref_point.y;
    las_bounds.min.y -= ref_point.y;

    // read only ground points into a cloud so that
    // the convex hull only contains the ground points
    let mut ground_cloud = PointCloud::new(
        las_reader
            .points()
            .filter_map(Result::ok)
            .filter_map(|p| {
                (p.classification == Classification::Ground && !p.is_withheld).then(|| {
                    let mut clone = p.clone();
                    clone.x += 2. * (random() - 0.5) / 1_000. - ref_point.x;
                    clone.y += 2. * (random() - 0.5) / 1_000. - ref_point.y;
                    clone
                })
            }) // add noise on the order of mm for KD-tree stability
            .collect(),
        las_bounds,
    );

    let map_bounds = ground_cloud.get_dfm_dimensions();
    let tl = Coord {
        x: map_bounds.min.x,
        y: map_bounds.max.y,
    };
    let convex_hull = ground_cloud.bounded_convex_hull(&map_bounds, 2. * crate::CELL_SIZE);

    // add the water points to the ground cloud
    let mut las_reader = Reader::from_path(las_path).unwrap();
    ground_cloud.add(
        las_reader
            .points()
            .filter_map(Result::ok)
            .filter_map(|p| {
                (p.classification == Classification::Water && !p.is_withheld).then(|| {
                    let mut clone = p.clone();
                    clone.x += 2. * (random() - 0.5) / 1_000. - ref_point.x;
                    clone.y += 2. * (random() - 0.5) / 1_000. - ref_point.y;
                    clone
                })
            }) // add noise on the order of mm for KD-tree stability
            .collect::<Vec<_>>(),
    );

    (ground_cloud, convex_hull, tl)
}
