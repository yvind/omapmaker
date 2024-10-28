use crate::geometry::{LineString, Point2D, PointCloud};

use fastrand::f64 as random;
use kiddo::immutable::float::kdtree::ImmutableKdTree;
use las::{point::Classification, Reader};
use std::path::PathBuf;

pub fn read_laz(
    las_path: &PathBuf,
    dist_to_hull_epsilon: f64,
    ref_point: Point2D,
) -> (
    PointCloud,
    ImmutableKdTree<f64, usize, 2, 32>,
    LineString,
    Point2D,
) {
    // read first and main laz file
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

    let mut ground_cloud = PointCloud::new(
        las_reader
            .points()
            .filter_map(Result::ok)
            .filter_map(|p| {
                ((p.classification == Classification::Ground
                    || p.classification == Classification::Water)
                    && !p.is_withheld)
                    .then(|| {
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
    let tl = Point2D {
        x: map_bounds.min.x,
        y: map_bounds.max.y,
    };
    let convex_hull = ground_cloud.bounded_convex_hull(&map_bounds, dist_to_hull_epsilon);

    let ground_tree: ImmutableKdTree<f64, usize, 2, 32> =
        ImmutableKdTree::new_from_slice(&ground_cloud.to_2d_slice());

    (ground_cloud, ground_tree, convex_hull, tl)
}
