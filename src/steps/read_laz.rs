use crate::geometry::{Line, Point2D, PointCloud, PointLaz};

use kiddo::immutable::float::kdtree::ImmutableKdTree;
use las::{point::Classification, Read, Reader};
use rand::random;
use std::{path::PathBuf, sync::Arc};

pub fn read_laz(
    neighbour_map: &[usize],
    las_paths: &[PathBuf],
    ref_point: &Point2D,
    cell_size: f64,
    margin: f64,
    dist_to_hull_epsilon: f64,
) -> (
    Arc<PointCloud>,
    Arc<ImmutableKdTree<f64, usize, 2, 32>>,
    Arc<Line>,
    usize,
    usize,
    Point2D,
) {
    // read first and main laz file
    let mut las_reader = Reader::from_path(&las_paths[neighbour_map[0]]).unwrap_or_else(|_| {
        panic!(
            "Could not read given laz/las file with path: {}",
            las_paths[neighbour_map[0]].to_string_lossy()
        )
    });

    // read laz file and build pointcloud and KD-tree

    let header = las_reader.header();
    let mut las_bounds = header.bounds();
    println!("Number of points: {:?}", header.number_of_points());

    las_bounds.max.x -= ref_point.x;
    las_bounds.min.x -= ref_point.x;
    las_bounds.max.y -= ref_point.y;
    las_bounds.min.y -= ref_point.y;

    println!("Filtering points...");
    let mut xyzir = PointCloud::new(
        las_reader
            .points()
            .map(|r| r.unwrap())
            .filter_map(|p| {
                ((p.classification == Classification::Ground
                    || p.classification == Classification::Water)
                    && !p.is_withheld)
                    .then(|| PointLaz {
                        x: p.x + 2. * (random::<f64>() - 0.5) / 1000. - ref_point.x,
                        y: p.y + 2. * (random::<f64>() - 0.5) / 1000. - ref_point.y,
                        z: p.z,
                        i: p.intensity as u32,
                        r: p.return_number,
                        c: if p.classification == Classification::Ground {
                            2
                        } else {
                            9
                        },
                        n: p.number_of_returns,
                    })
            }) // add noise on the order of mm for KD-tree stability
            .collect(),
        las_bounds,
    );

    let (width, height, map_bounds) = xyzir.get_dfm_dimensions(cell_size);
    let tl = Point2D {
        x: map_bounds.min.x,
        y: map_bounds.max.y,
    };
    let convex_hull = xyzir.bounded_convex_hull(cell_size, &map_bounds, dist_to_hull_epsilon * 2.);

    for &fi in neighbour_map.iter().skip(1) {
        let mut las_reader = Reader::from_path(&las_paths[fi]).unwrap_or_else(|_| {
            panic!(
                "Could not read given laz/las file with path: {}",
                las_paths[fi].to_string_lossy()
            )
        });

        xyzir.add(
            las_reader
                .points()
                .map(|r| r.unwrap())
                .filter_map(|p| {
                    (((p.classification == Classification::Ground
                        || p.classification == Classification::Water)
                        && !p.is_withheld)
                        && convex_hull
                            .almost_contains(&Point2D::new(p.x, p.y), margin)
                            .unwrap())
                    .then(|| PointLaz {
                        x: p.x + 2. * (random::<f64>() - 0.5) / 1000. - ref_point.x,
                        y: p.y + 2. * (random::<f64>() - 0.5) / 1000. - ref_point.y,
                        z: p.z,
                        i: p.intensity as u32,
                        r: p.return_number,
                        c: if p.classification == Classification::Ground {
                            2
                        } else {
                            9
                        },
                        n: p.number_of_returns,
                    })
                }) // add noise on the order of mm for KD-tree stability
                .collect(),
        )
    }

    println!("Building Kd-tree...");
    let point_tree: ImmutableKdTree<f64, usize, 2, 32> =
        ImmutableKdTree::new_from_slice(&xyzir.to_2d_slice());

    (
        Arc::new(xyzir),
        Arc::new(point_tree),
        Arc::new(convex_hull),
        width,
        height,
        tl,
    )
}
