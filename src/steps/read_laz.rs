use crate::geometry::{Point2D, PointCloud, PointLaz};

use kiddo::immutable::float::kdtree::ImmutableKdTree;
use las::{point::Classification, Read, Reader};
use rand::random;
use std::path::Path;

pub fn read_laz(
    las_index: usize,
    las_paths: Vec<&Path>,
    neighbour_map: &Vec<Vec<usize>>,
    ref_point: &Point2D,
) -> (PointCloud, ImmutableKdTree<f64, usize, 2, 32>, Point2D) {
    let mut las_reader = Reader::from_path(las_path).expect("Could not read given laz/las file");

    // read laz file and build pointcloud and KD-tree

    let header = las_reader.header();
    let mut las_bounds = header.bounds();
    println!("Number of points: {:?}", header.number_of_points());
    println!("Point cloud {:?}", las_bounds);

    let ref_point = Point2D {
        x: ((las_bounds.min.x + las_bounds.max.x) / 2.).round(),
        y: ((las_bounds.min.y + las_bounds.max.y) / 2.).round(),
    };

    las_bounds.max.x -= ref_point.x;
    las_bounds.min.x -= ref_point.x;
    las_bounds.max.y -= ref_point.y;
    las_bounds.min.y -= ref_point.y;

    println!("Filtering points...");
    let xyzir = PointCloud::new(
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
        las_bounds.clone(),
    );

    let sqm: f64 = (las_bounds.max.x - las_bounds.min.x) * (las_bounds.max.y - las_bounds.min.y);
    println!("Number of ground points: {}", xyzir.len());
    println!("Area: {:.3} sqkm", sqm / 1_000_000.);
    println!(
        "Ground point density: {:.2} points/sqm",
        xyzir.len() as f64 / sqm
    );

    println!("Building Kd-tree...");
    let point_tree: ImmutableKdTree<f64, usize, 2, 32> =
        ImmutableKdTree::new_from_slice(&xyzir.to_2d_slice());

    (xyzir, point_tree, ref_point)
}
