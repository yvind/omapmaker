use kiddo::{immutable::float::kdtree::ImmutableKdTree, SquaredEuclidean};
use las::Reader;

use std::{fs, path::PathBuf};

use crate::geometry::{Point2D, Rectangle};

pub fn map_laz(input: PathBuf) -> (Vec<Vec<usize>>, Vec<PathBuf>, Point2D) {
    if input.is_file() {
        single_file(input)
    } else {
        // check if dir contains more than 1 las/laz file
        let mut paths = lidar_in_directory(input);
        if paths.len() == 0 {
            panic!("No laz/las files were found in the given directory");
        } else if paths.len() == 1 {
            single_file(paths.swap_remove(0))
        } else {
            multiple_files(paths)
        }
    }
}

fn lidar_in_directory(input: PathBuf) -> Vec<PathBuf> {
    let extensions = ["las", "laz"];

    let paths = fs::read_dir(input)
        .expect("Could not read input directory")
        .filter_map(Result::ok)
        .filter(|entry| {
            entry.path().is_file()
                && (entry.path().extension().and_then(|e| e.to_str()) == Some(extensions[0])
                    || entry.path().extension().and_then(|e| e.to_str()) == Some(extensions[1]))
        })
        .map(|lf| lf.path())
        .collect();
    paths
}

fn single_file(input: PathBuf) -> (Vec<Vec<usize>>, Vec<PathBuf>, Point2D) {
    let las_reader = Reader::from_path(input.clone()).unwrap_or_else(|_| {
        panic!(
            "Could not read given laz/las file with path: {}",
            input.to_string_lossy()
        )
    });
    let bounds = las_reader.header().bounds();

    let ref_point = Point2D::new(
        (bounds.max.x + bounds.min.x) / 2.,
        (bounds.max.y + bounds.min.y) / 2.,
    );

    (vec![vec![0]], vec![input.clone()], ref_point)
}

fn multiple_files(paths: Vec<PathBuf>) -> (Vec<Vec<usize>>, Vec<PathBuf>, Point2D) {
    let mut tile_centers = Vec::with_capacity(paths.len());
    let mut tile_bounds = Vec::with_capacity(paths.len());
    let mut tile_names = Vec::with_capacity(paths.len());

    for las_path in paths {
        if let Ok(las_reader) = Reader::from_path(&las_path) {
            let b = las_reader.header().bounds();
            tile_centers.push([(b.min.x + b.max.x) / 2., (b.min.y + b.max.y) / 2.]);
            tile_bounds.push(Rectangle::from(b));
            tile_names.push(las_path);
        }
    }

    if tile_names.len() == 0 {
        panic!("Unable to read the las/laz-files found in the input directory");
    } else if tile_names.len() == 1 {
        return (
            vec![vec![0]],
            tile_names,
            Point2D::from(tile_centers.swap_remove(0)),
        );
    }

    let neighbours = neighbouring_tiles(&tile_centers, &tile_bounds);

    let mut ref_point = Point2D::default();
    tile_centers.iter().for_each(|tc| {
        ref_point.x += tc[0];
        ref_point.y += tc[1]
    });
    ref_point.x = (ref_point.x / (10 * tile_centers.len()) as f64).round() * 10.;
    ref_point.y = (ref_point.y / (10 * tile_centers.len()) as f64).round() * 10.;

    (neighbours, tile_names, ref_point)
}

fn neighbouring_tiles(tile_centers: &[[f64; 2]], tile_bounds: &[Rectangle]) -> Vec<Vec<usize>> {
    let tree: ImmutableKdTree<f64, usize, 2, 32> = ImmutableKdTree::new_from_slice(tile_centers);

    let mut avg_tile_size = 0.;
    tile_bounds
        .iter()
        .for_each(|r| avg_tile_size += r.max.x - r.min.x + r.max.y - r.min.y);
    avg_tile_size /= (2 * tile_bounds.len()) as f64;

    let margin = 0.5 * avg_tile_size;

    let mut tile_neighbours = Vec::with_capacity(tile_centers.len());
    for (i, point) in tile_centers.iter().enumerate() {
        let nn = tree.nearest_n::<SquaredEuclidean>(point, 9);
        let mut neighbours: Vec<usize> = nn.iter().map(|n| n.item).collect();

        neighbours.retain(|&e| tile_bounds[i].touch_margin(&tile_bounds[e], margin));

        tile_neighbours.push(neighbours);
    }
    tile_neighbours
}
