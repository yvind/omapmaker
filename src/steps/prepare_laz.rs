use crate::geometry::Point2D;

use kiddo::{immutable::float::kdtree::ImmutableKdTree, SquaredEuclidean};
use las::{Bounds, Read, Reader};
use std::{
    ffi::OsString,
    fs,
    path::{Path, PathBuf},
};

pub fn prepare_laz(
    input: PathBuf,
    margin: f64,
) -> (Vec<Vec<usize>>, Vec<PathBuf>, Point2D, OsString) {
    let filestem = Path::new(input.file_name().unwrap())
        .file_stem()
        .unwrap()
        .to_owned();

    let md = fs::metadata(&input).unwrap();
    if md.is_dir() {
        let (a, b, c) = multiple_files(&input, margin);
        (a, b, c, filestem)
    } else if md.is_file() {
        let (a, b, c) = single_file(input);
        (a, b, c, filestem)
    } else {
        panic!("Given input is not a recognizable file or directory")
    }
}

fn single_file(input: PathBuf) -> (Vec<Vec<usize>>, Vec<PathBuf>, Point2D) {
    let las_reader = Reader::from_path(&input).unwrap_or_else(|_| {
        panic!(
            "Could not read given laz/las file with path: {}",
            input.to_string_lossy()
        )
    });
    let bounds = las_reader.header().bounds();

    (
        vec![vec![0]],
        vec![input],
        Point2D::new(
            ((bounds.min.x + bounds.max.x) / 20.).round() * 10.,
            ((bounds.min.y + bounds.max.y) / 20.).round() * 10.,
        ),
    )
}

fn multiple_files(input: &PathBuf, margin: f64) -> (Vec<Vec<usize>>, Vec<PathBuf>, Point2D) {
    let mut tile_centers = Vec::new();
    let mut tile_bounds = Vec::new();
    let mut tile_names = Vec::new();

    for entry in fs::read_dir(input).expect("Could not read given laz-directory") {
        let las_path = entry
            .expect("Could not access files in the laz-directory")
            .path();

        if let Ok(las_reader) = Reader::from_path(&las_path) {
            let b = las_reader.header().bounds();
            tile_centers.push([(b.min.x + b.max.x) / 2., (b.min.y + b.max.y) / 2.]);
            tile_bounds.push(b);
            tile_names.push(las_path);
        }
    }

    let neighbours = neighbouring_tiles(&tile_centers, &tile_bounds, margin);

    let mut ref_point = Point2D::new(0., 0.);
    for c in tile_centers.iter() {
        ref_point.x += c[0];
        ref_point.y += c[1];
    }
    ref_point.x = (ref_point.x / (10. * tile_centers.len() as f64)).round() * 10.;
    ref_point.y = (ref_point.y / (10. * tile_centers.len() as f64)).round() * 10.;

    (neighbours, tile_names, ref_point)
}

fn neighbouring_tiles(
    tile_centers: &[[f64; 2]],
    tile_bounds: &[Bounds],
    margin: f64,
) -> Vec<Vec<usize>> {
    let tree: ImmutableKdTree<f64, usize, 2, 32> = ImmutableKdTree::new_from_slice(tile_centers);

    let mut tile_neighbours = vec![];
    for (i, point) in tile_centers.iter().enumerate() {
        let nn = tree.nearest_n::<SquaredEuclidean>(point, 9);
        let mut neighbours: Vec<usize> = nn.iter().map(|n| n.item).collect();

        neighbours.retain(|&e| boxes_touch(&tile_bounds[i], &tile_bounds[e], margin));

        tile_neighbours.push(neighbours);
    }
    tile_neighbours
}

fn boxes_touch(box1: &Bounds, box2: &Bounds, margin: f64) -> bool {
    // if box1 is entirely to the left or right of box2
    if box1.max.x < box2.min.x - margin || box1.min.x > box2.max.x + margin {
        return false;
    }

    // if box1 is entirely under or above box2
    if box1.max.y < box2.min.y - margin || box1.min.y > box2.max.y + margin {
        return false;
    }

    true
}
