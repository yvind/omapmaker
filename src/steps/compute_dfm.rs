use crate::dfm::{Dfm, FieldType};
use crate::geometry::{Line, Point2D, PointCloud};

use kiddo::{immutable::float::kdtree::ImmutableKdTree, SquaredEuclidean};
use std::sync::{mpsc, Arc};
use std::thread;

pub fn compute_dfms(
    pt: Arc<ImmutableKdTree<f64, usize, 2, 32>>,
    pc: Arc<PointCloud>,
    ch: Arc<Line>,
    num_threads: usize,
    width: usize,
    height: usize,
    cell_size: f64,
    tl: Point2D,
) -> (Dfm, Dfm, Dfm, Dfm, Dfm, Dfm) {
    if num_threads > 1 {
        compute_dfms_multithread(pt, pc, ch, num_threads, width, height, cell_size, tl)
    } else {
        compute_dfms_singlethread(pt, pc, ch, width, height, cell_size, tl)
    }
}

fn compute_dfms_multithread(
    pt: Arc<ImmutableKdTree<f64, usize, 2, 32>>,
    pc: Arc<PointCloud>,
    ch: Arc<Line>,
    num_threads: usize,
    width: usize,
    height: usize,
    cell_size: f64,
    tl: Point2D,
) -> (Dfm, Dfm, Dfm, Dfm, Dfm, Dfm) {
    let mut dem = Dfm::new(width, height, tl, cell_size);
    let mut grad_dem = dem.clone();
    let mut drm = dem.clone();
    let mut grad_drm = dem.clone();
    let mut dim = dem.clone();
    let mut grad_dim = dem.clone();
    let dummy = Arc::new(dem.clone());

    let (sender, receiver) = mpsc::channel();

    let mut thread_handles = vec![];
    let num_neighbours = 32;

    for i in 0..(num_threads - 1) {
        let pt_ref = pt.clone();
        let pc_ref = pc.clone();
        let ch_ref = ch.clone();
        let dummy_ref = dummy.clone();

        let thread_sender = sender.clone();

        thread_handles.push(thread::spawn(move || {
            let mut y_index = i;

            while y_index < height {
                for x_index in 0..width {
                    let coords: Point2D = dummy_ref.index2coord(x_index, y_index).unwrap();
                    if !ch_ref.contains(&coords).unwrap() {
                        continue;
                    }

                    // slow due to very many lookups
                    let nearest_n =
                        pt_ref.nearest_n::<SquaredEuclidean>(&[coords.x, coords.y], num_neighbours);
                    let neighbours: Vec<usize> = nearest_n.iter().map(|n| n.item).collect();

                    // slow due to matrix inversion
                    // gradients are almost for free
                    let (elev, grad_elev) =
                        pc_ref.interpolate_field(FieldType::Elevation, &neighbours, &coords, 0.5);
                    let (intens, grad_intens) =
                        pc_ref.interpolate_field(FieldType::Intensity, &neighbours, &coords, 1.);
                    let (rn, grad_rn) =
                        pc_ref.interpolate_field(FieldType::ReturnNumber, &neighbours, &coords, 1.);

                    thread_sender.send((elev, y_index, x_index, 0)).unwrap();
                    thread_sender.send((intens, y_index, x_index, 1)).unwrap();
                    thread_sender.send((rn, y_index, x_index, 2)).unwrap();
                    thread_sender
                        .send((grad_elev, y_index, x_index, 3))
                        .unwrap();
                    thread_sender
                        .send((grad_intens, y_index, x_index, 4))
                        .unwrap();
                    thread_sender.send((grad_rn, y_index, x_index, 5)).unwrap();
                }

                y_index += num_threads - 1;
            }
            drop(thread_sender);
        }));
    }
    drop(sender);

    for (value, yi, xi, ii) in receiver.iter() {
        match ii {
            0 => dem.field[yi][xi] = value,
            1 => dim.field[yi][xi] = value,
            2 => drm.field[yi][xi] = value,
            3 => grad_dem.field[yi][xi] = value,
            4 => grad_dim.field[yi][xi] = value,
            _ => grad_drm.field[yi][xi] = value,
        }
    }

    for t in thread_handles {
        t.join().unwrap();
    }

    (dem, grad_dem, drm, grad_drm, dim, grad_dim)
}

fn compute_dfms_singlethread(
    pt: Arc<ImmutableKdTree<f64, usize, 2, 32>>,
    pc: Arc<PointCloud>,
    ch: Arc<Line>,
    width: usize,
    height: usize,
    cell_size: f64,
    tl: Point2D,
) -> (Dfm, Dfm, Dfm, Dfm, Dfm, Dfm) {
    let mut dem = Dfm::new(width, height, tl, cell_size);
    let mut grad_dem = dem.clone();
    let mut drm = dem.clone();
    let mut grad_drm = dem.clone();
    let mut dim = dem.clone();
    let mut grad_dim = dem.clone();

    let num_neighbours = 32;

    for y_index in 0..height {
        for x_index in 0..width {
            let coords: Point2D = dem.index2coord(x_index, y_index).unwrap();
            if !ch.contains(&coords).unwrap() {
                continue;
            }

            // slow due to very many lookups
            let nearest_n = pt.nearest_n::<SquaredEuclidean>(&[coords.x, coords.y], num_neighbours);
            let neighbours: Vec<usize> = nearest_n.iter().map(|n| n.item).collect();

            // slow due to matrix inversion
            // gradients are almost for free
            let (elev, grad_elev) =
                pc.interpolate_field(FieldType::Elevation, &neighbours, &coords, 0.5);
            let (intens, grad_intens) =
                pc.interpolate_field(FieldType::Intensity, &neighbours, &coords, 1.);
            let (rn, grad_rn) =
                pc.interpolate_field(FieldType::ReturnNumber, &neighbours, &coords, 1.);

            dem.field[y_index][x_index] = elev;
            grad_dem.field[y_index][x_index] = grad_elev;
            drm.field[y_index][x_index] = rn;
            grad_drm.field[y_index][x_index] = grad_rn;
            dim.field[y_index][x_index] = intens;
            grad_dim.field[y_index][x_index] = grad_intens;
        }
    }

    (dem, grad_dem, drm, grad_drm, dim, grad_dim)
}
