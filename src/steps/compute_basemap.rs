use std::sync::{mpsc, Arc};
use std::thread;

use crate::geometry::Line;
use crate::map::{LineObject, MapObject, Omap, Symbol};
use crate::raster::Dfm;

pub fn compute_basemap(
    num_threads: usize,
    min_z: f64,
    max_z: f64,
    basemap_interval: f64,
    dem_arc: &Arc<Dfm>,
    hull: &Arc<Line>,
    hull_epsilon: f64,
    simplify_epsilon: f64,
    map: &mut Omap,
) {
    if num_threads > 1 {
        compute_basemap_contours_multithread(
            num_threads,
            min_z,
            max_z,
            basemap_interval,
            dem_arc,
            hull,
            hull_epsilon,
            simplify_epsilon,
            map,
        )
    } else {
        compute_basemap_contours_singlethread(
            min_z,
            max_z,
            basemap_interval,
            dem_arc,
            hull,
            hull_epsilon,
            simplify_epsilon,
            map,
        )
    }
}

fn compute_basemap_contours_multithread(
    num_threads: usize,
    min_z: f64,
    max_z: f64,
    basemap_interval: f64,
    dem_arc: &Arc<Dfm>,
    hull: &Arc<Line>,
    hull_epsilon: f64,
    simplify_epsilon: f64,
    map: &mut Omap,
) {
    let bm_levels = ((max_z - min_z) / basemap_interval).ceil() as usize;

    let (sender, receiver) = mpsc::channel();
    let mut thread_handles = vec![];

    for i in 0..(num_threads - 1) {
        let dem_ref = dem_arc.clone();
        let hull_ref = hull.clone();

        let thread_sender = sender.clone();

        thread_handles.push(thread::spawn(move || {
            let mut c_index = i;

            while c_index < bm_levels {
                let bm_level = c_index as f64 * basemap_interval + min_z.floor();

                let mut bm_contours = dem_ref.marching_squares(bm_level).unwrap();

                if simplify_epsilon > 0. {
                    for c in bm_contours.iter_mut() {
                        c.simplify(simplify_epsilon)
                    }
                }

                for c in bm_contours.iter_mut() {
                    c.fix_ends_to_line(&hull_ref, hull_epsilon)
                }

                thread_sender.send((bm_contours, bm_level)).unwrap();

                c_index += num_threads - 1;
            }
            drop(thread_sender);
        }));
    }
    drop(sender);

    for (contours, level) in receiver.iter() {
        for c in contours {
            let mut c_object = LineObject::from_line(c, Symbol::BasemapContour);
            c_object.add_auto_tag();
            c_object.add_tag("Elevation", format!("{:.2}", level).as_str());
            map.add_object(c_object);
        }
    }

    for handle in thread_handles {
        handle.join().unwrap();
    }
}

fn compute_basemap_contours_singlethread(
    min_z: f64,
    max_z: f64,
    basemap_interval: f64,
    dem: &Arc<Dfm>,
    hull: &Arc<Line>,
    hull_epsilon: f64,
    simplify_epsilon: f64,
    map: &mut Omap,
) {
    let bm_levels = ((max_z - min_z) / basemap_interval).ceil() as usize;

    for c_index in 0..bm_levels {
        let bm_level = c_index as f64 * basemap_interval + min_z.floor();

        let mut bm_contours = dem.marching_squares(bm_level).unwrap();

        if simplify_epsilon > 0. {
            for c in bm_contours.iter_mut() {
                c.simplify(simplify_epsilon)
            }
        }

        for c in bm_contours.iter_mut() {
            c.fix_ends_to_line(hull, hull_epsilon)
        }

        for c in bm_contours {
            let mut c_object = LineObject::from_line(c, Symbol::BasemapContour);
            c_object.add_auto_tag();
            c_object.add_tag("Elevation", format!("{:2}", bm_level).as_str());
            map.add_object(c_object);
        }
    }
}
