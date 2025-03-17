#![allow(clippy::too_many_arguments)]
use crate::geometry::{ContourLevel, ContourSet};
use crate::parameters::MapParameters;
use crate::raster::Dfm;

use omap::{LineObject, LineSymbol, MapObject, Omap, TagTrait};

use geo::Polygon;

use core::f64;
use geo::{BooleanOps, Simplify};
use std::sync::{Arc, Mutex};

// used for the naive iterative interpolation error correction contour algorithm
pub fn compute_naive_contours(
    true_dem: &Dfm,
    z_range: (f64, f64),
    cut_overlay: &Polygon,
    thresholds: (f64, f64),
    params: &MapParameters,
    map: &Arc<Mutex<Omap>>,
) {
    {
        // to make sure the old drawable map features are cleared even if no features are added
        let mut map = map.lock().unwrap();
        map.reserve_capacity(omap::Symbol::Contour, 1);
        map.reserve_capacity(omap::Symbol::Formline, 1);
        map.reserve_capacity(omap::Symbol::IndexContour, 1);
    }

    let (min_threshold, conv_threshold) = thresholds;

    let effective_interval = if params.formlines {
        params.contour_interval / 2.
    } else {
        params.contour_interval
    };

    let c_levels = ((z_range.1 - z_range.0) / effective_interval).ceil() as usize + 1;
    let start_level = (z_range.0 / effective_interval).floor() * effective_interval;

    let mut adjusted_dem = true_dem.clone();
    let mut interpolated_dfm = Dfm::new(true_dem.tl_coord);

    let mut contours = ContourSet::with_capacity(c_levels);

    let mut error;
    let mut prev_error = f64::MAX;
    let mut iterations = 0;
    loop {
        // extract contour set from adjusted_dem
        for c_index in 0..c_levels {
            let c_level = c_index as f64 * effective_interval + start_level;

            let c_contours = adjusted_dem.marching_squares(c_level);

            contours.0.push(ContourLevel::new(
                c_contours.simplify(&crate::SIMPLIFICATION_DIST),
                c_level,
            ));
        }

        if iterations >= params.contour_algo_steps {
            break;
        }

        // interpolate the contour set
        contours
            .interpolate(&mut interpolated_dfm, &adjusted_dem, 25)
            .unwrap();

        // calculate the error
        // should this only include contours inside the cut_bounds?
        //
        // a length exp of 0 gives bending energy, 1 gives bending force, 2 gives stiffness? (same units as a spring constant)
        // my guess is the exp should be 1 or 2 (or something in between)
        error = true_dem.error(&interpolated_dfm) + params.contour_algo_lambda * contours.energy(1);
        println!("iteration: {iterations}, error: {error}");

        if error <= min_threshold || (error - prev_error).abs() <= conv_threshold {
            break;
        }

        // adjust dem, increasing frequency decreasing amplitude
        adjusted_dem.adjust(
            true_dem,
            &interpolated_dfm,
            (params.contour_algo_steps - iterations) as usize * 30,
            (params.contour_algo_steps - iterations + 1) as f64
                / (params.contour_algo_steps + 1) as f64,
        );
        prev_error = error;
        iterations += 1;

        contours.0.clear();
    }

    for c_level in contours.0 {
        let z = c_level.z;

        let c_contours = cut_overlay.clip(&c_level.lines, false);

        let symbol = if z % (5. * params.contour_interval) == 0. {
            LineSymbol::IndexContour
        } else if z % params.contour_interval == 0. {
            LineSymbol::Contour
        } else {
            LineSymbol::Formline
        };
        for c in c_contours {
            let mut c_object = LineObject::from_line_string(c, symbol);
            c_object.add_elevation_tag(z);

            map.lock()
                .unwrap()
                .add_object(MapObject::LineObject(c_object));
        }
    }
}

// used for raw and smoothed contour extraction.
// smoothing happens on the DEM level
pub fn extract_contours(
    dem: &Dfm,
    z_range: (f64, f64),
    cut_overlay: &Polygon,
    params: &MapParameters,
    map: &Arc<Mutex<Omap>>,
) {
    {
        // to make sure the old drawable map features are cleared even if no features are added
        let mut map = map.lock().unwrap();
        map.reserve_capacity(omap::Symbol::Contour, 1);
        map.reserve_capacity(omap::Symbol::Formline, 1);
        map.reserve_capacity(omap::Symbol::IndexContour, 1);
    }

    let effective_interval = if params.formlines {
        params.contour_interval / 2.
    } else {
        params.contour_interval
    };

    let c_levels = ((z_range.1 - z_range.0) / effective_interval).ceil() as usize + 1;
    let start_level = (z_range.0 / effective_interval).floor() * effective_interval;

    for c_index in 0..c_levels {
        let c_level = c_index as f64 * effective_interval + start_level;

        let mut contours = dem.marching_squares(c_level);

        contours = contours.simplify(&crate::SIMPLIFICATION_DIST);

        contours = cut_overlay.clip(&contours, false);

        let symbol = if c_level % (5. * params.contour_interval) == 0. {
            LineSymbol::IndexContour
        } else if c_level % params.contour_interval == 0. {
            LineSymbol::Contour
        } else {
            LineSymbol::Formline
        };
        for c in contours {
            let mut c_object = LineObject::from_line_string(c, symbol);
            c_object.add_elevation_tag(c_level);

            map.lock()
                .unwrap()
                .add_object(MapObject::LineObject(c_object));
        }
    }
}
