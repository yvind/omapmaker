#![allow(clippy::too_many_arguments)]
use crate::geometry::{ContourLevel, ContourSet};
use crate::params::MapParams;
use crate::raster::Dfm;

use omap::{LineObject, LineSymbol, MapObject, Omap, TagTrait};

use geo::Polygon;
use spade::Point2;

use crate::SIDE_LENGTH;

use core::f64;
use geo::{BooleanOps, Simplify};
use std::sync::{Arc, Mutex};

pub fn compute_contours(
    true_dem: &Dfm,
    z_range: (f64, f64),
    cut_overlay: &Polygon,
    thresholds: (f64, f64),
    params: &MapParams,
    map: &Arc<Mutex<Omap>>,
) {
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

        // triangulate the contour set
        let tri = contours.triangulate(&adjusted_dem);
        let nn = tri.natural_neighbor();

        // interpolate triangulation
        for y_index in 0..SIDE_LENGTH {
            for x_index in 0..SIDE_LENGTH {
                let coords = interpolated_dfm.index2coord(x_index, y_index);
                let coords = Point2::new(coords.x, coords.y);

                if let Some(elev) = nn.interpolate(|p| p.data().z, coords) {
                    if elev.is_nan() {
                        println!("Nan in c1 interpolating!");
                    }
                    interpolated_dfm[(y_index, x_index)] = elev;
                }
            }
        }

        // calculate the contour set error
        error = contours.calculate_error(true_dem, &interpolated_dfm, params.contour_algo_lambda);

        if error <= min_threshold || (error - prev_error).abs() <= conv_threshold {
            break;
        }

        // adjust dem
        adjusted_dem.adjust(true_dem, &interpolated_dfm, 1.);
        prev_error = 3.;
        iterations += 1;

        contours.0.clear();
    }

    for c_index in 0..c_levels {
        let c_level = c_index as f64 * effective_interval + start_level;

        let mut c_contours = adjusted_dem.marching_squares(c_level);

        c_contours = c_contours.simplify(&crate::SIMPLIFICATION_DIST);

        c_contours = cut_overlay.clip(&c_contours, false);

        let symbol = if c_level % (5. * params.contour_interval) == 0. {
            LineSymbol::IndexContour
        } else if c_level % params.contour_interval == 0. {
            LineSymbol::Contour
        } else {
            LineSymbol::Formline
        };
        for c in c_contours {
            let mut c_object = LineObject::from_line_string(c, symbol);
            c_object.add_auto_tag();
            c_object.add_tag("Elevation", format!("{:.2}", c_level).as_str());

            map.lock()
                .unwrap()
                .add_object(MapObject::LineObject(c_object));
        }
    }
}
