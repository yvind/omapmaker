use crate::TILE_SIZE_PIXELS;
use crate::geometry::{ContourLevel, ContourSet};
use crate::map_gen::egui_map::{LineSymbol, MapObject};
use crate::parameters::{ContourAlgo, MapParameters};
use crate::raster::Dfm;
use crate::raster::dfm::Elevation;

use geo::{BooleanOps, Simplify};

use std::collections::HashMap;

// used for the naive iterative interpolation error correction contour algorithm
pub fn compute_naive_contours(
    true_dem: &Dfm<Elevation>,
    z_range: (f64, f64),
    cut_overlay: &geo::Polygon,
    thresholds: (f64, f64),
    params: &MapParameters,
) -> crate::Result<(Vec<MapObject>, f64, f64)> {
    let (min_threshold, conv_threshold) = thresholds;

    let effective_interval = if params.contour.form_lines {
        params.contour.interval / 2.
    } else {
        params.contour.interval
    };

    let c_levels = ((z_range.1 - z_range.0) / effective_interval).ceil() as usize + 1;
    let start_level = (z_range.0 / effective_interval).floor() * effective_interval;

    let mut adjusted_dem = true_dem.smoothen(15., 15, 10);
    let mut interpolated_dem = adjusted_dem.clone();

    let clip_poly = geo::Polygon::new(
        geo::LineString::new(vec![
            true_dem.index2coord(0, 0),
            true_dem.index2coord(TILE_SIZE_PIXELS - 1, 0),
            true_dem.index2coord(TILE_SIZE_PIXELS - 1, TILE_SIZE_PIXELS - 1),
            true_dem.index2coord(0, TILE_SIZE_PIXELS - 1),
            true_dem.index2coord(0, 0),
        ]),
        vec![],
    );

    let mut contours = ContourSet::with_capacity(c_levels);

    let mut error = 0.;
    let mut energy = 0.;

    let mut score;
    let mut prev_score = f64::MAX;
    let mut iterations = 0;

    loop {
        // extract contour set from adjusted_dem
        for c_index in 0..c_levels {
            let c_level = c_index as f64 * effective_interval + start_level;

            let mut c_contours = adjusted_dem
                .marching_squares(c_level)
                .simplify(crate::SIMPLIFICATION_DIST);

            // should clip the contours
            c_contours = clip_poly.clip(&c_contours, false);

            contours.0.push(ContourLevel::new(c_contours, c_level));
        }

        if iterations >= params.contour.algo_steps {
            break;
        }

        // interpolate the contour set
        contours.interpolate(&mut interpolated_dem, &adjusted_dem)?;

        // calculate the error
        // should this only include contours inside the cut_bounds?
        //
        // a length exp of 0 gives bending energy, 1 gives bending force, 2 gives stiffness? (same units as a spring constant)
        // my guess is the exp should be 1 or 2 (or something in between)
        error = true_dem.error(&interpolated_dem);
        energy = contours.energy(1);

        score = error + params.contour.algo_lambda * energy;

        if score <= min_threshold || (score - prev_score).abs() <= conv_threshold {
            break;
        }

        // adjust dem, increasing frequency decreasing amplitude
        let filter_half_size = ((params.contour.algo_steps - iterations) as f64
            / params.contour.algo_steps as f64
            * 30.) as usize;
        let filter_amplitude =
            (params.contour.algo_steps - iterations) as f64 / (params.contour.algo_steps as f64);

        adjusted_dem.adjust(
            true_dem,
            &interpolated_dem,
            filter_half_size,
            filter_amplitude,
        );
        prev_score = score;
        iterations += 1;

        contours.0.clear();
    }

    let mut objects = Vec::with_capacity(contours.0.len());

    for c_level in contours.0 {
        let z = c_level.z;

        let c_contours = cut_overlay.clip(&c_level.lines, false);

        let symbol = if z % (5. * params.contour.interval) == 0. {
            LineSymbol::IndexContour
        } else if z % params.contour.interval == 0. {
            LineSymbol::Contour
        } else {
            LineSymbol::FormLine
        };
        for c in c_contours {
            let mut c_object = MapObject::Line {
                object: c,
                symbol,
                tags: HashMap::new(),
            };
            c_object.add_elevation_tag(z);

            objects.push(c_object);
        }
    }

    Ok((objects, error, energy))
}

// used for raw and smoothed contour extraction, with scoring which complicates it a bit
// smoothing happens on the DEM level
pub fn extract_contours(
    true_dem: &Dfm<Elevation>,
    z_range: (f64, f64),
    cut_overlay: &geo::Polygon,
    params: &MapParameters,
    compute_energy: bool,
) -> crate::Result<(Vec<MapObject>, f64, f64)> {
    let effective_interval = if params.contour.form_lines {
        params.contour.interval / 2.
    } else {
        params.contour.interval
    };

    let dem = if params.contour.algorithm == ContourAlgo::Raw {
        true_dem
    } else {
        &true_dem.smoothen(15., 15, params.contour.algo_steps as usize)
    };

    let c_levels = ((z_range.1 - z_range.0) / effective_interval).ceil() as usize + 1;
    let start_level = (z_range.0 / effective_interval).floor() * effective_interval;

    let clip_poly = geo::Polygon::new(
        geo::LineString::new(vec![
            true_dem.index2coord(0, 0),
            true_dem.index2coord(TILE_SIZE_PIXELS - 1, 0),
            true_dem.index2coord(TILE_SIZE_PIXELS - 1, TILE_SIZE_PIXELS - 1),
            true_dem.index2coord(0, TILE_SIZE_PIXELS - 1),
            true_dem.index2coord(0, 0),
        ]),
        vec![],
    );

    let mut contour_set = ContourSet::with_capacity(c_levels);

    for c_index in 0..c_levels {
        let c_level = c_index as f64 * effective_interval + start_level;

        let mut contours = dem.marching_squares(c_level);

        contours = contours.simplify(crate::SIMPLIFICATION_DIST);

        // should clip the contours
        contours = clip_poly.clip(&contours, false);

        contour_set.0.push(ContourLevel::new(contours, c_level));
    }

    let (error, energy) = if compute_energy {
        let mut interpolated_dem = dem.clone();
        contour_set.interpolate(&mut interpolated_dem, dem)?;

        (true_dem.error(&interpolated_dem), contour_set.energy(1))
    } else {
        (0., 0.)
    };

    let mut objects = Vec::with_capacity(contour_set.0.len());
    for c_level in contour_set.0 {
        let contours = cut_overlay.clip(&c_level.lines, false);

        let symbol = if c_level.z % (5. * params.contour.interval) == 0. {
            LineSymbol::IndexContour
        } else if c_level.z % params.contour.interval == 0. {
            LineSymbol::Contour
        } else {
            LineSymbol::FormLine
        };
        for c in contours {
            let mut c_object = MapObject::Line {
                object: c,
                symbol,
                tags: HashMap::new(),
            };
            c_object.add_elevation_tag(c_level.z);

            objects.push(c_object);
        }
    }
    Ok((objects, error, energy))
}
