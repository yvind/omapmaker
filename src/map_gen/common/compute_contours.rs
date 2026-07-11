use crate::TILE_SIZE_PIXELS;
use crate::geometry::{ContourLevel, ContourSet, MapMultiPolygon};
use crate::map_gen::egui_map::{LineSymbol, MapObject};
use crate::parameters::{ContourAlgo, FormlinePruneAlgo, MapParameters};
use crate::raster::Dfm;
use crate::raster::dfm::Elevation;

use geo::{BooleanOps, Buffer, Euclidean, Intersects, Length, LineLocatePoint, Simplify};

use std::collections::HashMap;

const FORMLINE_PRUNE_BUFFER_METERS: f64 = 2.;
const FORMLINE_RECONNECT_GAP_METERS: f64 = 3.;

fn contour_symbol(elevation: f64, interval: f64) -> LineSymbol {
    if is_interval_level(elevation, 5. * interval) {
        LineSymbol::IndexContour
    } else if is_interval_level(elevation, interval) {
        LineSymbol::Contour
    } else {
        LineSymbol::FormLine
    }
}

fn is_interval_level(elevation: f64, interval: f64) -> bool {
    interval > 0. && (elevation / interval - (elevation / interval).round()).abs() <= 1e-8
}

struct FormlinePruner {
    important_terrain: geo::MultiPolygon,
    buffered_terrain: geo::MultiPolygon,
    scale: crate::parameters::Scale,
}

#[derive(Clone, Copy, Debug)]
struct FormlineRange {
    start: f64,
    end: f64,
    important: bool,
}

impl FormlinePruner {
    fn from_terrain_change(
        dem: &Dfm<Elevation>,
        clip_polygon: &geo::Polygon,
        params: &MapParameters,
    ) -> Self {
        let terrain_change = dem.terrain_change(params.contour.interval);
        Self::from_importance(
            &terrain_change,
            params.contour.form_line_prune_threshold,
            clip_polygon,
            params,
        )
    }

    fn from_contour_interpolation_error(
        contour_set: &ContourSet,
        contour_dem: &Dfm<Elevation>,
        true_dem: &Dfm<Elevation>,
        with_formlines: Option<&Dfm<Elevation>>,
        clip_polygon: &geo::Polygon,
        params: &MapParameters,
    ) -> crate::Result<Self> {
        let computed_with_formlines = if with_formlines.is_none() {
            let mut interpolated = contour_dem.clone();
            contour_set.interpolate(&mut interpolated, contour_dem)?;
            Some(interpolated)
        } else {
            None
        };
        let with_formlines = with_formlines
            .or(computed_with_formlines.as_ref())
            .expect("computed reconstruction when none was supplied");

        let contours_without_formlines = ContourSet(
            contour_set
                .0
                .iter()
                .filter(|level| {
                    contour_symbol(level.z, params.contour.interval) != LineSymbol::FormLine
                })
                .cloned()
                .collect(),
        );
        let mut without_formlines = contour_dem.clone();
        contours_without_formlines.interpolate(&mut without_formlines, contour_dem)?;

        let improvement =
            true_dem.interpolation_error_improvement(with_formlines, &without_formlines);
        Ok(Self::from_importance(
            &improvement,
            params.contour.form_line_error_threshold,
            clip_polygon,
            params,
        ))
    }

    fn from_importance<T: Clone>(
        importance: &Dfm<T>,
        threshold: f64,
        clip_polygon: &geo::Polygon,
        params: &MapParameters,
    ) -> Self {
        let change_contours = importance.marching_squares(threshold.max(0.));
        let important_terrain =
            geo::MultiPolygon::from_contours(change_contours, clip_polygon, false)
                .simplify(crate::SIMPLIFICATION_DIST);
        let buffered_terrain = important_terrain
            .buffer(FORMLINE_PRUNE_BUFFER_METERS)
            .simplify(crate::SIMPLIFICATION_DIST);

        Self {
            important_terrain,
            buffered_terrain,
            scale: params.scale,
        }
    }

    fn prune(&self, source: &geo::LineString) -> Vec<geo::LineString> {
        let source_length = Euclidean.length(source);
        if source.0.len() < 2 || source_length <= f64::EPSILON {
            return Vec::new();
        }

        // Put the seam of a closed contour in a pruned part. This keeps linear
        // referencing unambiguous when a retained fragment crosses index zero.
        if source.is_closed()
            && let Some(seam) = source.0[..source.0.len() - 1]
                .iter()
                .position(|coord| !self.buffered_terrain.intersects(&geo::Point(*coord)))
            && seam > 0
        {
            return self.prune(&rotate_closed_line(source, seam));
        }

        // LineSymbol minimum lengths are expressed in map micrometres. Convert
        // them back to projected ground metres before comparing geometry.
        let min_length = LineSymbol::FormLine.min_length(self.scale, source.is_closed())
            * self.scale.denominator()
            / 1_000_000.;

        let clipped = self
            .buffered_terrain
            .clip(&geo::MultiLineString::new(vec![source.clone()]), false);
        if let Some(closed_fragment) = clipped.0.iter().find(|fragment| fragment.is_closed()) {
            let important = Euclidean.length(&self.important_terrain.clip(
                &geo::MultiLineString::new(vec![closed_fragment.clone()]),
                false,
            )) > crate::SIMPLIFICATION_DIST;
            return if important || source_length >= min_length {
                vec![source.clone()]
            } else {
                Vec::new()
            };
        }

        let mut ranges = clipped
            .0
            .iter()
            .filter_map(|fragment| self.fragment_range(source, fragment))
            .collect::<Vec<_>>();

        merge_formline_ranges(&mut ranges, FORMLINE_RECONNECT_GAP_METERS / source_length);

        ranges.retain_mut(|range| {
            let length = (range.end - range.start) * source_length;
            if length >= min_length {
                return true;
            }
            if !range.important {
                return false;
            }

            elongate_range(range, (min_length / source_length).min(1.));
            true
        });
        merge_formline_ranges(&mut ranges, f64::EPSILON);

        ranges
            .into_iter()
            .filter_map(|range| line_substring(source, range.start, range.end))
            .collect()
    }

    fn fragment_range(
        &self,
        source: &geo::LineString,
        fragment: &geo::LineString,
    ) -> Option<FormlineRange> {
        let start = source.line_locate_point(&geo::Point(*fragment.0.first()?))?;
        let end = source.line_locate_point(&geo::Point(*fragment.0.last()?))?;
        let important = Euclidean.length(
            &self
                .important_terrain
                .clip(&geo::MultiLineString::new(vec![fragment.clone()]), false),
        ) > crate::SIMPLIFICATION_DIST;

        Some(FormlineRange {
            start: start.min(end),
            end: start.max(end),
            important,
        })
    }
}

fn prune_formline(pruner: Option<&FormlinePruner>, line: &geo::LineString) -> Vec<geo::LineString> {
    pruner
        .map(|pruner| pruner.prune(line))
        .unwrap_or_else(|| vec![line.clone()])
}

fn rotate_closed_line(source: &geo::LineString, seam: usize) -> geo::LineString {
    let unique = &source.0[..source.0.len() - 1];
    let mut rotated = Vec::with_capacity(source.0.len());
    rotated.extend_from_slice(&unique[seam..]);
    rotated.extend_from_slice(&unique[..=seam]);
    geo::LineString::new(rotated)
}

fn merge_formline_ranges(ranges: &mut Vec<FormlineRange>, max_gap_fraction: f64) {
    ranges.sort_by(|a, b| a.start.total_cmp(&b.start));
    let mut merged = Vec::<FormlineRange>::with_capacity(ranges.len());

    for range in ranges.drain(..) {
        if let Some(previous) = merged.last_mut()
            && range.start - previous.end <= max_gap_fraction
        {
            previous.end = previous.end.max(range.end);
            previous.important |= range.important;
        } else {
            merged.push(range);
        }
    }

    *ranges = merged;
}

fn elongate_range(range: &mut FormlineRange, target_fraction: f64) {
    let center = (range.start + range.end) / 2.;
    range.start = center - target_fraction / 2.;
    range.end = center + target_fraction / 2.;

    if range.start < 0. {
        range.end -= range.start;
        range.start = 0.;
    }
    if range.end > 1. {
        range.start -= range.end - 1.;
        range.end = 1.;
    }
    range.start = range.start.max(0.);
}

fn line_substring(
    source: &geo::LineString,
    start_fraction: f64,
    end_fraction: f64,
) -> Option<geo::LineString> {
    let total_length = Euclidean.length(source);
    let start_distance = start_fraction.clamp(0., 1.) * total_length;
    let end_distance = end_fraction.clamp(0., 1.) * total_length;
    if end_distance - start_distance <= f64::EPSILON {
        return None;
    }

    let mut output = Vec::new();
    let mut distance = 0.;
    for segment in source.0.windows(2) {
        let a = segment[0];
        let b = segment[1];
        let segment_length = (b.x - a.x).hypot(b.y - a.y);
        if segment_length <= f64::EPSILON {
            continue;
        }
        let next_distance = distance + segment_length;
        if next_distance < start_distance {
            distance = next_distance;
            continue;
        }
        if distance > end_distance {
            break;
        }

        let overlap_start = start_distance.max(distance);
        let overlap_end = end_distance.min(next_distance);
        if overlap_start <= overlap_end {
            push_unique_coord(
                &mut output,
                interpolate_coord(a, b, (overlap_start - distance) / segment_length),
            );
            push_unique_coord(
                &mut output,
                interpolate_coord(a, b, (overlap_end - distance) / segment_length),
            );
        }
        if next_distance >= end_distance {
            break;
        }
        distance = next_distance;
    }

    (output.len() >= 2).then(|| geo::LineString::new(output))
}

fn interpolate_coord(a: geo::Coord, b: geo::Coord, fraction: f64) -> geo::Coord {
    geo::Coord {
        x: a.x + (b.x - a.x) * fraction,
        y: a.y + (b.y - a.y) * fraction,
    }
}

fn push_unique_coord(coords: &mut Vec<geo::Coord>, coord: geo::Coord) {
    let is_duplicate = coords.last().is_some_and(|previous| {
        (previous.x - coord.x).abs() <= f64::EPSILON && (previous.y - coord.y).abs() <= f64::EPSILON
    });
    if !is_duplicate {
        coords.push(coord);
    }
}

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

    let formline_pruner = if params.contour.form_lines {
        match params.contour.form_line_prune_algorithm {
            FormlinePruneAlgo::None => None,
            FormlinePruneAlgo::TerrainChange => Some(FormlinePruner::from_terrain_change(
                true_dem, &clip_poly, params,
            )),
            FormlinePruneAlgo::InterpolationError => {
                Some(FormlinePruner::from_contour_interpolation_error(
                    &contours,
                    &adjusted_dem,
                    true_dem,
                    None,
                    &clip_poly,
                    params,
                )?)
            }
        }
    } else {
        None
    };

    let mut objects = Vec::with_capacity(contours.0.len());

    for c_level in contours.0 {
        let z = c_level.z;

        let symbol = contour_symbol(z, params.contour.interval);
        let lines = if symbol == LineSymbol::FormLine {
            let pruned = c_level
                .lines
                .0
                .iter()
                .flat_map(|line| prune_formline(formline_pruner.as_ref(), line))
                .collect();
            cut_overlay.clip(&geo::MultiLineString::new(pruned), false)
        } else {
            cut_overlay.clip(&c_level.lines, false)
        };
        for line in lines {
            let mut c_object = MapObject::Line {
                object: line,
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

    let needs_interpolated_dem = compute_energy
        || params.contour.form_lines
            && params.contour.form_line_prune_algorithm == FormlinePruneAlgo::InterpolationError;
    let interpolated_dem = if needs_interpolated_dem {
        let mut interpolated_dem = dem.clone();
        contour_set.interpolate(&mut interpolated_dem, dem)?;
        Some(interpolated_dem)
    } else {
        None
    };

    let (error, energy) = if compute_energy {
        (
            true_dem.error(
                interpolated_dem
                    .as_ref()
                    .expect("computed interpolation when scoring was requested"),
            ),
            contour_set.energy(1),
        )
    } else {
        (0., 0.)
    };

    let formline_pruner = if params.contour.form_lines {
        match params.contour.form_line_prune_algorithm {
            FormlinePruneAlgo::None => None,
            FormlinePruneAlgo::TerrainChange => Some(FormlinePruner::from_terrain_change(
                true_dem, &clip_poly, params,
            )),
            FormlinePruneAlgo::InterpolationError => {
                Some(FormlinePruner::from_contour_interpolation_error(
                    &contour_set,
                    dem,
                    true_dem,
                    interpolated_dem.as_ref(),
                    &clip_poly,
                    params,
                )?)
            }
        }
    } else {
        None
    };

    let mut objects = Vec::with_capacity(contour_set.0.len());
    for c_level in contour_set.0 {
        let symbol = contour_symbol(c_level.z, params.contour.interval);
        let lines = if symbol == LineSymbol::FormLine {
            let pruned = c_level
                .lines
                .0
                .iter()
                .flat_map(|line| prune_formline(formline_pruner.as_ref(), line))
                .collect();
            cut_overlay.clip(&geo::MultiLineString::new(pruned), false)
        } else {
            cut_overlay.clip(&c_level.lines, false)
        };
        for line in lines {
            let mut c_object = MapObject::Line {
                object: line,
                symbol,
                tags: HashMap::new(),
            };
            c_object.add_elevation_tag(c_level.z);

            objects.push(c_object);
        }
    }
    Ok((objects, error, energy))
}
