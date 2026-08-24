use crate::geometry::{ContourLevel, ContourSet, MapMultiPolygon};
use crate::map_gen::egui_map::{LineSymbol, MapObject};
use crate::parameters::{ContourAlgo, FormlinePruneAlgo, MapParameters};
use crate::raster::Dfm;
use crate::raster::dfm::Elevation;

use geo::{BooleanOps, Buffer, Contains, Euclidean, Length, LineLocatePoint, Simplify};

use std::collections::HashMap;

const FORMLINE_PRUNE_BUFFER_METERS: f64 = 2.;

fn contour_symbol(elevation: f32, interval: f32) -> LineSymbol {
    let frac = elevation / interval;
    let index_frac = frac / 5.;

    if (index_frac - index_frac.round()).abs() < 0.05 {
        LineSymbol::IndexContour
    } else if (frac - frac.round()).abs() < 0.05 {
        LineSymbol::Contour
    } else {
        LineSymbol::FormLine
    }
}

struct FormlinePruner {
    important_terrain: geo::MultiPolygon,
    buffered_terrain: geo::MultiPolygon,
    scale: crate::parameters::Scale,
    min_open_length_m: f64,
    min_closed_length_m: f64,
    reconnect_gap_m: f64,
    closed_seed_length_m: f64,
    closed_all_or_none_max_length_m: f64,
    protected_extrema: Vec<geo::Coord>,
}

#[derive(Clone, Copy, Debug)]
struct FormlineRange {
    source_line_id: u64,
    elevation: f32,
    start: f64,
    end: f64,
    important: bool,
}

impl FormlinePruner {
    fn minimum_open_length(&self) -> f64 {
        if self.min_open_length_m > 0. {
            self.min_open_length_m
        } else {
            LineSymbol::FormLine.min_length(self.scale, false) * self.scale.denominator()
                / 1_000_000.
        }
    }

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
        threshold: f32,
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
            min_open_length_m: params.contour.form_line_min_open_length_m,
            min_closed_length_m: params.contour.form_line_min_closed_length_m,
            reconnect_gap_m: params.contour.form_line_reconnect_gap_m.max(0.),
            closed_seed_length_m: params.contour.form_line_closed_seed_length_m.max(0.),
            closed_all_or_none_max_length_m: params
                .contour
                .form_line_closed_all_or_none_max_length_m
                .max(0.),
            protected_extrema: Vec::new(),
        }
    }

    fn with_protected_extrema(mut self, extrema: &[geo::Coord]) -> Self {
        self.protected_extrema.extend_from_slice(extrema);
        self
    }

    fn prune(
        &self,
        source_line_id: u64,
        elevation: f32,
        source: &geo::LineString,
    ) -> Vec<geo::LineString> {
        let source_length = Euclidean.length(source);
        if source.0.len() < 2 || source_length <= f64::EPSILON {
            return Vec::new();
        }

        let symbol_minimum = |closed| {
            LineSymbol::FormLine.min_length(self.scale, closed) * self.scale.denominator()
                / 1_000_000.
        };
        if source.is_closed() {
            let important = self
                .important_terrain
                .clip(&geo::MultiLineString::new(vec![source.clone()]), false);
            let has_seed = important
                .iter()
                .any(|fragment| Euclidean.length(fragment) >= self.closed_seed_length_m);
            if !has_seed {
                return Vec::new();
            }
            let ring = geo::Polygon::new(source.clone(), Vec::new());
            let protected = self
                .protected_extrema
                .iter()
                .any(|coordinate| ring.contains(&geo::Point(*coordinate)));
            if protected {
                return vec![source.clone()];
            }
            let closed_minimum = if self.min_closed_length_m > 0. {
                self.min_closed_length_m
            } else {
                symbol_minimum(true)
            };
            if source_length < closed_minimum {
                return Vec::new();
            }
            if source_length <= self.closed_all_or_none_max_length_m {
                return vec![source.clone()];
            }
        }

        // LineSymbol minimum lengths are expressed in map micrometres. Convert
        // them back to projected ground metres before comparing geometry.
        let min_length = self.minimum_open_length();

        let clipped = self
            .buffered_terrain
            .clip(&geo::MultiLineString::new(vec![source.clone()]), false);
        if clipped.iter().any(|fragment| fragment.is_closed()) {
            return vec![source.clone()];
        }

        let mut ranges = clipped
            .0
            .iter()
            .flat_map(|fragment| self.fragment_ranges(source_line_id, elevation, source, fragment))
            .collect::<Vec<_>>();

        merge_formline_ranges(&mut ranges, f64::EPSILON);
        if source.is_closed() && !ranges.is_empty() {
            let wrap_gap = 1. - ranges.last().expect("nonempty ranges").end + ranges[0].start;
            if let Some((gap_index, gap)) = ranges
                .windows(2)
                .enumerate()
                .map(|(index, pair)| (index, pair[1].start - pair[0].end))
                .max_by(|a, b| a.1.total_cmp(&b.1))
                && gap > wrap_gap + 1e-9
            {
                let seam = (ranges[gap_index].end + ranges[gap_index + 1].start) / 2.;
                return self.prune(source_line_id, elevation, &rotate_closed_line(source, seam));
            }
        }
        merge_formline_ranges(&mut ranges, self.reconnect_gap_m / source_length);
        if source.is_closed()
            && ranges.len() == 1
            && (1. - ranges[0].end + ranges[0].start) * source_length <= self.reconnect_gap_m
        {
            return vec![source.clone()];
        }

        ranges.retain(|range| {
            let length = (range.end - range.start) * source_length;
            length >= min_length || range.important
        });
        let target_fraction = (min_length / source_length).min(1.);
        let gap_fraction = self.reconnect_gap_m / source_length;
        for index in 0..ranges.len() {
            if (ranges[index].end - ranges[index].start) * source_length >= min_length {
                continue;
            }
            let lower = if index == 0 {
                0.
            } else {
                (ranges[index - 1].end + gap_fraction + f64::EPSILON).min(ranges[index].start)
            };
            let upper = if index + 1 == ranges.len() {
                1.
            } else {
                (ranges[index + 1].start - gap_fraction - f64::EPSILON).max(ranges[index].end)
            };
            let center = (ranges[index].start + ranges[index].end) / 2.;
            let available = upper - lower;
            let length = target_fraction.min(available);
            let mut start = center - length / 2.;
            let mut end = center + length / 2.;
            if start < lower {
                end += lower - start;
                start = lower;
            }
            if end > upper {
                start -= end - upper;
                end = upper;
            }
            ranges[index].start = start.max(lower);
            ranges[index].end = end.min(upper);
        }
        merge_formline_ranges(&mut ranges, f64::EPSILON);
        ranges.retain(|range| (range.end - range.start) * source_length >= min_length);

        ranges
            .into_iter()
            .filter_map(|range| line_substring(source, range.start, range.end))
            .collect()
    }

    fn fragment_ranges(
        &self,
        source_line_id: u64,
        elevation: f32,
        source: &geo::LineString,
        fragment: &geo::LineString,
    ) -> Vec<FormlineRange> {
        let Some(first) = fragment.0.first() else {
            return Vec::new();
        };
        let Some(last) = fragment.0.last() else {
            return Vec::new();
        };
        let Some(start) = source.line_locate_point(&geo::Point(*first)) else {
            return Vec::new();
        };
        let Some(end) = source.line_locate_point(&geo::Point(*last)) else {
            return Vec::new();
        };
        let important = Euclidean.length(
            &self
                .important_terrain
                .clip(&geo::MultiLineString::new(vec![fragment.clone()]), false),
        ) > crate::SIMPLIFICATION_DIST;

        let range = |start, end| FormlineRange {
            source_line_id,
            elevation,
            start,
            end,
            important,
        };
        let low = start.min(end);
        let high = start.max(end);
        let direct = high - low;
        let fragment_fraction = Euclidean.length(fragment) / Euclidean.length(source);
        if source.is_closed()
            && ((1. - direct) - fragment_fraction).abs() < (direct - fragment_fraction).abs()
        {
            let mut ranges = Vec::with_capacity(2);
            if low > f64::EPSILON {
                ranges.push(range(0., low));
            }
            if high < 1. - f64::EPSILON {
                ranges.push(range(high, 1.));
            }
            ranges
        } else {
            vec![range(low, high)]
        }
    }
}

fn prune_formline(
    pruner: Option<&FormlinePruner>,
    source_line_id: u64,
    elevation: f32,
    line: &geo::LineString,
) -> Vec<geo::LineString> {
    pruner
        .map(|pruner| pruner.prune(source_line_id, elevation, line))
        .unwrap_or_else(|| vec![line.clone()])
}

fn rotate_closed_line(source: &geo::LineString, seam_fraction: f64) -> geo::LineString {
    let target = seam_fraction.clamp(0., 1.) * Euclidean.length(source);
    let mut distance = 0.;
    for (index, segment) in source.0.windows(2).enumerate() {
        let length = (segment[1].x - segment[0].x).hypot(segment[1].y - segment[0].y);
        if distance + length >= target {
            let seam = interpolate_coord(segment[0], segment[1], (target - distance) / length);
            let mut rotated = Vec::with_capacity(source.0.len() + 1);
            push_unique_coord(&mut rotated, seam);
            for &coordinate in &source.0[index + 1..source.0.len() - 1] {
                push_unique_coord(&mut rotated, coordinate);
            }
            for &coordinate in &source.0[..=index] {
                push_unique_coord(&mut rotated, coordinate);
            }
            push_unique_coord(&mut rotated, seam);
            return geo::LineString::new(rotated);
        }
        distance += length;
    }
    source.clone()
}

fn merge_formline_ranges(ranges: &mut Vec<FormlineRange>, max_gap_fraction: f64) {
    ranges.sort_by(|a, b| a.start.total_cmp(&b.start));
    let mut merged = Vec::<FormlineRange>::with_capacity(ranges.len());

    for range in ranges.drain(..) {
        if let Some(previous) = merged.last_mut()
            && range.source_line_id == previous.source_line_id
            && range.elevation == previous.elevation
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
    z_range: (f32, f32),
    cut_overlay: &geo::Polygon,
    thresholds: (f32, f32),
    params: &MapParameters,
) -> crate::Result<(Vec<MapObject>, f32, f32)> {
    let (min_threshold, conv_threshold) = thresholds;
    let min_threshold = f64::from(min_threshold);
    let conv_threshold = f64::from(conv_threshold);

    let effective_interval = if params.contour.form_lines {
        params.contour.interval / 2.
    } else {
        params.contour.interval
    };

    let effective_interval_f64 = f64::from(effective_interval);
    let min_z = f64::from(z_range.0);
    let max_z = f64::from(z_range.1);
    let c_levels = ((max_z - min_z) / effective_interval_f64).ceil() as usize + 1;
    let start_level = (min_z / effective_interval_f64).floor() * effective_interval_f64;

    let mut adjusted_dem = true_dem.smoothen(15., 15, 10);
    let mut interpolated_dem = adjusted_dem.clone();

    let clip_poly = geo::Polygon::new(
        geo::LineString::new(vec![
            true_dem.index2coord(0, 0),
            true_dem.index2coord(true_dem.height() - 1, 0),
            true_dem.index2coord(true_dem.height() - 1, true_dem.width() - 1),
            true_dem.index2coord(0, true_dem.width() - 1),
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
            let c_level = (c_index as f64 * effective_interval_f64 + start_level) as f32;

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

        score = error + f64::from(params.contour.algo_lambda) * energy;

        if score <= min_threshold || (score - prev_score).abs() <= conv_threshold {
            break;
        }

        // adjust dem, increasing frequency decreasing amplitude
        let filter_half_size = ((params.contour.algo_steps - iterations) as f64
            / params.contour.algo_steps as f64
            * 30.) as usize;
        let filter_amplitude =
            (params.contour.algo_steps - iterations) as f32 / (params.contour.algo_steps as f32);

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

    for (level_index, c_level) in contours.0.into_iter().enumerate() {
        let z = c_level.z;

        let symbol = contour_symbol(z, params.contour.interval);
        let lines = if symbol == LineSymbol::FormLine {
            let pruned = c_level
                .lines
                .0
                .iter()
                .enumerate()
                .flat_map(|(line_index, line)| {
                    prune_formline(
                        formline_pruner.as_ref(),
                        ((level_index as u64) << 32) | line_index as u64,
                        z,
                        line,
                    )
                })
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

    Ok((objects, error as f32, energy as f32))
}

pub fn compute_scalar_field_contours(
    true_dem: &Dfm<Elevation>,
    z_range: (f32, f32),
    cut_overlay: &geo::Polygon,
    params: &MapParameters,
    compute_energy: bool,
) -> crate::Result<(Vec<MapObject>, f32, f32)> {
    let (adjusted, diagnostics) = super::contour_field::optimize_contour_field(
        true_dem,
        params.contour.interval,
        &params.contour.contour_field,
    )?;
    log::info!(
        "contour field: {} iterations in {:.2?}, adjustment max/rms {:.3}/{:.3} m, \
         bound fraction {:.3}, energies fidelity/TV/Hessian {:.3}/{:.3}/{:.3}, \
         persistence removed/preserved {}/{}",
        diagnostics.iterations,
        diagnostics.runtime,
        diagnostics.maximum_adjustment,
        diagnostics.rms_adjustment,
        diagnostics.fraction_at_bound,
        diagnostics.fidelity_energy,
        diagnostics.weighted_tv_energy,
        diagnostics.hessian_energy,
        diagnostics.persistence_pairs_removed,
        diagnostics.persistence_pairs_preserved,
    );
    let effective_interval = if params.contour.form_lines {
        params.contour.interval / 2.
    } else {
        params.contour.interval
    };
    let level_count =
        ((f64::from(z_range.1 - z_range.0) / f64::from(effective_interval)).ceil() as usize) + 1;
    let start = (f64::from(z_range.0) / f64::from(effective_interval)).floor()
        * f64::from(effective_interval);
    let clip_polygon = geo::Polygon::new(
        geo::LineString::new(vec![
            true_dem.index2coord(0, 0),
            true_dem.index2coord(true_dem.height() - 1, 0),
            true_dem.index2coord(true_dem.height() - 1, true_dem.width() - 1),
            true_dem.index2coord(0, true_dem.width() - 1),
            true_dem.index2coord(0, 0),
        ]),
        vec![],
    );
    let mut contour_set = ContourSet::with_capacity(level_count);
    for index in 0..level_count {
        let level = (start + index as f64 * f64::from(effective_interval)) as f32;
        let lines = clip_polygon.clip(&adjusted.marching_squares(level), false);
        contour_set.0.push(ContourLevel::new(lines, level));
    }

    let mut contour_dem = Dfm::<Elevation>::new_like(&adjusted);
    contour_dem.field.copy_from_slice(&adjusted.field);
    let needs_interpolation = compute_energy
        || params.contour.form_lines
            && params.contour.form_line_prune_algorithm == FormlinePruneAlgo::InterpolationError;
    let interpolated = if needs_interpolation {
        let mut interpolated = contour_dem.clone();
        contour_set.interpolate(&mut interpolated, &contour_dem)?;
        Some(interpolated)
    } else {
        None
    };
    let (error, energy) = if compute_energy {
        (
            true_dem.error(
                interpolated
                    .as_ref()
                    .expect("interpolation was requested for contour scoring"),
            ),
            contour_set.energy(1),
        )
    } else {
        (0., 0.)
    };
    let pruner = if params.contour.form_lines {
        Some(
            FormlinePruner::from_contour_interpolation_error(
                &contour_set,
                &contour_dem,
                true_dem,
                interpolated.as_ref(),
                &clip_polygon,
                params,
            )?
            .with_protected_extrema(&diagnostics.protected_extrema),
        )
    } else {
        None
    };

    let mut objects = Vec::new();
    for (level_index, contour) in contour_set.0.into_iter().enumerate() {
        let symbol = contour_symbol(contour.z, params.contour.interval);
        let lines = if symbol == LineSymbol::FormLine {
            let retained = contour
                .lines
                .iter()
                .enumerate()
                .flat_map(|(line_index, line)| {
                    prune_formline(
                        pruner.as_ref(),
                        ((level_index as u64) << 32) | line_index as u64,
                        contour.z,
                        line,
                    )
                })
                .collect();
            cut_overlay.clip(&geo::MultiLineString::new(retained), false)
        } else {
            cut_overlay.clip(&contour.lines, false)
        };
        for line in lines {
            if symbol == LineSymbol::FormLine
                && !line.is_closed()
                && pruner
                    .as_ref()
                    .is_some_and(|pruner| Euclidean.length(&line) < pruner.minimum_open_length())
            {
                continue;
            }
            for &coordinate in &line.0 {
                if let Some(original) = true_dem.sample_bilinear(coordinate) {
                    anyhow::ensure!(
                        (original - contour.z).abs() <= params.contour.interval * 0.25 + 1e-3,
                        "adjusted contour exceeded its vertical tolerance"
                    );
                }
            }
            let mut object = MapObject::Line {
                object: line,
                symbol,
                tags: HashMap::new(),
            };
            object.add_elevation_tag(contour.z);
            object.preserve_contour_geometry();
            objects.push(object);
        }
    }
    Ok((objects, error as f32, energy as f32))
}

// used for raw and smoothed contour extraction, with scoring which complicates it a bit
// smoothing happens on the DEM level
pub fn extract_contours(
    true_dem: &Dfm<Elevation>,
    z_range: (f32, f32),
    cut_overlay: &geo::Polygon,
    params: &MapParameters,
    compute_energy: bool,
) -> crate::Result<(Vec<MapObject>, f32, f32)> {
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

    let effective_interval_f64 = f64::from(effective_interval);
    let min_z = f64::from(z_range.0);
    let max_z = f64::from(z_range.1);
    let c_levels = ((max_z - min_z) / effective_interval_f64).ceil() as usize + 1;
    let start_level = (min_z / effective_interval_f64).floor() * effective_interval_f64;

    let clip_poly = geo::Polygon::new(
        geo::LineString::new(vec![
            true_dem.index2coord(0, 0),
            true_dem.index2coord(true_dem.height() - 1, 0),
            true_dem.index2coord(true_dem.height() - 1, true_dem.width() - 1),
            true_dem.index2coord(0, true_dem.width() - 1),
            true_dem.index2coord(0, 0),
        ]),
        vec![],
    );
    let mut contour_set = ContourSet::with_capacity(c_levels);

    for c_index in 0..c_levels {
        let c_level = (c_index as f64 * effective_interval_f64 + start_level) as f32;

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
    for (level_index, c_level) in contour_set.0.into_iter().enumerate() {
        let symbol = contour_symbol(c_level.z, params.contour.interval);
        let lines = if symbol == LineSymbol::FormLine {
            let pruned = c_level
                .lines
                .0
                .iter()
                .enumerate()
                .flat_map(|(line_index, line)| {
                    prune_formline(
                        formline_pruner.as_ref(),
                        ((level_index as u64) << 32) | line_index as u64,
                        c_level.z,
                        line,
                    )
                })
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
    Ok((objects, error as f32, energy as f32))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parameters::Scale;
    use crate::raster::DfmGrid;

    fn ring_pruner(
        protected: bool,
        closed_minimum: f64,
        all_or_none_maximum: f64,
    ) -> FormlinePruner {
        let important = geo::MultiPolygon::new(vec![
            geo::Rect::new(geo::coord! { x: 2., y: -1. }, geo::coord! { x: 6., y: 1. })
                .to_polygon(),
        ]);
        FormlinePruner {
            buffered_terrain: important.buffer(1.),
            important_terrain: important,
            scale: Scale::S15_000,
            min_open_length_m: 5.,
            min_closed_length_m: closed_minimum,
            reconnect_gap_m: 3.,
            closed_seed_length_m: 1.,
            closed_all_or_none_max_length_m: all_or_none_maximum,
            protected_extrema: protected
                .then_some(geo::coord! { x: 5., y: 5. })
                .into_iter()
                .collect(),
        }
    }

    fn square_ring() -> geo::LineString {
        geo::LineString::new(vec![
            geo::coord! { x: 0., y: 0. },
            geo::coord! { x: 10., y: 0. },
            geo::coord! { x: 10., y: 10. },
            geo::coord! { x: 0., y: 10. },
            geo::coord! { x: 0., y: 0. },
        ])
    }

    #[test]
    fn range_reconnection_respects_culled_arc_length() {
        let mut short_gap = vec![
            FormlineRange {
                source_line_id: 1,
                elevation: 2.5,
                start: 0.1,
                end: 0.3,
                important: true,
            },
            FormlineRange {
                source_line_id: 1,
                elevation: 2.5,
                start: 0.32,
                end: 0.5,
                important: false,
            },
        ];
        merge_formline_ranges(&mut short_gap, 0.03);
        assert_eq!(short_gap.len(), 1);

        let mut long_gap = vec![
            FormlineRange {
                source_line_id: 1,
                elevation: 2.5,
                start: 0.1,
                end: 0.3,
                important: true,
            },
            FormlineRange {
                source_line_id: 1,
                elevation: 2.5,
                start: 0.35,
                end: 0.5,
                important: true,
            },
        ];
        merge_formline_ranges(&mut long_gap, 0.03);
        assert_eq!(long_gap.len(), 2);

        let mut different_sources = short_gap;
        different_sources.push(FormlineRange {
            source_line_id: 2,
            elevation: 2.5,
            start: 0.51,
            end: 0.7,
            important: true,
        });
        merge_formline_ranges(&mut different_sources, 1.);
        assert_eq!(different_sources.len(), 2);
    }

    #[test]
    fn protected_seeded_small_ring_remains_closed() {
        let retained = ring_pruner(true, 100., 0.).prune(1, 2.5, &square_ring());
        assert_eq!(retained, vec![square_ring()]);
        assert!(retained[0].is_closed());
    }

    #[test]
    fn unprotected_subminimum_ring_is_removed() {
        assert!(
            ring_pruner(false, 100., 0.)
                .prune(1, 2.5, &square_ring())
                .is_empty()
        );
    }

    #[test]
    fn qualifying_small_ring_is_all_or_nothing() {
        let retained = ring_pruner(false, 20., 50.).prune(1, 2.5, &square_ring());
        assert_eq!(retained, vec![square_ring()]);
    }

    #[test]
    fn long_ring_pruning_is_independent_of_stored_seam() {
        let pruner = ring_pruner(false, 20., 0.);
        let first = pruner.prune(1, 2.5, &square_ring());
        let second = pruner.prune(1, 2.5, &rotate_closed_line(&square_ring(), 0.1));
        assert_eq!(first.len(), second.len());
        let first_length = first.iter().map(|line| Euclidean.length(line)).sum::<f64>();
        let second_length = second
            .iter()
            .map(|line| Euclidean.length(line))
            .sum::<f64>();
        assert!((first_length - second_length).abs() < 1e-6);
    }

    #[test]
    fn scalar_field_pipeline_publishes_only_bounded_contours() {
        let grid = DfmGrid::new(16, 16, 0.5, geo::coord! { x: 0.25, y: 7.75 }).unwrap();
        let mut source = Dfm::<Elevation>::new(grid);
        for y in 0..source.height() {
            for x in 0..source.width() {
                let point = source.index2coord(y, x);
                source[(y, x)] = (10. + 0.4 * point.x + 0.1 * point.y) as f32;
            }
        }
        let mut params = MapParameters::default();
        params.contour.algorithm = ContourAlgo::WeightedScalarField;
        params.contour.interval = 1.;
        params.contour.contour_field.max_iterations = 6;
        params.contour.contour_field.iterations_per_level = vec![2, 2, 2];
        params.contour.contour_field.slope_fit_radius_m = 1.;
        params.contour.contour_field.curvature_fit_radius_m = 1.;
        params.contour.contour_field.solver_guard_distance_m = 0.5;
        let cut = geo::Rect::new(source.index2coord(13, 2), source.index2coord(2, 13)).to_polygon();
        let z_range = source
            .field
            .iter()
            .fold((f32::INFINITY, f32::NEG_INFINITY), |(min, max), &value| {
                (min.min(value), max.max(value))
            });
        let (objects, _, _) =
            compute_scalar_field_contours(&source, z_range, &cut, &params, false).unwrap();
        assert!(!objects.is_empty());
        for object in objects {
            let MapObject::Line {
                object: line, tags, ..
            } = object
            else {
                panic!("contour pipeline emitted a non-line object");
            };
            let level = tags["Elevation"].parse::<f32>().unwrap();
            assert!(line.0.iter().all(|&coordinate| {
                source
                    .sample_bilinear(coordinate)
                    .is_none_or(|value| (value - level).abs() <= 0.251)
            }));
        }
    }
}
