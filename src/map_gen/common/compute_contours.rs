use crate::geometry::{ContourLevel, ContourSet, MapLineString, MapMultiPolygon};
use crate::map_gen::egui_map::{LineSymbol, MapObject};
use crate::parameters::{ContourAlgo, FormlinePruneAlgo, MapParameters};
use crate::raster::Dfm;
use crate::raster::dfm::{AdjustedElevation, Elevation, RasterMarker};

use geo::{BooleanOps, Buffer, Contains, Euclidean, Length, LineLocatePoint, Simplify};

use std::collections::HashMap;
use std::sync::Arc;

const FORMLINE_PRUNE_BUFFER_METERS: f64 = 2.;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ContourLevelKind {
    Index,
    Regular,
    FormLine,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct ContourLevelSpec {
    ordinal: i64,
    elevation: f32,
    kind: ContourLevelKind,
}

impl ContourLevelSpec {
    fn symbol(self) -> LineSymbol {
        match self.kind {
            ContourLevelKind::Index => LineSymbol::IndexContour,
            ContourLevelKind::Regular => LineSymbol::Contour,
            ContourLevelKind::FormLine => LineSymbol::FormLine,
        }
    }
}

fn plan_contour_levels(
    z_range: (f32, f32),
    regular_interval: f32,
    include_form_lines: bool,
) -> crate::Result<Vec<ContourLevelSpec>> {
    anyhow::ensure!(
        regular_interval.is_finite() && regular_interval > 0.,
        "contour interval must be finite and positive"
    );
    anyhow::ensure!(
        z_range.0.is_finite() && z_range.1.is_finite() && z_range.0 <= z_range.1,
        "contour elevation range must be finite and ordered"
    );

    let subdivisions = if include_form_lines { 2_i64 } else { 1_i64 };
    let scaled =
        |elevation: f32| f64::from(elevation) * subdivisions as f64 / f64::from(regular_interval);
    let first_f64 = scaled(z_range.0).floor();
    let last_f64 = scaled(z_range.1).ceil();
    anyhow::ensure!(
        first_f64 >= i64::MIN as f64 && last_f64 <= i64::MAX as f64,
        "contour elevation range is too large"
    );
    let first = first_f64 as i64;
    let last = last_f64 as i64;
    let level_count = last
        .checked_sub(first)
        .and_then(|span| span.checked_add(1))
        .and_then(|count| usize::try_from(count).ok())
        .ok_or_else(|| anyhow::anyhow!("too many contour levels"))?;
    anyhow::ensure!(level_count <= 1_000_000, "too many contour levels");

    let classify = |ordinal: i64| {
        if include_form_lines && ordinal.rem_euclid(2) != 0 {
            ContourLevelKind::FormLine
        } else {
            let regular_ordinal = ordinal / subdivisions;
            if regular_ordinal.rem_euclid(5) == 0 {
                ContourLevelKind::Index
            } else {
                ContourLevelKind::Regular
            }
        }
    };
    let mut levels = Vec::<ContourLevelSpec>::with_capacity(level_count);
    for ordinal in first..=last {
        let elevation = (ordinal as f64 * f64::from(regular_interval) / subdivisions as f64) as f32;
        let candidate = ContourLevelSpec {
            ordinal,
            elevation,
            kind: classify(ordinal),
        };
        if let Some(previous) = levels.last_mut()
            && previous.elevation == candidate.elevation
        {
            let represented = scaled(candidate.elevation).round() as i64;
            if ordinal.abs_diff(represented) < previous.ordinal.abs_diff(represented) {
                *previous = candidate;
            }
        } else {
            levels.push(candidate);
        }
    }
    Ok(levels)
}

#[derive(Clone, Copy, Debug)]
enum ContourSimplification {
    #[cfg(test)]
    None,
    DouglasPeucker(f64),
}

fn extraction_polygon<T: RasterMarker>(raster: &Dfm<T>) -> geo::Polygon {
    geo::Polygon::new(
        geo::LineString::new(vec![
            raster.index2coord(0, 0),
            raster.index2coord(raster.height() - 1, 0),
            raster.index2coord(raster.height() - 1, raster.width() - 1),
            raster.index2coord(0, raster.width() - 1),
            raster.index2coord(0, 0),
        ]),
        vec![],
    )
}

fn extract_contour_set<T: RasterMarker>(
    field: &Dfm<T>,
    levels: &[ContourLevelSpec],
    domain: &geo::Polygon,
    simplification: ContourSimplification,
) -> ContourSet {
    ContourSet(
        levels
            .iter()
            .map(|level| {
                let lines = field.marching_squares(level.elevation);
                let lines = match simplification {
                    #[cfg(test)]
                    ContourSimplification::None => lines,
                    ContourSimplification::DouglasPeucker(tolerance) => lines.simplify(tolerance),
                };
                ContourLevel::new(domain.clip(&lines, false), level.elevation)
            })
            .collect(),
    )
}

enum FormlineImportance {
    All,
    Areas {
        important: geo::MultiPolygon,
        buffered: geo::MultiPolygon,
    },
}

struct FormlineGeometryRules {
    scale: crate::parameters::Scale,
    min_open_length_m: f64,
    min_closed_length_m: f64,
    reconnect_gap_m: f64,
    closed_seed_length_m: f64,
    closed_all_or_none_max_length_m: f64,
}

struct FormlinePostprocessor {
    importance: FormlineImportance,
    rules: FormlineGeometryRules,
    protected_features: Vec<super::contour_field::ProtectedPersistenceFeature>,
}

#[derive(Clone, Copy, Debug)]
struct FormlineRange {
    source_line_id: u64,
    elevation: f32,
    start: f64,
    end: f64,
    important: bool,
}

impl FormlinePostprocessor {
    fn minimum_open_length(&self) -> f64 {
        if self.rules.min_open_length_m > 0. {
            self.rules.min_open_length_m
        } else {
            LineSymbol::FormLine.min_length(self.rules.scale, false)
                * self.rules.scale.denominator()
                / 1_000_000.
        }
    }

    fn all(params: &MapParameters) -> Self {
        Self {
            importance: FormlineImportance::All,
            rules: FormlineGeometryRules::from_params(params),
            protected_features: Vec::new(),
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

    fn from_contour_interpolation_error<T: RasterMarker>(
        contour_set: &ContourSet,
        contour_dem: &Dfm<T>,
        true_dem: &Dfm<Elevation>,
        with_formlines: Option<&Dfm<Elevation>>,
        levels: &[ContourLevelSpec],
        clip_polygon: &geo::Polygon,
        params: &MapParameters,
    ) -> crate::Result<Self> {
        let computed_with_formlines = if with_formlines.is_none() {
            let mut interpolated = Dfm::<Elevation>::new_like(contour_dem);
            interpolated.field.copy_from_slice(&contour_dem.field);
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
                .zip(levels)
                .filter(|(_, spec)| spec.kind != ContourLevelKind::FormLine)
                .map(|(level, _)| level)
                .cloned()
                .collect(),
        );
        let mut without_formlines = Dfm::<Elevation>::new_like(contour_dem);
        without_formlines.field.copy_from_slice(&contour_dem.field);
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

    fn from_importance<T: RasterMarker>(
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
            importance: FormlineImportance::Areas {
                important: important_terrain,
                buffered: buffered_terrain,
            },
            rules: FormlineGeometryRules::from_params(params),
            protected_features: Vec::new(),
        }
    }

    fn with_protected_features(
        mut self,
        features: &[super::contour_field::ProtectedPersistenceFeature],
    ) -> Self {
        self.protected_features.extend_from_slice(features);
        self
    }

    fn prune(
        &self,
        source_line_id: u64,
        elevation: f32,
        source: &geo::LineString,
        protected: bool,
    ) -> Vec<geo::LineString> {
        let source_length = Euclidean.length(source);
        if source.0.len() < 2 || source_length <= f64::EPSILON {
            return Vec::new();
        }

        let symbol_minimum = |closed| {
            LineSymbol::FormLine.min_length(self.rules.scale, closed)
                * self.rules.scale.denominator()
                / 1_000_000.
        };
        if source.is_closed() {
            if protected {
                return vec![source.clone()];
            }
            let important = self.clip_to_important(source, false);
            let has_seed = important
                .iter()
                .any(|fragment| Euclidean.length(fragment) >= self.rules.closed_seed_length_m);
            if !has_seed {
                return Vec::new();
            }
            let closed_minimum = if self.rules.min_closed_length_m > 0. {
                self.rules.min_closed_length_m
            } else {
                symbol_minimum(true)
            };
            if source_length < closed_minimum {
                return Vec::new();
            }
            if source_length <= self.rules.closed_all_or_none_max_length_m {
                return vec![source.clone()];
            }
        }

        // LineSymbol minimum lengths are expressed in map micrometres. Convert
        // them back to projected ground metres before comparing geometry.
        let min_length = self.minimum_open_length();

        let clipped = self.clip_to_important(source, true);
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
                return self.prune(
                    source_line_id,
                    elevation,
                    &rotate_closed_line(source, seam),
                    protected,
                );
            }
        }
        merge_formline_ranges(&mut ranges, self.rules.reconnect_gap_m / source_length);
        if source.is_closed()
            && ranges.len() == 1
            && (1. - ranges[0].end + ranges[0].start) * source_length <= self.rules.reconnect_gap_m
        {
            return vec![source.clone()];
        }

        ranges.retain(|range| {
            let length = (range.end - range.start) * source_length;
            length >= min_length || range.important
        });
        let target_fraction = (min_length / source_length).min(1.);
        let gap_fraction = self.rules.reconnect_gap_m / source_length;
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
        let important = match &self.importance {
            FormlineImportance::All => true,
            FormlineImportance::Areas { important, .. } => {
                Euclidean.length(
                    &important.clip(&geo::MultiLineString::new(vec![fragment.clone()]), false),
                ) > crate::SIMPLIFICATION_DIST
            }
        };

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

    fn protected_line_indices(
        &self,
        elevation: f32,
        lines: &geo::MultiLineString,
    ) -> std::collections::HashSet<usize> {
        let mut protected = std::collections::HashSet::new();
        for feature in &self.protected_features {
            let low = feature.extremum_elevation.min(feature.saddle_elevation);
            let high = feature.extremum_elevation.max(feature.saddle_elevation);
            if elevation < low || elevation > high {
                continue;
            }
            let expected_positive = feature.kind == super::contour_field::ExtremumKind::Maximum;
            let best = lines
                .iter()
                .enumerate()
                .filter_map(|(index, line)| {
                    let area = line.line_string_signed_area()?;
                    if (area > 0.) != expected_positive
                        || !geo::Polygon::new(line.clone(), Vec::new())
                            .contains(&geo::Point(feature.extremum))
                    {
                        return None;
                    }
                    Some((index, area.abs()))
                })
                .min_by(|a, b| a.1.total_cmp(&b.1));
            if let Some((index, _)) = best {
                protected.insert(index);
            }
        }
        protected
    }

    fn clip_to_important(&self, source: &geo::LineString, buffered: bool) -> geo::MultiLineString {
        match &self.importance {
            FormlineImportance::All => geo::MultiLineString::new(vec![source.clone()]),
            FormlineImportance::Areas {
                important,
                buffered: buffered_terrain,
            } => {
                let region = if buffered {
                    buffered_terrain
                } else {
                    important
                };
                region.clip(&geo::MultiLineString::new(vec![source.clone()]), false)
            }
        }
    }
}

impl FormlineGeometryRules {
    fn from_params(params: &MapParameters) -> Self {
        Self {
            scale: params.scale,
            min_open_length_m: params.contour.form_line_geometry.minimum_open_length_m,
            min_closed_length_m: params.contour.form_line_geometry.minimum_closed_length_m,
            reconnect_gap_m: params.contour.form_line_geometry.reconnect_gap_m.max(0.),
            closed_seed_length_m: params
                .contour
                .form_line_geometry
                .closed_seed_length_m
                .max(0.),
            closed_all_or_none_max_length_m: params
                .contour
                .form_line_geometry
                .closed_all_or_none_max_length_m
                .max(0.),
        }
    }
}

fn prune_formline(
    pruner: Option<&FormlinePostprocessor>,
    source_line_id: u64,
    elevation: f32,
    line: &geo::LineString,
    protected: bool,
) -> Vec<geo::LineString> {
    pruner
        .map(|pruner| pruner.prune(source_line_id, elevation, line, protected))
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

struct ContourPipelineOptions<'a> {
    compute_energy: bool,
    validate_vertical_tolerance: bool,
    preserve_geometry: bool,
    snap_boundary_to_source: bool,
    protected_features: &'a [super::contour_field::ProtectedPersistenceFeature],
}

struct ContourPipelineContext<'a, T: RasterMarker> {
    true_dem: &'a Dfm<Elevation>,
    contour_dem: &'a Dfm<T>,
    levels: &'a [ContourLevelSpec],
    extraction_domain: &'a geo::Polygon,
    output_clip: &'a geo::Polygon,
    params: &'a MapParameters,
}

fn finish_contours<T: RasterMarker>(
    contour_set: ContourSet,
    context: ContourPipelineContext<'_, T>,
    options: ContourPipelineOptions<'_>,
) -> crate::Result<(Vec<MapObject>, f32, f32)> {
    let ContourPipelineContext {
        true_dem,
        contour_dem,
        levels,
        extraction_domain,
        output_clip,
        params,
    } = context;
    anyhow::ensure!(
        contour_set.0.len() == levels.len(),
        "contour levels and extracted geometry are inconsistent"
    );
    let needs_interpolation = options.compute_energy
        || params.contour.form_lines
            && params.contour.form_line_prune_algorithm == FormlinePruneAlgo::InterpolationError;
    let interpolated = if needs_interpolation {
        let mut interpolated = Dfm::<Elevation>::new_like(contour_dem);
        interpolated.field.copy_from_slice(&contour_dem.field);
        contour_set.interpolate(&mut interpolated, contour_dem)?;
        Some(interpolated)
    } else {
        None
    };
    let (error, energy) = if options.compute_energy {
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

    let formline_postprocessor = if params.contour.form_lines {
        let postprocessor = match params.contour.form_line_prune_algorithm {
            FormlinePruneAlgo::None => FormlinePostprocessor::all(params),
            FormlinePruneAlgo::TerrainChange => {
                FormlinePostprocessor::from_terrain_change(true_dem, extraction_domain, params)
            }
            FormlinePruneAlgo::InterpolationError => {
                FormlinePostprocessor::from_contour_interpolation_error(
                    &contour_set,
                    contour_dem,
                    true_dem,
                    interpolated.as_ref(),
                    levels,
                    extraction_domain,
                    params,
                )?
            }
        };
        Some(postprocessor.with_protected_features(options.protected_features))
    } else {
        None
    };

    let objects = emit_contour_objects(
        true_dem,
        contour_set,
        levels,
        output_clip,
        formline_postprocessor.as_ref(),
        params.contour.interval,
        options.validate_vertical_tolerance,
        options.preserve_geometry,
        options.snap_boundary_to_source,
    )?;
    Ok((objects, error as f32, energy as f32))
}

#[allow(clippy::too_many_arguments)]
fn emit_contour_objects(
    true_dem: &Dfm<Elevation>,
    contour_set: ContourSet,
    levels: &[ContourLevelSpec],
    output_clip: &geo::Polygon,
    formline_postprocessor: Option<&FormlinePostprocessor>,
    regular_interval: f32,
    validate_vertical_tolerance: bool,
    preserve_geometry: bool,
    snap_boundary_to_source: bool,
) -> crate::Result<Vec<MapObject>> {
    let mut objects = Vec::new();
    for (level_index, (contour, level)) in contour_set.0.into_iter().zip(levels).enumerate() {
        debug_assert_eq!(contour.z, level.elevation);
        let symbol = level.symbol();
        let lines = if level.kind == ContourLevelKind::FormLine {
            let protected_line_indices = formline_postprocessor
                .map(|postprocessor| {
                    postprocessor.protected_line_indices(level.elevation, &contour.lines)
                })
                .unwrap_or_default();
            let retained = contour
                .lines
                .iter()
                .enumerate()
                .flat_map(|(line_index, line)| {
                    prune_formline(
                        formline_postprocessor,
                        ((level_index as u64) << 32) | line_index as u64,
                        level.elevation,
                        line,
                        protected_line_indices.contains(&line_index),
                    )
                })
                .collect();
            output_clip.clip(&geo::MultiLineString::new(retained), false)
        } else {
            output_clip.clip(&contour.lines, false)
        };

        let boundary_reference = snap_boundary_to_source
            .then(|| contour_boundary_reference(true_dem, level.elevation, output_clip));
        for mut line in lines {
            if level.kind == ContourLevelKind::FormLine
                && !line.is_closed()
                && formline_postprocessor.is_some_and(|postprocessor| {
                    Euclidean.length(&line) < postprocessor.minimum_open_length()
                })
            {
                continue;
            }
            if let Some(reference) = boundary_reference.as_deref() {
                snap_boundary_endpoints(
                    &mut line,
                    output_clip,
                    reference,
                    (4. * true_dem.grid.cell_size_m).powi(2),
                );
            }
            if validate_vertical_tolerance {
                validate_contour_vertices(
                    true_dem,
                    &line,
                    level.elevation,
                    super::contour_field::adjustment_bound(regular_interval),
                )?;
            }
            let mut object = MapObject::Line {
                object: line,
                symbol,
                tags: HashMap::new(),
            };
            object.add_elevation_tag(level.elevation);
            object.stabilize_contour_seam();
            if preserve_geometry {
                object.preserve_contour_geometry();
            }
            objects.push(object);
        }
    }
    Ok(objects)
}

fn contour_boundary_reference(
    true_dem: &Dfm<Elevation>,
    elevation: f32,
    output_clip: &geo::Polygon,
) -> Vec<geo::Coord> {
    let clipped = output_clip.clip(&true_dem.marching_squares(elevation), false);
    let mut endpoints = Vec::new();
    for line in clipped {
        if line.is_closed() || line.0.len() < 2 {
            continue;
        }
        for coordinate in [line.0[0], line.0[line.0.len() - 1]] {
            if squared_distance_to_polygon_boundary(coordinate, output_clip) <= 1e-12 {
                push_unique_coord(&mut endpoints, coordinate);
            }
        }
    }
    endpoints
}

fn snap_boundary_endpoints(
    line: &mut geo::LineString,
    output_clip: &geo::Polygon,
    reference: &[geo::Coord],
    maximum_squared_distance: f64,
) {
    if line.is_closed() || line.0.len() < 2 || reference.is_empty() {
        return;
    }
    for index in [0, line.0.len() - 1] {
        let endpoint = line.0[index];
        if squared_distance_to_polygon_boundary(endpoint, output_clip) > 1e-12 {
            continue;
        }
        if let Some(nearest) = reference.iter().min_by(|a, b| {
            squared_distance(endpoint, **a).total_cmp(&squared_distance(endpoint, **b))
        }) && squared_distance(endpoint, *nearest) <= maximum_squared_distance
        {
            line.0[index] = *nearest;
        }
    }
}

fn squared_distance_to_polygon_boundary(coordinate: geo::Coord, polygon: &geo::Polygon) -> f64 {
    std::iter::once(polygon.exterior())
        .chain(polygon.interiors())
        .flat_map(|ring| ring.0.windows(2))
        .map(|segment| squared_distance_to_segment(coordinate, segment[0], segment[1]))
        .min_by(f64::total_cmp)
        .unwrap_or(f64::INFINITY)
}

fn squared_distance_to_segment(point: geo::Coord, a: geo::Coord, b: geo::Coord) -> f64 {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let length_squared = dx * dx + dy * dy;
    if length_squared <= f64::EPSILON {
        return squared_distance(point, a);
    }
    let projection = ((point.x - a.x) * dx + (point.y - a.y) * dy) / length_squared;
    let projection = projection.clamp(0., 1.);
    squared_distance(
        point,
        geo::coord! {
            x: a.x + projection * dx,
            y: a.y + projection * dy,
        },
    )
}

fn squared_distance(a: geo::Coord, b: geo::Coord) -> f64 {
    (a.x - b.x).powi(2) + (a.y - b.y).powi(2)
}

fn validate_contour_vertices(
    true_dem: &Dfm<Elevation>,
    line: &geo::LineString,
    elevation: f32,
    tolerance: f32,
) -> crate::Result<()> {
    for &coordinate in &line.0 {
        if let Some(original) = true_dem.sample_bilinear(coordinate) {
            anyhow::ensure!(
                (original - elevation).abs() <= tolerance + 1e-3,
                "contour vertex at ({:.3}, {:.3}) exceeded its vertical tolerance: \
                 level={elevation:.3}, source={original:.3}, tolerance={tolerance:.3}",
                coordinate.x,
                coordinate.y,
            );
        }
    }
    Ok(())
}

// used for the naive iterative interpolation error correction contour algorithm
pub fn compute_naive_contours(
    true_dem: &Dfm<Elevation>,
    z_range: (f32, f32),
    cut_overlay: &geo::Polygon,
    params: &MapParameters,
    compute_energy: bool,
) -> crate::Result<(Vec<MapObject>, f32, f32)> {
    let levels = plan_contour_levels(z_range, params.contour.interval, params.contour.form_lines)?;
    let mut adjusted_dem = true_dem.smoothen(15., 15, 10);
    let mut interpolated_dem = adjusted_dem.clone();
    let extraction_domain = extraction_polygon(true_dem);
    let simplification = ContourSimplification::DouglasPeucker(crate::SIMPLIFICATION_DIST);

    for iteration in 0..usize::from(params.contour.algo_steps) {
        let contours =
            extract_contour_set(&adjusted_dem, &levels, &extraction_domain, simplification);
        contours.interpolate(&mut interpolated_dem, &adjusted_dem)?;
        let remaining = usize::from(params.contour.algo_steps) - iteration;
        let filter_half_size =
            (remaining as f64 / f64::from(params.contour.algo_steps) * 30.) as usize;
        let filter_amplitude = remaining as f32 / f32::from(params.contour.algo_steps);

        adjusted_dem.adjust(
            true_dem,
            &interpolated_dem,
            filter_half_size,
            filter_amplitude,
        );
    }

    let contours = extract_contour_set(&adjusted_dem, &levels, &extraction_domain, simplification);
    finish_contours(
        contours,
        ContourPipelineContext {
            true_dem,
            contour_dem: &adjusted_dem,
            levels: &levels,
            extraction_domain: &extraction_domain,
            output_clip: cut_overlay,
            params,
        },
        ContourPipelineOptions {
            compute_energy,
            validate_vertical_tolerance: false,
            preserve_geometry: false,
            snap_boundary_to_source: true,
            protected_features: &[],
        },
    )
}

#[allow(dead_code)]
pub fn compute_scalar_field_contours(
    true_dem: &Dfm<Elevation>,
    z_range: (f32, f32),
    cut_overlay: &geo::Polygon,
    params: &MapParameters,
    compute_energy: bool,
) -> crate::Result<(Vec<MapObject>, f32, f32)> {
    let produced = produce_scalar_contour_field(true_dem, params)?;
    compute_scalar_field_contours_from_produced(
        true_dem,
        z_range,
        cut_overlay,
        params,
        compute_energy,
        &produced,
    )
}

#[derive(Clone)]
pub(crate) struct ProducedContourField {
    pub(crate) adjusted: Arc<Dfm<AdjustedElevation>>,
    pub(crate) diagnostics: Arc<super::contour_field::ContourFieldDiagnostics>,
}

pub(crate) fn produce_scalar_contour_field(
    true_dem: &Dfm<Elevation>,
    params: &MapParameters,
) -> crate::Result<ProducedContourField> {
    let (adjusted, diagnostics) = super::contour_field::optimize_contour_field(
        true_dem,
        params.contour.interval,
        &params.contour.contour_field,
    )?;
    Ok(ProducedContourField {
        adjusted: Arc::new(adjusted),
        diagnostics: Arc::new(diagnostics),
    })
}

pub(crate) fn compute_scalar_field_contours_from_produced(
    true_dem: &Dfm<Elevation>,
    z_range: (f32, f32),
    cut_overlay: &geo::Polygon,
    params: &MapParameters,
    compute_energy: bool,
    produced: &ProducedContourField,
) -> crate::Result<(Vec<MapObject>, f32, f32)> {
    let adjusted = &produced.adjusted;
    let diagnostics = &produced.diagnostics;
    let level_timings = diagnostics
        .timings
        .levels
        .iter()
        .map(|level| {
            format!(
                "{:.1}m:{}/{:.1?}+{:.1?}+{:.1?}",
                level.cell_size_m,
                level.iterations,
                level.transfer,
                level.operator_norm,
                level.solve,
            )
        })
        .collect::<Vec<_>>()
        .join(",");
    let target_work = diagnostics.persistence.target_work;
    let publication_work = diagnostics.persistence.publication_work;
    log::info!(
        "contour field: {} iterations in {:.2?}, adjustment max/rms {:.3}/{:.3} m, \
         bound fraction {:.3}, energies fidelity/TV/alignment/Hessian {:.3}/{:.3}/{:.3}/{:.3}, \
         persistence requested/removed/preserved/unresolved {}/{}/{}/{}, \
         stages persistence/derivatives/salience/audit/diagnostics \
         {:.1?}/{:.1?}/{:.1?}/{:.1?}/{:.1?}, levels [{}], \
         persistence work target/audit diagrams {}/{}, passes {}/{}, candidates {}/{}, \
         cancellations {}/{}, cells {}/{}",
        diagnostics.solver.iterations,
        diagnostics.timings.total,
        diagnostics.published.maximum_adjustment,
        diagnostics.published.rms_adjustment,
        diagnostics.published.fraction_at_bound,
        diagnostics.published.fidelity_energy,
        diagnostics.published.weighted_tv_energy,
        diagnostics.published.alignment_energy,
        diagnostics.published.hessian_energy,
        diagnostics.persistence.requested,
        diagnostics.persistence.verified_removed,
        diagnostics.persistence.preserved,
        diagnostics.persistence.unresolved,
        diagnostics.timings.target_persistence,
        diagnostics.timings.derivatives,
        diagnostics.timings.salience,
        diagnostics.timings.publication_persistence,
        diagnostics.timings.published_diagnostics,
        level_timings,
        target_work.diagram_builds,
        publication_work.diagram_builds,
        target_work.cancellation_passes,
        publication_work.cancellation_passes,
        target_work.candidates_considered,
        publication_work.candidates_considered,
        target_work.cancellations_applied,
        publication_work.cancellations_applied,
        target_work.affected_cells_written,
        publication_work.affected_cells_written,
    );
    let levels = plan_contour_levels(z_range, params.contour.interval, params.contour.form_lines)?;
    let extraction_domain = extraction_polygon(true_dem);
    let contours = extract_contour_set(
        adjusted,
        &levels,
        &extraction_domain,
        ContourSimplification::DouglasPeucker(crate::SIMPLIFICATION_DIST),
    );
    finish_contours(
        contours,
        ContourPipelineContext {
            true_dem,
            contour_dem: adjusted,
            levels: &levels,
            extraction_domain: &extraction_domain,
            output_clip: cut_overlay,
            params,
        },
        ContourPipelineOptions {
            compute_energy,
            validate_vertical_tolerance: true,
            preserve_geometry: true,
            snap_boundary_to_source: true,
            protected_features: &diagnostics.protected_features,
        },
    )
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
    let dem = if params.contour.algorithm == ContourAlgo::Raw {
        true_dem
    } else {
        &true_dem.smoothen(15., 15, params.contour.algo_steps as usize)
    };
    let levels = plan_contour_levels(z_range, params.contour.interval, params.contour.form_lines)?;
    let extraction_domain = extraction_polygon(true_dem);
    let contours = extract_contour_set(
        dem,
        &levels,
        &extraction_domain,
        ContourSimplification::DouglasPeucker(crate::SIMPLIFICATION_DIST),
    );
    finish_contours(
        contours,
        ContourPipelineContext {
            true_dem,
            contour_dem: dem,
            levels: &levels,
            extraction_domain: &extraction_domain,
            output_clip: cut_overlay,
            params,
        },
        ContourPipelineOptions {
            compute_energy,
            validate_vertical_tolerance: false,
            preserve_geometry: false,
            snap_boundary_to_source: params.contour.algorithm != ContourAlgo::Raw,
            protected_features: &[],
        },
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parameters::Scale;
    use crate::raster::DfmGrid;

    fn ring_pruner(
        _protected: bool,
        closed_minimum: f64,
        all_or_none_maximum: f64,
    ) -> FormlinePostprocessor {
        let important = geo::MultiPolygon::new(vec![
            geo::Rect::new(geo::coord! { x: 2., y: -1. }, geo::coord! { x: 6., y: 1. })
                .to_polygon(),
        ]);
        FormlinePostprocessor {
            importance: FormlineImportance::Areas {
                buffered: important.buffer(1.),
                important,
            },
            rules: FormlineGeometryRules {
                scale: Scale::S15_000,
                min_open_length_m: 5.,
                min_closed_length_m: closed_minimum,
                reconnect_gap_m: 3.,
                closed_seed_length_m: 1.,
                closed_all_or_none_max_length_m: all_or_none_maximum,
            },
            protected_features: Vec::new(),
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

    fn square_ring_between(min: f64, max: f64) -> geo::LineString {
        geo::LineString::new(vec![
            geo::coord! { x: min, y: min },
            geo::coord! { x: max, y: min },
            geo::coord! { x: max, y: max },
            geo::coord! { x: min, y: max },
            geo::coord! { x: min, y: min },
        ])
    }

    #[test]
    fn level_planning_is_symmetric_across_zero() {
        let levels = plan_contour_levels((-12.6, 12.6), 5., true).unwrap();
        let actual = levels
            .iter()
            .map(|level| (level.ordinal, level.elevation, level.kind))
            .collect::<Vec<_>>();
        assert_eq!(
            actual,
            vec![
                (-6, -15., ContourLevelKind::Regular),
                (-5, -12.5, ContourLevelKind::FormLine),
                (-4, -10., ContourLevelKind::Regular),
                (-3, -7.5, ContourLevelKind::FormLine),
                (-2, -5., ContourLevelKind::Regular),
                (-1, -2.5, ContourLevelKind::FormLine),
                (0, 0., ContourLevelKind::Index),
                (1, 2.5, ContourLevelKind::FormLine),
                (2, 5., ContourLevelKind::Regular),
                (3, 7.5, ContourLevelKind::FormLine),
                (4, 10., ContourLevelKind::Regular),
                (5, 12.5, ContourLevelKind::FormLine),
                (6, 15., ContourLevelKind::Regular),
            ]
        );
    }

    #[test]
    fn every_fifth_regular_level_is_an_index_contour() {
        let levels = plan_contour_levels((-30., 30.), 5., false).unwrap();
        for level in levels {
            assert_eq!(
                level.kind == ContourLevelKind::Index,
                level.ordinal.rem_euclid(5) == 0
            );
        }
    }

    #[test]
    fn invalid_level_inputs_are_rejected() {
        assert!(plan_contour_levels((0., 1.), 0., false).is_err());
        assert!(plan_contour_levels((0., 1.), f32::NAN, false).is_err());
        assert!(plan_contour_levels((2., 1.), 1., false).is_err());
        assert!(plan_contour_levels((0., f32::INFINITY), 1., false).is_err());
    }

    #[test]
    fn large_levels_do_not_drift_between_symbols() {
        let levels = plan_contour_levels((49_999_990., 50_000_010.), 1., true).unwrap();
        assert!(
            levels
                .windows(2)
                .all(|pair| pair[0].elevation < pair[1].elevation)
        );
        for level in levels {
            let expected = if level.ordinal.rem_euclid(2) != 0 {
                ContourLevelKind::FormLine
            } else if (level.ordinal / 2).rem_euclid(5) == 0 {
                ContourLevelKind::Index
            } else {
                ContourLevelKind::Regular
            };
            assert_eq!(level.kind, expected);
        }
    }

    #[test]
    fn persistence_protection_selects_only_the_matching_nested_ring() {
        let mut postprocessor = ring_pruner(false, 0., 0.);
        postprocessor.protected_features =
            vec![super::super::contour_field::ProtectedPersistenceFeature {
                pair_id: 1,
                kind: super::super::contour_field::ExtremumKind::Maximum,
                extremum_index: 0,
                extremum: geo::coord! { x: 5., y: 5. },
                extremum_elevation: 10.,
                saddle_elevation: 2.,
                persistence: 8.,
            }];
        let rings = geo::MultiLineString::new(vec![
            square_ring_between(0., 10.),
            square_ring_between(2., 8.),
        ]);

        let protected = postprocessor.protected_line_indices(5., &rings);

        assert_eq!(protected, std::collections::HashSet::from([1]));
        assert!(postprocessor.protected_line_indices(1., &rings).is_empty());
    }

    #[test]
    fn persistence_protection_respects_extremum_polarity() {
        let mut postprocessor = ring_pruner(false, 0., 0.);
        postprocessor.protected_features =
            vec![super::super::contour_field::ProtectedPersistenceFeature {
                pair_id: 2,
                kind: super::super::contour_field::ExtremumKind::Minimum,
                extremum_index: 0,
                extremum: geo::coord! { x: 5., y: 5. },
                extremum_elevation: 0.,
                saddle_elevation: 8.,
                persistence: 8.,
            }];
        let mut negative = square_ring();
        negative.0.reverse();
        let rings = geo::MultiLineString::new(vec![square_ring(), negative]);

        assert_eq!(
            postprocessor.protected_line_indices(5., &rings),
            std::collections::HashSet::from([1])
        );
    }

    #[test]
    fn naive_diagnostics_do_not_change_contour_geometry() {
        let grid = DfmGrid::new(16, 16, 0.5, geo::coord! { x: 0.25, y: 7.75 }).unwrap();
        let mut source = Dfm::<Elevation>::new(grid);
        for y in 0..source.height() {
            for x in 0..source.width() {
                let point = source.index2coord(y, x);
                source[(y, x)] = (10. + 0.35 * point.x + 0.1 * point.y) as f32;
            }
        }
        let z_range = source
            .field
            .iter()
            .fold((f32::INFINITY, f32::NEG_INFINITY), |(min, max), &value| {
                (min.min(value), max.max(value))
            });
        let cut = geo::Rect::new(source.index2coord(14, 1), source.index2coord(1, 14)).to_polygon();
        let mut params = MapParameters::default();
        params.contour.algorithm = ContourAlgo::NaiveIterations;
        params.contour.interval = 1.;
        params.contour.algo_steps = 2;

        let without_diagnostics =
            compute_naive_contours(&source, z_range, &cut, &params, false).unwrap();
        let with_diagnostics =
            compute_naive_contours(&source, z_range, &cut, &params, true).unwrap();

        let snapshot = |objects: Vec<MapObject>| {
            objects
                .into_iter()
                .map(|object| {
                    let MapObject::Line {
                        object,
                        symbol,
                        tags,
                    } = object
                    else {
                        panic!("contour pipeline emitted a non-line object");
                    };
                    (
                        symbol,
                        tags["Elevation"].clone(),
                        object
                            .0
                            .into_iter()
                            .map(|coordinate| (coordinate.x.to_bits(), coordinate.y.to_bits()))
                            .collect::<Vec<_>>(),
                    )
                })
                .collect::<Vec<_>>()
        };
        assert_eq!(
            snapshot(without_diagnostics.0),
            snapshot(with_diagnostics.0)
        );
        assert_eq!((without_diagnostics.1, without_diagnostics.2), (0., 0.));
    }

    #[test]
    fn shared_pipeline_honors_each_formline_importance_mode() {
        let grid = DfmGrid::new(20, 12, 0.5, geo::coord! { x: 0.25, y: 5.75 }).unwrap();
        let mut source = Dfm::<Elevation>::new(grid);
        for y in 0..source.height() {
            for x in 0..source.width() {
                let point = source.index2coord(y, x);
                source[(y, x)] = (10. + point.x * 0.8 + point.y * 0.05) as f32;
            }
        }
        let z_range = source
            .field
            .iter()
            .fold((f32::INFINITY, f32::NEG_INFINITY), |(min, max), &value| {
                (min.min(value), max.max(value))
            });
        let cut = geo::Rect::new(source.index2coord(10, 1), source.index2coord(1, 18)).to_polygon();
        let mut params = MapParameters::default();
        params.contour.algorithm = ContourAlgo::Raw;
        params.contour.interval = 2.;
        params.contour.form_lines = true;
        params.contour.form_line_geometry.minimum_open_length_m = 0.1;
        params.contour.form_line_geometry.minimum_closed_length_m = 0.1;
        params.contour.form_line_geometry.closed_seed_length_m = 0.;
        params.contour.form_line_prune_threshold = f32::MAX;
        params.contour.form_line_error_threshold = f32::MAX;
        let formline_count = |objects: &[MapObject]| {
            objects
                .iter()
                .filter(|object| {
                    matches!(
                        object,
                        MapObject::Line {
                            symbol: LineSymbol::FormLine,
                            ..
                        }
                    )
                })
                .count()
        };

        params.contour.form_line_prune_algorithm = FormlinePruneAlgo::None;
        let all = extract_contours(&source, z_range, &cut, &params, false).unwrap();
        params.contour.form_line_prune_algorithm = FormlinePruneAlgo::TerrainChange;
        let terrain = extract_contours(&source, z_range, &cut, &params, false).unwrap();
        params.contour.form_line_prune_algorithm = FormlinePruneAlgo::InterpolationError;
        let interpolation = extract_contours(&source, z_range, &cut, &params, false).unwrap();

        assert!(formline_count(&all.0) > 0);
        assert_eq!(formline_count(&terrain.0), 0);
        assert_eq!(formline_count(&interpolation.0), 0);
    }

    #[test]
    fn adjusted_tile_contours_use_the_same_source_boundary_crossing() {
        let make_tile =
            |top_left_x: f64, adjustment: f32, clip: geo::Polygon, kind: ContourLevelKind| {
                let grid =
                    DfmGrid::new(16, 16, 0.5, geo::coord! { x: top_left_x, y: 5.75 }).unwrap();
                let mut source = Dfm::<Elevation>::new(grid);
                for y in 0..source.height() {
                    for x in 0..source.width() {
                        source[(y, x)] = source.index2coord(y, x).y as f32;
                    }
                }
                let mut adjusted = source.clone();
                adjusted
                    .field
                    .iter_mut()
                    .for_each(|value| *value += adjustment);
                let levels = [ContourLevelSpec {
                    ordinal: 6,
                    elevation: 3.,
                    kind,
                }];
                let domain = extraction_polygon(&source);
                let contours =
                    extract_contour_set(&adjusted, &levels, &domain, ContourSimplification::None);
                emit_contour_objects(
                    &source, contours, &levels, &clip, None, 1., true, false, true,
                )
                .unwrap()
            };
        let left_clip = geo::Rect::new(
            geo::coord! { x: 0.5, y: -1. },
            geo::coord! { x: 6., y: 5.5 },
        )
        .to_polygon();
        let right_clip = geo::Rect::new(
            geo::coord! { x: 6., y: -1. },
            geo::coord! { x: 12.5, y: 5.5 },
        )
        .to_polygon();

        for kind in [ContourLevelKind::Regular, ContourLevelKind::FormLine] {
            let left = make_tile(0.25, 0.2, left_clip.clone(), kind);
            let right = make_tile(5.25, -0.2, right_clip.clone(), kind);
            let seam_endpoint = |objects: &[MapObject]| {
                objects.iter().find_map(|object| {
                    let MapObject::Line { object, .. } = object else {
                        return None;
                    };
                    object
                        .0
                        .iter()
                        .copied()
                        .find(|coordinate| (coordinate.x - 6.).abs() < 1e-9)
                })
            };

            assert_eq!(seam_endpoint(&left), Some(geo::coord! { x: 6., y: 3. }));
            assert_eq!(seam_endpoint(&left), seam_endpoint(&right));
        }
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
    fn protected_small_ring_does_not_require_an_importance_seed() {
        let mut pruner = ring_pruner(true, 100., 0.);
        pruner.importance = FormlineImportance::Areas {
            important: geo::MultiPolygon::new(Vec::new()),
            buffered: geo::MultiPolygon::new(Vec::new()),
        };
        let retained = pruner.prune(1, 2.5, &square_ring(), true);
        assert_eq!(retained, vec![square_ring()]);
        assert!(retained[0].is_closed());
    }

    #[test]
    fn unprotected_subminimum_ring_is_removed() {
        assert!(
            ring_pruner(false, 100., 0.)
                .prune(1, 2.5, &square_ring(), false)
                .is_empty()
        );
    }

    #[test]
    fn qualifying_small_ring_is_all_or_nothing() {
        let retained = ring_pruner(false, 20., 50.).prune(1, 2.5, &square_ring(), false);
        assert_eq!(retained, vec![square_ring()]);
    }

    #[test]
    fn long_ring_pruning_is_independent_of_stored_seam() {
        let pruner = ring_pruner(false, 20., 0.);
        let first = pruner.prune(1, 2.5, &square_ring(), false);
        let second = pruner.prune(1, 2.5, &rotate_closed_line(&square_ring(), 0.1), false);
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
        let produced = produce_scalar_contour_field(&source, &params).unwrap();
        let (objects, _, _) = compute_scalar_field_contours_from_produced(
            &source, z_range, &cut, &params, false, &produced,
        )
        .unwrap();
        let (scored_objects, error, energy) = compute_scalar_field_contours_from_produced(
            &source, z_range, &cut, &params, true, &produced,
        )
        .unwrap();
        let signature = |objects: &[MapObject]| {
            objects
                .iter()
                .map(|object| {
                    let MapObject::Line {
                        object,
                        symbol,
                        tags,
                    } = object
                    else {
                        panic!("contour pipeline emitted a non-line object");
                    };
                    (
                        *symbol,
                        tags["Elevation"].clone(),
                        object
                            .0
                            .iter()
                            .map(|coordinate| (coordinate.x.to_bits(), coordinate.y.to_bits()))
                            .collect::<Vec<_>>(),
                    )
                })
                .collect::<Vec<_>>()
        };
        assert_eq!(signature(&objects), signature(&scored_objects));
        assert!(error.is_finite() && energy.is_finite());
        assert!(!objects.is_empty());
        for object in objects {
            let MapObject::Line {
                object: line, tags, ..
            } = object
            else {
                panic!("contour pipeline emitted a non-line object");
            };
            let level = tags["Elevation"].parse::<f32>().unwrap();
            let tolerance =
                super::super::contour_field::adjustment_bound(params.contour.interval) + 1e-3;
            assert!(line.0.iter().all(|&coordinate| {
                source
                    .sample_bilinear(coordinate)
                    .is_none_or(|value| (value - level).abs() <= tolerance)
            }));
        }
    }

    #[test]
    fn contour_simplification_runs_after_dfm_extraction() {
        let grid = DfmGrid::new(48, 24, 0.5, geo::coord! { x: 0.25, y: 11.75 }).unwrap();
        let mut field = Dfm::<Elevation>::new(grid);
        for y in 0..field.height() {
            for x in 0..field.width() {
                let point = field.index2coord(y, x);
                field[(y, x)] =
                    (point.y - 5. + 0.08 * (point.x * std::f64::consts::PI).sin()) as f32;
            }
        }
        let levels = [ContourLevelSpec {
            ordinal: 0,
            elevation: 0.,
            kind: ContourLevelKind::Index,
        }];
        let domain = extraction_polygon(&field);
        let unsimplified =
            extract_contour_set(&field, &levels, &domain, ContourSimplification::None);
        let simplified = extract_contour_set(
            &field,
            &levels,
            &domain,
            ContourSimplification::DouglasPeucker(crate::SIMPLIFICATION_DIST),
        );
        let vertex_count = |set: &ContourSet| {
            set.0[0]
                .lines
                .iter()
                .map(|line| line.0.len())
                .sum::<usize>()
        };

        assert!(vertex_count(&simplified) < vertex_count(&unsimplified));
    }
}
