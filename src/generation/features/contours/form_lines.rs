use geo::{BooleanOps, Buffer, Contains, Euclidean, Length, LineLocatePoint, Simplify};

use crate::{
    geometry::{ContourSet, MapLineString, MapMultiPolygon},
    map::LineSymbol,
    parameters::MapParameters,
    raster::{Dfm, Elevation, RasterMarker},
};

use super::{ContourLevelKind, ContourLevelSpec, field};

const FORMLINE_PRUNE_BUFFER_METERS: f64 = 2.;

pub(super) enum FormlineImportance {
    All,
    Areas {
        important: geo::MultiPolygon,
        buffered: geo::MultiPolygon,
    },
}

pub(super) struct FormlineGeometryRules {
    pub(super) scale: crate::parameters::Scale,
    pub(super) min_open_length_m: f64,
    pub(super) min_closed_length_m: f64,
    pub(super) reconnect_gap_m: f64,
    pub(super) closed_seed_length_m: f64,
    pub(super) closed_all_or_none_max_length_m: f64,
}

pub(super) struct FormlinePostprocessor {
    pub(super) importance: FormlineImportance,
    pub(super) rules: FormlineGeometryRules,
    pub(super) protected_features: Vec<field::ProtectedPersistenceFeature>,
}

#[derive(Clone, Copy, Debug)]
pub(super) struct FormlineRange {
    pub(super) source_line_id: u64,
    pub(super) elevation: f32,
    pub(super) start: f64,
    pub(super) end: f64,
    pub(super) important: bool,
}

impl FormlinePostprocessor {
    pub(super) fn minimum_open_length(&self) -> f64 {
        if self.rules.min_open_length_m > 0. {
            self.rules.min_open_length_m
        } else {
            LineSymbol::FormLine.min_length(self.rules.scale, false)
                * self.rules.scale.denominator()
                / 1_000_000.
        }
    }

    pub(super) fn all(params: &MapParameters) -> Self {
        Self {
            importance: FormlineImportance::All,
            rules: FormlineGeometryRules::from_params(params),
            protected_features: Vec::new(),
        }
    }

    pub(super) fn from_terrain_change(
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

    pub(super) fn from_contour_interpolation_error<T: RasterMarker>(
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

    pub(super) fn with_protected_features(
        mut self,
        features: &[field::ProtectedPersistenceFeature],
    ) -> Self {
        self.protected_features.extend_from_slice(features);
        self
    }

    pub(super) fn prune(
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

    pub(super) fn protected_line_indices(
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
            let expected_positive = feature.kind == field::ExtremumKind::Maximum;
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

pub(super) fn prune_formline(
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

pub(super) fn rotate_closed_line(source: &geo::LineString, seam_fraction: f64) -> geo::LineString {
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

pub(super) fn merge_formline_ranges(ranges: &mut Vec<FormlineRange>, max_gap_fraction: f64) {
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

pub(super) fn push_unique_coord(coords: &mut Vec<geo::Coord>, coord: geo::Coord) {
    let is_duplicate = coords.last().is_some_and(|previous| {
        (previous.x - coord.x).abs() <= f64::EPSILON && (previous.y - coord.y).abs() <= f64::EPSILON
    });
    if !is_duplicate {
        coords.push(coord);
    }
}
