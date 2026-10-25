use crate::{
    cancellation::CancellationToken,
    generation::pipeline::{self, PipelineSteps, PreparedTile},
    generation::preview::{MapPreviewSection, RegenerationScope},
    map::{AreaSymbol, InternalMap, LineSymbol, PointSymbol},
    parameters::{CliffAlgorithm, MapParameters},
};

use rayon::{ThreadPool, prelude::*};

pub(crate) struct PreviewUpdate {
    pub(crate) document: InternalMap,
    pub(crate) contour_score: Option<(f32, f32)>,
}

#[allow(clippy::too_many_arguments)]
pub fn regenerate_map_tile(
    thread_pool: &ThreadPool,
    tiles: &[PreparedTile],
    ref_point: geo::Coord,
    params: &MapParameters,
    old_params: &Option<MapParameters>,
    scope: RegenerationScope,
    preview_section_reached: Option<MapPreviewSection>,
    cancellation: &CancellationToken,
) -> crate::Result<Option<PreviewUpdate>> {
    let mut omap = InternalMap::new(ref_point, params.scale, params.output.crs.clone());

    let steps = changed_steps(params, old_params.as_ref(), scope, preview_section_reached);

    if steps.intensity {
        // make sure the symbols used in the prev generation are cleared
        if let Some(old_params) = &old_params {
            for filter in old_params.intensity.filters.iter() {
                omap.reserve_capacity(filter.symbol, 0);
            }
        }
    }
    if !params.contour.enabled || !params.contour.basemap_contour {
        // make sure that the basemap gets removed if it is toggled off
        omap.reserve_capacity(LineSymbol::NegBasemapContour, 0);
        omap.reserve_capacity(LineSymbol::BasemapContour, 0);
    }

    let mut tot_energy = 0_f64;
    let mut tot_error = 0_f64;

    let outputs = thread_pool.install(|| {
        tiles
            .par_iter()
            .map(|tile| {
                cancellation.check()?;
                let output = pipeline::compute_tile_cancellable(
                    tile,
                    params,
                    steps,
                    steps.contours,
                    cancellation,
                )?;
                cancellation.check()?;
                Ok(output)
            })
            .collect::<anyhow::Result<Vec<_>>>()
    });

    let outputs = match outputs {
        Ok(outputs) => outputs,
        Err(_) if cancellation.is_cancelled() => return Ok(None),
        Err(error) => return Err(error),
    };

    if cancellation.is_cancelled() {
        return Ok(None);
    }

    let mut boulder_diagnostics = crate::generation::features::BoulderDiagnostics::default();
    for output in outputs {
        if let Some(diagnostics) = output.boulder_diagnostics {
            boulder_diagnostics.add(diagnostics);
        }
        tot_energy += f64::from(output.contour_energy);
        tot_error += f64::from(output.contour_error);
        for object in output.objects {
            omap.add_object(object);
        }
    }

    if steps.buildings {
        omap.merge_areas(AreaSymbol::Building, 2. * crate::SIMPLIFICATION_DIST)?;
    }

    let mut min_size_filter_symbols = params.min_size_filter_symbols(
        steps.openness,
        steps.vegetation,
        steps.buildings,
        steps.cliffs,
        steps.intensity,
        steps.water || steps.marsh,
    );
    if steps.boulder_fields
        && params.boulder_fields.enabled
        && params.geometry.boulder_fields.min_size_filter
    {
        min_size_filter_symbols.push(AreaSymbol::BoulderField);
    }
    omap.merge_and_filter_min_size(min_size_filter_symbols)?;

    omap.resolve_ground_cover_conflicts();

    if old_params.is_none() {
        // remove empty hashmap entries
        // no need to do this if the tile is simply an update
        // as then the empty entries are used to mark removal of objects from the map
        omap.remove_empty_keys();
    }

    if steps.basemap {
        omap.reserve_capacity(LineSymbol::BasemapContour, 1);
        omap.reserve_capacity(LineSymbol::NegBasemapContour, 1);
        omap.mark_basemap_depressions();
    }

    if steps.openness {
        omap.reserve_capacity(AreaSymbol::RoughOpenLand, 0);
    }
    if steps.vegetation {
        omap.reserve_capacity(AreaSymbol::LightGreen, 0);
        omap.reserve_capacity(AreaSymbol::MediumGreen, 0);
        omap.reserve_capacity(AreaSymbol::DarkGreen, 0);
        omap.reserve_capacity(AreaSymbol::Undergrowth, 0);
        omap.reserve_capacity(LineSymbol::ClearVegetationBoundary, 0);
    }
    if steps.buildings {
        omap.reserve_capacity(AreaSymbol::Building, 0);
    }
    if steps.cliffs {
        omap.reserve_capacity(AreaSymbol::GiganticBoulder, 0);
        omap.reserve_capacity(LineSymbol::Cliff, 0);
        omap.reserve_capacity(LineSymbol::ImpassableCliff, 0);
    }
    if steps.boulder_fields {
        omap.reserve_capacity(AreaSymbol::BoulderField, 0);
    }
    if steps.boulders {
        omap.reserve_capacity(PointSymbol::SmallBoulder, 0);
        omap.reserve_capacity(PointSymbol::LargeBoulder, 0);
    }
    if steps.water {
        omap.reserve_capacity(AreaSymbol::UncrossableWaterWithBankLine, 0);
    }
    if steps.marsh {
        omap.reserve_capacity(AreaSymbol::Marsh, 0);
    }
    if steps.streams {
        omap.reserve_capacity(LineSymbol::SmallCrossableWatercourse, 0);
    }
    if steps.intensity {
        for filter in params.intensity.filters.iter() {
            omap.reserve_capacity(filter.symbol, 0);
        }
    }

    // Contours from all preview tiles must be stitched, without reversing
    // them, before the closed-loop post-processing below.
    if steps.streams && (steps.contours || steps.cliffs) {
        omap.merge_lines_with_symbol_distance(
            5. * crate::SIMPLIFICATION_DIST,
            LineSymbol::SmallCrossableWatercourse,
            params.streams.endpoint_merge_distance_m,
        );
    } else if steps.contours || steps.cliffs {
        omap.merge_lines(5. * crate::SIMPLIFICATION_DIST);
    } else if steps.streams || steps.vegetation {
        omap.merge_lines(params.streams.endpoint_merge_distance_m);
    }
    if steps.vegetation {
        omap.filter_vegetation_boundaries_min_length(params.vegetation.boundaries.minimum_length_m);
    }
    if steps.contours && params.contour.form_lines {
        omap.filter_formlines_min_length();
    }

    if steps.cliffs && params.geometry.cliffs.min_size_filter {
        omap.filter_cliff_min_size(5. * crate::SIMPLIFICATION_DIST);
    }

    if steps.contours {
        omap.reserve_capacity(LineSymbol::Contour, 0);
        omap.reserve_capacity(LineSymbol::IndexContour, 0);
        omap.reserve_capacity(LineSymbol::FormLine, 0);
        omap.reserve_capacity(PointSymbol::DotKnoll, 1);
        omap.reserve_capacity(PointSymbol::ElongatedDotKnoll, 1);
        omap.reserve_capacity(PointSymbol::UDepression, 1);
        omap.reserve_capacity(PointSymbol::SlopeLineContour, 1);
        omap.reserve_capacity(PointSymbol::SlopeLineFormLine, 1);
        omap.make_dotknolls_and_depressions(
            params.contour.dot_knoll_area.0,
            params.contour.dot_knoll_area.1,
            1.5,
        );
        omap.add_depression_slope_lines();
    }

    if cancellation.is_cancelled() {
        return Ok(None);
    }

    if steps.boulders {
        omap.filter_boulder_points(params.boulder.minimum_separation_m);
    }

    let contour_score = if steps.contours {
        tot_energy /= tiles.len() as f64;
        tot_error /= tiles.len() as f64;
        Some((tot_error as f32, tot_energy as f32))
    } else {
        None
    };

    Ok(Some(PreviewUpdate {
        document: omap,
        contour_score,
    }))
}

fn changed_steps(
    new: &MapParameters,
    old: Option<&MapParameters>,
    scope: RegenerationScope,
    preview_section_reached: Option<MapPreviewSection>,
) -> PipelineSteps {
    let mut steps = PipelineSteps::default();

    let Some(old) = old else {
        force_scope(&mut steps, scope);
        include_ground_cover_dependencies(&mut steps, preview_section_reached);
        limit_to_reached_sections(&mut steps, preview_section_reached);
        return steps;
    };

    if new.scale != old.scale {
        // Scale affects symbol-size filtering and line/point geometry across
        // the preview. Recompute every section that has already been reached
        // instead of returning an empty update for `RegenerationScope::Changed`.
        steps = PipelineSteps {
            basemap: true,
            contours: true,
            openness: true,
            vegetation: true,
            buildings: true,
            cliffs: true,
            boulders: true,
            boulder_fields: true,
            intensity: true,
            water: true,
            streams: true,
            marsh: true,
        };
        limit_to_reached_sections(&mut steps, preview_section_reached);
        return steps;
    }

    steps.intensity = new.intensity.enabled != old.intensity.enabled
        || new.intensity.filters != old.intensity.filters
        || new.geometry.intensity != old.geometry.intensity;

    steps.openness = new.vegetation.openness_enabled != old.vegetation.openness_enabled
        || new.vegetation.yellow != old.vegetation.yellow
        || new.geometry.openness != old.geometry.openness;
    steps.vegetation = new.vegetation.enabled != old.vegetation.enabled
        || new.vegetation.green != old.vegetation.green
        || new.vegetation.weights != old.vegetation.weights
        || new.vegetation.undergrowth_enabled != old.vegetation.undergrowth_enabled
        || new.vegetation.undergrowth_threshold != old.vegetation.undergrowth_threshold
        || new.vegetation.boundaries != old.vegetation.boundaries
        || new.geometry.vegetation != old.geometry.vegetation;
    let buildings_changed =
        new.building != old.building || new.geometry.buildings != old.geometry.buildings;
    steps.buildings = buildings_changed;
    let polynomial_fit_changed = new.cliff.algorithm == CliffAlgorithm::PolynomialFit
        && (new.contour.contour_field.slope_fit_radius_m
            != old.contour.contour_field.slope_fit_radius_m
            || new.contour.contour_field.curvature_fit_radius_m
                != old.contour.contour_field.curvature_fit_radius_m);
    steps.cliffs = new.cliff.enabled != old.cliff.enabled
        || new.cliff.algorithm != old.cliff.algorithm
        || new.cliff.cliff != old.cliff.cliff
        || new.geometry.cliffs != old.geometry.cliffs
        || new.cliff.collapse != old.cliff.collapse
        || new.cliff.minimum_cliff_height_m != old.cliff.minimum_cliff_height_m
        || new.cliff.impassable_cliff_height_m != old.cliff.impassable_cliff_height_m
        || new.cliff.collapse_linearity != old.cliff.collapse_linearity
        || polynomial_fit_changed;
    steps.water = new.water != old.water || new.geometry.water != old.geometry.water;
    steps.streams = new.streams != old.streams || new.geometry.streams != old.geometry.streams;
    steps.marsh = new.marsh != old.marsh || new.geometry.marsh != old.geometry.marsh || steps.water;
    steps.boulders = new.boulder != old.boulder || buildings_changed || steps.cliffs;
    steps.boulder_fields = new.boulder_fields != old.boulder_fields
        || new.geometry.boulder_fields != old.geometry.boulder_fields
        || buildings_changed;

    // Building precedence changes the geometry of these layers. Regenerate
    // the affected counterparts together so incremental preview updates do
    // not leave stale overlaps behind.
    if buildings_changed {
        steps.openness = true;
        steps.vegetation = true;
        steps.cliffs = true;
        steps.intensity = true;
        steps.water = true;
        steps.marsh = true;
    }
    if steps.openness
        || steps.vegetation
        || steps.cliffs
        || steps.intensity
        || steps.water
        || steps.marsh
    {
        steps.buildings = true;
    }

    steps.basemap = new.contour.enabled != old.contour.enabled
        || new.contour.basemap_interval != old.contour.basemap_interval
        || new.contour.basemap_contour != old.contour.basemap_contour;

    steps.contours = new.contour.enabled != old.contour.enabled
        || new.contour.algorithm != old.contour.algorithm
        || new.contour.algo_lambda != old.contour.algo_lambda
        || new.contour.algo_steps != old.contour.algo_steps
        || new.contour.contour_field != old.contour.contour_field
        || new.geometry.contours != old.geometry.contours
        || new.contour.form_lines != old.contour.form_lines
        || new.contour.form_line_prune_algorithm != old.contour.form_line_prune_algorithm
        || new.contour.form_line_prune_threshold != old.contour.form_line_prune_threshold
        || new.contour.form_line_error_threshold != old.contour.form_line_error_threshold
        || new.contour.form_line_geometry != old.contour.form_line_geometry
        || new.contour.interval != old.contour.interval
        || new.contour.dot_knoll_area.0 != old.contour.dot_knoll_area.0
        || new.contour.dot_knoll_area.1 != old.contour.dot_knoll_area.1;

    force_scope(&mut steps, scope);
    // Point cleanup needs current building/cliff/knoll geometry even when
    // only a boulder threshold changed. Refresh these layers together.
    if preview_section_reached.is_some_and(|section| section >= MapPreviewSection::Boulders)
        && (steps.boulders || steps.cliffs || steps.contours)
    {
        steps.boulders = true;
        steps.buildings = true;
        steps.cliffs = true;
        steps.contours = true;
    }
    include_ground_cover_dependencies(&mut steps, preview_section_reached);
    limit_to_reached_sections(&mut steps, preview_section_reached);
    steps
}

/// 211 and 409 share exclusions and promoted green shades. Refresh these
/// together without running individual-boulder detection or contour fitting.
fn include_ground_cover_dependencies(
    steps: &mut PipelineSteps,
    reached: Option<MapPreviewSection>,
) {
    if reached.is_some_and(|section| section >= MapPreviewSection::BoulderFields)
        && (steps.vegetation || steps.boulder_fields)
    {
        steps.vegetation = true;
        steps.boulder_fields = true;
        steps.buildings = true;
    }
}

fn limit_to_reached_sections(
    steps: &mut PipelineSteps,
    preview_section_reached: Option<MapPreviewSection>,
) {
    let Some(reached) = preview_section_reached else {
        *steps = PipelineSteps::default();
        return;
    };

    if reached < MapPreviewSection::Contours {
        steps.basemap = false;
        steps.contours = false;
    }
    if reached < MapPreviewSection::Openness {
        steps.openness = false;
    }
    if reached < MapPreviewSection::Vegetation {
        steps.vegetation = false;
    }
    if reached < MapPreviewSection::Buildings {
        steps.buildings = false;
    }
    if reached < MapPreviewSection::Cliffs {
        steps.cliffs = false;
    }
    if reached < MapPreviewSection::Boulders {
        steps.boulders = false;
    }
    if reached < MapPreviewSection::BoulderFields {
        steps.boulder_fields = false;
    }
    if reached < MapPreviewSection::Water {
        steps.water = false;
    }
    if reached < MapPreviewSection::Marsh {
        steps.marsh = false;
    }
    if reached < MapPreviewSection::Streams {
        steps.streams = false;
    }
    if reached < MapPreviewSection::Intensity {
        steps.intensity = false;
    }
}

fn force_scope(steps: &mut PipelineSteps, scope: RegenerationScope) {
    match scope {
        RegenerationScope::Changed => (),
        RegenerationScope::Section(MapPreviewSection::Openness) => {
            steps.openness = true;
            steps.buildings = true;
        }
        RegenerationScope::Section(MapPreviewSection::Vegetation) => {
            steps.vegetation = true;
            steps.buildings = true;
        }
        RegenerationScope::Section(MapPreviewSection::Buildings) => {
            steps.buildings = true;
            steps.openness = true;
            steps.vegetation = true;
            steps.cliffs = true;
            steps.boulders = true;
            steps.boulder_fields = true;
            steps.intensity = true;
            steps.water = true;
            steps.marsh = true;
        }
        RegenerationScope::Section(MapPreviewSection::Cliffs) => {
            steps.cliffs = true;
            steps.buildings = true;
        }
        RegenerationScope::Section(MapPreviewSection::Boulders) => {
            steps.boulders = true;
            steps.cliffs = true;
            steps.buildings = true;
            steps.contours = true;
        }
        RegenerationScope::Section(MapPreviewSection::BoulderFields) => {
            steps.boulder_fields = true;
            steps.vegetation = true;
            steps.buildings = true;
        }
        RegenerationScope::Section(MapPreviewSection::Water) => {
            steps.water = true;
            steps.buildings = true;
        }
        RegenerationScope::Section(MapPreviewSection::Marsh) => {
            steps.marsh = true;
            steps.buildings = true;
        }
        RegenerationScope::Section(MapPreviewSection::Streams) => steps.streams = true,
        RegenerationScope::Section(MapPreviewSection::Intensity) => {
            steps.intensity = true;
            steps.buildings = true;
        }
        RegenerationScope::Section(MapPreviewSection::Contours) => {
            steps.contours = true;
            steps.basemap = true;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn boundary_controls_refresh_the_vegetation_preview() {
        let old = MapParameters::default();
        for mutate in [
            (|p: &mut MapParameters| p.vegetation.boundaries.enabled = false)
                as fn(&mut MapParameters),
            |p| p.vegetation.boundaries.minimum_height_change_m += 1.,
            |p| p.vegetation.boundaries.minimum_gradient += 0.5,
            |p| p.vegetation.boundaries.minimum_length_m += 5.,
            |p| p.vegetation.boundaries.smoothing_m += 0.5,
        ] {
            let mut new = old.clone();
            mutate(&mut new);
            let steps = changed_steps(
                &new,
                Some(&old),
                RegenerationScope::Changed,
                Some(MapPreviewSection::Vegetation),
            );
            assert!(steps.vegetation);
            assert!(!steps.contours && !steps.cliffs && !steps.boulders);
        }
    }

    #[test]
    fn ground_cover_changes_refresh_fields_and_vegetation_without_individual_boulders() {
        let old = MapParameters::default();
        for mutate in [
            (|p: &mut MapParameters| p.vegetation.undergrowth_enabled = false)
                as fn(&mut MapParameters),
            |p| p.vegetation.undergrowth_threshold += 0.1,
            |p| p.boulder_fields.enabled = false,
            |p| p.boulder_fields.threshold += 0.1,
            |p| p.geometry.boulder_fields.min_size_filter = false,
        ] {
            let mut new = old.clone();
            mutate(&mut new);
            let steps = changed_steps(
                &new,
                Some(&old),
                RegenerationScope::Changed,
                Some(MapPreviewSection::Intensity),
            );
            assert!(steps.vegetation && steps.boulder_fields && steps.buildings);
            assert!(!steps.boulders && !steps.cliffs && !steps.contours);
        }
        let mut new = old.clone();
        new.vegetation.undergrowth_enabled = false;
        let steps = changed_steps(
            &new,
            Some(&old),
            RegenerationScope::Changed,
            Some(MapPreviewSection::Vegetation),
        );
        assert!(steps.vegetation);
        assert!(!steps.boulders);
    }

    #[test]
    fn field_preview_waits_for_its_own_stage_and_does_not_run_point_detection() {
        let old = MapParameters::default();
        let mut new = old.clone();
        new.boulder_fields.threshold += 0.1;
        let early = changed_steps(
            &new,
            Some(&old),
            RegenerationScope::Changed,
            Some(MapPreviewSection::Boulders),
        );
        assert!(!early.boulder_fields);
        assert!(!early.boulders);
        let fields = first_generation_steps(MapPreviewSection::BoulderFields);
        assert!(fields.boulder_fields && fields.vegetation && fields.buildings);
        assert!(!fields.boulders && !fields.cliffs && !fields.contours);
        let mut points = old.clone();
        points.boulder.minimum_height_m += 0.5;
        let changed = changed_steps(
            &points,
            Some(&old),
            RegenerationScope::Changed,
            Some(MapPreviewSection::Intensity),
        );
        assert!(changed.boulders);
        assert!(!changed.boulder_fields && !changed.vegetation);
    }

    fn first_generation_steps(section: MapPreviewSection) -> PipelineSteps {
        changed_steps(
            &MapParameters::default(),
            None,
            RegenerationScope::Section(section),
            Some(section),
        )
    }

    #[test]
    fn toggling_any_generation_step_refreshes_its_preview_in_both_directions() {
        let enabled = MapParameters::default();
        let switches: [fn(&mut MapParameters) -> &mut bool; 12] = [
            |params| &mut params.contour.enabled,
            |params| &mut params.contour.basemap_contour,
            |params| &mut params.vegetation.openness_enabled,
            |params| &mut params.vegetation.enabled,
            |params| &mut params.building.enabled,
            |params| &mut params.cliff.enabled,
            |params| &mut params.boulder.enabled,
            |params| &mut params.boulder_fields.enabled,
            |params| &mut params.water.enabled,
            |params| &mut params.marsh.enabled,
            |params| &mut params.streams.enabled,
            |params| &mut params.intensity.enabled,
        ];
        for (index, switch) in switches.into_iter().enumerate() {
            let mut disabled = enabled.clone();
            *switch(&mut disabled) = false;
            for (new, old) in [(&disabled, &enabled), (&enabled, &disabled)] {
                let steps = changed_steps(
                    new,
                    Some(old),
                    RegenerationScope::Changed,
                    Some(MapPreviewSection::Intensity),
                );
                let refreshed = [
                    steps.contours && steps.basemap,
                    steps.basemap,
                    steps.openness,
                    steps.vegetation,
                    steps.buildings,
                    steps.cliffs,
                    steps.boulders,
                    steps.boulder_fields,
                    steps.water,
                    steps.marsh,
                    steps.streams,
                    steps.intensity,
                ];
                assert!(refreshed[index], "Step {index} was not refreshed");
            }
        }
    }

    #[test]
    fn boulder_preview_refreshes_conflicts_and_clears_disabled_symbols() {
        let steps = first_generation_steps(MapPreviewSection::Boulders);
        assert!(steps.boulders && steps.contours && steps.cliffs && steps.buildings);
        assert!(!steps.boulder_fields && !steps.vegetation);
        assert!(!steps.water);
        let old = MapParameters::default();
        let mut new = old.clone();
        new.boulder.enabled = false;
        let steps = changed_steps(
            &new,
            Some(&old),
            RegenerationScope::Changed,
            Some(MapPreviewSection::Intensity),
        );
        assert!(steps.boulders && steps.contours && steps.cliffs && steps.buildings);
        let early = changed_steps(
            &new,
            Some(&old),
            RegenerationScope::Changed,
            Some(MapPreviewSection::Cliffs),
        );
        assert!(!early.boulders);
    }

    #[test]
    fn preview_sections_do_not_compute_features_before_they_are_reached() {
        let contours = first_generation_steps(MapPreviewSection::Contours);
        assert!(contours.contours);
        assert!(!contours.openness);
        assert!(!contours.vegetation);
        assert!(!contours.buildings);
        assert!(!contours.cliffs);
        assert!(!contours.water);
        assert!(!contours.intensity);

        let openness = first_generation_steps(MapPreviewSection::Openness);
        assert!(openness.openness);
        assert!(!openness.buildings);
        assert!(!openness.cliffs);
        assert!(!openness.water);

        let vegetation = first_generation_steps(MapPreviewSection::Vegetation);
        assert!(vegetation.vegetation);
        assert!(!vegetation.buildings);
        assert!(!vegetation.cliffs);
        assert!(!vegetation.water);

        let buildings = first_generation_steps(MapPreviewSection::Buildings);
        assert!(buildings.buildings);
        assert!(buildings.openness);
        assert!(buildings.vegetation);
        assert!(!buildings.cliffs);
        assert!(!buildings.water);
        assert!(!buildings.streams);
        assert!(!buildings.intensity);

        let water = first_generation_steps(MapPreviewSection::Water);
        assert!(water.water);
        assert!(water.buildings);
        assert!(!water.marsh);
        assert!(!water.streams);

        let marsh = first_generation_steps(MapPreviewSection::Marsh);
        assert!(marsh.marsh);
        assert!(marsh.buildings);
        assert!(!marsh.water);
        assert!(!marsh.streams);

        let streams = first_generation_steps(MapPreviewSection::Streams);
        assert!(streams.streams);
        assert!(!streams.water);
        assert!(!streams.marsh);
    }

    #[test]
    fn changing_scale_regenerates_every_reached_section() {
        let old = MapParameters::default();
        let mut new = old.clone();
        new.scale = match old.scale {
            crate::parameters::Scale::S10_000 => crate::parameters::Scale::S15_000,
            crate::parameters::Scale::S15_000 => crate::parameters::Scale::S10_000,
        };
        new.contour.interval += 1.;

        let contour_steps = changed_steps(
            &new,
            Some(&old),
            RegenerationScope::Changed,
            Some(MapPreviewSection::Contours),
        );
        assert!(contour_steps.basemap);
        assert!(contour_steps.contours);
        assert!(!contour_steps.openness);

        let all_steps = changed_steps(
            &new,
            Some(&old),
            RegenerationScope::Changed,
            Some(MapPreviewSection::Intensity),
        );
        assert!(all_steps.basemap);
        assert!(all_steps.contours);
        assert!(all_steps.openness);
        assert!(all_steps.vegetation);
        assert!(all_steps.buildings);
        assert!(all_steps.cliffs);
        assert!(all_steps.intensity);
        assert!(all_steps.water);
        assert!(all_steps.streams);
        assert!(all_steps.marsh);
    }

    #[test]
    fn changing_cliff_algorithm_regenerates_cliffs() {
        let old = MapParameters::default();
        let mut new = old.clone();
        new.cliff.algorithm = match old.cliff.algorithm {
            CliffAlgorithm::SobelSlope => CliffAlgorithm::PolynomialFit,
            CliffAlgorithm::PolynomialFit => CliffAlgorithm::SobelSlope,
        };

        assert!(
            changed_steps(
                &new,
                Some(&old),
                RegenerationScope::Changed,
                Some(MapPreviewSection::Cliffs),
            )
            .cliffs
        );
    }

    #[test]
    fn changing_cliff_height_thresholds_regenerates_cliffs() {
        let old = MapParameters::default();
        for new in [
            {
                let mut new = old.clone();
                new.cliff.minimum_cliff_height_m += 0.5;
                new
            },
            {
                let mut new = old.clone();
                new.cliff.impassable_cliff_height_m += 0.5;
                new
            },
        ] {
            assert!(
                changed_steps(
                    &new,
                    Some(&old),
                    RegenerationScope::Changed,
                    Some(MapPreviewSection::Cliffs),
                )
                .cliffs
            );
        }
    }

    #[test]
    fn changing_cliff_rdp_parameters_regenerates_cliffs() {
        let old = MapParameters::default();
        for new in [
            {
                let mut new = old.clone();
                new.geometry.cliffs.rdp.enabled = !old.geometry.cliffs.rdp.enabled;
                new
            },
            {
                let mut new = old.clone();
                new.geometry.cliffs.rdp.tolerance_m += 0.25;
                new
            },
        ] {
            assert!(
                changed_steps(
                    &new,
                    Some(&old),
                    RegenerationScope::Changed,
                    Some(MapPreviewSection::Cliffs),
                )
                .cliffs
            );
        }
    }

    #[test]
    fn changing_cliff_hole_area_regenerates_cliffs() {
        let old = MapParameters::default();
        let mut new = old.clone();
        new.geometry.cliffs.maximum_hole_area_m2 += 1.;

        assert!(
            changed_steps(
                &new,
                Some(&old),
                RegenerationScope::Changed,
                Some(MapPreviewSection::Cliffs),
            )
            .cliffs
        );
    }

    #[test]
    fn changing_cliff_minimum_size_toggle_regenerates_cliffs() {
        let old = MapParameters::default();
        let mut new = old.clone();
        new.geometry.cliffs.min_size_filter = !old.geometry.cliffs.min_size_filter;

        assert!(
            changed_steps(
                &new,
                Some(&old),
                RegenerationScope::Changed,
                Some(MapPreviewSection::Cliffs),
            )
            .cliffs
        );
    }

    #[test]
    fn polynomial_fit_changes_do_not_regenerate_sobel_cliffs() {
        let mut old = MapParameters::default();
        old.cliff.algorithm = CliffAlgorithm::SobelSlope;
        let mut new = old.clone();
        new.contour.contour_field.curvature_fit_radius_m += 1.;

        assert!(
            !changed_steps(
                &new,
                Some(&old),
                RegenerationScope::Changed,
                Some(MapPreviewSection::Cliffs),
            )
            .cliffs
        );

        new.cliff.algorithm = CliffAlgorithm::PolynomialFit;
        assert!(
            changed_steps(
                &new,
                Some(&old),
                RegenerationScope::Changed,
                Some(MapPreviewSection::Cliffs),
            )
            .cliffs
        );
    }

    #[test]
    fn shared_contour_parameters_do_not_compute_cliffs_early() {
        let old = MapParameters::default();
        let mut new = old.clone();
        new.contour.contour_field.curvature_fit_radius_m += 1.;

        let steps = changed_steps(
            &new,
            Some(&old),
            RegenerationScope::Changed,
            Some(MapPreviewSection::Contours),
        );

        assert!(steps.contours);
        assert!(!steps.buildings);
        assert!(!steps.cliffs);
    }

    #[test]
    fn marsh_sensitivity_recomputes_only_marsh_and_its_building_exclusion() {
        let old = MapParameters::default();
        let mut new = old.clone();
        new.marsh.sensitivity = 0.75;
        let steps = changed_steps(
            &new,
            Some(&old),
            RegenerationScope::Changed,
            Some(MapPreviewSection::Intensity),
        );
        assert!(steps.marsh);
        assert!(steps.buildings);
        assert!(!steps.water);
        assert!(!steps.streams);
        assert!(!steps.contours);
    }

    #[test]
    fn water_and_stream_parameters_regenerate_independent_sections() {
        let old = MapParameters::default();

        let mut water_changed = old.clone();
        water_changed.water.threshold += 0.05;
        let water_steps = changed_steps(
            &water_changed,
            Some(&old),
            RegenerationScope::Changed,
            Some(MapPreviewSection::Intensity),
        );
        assert!(water_steps.water);
        assert!(water_steps.marsh);
        assert!(!water_steps.streams);

        let mut streams_changed = old.clone();
        streams_changed.streams.minimum_catchment_area_m2 += 100.;
        let stream_steps = changed_steps(
            &streams_changed,
            Some(&old),
            RegenerationScope::Changed,
            Some(MapPreviewSection::Intensity),
        );
        assert!(stream_steps.streams);
        assert!(!stream_steps.water);
        assert!(!stream_steps.marsh);

        let mut vectorization_changed = old.clone();
        vectorization_changed
            .streams
            .onnx_vectorization
            .confidence_threshold = 0.6;
        let vectorization_steps = changed_steps(
            &vectorization_changed,
            Some(&old),
            RegenerationScope::Changed,
            Some(MapPreviewSection::Intensity),
        );
        assert!(vectorization_steps.streams);
        assert!(!vectorization_steps.water);
        assert!(!vectorization_steps.marsh);
    }
}
