use crate::{
    comms::{OmapComms, messages::*},
    drawable::DrawableOmap,
    map_gen::{
        egui_map::{AreaSymbol, LineSymbol, PointSymbol, TempMap},
        pipeline::{self, PipelineSteps, PreparedTile},
    },
    parameters::{CliffAlgorithm, MapParameters},
};

use rayon::{ThreadPool, prelude::*};

#[allow(clippy::too_many_arguments)]
pub fn regenerate_map_tile(
    sender: &OmapComms<FrontendTask, BackendTask>,
    job_id: JobId,
    thread_pool: &ThreadPool,
    tiles: &[PreparedTile],
    hull: &geo::Polygon,
    ref_point: geo::Coord,
    params: &MapParameters,
    old_params: &Option<MapParameters>,
    scope: RegenerationScope,
    preview_section_reached: Option<MapPreviewSection>,
    cancellation: &CancellationToken,
) {
    let mut omap = TempMap::new(ref_point, params.scale, params.output.crs.clone());

    let steps = changed_steps(params, old_params.as_ref(), scope, preview_section_reached);

    if steps.intensity {
        // make sure the symbols used in the prev generation are cleared
        if let Some(old_params) = &old_params {
            for filter in old_params.intensity.filters.iter() {
                omap.reserve_capacity(filter.symbol, 0);
            }
        }
    }
    if !params.contour.basemap_contour {
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
        Ok(o) => o,
        Err(e) => {
            if cancellation.is_cancelled() {
                return;
            }
            let _ = sender.send(FrontendTask::Error(e.to_string(), true));
            return;
        }
    };

    if cancellation.is_cancelled() {
        return;
    }

    for output in outputs {
        tot_energy += f64::from(output.contour_energy);
        tot_error += f64::from(output.contour_error);
        for object in output.objects {
            omap.add_object(object);
        }
    }

    if steps.buildings
        && let Err(e) = omap.merge_areas(AreaSymbol::Building, 2. * crate::SIMPLIFICATION_DIST)
    {
        let _ = sender.send(FrontendTask::Error(e.to_string(), true));
        return;
    }

    let min_size_filter_symbols = params.min_size_filter_symbols(
        steps.openness,
        steps.vegetation,
        steps.buildings,
        steps.cliffs,
        steps.intensity,
        steps.water || steps.marsh,
    );
    if let Err(e) = omap.merge_and_filter_min_size(min_size_filter_symbols) {
        let _ = sender.send(FrontendTask::Error(e.to_string(), true));
        return;
    }

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
    }
    if steps.buildings {
        omap.reserve_capacity(AreaSymbol::Building, 0);
    }
    if steps.cliffs {
        omap.reserve_capacity(AreaSymbol::GiganticBoulder, 0);
        omap.reserve_capacity(LineSymbol::Cliff, 0);
        omap.reserve_capacity(LineSymbol::ImpassableCliff, 0);
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

    if steps.contours && steps.streams {
        omap.merge_lines_with_symbol_distance(
            5. * crate::SIMPLIFICATION_DIST,
            LineSymbol::SmallCrossableWatercourse,
            params.streams.endpoint_merge_distance_m(),
        );
    } else if steps.contours {
        omap.merge_lines(5. * crate::SIMPLIFICATION_DIST);
    } else if steps.streams {
        omap.merge_lines(params.streams.endpoint_merge_distance_m());
    }

    if steps.contours {
        omap.reserve_capacity(PointSymbol::DotKnoll, 1);
        omap.reserve_capacity(PointSymbol::ElongatedDotKnoll, 1);
        omap.reserve_capacity(PointSymbol::UDepression, 1);
        omap.make_dotknolls_and_depressions(
            params.contour.dot_knoll_area.0,
            params.contour.dot_knoll_area.1,
            1.5,
        );
    }

    let map = match DrawableOmap::from_temp_map(omap, hull.exterior().clone(), &params.geometry) {
        Ok(m) => m,
        Err(e) => {
            let _ = sender.send(FrontendTask::Error(e.to_string(), true));
            return;
        }
    };

    if cancellation.is_cancelled() {
        return;
    }

    if steps.contours {
        tot_energy /= tiles.len() as f64;
        tot_error /= tiles.len() as f64;

        let _ = sender.send(FrontendTask::UpdateVariable(Variable::ContourScore(
            job_id,
            (tot_error as f32, tot_energy as f32),
        )));
    }

    let _ = sender.send(FrontendTask::UpdateVariable(Variable::MapTile(
        job_id,
        Box::new(map),
    )));
    let _ = sender.send(FrontendTask::TaskComplete(TaskComplete::RegenerateMap(
        job_id,
    )));
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
        limit_to_reached_sections(&mut steps, preview_section_reached);
        return steps;
    };

    if new.scale != old.scale {
        force_scope(&mut steps, scope);
        limit_to_reached_sections(&mut steps, preview_section_reached);
        return steps;
    }

    steps.intensity = new.intensity.filters != old.intensity.filters
        || new.geometry.intensity != old.geometry.intensity;

    steps.openness = new.vegetation.yellow != old.vegetation.yellow
        || new.geometry.openness != old.geometry.openness;
    steps.vegetation = new.vegetation.green != old.vegetation.green
        || new.vegetation.weights != old.vegetation.weights
        || new.geometry.vegetation != old.geometry.vegetation;
    let buildings_changed =
        new.building != old.building || new.geometry.buildings != old.geometry.buildings;
    steps.buildings = buildings_changed;
    let polynomial_fit_changed = new.cliff.algorithm == CliffAlgorithm::PolynomialFit
        && (new.contour.contour_field.slope_fit_radius_m
            != old.contour.contour_field.slope_fit_radius_m
            || new.contour.contour_field.curvature_fit_radius_m
                != old.contour.contour_field.curvature_fit_radius_m);
    steps.cliffs = new.cliff.algorithm != old.cliff.algorithm
        || new.cliff.cliff != old.cliff.cliff
        || new.geometry.cliffs != old.geometry.cliffs
        || new.cliff.collapse != old.cliff.collapse
        || new.cliff.collapse_amount_small_cliff != old.cliff.collapse_amount_small_cliff
        || new.cliff.collapse_amount_large_cliff != old.cliff.collapse_amount_large_cliff
        || new.cliff.collapse_linearity != old.cliff.collapse_linearity
        || polynomial_fit_changed;
    steps.water = new.water != old.water || new.geometry.water != old.geometry.water;
    steps.streams = new.streams != old.streams || new.geometry.streams != old.geometry.streams;
    steps.marsh = new.marsh != old.marsh || new.geometry.marsh != old.geometry.marsh || steps.water;

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

    steps.basemap = new.contour.basemap_interval != old.contour.basemap_interval
        || new.contour.basemap_contour != old.contour.basemap_contour;

    steps.contours = new.contour.algorithm != old.contour.algorithm
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
    limit_to_reached_sections(&mut steps, preview_section_reached);
    steps
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
            steps.intensity = true;
            steps.water = true;
            steps.marsh = true;
        }
        RegenerationScope::Section(MapPreviewSection::Cliffs) => {
            steps.cliffs = true;
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

    fn first_generation_steps(section: MapPreviewSection) -> PipelineSteps {
        changed_steps(
            &MapParameters::default(),
            None,
            RegenerationScope::Section(section),
            Some(section),
        )
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
    fn changing_cliff_algorithm_regenerates_cliffs() {
        let old = MapParameters::default();
        assert_eq!(old.cliff.algorithm, CliffAlgorithm::PolynomialFit);
        let mut new = old.clone();
        new.cliff.algorithm = CliffAlgorithm::SobelSlope;

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
