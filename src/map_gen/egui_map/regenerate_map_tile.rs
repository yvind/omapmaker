use crate::{
    comms::{OmapComms, messages::*},
    drawable::DrawableOmap,
    map_gen::{
        egui_map::{AreaSymbol, LineSymbol, PointSymbol, TempMap},
        pipeline::{self, PipelineSteps, PreparedTile},
    },
    parameters::MapParameters,
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
) {
    let mut omap = TempMap::new(ref_point, params.scale, params.output.crs.clone());

    let steps = changed_steps(params, old_params.as_ref(), scope);

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

    let mut tot_energy = 0.;
    let mut tot_error = 0.;

    let outputs = thread_pool.install(|| {
        tiles
            .par_iter()
            .map(|tile| pipeline::compute_tile(tile, params, steps, steps.contours))
            .collect::<anyhow::Result<Vec<_>>>()
    });

    let outputs = match outputs {
        Ok(o) => o,
        Err(e) => {
            let _ = sender.send(FrontendTask::Error(e.to_string(), true));
            return;
        }
    };

    for output in outputs {
        tot_energy += output.contour_energy;
        tot_error += output.contour_error;
        for object in output.objects {
            omap.add_object(object);
        }
    }

    let min_size_filter_symbols = params.min_size_filter_symbols(
        steps.openness,
        steps.vegetation,
        steps.cliffs,
        steps.intensity,
        steps.water,
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
    if steps.cliffs {
        omap.reserve_capacity(AreaSymbol::GiganticBoulder, 0);
    }
    if steps.water {
        omap.reserve_capacity(AreaSymbol::UncrossableWaterWithBankLine, 0);
    }
    if steps.intensity {
        for filter in params.intensity.filters.iter() {
            omap.reserve_capacity(filter.symbol, 0);
        }
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
    let _ = sender.send(FrontendTask::TaskComplete(TaskDone::RegenerateMap(job_id)));
}

fn changed_steps(
    new: &MapParameters,
    old: Option<&MapParameters>,
    scope: RegenerationScope,
) -> PipelineSteps {
    let mut steps = PipelineSteps {
        contours: true,
        ..PipelineSteps::default()
    };
    let Some(old) = old else {
        steps.basemap = new.contour.basemap_contour;
        force_scope(&mut steps, scope);
        return steps;
    };

    if new.scale != old.scale {
        force_scope(&mut steps, scope);
        return steps;
    }

    steps.intensity = new.intensity.filters != old.intensity.filters
        || new.geometry.intensity != old.geometry.intensity;

    steps.openness = new.vegetation.yellow != old.vegetation.yellow
        || new.geometry.openness != old.geometry.openness;
    steps.vegetation = new.vegetation.green != old.vegetation.green
        || new.vegetation.weights != old.vegetation.weights
        || new.geometry.vegetation != old.geometry.vegetation;
    steps.cliffs = new.cliff.cliff != old.cliff.cliff || new.geometry.cliffs != old.geometry.cliffs;
    steps.water = new.water != old.water || new.geometry.water != old.geometry.water;

    steps.basemap = new.contour.basemap_interval != old.contour.basemap_interval
        || new.contour.basemap_contour != old.contour.basemap_contour;

    steps.contours = new.contour.algorithm != old.contour.algorithm
        || new.contour.algo_lambda != old.contour.algo_lambda
        || new.contour.algo_steps != old.contour.algo_steps
        || new.geometry.contours != old.geometry.contours
        || new.contour.form_lines != old.contour.form_lines
        || new.contour.form_line_prune_algorithm != old.contour.form_line_prune_algorithm
        || new.contour.form_line_prune_threshold != old.contour.form_line_prune_threshold
        || new.contour.form_line_error_threshold != old.contour.form_line_error_threshold
        || new.contour.interval != old.contour.interval
        || new.contour.dot_knoll_area.0 != old.contour.dot_knoll_area.0
        || new.contour.dot_knoll_area.1 != old.contour.dot_knoll_area.1;

    force_scope(&mut steps, scope);
    steps
}

fn force_scope(steps: &mut PipelineSteps, scope: RegenerationScope) {
    match scope {
        RegenerationScope::Changed => (),
        RegenerationScope::Section(MapPreviewSection::Openness) => steps.openness = true,
        RegenerationScope::Section(MapPreviewSection::Vegetation) => steps.vegetation = true,
        RegenerationScope::Section(MapPreviewSection::Cliffs) => steps.cliffs = true,
        RegenerationScope::Section(MapPreviewSection::Water) => steps.water = true,
        RegenerationScope::Section(MapPreviewSection::Intensity) => steps.intensity = true,
    }
}
