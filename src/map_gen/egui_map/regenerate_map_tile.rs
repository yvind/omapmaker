#![allow(clippy::too_many_arguments)]

use crate::{
    comms::messages::*,
    drawable::DrawableOmap,
    map_gen,
    map_gen::egui_map::{AreaSymbol, LineSymbol, PointSymbol, TempMap},
    parameters::{ContourAlgo, MapParameters},
    raster::{Dfm, Threshold},
};

use std::sync::{Arc, Mutex, mpsc::Sender};
use std::thread;

pub fn regenerate_map_tile(
    sender: Sender<FrontendTask>,
    dem: &[Dfm],
    g_dem: &[Dfm],
    drm: &[Dfm],
    dim: &[Dfm],
    cut_bounds: &[geo::Polygon],
    hull: &geo::Polygon,
    ref_point: geo::Coord,
    z_range: (f64, f64),
    params: &MapParameters,
    old_params: &Option<MapParameters>,
    scope: RegenerationScope,
) {
    let omap = Arc::new(Mutex::new(TempMap::new(
        ref_point,
        params.scale,
        params.output.crs.clone(),
    )));

    let needs_update = needs_regeneration(params, old_params.as_ref(), scope);

    if needs_update.intensities {
        // make sure the symbols used in the prev generation are cleared
        if let Some(old_params) = &old_params {
            let mut map = omap.lock().unwrap();
            for filter in old_params.intensity.filters.iter() {
                map.reserve_capacity(filter.symbol, 0);
            }
        }
    }
    if !params.contour.basemap_contour {
        // make sure that the basemap gets removed if it is toggled off
        let mut ac_map = omap.lock().unwrap();
        ac_map.reserve_capacity(LineSymbol::NegBasemapContour, 0);
        ac_map.reserve_capacity(LineSymbol::BasemapContour, 0);
    }

    let tot_energy = Arc::new(Mutex::new(0.));
    let tot_error = Arc::new(Mutex::new(0.));

    thread::scope(|s| {
        for i in 0..dem.len() {
            let omap = omap.clone();
            let tot_energy = tot_energy.clone();
            let tot_error = tot_error.clone();

            let _ = thread::Builder::new()
                .stack_size(crate::STACK_SIZE * 1024 * 1024)
                .spawn_scoped(s, move || {
                    if needs_update.contours {
                        let (error, energy) = match &params.contour.algorithm {
                            ContourAlgo::AI => (0., 0.),
                            ContourAlgo::NaiveIterations => {
                                let (objects, error, energy) =
                                    map_gen::common::compute_naive_contours(
                                        &dem[i],
                                        z_range,
                                        &cut_bounds[i],
                                        (0.1, 0.0),
                                        params,
                                    );
                                add_objects(&omap, objects);
                                (error, energy)
                            }
                            ContourAlgo::NormalFieldSmoothing => {
                                let (objects, error, energy) = map_gen::common::extract_contours(
                                    &dem[i],
                                    z_range,
                                    &cut_bounds[i],
                                    params,
                                    true,
                                );
                                add_objects(&omap, objects);
                                (error, energy)
                            }
                            ContourAlgo::Raw => {
                                let (objects, error, energy) = map_gen::common::extract_contours(
                                    &dem[i],
                                    z_range,
                                    &cut_bounds[i],
                                    params,
                                    true,
                                );
                                add_objects(&omap, objects);
                                (error, energy)
                            }
                        };
                        {
                            let mut energy_lock =
                                tot_energy.lock().expect("Could not lock energy mutex");
                            *energy_lock += energy;
                        }
                        {
                            let mut error_lock =
                                tot_error.lock().expect("Could not lock error mutex");
                            *error_lock += error;
                        }
                    }

                    if params.contour.basemap_contour
                        && params.contour.basemap_interval >= 0.1
                        && needs_update.basemap
                    {
                        let objects = map_gen::common::compute_basemap(
                            &dem[i],
                            z_range,
                            &cut_bounds[i],
                            params.contour.basemap_interval,
                        );
                        add_objects(&omap, objects);
                    }

                    if needs_update.yellow {
                        let objects = map_gen::common::compute_vegetation(
                            &drm[i],
                            Threshold::Upper(params.vegetation.yellow),
                            hull,
                            &cut_bounds[i],
                            AreaSymbol::RoughOpenLand,
                            params,
                            &params.geometry.openness.buffer_rules,
                        );
                        add_objects(&omap, objects);
                    }

                    if needs_update.l_green {
                        let objects = map_gen::common::compute_vegetation(
                            &drm[i],
                            Threshold::Lower(params.vegetation.green.0),
                            hull,
                            &cut_bounds[i],
                            AreaSymbol::LightGreen,
                            params,
                            &params.geometry.vegetation.buffer_rules,
                        );
                        add_objects(&omap, objects);
                    }

                    if needs_update.m_green {
                        let objects = map_gen::common::compute_vegetation(
                            &drm[i],
                            Threshold::Lower(params.vegetation.green.1),
                            hull,
                            &cut_bounds[i],
                            AreaSymbol::MediumGreen,
                            params,
                            &params.geometry.vegetation.buffer_rules,
                        );
                        add_objects(&omap, objects);
                    }

                    if needs_update.d_green {
                        let objects = map_gen::common::compute_vegetation(
                            &drm[i],
                            Threshold::Lower(params.vegetation.green.2),
                            hull,
                            &cut_bounds[i],
                            AreaSymbol::DarkGreen,
                            params,
                            &params.geometry.vegetation.buffer_rules,
                        );
                        add_objects(&omap, objects);
                    }

                    if needs_update.cliff {
                        let objects = map_gen::common::compute_cliffs(
                            &g_dem[i],
                            hull,
                            &cut_bounds[i],
                            params,
                        );
                        add_objects(&omap, objects);
                    }

                    if needs_update.intensities {
                        let objects = map_gen::common::compute_intensity(
                            &dim[i],
                            hull,
                            &cut_bounds[i],
                            params,
                            &params.geometry.intensity.buffer_rules,
                        );
                        add_objects(&omap, objects);
                    }
                });
        }
    });

    let mut omap = Arc::<Mutex<TempMap>>::into_inner(omap)
        .unwrap()
        .into_inner()
        .unwrap();

    if old_params.is_none() {
        // remove empty hashmap entries
        // no need to do this if the tile is simply an update
        // as then the empty entries are used to mark removal of objects from the map
        omap.remove_empty_keys();
    }

    // omap.merge_lines(5. * crate::SIMPLIFICATION_DIST);

    if needs_update.basemap {
        omap.reserve_capacity(LineSymbol::BasemapContour, 1);
        omap.reserve_capacity(LineSymbol::NegBasemapContour, 1);
        omap.mark_basemap_depressions();
    }

    if needs_update.contours {
        omap.reserve_capacity(PointSymbol::DotKnoll, 1);
        omap.reserve_capacity(PointSymbol::ElongatedDotKnoll, 1);
        omap.reserve_capacity(PointSymbol::UDepression, 1);
        omap.make_dotknolls_and_depressions(
            params.contour.dot_knoll_area.0,
            params.contour.dot_knoll_area.1,
            1.5,
        );
    }

    let map = DrawableOmap::from_temp_map(omap, hull.exterior().clone(), &params.geometry);

    if needs_update.contours {
        let mut tot_energy = tot_energy
            .lock()
            .expect("Could not lock energy mutex after scoped threads");
        let mut tot_error = tot_error
            .lock()
            .expect("Could not lock error mutex after scoped threads");

        *tot_energy /= dem.len() as f64;
        *tot_error /= dem.len() as f64;

        sender
            .send(FrontendTask::UpdateVariable(Variable::ContourScore((
                *tot_error as f32,
                *tot_energy as f32,
            ))))
            .unwrap();
    }

    sender
        .send(FrontendTask::UpdateVariable(Variable::MapTile(Box::new(
            map,
        ))))
        .unwrap();
    sender
        .send(FrontendTask::TaskComplete(TaskDone::RegenerateMap))
        .unwrap();
}

fn add_objects(omap: &Arc<Mutex<TempMap>>, objects: Vec<crate::map_gen::egui_map::MapObject>) {
    let mut omap = omap.lock().unwrap();
    for object in objects {
        omap.add_object(object);
    }
}

fn needs_regeneration(
    new: &MapParameters,
    old: Option<&MapParameters>,
    scope: RegenerationScope,
) -> UpdateMap {
    let mut update_map = UpdateMap::default();
    if old.is_none() {
        update_map.force_scope(scope);
        return update_map;
    }
    let old = old.unwrap();

    if new.scale != old.scale {
        update_map.force_scope(scope);
        return update_map;
    }

    if new.intensity.filters.len() == old.intensity.filters.len()
        && new.geometry.intensity == old.geometry.intensity
    {
        update_map.intensities = false;

        for (new, old) in new
            .intensity
            .filters
            .iter()
            .zip(old.intensity.filters.iter())
        {
            if new != old {
                update_map.intensities = true;
                break;
            }
        }
    }

    update_map.yellow = new.vegetation.yellow != old.vegetation.yellow
        || new.geometry.openness != old.geometry.openness;
    update_map.l_green = new.vegetation.green.0 != old.vegetation.green.0
        || new.geometry.vegetation != old.geometry.vegetation;
    update_map.m_green = new.vegetation.green.1 != old.vegetation.green.1
        || new.geometry.vegetation != old.geometry.vegetation;
    update_map.d_green = new.vegetation.green.2 != old.vegetation.green.2
        || new.geometry.vegetation != old.geometry.vegetation;
    update_map.cliff =
        new.vegetation.cliff != old.vegetation.cliff || new.geometry.cliffs != old.geometry.cliffs;

    update_map.basemap = new.contour.basemap_interval != old.contour.basemap_interval
        || new.contour.basemap_contour != old.contour.basemap_contour;

    update_map.contours = new.contour.algorithm != old.contour.algorithm
        || new.contour.algo_lambda != old.contour.algo_lambda
        || new.contour.algo_steps != old.contour.algo_steps
        || new.geometry.contours != old.geometry.contours
        || new.contour.form_lines != old.contour.form_lines
        || (new.contour.form_lines && (new.contour.form_line_prune != old.contour.form_line_prune))
        || new.contour.interval != old.contour.interval
        || new.contour.dot_knoll_area.0 != old.contour.dot_knoll_area.0
        || new.contour.dot_knoll_area.1 != old.contour.dot_knoll_area.1;

    update_map.force_scope(scope);
    update_map
}

struct UpdateMap {
    pub basemap: bool,
    pub contours: bool,
    pub yellow: bool,
    pub l_green: bool,
    pub m_green: bool,
    pub d_green: bool,
    pub cliff: bool,
    pub intensities: bool,
}

impl Default for UpdateMap {
    fn default() -> Self {
        Self {
            basemap: false,
            contours: true,
            yellow: false,
            l_green: false,
            m_green: false,
            d_green: false,
            cliff: false,
            intensities: false,
        }
    }
}

impl UpdateMap {
    fn force_scope(&mut self, scope: RegenerationScope) {
        match scope {
            RegenerationScope::Changed => (),
            RegenerationScope::Section(MapPreviewSection::Openness) => self.yellow = true,
            RegenerationScope::Section(MapPreviewSection::Vegetation) => {
                self.l_green = true;
                self.m_green = true;
                self.d_green = true;
            }
            RegenerationScope::Section(MapPreviewSection::Cliffs) => self.cliff = true,
            RegenerationScope::Section(MapPreviewSection::Intensity) => self.intensities = true,
        }
    }
}
