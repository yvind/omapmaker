#![allow(clippy::too_many_arguments)]

use crate::{
    comms::messages::*,
    drawable::DrawableOmap,
    map_gen,
    map_gen::egui_map::{AreaSymbol, LineSymbol, PointSymbol, TempMap},
    parameters::{ContourAlgo, MapParameters},
    raster::{Dfm, Threshold},
};

use rayon::{ThreadPool, prelude::*};
use std::sync::{Arc, Mutex, mpsc::Sender};

pub fn regenerate_map_tile(
    sender: Sender<FrontendTask>,
    thread_pool: &ThreadPool,
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
    if let Err(e) = try_regenerate_map_tile(
        sender.clone(),
        thread_pool,
        dem,
        g_dem,
        drm,
        dim,
        cut_bounds,
        hull,
        ref_point,
        z_range,
        params,
        old_params,
        scope,
    ) {
        let _ = sender.send(FrontendTask::Error(e.to_string(), true));
    }
}

fn try_regenerate_map_tile(
    sender: Sender<FrontendTask>,
    thread_pool: &ThreadPool,
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
) -> crate::Result<()> {
    let omap = Arc::new(Mutex::new(TempMap::new(
        ref_point,
        params.scale,
        params.output.crs.clone(),
    )));

    let needs_update = needs_regeneration(params, old_params.as_ref(), scope);

    if needs_update.intensities {
        // make sure the symbols used in the prev generation are cleared
        if let Some(old_params) = &old_params
            && let Ok(mut map) = omap.lock()
        {
            for filter in old_params.intensity.filters.iter() {
                map.reserve_capacity(filter.symbol, 0);
            }
        }
    }
    if !params.contour.basemap_contour {
        // make sure that the basemap gets removed if it is toggled off
        if let Ok(mut ac_map) = omap.lock() {
            ac_map.reserve_capacity(LineSymbol::NegBasemapContour, 0);
            ac_map.reserve_capacity(LineSymbol::BasemapContour, 0);
        }
    }

    let tot_energy = Arc::new(Mutex::new(0.));
    let tot_error = Arc::new(Mutex::new(0.));

    thread_pool.install(|| {
        (0..dem.len()).into_par_iter().for_each(|i| {
            let omap = omap.clone();
            let tot_energy = tot_energy.clone();
            let tot_error = tot_error.clone();
            let sender = sender.clone();

            if needs_update.contours {
                let (error, energy) = match &params.contour.algorithm {
                    ContourAlgo::NaiveIterations => {
                        let Ok((objects, error, energy)) = map_gen::common::compute_naive_contours(
                            &dem[i],
                            z_range,
                            &cut_bounds[i],
                            (0.1, 0.0),
                            params,
                        ) else {
                            let _ = sender.send(FrontendTask::Error(
                                "Failed to compute naive contours".to_string(),
                                true,
                            ));
                            return;
                        };
                        add_objects(&omap, objects);
                        (error, energy)
                    }
                    ContourAlgo::NormalFieldSmoothing => {
                        let Ok((objects, error, energy)) = map_gen::common::extract_contours(
                            &dem[i],
                            z_range,
                            &cut_bounds[i],
                            params,
                            true,
                        ) else {
                            let _ = sender.send(FrontendTask::Error(
                                "Failed to extract smoothed contours".to_string(),
                                true,
                            ));
                            return;
                        };
                        add_objects(&omap, objects);
                        (error, energy)
                    }
                    ContourAlgo::Raw => {
                        let Ok((objects, error, energy)) = map_gen::common::extract_contours(
                            &dem[i],
                            z_range,
                            &cut_bounds[i],
                            params,
                            true,
                        ) else {
                            let _ = sender.send(FrontendTask::Error(
                                "Failed to extract raw contours".to_string(),
                                true,
                            ));
                            return;
                        };
                        add_objects(&omap, objects);
                        (error, energy)
                    }
                };
                {
                    if let Ok(mut energy_lock) = tot_energy.lock() {
                        *energy_lock += energy;
                    }
                }
                {
                    if let Ok(mut error_lock) = tot_error.lock() {
                        *error_lock += error;
                    }
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
                if objects.is_empty() {
                    clear_objects(&omap, AreaSymbol::RoughOpenLand);
                } else {
                    add_objects(&omap, objects);
                }
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
                if objects.is_empty() {
                    clear_objects(&omap, AreaSymbol::LightGreen);
                } else {
                    add_objects(&omap, objects);
                }
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
                if objects.is_empty() {
                    clear_objects(&omap, AreaSymbol::MediumGreen);
                } else {
                    add_objects(&omap, objects);
                }
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
                if objects.is_empty() {
                    clear_objects(&omap, AreaSymbol::DarkGreen);
                } else {
                    add_objects(&omap, objects);
                }
            }

            if needs_update.cliff {
                let objects =
                    map_gen::common::compute_cliffs(&g_dem[i], hull, &cut_bounds[i], params);
                if objects.is_empty() {
                    clear_objects(&omap, AreaSymbol::GiganticBoulder);
                } else {
                    add_objects(&omap, objects);
                }
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
    });

    let mut omap = Arc::<Mutex<TempMap>>::into_inner(omap)
        .ok_or_else(|| {
            anyhow::anyhow!("Could not get inner preview map; a worker still holds a reference")
        })?
        .into_inner()
        .map_err(|_| anyhow::anyhow!("Preview map mutex was poisoned"))?;

    if old_params.is_none() {
        // remove empty hashmap entries
        // no need to do this if the tile is simply an update
        // as then the empty entries are used to mark removal of objects from the map
        omap.remove_empty_keys();
    }

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

    let map = DrawableOmap::from_temp_map(omap, hull.exterior().clone(), &params.geometry)?;

    if needs_update.contours {
        let mut tot_energy = tot_energy
            .lock()
            .map_err(|_| anyhow::anyhow!("Could not lock contour energy after regeneration"))?;
        let mut tot_error = tot_error
            .lock()
            .map_err(|_| anyhow::anyhow!("Could not lock contour error after regeneration"))?;

        *tot_energy /= dem.len() as f64;
        *tot_error /= dem.len() as f64;

        let _ = sender.send(FrontendTask::UpdateVariable(Variable::ContourScore((
            *tot_error as f32,
            *tot_energy as f32,
        ))));
    }

    let _ = sender.send(FrontendTask::UpdateVariable(Variable::MapTile(Box::new(
        map,
    ))));
    let _ = sender.send(FrontendTask::TaskComplete(TaskDone::RegenerateMap));
    Ok(())
}

fn add_objects(omap: &Arc<Mutex<TempMap>>, objects: Vec<crate::map_gen::egui_map::MapObject>) {
    if let Ok(mut omap) = omap.lock() {
        for object in objects {
            omap.add_object(object);
        }
    }
}

fn clear_objects(omap: &Arc<Mutex<TempMap>>, symbol: impl Into<crate::map_gen::egui_map::Symbol>) {
    if let Ok(mut omap) = omap.lock() {
        omap.reserve_capacity(symbol, 0);
    }
}

fn needs_regeneration(
    new: &MapParameters,
    old: Option<&MapParameters>,
    scope: RegenerationScope,
) -> UpdateMap {
    let mut update_map = UpdateMap::default();
    let Some(old) = old else {
        update_map.force_scope(scope);
        return update_map;
    };

    if new.scale != old.scale {
        update_map.force_scope(scope);
        return update_map;
    }

    update_map.intensities = new.intensity.filters != old.intensity.filters
        || new.geometry.intensity != old.geometry.intensity;

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
        || new.contour.form_lines
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
