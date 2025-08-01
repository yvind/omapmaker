#![allow(clippy::too_many_arguments)]

use omap::{
    symbols::{AreaSymbol, LineSymbol, PointSymbol},
    Omap,
};

use crate::{
    comms::messages::*,
    drawable::DrawableOmap,
    map_gen,
    parameters::{ContourAlgo, MapParameters},
    raster::{Dfm, Threshold},
};

use std::sync::{mpsc::Sender, Arc, Mutex};
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
) {
    let omap = Arc::new(Mutex::new(
        Omap::new(ref_point, params.scale, params.output_epsg, None)
            .expect("Could not generate new map tile"),
    ));

    let needs_update = needs_regeneration(params, old_params.as_ref());

    if needs_update.intensities {
        // make sure the symbols used in the prev generation are cleared
        if let Some(old_params) = &old_params {
            let mut map = omap.lock().unwrap();
            for filter in old_params.intensity_filters.iter() {
                map.reserve_capacity(filter.symbol, 0);
            }
        }
    }
    if !params.basemap_contour {
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
                        let (error, energy) = match &params.contour_algorithm {
                            ContourAlgo::AI => (0., 0.),
                            ContourAlgo::NaiveIterations => {
                                map_gen::common::compute_naive_contours(
                                    &dem[i],
                                    z_range,
                                    &cut_bounds[i],
                                    (0.1, 0.0),
                                    params,
                                    &omap,
                                )
                            }
                            ContourAlgo::NormalFieldSmoothing => map_gen::common::extract_contours(
                                &dem[i],
                                z_range,
                                &cut_bounds[i],
                                params,
                                &omap,
                                true,
                            ),
                            ContourAlgo::Raw => map_gen::common::extract_contours(
                                &dem[i],
                                z_range,
                                &cut_bounds[i],
                                params,
                                &omap,
                                true,
                            ),
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

                    if params.basemap_contour
                        && params.basemap_interval >= 0.1
                        && needs_update.basemap
                    {
                        map_gen::common::compute_basemap(
                            &dem[i],
                            z_range,
                            &cut_bounds[i],
                            params.basemap_interval,
                            &omap,
                        );
                    }

                    if needs_update.yellow {
                        map_gen::common::compute_vegetation(
                            &drm[i],
                            Threshold::Upper(params.yellow),
                            hull,
                            &cut_bounds[i],
                            AreaSymbol::RoughOpenLand,
                            params,
                            &omap,
                        );
                    }

                    if needs_update.l_green {
                        map_gen::common::compute_vegetation(
                            &drm[i],
                            Threshold::Lower(params.green.0),
                            hull,
                            &cut_bounds[i],
                            AreaSymbol::LightGreen,
                            params,
                            &omap,
                        );
                    }

                    if needs_update.m_green {
                        map_gen::common::compute_vegetation(
                            &drm[i],
                            Threshold::Lower(params.green.1),
                            hull,
                            &cut_bounds[i],
                            AreaSymbol::MediumGreen,
                            params,
                            &omap,
                        );
                    }

                    if needs_update.d_green {
                        map_gen::common::compute_vegetation(
                            &drm[i],
                            Threshold::Lower(params.green.2),
                            hull,
                            &cut_bounds[i],
                            AreaSymbol::DarkGreen,
                            params,
                            &omap,
                        );
                    }

                    if needs_update.cliff {
                        map_gen::common::compute_cliffs(
                            &g_dem[i],
                            hull,
                            &cut_bounds[i],
                            params,
                            &omap,
                        );
                    }

                    if needs_update.intensities {
                        map_gen::common::compute_intensity(
                            &dim[i],
                            hull,
                            &cut_bounds[i],
                            params,
                            &omap,
                        )
                    }
                });
        }
    });

    let mut omap = Arc::<Mutex<Omap>>::into_inner(omap)
        .unwrap()
        .into_inner()
        .unwrap();

    if old_params.is_none() {
        // remove empty hashmap entries
        // no need to do this if the tile is simply an update
        // as then the empty entries are used to mark removal of objects from the map
        omap.remove_empty_keys();
    }

    omap.merge_lines(5. * crate::SIMPLIFICATION_DIST);

    if needs_update.basemap {
        omap.reserve_capacity(LineSymbol::BasemapContour, 1);
        omap.reserve_capacity(LineSymbol::NegBasemapContour, 1);
        omap.mark_basemap_depressions();
    }

    if needs_update.contours {
        omap.reserve_capacity(PointSymbol::DotKnoll, 1);
        omap.reserve_capacity(PointSymbol::ElongatedDotKnoll, 1);
        omap.reserve_capacity(PointSymbol::UDepression, 1);
        omap.make_dotknolls_and_depressions(params.dot_knoll_area.0, params.dot_knoll_area.1, 1.5);
    }

    let bez_error = if params.bezier_bool {
        Some(params.bezier_error)
    } else {
        None
    };

    let map = DrawableOmap::from_omap(omap, hull.exterior().clone(), bez_error);

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

fn needs_regeneration(new: &MapParameters, old: Option<&MapParameters>) -> UpdateMap {
    let mut update_map = UpdateMap::default();
    if old.is_none() {
        return update_map;
    }
    let old = old.unwrap();

    if new.scale != old.scale
        || new.bezier_bool != old.bezier_bool
        || (new.bezier_bool && (new.bezier_error != old.bezier_error))
    {
        return update_map;
    }

    let mut buffer_update = true;
    if new.buffer_rules.len() == old.buffer_rules.len() {
        buffer_update = false;

        for (new, old) in new.buffer_rules.iter().zip(old.buffer_rules.iter()) {
            if new != old {
                buffer_update = true;
                break;
            }
        }
    }

    if new.intensity_filters.len() == old.intensity_filters.len() && !buffer_update {
        update_map.intensities = false;

        for (new, old) in new
            .intensity_filters
            .iter()
            .zip(old.intensity_filters.iter())
        {
            if new != old {
                update_map.intensities = true;
                break;
            }
        }
    }

    update_map.yellow = new.yellow != old.yellow || buffer_update;
    update_map.l_green = new.green.0 != old.green.0 || buffer_update;
    update_map.m_green = new.green.1 != old.green.1 || buffer_update;
    update_map.d_green = new.green.2 != old.green.2 || buffer_update;
    update_map.cliff = new.cliff != old.cliff || buffer_update;

    update_map.basemap =
        new.basemap_interval != old.basemap_interval || new.basemap_contour != old.basemap_contour;

    update_map.contours = new.contour_algorithm != old.contour_algorithm
        || new.contour_algo_lambda != old.contour_algo_lambda
        || new.contour_algo_steps != old.contour_algo_steps
        || new.form_lines != old.form_lines
        || (new.form_lines && (new.form_line_prune != old.form_line_prune))
        || new.contour_interval != old.contour_interval
        || new.dot_knoll_area.0 != old.dot_knoll_area.0
        || new.dot_knoll_area.1 != old.dot_knoll_area.1;

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
            basemap: true,
            contours: true,
            yellow: true,
            l_green: true,
            m_green: true,
            d_green: true,
            cliff: true,
            intensities: true,
        }
    }
}
