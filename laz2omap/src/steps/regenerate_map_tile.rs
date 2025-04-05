#![allow(clippy::too_many_arguments)]

use omap::{AreaSymbol, Omap, Symbol};

use crate::{
    comms::messages::*,
    parameters::MapParameters,
    raster::{Dfm, Threshold},
    DrawableOmap,
};

use std::sync::{mpsc::Sender, Arc, Mutex};

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
    params: MapParameters,
    old_params: Option<MapParameters>,
) {
    let omap = Arc::new(Mutex::new(Omap::new(
        ref_point,
        params.output_epsg,
        params.scale,
    )));

    let needs_update = needs_regeneration(&params, old_params.as_ref());

    let mut tot_energy = 0.;
    let mut tot_error = 0.;

    for i in 0..dem.len() {
        if needs_update.contours {
            let (error, energy) = match params.contour_algorithm {
                crate::parameters::ContourAlgo::AI => (0., 0.),
                crate::parameters::ContourAlgo::NaiveIterations => {
                    crate::steps::compute_naive_contours(
                        &dem[i],
                        z_range,
                        &cut_bounds[i],
                        (0.1, 0.0),
                        &params,
                        &omap,
                    )
                }
                crate::parameters::ContourAlgo::NormalFieldSmoothing => {
                    crate::steps::extract_contours(&dem[i], z_range, &cut_bounds[i], &params, &omap)
                }
                crate::parameters::ContourAlgo::Raw => {
                    crate::steps::extract_contours(&dem[i], z_range, &cut_bounds[i], &params, &omap)
                }
            };
            tot_error += error;
            tot_energy += energy;
        }

        if params.basemap_contour && params.basemap_interval >= 0.1 && needs_update.basemap {
            crate::steps::compute_basemap(
                &dem[i],
                z_range,
                &cut_bounds[i],
                params.basemap_interval,
                &omap,
            );
        } else if !params.basemap_contour {
            // make sure that the basemap gets removed if it is toggeled off
            let mut ac_map = omap.lock().unwrap();
            ac_map.reserve_capacity(Symbol::NegBasemapContour, 0);
            ac_map.reserve_capacity(Symbol::BasemapContour, 0);
        }

        if needs_update.yellow {
            crate::steps::compute_vegetation(
                &drm[i],
                Threshold::Upper(params.yellow),
                hull.exterior(),
                &cut_bounds[i],
                AreaSymbol::RoughOpenLand,
                &params,
                &omap,
            );
        }

        if needs_update.l_green {
            crate::steps::compute_vegetation(
                &drm[i],
                Threshold::Lower(params.green.0),
                hull.exterior(),
                &cut_bounds[i],
                AreaSymbol::LightGreen,
                &params,
                &omap,
            );
        }

        if needs_update.m_green {
            crate::steps::compute_vegetation(
                &drm[i],
                Threshold::Lower(params.green.1),
                hull.exterior(),
                &cut_bounds[i],
                AreaSymbol::MediumGreen,
                &params,
                &omap,
            );
        }

        if needs_update.d_green {
            crate::steps::compute_vegetation(
                &drm[i],
                Threshold::Lower(params.green.2),
                hull.exterior(),
                &cut_bounds[i],
                AreaSymbol::DarkGreen,
                &params,
                &omap,
            );
        }

        if needs_update.cliff {
            crate::steps::compute_cliffs(
                &g_dem[i],
                hull.exterior(),
                &cut_bounds[i],
                &params,
                &omap,
            );
        }

        if needs_update.intensities {
            // make sure the symbols used in the prev generation are cleared
            if let Some(old_params) = &old_params {
                let mut map = omap.lock().unwrap();
                for filter in old_params.intensity_filters.iter() {
                    map.reserve_capacity(filter.symbol, 0);
                }
            }

            crate::steps::compute_intensity(
                &dim[i],
                hull.exterior(),
                &cut_bounds[i],
                &params,
                &omap,
            )
        }
    }

    let mut omap = Arc::<Mutex<Omap>>::into_inner(omap)
        .unwrap()
        .into_inner()
        .unwrap();

    let bez_error = if params.bezier_bool {
        Some(params.bezier_error)
    } else {
        None
    };

    omap.merge_lines(5. * crate::SIMPLIFICATION_DIST);
    omap.mark_basemap_depressions();
    omap.make_dotknolls_and_depressions(10., 160., 1.5);

    let map = DrawableOmap::from_omap(omap, hull.exterior().clone(), bez_error);

    if needs_update.contours {
        tot_energy /= dem.len() as f64;
        tot_error /= dem.len() as f64;

        sender
            .send(FrontendTask::UpdateVariable(Variable::ContourScore((
                tot_error as f32,
                tot_energy as f32,
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

    update_map.yellow = new.yellow != old.yellow;
    update_map.l_green = new.green.0 != old.green.0;
    update_map.m_green = new.green.1 != old.green.1;
    update_map.d_green = new.green.2 != old.green.2;
    update_map.cliff = new.cliff != old.cliff;

    update_map.basemap =
        new.basemap_interval != old.basemap_interval || new.basemap_contour != old.basemap_contour;

    update_map.contours = new.contour_algorithm != old.contour_algorithm
        || new.contour_algo_lambda != old.contour_algo_lambda
        || new.contour_algo_steps != old.contour_algo_steps
        || new.formlines != old.formlines
        || (new.formlines && (new.formline_prune != old.formline_prune))
        || new.contour_interval != old.contour_interval;

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
