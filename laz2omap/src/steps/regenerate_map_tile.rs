use omap::{AreaSymbol, Omap};

use crate::{
    comms::messages::*,
    params::MapParams,
    raster::{Dfm, Threshold},
    DrawableOmap,
};

use std::sync::{mpsc::Sender, Arc, Mutex};

pub fn regenerate_map_tile(
    sender: Sender<FrontendTask>,
    dem: &Vec<Dfm>,
    g_dem: &Vec<Dfm>,
    drm: &Vec<Dfm>,
    cut_bounds: &Vec<geo::Polygon>,
    hull: &geo::Polygon,
    ref_point: geo::Coord,
    z_range: (f64, f64),
    params: MapParams,
) {
    let omap = Arc::new(Mutex::new(Omap::new(
        ref_point,
        params.output_epsg,
        params.scale,
    )));

    for i in 0..dem.len() {
        crate::steps::compute_vegetation(
            &drm[i],
            Threshold::Upper(params.yellow),
            hull.exterior(),
            &cut_bounds[i],
            &params,
            AreaSymbol::RoughOpenLand,
            &omap,
        );

        crate::steps::compute_vegetation(
            &drm[i],
            Threshold::Lower(params.green.0),
            hull.exterior(),
            &cut_bounds[i],
            &params,
            AreaSymbol::LightGreen,
            &omap,
        );

        crate::steps::compute_vegetation(
            &drm[i],
            Threshold::Lower(params.green.1),
            hull.exterior(),
            &cut_bounds[i],
            &params,
            AreaSymbol::MediumGreen,
            &omap,
        );

        crate::steps::compute_vegetation(
            &drm[i],
            Threshold::Lower(params.green.2),
            hull.exterior(),
            &cut_bounds[i],
            &params,
            AreaSymbol::DarkGreen,
            &omap,
        );

        if params.basemap_contour && params.basemap_interval >= 0.1 {
            crate::steps::compute_basemap(&dem[i], z_range, &cut_bounds[i], &params, &omap);
        }

        crate::steps::compute_cliffs(&g_dem[i], hull.exterior(), &cut_bounds[i], &params, &omap);
    }

    let omap = Arc::<Mutex<Omap>>::into_inner(omap)
        .unwrap()
        .into_inner()
        .unwrap();

    let map = DrawableOmap::from_omap(omap, hull.exterior().clone());

    sender
        .send(FrontendTask::UpdateVariable(Variable::MapTile(Box::new(
            map,
        ))))
        .unwrap();
    sender
        .send(FrontendTask::TaskComplete(TaskDone::RegenerateMap))
        .unwrap();
}
