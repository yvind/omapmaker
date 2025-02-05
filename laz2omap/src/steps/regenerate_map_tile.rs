use omap::Omap;

use crate::{comms::messages::*, params::MapParams, raster::Dfm, DrawableOmap};

use std::sync::{mpsc::Sender, Arc, Mutex};

pub fn regenerate_map_tile(
    sender: Sender<FrontendTask>,
    dem: &Dfm,
    g_dem: &Dfm,
    hull: &geo::LineString,
    ref_point: geo::Coord,
    z_range: (f64, f64),
    params: MapParams,
) {
    let omap = Arc::new(Mutex::new(Omap::new(
        ref_point,
        params.output_epsg,
        params.scale,
    )));

    if params.basemap_contour && params.basemap_interval >= 0.1 {
        crate::steps::compute_basemap(dem, z_range, None, &params, &omap);
    }

    crate::steps::compute_cliffs(g_dem, hull, None, &params, &omap);

    let omap = Arc::<Mutex<Omap>>::into_inner(omap)
        .unwrap()
        .into_inner()
        .unwrap();

    let map = DrawableOmap::from_omap(omap, hull.clone());

    sender.send(FrontendTask::UpdateMap(Box::new(map))).unwrap();
    sender
        .send(FrontendTask::TaskComplete(TaskDone::RegenerateMap))
        .unwrap();
}
