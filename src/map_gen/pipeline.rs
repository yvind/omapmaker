use crate::{
    geometry::PointCloud,
    map_gen,
    map_gen::egui_map::{AreaSymbol, MapObject},
    parameters::{ContourAlgo, MapParameters},
    raster::{Dfm, Threshold},
    statistics::LidarStats,
};
use geo::{Area, BooleanOps, Polygon, Rect};
use std::cmp::Ordering;

pub struct TileRasters {
    pub dem: Dfm,
    pub slope: Dfm,
    pub return_number: Dfm,
    pub intensity: Dfm,
}

pub struct PreparedTile {
    pub rasters: TileRasters,
    pub hull: Polygon,
    pub cut_overlay: Polygon,
    pub z_range: (f64, f64),
}

pub struct PipelineOutput {
    pub objects: Vec<MapObject>,
    pub contour_error: f64,
    pub contour_energy: f64,
}

#[derive(Clone, Copy, Default)]
pub struct PipelineSteps {
    pub basemap: bool,
    pub contours: bool,
    pub openness: bool,
    pub vegetation: bool,
    pub cliffs: bool,
    pub intensity: bool,
}

impl PreparedTile {
    pub fn new(
        dem: Dfm,
        return_number: Dfm,
        intensity: Dfm,
        hull: Polygon,
        cut_overlay: Polygon,
        z_range: (f64, f64),
    ) -> Self {
        Self {
            rasters: TileRasters {
                slope: dem.slope(3),
                dem,
                return_number,
                intensity,
            },
            hull,
            cut_overlay,
            z_range,
        }
    }

    pub fn from_cloud(
        ground_cloud: PointCloud,
        stats: &LidarStats,
        convex_hull: Polygon,
        cut_bounds: Rect,
    ) -> crate::Result<Option<Self>> {
        let mut mp = cut_bounds.to_polygon().intersection(&convex_hull);
        if mp.0.is_empty() {
            return Ok(None);
        }

        mp.0.sort_by(|a, b| {
            a.signed_area()
                .partial_cmp(&b.signed_area())
                .unwrap_or(Ordering::Equal)
        });

        let (dem, return_number, intensity, z_range) =
            map_gen::common::compute_dfms(ground_cloud, stats)?;
        Ok(Some(Self::new(
            dem,
            return_number,
            intensity,
            convex_hull,
            mp.0.swap_remove(0),
            z_range,
        )))
    }
}

pub fn compute_tile(
    tile: &PreparedTile,
    params: &MapParameters,
    steps: PipelineSteps,
    compute_contour_score: bool,
) -> crate::Result<PipelineOutput> {
    let mut objects = Vec::new();
    let mut contour_error = 0.;
    let mut contour_energy = 0.;

    if steps.basemap && params.contour.basemap_contour && params.contour.basemap_interval >= 0.1 {
        objects.extend(map_gen::common::compute_basemap(
            &tile.rasters.dem,
            tile.z_range,
            &tile.cut_overlay,
            params.contour.basemap_interval,
        ));
    }

    if steps.contours {
        let (contours, error, energy) = match params.contour.algorithm {
            ContourAlgo::NaiveIterations => map_gen::common::compute_naive_contours(
                &tile.rasters.dem,
                tile.z_range,
                &tile.cut_overlay,
                if compute_contour_score {
                    (0.1, 0.0)
                } else {
                    (0.9, 1.1)
                },
                params,
            )?,
            ContourAlgo::NormalFieldSmoothing | ContourAlgo::Raw => {
                map_gen::common::extract_contours(
                    &tile.rasters.dem,
                    tile.z_range,
                    &tile.cut_overlay,
                    params,
                    compute_contour_score,
                )?
            }
        };
        objects.extend(contours);
        contour_error = error;
        contour_energy = energy;
    }

    if steps.openness {
        objects.extend(map_gen::common::compute_vegetation(
            &tile.rasters.return_number,
            Threshold::Upper(params.vegetation.yellow),
            &tile.hull,
            &tile.cut_overlay,
            AreaSymbol::RoughOpenLand,
            params,
            &params.geometry.openness.buffer_rules,
        ));
    }

    if steps.vegetation {
        for (threshold, symbol) in [
            (params.vegetation.green.0, AreaSymbol::LightGreen),
            (params.vegetation.green.1, AreaSymbol::MediumGreen),
            (params.vegetation.green.2, AreaSymbol::DarkGreen),
        ] {
            objects.extend(map_gen::common::compute_vegetation(
                &tile.rasters.return_number,
                Threshold::Lower(threshold),
                &tile.hull,
                &tile.cut_overlay,
                symbol,
                params,
                &params.geometry.vegetation.buffer_rules,
            ));
        }
    }

    if steps.cliffs {
        objects.extend(map_gen::common::compute_cliffs(
            &tile.rasters.slope,
            &tile.hull,
            &tile.cut_overlay,
            params,
        ));
    }

    if steps.intensity {
        objects.extend(map_gen::common::compute_intensity(
            &tile.rasters.intensity,
            &tile.hull,
            &tile.cut_overlay,
            params,
            &params.geometry.intensity.buffer_rules,
        ));
    }

    Ok(PipelineOutput {
        objects,
        contour_error,
        contour_energy,
    })
}
