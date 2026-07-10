use crate::{
    geometry::PointCloud,
    map_gen::{
        self,
        common::ComputedDfms,
        egui_map::{AreaSymbol, MapObject},
    },
    parameters::{ContourAlgo, MapParameters, VegetationWeights},
    raster::{
        Dfm, Threshold,
        dfm::{
            Elevation, Ground, HeightAboveGround, HighVegetation, Intensity, LastReturn,
            LowVegetation, MediumVegetation, Ndvd, Returns, Slope, SurfaceObjects,
        },
    },
    statistics::LidarStats,
};
use geo::{Area, BooleanOps};
use std::cmp::Ordering;

pub struct TileRasters {
    pub dem: Dfm<Elevation>,
    pub slope: Dfm<Slope>,
    pub return_number: Dfm<Returns>,
    pub intensity: Dfm<Intensity>,
    pub last_return: Dfm<LastReturn>,
    pub ground_vegetation: Dfm<Ground>,
    pub low_vegetation: Dfm<LowVegetation>,
    pub medium_vegetation: Dfm<MediumVegetation>,
    pub high_vegetation: Dfm<HighVegetation>,
    pub surface_objects: Dfm<SurfaceObjects>,
    pub canopy_height: Dfm<HeightAboveGround>,
}

pub struct PreparedTile {
    pub rasters: TileRasters,
    pub hull: geo::Polygon,
    pub cut_overlay: geo::Polygon,
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
    pub fn new(dfms: ComputedDfms, hull: geo::Polygon, cut_overlay: geo::Polygon) -> Self {
        let ComputedDfms {
            dem,
            return_number,
            intensity,
            last_return,
            ground_vegetation,
            low_vegetation,
            medium_vegetation,
            high_vegetation,
            surface_objects,
            canopy_height,
            z_range,
        } = dfms;

        Self {
            rasters: TileRasters {
                slope: dem.slope(),
                dem,
                return_number,
                intensity,
                last_return,
                ground_vegetation,
                low_vegetation,
                medium_vegetation,
                high_vegetation,
                surface_objects,
                canopy_height,
            },
            hull,
            cut_overlay,
            z_range,
        }
    }

    pub fn from_cloud(
        ground_cloud: PointCloud,
        all_point_cloud: PointCloud,
        stats: &LidarStats,
        convex_hull: geo::Polygon,
        cut_bounds: geo::Rect,
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

        let dfms =
            map_gen::common::compute_dfms(ground_cloud, stats, &all_point_cloud, cut_bounds)?;
        Ok(Some(Self::new(dfms, convex_hull, mp.0.swap_remove(0))))
    }
}

impl TileRasters {
    pub fn compute_ndvd(&self, weights: VegetationWeights) -> Dfm<Ndvd> {
        map_gen::common::compute_ndvd(
            &self.ground_vegetation,
            &self.low_vegetation,
            &self.medium_vegetation,
            &self.high_vegetation,
            weights,
        )
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
        let ndvd = tile.rasters.compute_ndvd(params.vegetation.weights);
        for (threshold, symbol) in [
            (params.vegetation.green.0, AreaSymbol::LightGreen),
            (params.vegetation.green.1, AreaSymbol::MediumGreen),
            (params.vegetation.green.2, AreaSymbol::DarkGreen),
        ] {
            objects.extend(map_gen::common::compute_vegetation(
                &ndvd,
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
            &params.geometry.cliffs.buffer_rules,
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
