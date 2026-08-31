use crate::{
    map::MapObject,
    parameters::{ContourAlgo, MapParameters},
    raster::{ContourTerrain, Dfm, Elevation},
};

use super::super::{
    ContourPipelineContext, ContourPipelineOptions, ContourSimplification, extract_contour_set,
    extraction_polygon, finish_contours, generalized_contour_terrain, plan_contour_levels,
};

/// Extract raw or normal-field-smoothed contours from the shared terrain.
pub fn extract_contours(
    true_dem: &Dfm<Elevation>,
    contour_dem: &Dfm<ContourTerrain>,
    z_range: (f32, f32),
    cut_overlay: &geo::Polygon,
    params: &MapParameters,
    compute_energy: bool,
) -> crate::Result<(Vec<MapObject>, f32, f32)> {
    let generalized;
    let dem = if params.contour.algorithm == ContourAlgo::Raw {
        contour_dem
    } else {
        generalized = generalized_contour_terrain(
            contour_dem,
            params.contour.algo_steps as usize,
            params.contour.interval,
        );
        &generalized
    };
    let levels = plan_contour_levels(z_range, params.contour.interval, params.contour.form_lines)?;
    let extraction_domain = extraction_polygon(true_dem);
    let contours = extract_contour_set(
        dem,
        &levels,
        &extraction_domain,
        ContourSimplification::DouglasPeucker(crate::SIMPLIFICATION_DIST),
    );
    finish_contours(
        contours,
        ContourPipelineContext {
            true_dem,
            contour_dem: dem,
            levels: &levels,
            extraction_domain: &extraction_domain,
            output_clip: cut_overlay,
            params,
        },
        ContourPipelineOptions {
            compute_energy,
            validate_vertical_tolerance: false,
            preserve_geometry: false,
            snap_boundary_to_source: true,
            protected_features: &[],
        },
    )
}
