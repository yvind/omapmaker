use crate::{
    map::MapObject,
    parameters::MapParameters,
    raster::{Dfm, Elevation, dfm::TerrainSmoothing},
};

use super::super::{
    ContourPipelineContext, ContourPipelineOptions, ContourSimplification, extract_contour_set,
    extraction_polygon, finish_contours, plan_contour_levels,
};

// used for the naive iterative interpolation error correction contour algorithm
pub fn compute_naive_contours(
    true_dem: &Dfm<Elevation>,
    z_range: (f32, f32),
    cut_overlay: &geo::Polygon,
    params: &MapParameters,
    compute_energy: bool,
) -> crate::Result<(Vec<MapObject>, f32, f32)> {
    let levels = plan_contour_levels(z_range, params.contour.interval, params.contour.form_lines)?;
    let mut adjusted_dem: Dfm<Elevation> =
        true_dem.feature_preserving_smooth_as(TerrainSmoothing {
            max_normal_difference_degrees: 15.,
            radius_m: 3.5,
            iterations: 10,
            max_elevation_change_m: (0.1 * params.contour.interval.abs()).max(0.25),
        });
    let mut interpolated_dem = adjusted_dem.clone();
    let extraction_domain = extraction_polygon(true_dem);
    let simplification = ContourSimplification::DouglasPeucker(crate::SIMPLIFICATION_DIST);

    for iteration in 0..usize::from(params.contour.algo_steps) {
        let contours =
            extract_contour_set(&adjusted_dem, &levels, &extraction_domain, simplification);
        contours.interpolate(&mut interpolated_dem, &adjusted_dem)?;
        let remaining = usize::from(params.contour.algo_steps) - iteration;
        let filter_half_size =
            (remaining as f64 / f64::from(params.contour.algo_steps) * 30.) as usize;
        let filter_amplitude = remaining as f32 / f32::from(params.contour.algo_steps);

        adjusted_dem.adjust(
            true_dem,
            &interpolated_dem,
            filter_half_size,
            filter_amplitude,
        );
    }

    let contours = extract_contour_set(&adjusted_dem, &levels, &extraction_domain, simplification);
    finish_contours(
        contours,
        ContourPipelineContext {
            true_dem,
            contour_dem: &adjusted_dem,
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
