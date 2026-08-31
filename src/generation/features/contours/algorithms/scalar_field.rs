use std::sync::Arc;

use crate::{
    map::MapObject,
    parameters::MapParameters,
    raster::{Dfm, Elevation},
};

use super::super::{
    ContourPipelineContext, ContourPipelineOptions, ContourSimplification, ProducedContourField,
    extract_contour_set, extraction_polygon, field, finish_contours, plan_contour_levels,
};

#[cfg(test)]
pub(crate) fn produce_scalar_contour_field(
    true_dem: &Dfm<Elevation>,
    params: &MapParameters,
) -> crate::Result<ProducedContourField> {
    let (adjusted, diagnostics) = field::optimize_contour_field(
        true_dem,
        params.contour.interval,
        &params.contour.contour_field,
    )?;
    Ok(ProducedContourField {
        adjusted: Arc::new(adjusted),
        diagnostics: Arc::new(diagnostics),
    })
}

pub(crate) fn produce_scalar_contour_field_from_fitted(
    true_dem: &Dfm<Elevation>,
    params: &MapParameters,
    fitted: &field::FittedTerrain,
) -> crate::Result<ProducedContourField> {
    let (adjusted, diagnostics) = field::optimize_contour_field_with_fitted(
        true_dem,
        params.contour.interval,
        &params.contour.contour_field,
        fitted,
    )?;
    Ok(ProducedContourField {
        adjusted: Arc::new(adjusted),
        diagnostics: Arc::new(diagnostics),
    })
}

pub(crate) fn compute_scalar_field_contours_from_produced(
    true_dem: &Dfm<Elevation>,
    z_range: (f32, f32),
    cut_overlay: &geo::Polygon,
    params: &MapParameters,
    compute_energy: bool,
    produced: &ProducedContourField,
) -> crate::Result<(Vec<MapObject>, f32, f32)> {
    let adjusted = &produced.adjusted;
    let diagnostics = &produced.diagnostics;
    let level_timings = diagnostics
        .timings
        .levels
        .iter()
        .map(|level| {
            format!(
                "{:.1}m:{}/{:.1?}+{:.1?}+{:.1?}",
                level.cell_size_m,
                level.iterations,
                level.transfer,
                level.operator_norm,
                level.solve,
            )
        })
        .collect::<Vec<_>>()
        .join(",");
    let target_work = diagnostics.persistence.target_work;
    let publication_work = diagnostics.persistence.publication_work;
    log::info!(
        "contour field: {} iterations in {:.2?}, adjustment max/rms {:.3}/{:.3} m, \
         bound fraction {:.3}, energies fidelity/TV/alignment/Hessian {:.3}/{:.3}/{:.3}/{:.3}, \
         persistence requested/removed/preserved/unresolved {}/{}/{}/{}, \
         stages persistence/derivatives/salience/audit/diagnostics \
         {:.1?}/{:.1?}/{:.1?}/{:.1?}/{:.1?}, levels [{}], \
         persistence work target/audit diagrams {}/{}, passes {}/{}, candidates {}/{}, \
         cancellations {}/{}, cells {}/{}",
        diagnostics.solver.iterations,
        diagnostics.timings.total,
        diagnostics.published.maximum_adjustment,
        diagnostics.published.rms_adjustment,
        diagnostics.published.fraction_at_bound,
        diagnostics.published.fidelity_energy,
        diagnostics.published.weighted_tv_energy,
        diagnostics.published.alignment_energy,
        diagnostics.published.hessian_energy,
        diagnostics.persistence.requested,
        diagnostics.persistence.verified_removed,
        diagnostics.persistence.preserved,
        diagnostics.persistence.unresolved,
        diagnostics.timings.target_persistence,
        diagnostics.timings.derivatives,
        diagnostics.timings.salience,
        diagnostics.timings.publication_persistence,
        diagnostics.timings.published_diagnostics,
        level_timings,
        target_work.diagram_builds,
        publication_work.diagram_builds,
        target_work.cancellation_passes,
        publication_work.cancellation_passes,
        target_work.candidates_considered,
        publication_work.candidates_considered,
        target_work.cancellations_applied,
        publication_work.cancellations_applied,
        target_work.affected_cells_written,
        publication_work.affected_cells_written,
    );
    let levels = plan_contour_levels(z_range, params.contour.interval, params.contour.form_lines)?;
    let extraction_domain = extraction_polygon(true_dem);
    let contours = extract_contour_set(
        adjusted,
        &levels,
        &extraction_domain,
        ContourSimplification::DouglasPeucker(crate::SIMPLIFICATION_DIST),
    );
    finish_contours(
        contours,
        ContourPipelineContext {
            true_dem,
            contour_dem: adjusted,
            levels: &levels,
            extraction_domain: &extraction_domain,
            output_clip: cut_overlay,
            params,
        },
        ContourPipelineOptions {
            compute_energy,
            validate_vertical_tolerance: true,
            preserve_geometry: true,
            snap_boundary_to_source: true,
            protected_features: &diagnostics.protected_features,
        },
    )
}
