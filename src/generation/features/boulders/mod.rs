mod candidate;
mod diagnostics;
mod evidence;
mod opacity;
mod sides;
mod surface;
mod terrain;
#[cfg(test)]
mod tests;
mod trees;

use geo::{BoundingRect, Intersects};

use crate::{
    geometry::PointCloud,
    map::MapObject,
    parameters::BoulderParameters,
    raster::{Dfm, Elevation},
};

use candidate::classify;
pub(crate) use diagnostics::BoulderDiagnostics;
use diagnostics::Rejection;
pub(crate) use evidence::{BoulderEvidence, compute_boulder_evidence};

#[derive(Default)]
pub(crate) struct BoulderDetection {
    pub(crate) objects: Vec<MapObject>,
    pub(crate) diagnostics: BoulderDiagnostics,
}

pub(crate) fn detect_boulders(
    evidence: &BoulderEvidence,
    cloud: &PointCloud,
    dem: &Dfm<Elevation>,
    hull: &geo::Polygon,
    cut_overlay: &geo::Polygon,
    params: &BoulderParameters,
) -> BoulderDetection {
    if !params.enabled {
        return BoulderDetection::default();
    }
    let seed_height = params.minimum_height_m.max(1.);
    // Grow onto rounded shoulders without making local relief a second height
    // threshold. Tree-like return layers are removed before segmentation.
    let grow_height = seed_height * 0.35;
    let response = evidence.filtered_surface(grow_height, params);
    let mut visited = vec![false; response.len()];
    let mut objects = Vec::new();
    let mut diagnostics = BoulderDiagnostics::default();
    for seed in 0..response.len() {
        // Earlier echoes may come from canopy above an opaque boulder;
        // valid last echoes can seed it regardless of pulse multiplicity.
        if visited[seed] || response[seed] < seed_height {
            continue;
        }
        let mut cells = vec![seed];
        visited[seed] = true;
        let mut cursor = 0;
        while cursor < cells.len() {
            let cell = cells[cursor];
            let row = cell / dem.width();
            let column = cell % dem.width();
            for y in row.saturating_sub(1)..=(row + 1).min(dem.height() - 1) {
                for x in column.saturating_sub(1)..=(column + 1).min(dem.width() - 1) {
                    let neighbor = y * dem.width() + x;
                    // A separate elevated canopy must not grow into the
                    // lower object through XY adjacency alone.
                    if !visited[neighbor]
                        && response[neighbor] >= grow_height
                        && (response[neighbor] - response[cell]).abs() <= 1.5
                    {
                        visited[neighbor] = true;
                        cells.push(neighbor);
                    }
                }
            }
            cursor += 1;
        }
        diagnostics.proposed += 1;
        let initial = classify(&cells, &response, evidence, cloud, dem, params);
        let classified = if matches!(initial, Err(Rejection::Shape | Rejection::Pointed)) {
            let recovered = surface::coherent_cores(&cells, &response, dem, seed_height)
                .iter()
                .filter_map(|core| classify(core, &response, evidence, cloud, dem, params).ok())
                .collect::<Vec<_>>();
            if recovered.is_empty() {
                vec![initial]
            } else {
                recovered.into_iter().map(Ok).collect()
            }
        } else {
            vec![initial]
        };
        for classification in classified {
            match classification {
                Ok(candidate) => {
                    if !hull.intersects(&candidate.position)
                        || !cut_overlay.intersects(&candidate.position)
                    {
                        continue;
                    }
                    // Half-open ownership prevents a point exactly on a shared
                    // cut edge from being emitted by both adjacent tiles.
                    let ownership = cut_overlay.bounding_rect().expect("nonempty cut overlay");
                    if candidate.position.x() >= ownership.max().x
                        || candidate.position.y() >= ownership.max().y
                    {
                        continue;
                    }
                    let x = (candidate.position.x() - dem.grid.top_left.x) / dem.grid.cell_size_m;
                    let y = (dem.grid.top_left.y - candidate.position.y()) / dem.grid.cell_size_m;
                    let inner = dem.grid.inner;
                    if x < inner.left as f64
                        || x >= inner.right as f64
                        || y < inner.top as f64
                        || y >= inner.bottom as f64
                    {
                        continue;
                    }
                    objects.push(candidate.into_map_object(diagnostics.proposed));
                }
                Err(reason) => {
                    diagnostics.reject(reason);
                }
            }
        }
    }
    diagnostics.detected = objects.len();
    log::debug!("{}", diagnostics.summary());
    BoulderDetection {
        objects,
        diagnostics,
    }
}
