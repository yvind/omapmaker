use std::collections::HashMap;

use crate::{
    geometry::{PointCloud, PointLaz},
    map::{MapObject, PointSymbol},
    parameters::BoulderParameters,
    raster::{Dfm, Elevation},
};

use super::evidence::{is_last, object_class};
use super::{BoulderEvidence, Rejection, opacity, sides, surface, terrain, trees};

pub(super) struct Candidate {
    pub(super) position: geo::Point,
    height: f64,
    diameter: f64,
    surface_area: f64,
    footprint: geo::Rect,
    confidence: f64,
    under_surface_fraction: f64,
    nonterminal_fraction: f64,
    grounded_side_fraction: f64,
    minimum_side_slope: f64,
    top_area: f64,
    top_fraction: f64,
    surface_roughness: Option<f64>,
    dome_fraction: f64,
    last_support: usize,
    single_support: usize,
}

impl Candidate {
    pub(super) fn into_map_object(self, candidate_id: usize) -> MapObject {
        let tags = [
            ("boulder_height_m", self.height.to_string()),
            ("boulder_diameter_m", self.diameter.to_string()),
            ("boulder_surface_area_m2", self.surface_area.to_string()),
            (
                "boulder_footprint_min_x",
                self.footprint.min().x.to_string(),
            ),
            (
                "boulder_footprint_min_y",
                self.footprint.min().y.to_string(),
            ),
            (
                "boulder_footprint_max_x",
                self.footprint.max().x.to_string(),
            ),
            (
                "boulder_footprint_max_y",
                self.footprint.max().y.to_string(),
            ),
            ("boulder_confidence", self.confidence.to_string()),
            (
                "boulder_under_surface_fraction",
                self.under_surface_fraction.to_string(),
            ),
            (
                "boulder_nonterminal_fraction",
                self.nonterminal_fraction.to_string(),
            ),
            ("boulder_last_return_support", self.last_support.to_string()),
            (
                "boulder_single_return_support",
                self.single_support.to_string(),
            ),
            ("boulder_top_area_m2", self.top_area.to_string()),
            ("boulder_top_fraction", self.top_fraction.to_string()),
            ("boulder_dome_fraction", self.dome_fraction.to_string()),
            (
                "boulder_surface_roughness_m",
                self.surface_roughness.map_or_else(
                    || "unmeasured".to_string(),
                    |roughness| roughness.to_string(),
                ),
            ),
            (
                "boulder_grounded_side_fraction",
                self.grounded_side_fraction.to_string(),
            ),
            (
                "boulder_min_side_slope_degrees",
                self.minimum_side_slope.to_string(),
            ),
            ("boulder_candidate_id", candidate_id.to_string()),
        ]
        .into_iter()
        .map(|(key, value)| (key.to_string(), value))
        .collect();
        MapObject::Point {
            object: self.position,
            symbol: if self.height >= 2. {
                PointSymbol::LargeBoulder
            } else {
                PointSymbol::SmallBoulder
            },
            rotation: 0.,
            tags,
        }
    }
}

pub(super) fn classify(
    cells: &[usize],
    response: &[f32],
    evidence: &BoulderEvidence,
    cloud: &PointCloud,
    dem: &Dfm<Elevation>,
    params: &BoulderParameters,
) -> Result<Candidate, Rejection> {
    let (min_x, max_x, min_y, max_y) = cells.iter().fold(
        (usize::MAX, 0, usize::MAX, 0),
        |(min_x, max_x, min_y, max_y), &cell| {
            let x = cell % dem.width();
            let y = cell / dem.width();
            (min_x.min(x), max_x.max(x), min_y.min(y), max_y.max(y))
        },
    );
    let cell_size = dem.grid.cell_size_m;
    let diameter = ((max_x - min_x + 1) as f64).hypot((max_y - min_y + 1) as f64) * cell_size;
    let compactness = cells.len() as f64 / ((max_x - min_x + 1) * (max_y - min_y + 1)) as f64;
    if cells.len() < 4 || compactness < 0.4 {
        return Err(Rejection::Shape);
    }
    let count = cells.len() as f64;
    let mean_x = cells
        .iter()
        .map(|cell| (cell % dem.width()) as f64)
        .sum::<f64>()
        / count;
    let mean_y = cells
        .iter()
        .map(|cell| (cell / dem.width()) as f64)
        .sum::<f64>()
        / count;
    let (mut xx, mut xy, mut yy) = (1. / 12., 0., 1. / 12.);
    for &cell in cells {
        let dx = (cell % dem.width()) as f64 - mean_x;
        let dy = (cell / dem.width()) as f64 - mean_y;
        xx += dx * dx / count;
        xy += dx * dy / count;
        yy += dy * dy / count;
    }
    let spread = (xx - yy).hypot(2. * xy);
    let elongation = ((xx + yy + spread) / (xx + yy - spread).max(1e-9)).sqrt();
    if elongation > 2.5 {
        return Err(Rejection::Shape);
    }

    let center = geo::coord! {
        x: dem.grid.top_left.x + mean_x * cell_size,
        y: dem.grid.top_left.y - mean_y * cell_size,
    };
    let plane = terrain::context_plane(dem, (min_x, max_x, min_y, max_y), center)
        .ok_or(Rejection::Context)?;
    let mut support = Vec::new();
    let mut surface_heights = HashMap::new();
    for &cell in cells {
        let mut cell_heights = Vec::new();
        let mut cell_support = Vec::new();
        for &index in evidence.points_in_cell(cell) {
            let point = &cloud.points[index];
            if !is_last(point) || !object_class(point.0.classification) {
                continue;
            }
            let height = f64::from(evidence.heights_above_ground[index]);
            // Only the selected local layer provides height and placement.
            // Low side returns and overhead last echoes stay independent.
            if height >= f64::from(params.minimum_height_m.max(1.)) * 0.2
                && (height - f64::from(response[cell])).abs() <= opacity::SURFACE_TOLERANCE_M
            {
                cell_support.push((point, height));
            }
        }
        for (point, height) in cell_support {
            support.push((point, height));
            cell_heights.push(height);
        }
        if !cell_heights.is_empty() {
            cell_heights.sort_unstable_by(f64::total_cmp);
            let top = cell_heights[((cell_heights.len() - 1) as f64 * 0.98).round() as usize];
            surface_heights.insert(cell, top);
        }
    }
    // Duplicate observations cannot turn one physical return into support.
    support.sort_unstable_by(|(a, _), (b, _)| {
        a.x()
            .total_cmp(&b.x())
            .then(a.y().total_cmp(&b.y()))
            .then(a.0.z.total_cmp(&b.0.z))
            .then(a.0.number_of_returns.cmp(&b.0.number_of_returns))
    });
    support.dedup_by(|(a, _), (b, _)| a.x() == b.x() && a.y() == b.y() && a.0.z == b.0.z);
    if support.len() < 6 || surface_heights.len() < 4 {
        return Err(Rejection::Support);
    }
    // Sparse rocks often have just one observation at their highest point.
    // Validate that peak against a neighboring surface cell instead of always
    // deleting the highest return with a percentile rounded downward.
    let height = surface::supported_peak(&surface_heights, dem).ok_or(Rejection::Support)?;
    if height < f64::from(params.minimum_height_m.max(1.)) {
        return Err(Rejection::Height);
    }
    // A small patch high in a crown can look locally opaque after vegetation
    // filtering has removed its surroundings. Its vertical proportions still
    // contradict a broad boulder. Base this on observed area, not the inflated
    // bounding box of a sparse fragment.
    let supported_diameter =
        (4. * surface_heights.len() as f64 * cell_size.powi(2) / std::f64::consts::PI).sqrt();
    if height / supported_diameter > f64::from(params.maximum_height_to_diameter) {
        return Err(Rejection::Shape);
    }
    trees::check(&surface_heights, height, evidence, dem)?;
    let surface = surface::shape(&surface_heights, dem, height, params)?;
    let opacity = opacity::check(&surface_heights, evidence, cloud, dem, params)?;

    let sides = sides::grounded_runoffs(
        &surface_heights,
        evidence,
        cloud,
        dem,
        center,
        height,
        params,
    )
    .ok_or(Rejection::Sides)?;
    // Physical checks decide acceptance; this score ranks accepted boulders
    // when their symbols compete for map space.
    let confidence = 0.2 * sides.grounded_fraction
        + 0.35 * (1. - opacity.under_surface_fraction)
        + 0.25 * (1. - opacity.nonterminal_fraction)
        + 0.1 * compactness
        + 0.1 / elongation
        - 0.1 * (plane.roughness / height).min(1.)
        - if surface.roughness.is_none() {
            0.05
        } else {
            0.
        };
    // Choose a supported upper-surface return nearest its robust centroid.
    // Extreme outliers cannot control either the symbol height or placement.
    let upper = support
        .iter()
        .filter(|(_, z)| *z >= height * 0.7 && *z <= height + 0.1)
        .collect::<Vec<_>>();
    let mean = upper
        .iter()
        .fold((0., 0.), |(x, y), (p, _)| (x + p.x(), y + p.y()));
    let mean = (mean.0 / upper.len() as f64, mean.1 / upper.len() as f64);
    let (point, _) = upper
        .into_iter()
        .min_by(|(a, _), (b, _)| {
            let distance = |p: &PointLaz| (p.x() - mean.0).powi(2) + (p.y() - mean.1).powi(2);
            distance(a)
                .total_cmp(&distance(b))
                .then(a.x().total_cmp(&b.x()))
                .then(a.y().total_cmp(&b.y()))
        })
        .ok_or(Rejection::Support)?;
    Ok(Candidate {
        position: geo::Point::new(point.x(), point.y()),
        height,
        diameter,
        surface_area: surface_heights.len() as f64 * cell_size.powi(2),
        footprint: geo::Rect::new(
            (
                dem.grid.top_left.x + (min_x as f64 - 0.5) * cell_size,
                dem.grid.top_left.y - (max_y as f64 + 0.5) * cell_size,
            ),
            (
                dem.grid.top_left.x + (max_x as f64 + 0.5) * cell_size,
                dem.grid.top_left.y - (min_y as f64 - 0.5) * cell_size,
            ),
        ),
        confidence,
        under_surface_fraction: opacity.under_surface_fraction,
        nonterminal_fraction: opacity.nonterminal_fraction,
        grounded_side_fraction: sides.grounded_fraction,
        minimum_side_slope: sides.minimum_slope_degrees,
        top_area: surface.top_area,
        top_fraction: surface.top_fraction,
        surface_roughness: surface.roughness,
        dome_fraction: surface.dome_fraction,
        last_support: support.len(),
        single_support: support
            .iter()
            .filter(|(point, _)| point.0.number_of_returns == 1)
            .count(),
    })
}
