use std::collections::VecDeque;

use geo::BooleanOps;

use crate::{
    map_gen::egui_map::{AreaSymbol, MapObject},
    parameters::{MapParameters, MarshEvidenceWeights, MarshParameters},
    raster::{
        BuildingProbability, D8Flow, Dfm, Elevation, FloodFill, FlowAccumulation,
        GroundPointDensity, MarshHydrology, MarshMask, MarshProbability, MarshReason, MarshSupport,
        PointDensity, Threshold, WetnessScore,
    },
};

/// Stable codes written into [`MarshDetection::reason`]. Exclusions take
/// precedence; otherwise the code names the strongest weighted positive
/// evidence family.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MarshReasonCode {
    None = 0,
    InsufficientObservationSupport = 1,
    OpenWater = 2,
    Building = 3,
    NonPlanarSurface = 4,
    EdgeDependentDrainage = 5,
    Terrain = 10,
    Hydrology = 11,
}

pub struct MarshDetection {
    /// Evidence score before observation confidence is applied.
    pub wetness_score: Dfm<WetnessScore>,
    /// Final bounded score used for seeding and growth.
    pub probability: Dfm<MarshProbability>,
    /// Local all-return/ground-return observation confidence.
    pub support: Dfm<MarshSupport>,
    /// Exclusion or strongest-evidence diagnostic code.
    pub reason: Dfm<MarshReason>,
    /// Segmented, morphologically cleaned output mask.
    pub mask: Dfm<MarshMask>,
}

#[allow(clippy::too_many_arguments)]
pub fn compute_marsh_detection(
    dem: &Dfm<Elevation>,
    flow: &D8Flow,
    accumulation: &Dfm<FlowAccumulation>,
    hydrology: &MarshHydrology,
    point_density: &Dfm<PointDensity>,
    ground_point_density: &Dfm<GroundPointDensity>,
    open_water: &Dfm<FloodFill>,
    buildings: Option<&Dfm<BuildingProbability>>,
    params: &MarshParameters,
) -> crate::Result<MarshDetection> {
    validate_parameters(params)?;
    for grid in [
        &accumulation.grid,
        &hydrology.height_above_drainage.grid,
        &hydrology.downslope_distance_to_drainage.grid,
        &hydrology.depression_depth.grid,
        &point_density.grid,
        &ground_point_density.grid,
        &open_water.grid,
    ] {
        dem.grid.ensure_compatible(grid)?;
    }
    if let Some(buildings) = buildings {
        dem.grid.ensure_compatible(&buildings.grid)?;
    }
    let mut wetness_score = Dfm::<WetnessScore>::new_like(dem);
    let mut probability = Dfm::<MarshProbability>::new_like(dem);
    let mut support = Dfm::<MarshSupport>::new_like(dem);
    let mut reason = Dfm::<MarshReason>::new_like(dem);
    wetness_score.field.fill(0.);
    probability.field.fill(0.);
    support.field.fill(0.);
    reason.field.fill(MarshReasonCode::None as u8 as f32);

    if !params.enabled {
        return Ok(MarshDetection {
            wetness_score,
            probability,
            support,
            reason,
            mask: empty_mask(dem),
        });
    }

    let radius_cells = (params.observation_radius_m / dem.grid.cell_size_m).ceil() as usize;
    let local_point_density = local_box_mean(point_density, radius_cells);
    let local_ground_density = local_box_mean(ground_point_density, radius_cells);
    let planarity_radius_cells = (params.planarity_radius_m / dem.grid.cell_size_m).ceil() as usize;
    let planarity_rmse = local_plane_rmse(dem, planarity_radius_cells);
    let weights = normalized_weights(params.weights);
    let cell_area = dem.grid.cell_size_m.powi(2) as f32;

    for index in 0..dem.field.len() {
        let all_support =
            (local_point_density[index] / params.supported_point_density_m2).clamp(0., 1.);
        let ground_support =
            (local_ground_density[index] / params.supported_ground_density_m2).clamp(0., 1.);
        // All returns show that the cell was observed; ground returns show
        // whether terrain evidence is trustworthy. Missing ground beneath
        // vegetation lowers confidence without being treated as wetness.
        let observation_support =
            (0.35 * all_support + 0.65 * (all_support * ground_support).sqrt()).clamp(0., 1.);
        support.field[index] = observation_support;

        if open_water.field[index] >= 0.5 {
            reason.field[index] = MarshReasonCode::OpenWater as u8 as f32;
            continue;
        }
        if buildings.is_some_and(|mask| mask.field[index] >= 0.5) {
            reason.field[index] = MarshReasonCode::Building as u8 as f32;
            continue;
        }
        let local_planarity_rmse = planarity_rmse[index];
        if !local_planarity_rmse.is_finite()
            || local_planarity_rmse > params.maximum_planarity_rmse_m
        {
            reason.field[index] = MarshReasonCode::NonPlanarSurface as u8 as f32;
            continue;
        }

        // A plane is flat even when it is tilted. Plane-fit residual therefore
        // accepts smooth sloping marshes while rejecting rough terrain.
        let planarity_score =
            finite_decreasing_score(local_planarity_rmse, params.maximum_planarity_rmse_m);
        let depression_score = preferred_depression_score(
            hydrology.depression_depth.field[index],
            params.preferred_depression_depth_m,
        );
        let terrain_score = planarity_score;

        let area = accumulation.field[index];
        let accumulation_score = if area.is_finite() && area > 0. {
            ((1. + area / cell_area).ln()
                / (1. + params.drainage_initiation_area_m2 / cell_area).ln())
            .clamp(0., 1.)
        } else {
            0.
        };
        let hand_score = finite_decreasing_score(
            hydrology.height_above_drainage.field[index],
            params.maximum_height_above_drainage_m,
        );
        let distance_score = finite_decreasing_score(
            hydrology.downslope_distance_to_drainage.field[index],
            params.maximum_downslope_distance_m,
        );
        let hydrology_score = (0.25 * accumulation_score
            + 0.3 * hand_score
            + 0.2 * distance_score
            + 0.25 * depression_score)
            .clamp(0., 1.);

        let terrain_contribution = weights.terrain * terrain_score;
        let hydrology_contribution = weights.hydrology * hydrology_score;
        // Planarity is a shape requirement, not evidence of wetness. It can
        // reduce confidence but cannot rescue a hydrologically dry cell.
        let agreement = 0.65 + 0.35 * planarity_score;
        let combined = ((terrain_contribution + hydrology_contribution) * agreement).clamp(0., 1.);
        wetness_score.field[index] = hydrology_score;
        probability.field[index] = combined * (0.35 + 0.65 * observation_support);

        reason.field[index] = if observation_support < 0.2 {
            MarshReasonCode::InsufficientObservationSupport as u8 as f32
        } else if !hydrology.height_above_drainage.field[index].is_finite()
            || !hydrology.downslope_distance_to_drainage.field[index].is_finite()
        {
            MarshReasonCode::EdgeDependentDrainage as u8 as f32
        } else if hydrology_contribution >= terrain_contribution {
            MarshReasonCode::Hydrology as u8 as f32
        } else {
            MarshReasonCode::Terrain as u8 as f32
        };
    }

    let (seed_threshold, growth_threshold) = effective_thresholds(params);
    let seed_cells = probability
        .field
        .iter()
        .zip(&wetness_score.field)
        .map(|(probability, wetness)| {
            probability.is_finite()
                && *probability >= seed_threshold
                && *wetness >= params.minimum_wetness_score
        })
        .collect::<Vec<_>>();
    let growth_cells = probability
        .field
        .iter()
        .zip(&wetness_score.field)
        .map(|(probability, wetness)| {
            probability.is_finite()
                && *probability >= growth_threshold
                && *wetness >= params.minimum_wetness_score
        })
        .collect::<Vec<_>>();
    let mut retained = flow.grow_mask_along_flow(&seed_cells, &growth_cells);
    let closing_cells = metres_to_cells(params.closing_radius_m, dem.grid.cell_size_m);
    retained = close_mask(&retained, dem.width(), dem.height(), closing_cells);
    let opening_cells = metres_to_cells(params.opening_radius_m, dem.grid.cell_size_m);
    retained = open_mask(&retained, dem.width(), dem.height(), opening_cells);
    fill_small_holes(
        &mut retained,
        dem.width(),
        dem.height(),
        dem.grid.cell_size_m.powi(2),
        params.maximum_hole_area_m2,
    );
    // Morphology may regularize the boundary, but it must not add dry,
    // non-planar, or otherwise excluded cells outside the growth set.
    for index in 0..retained.len() {
        retained[index] &= growth_cells[index]
            && open_water.field[index] < 0.5
            && !buildings.is_some_and(|mask| mask.field[index] >= 0.5);
    }
    retain_minimum_area_components(
        &mut retained,
        dem.width(),
        dem.height(),
        dem.grid.cell_size_m.powi(2),
        params.minimum_polygon_area_m2,
    );
    let mut mask = Dfm::<MarshMask>::new_like(dem);
    for (value, retained) in mask.field.iter_mut().zip(retained) {
        *value = if retained { 1. } else { 0. };
    }

    Ok(MarshDetection {
        wetness_score,
        probability,
        support,
        reason,
        mask,
    })
}

pub fn marsh_objects(
    detection: &MarshDetection,
    hull: &geo::Polygon,
    cut_overlay: &geo::Polygon,
    params: &MapParameters,
    exclusions: &geo::MultiPolygon,
) -> Vec<MapObject> {
    let mut objects = super::compute_vegetation(
        &detection.mask,
        Threshold::Lower(0.5),
        hull,
        cut_overlay,
        AreaSymbol::Marsh,
        params,
        &params.geometry.marsh.buffer_rules,
    );

    if !exclusions.0.is_empty() {
        let mut clipped = Vec::new();
        for object in objects {
            let MapObject::Area {
                object,
                symbol,
                tags,
            } = object
            else {
                unreachable!("marsh vectorization emits only areas");
            };
            clipped.extend(object.difference(exclusions).into_iter().map(|object| {
                MapObject::Area {
                    object,
                    symbol,
                    tags: tags.clone(),
                }
            }));
        }
        objects = clipped;
    }

    for object in &mut objects {
        if let MapObject::Area { tags, .. } = object {
            tags.insert("Detector".into(), "flow-marsh-v2".into());
        }
    }
    objects
}

fn validate_parameters(params: &MarshParameters) -> crate::Result<()> {
    anyhow::ensure!(
        (0.0..=1.0).contains(&params.sensitivity),
        "marsh sensitivity must be in 0..=1"
    );
    anyhow::ensure!(
        params.minimum_polygon_area_m2.is_finite() && params.minimum_polygon_area_m2 >= 0.,
        "marsh minimum area must be non-negative and finite"
    );
    anyhow::ensure!(
        params.maximum_planarity_rmse_m.is_finite() && params.maximum_planarity_rmse_m > 0.,
        "marsh maximum planarity residual must be positive and finite"
    );
    for (name, value) in [
        (
            "drainage-initiation area",
            f64::from(params.drainage_initiation_area_m2),
        ),
        (
            "maximum HAND",
            f64::from(params.maximum_height_above_drainage_m),
        ),
        (
            "maximum downslope distance",
            f64::from(params.maximum_downslope_distance_m),
        ),
        (
            "preferred depression depth",
            f64::from(params.preferred_depression_depth_m),
        ),
        ("observation radius", params.observation_radius_m),
        ("planarity radius", params.planarity_radius_m),
        (
            "supported point density",
            f64::from(params.supported_point_density_m2),
        ),
        (
            "supported ground density",
            f64::from(params.supported_ground_density_m2),
        ),
    ] {
        anyhow::ensure!(
            value.is_finite() && value > 0.,
            "marsh {name} must be positive and finite"
        );
    }
    for (name, value) in [
        ("closing radius", params.closing_radius_m),
        ("opening radius", params.opening_radius_m),
        ("maximum hole area", params.maximum_hole_area_m2),
    ] {
        anyhow::ensure!(
            value.is_finite() && value >= 0.,
            "marsh {name} must be non-negative and finite"
        );
    }
    anyhow::ensure!(
        (0.0..=1.0).contains(&params.seed_threshold)
            && (0.0..=1.0).contains(&params.growth_threshold)
            && (0.0..=1.0).contains(&params.minimum_wetness_score),
        "marsh probability thresholds must be in 0..=1"
    );
    for weight in [params.weights.terrain, params.weights.hydrology] {
        anyhow::ensure!(
            weight.is_finite() && weight >= 0.,
            "marsh weights must be non-negative and finite"
        );
    }
    anyhow::ensure!(
        params.weights.terrain + params.weights.hydrology > 0.,
        "at least one marsh evidence weight must be positive"
    );
    Ok(())
}

fn normalized_weights(weights: MarshEvidenceWeights) -> MarshEvidenceWeights {
    let sum = weights.terrain + weights.hydrology;
    MarshEvidenceWeights {
        terrain: weights.terrain / sum,
        hydrology: weights.hydrology / sum,
    }
}

fn effective_thresholds(params: &MarshParameters) -> (f32, f32) {
    let offset = (0.5 - params.sensitivity) * 0.24;
    let growth = (params.growth_threshold + offset).clamp(0., 0.94);
    let seed = (params.seed_threshold + offset).clamp(growth + 0.05, 1.);
    (seed, growth)
}

fn increasing_ramp(value: f32, low: f32, high: f32) -> f32 {
    if !value.is_finite() {
        return 0.;
    }
    smoothstep(((value - low) / (high - low).max(f32::EPSILON)).clamp(0., 1.))
}

fn decreasing_ramp(value: f32, low: f32, high: f32) -> f32 {
    1. - increasing_ramp(value, low, high)
}

fn finite_decreasing_score(value: f32, maximum: f32) -> f32 {
    if value.is_finite() {
        1. - smoothstep((value / maximum).clamp(0., 1.))
    } else {
        0.
    }
}

fn preferred_depression_score(depth: f32, preferred: f32) -> f32 {
    if !depth.is_finite() {
        return 0.;
    }
    if depth <= preferred {
        increasing_ramp(depth, 0., preferred)
    } else {
        decreasing_ramp(depth, preferred, 4. * preferred)
    }
}

fn smoothstep(value: f32) -> f32 {
    value * value * (3. - 2. * value)
}

#[derive(Clone, Copy, Default)]
struct PlaneMoments {
    count: f64,
    x: f64,
    y: f64,
    z: f64,
    xx: f64,
    xy: f64,
    yy: f64,
    xz: f64,
    yz: f64,
    zz: f64,
}

impl PlaneMoments {
    fn sample(x: f64, y: f64, z: f64) -> Self {
        Self {
            count: 1.,
            x,
            y,
            z,
            xx: x * x,
            xy: x * y,
            yy: y * y,
            xz: x * z,
            yz: y * z,
            zz: z * z,
        }
    }

    fn add(self, other: Self) -> Self {
        Self {
            count: self.count + other.count,
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
            xx: self.xx + other.xx,
            xy: self.xy + other.xy,
            yy: self.yy + other.yy,
            xz: self.xz + other.xz,
            yz: self.yz + other.yz,
            zz: self.zz + other.zz,
        }
    }

    fn sub(self, other: Self) -> Self {
        Self {
            count: self.count - other.count,
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
            xx: self.xx - other.xx,
            xy: self.xy - other.xy,
            yy: self.yy - other.yy,
            xz: self.xz - other.xz,
            yz: self.yz - other.yz,
            zz: self.zz - other.zz,
        }
    }
}

/// Residual from a local best-fit plane. Unlike absolute slope, this measures
/// whether the surface itself is flat and therefore gives the same result for
/// a horizontal marsh and a smoothly tilted one.
fn local_plane_rmse(dem: &Dfm<Elevation>, radius: usize) -> Vec<f32> {
    let width = dem.width();
    let height = dem.height();
    let stride = width + 1;
    let cell_size = dem.grid.cell_size_m;
    let mut prefix = vec![PlaneMoments::default(); stride * (height + 1)];
    for y in 0..height {
        for x in 0..width {
            let sample = PlaneMoments::sample(
                x as f64 * cell_size,
                y as f64 * cell_size,
                f64::from(dem[(y, x)]),
            );
            let destination = (y + 1) * stride + x + 1;
            prefix[destination] = sample
                .add(prefix[y * stride + x + 1])
                .add(prefix[(y + 1) * stride + x])
                .sub(prefix[y * stride + x]);
        }
    }

    let mut residuals = vec![f32::INFINITY; width * height];
    for y in 0..height {
        let top = y.saturating_sub(radius);
        let bottom = (y + radius + 1).min(height);
        for x in 0..width {
            let left = x.saturating_sub(radius);
            let right = (x + radius + 1).min(width);
            let moments = prefix[bottom * stride + right]
                .sub(prefix[top * stride + right])
                .sub(prefix[bottom * stride + left])
                .add(prefix[top * stride + left]);
            if moments.count < 3. {
                continue;
            }
            let sxx = moments.xx - moments.x * moments.x / moments.count;
            let sxy = moments.xy - moments.x * moments.y / moments.count;
            let syy = moments.yy - moments.y * moments.y / moments.count;
            let sxz = moments.xz - moments.x * moments.z / moments.count;
            let syz = moments.yz - moments.y * moments.z / moments.count;
            let szz = (moments.zz - moments.z * moments.z / moments.count).max(0.);
            let determinant = sxx * syy - sxy * sxy;
            if determinant <= f64::EPSILON {
                continue;
            }
            let plane_x = (sxz * syy - syz * sxy) / determinant;
            let plane_y = (syz * sxx - sxz * sxy) / determinant;
            let residual_sum = (szz - plane_x * sxz - plane_y * syz).max(0.);
            residuals[y * width + x] = (residual_sum / moments.count).sqrt() as f32;
        }
    }
    residuals
}

fn local_box_mean<T: crate::raster::RasterMarker>(source: &Dfm<T>, radius: usize) -> Vec<f32> {
    let width = source.width();
    let height = source.height();
    let stride = width + 1;
    let mut prefix = vec![0_f64; stride * (height + 1)];
    for y in 0..height {
        let mut row_sum = 0.;
        for x in 0..width {
            row_sum += f64::from(source[(y, x)].max(0.));
            prefix[(y + 1) * stride + x + 1] = prefix[y * stride + x + 1] + row_sum;
        }
    }
    let mut mean = vec![0.; width * height];
    for y in 0..height {
        let top = y.saturating_sub(radius);
        let bottom = (y + radius + 1).min(height);
        for x in 0..width {
            let left = x.saturating_sub(radius);
            let right = (x + radius + 1).min(width);
            let sum = prefix[bottom * stride + right] + prefix[top * stride + left]
                - prefix[top * stride + right]
                - prefix[bottom * stride + left];
            mean[y * width + x] = (sum / ((bottom - top) * (right - left)) as f64) as f32;
        }
    }
    mean
}

fn metres_to_cells(metres: f64, cell_size_m: f64) -> usize {
    if metres <= 0. {
        0
    } else {
        (metres / cell_size_m).ceil() as usize
    }
}

fn disk_offsets(radius: usize) -> Vec<(isize, isize)> {
    let radius = radius as isize;
    let radius2 = radius * radius;
    (-radius..=radius)
        .flat_map(|dy| {
            (-radius..=radius)
                .filter_map(move |dx| ((dx * dx + dy * dy) <= radius2).then_some((dy, dx)))
        })
        .collect()
}

fn dilate(mask: &[bool], width: usize, height: usize, radius: usize) -> Vec<bool> {
    if radius == 0 {
        return mask.to_vec();
    }
    let offsets = disk_offsets(radius);
    let mut output = vec![false; mask.len()];
    for y in 0..height {
        for x in 0..width {
            output[y * width + x] = offsets.iter().any(|&(dy, dx)| {
                let ny = y as isize + dy;
                let nx = x as isize + dx;
                ny >= 0
                    && nx >= 0
                    && ny < height as isize
                    && nx < width as isize
                    && mask[ny as usize * width + nx as usize]
            });
        }
    }
    output
}

fn erode(mask: &[bool], width: usize, height: usize, radius: usize) -> Vec<bool> {
    if radius == 0 {
        return mask.to_vec();
    }
    let offsets = disk_offsets(radius);
    let mut output = vec![false; mask.len()];
    for y in 0..height {
        for x in 0..width {
            output[y * width + x] = offsets.iter().all(|&(dy, dx)| {
                let ny = y as isize + dy;
                let nx = x as isize + dx;
                ny >= 0
                    && nx >= 0
                    && ny < height as isize
                    && nx < width as isize
                    && mask[ny as usize * width + nx as usize]
            });
        }
    }
    output
}

fn close_mask(mask: &[bool], width: usize, height: usize, radius: usize) -> Vec<bool> {
    erode(&dilate(mask, width, height, radius), width, height, radius)
}

fn open_mask(mask: &[bool], width: usize, height: usize, radius: usize) -> Vec<bool> {
    dilate(&erode(mask, width, height, radius), width, height, radius)
}

fn fill_small_holes(
    mask: &mut [bool],
    width: usize,
    height: usize,
    cell_area: f64,
    maximum_area: f64,
) {
    let maximum_cells = (maximum_area / cell_area).floor() as usize;
    if maximum_cells == 0 {
        return;
    }
    let mut visited = vec![false; mask.len()];
    for start in 0..mask.len() {
        if mask[start] || visited[start] {
            continue;
        }
        visited[start] = true;
        let mut queue = VecDeque::from([start]);
        let mut component = Vec::new();
        let mut touches_edge = false;
        while let Some(index) = queue.pop_front() {
            component.push(index);
            let y = index / width;
            let x = index % width;
            touches_edge |= y == 0 || x == 0 || y + 1 == height || x + 1 == width;
            for neighbour in neighbors4(y, x, width, height) {
                if !mask[neighbour] && !visited[neighbour] {
                    visited[neighbour] = true;
                    queue.push_back(neighbour);
                }
            }
        }
        if !touches_edge && component.len() <= maximum_cells {
            for index in component {
                mask[index] = true;
            }
        }
    }
}

fn retain_minimum_area_components(
    mask: &mut [bool],
    width: usize,
    height: usize,
    cell_area: f64,
    minimum_area: f64,
) {
    let minimum_cells = (minimum_area / cell_area).ceil() as usize;
    if minimum_cells <= 1 {
        return;
    }
    let mut visited = vec![false; mask.len()];
    for start in 0..mask.len() {
        if !mask[start] || visited[start] {
            continue;
        }
        visited[start] = true;
        let mut queue = VecDeque::from([start]);
        let mut component = Vec::new();
        while let Some(index) = queue.pop_front() {
            component.push(index);
            let y = index / width;
            let x = index % width;
            for neighbour in neighbors8(y, x, width, height) {
                if mask[neighbour] && !visited[neighbour] {
                    visited[neighbour] = true;
                    queue.push_back(neighbour);
                }
            }
        }
        if component.len() < minimum_cells {
            for index in component {
                mask[index] = false;
            }
        }
    }
}

fn neighbors4(y: usize, x: usize, width: usize, height: usize) -> impl Iterator<Item = usize> {
    [(-1_isize, 0_isize), (0, 1), (1, 0), (0, -1)]
        .into_iter()
        .filter_map(move |(dy, dx)| {
            let ny = y as isize + dy;
            let nx = x as isize + dx;
            (ny >= 0 && nx >= 0 && ny < height as isize && nx < width as isize)
                .then(|| ny as usize * width + nx as usize)
        })
}

fn neighbors8(y: usize, x: usize, width: usize, height: usize) -> impl Iterator<Item = usize> {
    (-1_isize..=1).flat_map(move |dy| {
        (-1_isize..=1).filter_map(move |dx| {
            if dy == 0 && dx == 0 {
                return None;
            }
            let ny = y as isize + dy;
            let nx = x as isize + dx;
            (ny >= 0 && nx >= 0 && ny < height as isize && nx < width as isize)
                .then(|| ny as usize * width + nx as usize)
        })
    })
}

fn empty_mask<T: crate::raster::RasterMarker>(source: &Dfm<T>) -> Dfm<MarshMask> {
    let mut mask = Dfm::<MarshMask>::new_like(source);
    mask.field.fill(0.);
    mask
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::raster::{
        DepressionDepth, DfmGrid, DownslopeDistanceToDrainage, HeightAboveDrainage,
    };

    #[test]
    fn probability_requires_observation_support_and_excludes_water() {
        let grid = DfmGrid::standard(geo::coord! { x: 0., y: 256. });
        let mut dem = Dfm::<Elevation>::new(grid.clone());
        let mut accumulation = Dfm::<FlowAccumulation>::new(grid.clone());
        let mut hand = Dfm::<HeightAboveDrainage>::new(grid.clone());
        let mut distance = Dfm::<DownslopeDistanceToDrainage>::new(grid.clone());
        let mut depression = Dfm::<DepressionDepth>::new(grid.clone());
        let mut density = Dfm::<PointDensity>::new(grid.clone());
        let mut ground_density = Dfm::<GroundPointDensity>::new(grid.clone());
        let mut water = Dfm::<FloodFill>::new(grid.clone());
        for y in 0..dem.height() {
            for x in 0..dem.width() {
                // Smooth plane with substantial absolute slope.
                dem[(y, x)] = 100. + 0.3 * x as f32 - 0.2 * y as f32;
            }
        }
        dem[(120, 120)] += 1.;
        accumulation.field.fill(5_000.);
        hand.field.fill(0.1);
        distance.field.fill(2.);
        depression.field.fill(0.2);
        density.field.fill(8.);
        ground_density.field.fill(2.);
        water.field.fill(0.);
        water[(256, 256)] = 1.;
        for y in 0..4 {
            for x in 0..4 {
                density[(y, x)] = 0.;
                ground_density[(y, x)] = 0.;
            }
        }
        let hydrology = MarshHydrology {
            height_above_drainage: hand,
            downslope_distance_to_drainage: distance,
            depression_depth: depression,
        };
        let params = MarshParameters {
            minimum_polygon_area_m2: 0.,
            closing_radius_m: 0.,
            opening_radius_m: 0.,
            ..MarshParameters::default()
        };
        let corrected = dem.hydrological_correction();
        let flow = dem.hydrological_analysis_with_corrected(&corrected);
        let detection = compute_marsh_detection(
            &dem,
            &flow,
            &accumulation,
            &hydrology,
            &density,
            &ground_density,
            &water,
            None,
            &params,
        )
        .unwrap();
        assert!(detection.probability[(100, 100)] > detection.probability[(0, 0)]);
        assert_eq!(detection.mask[(100, 100)], 1.);
        assert_eq!(detection.mask[(120, 120)], 0.);
        assert_eq!(
            detection.reason[(120, 120)],
            MarshReasonCode::NonPlanarSurface as u8 as f32
        );
        assert_eq!(detection.mask[(256, 256)], 0.);
        assert_eq!(
            detection.reason[(256, 256)],
            MarshReasonCode::OpenWater as u8 as f32
        );
    }

    #[test]
    fn tilted_plane_is_flat_but_local_bump_is_not() {
        let grid = DfmGrid::new(9, 9, 1., geo::coord! { x: 0., y: 9. }).unwrap();
        let mut dem = Dfm::<Elevation>::new(grid);
        for y in 0..dem.height() {
            for x in 0..dem.width() {
                dem[(y, x)] = 10. + 0.6 * x as f32 - 0.4 * y as f32;
            }
        }
        let planar = local_plane_rmse(&dem, 2);
        assert!(planar[4 * dem.width() + 4] < 1.0e-5);

        dem[(4, 4)] += 1.;
        let rough = local_plane_rmse(&dem, 2);
        assert!(rough[4 * dem.width() + 4] > 0.15);
    }

    #[test]
    fn metre_to_cell_conversion_is_resolution_stable() {
        assert_eq!(metres_to_cells(1.5, 0.5), 3);
        assert_eq!(metres_to_cells(1.5, 1.), 2);
        assert_eq!(metres_to_cells(0., 0.5), 0);
    }

    #[test]
    fn planar_but_dry_surface_is_not_a_high_confidence_seed() {
        let weights = normalized_weights(MarshEvidenceWeights::default());
        let terrain_score = 1.;
        let hydrology_score = 0.;
        let agreement = 1.;
        let score =
            (weights.terrain * terrain_score + weights.hydrology * hydrology_score) * agreement;
        assert!(score < MarshParameters::default().seed_threshold);
    }
}
