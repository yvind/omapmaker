use crate::raster::{Dfm, Elevation};
use std::sync::Mutex;

#[cfg(feature = "stream-svf-slope")]
use crate::inference::{
    contract::InputDescriptor,
    input::{NamedRaster, NchwInput, build_nchw},
    runtime::InferenceLane,
};
#[cfg(feature = "stream-svf-slope")]
use rayon::prelude::*;
#[cfg(feature = "stream-svf-slope")]
use std::{collections::VecDeque, sync::Arc};

#[cfg(feature = "stream-svf-slope")]
pub(crate) use crate::inference::prediction::PredictionRaster as Prediction;
#[cfg(not(feature = "stream-svf-slope"))]
pub(crate) struct Prediction;

#[cfg(feature = "stream-svf-slope")]
const MODEL_ID: &str = "ditches_streams_svf_slope";

/// Source rasters required by the ditches/streams SVF+slope model.
#[cfg_attr(not(feature = "stream-svf-slope"), allow(dead_code))]
pub(crate) struct Input<'a> {
    pub dem: &'a Dfm<Elevation>,
}

#[cfg(feature = "stream-svf-slope")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct PredictionCacheKey {
    tile_revision: u64,
    model_id: &'static str,
    onnx_sha256: &'static str,
    manifest_sha256: &'static str,
    contract_version: u32,
}

#[derive(Default)]
pub(crate) struct PredictionCache {
    #[cfg(feature = "stream-svf-slope")]
    entries: VecDeque<(PredictionCacheKey, Arc<Prediction>)>,
}

pub(crate) fn predict(
    cache: &Mutex<PredictionCache>,
    tile_revision: u64,
    input: Input<'_>,
    cancellation: &crate::cancellation::CancellationToken,
) -> crate::Result<std::sync::Arc<Prediction>> {
    #[cfg(feature = "stream-svf-slope")]
    {
        cancellation.check()?;
        let inference = inference_lane()?;
        let descriptor = inference.descriptor();
        let key = PredictionCacheKey {
            tile_revision,
            model_id: descriptor.id,
            onnx_sha256: descriptor.onnx_sha256,
            manifest_sha256: descriptor.manifest_sha256,
            contract_version: descriptor.contract_version,
        };
        let mut cache_guard = cache.lock().expect("prediction cache poisoned");
        if let Some(position) = cache_guard
            .entries
            .iter()
            .position(|(candidate, _)| *candidate == key)
        {
            let entry = cache_guard
                .entries
                .remove(position)
                .expect("cache position exists");
            let prediction = Arc::clone(&entry.1);
            cache_guard.entries.push_back(entry);
            return Ok(prediction);
        }
        drop(cache_guard);

        let input = build_input(&descriptor.input, input)?;
        cancellation.check()?;
        let prediction = Arc::new(inference.predict(input, cancellation)?);
        cancellation.check()?;
        let mut cache_guard = cache.lock().expect("prediction cache poisoned");
        if cache_guard.entries.len() == 2 {
            cache_guard.entries.pop_front();
        }
        cache_guard
            .entries
            .push_back((key, Arc::clone(&prediction)));
        Ok(prediction)
    }
    #[cfg(not(feature = "stream-svf-slope"))]
    {
        let _ = (cache, tile_revision, input, cancellation);
        Err(crate::Error::AlgorithmUnavailable {
            feature: "streams",
            algorithm: "ditches-streams-svf-slope",
        }
        .into())
    }
}

#[cfg(feature = "stream-svf-slope")]
fn inference_lane() -> crate::Result<Arc<InferenceLane>> {
    static LANE: Mutex<Option<Arc<InferenceLane>>> = Mutex::new(None);
    let mut slot = LANE.lock().expect("inference lane cache poisoned");
    if let Some(lane) = slot.as_ref() {
        return Ok(Arc::clone(lane));
    }
    let lane = Arc::new(InferenceLane::new(MODEL_ID)?);
    *slot = Some(Arc::clone(&lane));
    Ok(lane)
}

// These are the parameters used to prepare the rasters on which the embedded
// ditch/stream model was trained. RVT expresses the radius in pixels and the
// supplied training script deliberately passes a resolution of one.
#[cfg(feature = "stream-svf-slope")]
const RVT_SVF_DIRECTIONS: usize = 16;
#[cfg(feature = "stream-svf-slope")]
const RVT_SVF_RADIUS_PIXELS: usize = 10;
#[cfg(feature = "stream-svf-slope")]
const RVT_SVF_MIN_RADIUS_PIXELS: usize = 1;
#[cfg(feature = "stream-svf-slope")]
const RVT_SVF_SAMPLES_PER_PIXEL: usize = 3;
#[cfg(feature = "stream-svf-slope")]
const RVT_SVF_RESOLUTION: f32 = 1.;

#[cfg(feature = "stream-svf-slope")]
fn build_input(descriptor: &InputDescriptor, input: Input<'_>) -> crate::Result<NchwInput> {
    let sky_view_factor = rvt_sky_view_factor(input.dem);
    let normalized_slope = whitebox_normalized_slope(input.dem);
    build_nchw(
        descriptor,
        &[
            NamedRaster {
                name: "sky_view_factor",
                grid: &input.dem.grid,
                values: &sky_view_factor,
            },
            NamedRaster {
                name: "slope",
                grid: &input.dem.grid,
                values: &normalized_slope,
            },
        ],
    )
}

#[cfg(feature = "stream-svf-slope")]
#[derive(Clone, Copy, Debug)]
struct HorizonOffset {
    axis_0_shift: isize,
    axis_1_shift: isize,
    distance: f32,
}

/// Port of `rvt.vis.sky_view_factor` for the exact parameters in the model's
/// training pipeline: resolution 1, 16 directions, radius 10 and noise 0.
#[cfg(feature = "stream-svf-slope")]
fn rvt_sky_view_factor(dem: &Dfm<Elevation>) -> Vec<f32> {
    let directions = rvt_horizon_offsets();
    let width = dem.width();
    let height = dem.height();
    let mut output = vec![0.; dem.field.len()];
    output
        .par_chunks_mut(width)
        .enumerate()
        .for_each(|(y, row)| {
            for (x, value) in row.iter_mut().enumerate() {
                let center = dem[(y, x)] / RVT_SVF_RESOLUTION;
                if center.is_nan() || dem[(y, x)] == f32::MIN {
                    *value = f32::NAN;
                    continue;
                }

                let mut sky = 0.;
                for direction in &directions {
                    // RVT initializes the maximum to -1000 and uses np.fmax,
                    // which ignores a NaN sample when the other value is valid.
                    let mut maximum_slope = -1000_f32;
                    for offset in direction {
                        // np.roll(a, shift)[p] reads a[p - shift]. The padded
                        // array is reflected without repeating its edge cell.
                        let sample_y = reflect_index(y as isize - offset.axis_0_shift, height);
                        let sample_x = reflect_index(x as isize - offset.axis_1_shift, width);
                        let raw_sample = dem[(sample_y, sample_x)];
                        let sample = if raw_sample == f32::MIN {
                            f32::NAN
                        } else {
                            raw_sample / RVT_SVF_RESOLUTION
                        };
                        let slope = (sample - center) / offset.distance;
                        if !slope.is_nan() {
                            maximum_slope = maximum_slope.max(slope);
                        }
                    }
                    let horizon = maximum_slope.atan().max(0.);
                    sky += 1. - horizon.sin();
                }
                *value = sky / RVT_SVF_DIRECTIONS as f32;
            }
        });
    output
}

/// Port of RVT's `horizon_shift_vector`. Sampling the ray every third of a
/// pixel and then rounding/deduplicating is significant for the non-cardinal
/// directions; a simple 8- or 16-neighbour radial walk is not equivalent.
#[cfg(feature = "stream-svf-slope")]
fn rvt_horizon_offsets() -> Vec<Vec<HorizonOffset>> {
    (0..RVT_SVF_DIRECTIONS)
        .map(|direction| {
            let angle = 2. * std::f64::consts::PI * direction as f64 / RVT_SVF_DIRECTIONS as f64;
            let axis_0 = angle.cos();
            let axis_1 = angle.sin();
            let mut shifts = Vec::<(isize, isize)>::new();
            let number_of_samples =
                (RVT_SVF_RADIUS_PIXELS - RVT_SVF_MIN_RADIUS_PIXELS) * RVT_SVF_SAMPLES_PER_PIXEL;
            for sample in 0..=number_of_samples {
                let radius = RVT_SVF_MIN_RADIUS_PIXELS as f64
                    + sample as f64 / RVT_SVF_SAMPLES_PER_PIXEL as f64;
                let shift = (
                    (axis_0 * radius).round_ties_even() as isize,
                    (axis_1 * radius).round_ties_even() as isize,
                );
                if !shifts.contains(&shift) {
                    shifts.push(shift);
                }
            }
            shifts
                .into_iter()
                .map(|(axis_0_shift, axis_1_shift)| HorizonOffset {
                    axis_0_shift,
                    axis_1_shift,
                    distance: (axis_0_shift as f32).hypot(axis_1_shift as f32),
                })
                .collect()
        })
        .collect()
}

/// Index into NumPy's `pad(mode="reflect")` extension of an array. Unlike
/// clamping or symmetric padding, reflection does not repeat the edge cell.
#[cfg(feature = "stream-svf-slope")]
fn reflect_index(index: isize, length: usize) -> usize {
    debug_assert!(length >= 2);
    let period = 2 * (length as isize - 1);
    let reflected = index.rem_euclid(period);
    if reflected < length as isize {
        reflected as usize
    } else {
        (period - reflected) as usize
    }
}

/// Port of WhiteboxTools' projected-coordinate `Slope` operation followed by
/// the training script's degree normalization (`slope / 90`). Whitebox uses a
/// third-order bivariate polynomial over a 5x5 neighbourhood (Florinsky), and
/// substitutes the centre elevation for samples beyond the raster edge.
#[cfg(feature = "stream-svf-slope")]
fn whitebox_normalized_slope(dem: &Dfm<Elevation>) -> Vec<f32> {
    let width = dem.width();
    let height = dem.height();
    let resolution = dem.grid.cell_size_m;
    let mut output = vec![0.; dem.field.len()];
    output
        .par_chunks_mut(width)
        .enumerate()
        .for_each(|(y, row)| {
            for (x, value) in row.iter_mut().enumerate() {
                let center = f64::from(dem[(y, x)]);
                if !center.is_finite() || dem[(y, x)] == f32::MIN {
                    *value = f32::NAN;
                    continue;
                }

                let mut z = [center; 25];
                for offset_y in -2..=2 {
                    for offset_x in -2..=2 {
                        let index = ((offset_y + 2) * 5 + offset_x + 2) as usize;
                        let sample_y = y as isize + offset_y;
                        let sample_x = x as isize + offset_x;
                        if (0..height as isize).contains(&sample_y)
                            && (0..width as isize).contains(&sample_x)
                        {
                            let sample = dem[(sample_y as usize, sample_x as usize)];
                            if sample.is_finite() && sample != f32::MIN {
                                z[index] = f64::from(sample);
                            }
                        }
                    }
                }

                let p = (44. * (z[3] + z[23] - z[1] - z[21])
                    + 31. * (z[0] + z[20] - z[4] - z[24] + 2. * (z[8] + z[18] - z[6] - z[16]))
                    + 17. * (z[14] - z[10] + 4. * (z[13] - z[11]))
                    + 5. * (z[9] + z[19] - z[5] - z[15]))
                    / (420. * resolution);
                let q = (44. * (z[5] + z[9] - z[15] - z[19])
                    + 31. * (z[20] + z[24] - z[0] - z[4] + 2. * (z[6] + z[8] - z[16] - z[18]))
                    + 17. * (z[2] - z[22] + 4. * (z[7] - z[17]))
                    + 5. * (z[1] + z[3] - z[21] - z[23]))
                    / (420. * resolution);

                // Whitebox writes a Float32 degree raster, which the training
                // script then reads and divides by 90 as Float32.
                let slope_degrees = p.hypot(q).atan().to_degrees() as f32;
                *value = slope_degrees / 90.;
            }
        });
    output
}

#[cfg(all(test, feature = "stream-svf-slope"))]
mod tests {
    use super::*;
    use crate::raster::DfmGrid;

    fn reference_terrain() -> Dfm<Elevation> {
        let grid = DfmGrid::new(12, 13, 0.5, geo::coord! { x: 0., y: 6. }).unwrap();
        let mut dem = Dfm::new(grid);
        for y in 0..dem.height() {
            for x in 0..dem.width() {
                let mut elevation = 100_f32;
                elevation += 0.17 * x as f32;
                elevation -= 0.09 * y as f32;
                elevation += 0.013 * ((x * y) % 7) as f32;
                if x == 7 && y >= 3 {
                    elevation += 1.75;
                }
                dem[(y, x)] = elevation;
            }
        }
        dem
    }

    #[test]
    fn rvt_svf_matches_python_reference_including_reflected_edges() {
        let dem = reference_terrain();
        let actual = rvt_sky_view_factor(&dem);
        let reference = [
            ((0, 0), 0.823_123_6),
            ((0, 11), 0.962_268_9),
            ((3, 7), 1.),
            ((6, 6), 0.620_483_64),
            ((6, 7), 0.981_218_1),
            ((12, 0), 0.704_620_1),
            ((12, 11), 0.792_222_26),
        ];
        for ((y, x), expected) in reference {
            assert!(
                (actual[y * dem.width() + x] - expected).abs() < 2e-6,
                "SVF differs from RVT at ({y}, {x}): {} != {expected}",
                actual[y * dem.width() + x]
            );
        }
    }

    #[test]
    fn rvt_svf_is_one_on_a_flat_raster() {
        let mut dem = Dfm::new(DfmGrid::new(12, 13, 0.5, geo::coord! { x: 0., y: 6. }).unwrap());
        dem.field.fill(42.);
        assert!(rvt_sky_view_factor(&dem).iter().all(|&value| value == 1.));
    }

    #[test]
    fn whitebox_slope_matches_normalized_python_reference() {
        let dem = reference_terrain();
        let actual = whitebox_normalized_slope(&dem);
        let reference = [
            ((0, 0), 0.056_696_92),
            ((0, 11), 0.115_947_07),
            ((3, 7), 0.355_730_77),
            ((6, 6), 0.769_035),
            ((6, 7), 0.264_941_13),
            ((12, 0), 0.157_110_02),
            ((12, 11), 0.085_218_005),
        ];
        for ((y, x), expected) in reference {
            assert!(
                (actual[y * dem.width() + x] - expected).abs() < 2e-6,
                "slope differs from Whitebox at ({y}, {x}): {} != {expected}",
                actual[y * dem.width() + x]
            );
        }
    }

    #[test]
    fn whitebox_slope_is_degrees_divided_by_ninety() {
        let grid = DfmGrid::new(9, 9, 0.5, geo::coord! { x: 0., y: 4. }).unwrap();
        let mut dem = Dfm::new(grid);
        for y in 0..dem.height() {
            for x in 0..dem.width() {
                dem[(y, x)] = 10. + 0.4 * x as f32 - 0.3 * y as f32;
            }
        }
        // The per-pixel gradient (0.4, -0.3) is divided by the 0.5 m
        // resolution, giving a physical gradient magnitude of 1.
        let expected = 1_f64.atan() / std::f64::consts::FRAC_PI_2;
        let actual = whitebox_normalized_slope(&dem)[4 * dem.width() + 4];
        assert!((f64::from(actual) - expected).abs() < 2e-6);
    }
}
