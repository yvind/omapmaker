use crate::feature_extraction::contract::{InputDescriptor, InvalidPolicy};
use crate::map_gen::pipeline::TileRasters;
use crate::raster::{Dfm, DfmGrid, Elevation};
use rayon::prelude::*;

// These are the parameters used to prepare the rasters on which the embedded
// ditch/stream model was trained. RVT expresses the radius in pixels and the
// supplied training script deliberately passes a resolution of one.
const RVT_SVF_DIRECTIONS: usize = 16;
const RVT_SVF_RADIUS_PIXELS: usize = 10;
const RVT_SVF_MIN_RADIUS_PIXELS: usize = 1;
const RVT_SVF_SAMPLES_PER_PIXEL: usize = 3;
const RVT_SVF_RESOLUTION: f32 = 1.;

pub struct NamedRaster<'a> {
    pub name: &'a str,
    pub grid: &'a DfmGrid,
    pub values: &'a [f32],
}

#[derive(Clone, Debug)]
pub struct NchwInput {
    pub values: Vec<f32>,
    pub shape: [usize; 4],
    pub grid: DfmGrid,
}

pub fn build_input(
    descriptor: &InputDescriptor,
    rasters: &TileRasters,
) -> crate::Result<NchwInput> {
    let model_dem = &rasters.dem;
    let sky_view_factor = descriptor
        .channels
        .contains(&"sky_view_factor")
        .then(|| rvt_sky_view_factor(model_dem));
    let normalized_slope = descriptor
        .channels
        .contains(&"slope")
        .then(|| whitebox_normalized_slope(model_dem));
    let mut sources = Vec::with_capacity(descriptor.channels.len());
    for &name in descriptor.channels {
        let (grid, values): (&DfmGrid, &[f32]) = match name {
            "elevation" | "normalized_elevation" => (&rasters.dem.grid, &rasters.dem.field),
            "slope" => (
                &model_dem.grid,
                normalized_slope.as_deref().expect("slope was requested"),
            ),
            "return_number" => (&rasters.return_number.grid, &rasters.return_number.field),
            "intensity" => (&rasters.intensity.grid, &rasters.intensity.field),
            "last_return" => (&rasters.last_return.grid, &rasters.last_return.field),
            "ground_vegetation" => (
                &rasters.ground_vegetation.grid,
                &rasters.ground_vegetation.field,
            ),
            "low_vegetation" => (&rasters.low_vegetation.grid, &rasters.low_vegetation.field),
            "medium_vegetation" => (
                &rasters.medium_vegetation.grid,
                &rasters.medium_vegetation.field,
            ),
            "high_vegetation" => (
                &rasters.high_vegetation.grid,
                &rasters.high_vegetation.field,
            ),
            "ground_relief_2m" => (
                &rasters.ground_relief_2m.grid,
                &rasters.ground_relief_2m.field,
            ),
            "ground_relief_5m" => (
                &rasters.ground_relief_5m.grid,
                &rasters.ground_relief_5m.field,
            ),
            "hard_object_height" => (
                &rasters.hard_object_height.grid,
                &rasters.hard_object_height.field,
            ),
            "hard_object_confidence" => (
                &rasters.hard_object_confidence.grid,
                &rasters.hard_object_confidence.field,
            ),
            "vegetation_likelihood" => (
                &rasters.vegetation_likelihood.grid,
                &rasters.vegetation_likelihood.field,
            ),
            "filtered_surface" => (
                &rasters.filtered_surface.grid,
                &rasters.filtered_surface.field,
            ),
            "water" => (&rasters.water.grid, &rasters.water.field),
            "canopy_height" => (&rasters.canopy_height.grid, &rasters.canopy_height.field),
            "point_density" => (&rasters.point_density.grid, &rasters.point_density.field),
            "ground_point_density" => (
                &rasters.ground_point_density.grid,
                &rasters.ground_point_density.field,
            ),
            "sky_view_factor" => (
                &model_dem.grid,
                sky_view_factor.as_deref().expect("SVF was requested"),
            ),
            _ => anyhow::bail!("model requests unsupported raster channel {name:?}"),
        };
        sources.push(NamedRaster { name, grid, values });
    }
    build_nchw(descriptor, &sources)
}

pub fn build_nchw(
    descriptor: &InputDescriptor,
    sources: &[NamedRaster<'_>],
) -> crate::Result<NchwInput> {
    anyhow::ensure!(
        sources.len() == descriptor.channels.len(),
        "expected {} input rasters, got {}",
        descriptor.channels.len(),
        sources.len()
    );
    let first = sources
        .first()
        .ok_or_else(|| anyhow::anyhow!("model has no input channels"))?;
    anyhow::ensure!(
        (first.grid.cell_size_m - descriptor.cell_size).abs() <= 1e-9,
        "model expects {} m cells, source has {} m cells",
        descriptor.cell_size,
        first.grid.cell_size_m
    );
    anyhow::ensure!(
        first.grid.width >= descriptor.width && first.grid.height >= descriptor.height,
        "model input {} × {} does not fit source grid {} × {}",
        descriptor.width,
        descriptor.height,
        first.grid.width,
        first.grid.height
    );
    let x_offset = (first.grid.width - descriptor.width) / 2;
    let y_offset = (first.grid.height - descriptor.height) / 2;
    anyhow::ensure!(
        first.grid.width - descriptor.width == x_offset * 2
            && first.grid.height - descriptor.height == y_offset * 2,
        "model input must be a centred crop of the source grid"
    );

    let mut values =
        Vec::with_capacity(descriptor.channels.len() * descriptor.width * descriptor.height);
    for (&channel, source) in descriptor.channels.iter().zip(sources) {
        anyhow::ensure!(
            source.name == channel,
            "input channels are not in manifest order"
        );
        first.grid.ensure_compatible(source.grid)?;
        anyhow::ensure!(
            source.values.len() == source.grid.width * source.grid.height,
            "raster channel {channel:?} has an invalid data length"
        );
        let normalization = descriptor
            .normalization
            .iter()
            .find(|entry| entry.channel == channel)
            .ok_or_else(|| anyhow::anyhow!("channel {channel:?} has no normalization"))?
            .normalization;
        for y in y_offset..y_offset + descriptor.height {
            let row = &source.values[y * source.grid.width + x_offset
                ..y * source.grid.width + x_offset + descriptor.width];
            for &raw in row {
                if !raw.is_finite() || raw == f32::MIN {
                    match descriptor.invalid_policy {
                        InvalidPolicy::RejectTile => {
                            anyhow::bail!("channel {channel:?} contains an invalid raster value")
                        }
                    }
                }
                let value = normalization.apply(raw);
                anyhow::ensure!(
                    value.is_finite(),
                    "normalization produced a non-finite value for channel {channel:?}"
                );
                values.push(value);
            }
        }
    }

    let mut grid = DfmGrid::new(
        descriptor.width,
        descriptor.height,
        descriptor.cell_size,
        first.grid.coord(y_offset, x_offset),
    )?;
    grid.inner.top = first
        .grid
        .inner
        .top
        .saturating_sub(y_offset)
        .min(grid.height);
    grid.inner.bottom = first
        .grid
        .inner
        .bottom
        .saturating_sub(y_offset)
        .min(grid.height);
    grid.inner.left = first
        .grid
        .inner
        .left
        .saturating_sub(x_offset)
        .min(grid.width);
    grid.inner.right = first
        .grid
        .inner
        .right
        .saturating_sub(x_offset)
        .min(grid.width);

    Ok(NchwInput {
        values,
        shape: [
            1,
            descriptor.channels.len(),
            descriptor.height,
            descriptor.width,
        ],
        grid,
    })
}

#[derive(Clone, Copy, Debug)]
struct HorizonOffset {
    axis_0_shift: isize,
    axis_1_shift: isize,
    distance: f32,
}

/// Port of `rvt.vis.sky_view_factor` for the exact parameters in the model's
/// training pipeline: resolution 1, 16 directions, radius 10 and noise 0.
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::feature_extraction::contract::{ChannelNormalization, Normalization};

    static NORMALIZATION: &[ChannelNormalization] = &[
        ChannelNormalization {
            channel: "first",
            normalization: Normalization::MinMax {
                minimum: 0.,
                maximum: 10.,
            },
        },
        ChannelNormalization {
            channel: "second",
            normalization: Normalization::Standard {
                mean: 10.,
                standard_deviation: 2.,
            },
        },
    ];

    fn descriptor() -> InputDescriptor {
        InputDescriptor {
            width: 2,
            height: 2,
            cell_size: 1.,
            halo_cells: 0,
            channels: &["first", "second"],
            normalization: NORMALIZATION,
            invalid_policy: InvalidPolicy::RejectTile,
        }
    }

    #[test]
    fn stacks_channels_in_contiguous_nchw_order() {
        let grid = DfmGrid::new(2, 2, 1., geo::coord! { x: 0., y: 1. }).unwrap();
        let first = [0., 5., 10., 2.5];
        let second = [8., 10., 12., 14.];
        let input = build_nchw(
            &descriptor(),
            &[
                NamedRaster {
                    name: "first",
                    grid: &grid,
                    values: &first,
                },
                NamedRaster {
                    name: "second",
                    grid: &grid,
                    values: &second,
                },
            ],
        )
        .unwrap();
        assert_eq!(input.shape, [1, 2, 2, 2]);
        assert_eq!(input.values, [0., 0.5, 1., 0.25, -1., 0., 1., 2.]);
    }

    #[test]
    fn rejects_grid_mismatch_and_invalid_values() {
        let grid = DfmGrid::new(2, 2, 1., geo::coord! { x: 0., y: 1. }).unwrap();
        let shifted = DfmGrid::new(2, 2, 1., geo::coord! { x: 1., y: 1. }).unwrap();
        let values = [1., 2., 3., 4.];
        assert!(
            build_nchw(
                &descriptor(),
                &[
                    NamedRaster {
                        name: "first",
                        grid: &grid,
                        values: &values
                    },
                    NamedRaster {
                        name: "second",
                        grid: &shifted,
                        values: &values
                    },
                ]
            )
            .is_err()
        );
        let invalid = [1., f32::NAN, 3., 4.];
        assert!(
            build_nchw(
                &descriptor(),
                &[
                    NamedRaster {
                        name: "first",
                        grid: &grid,
                        values: &invalid
                    },
                    NamedRaster {
                        name: "second",
                        grid: &grid,
                        values: &values
                    },
                ]
            )
            .is_err()
        );
    }

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
