use crate::inference::contract::{InputDescriptor, InvalidPolicy};
use crate::raster::DfmGrid;

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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::inference::contract::{ChannelNormalization, Normalization};

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
}
