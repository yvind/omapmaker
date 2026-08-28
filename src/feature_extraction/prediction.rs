use crate::feature_extraction::contract::ModelDescriptor;
use crate::raster::{Dfm, DfmGrid, ModelPrediction};

#[derive(Clone, Debug)]
pub struct PredictionChannel {
    pub name: &'static str,
    pub raster: Dfm<ModelPrediction>,
}

#[derive(Clone, Debug)]
pub struct PredictionRaster {
    #[allow(dead_code)]
    pub model: &'static ModelDescriptor,
    pub channels: Vec<PredictionChannel>,
}

impl PredictionRaster {
    pub fn from_nchw(
        model: &'static ModelDescriptor,
        grid: DfmGrid,
        shape: [usize; 4],
        values: Vec<f32>,
    ) -> crate::Result<Self> {
        let expected = [
            1,
            model.output.channels.len(),
            model.output.height,
            model.output.width,
        ];
        anyhow::ensure!(
            shape == expected,
            "model output shape {shape:?}, expected {expected:?}"
        );
        anyhow::ensure!(
            values.len() == shape.iter().product::<usize>(),
            "model output data length does not match its shape"
        );
        anyhow::ensure!(
            values.iter().all(|value| value.is_finite()),
            "model output contains non-finite values"
        );

        let channel_len = grid.width * grid.height;
        let channels = model
            .output
            .channels
            .iter()
            .enumerate()
            .map(|(index, &name)| {
                let mut raster = Dfm::new(grid.clone());
                raster
                    .field
                    .copy_from_slice(&values[index * channel_len..(index + 1) * channel_len]);
                PredictionChannel { name, raster }
            })
            .collect();
        Ok(Self { model, channels })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::feature_extraction::contract::{
        Activation, InputDescriptor, InvalidPolicy, OutputDescriptor,
    };

    static MODEL: ModelDescriptor = ModelDescriptor {
        schema_version: 1,
        contract_version: 1,
        id: "test_model",
        name: "Test model",
        revision: "1",
        onnx_sha256: "source",
        manifest_sha256: "manifest",
        input: InputDescriptor {
            width: 2,
            height: 2,
            cell_size: 1.,
            halo_cells: 0,
            channels: &["input"],
            normalization: &[],
            invalid_policy: InvalidPolicy::RejectTile,
        },
        output: OutputDescriptor {
            width: 2,
            height: 2,
            channels: &["first", "second"],
            activation: Activation::Identity,
        },
    };

    #[test]
    fn validates_shape_and_splits_output_channels() {
        let grid = DfmGrid::new(2, 2, 1., geo::coord! { x: 0., y: 1. }).unwrap();
        let prediction = PredictionRaster::from_nchw(
            &MODEL,
            grid.clone(),
            [1, 2, 2, 2],
            vec![0., 1., 2., 3., 4., 5., 6., 7.],
        )
        .unwrap();
        assert_eq!(prediction.channels[1].name, "second");
        assert_eq!(
            prediction.channels[1].raster.field.as_ref(),
            [4., 5., 6., 7.]
        );
        assert!(
            PredictionRaster::from_nchw(&MODEL, grid.clone(), [1, 1, 2, 2], vec![0.; 4]).is_err()
        );
        assert!(
            PredictionRaster::from_nchw(&MODEL, grid, [1, 2, 2, 2], vec![f32::NAN; 8]).is_err()
        );
    }
}
