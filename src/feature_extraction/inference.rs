use super::contract::{Activation, ModelDescriptor};
use super::input::NchwInput;
use super::prediction::PredictionRaster;
use crate::comms::messages::CancellationToken;
use burn::tensor::{Device, DeviceKind, Tensor, TensorData, activation};
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::Mutex;

mod compiled_models {
    include!(concat!(env!("OUT_DIR"), "/models/registry.rs"));
}

pub struct InferenceLane {
    descriptor: &'static ModelDescriptor,
    device: Device,
    model: Mutex<compiled_models::CompiledModel>,
}

impl InferenceLane {
    pub fn new(model_id: &str) -> crate::Result<Self> {
        let descriptor = descriptor(model_id)
            .ok_or_else(|| anyhow::anyhow!("unknown compiled model {model_id:?}"))?;
        let device = Device::wgpu(DeviceKind::DefaultDevice);
        let model = catch_unwind(AssertUnwindSafe(|| {
            compiled_models::CompiledModel::load(descriptor.id, &device)
        }))
        .map_err(|panic| anyhow::anyhow!("WGPU initialization failed: {}", panic_message(panic)))?
        .map_err(|error| anyhow::anyhow!("cannot load embedded model on WGPU: {error}"))?;
        log::info!(
            "Loaded embedded model {:?} on WGPU ({:?})",
            descriptor.id,
            device
        );
        Ok(Self {
            descriptor,
            device,
            model: Mutex::new(model),
        })
    }

    pub fn descriptor(&self) -> &'static ModelDescriptor {
        self.descriptor
    }

    pub fn predict(
        &self,
        input: NchwInput,
        cancellation: &CancellationToken,
    ) -> crate::Result<PredictionRaster> {
        cancellation.check()?;
        anyhow::ensure!(
            input.shape
                == [
                    1,
                    self.descriptor.input.channels.len(),
                    self.descriptor.input.height,
                    self.descriptor.input.width,
                ],
            "input tensor shape does not match model {:?}",
            self.descriptor.id
        );
        cancellation.check()?;
        let model = self.model.lock().expect("inference lane poisoned");
        let result = catch_unwind(AssertUnwindSafe(|| {
            let tensor =
                Tensor::<4>::from_data(TensorData::new(input.values, input.shape), &self.device);
            let output = model.forward(tensor);
            let output = match self.descriptor.output.activation {
                Activation::Identity => output,
                Activation::Sigmoid => activation::sigmoid(output),
                Activation::Softmax => activation::softmax(output, 1),
            };
            let shape = output.dims();
            let values = output
                .into_data()
                .try_to_vec::<f32>()
                .map_err(|error| anyhow::anyhow!("cannot read model output: {error}"))?;
            Ok::<_, anyhow::Error>((shape, values))
        }))
        .map_err(|panic| anyhow::anyhow!("model forward failed: {}", panic_message(panic)))??;
        cancellation.check()?;
        PredictionRaster::from_nchw(self.descriptor, input.grid, result.0, result.1)
    }
}

pub fn descriptors() -> &'static [ModelDescriptor] {
    debug_assert!(
        compiled_models::MODELS
            .iter()
            .all(|model| model.schema_version == super::contract::MANIFEST_SCHEMA_VERSION)
    );
    compiled_models::MODELS
}

fn descriptor(id: &str) -> Option<&'static ModelDescriptor> {
    descriptors().iter().find(|descriptor| descriptor.id == id)
}

fn panic_message(panic: Box<dyn std::any::Any + Send>) -> String {
    if let Some(message) = panic.downcast_ref::<String>() {
        message.clone()
    } else if let Some(message) = panic.downcast_ref::<&str>() {
        (*message).to_owned()
    } else {
        "unknown backend panic".to_owned()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::feature_extraction::input::NchwInput;
    use crate::raster::DfmGrid;

    #[test]
    fn compiled_model_contracts_match_generated_forwards() {
        for descriptor in descriptors() {
            let lane = InferenceLane::new(descriptor.id).unwrap();
            let input = NchwInput {
                values: vec![
                    0.;
                    descriptor.input.channels.len()
                        * descriptor.input.width
                        * descriptor.input.height
                ],
                shape: [
                    1,
                    descriptor.input.channels.len(),
                    descriptor.input.height,
                    descriptor.input.width,
                ],
                grid: DfmGrid::new(
                    descriptor.input.width,
                    descriptor.input.height,
                    descriptor.input.cell_size,
                    geo::coord! { x: 0., y: 0. },
                )
                .unwrap(),
            };
            let prediction = lane.predict(input, &CancellationToken::default()).unwrap();
            assert_eq!(prediction.channels.len(), descriptor.output.channels.len());
            assert!(
                prediction
                    .channels
                    .iter()
                    .flat_map(|channel| channel.raster.field.iter())
                    .all(|value| value.is_finite())
            );
        }
    }
}
