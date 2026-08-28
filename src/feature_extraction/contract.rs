pub const MANIFEST_SCHEMA_VERSION: u32 = 1;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[allow(dead_code)]
pub enum Activation {
    Identity,
    Sigmoid,
    Softmax,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InvalidPolicy {
    RejectTile,
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[allow(dead_code)]
pub enum Normalization {
    MinMax { minimum: f32, maximum: f32 },
    Standard { mean: f32, standard_deviation: f32 },
}

impl Normalization {
    pub fn apply(self, value: f32) -> f32 {
        match self {
            Self::MinMax { minimum, maximum } => (value - minimum) / (maximum - minimum),
            Self::Standard {
                mean,
                standard_deviation,
            } => (value - mean) / standard_deviation,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ChannelNormalization {
    pub channel: &'static str,
    pub normalization: Normalization,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InputDescriptor {
    pub width: usize,
    pub height: usize,
    pub cell_size: f64,
    pub halo_cells: usize,
    pub channels: &'static [&'static str],
    pub normalization: &'static [ChannelNormalization],
    pub invalid_policy: InvalidPolicy,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OutputDescriptor {
    pub width: usize,
    pub height: usize,
    pub channels: &'static [&'static str],
    pub activation: Activation,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ModelDescriptor {
    pub schema_version: u32,
    pub contract_version: u32,
    pub id: &'static str,
    pub name: &'static str,
    pub revision: &'static str,
    pub onnx_sha256: &'static str,
    pub manifest_sha256: &'static str,
    pub input: InputDescriptor,
    pub output: OutputDescriptor,
}
