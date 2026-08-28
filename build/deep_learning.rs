use burn_onnx::{LoadStrategy, ModelGen};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::fmt::Write as _;
use std::fs;
use std::path::{Path, PathBuf};

const CATALOG: &str = "onnx";
const SCHEMA_VERSION: u32 = 1;

#[derive(Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Manifest {
    schema_version: u32,
    id: String,
    name: String,
    #[serde(default = "default_revision")]
    revision: String,
    #[serde(default = "default_contract_version")]
    contract_version: u32,
    input: Input,
    output: Output,
}

#[derive(Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Input {
    layout: String,
    dtype: String,
    width: usize,
    height: usize,
    cell_size: f64,
    halo_cells: usize,
    channels: Vec<String>,
    invalid_policy: InvalidPolicy,
    normalization: Vec<Normalization>,
}

#[derive(Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Output {
    layout: String,
    dtype: String,
    width: usize,
    height: usize,
    channels: Vec<String>,
    activation: Activation,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
enum InvalidPolicy {
    RejectTile,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
enum Activation {
    Identity,
    Sigmoid,
    Softmax,
}

#[derive(Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Normalization {
    channel: String,
    kind: NormalizationKind,
    #[serde(skip_serializing_if = "Option::is_none")]
    minimum: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    maximum: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    mean: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    standard_deviation: Option<f32>,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
enum NormalizationKind {
    MinMax,
    Standard,
}

struct CatalogModel {
    manifest: Manifest,
    onnx_hash: String,
    manifest_hash: String,
}

fn default_revision() -> String {
    "1".into()
}

const fn default_contract_version() -> u32 {
    1
}

pub fn run() -> Result<(), String> {
    println!("cargo:rerun-if-changed={CATALOG}");
    let catalog = Path::new(CATALOG);
    let mut directories = fs::read_dir(catalog)
        .map_err(|error| format!("cannot read {}: {error}", catalog.display()))?
        .filter_map(|entry| match entry {
            Ok(entry) if entry.file_type().is_ok_and(|kind| kind.is_dir()) => {
                Some(Ok(entry.path()))
            }
            Ok(_) => None,
            Err(error) => Some(Err(error.to_string())),
        })
        .collect::<Result<Vec<_>, _>>()?;
    directories.sort();
    if directories.is_empty() {
        return Err(format!("{CATALOG} contains no compiled models"));
    }

    let mut ids = BTreeSet::new();
    let mut models = Vec::with_capacity(directories.len());
    for directory in directories {
        let id = directory
            .file_name()
            .and_then(|name| name.to_str())
            .ok_or_else(|| {
                format!(
                    "model directory is not valid UTF-8: {}",
                    directory.display()
                )
            })?;
        validate_id(id)?;
        if !ids.insert(id.to_owned()) {
            return Err(format!("duplicate model ID {id:?}"));
        }

        let onnx_path = directory.join("model.onnx");
        let manifest_path = directory.join("model.toml");
        require_catalog_files(&directory, &onnx_path, &manifest_path)?;
        println!("cargo:rerun-if-changed={}", onnx_path.display());
        println!("cargo:rerun-if-changed={}", manifest_path.display());

        let source = fs::read_to_string(&manifest_path)
            .map_err(|error| format!("cannot read {}: {error}", manifest_path.display()))?;
        let manifest: Manifest = toml::from_str(&source)
            .map_err(|error| format!("invalid {}: {error}", manifest_path.display()))?;
        validate_manifest(id, &manifest)?;
        let normalized = toml::to_string(&manifest)
            .map_err(|error| format!("cannot normalize {}: {error}", manifest_path.display()))?;
        let onnx = fs::read(&onnx_path)
            .map_err(|error| format!("cannot read {}: {error}", onnx_path.display()))?;

        ModelGen::new()
            .input(path_str(&onnx_path)?)
            .out_dir(&format!("models/{id}"))
            .load_strategy(LoadStrategy::Embedded)
            .run_from_script();

        models.push(CatalogModel {
            manifest,
            onnx_hash: hash(&onnx),
            manifest_hash: hash(normalized.as_bytes()),
        });
    }

    let out_dir = PathBuf::from(
        std::env::var_os("OUT_DIR").ok_or_else(|| "Cargo did not set OUT_DIR".to_owned())?,
    )
    .join("models");
    fs::create_dir_all(&out_dir)
        .map_err(|error| format!("cannot create {}: {error}", out_dir.display()))?;
    fs::write(out_dir.join("registry.rs"), registry(&models))
        .map_err(|error| format!("cannot write generated registry: {error}"))?;
    Ok(())
}

fn path_str(path: &Path) -> Result<&str, String> {
    path.to_str()
        .ok_or_else(|| format!("path is not valid UTF-8: {}", path.display()))
}

fn require_catalog_files(directory: &Path, onnx: &Path, manifest: &Path) -> Result<(), String> {
    if !onnx.is_file() || !manifest.is_file() {
        return Err(format!(
            "{} must contain model.onnx and model.toml",
            directory.display()
        ));
    }
    for entry in fs::read_dir(directory).map_err(|error| error.to_string())? {
        let path = entry.map_err(|error| error.to_string())?.path();
        if path.is_file()
            && matches!(
                path.extension().and_then(|value| value.to_str()),
                Some("onnx" | "toml")
            )
            && path != onnx
            && path != manifest
        {
            return Err(format!(
                "{} contains an unexpected ONNX or manifest file",
                directory.display()
            ));
        }
    }
    Ok(())
}

fn validate_id(id: &str) -> Result<(), String> {
    let mut chars = id.chars();
    let valid = chars.next().is_some_and(|first| first.is_ascii_lowercase())
        && chars.all(|character| {
            character.is_ascii_lowercase() || character.is_ascii_digit() || character == '_'
        })
        && !id.ends_with('_')
        && !id.contains("__")
        && !RUST_KEYWORDS.contains(&id);
    if valid {
        Ok(())
    } else {
        Err(format!(
            "invalid model ID {id:?}; expected an ASCII snake_case Rust identifier"
        ))
    }
}

fn validate_manifest(directory_id: &str, manifest: &Manifest) -> Result<(), String> {
    if manifest.schema_version != SCHEMA_VERSION {
        return Err(format!(
            "model {directory_id}: unsupported schema version {} (expected {SCHEMA_VERSION})",
            manifest.schema_version
        ));
    }
    if manifest.id != directory_id {
        return Err(format!(
            "model {directory_id}: manifest ID {:?} does not match its directory",
            manifest.id
        ));
    }
    if manifest.name.trim().is_empty() || manifest.revision.trim().is_empty() {
        return Err(format!(
            "model {directory_id}: name and revision must not be empty"
        ));
    }
    if manifest.contract_version == 0 {
        return Err(format!(
            "model {directory_id}: contract_version must be positive"
        ));
    }
    if manifest.input.layout != "nchw"
        || manifest.output.layout != "nchw"
        || manifest.input.dtype != "f32"
        || manifest.output.dtype != "f32"
    {
        return Err(format!(
            "model {directory_id}: only NCHW f32 tensors are supported"
        ));
    }
    if manifest.input.width == 0
        || manifest.input.height == 0
        || manifest.output.width != manifest.input.width
        || manifest.output.height != manifest.input.height
    {
        return Err(format!(
            "model {directory_id}: input and output need equal, positive spatial dimensions"
        ));
    }
    if !manifest.input.cell_size.is_finite() || manifest.input.cell_size <= 0. {
        return Err(format!(
            "model {directory_id}: cell_size must be positive and finite"
        ));
    }
    if manifest.input.halo_cells * 2 >= manifest.input.width.min(manifest.input.height) {
        return Err(format!(
            "model {directory_id}: halo consumes the entire tensor"
        ));
    }
    validate_channels(directory_id, "input", &manifest.input.channels)?;
    validate_channels(directory_id, "output", &manifest.output.channels)?;

    let declared = manifest.input.channels.iter().collect::<BTreeSet<_>>();
    let normalized = manifest
        .input
        .normalization
        .iter()
        .map(|normalization| &normalization.channel)
        .collect::<BTreeSet<_>>();
    if declared.len() != manifest.input.normalization.len() || declared != normalized {
        return Err(format!(
            "model {directory_id}: every input channel needs exactly one normalization entry"
        ));
    }
    for normalization in &manifest.input.normalization {
        let valid = match normalization.kind {
            NormalizationKind::MinMax => {
                matches!((normalization.minimum, normalization.maximum), (Some(min), Some(max)) if min.is_finite() && max.is_finite() && max > min)
                    && normalization.mean.is_none()
                    && normalization.standard_deviation.is_none()
            }
            NormalizationKind::Standard => {
                matches!((normalization.mean, normalization.standard_deviation), (Some(mean), Some(deviation)) if mean.is_finite() && deviation.is_finite() && deviation > 0.)
                    && normalization.minimum.is_none()
                    && normalization.maximum.is_none()
            }
        };
        if !valid {
            return Err(format!(
                "model {directory_id}: invalid {:?} normalization for {:?}",
                normalization.kind, normalization.channel
            ));
        }
    }
    Ok(())
}

fn validate_channels(model: &str, kind: &str, channels: &[String]) -> Result<(), String> {
    if channels.is_empty() || channels.iter().any(|channel| channel.trim().is_empty()) {
        return Err(format!("model {model}: {kind} channels must not be empty"));
    }
    if channels.iter().collect::<BTreeSet<_>>().len() != channels.len() {
        return Err(format!("model {model}: duplicate {kind} channel"));
    }
    Ok(())
}

fn hash(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    digest.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn registry(models: &[CatalogModel]) -> String {
    let mut code = String::from(
        "// @generated by build.rs; do not edit.\nuse burn::tensor::{Device, Tensor};\n\n",
    );
    for model in models {
        let id = &model.manifest.id;
        writeln!(
            code,
            "#[allow(dead_code)] pub mod {id} {{ include!(concat!(env!(\"OUT_DIR\"), \"/models/{id}/model.rs\")); }}"
        )
        .unwrap();
    }

    code.push_str("\npub enum CompiledModel {\n");
    for model in models {
        let id = &model.manifest.id;
        let variant = rust_variant(id);
        writeln!(code, "    {variant}({id}::Model),").unwrap();
    }
    code.push_str("}\n\nimpl CompiledModel {\n    pub fn load(id: &str, device: &Device) -> Result<Self, String> {\n        match id {\n");
    for model in models {
        let id = &model.manifest.id;
        let variant = rust_variant(id);
        writeln!(
            code,
            "            {id:?} => Ok(Self::{variant}({id}::Model::from_embedded(device))),"
        )
        .unwrap();
    }
    code.push_str("            _ => Err(format!(\"unknown compiled model {id:?}\")),\n        }\n    }\n\n    pub fn forward(&self, input: Tensor<4>) -> Tensor<4> {\n        match self {\n");
    for model in models {
        let id = &model.manifest.id;
        let variant = rust_variant(id);
        writeln!(
            code,
            "            Self::{variant}(model) => model.forward(input),"
        )
        .unwrap();
    }
    code.push_str("        }\n    }\n}\n\n");

    for model in models {
        let manifest = &model.manifest;
        let upper = manifest.id.to_ascii_uppercase();
        writeln!(code, "static {upper}_NORMALIZATION: &[crate::feature_extraction::contract::ChannelNormalization] = &[").unwrap();
        for normalization in &manifest.input.normalization {
            let kind = match normalization.kind {
                NormalizationKind::MinMax => format!(
                    "crate::feature_extraction::contract::Normalization::MinMax {{ minimum: {:?}, maximum: {:?} }}",
                    normalization.minimum.unwrap(),
                    normalization.maximum.unwrap()
                ),
                NormalizationKind::Standard => format!(
                    "crate::feature_extraction::contract::Normalization::Standard {{ mean: {:?}, standard_deviation: {:?} }}",
                    normalization.mean.unwrap(),
                    normalization.standard_deviation.unwrap()
                ),
            };
            writeln!(code, "    crate::feature_extraction::contract::ChannelNormalization {{ channel: {:?}, normalization: {kind} }},", normalization.channel).unwrap();
        }
        code.push_str("];\n");
    }

    code.push_str(
        "\npub static MODELS: &[crate::feature_extraction::contract::ModelDescriptor] = &[\n",
    );
    for model in models {
        let manifest = &model.manifest;
        let upper = manifest.id.to_ascii_uppercase();
        let activation = match manifest.output.activation {
            Activation::Identity => "Identity",
            Activation::Sigmoid => "Sigmoid",
            Activation::Softmax => "Softmax",
        };
        writeln!(
            code,
            "    crate::feature_extraction::contract::ModelDescriptor {{"
        )
        .unwrap();
        writeln!(code, "        schema_version: {}, contract_version: {}, id: {:?}, name: {:?}, revision: {:?},", manifest.schema_version, manifest.contract_version, manifest.id, manifest.name, manifest.revision).unwrap();
        writeln!(
            code,
            "        onnx_sha256: {:?}, manifest_sha256: {:?},",
            model.onnx_hash, model.manifest_hash
        )
        .unwrap();
        writeln!(code, "        input: crate::feature_extraction::contract::InputDescriptor {{ width: {}, height: {}, cell_size: {:?}, halo_cells: {}, channels: &{:?}, normalization: {upper}_NORMALIZATION, invalid_policy: crate::feature_extraction::contract::InvalidPolicy::RejectTile }},", manifest.input.width, manifest.input.height, manifest.input.cell_size, manifest.input.halo_cells, manifest.input.channels).unwrap();
        writeln!(code, "        output: crate::feature_extraction::contract::OutputDescriptor {{ width: {}, height: {}, channels: &{:?}, activation: crate::feature_extraction::contract::Activation::{activation} }},", manifest.output.width, manifest.output.height, manifest.output.channels).unwrap();
        code.push_str("    },\n");
    }
    code.push_str("];\n");
    code
}

fn rust_variant(id: &str) -> String {
    id.split('_')
        .map(|part| {
            let mut chars = part.chars();
            chars
                .next()
                .map(|first| first.to_ascii_uppercase().to_string() + chars.as_str())
                .unwrap_or_default()
        })
        .collect()
}

const RUST_KEYWORDS: &[&str] = &[
    "as", "break", "const", "continue", "crate", "else", "enum", "extern", "false", "fn", "for",
    "if", "impl", "in", "let", "loop", "match", "mod", "move", "mut", "pub", "ref", "return",
    "self", "Self", "static", "struct", "super", "trait", "true", "type", "unsafe", "use", "where",
    "while", "async", "await", "dyn", "abstract", "become", "box", "do", "final", "macro",
    "override", "priv", "typeof", "unsized", "virtual", "yield", "try",
];
