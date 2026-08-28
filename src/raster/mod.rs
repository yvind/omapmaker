pub mod dfm;
pub mod geotiff;
pub mod grid;
mod hydrology;
pub mod resample;

pub use self::dfm::Dfm;
pub use self::grid::{DfmGrid, DfmPixelBounds};
pub use self::hydrology::{D8Flow, MarshHydrology, accumulate_cross_tile_flow};
#[allow(unused_imports)]
pub use self::resample::MaskRestriction;

pub enum Threshold {
    Upper(f32),
    #[allow(dead_code)]
    Lower(f32),
}

impl Threshold {
    pub fn inner(&self) -> f32 {
        match self {
            Threshold::Upper(t) => *t,
            Threshold::Lower(t) => *t,
        }
    }

    pub fn is_upper(&self) -> bool {
        match self {
            Threshold::Upper(_) => true,
            Threshold::Lower(_) => false,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Elevation;
/// Elevation derived specifically for contour extraction. The canonical
/// [`Elevation`] raster remains untouched.
#[derive(Clone, Copy, Debug)]
pub struct ContourTerrain;
#[derive(Clone, Copy, Debug)]
pub struct AdjustedElevation;
#[derive(Clone, Copy, Debug)]
pub struct TargetElevation;
#[derive(Clone, Copy, Debug)]
pub struct Slope;
#[derive(Clone, Copy, Debug)]
pub struct CliffStrength;
#[derive(Clone, Copy, Debug)]
pub struct ProfileChange;
#[derive(Clone, Copy, Debug)]
pub struct TangentChange;
#[derive(Clone, Copy, Debug)]
pub struct FitConfidence;
#[derive(Clone, Copy, Debug)]
pub struct DirectionConfidence;
#[derive(Clone, Copy, Debug)]
pub struct TerrainSalience;
#[derive(Clone, Copy, Debug)]
pub struct IsolineTangentX;
#[derive(Clone, Copy, Debug)]
pub struct IsolineTangentY;
#[derive(Clone, Copy, Debug)]
pub struct AlignmentConfidence;
#[derive(Clone, Copy, Debug)]
pub struct ContourCost;
#[derive(Clone, Copy, Debug)]
pub struct SmoothnessWeight;
#[derive(Clone, Copy, Debug)]
pub struct VerticalAdjustment;
#[derive(Clone, Copy, Debug)]
pub struct AdjustmentBoundMask;
#[derive(Clone, Copy, Debug)]
pub struct TerrainChange;
#[derive(Clone, Copy, Debug)]
pub struct InterpolationErrorImprovement;
#[derive(Clone, Copy, Debug)]
pub struct Hillshade;
#[derive(Clone, Copy, Debug)]
pub struct Returns;
#[derive(Clone, Copy, Debug)]
pub struct Intensity;
#[derive(Clone, Copy, Debug)]
pub struct HeightAboveGround;
#[derive(Clone, Copy, Debug)]
pub struct HeightAboveGroundMean;
#[derive(Clone, Copy, Debug)]
pub struct HeightAboveGroundMax;
#[derive(Clone, Copy, Debug)]
pub struct ElevatedPointCount;
#[derive(Clone, Copy, Debug)]
pub struct PlanarPointFraction;
#[derive(Clone, Copy, Debug)]
pub struct PlaneResidual;
#[derive(Clone, Copy, Debug)]
pub struct SurfaceNormalX;
#[derive(Clone, Copy, Debug)]
pub struct SurfaceNormalY;
#[derive(Clone, Copy, Debug)]
pub struct SurfaceNormalZ;
#[derive(Clone, Copy, Debug)]
pub struct BuildingProbability;
#[derive(Clone, Copy, Debug)]
pub struct BuildingCandidateId;
#[derive(Clone, Copy, Debug)]
pub struct LastReturn;
#[derive(Clone, Copy, Debug)]
pub struct Ground;
#[derive(Clone, Copy, Debug)]
pub struct LowVegetation;
#[derive(Clone, Copy, Debug)]
pub struct MediumVegetation;
#[derive(Clone, Copy, Debug)]
pub struct HighVegetation;
#[derive(Clone, Copy, Debug)]
pub struct SurfaceObjects;
/// Signed residual from the canonical DEM relative to a 2 m local terrain
/// baseline. Positive values are locally proud of the surrounding terrain.
#[derive(Clone, Copy, Debug)]
pub struct GroundRelief2m;
/// Signed residual from the canonical DEM relative to a 5 m local terrain
/// baseline. Positive values are locally proud of the surrounding terrain.
#[derive(Clone, Copy, Debug)]
pub struct GroundRelief5m;
/// Vegetation-suppressed object height above the canonical DEM, in metres.
#[derive(Clone, Copy, Debug)]
pub struct HardObjectHeight;
/// Observation and surface-coherence confidence for [`HardObjectHeight`].
#[derive(Clone, Copy, Debug)]
pub struct HardObjectConfidence;
/// Likelihood that local elevated returns came from vegetation.
#[derive(Clone, Copy, Debug)]
pub struct VegetationLikelihood;
/// Canonical DEM with accepted hard-object heights added back in.
#[derive(Clone, Copy, Debug)]
pub struct FilteredSurface;
#[derive(Clone, Copy, Debug)]
pub struct Water;
#[derive(Clone, Copy, Debug)]
pub struct Ndvd;
#[derive(Clone, Copy, Debug)]
pub struct PointDensity;
#[derive(Clone, Copy, Debug)]
pub struct GroundPointDensity;
#[derive(Clone, Copy, Debug)]
pub struct HydroCorrected;
#[derive(Clone, Copy, Debug)]
pub struct FloodFill;
#[derive(Clone, Copy, Debug)]
pub struct FlowAccumulation;
#[derive(Clone, Copy, Debug)]
pub struct HeightAboveDrainage;
#[derive(Clone, Copy, Debug)]
pub struct DownslopeDistanceToDrainage;
#[derive(Clone, Copy, Debug)]
pub struct DepressionDepth;
#[derive(Clone, Copy, Debug)]
pub struct WetnessScore;
#[derive(Clone, Copy, Debug)]
pub struct MarshProbability;
#[derive(Clone, Copy, Debug)]
pub struct MarshSupport;
#[derive(Clone, Copy, Debug)]
pub struct MarshReason;
#[derive(Clone, Copy, Debug)]
pub struct MarshMask;
#[derive(Clone, Copy, Debug)]
#[cfg_attr(not(feature = "deep-learning"), allow(dead_code))]
pub struct ModelPrediction;

/// Zero sized marker trait for strict typing on Dfm
pub trait RasterMarker: Copy {}

/// Marker for rasters whose values may be averaged and bilinearly interpolated.
pub trait ContinuousRasterMarker: RasterMarker {}

/// Marker for elevation-like rasters on which terrain smoothing is meaningful.
///
/// Keeping this separate from [`ContinuousRasterMarker`] prevents filters based
/// on surface normals from being applied to intensity, return, probability, or
/// count rasters merely because their values are continuous.
pub trait TerrainRasterMarker: ContinuousRasterMarker {}

/// Marker for categorical zero/nonzero rasters that require an explicit
/// restriction policy.
pub trait MaskRasterMarker: RasterMarker {}

macro_rules! mask_raster_markers {
    ($($marker:ty),+ $(,)?) => {
        $(impl RasterMarker for $marker {})+
        $(impl MaskRasterMarker for $marker {})+
    };
}

mask_raster_markers!(FloodFill, AdjustmentBoundMask, MarshMask);

macro_rules! continuous_raster_markers {
    ($($marker:ty),+ $(,)?) => {
        $(impl RasterMarker for $marker {})+
        $(impl ContinuousRasterMarker for $marker {})+
    };
}

continuous_raster_markers!(
    Elevation,
    ModelPrediction,
    ContourTerrain,
    AdjustedElevation,
    TargetElevation,
    Slope,
    CliffStrength,
    ProfileChange,
    TangentChange,
    FitConfidence,
    DirectionConfidence,
    TerrainSalience,
    IsolineTangentX,
    IsolineTangentY,
    AlignmentConfidence,
    ContourCost,
    SmoothnessWeight,
    VerticalAdjustment,
    TerrainChange,
    InterpolationErrorImprovement,
    Hillshade,
    HydroCorrected,
    Returns,
    Intensity,
    HeightAboveGround,
    HeightAboveGroundMean,
    HeightAboveGroundMax,
    ElevatedPointCount,
    PlanarPointFraction,
    PlaneResidual,
    SurfaceNormalX,
    SurfaceNormalY,
    SurfaceNormalZ,
    BuildingProbability,
    LastReturn,
    Ground,
    LowVegetation,
    MediumVegetation,
    HighVegetation,
    SurfaceObjects,
    GroundRelief2m,
    GroundRelief5m,
    HardObjectHeight,
    HardObjectConfidence,
    VegetationLikelihood,
    FilteredSurface,
    Water,
    Ndvd,
    PointDensity,
    GroundPointDensity,
    FlowAccumulation,
    HeightAboveDrainage,
    DownslopeDistanceToDrainage,
    DepressionDepth,
    WetnessScore,
    MarshProbability,
    MarshSupport
);

// Candidate IDs are categorical and must never be averaged during resampling.
impl RasterMarker for BuildingCandidateId {}
// Reason codes are categorical and must never be averaged during resampling.
impl RasterMarker for MarshReason {}

impl TerrainRasterMarker for Elevation {}
impl TerrainRasterMarker for ContourTerrain {}
impl TerrainRasterMarker for FilteredSurface {}
