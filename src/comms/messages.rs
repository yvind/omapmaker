use proj_core::CrsDef;

use crate::{
    drawable::DrawableOmap,
    gui::modals::OmapModal,
    parameters::{FileParameters, MapParameters},
    statistics::LidarStats,
};
use std::path::PathBuf;

pub type JobId = u64;

pub enum FrontendTask {
    ProgressBar(ProgressBar),
    Log(String),
    UpdateVariable(Variable),
    DelegateTask(Task),
    TaskComplete(TaskDone),
    OpenModal(OmapModal),
    NextState,
    PrevState,
    Error(String, bool),
}

pub enum BackendTask {
    ClearParams,
    SetWorkerThreads(usize),
    InitializeMapTile(Box<InitializeMapTileTask>),
    ParseCrs(Vec<PathBuf>),
    MapSpatialLidarRelations(Vec<PathBuf>, Option<Vec<Option<CrsDef>>>),
    ConvertCopc(Box<ConvertCopcTask>),
    RegenerateMap(JobId, Box<MapParameters>, RegenerationScope),
    Reset,
    MakeMap(Box<MakeMapTask>),
}

pub struct InitializeMapTileTask {
    pub paths: Vec<PathBuf>,
    pub test_area: geo::Rect,
    pub stats: LidarStats,
}

pub struct ConvertCopcTask {
    pub paths: Vec<PathBuf>,
    pub in_epsg: Vec<Option<CrsDef>>,
    pub out_epsg: Option<CrsDef>,
    pub save_location: PathBuf,
    pub bounds: Vec<[walkers::Position; 4]>,
    pub polygon: geo::LineString,
    pub write_single_copc: bool,
}

pub struct MakeMapTask {
    pub map_params: MapParameters,
    pub file_params: FileParameters,
    pub polygon_filter: geo::LineString,
    pub stats: LidarStats,
}

pub enum Task {
    RegenerateMap,
    Reset,
    SetCrs(SetCrs),
    ShowComponents,
    QueryDropComponents,
    DropComponents,
    GetOutputCRS,
    DoConnectedComponentAnalysis,
}

pub enum ProgressBar {
    Start,
    Finish,
    Inc(f32),
}

pub enum RegenerationScope {
    Changed,
    Section(MapPreviewSection),
}

pub enum MapPreviewSection {
    Openness,
    Vegetation,
    Cliffs,
    Intensity,
}

pub enum TaskDone {
    InitializeMapTile,
    ParseCrs(SetCrs),
    MapSpatialLidarRelations,
    DropComponents,
    ConvertCopc,
    OutputCrs,
    RegenerateMap(JobId),
    Reset,
    MakeMap,
}

pub enum SetCrs {
    Auto,
    SetAllEpsg,
    SetEachCrs,
    Local,
    Default,
    DropAll,
}

pub enum Variable {
    MapTile(JobId, Box<DrawableOmap>),
    Paths(Vec<PathBuf>),
    Boundaries(Vec<[walkers::Position; 4]>),
    BoundaryAreas(Vec<f64>),
    Home(walkers::Position),
    CrsDefs(Vec<Option<CrsDef>>),
    CrsLessString(usize),
    CrsLessCheckBox(usize),
    ConnectedComponents(Vec<Vec<usize>>),
    ContourScore(JobId, (f32, f32)),
    Stats(Box<LidarStats>),
    SingleCopcPath(PathBuf),
}
