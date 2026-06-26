use proj_core::CrsDef;

use crate::{
    drawable::DrawableOmap,
    gui::modals::OmapModal,
    neighbors::Neighborhood,
    parameters::{FileParameters, MapParameters},
    statistics::LidarStats,
};
use std::path::PathBuf;

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
    TileSelectedFile(PathBuf, Option<CrsDef>),
    InitializeMapTile(PathBuf, Neighborhood, LidarStats),
    ParseCrs(Vec<PathBuf>),
    MapSpatialLidarRelations(Vec<PathBuf>, Option<Vec<Option<CrsDef>>>),
    ConvertCopc(
        Vec<PathBuf>,
        Vec<Option<CrsDef>>,
        Option<CrsDef>,
        PathBuf,
        Vec<[walkers::Position; 4]>,
        geo::LineString,
        bool,
    ),
    RegenerateMap(Box<MapParameters>, RegenerationScope), // boxed to keep the enum variant small
    Reset,
    MakeMap(
        Box<MapParameters>,
        Box<FileParameters>,
        geo::LineString,
        LidarStats,
    ),
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
    TileSelectedFile,
    InitializeMapTile,
    ParseCrs(SetCrs),
    MapSpatialLidarRelations,
    DropComponents,
    ConvertCopc,
    OutputCrs,
    RegenerateMap,
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
    MapTile(Box<DrawableOmap>),
    TileBounds(Vec<[walkers::Position; 4]>),
    TileNeighbors(Vec<Neighborhood>),
    Paths(Vec<PathBuf>),
    Boundaries(Vec<[walkers::Position; 4]>),
    BoundaryAreas(Vec<f64>),
    Home(walkers::Position),
    CrsDefs(Vec<Option<CrsDef>>),
    CrsLessString(usize),
    CrsLessCheckBox(usize),
    ConnectedComponents(Vec<Vec<usize>>),
    ContourScore((f32, f32)),
    Stats(LidarStats),
    SingleCopcPath(PathBuf),
}
