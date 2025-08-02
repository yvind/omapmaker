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
    TileSelectedFile(PathBuf, Option<u16>),
    InitializeMapTile(PathBuf, Neighborhood, LidarStats),
    ParseCrs(Vec<PathBuf>),
    MapSpatialLidarRelations(Vec<PathBuf>, Option<Vec<u16>>),
    ConvertCopc(
        Vec<PathBuf>,
        Vec<u16>,
        Option<u16>,
        usize,
        Vec<[walkers::Position; 4]>,
        geo::LineString,
    ),
    RegenerateMap(Box<MapParameters>), // boxed to keep the enum variant small
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
    Home(walkers::Position),
    CrsEPSG(Vec<u16>),
    CrsLessString(usize),
    CrsLessCheckBox(usize),
    ConnectedComponents(Vec<Vec<usize>>),
    ContourScore((f32, f32)),
    Stats(LidarStats),
}
