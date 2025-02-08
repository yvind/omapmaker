use crate::{
    params::{FileParams, MapParams},
    DrawableOmap,
};
use std::path::PathBuf;

pub enum FrontendTask {
    ProgressBar(ProgressBar),
    Log(String),
    UpdateVariable(Variable),
    DelegateTask(Task),
    TaskComplete(TaskDone),
    OpenCrsModal,
    NextState,
    PrevState,
    Error(String, bool),
}

pub enum BackendTask {
    TileSelectedFile(PathBuf, Option<u16>),
    InitializeMapTile(PathBuf, [Option<usize>; 9]),
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
    RegenerateMap(Box<MapParams>), // boxed to keep the enum variant small
    Reset,
    MakeMap(Box<MapParams>, Box<FileParams>, geo::LineString),
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
    TileNeighbours(Vec<[Option<usize>; 9]>),
    Paths(Vec<PathBuf>),
    Boundaries(Vec<[walkers::Position; 4]>),
    Home(walkers::Position),
    CrsEPSG(Vec<u16>),
    CrsLessString(usize),
    CrsLessCheckBox(usize),
    ConnectedComponents(Vec<Vec<usize>>),
}
