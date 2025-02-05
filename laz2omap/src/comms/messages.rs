use crate::{
    params::{FileParams, MapParams},
    DrawableOmap,
};
use std::path::PathBuf;

pub enum Task {
    RegenerateMap,
    Reset,
    SetCrs(SetCrs),
    ShowComponents,
    QueryDropComponents,
    DropComponents,
    Error(String, bool),
    GetOutputCRS,
    DoConnectedComponentAnalysis,
}

pub enum FrontendTask {
    StartProgressBar,
    IncrementProgressBar(f32),
    FinishProgrssBar,
    Log(String),
    SetVariable(Variable),
    TaskComplete(TaskDone),
    CrsModal,
    DelegateTask(Task),
    UpdateMap(Box<DrawableOmap>),
    NextState,
    PrevState,
    BackendError(String, bool),
}

pub enum BackendTask {
    InitializeMapTile(PathBuf),
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
    HeartBeat,
}

pub enum TaskDone {
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
    Paths(Vec<PathBuf>),
    Boundaries(Vec<[walkers::Position; 4]>),
    Home(walkers::Position),
    CrsEPSG(Vec<u16>),
    CrsLessString(usize),
    CrsLessCheckBox(usize),
    ConnectedComponents(Vec<Vec<usize>>),
}
