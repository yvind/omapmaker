use crate::drawing::{DrawableOmap, GuiVariables};
use std::path::PathBuf;

pub enum Task {
    RegenerateMap,
    Reset,
    SetCrs(SetCrs),
    ShowComponents,
    QueryDropComponents,
    DropComponents,
    Error(String),
    GetOutputCRS,
    DoConnectedComponentAnalysis,
}

pub enum FrontEndTask {
    Log(String),
    SetVariable(Variable),
    TaskComplete(TaskDone),
    CrsModal,
    DelegateTask(Task),
    UpdateMap(Box<DrawableOmap>),
    NextState,
    PrevState,
    BackendError(String),
}

pub enum BackendTask {
    ParseCrs(Vec<PathBuf>),
    ConnectedComponentAnalysis(Vec<PathBuf>, Option<Vec<u16>>),
    ConvertCopc(Option<u16>),
    RegenerateMap(Box<GuiVariables>), // boxed to keep the enum variant small
    Reset,
    MakeMap(Box<GuiVariables>),
    HeartBeat,
}

pub enum TaskDone {
    ParseCrs(SetCrs),
    ConnectedComponentAnalysis,
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
    Boundaries(Vec<[walkers::Position; 4]>),
    Home(walkers::Position),
    CrsEPSG(Vec<u16>),
    CrsLessString(usize),
    CrsLessCheckBox(usize),
    ConnectedComponents(Vec<Vec<usize>>),
}
