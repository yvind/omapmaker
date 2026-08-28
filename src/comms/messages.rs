use proj_core::CrsDef;

use crate::{
    drawable::DrawableOmap,
    gui::modals::OmapModal,
    parameters::{FileParameters, MapParameters},
    statistics::LidarStats,
};
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

pub type JobId = u64;

#[derive(Clone, Debug, Default)]
pub struct CancellationToken(Arc<AtomicBool>);

impl CancellationToken {
    pub fn cancel(&self) {
        self.0.store(true, Ordering::Release);
    }

    pub fn is_cancelled(&self) -> bool {
        self.0.load(Ordering::Acquire)
    }

    pub fn check(&self) -> crate::Result<()> {
        anyhow::ensure!(!self.is_cancelled(), "preview job was cancelled");
        Ok(())
    }
}

/// Notifications produced by backend work and applied by the frontend.
///
/// These messages carry status or data only; they never specify a modal, state
/// transition, or follow-up operation. Workflow decisions belong to [`Task`]
/// and are made by `OmapMaker::start_task`.
pub enum FrontendTask {
    ProgressBar(ProgressBar),
    Log(String),
    UpdateVariable(Variable),
    TaskComplete(TaskComplete),
    Error(String, bool),
}

/// Work requests executed by the backend worker.
pub enum BackendTask {
    ClearParams,
    SetWorkerThreads(usize),
    InitializeMapTile(Box<InitializeMapTileTask>),
    ParseCrs(Vec<PathBuf>),
    MapSpatialLidarRelations(Vec<PathBuf>, Option<Vec<Option<CrsDef>>>),
    ConvertCopc(Box<ConvertCopcTask>),
    RegenerateMap(
        JobId,
        Box<MapParameters>,
        RegenerationScope,
        CancellationToken,
    ),
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
    pub in_crs: Vec<Option<CrsDef>>,
    pub out_crs: Option<CrsDef>,
    pub save_location: PathBuf,
    pub bounds: Vec<[walkers::Position; 4]>,
    pub polygon: geo::LineString,
    pub write_single_copc: bool,
    pub budget_gb: u8,
}

pub struct MakeMapTask {
    pub map_params: MapParameters,
    pub file_params: FileParameters,
    pub polygon_filter: geo::LineString,
    pub stats: LidarStats,
}

/// Frontend workflow commands handled by `OmapMaker::start_task`.
///
/// GUI events should send the narrowest applicable variant and let
/// `start_task` gather application state, construct backend jobs, and decide
/// subsequent workflow steps.
pub enum Task {
    FrontendTask(FrontendTask),
    SetWorkerThreads,
    ParseCrs(Vec<PathBuf>),
    SetCrs(SetCrs),
    GetOutputCRS,
    OutputCrsSelected,
    DoConnectedComponentAnalysis,
    QueryDropComponents,
    ShowComponents,
    DropComponents,
    ConvertCopc,
    InitializeMapTile,
    RegenerateMap(RegenerationScope),
    MakeMap,
    OpenModal(OmapModal),
    NextState,
    PrevState,
    Reset,
}

pub enum ProgressBar {
    Start,
    Finish,
    Inc(f32),
}

#[derive(Clone, Copy, Debug)]
pub enum RegenerationScope {
    Changed,
    Section(MapPreviewSection),
}

/// Preview sections in the order they become available in the adjustment flow.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum MapPreviewSection {
    Contours,
    Openness,
    Vegetation,
    Buildings,
    Cliffs,
    Water,
    Marsh,
    Streams,
    Intensity,
}

pub enum TaskComplete {
    InitializeMapTile,
    ParseCrs,
    MapSpatialLidarRelations,
    ConvertCopc,
    RegenerateMap(JobId),
    Reset,
    MakeMap,
}

pub enum SetCrs {
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cancellation_is_shared_between_job_participants() {
        let frontend = CancellationToken::default();
        let backend = frontend.clone();
        assert!(backend.check().is_ok());
        frontend.cancel();
        assert!(backend.is_cancelled());
        assert!(backend.check().is_err());
    }
}
