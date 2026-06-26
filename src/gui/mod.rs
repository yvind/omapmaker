pub mod gui_variables;
pub mod main_panel;
pub mod map_controls;
pub mod map_plugins;
pub mod modals;
pub mod side_panel;
pub mod terminal_like;

pub use gui_variables::GuiVariables;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ProcessStage {
    AdjustContours,
    AdjustOpenness,
    AdjustVegetation,
    AdjustCliffs,
    AdjustIntensity,
    CheckLidar,
    ShowComponents,
    ChooseSquare,
    ChooseSubTile,
    ConvertingCOPC,
    DrawPolygon,
    ExportDone,
    MakeMap,
    Welcome,
}

impl ProcessStage {
    pub fn next(&mut self) {
        match self {
            ProcessStage::Welcome => *self = ProcessStage::CheckLidar,
            ProcessStage::CheckLidar => *self = ProcessStage::ChooseSquare,
            ProcessStage::ChooseSquare => *self = ProcessStage::ChooseSubTile,
            ProcessStage::ChooseSubTile => *self = ProcessStage::ConvertingCOPC,
            ProcessStage::ConvertingCOPC => *self = ProcessStage::AdjustContours,
            ProcessStage::AdjustContours => *self = ProcessStage::AdjustOpenness,
            ProcessStage::AdjustOpenness => *self = ProcessStage::AdjustVegetation,
            ProcessStage::AdjustVegetation => *self = ProcessStage::AdjustCliffs,
            ProcessStage::AdjustCliffs => *self = ProcessStage::AdjustIntensity,
            ProcessStage::AdjustIntensity => *self = ProcessStage::MakeMap,
            ProcessStage::MakeMap => *self = ProcessStage::ExportDone,
            _ => unreachable!("Should not call next on state for {:?} variant.", self),
        };
    }

    pub fn prev(&mut self) {
        match self {
            ProcessStage::AdjustContours => *self = ProcessStage::ChooseSquare,
            ProcessStage::AdjustOpenness => *self = ProcessStage::AdjustContours,
            ProcessStage::AdjustVegetation => *self = ProcessStage::AdjustOpenness,
            ProcessStage::AdjustCliffs => *self = ProcessStage::AdjustVegetation,
            ProcessStage::AdjustIntensity => *self = ProcessStage::AdjustCliffs,
            ProcessStage::ChooseSubTile => *self = ProcessStage::ChooseSquare,
            ProcessStage::ShowComponents => *self = ProcessStage::CheckLidar,
            _ => unreachable!("Should not call prev on state for {:?} variant.", self),
        }
    }

    pub fn is_adjustment(self) -> bool {
        matches!(
            self,
            ProcessStage::AdjustContours
                | ProcessStage::AdjustOpenness
                | ProcessStage::AdjustVegetation
                | ProcessStage::AdjustCliffs
                | ProcessStage::AdjustIntensity
        )
    }
}
