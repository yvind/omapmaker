pub mod gui_variables;
pub mod main_panel;
pub mod map_controls;
pub mod map_plugins;
pub mod modals;
pub mod side_panel;
pub mod terminal_like;

pub use gui_variables::GuiVariables;

#[derive(PartialEq, Eq, Debug)]
pub enum ProcessStage {
    AdjustSliders,
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
            ProcessStage::ConvertingCOPC => *self = ProcessStage::AdjustSliders,
            ProcessStage::AdjustSliders => *self = ProcessStage::MakeMap,
            ProcessStage::MakeMap => *self = ProcessStage::ExportDone,
            _ => unreachable!("Should not call next on state for {:?} variant.", self),
        };
    }

    pub fn prev(&mut self) {
        match self {
            ProcessStage::AdjustSliders => *self = ProcessStage::ChooseSquare,
            ProcessStage::ChooseSubTile => *self = ProcessStage::ChooseSquare,
            ProcessStage::ShowComponents => *self = ProcessStage::CheckLidar,
            _ => unreachable!("Should not call prev on state for {:?} variant.", self),
        }
    }
}
