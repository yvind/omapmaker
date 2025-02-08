mod gui_variables;
mod main_panel;
mod map_controls;
mod map_plugins;
mod modals;
mod omap_maker;
mod side_panel;
mod terminal_like;

pub use gui_variables::GuiVariables;
pub use omap_maker::OmapMaker;

#[derive(PartialEq, Eq)]
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
            _ => unreachable!("Should not call next on state for this variant."),
        }
    }

    pub fn prev(&mut self) {
        match self {
            ProcessStage::AdjustSliders => *self = ProcessStage::ChooseSquare,
            ProcessStage::ShowComponents => *self = ProcessStage::CheckLidar,
            _ => unreachable!("Should not call prev on state for this variant."),
        }
    }
}
