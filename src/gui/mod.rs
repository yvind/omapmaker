pub mod gui_variables;
pub mod main_panel;
pub mod map_controls;
pub mod map_plugins;
pub mod modals;
pub mod side_panel;
pub mod terminal_like;
pub mod tile_sources;

pub use gui_variables::GuiVariables;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ProcessStage {
    AdjustContours,
    AdjustOpenness,
    AdjustVegetation,
    AdjustBuildings,
    AdjustCliffs,
    AdjustWater,
    AdjustMarsh,
    AdjustStreams,
    AdjustIntensity,
    CheckLidar,
    ShowComponents,
    ChooseSquare,
    ConvertingCOPC,
    DrawPolygon,
    ExportDone,
    MakeMap,
    PrepareMapPreview,
    Welcome,
}

impl ProcessStage {
    pub fn next(&mut self) {
        match self {
            ProcessStage::Welcome => *self = ProcessStage::CheckLidar,
            ProcessStage::CheckLidar => *self = ProcessStage::DrawPolygon,
            ProcessStage::DrawPolygon => *self = ProcessStage::ConvertingCOPC,
            ProcessStage::ConvertingCOPC => *self = ProcessStage::ChooseSquare,
            ProcessStage::ChooseSquare => *self = ProcessStage::PrepareMapPreview,
            ProcessStage::PrepareMapPreview => *self = ProcessStage::AdjustContours,
            ProcessStage::AdjustContours => *self = ProcessStage::AdjustOpenness,
            ProcessStage::AdjustOpenness => *self = ProcessStage::AdjustVegetation,
            ProcessStage::AdjustVegetation => *self = ProcessStage::AdjustBuildings,
            ProcessStage::AdjustBuildings => *self = ProcessStage::AdjustCliffs,
            ProcessStage::AdjustCliffs => *self = ProcessStage::AdjustWater,
            ProcessStage::AdjustWater => *self = ProcessStage::AdjustMarsh,
            ProcessStage::AdjustMarsh => *self = ProcessStage::AdjustStreams,
            ProcessStage::AdjustStreams => *self = ProcessStage::AdjustIntensity,
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
            ProcessStage::AdjustBuildings => *self = ProcessStage::AdjustVegetation,
            ProcessStage::AdjustCliffs => *self = ProcessStage::AdjustBuildings,
            ProcessStage::AdjustIntensity => *self = ProcessStage::AdjustStreams,
            ProcessStage::AdjustStreams => *self = ProcessStage::AdjustMarsh,
            ProcessStage::AdjustMarsh => *self = ProcessStage::AdjustWater,
            ProcessStage::AdjustWater => *self = ProcessStage::AdjustCliffs,
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
                | ProcessStage::AdjustBuildings
                | ProcessStage::AdjustCliffs
                | ProcessStage::AdjustWater
                | ProcessStage::AdjustMarsh
                | ProcessStage::AdjustStreams
                | ProcessStage::AdjustIntensity
        )
    }
}

#[cfg(test)]
mod tests {
    use super::ProcessStage;

    #[test]
    fn water_marsh_and_streams_are_separate_consecutive_adjustment_steps() {
        let mut stage = ProcessStage::AdjustWater;
        stage.next();
        assert_eq!(stage, ProcessStage::AdjustMarsh);
        stage.next();
        assert_eq!(stage, ProcessStage::AdjustStreams);
        stage.next();
        assert_eq!(stage, ProcessStage::AdjustIntensity);

        stage.prev();
        assert_eq!(stage, ProcessStage::AdjustStreams);
        stage.prev();
        assert_eq!(stage, ProcessStage::AdjustMarsh);
        stage.prev();
        assert_eq!(stage, ProcessStage::AdjustWater);
    }
}
