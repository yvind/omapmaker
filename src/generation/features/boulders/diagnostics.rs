#[derive(Clone, Copy, Debug)]
pub(super) enum Rejection {
    Shape,
    Context,
    Support,
    Height,
    Opacity,
    Confidence,
    Sides,
    Pointed,
    Roughness,
    TreeStructure,
}

#[derive(Default)]
pub(crate) struct BoulderDiagnostics {
    pub(super) proposed: usize,
    pub(super) detected: usize,
    rejected_tree_structure: usize,
    rejected_opacity: usize,
    rejected_shape: usize,
    rejected_height: usize,
    rejected_support: usize,
    rejected_sides: usize,
    rejected_context: usize,
    rejected_confidence: usize,
}

impl BoulderDiagnostics {
    pub(super) fn reject(&mut self, reason: Rejection) {
        let count = match reason {
            Rejection::TreeStructure => &mut self.rejected_tree_structure,
            Rejection::Opacity => &mut self.rejected_opacity,
            Rejection::Shape | Rejection::Pointed | Rejection::Roughness => {
                &mut self.rejected_shape
            }
            Rejection::Height => &mut self.rejected_height,
            Rejection::Support => &mut self.rejected_support,
            Rejection::Sides => &mut self.rejected_sides,
            Rejection::Context => &mut self.rejected_context,
            Rejection::Confidence => &mut self.rejected_confidence,
        };
        *count += 1;
    }

    pub(crate) fn add(&mut self, other: Self) {
        self.proposed += other.proposed;
        self.detected += other.detected;
        self.rejected_tree_structure += other.rejected_tree_structure;
        self.rejected_opacity += other.rejected_opacity;
        self.rejected_shape += other.rejected_shape;
        self.rejected_height += other.rejected_height;
        self.rejected_support += other.rejected_support;
        self.rejected_sides += other.rejected_sides;
        self.rejected_context += other.rejected_context;
        self.rejected_confidence += other.rejected_confidence;
    }

    pub(crate) fn summary(&self) -> String {
        format!(
            "Boulders: {} detected from {} candidates; rejected tree structure={}, opacity={}, footprint/top shape={}, height={}, insufficient surface returns={}, surrounding ground={}, terrain={}, confidence={}",
            self.detected,
            self.proposed,
            self.rejected_tree_structure,
            self.rejected_opacity,
            self.rejected_shape,
            self.rejected_height,
            self.rejected_support,
            self.rejected_sides,
            self.rejected_context,
            self.rejected_confidence,
        )
    }
}
