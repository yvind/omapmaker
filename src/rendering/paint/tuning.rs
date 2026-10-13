//! Screen-space tolerances for curve flattening, culling and fill patterns.

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RenderTuning {
    /// How far a flattened curve may sit from the true one, in screen pixels.
    pub curve_tolerance_px: f32,
    /// Smallest a primitive's on-screen bounding-box diagonal may be before
    /// it is culled entirely.
    pub min_visible_px: f32,
    /// Smallest on-screen spacing a scattered pattern's items may have before
    /// paint time starts thinning them out.
    pub min_pattern_pitch_px: f32,
    /// Closest two vertices of a stroked path may land on screen before the
    /// second is dropped as indistinguishable from the first.
    pub min_vertex_step_px: f32,
}

impl Default for RenderTuning {
    fn default() -> Self {
        Self {
            curve_tolerance_px: 0.1,
            min_visible_px: 1.5,
            min_pattern_pitch_px: 2.0,
            min_vertex_step_px: 1.0,
        }
    }
}
