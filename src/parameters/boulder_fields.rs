/// Area detection for symbol 211, independent of individual boulders.
#[derive(Clone, Debug, PartialEq)]
pub struct BoulderFieldParameters {
    pub enabled: bool,
    pub threshold: f32,
}

impl Default for BoulderFieldParameters {
    fn default() -> Self {
        Self {
            enabled: true,
            threshold: 0.5,
        }
    }
}
