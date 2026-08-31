#[cfg(feature = "stream-svf-slope")]
#[path = "build/deep_learning.rs"]
mod deep_learning;

#[cfg(feature = "stream-svf-slope")]
fn main() {
    deep_learning::run().unwrap_or_else(|error| panic!("deep-learning model catalog: {error}"));
}

#[cfg(not(feature = "stream-svf-slope"))]
fn main() {}
