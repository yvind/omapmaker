#[cfg(feature = "deep-learning")]
#[path = "build/deep_learning.rs"]
mod deep_learning;

#[cfg(feature = "deep-learning")]
fn main() {
    deep_learning::run().unwrap_or_else(|error| panic!("deep-learning model catalog: {error}"));
}

#[cfg(not(feature = "deep-learning"))]
fn main() {}
