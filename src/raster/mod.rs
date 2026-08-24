pub mod dfm;
pub mod geotiff;
mod hydrology;

pub use self::dfm::Dfm;
pub use self::hydrology::{D8Flow, accumulate_cross_tile_flow};

pub enum Threshold {
    Upper(f32),
    #[allow(dead_code)]
    Lower(f32),
}

impl Threshold {
    pub fn inner(&self) -> f32 {
        match self {
            Threshold::Upper(t) => *t,
            Threshold::Lower(t) => *t,
        }
    }

    pub fn is_upper(&self) -> bool {
        match self {
            Threshold::Upper(_) => true,
            Threshold::Lower(_) => false,
        }
    }
}
