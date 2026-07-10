pub mod dfm;
pub mod geotiff;

pub use self::dfm::Dfm;

pub enum Threshold {
    Upper(f64),
    #[allow(dead_code)]
    Lower(f64),
}

impl Threshold {
    pub fn inner(&self) -> f64 {
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
