pub mod dfm;

pub use self::dfm::Dfm;

pub enum FieldType {
    Elevation,
    Intensity,
    ReturnNumber,
}

pub enum Threshold {
    Upper(f64),
    Lower(f64),
}
