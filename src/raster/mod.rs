pub mod dfm;

pub use self::dfm::Dfm;

pub enum FieldType {
    Elevation,
    Intensity,
    ReturnNumber,
}
