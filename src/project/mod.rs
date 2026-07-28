pub mod polygon;

pub fn get_global_crs() -> proj_core::CrsDef {
    proj_wkt::parse_crs("4326").unwrap()
}
