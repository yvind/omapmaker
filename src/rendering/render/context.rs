use omap::{colors::ColorSet, symbols::SymbolSet};

use crate::rendering::geo::GeoTransform;

/// What a symbol is resolved into primitives against: the symbol set it may
/// refer into by id, the colours it is drawn in, and the transform that places
/// it on the globe.
#[derive(Clone, Copy)]
pub(super) struct Context<'a> {
    pub symbol_set: &'a SymbolSet,
    pub colors: &'a ColorSet,
    pub geo: &'a GeoTransform,
}
