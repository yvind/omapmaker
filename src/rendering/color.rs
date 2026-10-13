//! Colour-table draw-order resolution: what a reader sees where two features'
//! ink overlaps, independent of how either is actually painted.
//!
//! Works in `omap`'s own [`Rgb`], not a renderer's colour type, so this stays
//! shareable with the navigation layer, which only ever wants the priority
//! half of the answer.

use omap::UnitF64;
use omap::colors::{ColorSet, Rgb, SymbolColor};
use omap::symbols::{Element, FillPattern, PointSymbol, Symbol, SymbolId, SymbolSet};

/// Priority for a colour that could not be resolved, which puts it far behind
/// everything with a real one. Shared with `omap-render`'s own default z for
/// an unresolved primitive — see its `Z_UNKNOWN`.
pub const UNKNOWN_PRIORITY: i32 = 10_000;

/// Resolve a [`SymbolColor`] into an RGB colour and its priority index in the
/// colour set. Priority doubles as the paint order key.
pub fn resolve_color(sc: &SymbolColor, colors: &ColorSet) -> Option<(Rgb, i32)> {
    match sc {
        SymbolColor::NoColor => None,
        // Registration black always prints on top of everything in Mapper.
        SymbolColor::RegistrationBlack => Some((
            Rgb {
                r: UnitF64::zero(),
                g: UnitF64::zero(),
                b: UnitF64::zero(),
            },
            -1,
        )),
        SymbolColor::Color(id) => {
            // `ColorSet::rgb` matches how Mapper displays the colour (honouring
            // each colour's RGB mode) and resolves spot-colour compositions
            // rather than reporting pure black for them (e.g. "Black 35%").
            let rgb = colors.rgb(*id).ok()?;
            let prio = colors
                .priority_of(*id)
                .map(|p| p as i32)
                .unwrap_or(UNKNOWN_PRIORITY);
            Some((rgb, prio))
        }
    }
}

/// The representative colour + priority for a symbol, used for both fill/stroke
/// colour and z-ordering. Combined symbols recurse into their first component.
pub(crate) fn symbol_main_color(
    symbol: &Symbol,
    symbol_set: &SymbolSet,
    colors: &ColorSet,
    depth: u8,
) -> Option<(Rgb, i32)> {
    if depth > 8 {
        return None;
    }
    match symbol {
        Symbol::Area(a) => resolve_color(&a.color, colors),
        Symbol::Line(l) => resolve_color(&l.color, colors),
        Symbol::Text(t) => resolve_color(&t.color, colors),
        Symbol::Point(p) => {
            resolve_color(&p.inner_color, colors).or_else(|| resolve_color(&p.outer_color, colors))
        }
        Symbol::CombinedArea(s) => s
            .components()
            .find_map(|c| combined_area_part_color(c, symbol_set, colors, depth)),
        Symbol::CombinedLine(s) => s
            .components()
            .find_map(|c| combined_line_part_color(c, symbol_set, colors, depth)),
    }
}

fn combined_area_part_color(
    part: &omap::symbols::PublicOrPrivateSymbol<
        omap::symbols::PathSymbolId,
        omap::symbols::AreaOrLineSymbol,
    >,
    symbol_set: &SymbolSet,
    colors: &ColorSet,
    depth: u8,
) -> Option<(Rgb, i32)> {
    use omap::symbols::{AreaOrLineSymbol, PublicOrPrivateSymbol};
    match part {
        PublicOrPrivateSymbol::Public(id) => symbol_main_color(
            symbol_set.get(SymbolId::from(*id))?.symbol(),
            symbol_set,
            colors,
            depth + 1,
        ),
        PublicOrPrivateSymbol::Private(AreaOrLineSymbol::Area(a)) => {
            resolve_color(&a.color, colors)
        }
        PublicOrPrivateSymbol::Private(AreaOrLineSymbol::Line(l)) => {
            resolve_color(&l.color, colors)
        }
    }
}

fn combined_line_part_color(
    part: &omap::symbols::PublicOrPrivateSymbol<
        omap::symbols::LinePathSymbolId,
        omap::symbols::LineSymbol,
    >,
    symbol_set: &SymbolSet,
    colors: &ColorSet,
    depth: u8,
) -> Option<(Rgb, i32)> {
    use omap::symbols::PublicOrPrivateSymbol;
    match part {
        PublicOrPrivateSymbol::Public(id) => symbol_main_color(
            symbol_set.get(SymbolId::from(*id))?.symbol(),
            symbol_set,
            colors,
            depth + 1,
        ),
        PublicOrPrivateSymbol::Private(l) => resolve_color(&l.color, colors),
    }
}

/// Priority in the map's colour table of the topmost ink `symbol` lays over the
/// ground it covers, or `None` if nothing about it resolves to a colour.
///
/// This is the map's own answer to "what does a reader see here?", and it is
/// exported because the navigation layer orders overlapping features by it —
/// vegetation drawn under a road does not slow anyone down, because nobody can
/// see it. Sharing one resolver is the same rule [`crate::rendering::paper`] already
/// follows for geometry: the one thing worse than two traversals is two answers
/// about which of two overlapping features the map is actually showing.
///
/// *Over the ground it covers* is what makes this different from the z of any
/// one primitive. An area's outline strokes are excluded — a border covers its
/// own thin strip, not the interior — while fill patterns are included, which is
/// what puts ISSprOM `512.3xx` "passable at two levels" (a white hatch at
/// priority 11) above the building it is drawn over (black 50%, priority 15). A
/// line's decorations are included for the opposite reason: `512.100` bridge
/// carries no colour of its own and is drawn entirely by the black dashes along
/// it.
///
/// A combined symbol resolves through its first component that has a colour,
/// which on real files is the fill — taking the topmost over all of them would
/// return the black outline every out-of-bounds area is drawn with, and rank it
/// above every fill on the map.
pub fn ground_priority(symbol: &Symbol, symbol_set: &SymbolSet, colors: &ColorSet) -> Option<i32> {
    representative_color(symbol, symbol_set, colors, 0).map(|(_, priority)| priority)
}

/// Keep whichever of two `(colour, priority)` options is topmost (lowest index).
/// Generic because the caller's colour type doesn't matter — only the
/// priority half is ever compared — which is what lets `omap-render` reuse
/// this on its own `Color32` pairs rather than keeping a second copy.
pub fn min_prio<T>(a: Option<(T, i32)>, b: Option<(T, i32)>) -> Option<(T, i32)> {
    match (a, b) {
        (Some(x), Some(y)) => Some(if x.1 <= y.1 { x } else { y }),
        (a, b) => a.or(b),
    }
}

/// A representative `(colour, priority)` found anywhere in a symbol tree — used
/// as a fallback for a symbol whose primary colour is NoColor (e.g. a course
/// line coloured only through its dash/mid symbols). Picks the topmost colour.
pub(crate) fn representative_color(
    symbol: &Symbol,
    symbol_set: &SymbolSet,
    colors: &ColorSet,
    depth: u8,
) -> Option<(Rgb, i32)> {
    if depth > 8 {
        return None;
    }
    match symbol {
        Symbol::Point(p) => point_dot_style(p, colors, depth),
        Symbol::Line(s) => {
            let mut best = resolve_color(&s.color, colors);
            let subs = [
                s.start_symbol.as_deref(),
                s.end_symbol.as_deref(),
                s.mid_symbol.as_ref().map(|m| &m.mid_symbol),
                s.dash_symbol.as_ref().map(|d| &d.dash_symbol),
            ];
            for ps in subs.into_iter().flatten() {
                let candidate = point_dot_style(ps, colors, depth + 1);
                best = min_prio(best, candidate);
            }
            best
        }
        Symbol::Area(s) => {
            let mut best = resolve_color(&s.color, colors);
            for p in &s.patterns {
                let candidate = match p {
                    FillPattern::LinePattern { line_color, .. } => {
                        resolve_color(line_color, colors)
                    }
                    FillPattern::PointPattern { point, .. } => {
                        point_dot_style(point, colors, depth + 1)
                    }
                };
                best = min_prio(best, candidate);
            }
            best
        }
        // Combined symbols: their primary colour normally resolves already, so
        // reuse the first-component logic.
        Symbol::CombinedArea(_) | Symbol::CombinedLine(_) => {
            symbol_main_color(symbol, symbol_set, colors, depth)
        }
        Symbol::Text(t) => resolve_color(&t.color, colors),
    }
}

/// Derive a representative `(colour, priority)` for a point symbol. Pattern
/// and line-decoration symbols usually carry no top-level circle colour — it
/// lives in an `element` sub-symbol — so dig in when the top level is
/// uncoloured.
fn point_dot_style(ps: &PointSymbol, colors: &ColorSet, depth: u8) -> Option<(Rgb, i32)> {
    if depth > 8 {
        return None;
    }
    if let Some(c) = resolve_color(&ps.inner_color, colors) {
        return Some(c);
    }
    if let Some(c) = resolve_color(&ps.outer_color, colors) {
        return Some(c);
    }
    ps.elements.iter().find_map(|e| match e {
        Element::Point { symbol, .. } => point_dot_style(symbol, colors, depth + 1),
        Element::Area { symbol, .. } => resolve_color(&symbol.color, colors),
        Element::Line { symbol, .. } => resolve_color(&symbol.color, colors),
    })
}
