//! Layer indices and names.
//!
//! Layers are ordered by decode priority (terrain first); the index doubles as
//! the 4-bit `layer` field of the sort key (see [`crate::tileid`]). This mirrors
//! the C tiler's `layers.h` and the reference schema in FORMAT.md §9.
//!
//! NOTE: confirm this set against the C `layers.h` before relying on it for
//! cross-implementation parity — the Overture/Natural Earth runs key inputs by
//! these indices on the CLI (`--input N:path`).

/// Layer index, also used as the sort-key `layer` field.
pub type LayerIndex = u8;

pub const TERRAIN: LayerIndex = 0;
pub const LAND_COVER: LayerIndex = 1;
pub const BATHYMETRY: LayerIndex = 2;
pub const WATER: LayerIndex = 3;
pub const LAND: LayerIndex = 4;
pub const TRANSPORTATION: LayerIndex = 5;
pub const LAND_USE: LayerIndex = 6;

/// Number of defined layers.
pub const COUNT: usize = 7;

/// Stable layer names, indexed by [`LayerIndex`].
pub const NAMES: [&str; COUNT] = [
    "terrain",
    "land_cover",
    "bathymetry",
    "water",
    "land",
    "transportation",
    "land_use",
];

/// Returns the name for a layer index, if defined.
pub fn name(index: LayerIndex) -> Option<&'static str> {
    NAMES.get(index as usize).copied()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names_align_with_indices() {
        assert_eq!(name(TERRAIN), Some("terrain"));
        assert_eq!(name(LAND_USE), Some("land_use"));
        assert_eq!(name(COUNT as u8), None);
        assert_eq!(NAMES.len(), COUNT);
    }
}
