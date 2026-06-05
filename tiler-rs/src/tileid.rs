//! The 64-bit sort key that drives the whole pipeline.
//!
//! Every clipped feature becomes a sort record keyed by (see TILER.md §1):
//! ```text
//! bits [63..16]  tile_id (48 bits)  — Hilbert-ordered, zoom-prefixed
//! bits [15..12]  layer   (4 bits)   — up to 16 layers
//! bits [11..0]   rank    (12 bits)  — feature priority within layer
//! ```
//! Sorting by this key makes all records for a tile adjacent, ordered by layer
//! then rank — exactly the grouping the encoder consumes.

use crate::hilbert;

/// Bits reserved for the layer index.
pub const LAYER_BITS: u32 = 4;
/// Bits reserved for the within-layer rank.
pub const RANK_BITS: u32 = 12;
/// Maximum encodable layer index (inclusive).
pub const MAX_LAYER: u8 = (1 << LAYER_BITS) - 1;
/// Maximum encodable rank (inclusive).
pub const MAX_RANK: u16 = (1 << RANK_BITS) - 1;

const LAYER_MASK: u64 = (1 << LAYER_BITS) - 1;
const RANK_MASK: u64 = (1 << RANK_BITS) - 1;

/// Packs a Hilbert tile id, layer, and rank into the 64-bit sort key.
///
/// `layer` and `rank` are saturated to their field widths so a misbehaving
/// caller can never corrupt the tile id bits (defensive: errors out of
/// existence rather than producing a wrong-tile key).
pub fn sort_key(tile_id: u64, layer: u8, rank: u16) -> u64 {
    let layer = (layer.min(MAX_LAYER) as u64) & LAYER_MASK;
    let rank = (rank.min(MAX_RANK) as u64) & RANK_MASK;
    (tile_id << RANK_BITS << LAYER_BITS) | (layer << RANK_BITS) | rank
}

/// Extracts the Hilbert tile id from a sort key.
pub fn key_tile_id(key: u64) -> u64 {
    key >> (RANK_BITS + LAYER_BITS)
}

/// Extracts the layer index from a sort key.
pub fn key_layer(key: u64) -> u8 {
    ((key >> RANK_BITS) & LAYER_MASK) as u8
}

/// Extracts the rank from a sort key.
pub fn key_rank(key: u64) -> u16 {
    (key & RANK_MASK) as u16
}

/// Convenience: builds a sort key directly from a `(z, x, y)` address.
pub fn sort_key_for(z: u8, x: u32, y: u32, layer: u8, rank: u16) -> u64 {
    sort_key(hilbert::tile_id(z, x, y), layer, rank)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pack_unpack_roundtrip() {
        let tid = hilbert::tile_id(14, 9000, 4096);
        let key = sort_key(tid, 5, 1234);
        assert_eq!(key_tile_id(key), tid);
        assert_eq!(key_layer(key), 5);
        assert_eq!(key_rank(key), 1234);
    }

    #[test]
    fn ordering_is_tile_then_layer_then_rank() {
        let t0 = hilbert::tile_id(10, 1, 1);
        let t1 = hilbert::tile_id(10, 2, 1); // different (likely larger) tile id
        let (lo, hi) = if t0 < t1 { (t0, t1) } else { (t1, t0) };

        // Same tile: layer dominates rank.
        assert!(sort_key(lo, 0, MAX_RANK) < sort_key(lo, 1, 0));
        // Same tile and layer: rank breaks the tie.
        assert!(sort_key(lo, 1, 5) < sort_key(lo, 1, 6));
        // Different tile dominates everything.
        assert!(sort_key(lo, MAX_LAYER, MAX_RANK) < sort_key(hi, 0, 0));
    }

    #[test]
    fn fields_saturate_instead_of_overflowing() {
        let tid = hilbert::tile_id(0, 0, 0);
        // Oversized layer/rank must not bleed into the tile id.
        let key = sort_key(tid, 255, u16::MAX);
        assert_eq!(key_tile_id(key), tid);
        assert_eq!(key_layer(key), MAX_LAYER);
        assert_eq!(key_rank(key), MAX_RANK);
    }
}
