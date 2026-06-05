//! Hilbert curve mapping for tile ordering.
//!
//! Converts between `(x, y)` coordinates and Hilbert-curve distance on a
//! `2^order` square grid, and packs/unpacks zoom-prefixed tile ids.
//!
//! Tile id layout (see TILER.md §1):
//! ```text
//! bits [47..42]  zoom    (6 bits, max 63)
//! bits [41..0]   hilbert (42 bits, sufficient up to z=20)
//! ```
//! At zoom `z` the grid is `2^z × 2^z` (one root tile at z0), so the Hilbert
//! `order` is `z` — matching the C client/server. (FORMAT.md §2 describes a
//! `2^(z+1)`-column scheme but is out of date.)

/// Number of bits reserved for the Hilbert distance within a tile id.
pub const HILBERT_BITS: u32 = 42;
/// Mask selecting the Hilbert distance portion of a tile id.
pub const HILBERT_MASK: u64 = (1u64 << HILBERT_BITS) - 1;

/// Maps `(x, y)` on a `2^order` grid to its Hilbert-curve distance.
pub fn xy2d(order: u32, mut x: u32, mut y: u32) -> u64 {
    let n: u32 = 1 << order;
    let mut d: u64 = 0;
    let mut s: u32 = n / 2;
    while s > 0 {
        let rx: u32 = if (x & s) > 0 { 1 } else { 0 };
        let ry: u32 = if (y & s) > 0 { 1 } else { 0 };
        d += (s as u64) * (s as u64) * ((3 * rx) ^ ry) as u64;
        rot(n, &mut x, &mut y, rx, ry);
        s /= 2;
    }
    d
}

/// Maps a Hilbert-curve distance `d` on a `2^order` grid back to `(x, y)`.
pub fn d2xy(order: u32, mut d: u64) -> (u32, u32) {
    let n: u32 = 1 << order;
    let mut x: u32 = 0;
    let mut y: u32 = 0;
    let mut s: u32 = 1;
    while s < n {
        let rx: u32 = 1 & (d / 2) as u32;
        let ry: u32 = 1 & (d as u32 ^ rx);
        rot(s, &mut x, &mut y, rx, ry);
        x += s * rx;
        y += s * ry;
        d /= 4;
        s *= 2;
    }
    (x, y)
}

/// Rotates/reflects a quadrant to keep the Hilbert curve continuous.
fn rot(n: u32, x: &mut u32, y: &mut u32, rx: u32, ry: u32) {
    if ry == 0 {
        if rx == 1 {
            *x = n - 1 - *x;
            *y = n - 1 - *y;
        }
        std::mem::swap(x, y);
    }
}

/// Encodes a `(z, x, y)` tile address into a zoom-prefixed, Hilbert-ordered id.
///
/// The grid at level `z` is `2^z × 2^z` (one root tile at z0), so the Hilbert
/// `order` is `z` — matching the C client/server (`tiler/src/hilbert.c`).
/// (Note: this differs from FORMAT.md §2, which is out of date.)
pub fn tile_id(z: u8, x: u32, y: u32) -> u64 {
    let h = xy2d(z as u32, x, y) & HILBERT_MASK;
    ((z as u64) << HILBERT_BITS) | h
}

/// Decodes a tile id back into its `(z, x, y)` address.
pub fn tile_id_decode(id: u64) -> (u8, u32, u32) {
    let z = (id >> HILBERT_BITS) as u8 & 0x3F;
    let h = id & HILBERT_MASK;
    let (x, y) = d2xy(z as u32, h);
    (z, x, y)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn xy2d_d2xy_roundtrip() {
        for order in 1..=12u32 {
            let n = 1u32 << order;
            // Sample the grid (full sweep for small orders, strided for large).
            let step = (n / 64).max(1);
            let mut x = 0;
            while x < n {
                let mut y = 0;
                while y < n {
                    let d = xy2d(order, x, y);
                    assert_eq!(d2xy(order, d), (x, y), "order={order} x={x} y={y}");
                    y += step;
                }
                x += step;
            }
        }
    }

    #[test]
    fn hilbert_distance_is_a_permutation() {
        // Every cell maps to a distinct distance in [0, n*n).
        for order in 1..=8u32 {
            let n = 1u32 << order;
            let mut seen = vec![false; (n as usize) * (n as usize)];
            for x in 0..n {
                for y in 0..n {
                    let d = xy2d(order, x, y) as usize;
                    assert!(d < seen.len());
                    assert!(!seen[d], "duplicate distance at order={order}");
                    seen[d] = true;
                }
            }
            assert!(seen.into_iter().all(|b| b));
        }
    }

    #[test]
    fn adjacent_distances_are_grid_neighbours() {
        // Defining property of the Hilbert curve: consecutive distances are
        // always one grid step apart (Manhattan distance 1).
        for order in 1..=8u32 {
            let n = 1u32 << order;
            let total = (n as u64) * (n as u64);
            let mut prev = d2xy(order, 0);
            for d in 1..total {
                let cur = d2xy(order, d);
                let dx = (cur.0 as i64 - prev.0 as i64).abs();
                let dy = (cur.1 as i64 - prev.1 as i64).abs();
                assert_eq!(dx + dy, 1, "order={order} d={d}");
                prev = cur;
            }
        }
    }

    #[test]
    fn tile_id_roundtrip() {
        // Grid at level z is 2^z × 2^z, so x,y < 2^z.
        let cases = [(0u8, 0u32, 0u32), (1, 1, 0), (5, 31, 17), (14, 9000, 4096)];
        for (z, x, y) in cases {
            let id = tile_id(z, x, y);
            assert_eq!(tile_id_decode(id), (z, x, y), "z={z} x={x} y={y}");
        }
    }

    #[test]
    fn tile_id_orders_by_zoom() {
        // Higher zoom => larger id (zoom occupies the top bits).
        assert!(tile_id(3, 7, 3) < tile_id(4, 0, 0));
    }
}
