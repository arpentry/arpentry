//! Forest tree scatter (port of `gen/tree.c`).
//!
//! Trees sit on a fixed global grid (so the same tree always lands in the same
//! place regardless of which tile asks for it), jittered by simplex noise, and
//! restricted to a radius around the town centre. Each tree gets a stable id
//! from its grid cell so the client can dedupe across tiles.

use super::noise::simplex2;
use super::terrain::terrain_elevation;
use crate::project::Bounds;

/// Tree class indices into the tile-scope value dictionary.
pub const TREE_VAL_OAK: u32 = 15;
pub const TREE_VAL_PINE: u32 = 16;
pub const TREE_VAL_BIRCH: u32 = 17;

/// Maximum trees emitted per tile.
pub const TREE_GRID_MAX: usize = 4096;

// Fixed global grid spacing in degrees (~55 m at the equator).
const CELL_DEG: f64 = 0.0005;
// Noise frequency for jitter.
const JITTER_FREQ: f64 = 50.0;
// Trees only within this radius (degrees) of the town centre (0, 0).
const TREE_RADIUS_DEG: f64 = 0.15;

/// A single tree: geodetic position, class value, and stable id.
pub struct TreePoint {
    pub lon: f64,
    pub lat: f64,
    pub class_val: u32,
    pub id: u64,
}

/// Generates tree positions overlapping the given tile bounds (at most
/// [`TREE_GRID_MAX`]). Buffer-zone filtering is the caller's concern.
pub fn generate_trees(bounds: &Bounds) -> Vec<TreePoint> {
    // Clamp iteration to the tree area.
    let lo_lon = bounds.west.max(-TREE_RADIUS_DEG);
    let hi_lon = bounds.east.min(TREE_RADIUS_DEG);
    let lo_lat = bounds.south.max(-TREE_RADIUS_DEG);
    let hi_lat = bounds.north.min(TREE_RADIUS_DEG);

    if lo_lon >= hi_lon || lo_lat >= hi_lat {
        return Vec::new();
    }

    // Snap to the global grid.
    let c0 = (lo_lon / CELL_DEG).floor() as i32;
    let c1 = (hi_lon / CELL_DEG).ceil() as i32;
    let r0 = (lo_lat / CELL_DEG).floor() as i32;
    let r1 = (hi_lat / CELL_DEG).ceil() as i32;

    let mut out = Vec::new();

    'outer: for r in r0..=r1 {
        for c in c0..=c1 {
            if out.len() >= TREE_GRID_MAX {
                break 'outer;
            }
            let mut lon = (c as f64 + 0.5) * CELL_DEG;
            let mut lat = (r as f64 + 0.5) * CELL_DEG;

            // Only within radius.
            if lon * lon + lat * lat > TREE_RADIUS_DEG * TREE_RADIUS_DEG {
                continue;
            }
            // Skip water.
            if terrain_elevation(lon, lat) <= 0.0 {
                continue;
            }

            // Deterministic jitter from simplex noise.
            let jx = simplex2(lon * JITTER_FREQ, lat * JITTER_FREQ);
            let jy = simplex2(lat * JITTER_FREQ + 100.0, lon * JITTER_FREQ + 100.0);
            lon += jx * CELL_DEG * 0.4;
            lat += jy * CELL_DEG * 0.4;

            // Deterministic class from a position hash.
            let th = (c as u32).wrapping_mul(73_856_093) ^ (r as u32).wrapping_mul(19_349_663);
            let class_val = match th % 3 {
                0 => TREE_VAL_OAK,
                1 => TREE_VAL_PINE,
                _ => TREE_VAL_BIRCH,
            };

            let id = ((c as u32 as u64) << 32) | (r as u32 as u64);
            out.push(TreePoint { lon, lat, class_val, id });
        }
    }

    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_trees_far_from_town() {
        let b = Bounds::of_tile(4, 0, 0); // far from (0, 0)
        assert!(generate_trees(&b).is_empty());
    }

    #[test]
    fn trees_are_deterministic_near_town() {
        let b = Bounds::of_tile(10, 512, 512); // straddles (0, 0)
        let a = generate_trees(&b);
        let c = generate_trees(&b);
        assert_eq!(a.len(), c.len());
        if let (Some(t0), Some(t1)) = (a.first(), c.first()) {
            assert_eq!(t0.id, t1.id);
            assert_eq!(t0.lon, t1.lon);
        }
    }
}
