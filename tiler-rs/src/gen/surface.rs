//! Procedural biome surface: per-vertex classification + marching-squares
//! polygon extraction (port of `gen/surface.c`).
//!
//! A `SURFACE_VERTS²` grid of biome classes (driven by elevation + moisture) is
//! turned into one polygon patch per (cell, class) pair via a clockwise
//! perimeter walk. Patches are emitted in tile-local quantized coordinates with
//! a closed ring (last vertex repeats the first), matching the C encoder.

use super::terrain::{terrain_elevation, terrain_moisture};
use crate::project::{self, Bounds};

/// Marching-squares grid resolution (matches terrain detail).
pub const SURFACE_GRID: usize = 128;
/// Extra cells of buffer on each side.
pub const SURFACE_BUFFER: usize = 8;
/// Total cells per side (`SURFACE_GRID + 2 * SURFACE_BUFFER`).
pub const SURFACE_TOTAL: usize = SURFACE_GRID + 2 * SURFACE_BUFFER;
/// Classification vertices per side (`SURFACE_TOTAL + 1`).
pub const SURFACE_VERTS: usize = SURFACE_TOTAL + 1;

// Surface class indices into the tile-scope value dictionary.
pub const SURFACE_VAL_WATER: u32 = 0;
pub const SURFACE_VAL_DESERT: u32 = 1;
pub const SURFACE_VAL_FOREST: u32 = 2;
pub const SURFACE_VAL_GRASSLAND: u32 = 3;
pub const SURFACE_VAL_CROPLAND: u32 = 4;
pub const SURFACE_VAL_SHRUB: u32 = 5;
pub const SURFACE_VAL_ICE: u32 = 6;

// Biome classification thresholds.
const BIOME_ELEV_ICE: f64 = 3000.0;
const BIOME_ELEV_MID: f64 = 1500.0;
const BIOME_ELEV_LOW: f64 = 400.0;
const BIOME_MOIST_WET: f64 = 0.55;
const BIOME_MOIST_DRY: f64 = 0.25;

/// A single polygon patch from one marching-squares cell: a closed ring of
/// tile-local quantized vertices with its biome class.
pub struct MsPatch {
    pub x: Vec<u16>,
    pub y: Vec<u16>,
    pub cls: u32,
}

/// Classifies the biome at each vertex of the marching-squares grid.
fn classify_surface(bounds: &Bounds) -> Vec<u32> {
    let lon_span = bounds.width();
    let lat_span = bounds.height();
    let mut vert_class = vec![0u32; SURFACE_VERTS * SURFACE_VERTS];

    for vr in 0..SURFACE_VERTS {
        let v = (vr as f64 - SURFACE_BUFFER as f64) / SURFACE_GRID as f64;
        let lat = bounds.south + v * lat_span;
        for vc in 0..SURFACE_VERTS {
            let u = (vc as f64 - SURFACE_BUFFER as f64) / SURFACE_GRID as f64;
            let lon = bounds.west + u * lon_span;
            let e = terrain_elevation(lon, lat);
            let m = terrain_moisture(lon, lat);

            let cls = if e < 0.0 {
                SURFACE_VAL_WATER
            } else if e > BIOME_ELEV_ICE {
                SURFACE_VAL_ICE
            } else if e > BIOME_ELEV_MID {
                if m > BIOME_MOIST_WET { SURFACE_VAL_FOREST } else { SURFACE_VAL_SHRUB }
            } else if e > BIOME_ELEV_LOW {
                if m > BIOME_MOIST_WET { SURFACE_VAL_FOREST } else { SURFACE_VAL_GRASSLAND }
            } else if m > BIOME_MOIST_WET {
                SURFACE_VAL_FOREST
            } else if m > BIOME_MOIST_DRY {
                SURFACE_VAL_CROPLAND
            } else {
                SURFACE_VAL_DESERT
            };

            vert_class[vr * SURFACE_VERTS + vc] = cls;
        }
    }
    vert_class
}

/// Generates surface polygon patches via marching squares.
pub fn generate_surface_patches(bounds: &Bounds) -> Vec<MsPatch> {
    let vert_class = classify_surface(bounds);
    let mut patches: Vec<MsPatch> = Vec::new();
    let qn = project::quantize_unit;
    let grid = SURFACE_GRID as f64;
    let buf = SURFACE_BUFFER as f64;

    for r in 0..SURFACE_TOTAL {
        for c in 0..SURFACE_TOTAL {
            let cl_tl = vert_class[r * SURFACE_VERTS + c];
            let cl_tr = vert_class[r * SURFACE_VERTS + c + 1];
            let cl_bl = vert_class[(r + 1) * SURFACE_VERTS + c];
            let cl_br = vert_class[(r + 1) * SURFACE_VERTS + c + 1];

            // Unique classes present in this cell, in corner order.
            let corners = [cl_tl, cl_tr, cl_bl, cl_br];
            let mut unique: Vec<u32> = Vec::with_capacity(4);
            for &corner in &corners {
                if !unique.contains(&corner) {
                    unique.push(corner);
                }
            }

            // Quantized cell corner + edge-midpoint coordinates.
            let xl = qn((c as f64 - buf) / grid);
            let xm = qn((c as f64 - buf + 0.5) / grid);
            let xr = qn((c as f64 - buf + 1.0) / grid);
            let yt = qn((r as f64 - buf) / grid);
            let ym = qn((r as f64 - buf + 0.5) / grid);
            let yb = qn((r as f64 - buf + 1.0) / grid);

            for &cls in &unique {
                // Perimeter walk (clockwise from TL).
                let mut xs: Vec<u16> = Vec::with_capacity(9);
                let mut ys: Vec<u16> = Vec::with_capacity(9);
                let push = |px: u16, py: u16, xs: &mut Vec<u16>, ys: &mut Vec<u16>| {
                    xs.push(px);
                    ys.push(py);
                };

                if cl_tl == cls { push(xl, yt, &mut xs, &mut ys); }
                if cl_tl != cl_tr && (cl_tl == cls || cl_tr == cls) { push(xm, yt, &mut xs, &mut ys); }
                if cl_tr == cls { push(xr, yt, &mut xs, &mut ys); }
                if cl_tr != cl_br && (cl_tr == cls || cl_br == cls) { push(xr, ym, &mut xs, &mut ys); }
                if cl_br == cls { push(xr, yb, &mut xs, &mut ys); }
                if cl_bl != cl_br && (cl_bl == cls || cl_br == cls) { push(xm, yb, &mut xs, &mut ys); }
                if cl_bl == cls { push(xl, yb, &mut xs, &mut ys); }
                if cl_tl != cl_bl && (cl_tl == cls || cl_bl == cls) { push(xl, ym, &mut xs, &mut ys); }

                if xs.len() < 3 {
                    continue;
                }
                // Close the ring.
                xs.push(xs[0]);
                ys.push(ys[0]);
                patches.push(MsPatch { x: xs, y: ys, cls });
            }
        }
    }
    patches
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn patches_are_closed_rings_with_known_classes() {
        let b = Bounds::of_tile(6, 33, 30);
        let patches = generate_surface_patches(&b);
        assert!(!patches.is_empty());
        for p in &patches {
            assert_eq!(p.x.len(), p.y.len());
            assert!(p.x.len() >= 4); // >=3 vertices + closing repeat
            assert_eq!(p.x.first(), p.x.last());
            assert_eq!(p.y.first(), p.y.last());
            assert!(p.cls <= SURFACE_VAL_ICE);
        }
    }
}
