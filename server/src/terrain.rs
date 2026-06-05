//! Terrain mesh generation (TILER.md §encode/terrain, FORMAT.md §9 terrain).
//!
//! The client requires every tile to carry a `terrain` layer with a
//! `MeshGeometry`; tiles without one are discarded wholesale. This module
//! produces a flat grid mesh in tile-local quantized coordinates (z = 0, on the
//! ellipsoid). The mesh is identical for every tile (the quantized tile-proper
//! span is always the same), so the pipeline builds it once and reuses it.
//!
//! Real DEM-driven elevation is a later step; this is the empty/flat parity
//! mesh that makes the client render a tile at all.

use crate::project::{BUFFER, EXTENT};

/// A triangulated mesh in tile-local quantized coordinates.
pub struct TerrainMesh {
    /// Vertex X (quantized uint16).
    pub x: Vec<u16>,
    /// Vertex Y (quantized uint16).
    pub y: Vec<u16>,
    /// Vertex elevation, int32 millimetres above the ellipsoid (0 = flat).
    pub z: Vec<i32>,
    /// Triangle indices.
    pub indices: Vec<u32>,
    /// Octahedral int8×2 per-vertex normals (all "up" for a flat mesh).
    pub normals: Vec<i8>,
}

/// Builds a flat `grid`×`grid`-cell mesh spanning the tile proper.
///
/// Vertices run from the west/south edge (quantized [`BUFFER`] = 16384) to the
/// east/north edge (`BUFFER + EXTENT` = 49152) so adjacent tiles share edge
/// positions. `grid` is clamped to at least 1.
pub fn flat_mesh(grid: u32) -> TerrainMesh {
    let grid = grid.max(1);
    let n = grid + 1; // vertices per side
    let origin = BUFFER as u32; // 16384
    let step = EXTENT as u32 / grid; // span EXTENT (32768) across `grid` cells

    let vcount = (n * n) as usize;
    let mut x = Vec::with_capacity(vcount);
    let mut y = Vec::with_capacity(vcount);
    for row in 0..n {
        for col in 0..n {
            x.push((origin + col * step) as u16);
            y.push((origin + row * step) as u16);
        }
    }

    // Two triangles per cell, wound CCW for an outward-facing surface (cols
    // east, rows north) to match the client's CullMode_Back + FrontFace_CCW
    // (cf. server/src/gen/terrain.c `build_indices`). The reverse winding shows
    // the back face (culled).
    let mut indices = Vec::with_capacity((grid * grid * 6) as usize);
    for row in 0..grid {
        for col in 0..grid {
            let tl = row * n + col;
            let tr = tl + 1;
            let bl = (row + 1) * n + col;
            let br = bl + 1;
            indices.extend_from_slice(&[tl, tr, br, tl, br, bl]);
        }
    }

    // Octahedral encoding of the +Z unit normal is (0, 0).
    let normals = vec![0i8; vcount * 2];

    TerrainMesh { x, y, z: vec![0; vcount], indices, normals }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flat_mesh_dimensions_and_span() {
        let m = flat_mesh(16);
        let n = 17usize;
        assert_eq!(m.x.len(), n * n);
        assert_eq!(m.y.len(), n * n);
        assert_eq!(m.z.len(), n * n);
        assert_eq!(m.normals.len(), n * n * 2);
        assert_eq!(m.indices.len(), 16 * 16 * 6);
        // Spans the tile proper: west/south edge to east/north edge.
        assert_eq!(*m.x.iter().min().unwrap(), 16384);
        assert_eq!(*m.x.iter().max().unwrap(), 49152);
        assert_eq!(*m.y.iter().min().unwrap(), 16384);
        assert_eq!(*m.y.iter().max().unwrap(), 49152);
        assert!(m.z.iter().all(|&z| z == 0));
        // All indices in range.
        assert!(m.indices.iter().all(|&i| (i as usize) < m.x.len()));
    }

    #[test]
    fn grid_one_is_a_single_quad() {
        let m = flat_mesh(1);
        assert_eq!(m.x.len(), 4);
        assert_eq!(m.indices.len(), 6);
    }
}
