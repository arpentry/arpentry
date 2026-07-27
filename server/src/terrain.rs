//! Terrain mesh generation (TILER.md §encode/terrain, FORMAT.md §9 terrain).
//!
//! The client requires every tile to carry a `terrain` layer with a
//! `MeshGeometry`; tiles without one are discarded wholesale. This module
//! produces a flat grid mesh in tile-local quantized coordinates (z = 0, on the
//! ellipsoid). The mesh is identical for every tile (the quantized tile-proper
//! span is always the same), so the pipeline builds it once and reuses it.
//!
//! [`flat_mesh`] is the empty/flat parity mesh; [`elevated_mesh`] samples a DEM
//! (e.g. Mapterhorn terrain) to produce real per-vertex elevation and normals.

use std::f64::consts::PI;

use crate::project::{self, Bounds, BUFFER, EXTENT};

/// Terrain mesh resolution: cells per tile side. The rendered ground is this
/// coarse, so road and structure geometry must take its elevation from this same
/// grid surface ([`surface_height`]) — not the raw DEM — or it floats off the
/// drawn ground.
pub const TERRAIN_GRID: u32 = 16;

/// Cells per tile side at the reference (detail) zoom: ~3–5 m cells at z16
/// mid-latitude, fine enough that the street benches the ground stage cuts
/// (D3) capture lattice corners and their cut/fill creases read as surfaces
/// rather than facets — a coarse cell spans tens of metres of height on a
/// steep flank and crosses straight through the level road it should carry.
/// Both grids divide [`EXTENT`] exactly, so quantization and tile-edge
/// sharing stay exact.
pub const TERRAIN_GRID_DETAIL: u32 = 128;

/// The rendered-lattice resolution for zoom `z` when the reference zoom is
/// `z_ref`: the detail grid at (and past) the reference zoom, then halved per
/// rung down to the base grid. One function, so the mesh, its drape mirror
/// ([`surface_height`]), and the road densifier can never disagree.
///
/// The ladder is graded rather than binary because a tile's *metric* cell size
/// is what the eye reads, and a rung covers four times the area of the one
/// below it: dropping straight from the detail grid to the base grid shrinks
/// the vertex count 64-fold in one step, so the terrain one zoom out from the
/// reference collapsed from ~3 m cells to ~50 m and the hillsides went blocky
/// the moment the camera pulled back (or looked into the distance, where a
/// tilted view draws coarser rungs). Halving per rung keeps the cell size
/// roughly doubling instead, and costs almost nothing: a rung has a quarter of
/// the tiles and a quarter of the vertices each, so the three graded rungs
/// together add ~7 % to the reference rung's vertices.
pub fn grid_for(z: u8, z_ref: u8) -> u32 {
    if z >= z_ref {
        return TERRAIN_GRID_DETAIL;
    }
    match z_ref - z {
        1 => TERRAIN_GRID_DETAIL / 2,
        2 => TERRAIN_GRID_DETAIL / 4,
        _ => TERRAIN_GRID,
    }
    .max(TERRAIN_GRID)
}

/// A triangulated mesh in tile-local quantized coordinates.
#[derive(Debug, Clone)]
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
    /// Signed across-carriageway coordinate per vertex, snorm `-127..127` = ±1
    /// (±1 at the paved edge, 0 at the centre), for analytic edge antialiasing
    /// of drivable surface meshes. Empty when the mesh carries none (terrain,
    /// buildings, plates) — the client then falls back to MSAA for that mesh.
    pub edge_across: Vec<i8>,
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

    TerrainMesh { x, y, z: vec![0; vcount], indices, normals, edge_across: Vec::new() }
}

/// Builds a `grid`×`grid`-cell mesh for the tile with per-vertex elevation
/// sampled from `sample(lon, lat) -> metres` and finite-difference normals.
///
/// The horizontal vertex grid is identical to [`flat_mesh`] (tile-proper edges
/// at quantized 16384/49152, so adjacent tiles share edge positions). Elevation
/// comes from the sampler; normals are computed from centred differences over a
/// one-vertex halo sampled just outside the tile, which keeps slopes continuous
/// across tile borders. Returns the mesh and its `(min, max)` elevation in
/// metres for the tileset's elevation range.
pub fn elevated_mesh<F>(grid: u32, bounds: &Bounds, mut sample: F) -> (TerrainMesh, f64, f64)
where
    F: FnMut(f64, f64) -> f64,
{
    let grid = grid.max(1);
    let n = grid + 1; // vertices per side
    let cell_lon = bounds.width() / grid as f64;
    let cell_lat = bounds.height() / grid as f64;

    // Approximate cell size in metres for the finite-difference slope.
    let mid_lat = (bounds.south + bounds.north) * 0.5;
    let cell_w_m = cell_lon * 111_319.5 * (mid_lat * PI / 180.0).cos();
    let cell_h_m = cell_lat * 111_319.5;

    // Padded elevation grid: one halo row/column on each side (rows/cols -1..=n).
    let pad_w = (n + 2) as usize;
    let mut elev = vec![0.0f64; pad_w * pad_w];
    for prow in 0..pad_w {
        let lat = bounds.south + (prow as f64 - 1.0) * cell_lat;
        for pcol in 0..pad_w {
            let lon = bounds.west + (pcol as f64 - 1.0) * cell_lon;
            elev[prow * pad_w + pcol] = sample(lon, lat);
        }
    }

    let vcount = (n * n) as usize;
    let mut x = Vec::with_capacity(vcount);
    let mut y = Vec::with_capacity(vcount);
    let mut z = Vec::with_capacity(vcount);
    let mut normals = vec![0i8; vcount * 2];
    let (mut emin, mut emax) = (f64::INFINITY, f64::NEG_INFINITY);

    for row in 0..n {
        let lat = bounds.south + row as f64 * cell_lat;
        let lat_r = lat * (PI / 180.0);
        let (sin_lat, cos_lat) = (lat_r.sin(), lat_r.cos());
        for col in 0..n {
            let lon = bounds.west + col as f64 * cell_lon;
            let pi = (row + 1) as usize * pad_w + (col + 1) as usize;
            let e = elev[pi];
            emin = emin.min(e);
            emax = emax.max(e);

            x.push(project::quantize_x(lon, bounds));
            y.push(project::quantize_y(lat, bounds));
            z.push(project::quantize_z(e));

            // Centred finite differences using the padded neighbours.
            let dz_dx = (elev[pi + 1] - elev[pi - 1]) / (2.0 * cell_w_m);
            let dz_dy = (elev[pi + pad_w] - elev[pi - pad_w]) / (2.0 * cell_h_m);

            // ENU basis → ECEF surface normal (up tilted against the slope).
            let lon_r = lon * (PI / 180.0);
            let (sin_lon, cos_lon) = (lon_r.sin(), lon_r.cos());
            let (ex, ey, ez) = (-sin_lon, cos_lon, 0.0);
            let (nx_e, ny_e, nz_e) = (-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat);
            let (ux, uy, uz) = (cos_lat * cos_lon, cos_lat * sin_lon, sin_lat);
            let mut nx = ux - dz_dx * ex - dz_dy * nx_e;
            let mut ny = uy - dz_dx * ey - dz_dy * ny_e;
            let mut nz = uz - dz_dx * ez - dz_dy * nz_e;
            let len = (nx * nx + ny * ny + nz * nz).sqrt();
            if len > 0.0 {
                nx /= len;
                ny /= len;
                nz /= len;
            }
            let idx = (row * n + col) as usize;
            let (ox, oy) = encode_octahedral(nx, ny, nz);
            normals[idx * 2] = ox;
            normals[idx * 2 + 1] = oy;
        }
    }

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

    if !emin.is_finite() {
        emin = 0.0;
        emax = 0.0;
    }
    (TerrainMesh { x, y, z, indices, normals, edge_across: Vec::new() }, emin, emax)
}

/// Height in metres of the rendered terrain surface at `(lon, lat)`, matching the
/// triangulated [`elevated_mesh`] exactly: the per-tile `grid` is a regular
/// lattice of `sample(lon, lat)` elevations, split into two triangles per cell
/// along the (row,col)–(row+1,col+1) diagonal. Road and structure geometry
/// elevate from this so they sit on the drawn ground. `grid` must be the same
/// [`grid_for`] resolution the tile's mesh was built with.
///
/// The lattice is global — every tile's grid lines coincide (a tile spans exactly
/// `grid` cells), so a world point sampled from any tile's `bounds` yields
/// the same height. `bounds` only anchors the lattice; points in the buffer or a
/// neighbour tile fall on extended cell indices and stay consistent.
pub fn surface_height(
    bounds: &Bounds,
    grid: u32,
    lon: f64,
    lat: f64,
    sample: &mut dyn FnMut(f64, f64) -> f64,
) -> f64 {
    let grid = grid.max(1) as f64;
    let cell_lon = bounds.width() / grid;
    let cell_lat = bounds.height() / grid;
    // Cell indices, possibly negative or >= grid for buffer / neighbour points.
    let gx = (lon - bounds.west) / cell_lon;
    let gy = (lat - bounds.south) / cell_lat;
    let (col, row) = (gx.floor(), gy.floor());
    let (fx, fy) = (gx - col, gy - row);
    // Corner coordinates on the global lattice.
    let lon0 = bounds.west + col * cell_lon;
    let lat0 = bounds.south + row * cell_lat;
    let (lon1, lat1) = (lon0 + cell_lon, lat0 + cell_lat);
    let e00 = sample(lon0, lat0); // (row,   col)
    let e10 = sample(lon1, lat0); // (row,   col+1)
    let e01 = sample(lon0, lat1); // (row+1, col)
    let e11 = sample(lon1, lat1); // (row+1, col+1)
    // Planar interpolation over the triangle (fx,fy) lands in; both halves agree
    // on the shared diagonal, so the surface is continuous.
    if fx >= fy {
        e00 + (e10 - e00) * fx + (e11 - e10) * fy
    } else {
        e00 + (e11 - e01) * fx + (e01 - e00) * fy
    }
}

/// Octahedral encoding of a unit normal to int8×2 (matches the client's
/// `decode_octahedral` in `terrain.wgsl` and the procedural generator).
pub(crate) fn encode_octahedral(nx: f64, ny: f64, nz: f64) -> (i8, i8) {
    let sum = nx.abs() + ny.abs() + nz.abs();
    if sum < 1e-15 {
        return (0, 127);
    }
    let mut u = nx / sum;
    let mut v = ny / sum;

    // Reflect the lower hemisphere.
    if nz < 0.0 {
        let old_u = u;
        u = (1.0 - v.abs()) * if old_u >= 0.0 { 1.0 } else { -1.0 };
        v = (1.0 - old_u.abs()) * if v >= 0.0 { 1.0 } else { -1.0 };
    }

    // Quantize to int8 [-127, 127] (round half away from zero).
    let q = |c: f64| -> i8 {
        let c = c * 127.0;
        let r = if c >= 0.0 { c + 0.5 } else { c - 0.5 };
        r.clamp(-127.0, 127.0) as i8
    };
    (q(u), q(v))
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

    #[test]
    fn elevated_mesh_carries_sampled_elevation() {
        let b = Bounds::of_tile(8, 130, 90);
        // A constant 1000 m surface: every vertex z == 1000 m, normals point up.
        let (m, emin, emax) = elevated_mesh(16, &b, |_, _| 1000.0);
        let n = 17usize;
        assert_eq!(m.x.len(), n * n);
        assert_eq!(m.z.len(), n * n);
        assert_eq!(m.normals.len(), n * n * 2);
        assert!(m.z.iter().all(|&z| z == project::quantize_z(1000.0)));
        assert_eq!((emin, emax), (1000.0, 1000.0));
        // Shares flat_mesh's horizontal grid: tile-proper edges at 16384/49152.
        assert_eq!(*m.x.iter().min().unwrap(), 16384);
        assert_eq!(*m.x.iter().max().unwrap(), 49152);
        // A constant-elevation surface's normal is the geodetic "up" (ECEF
        // radial) at each vertex. Decode the centre vertex and check it points
        // up at the tile centre's lon/lat.
        let centre = (n / 2) * n + n / 2;
        let (nx, ny, nz) = decode_octahedral(m.normals[centre * 2], m.normals[centre * 2 + 1]);
        let lon = ((b.west + b.east) * 0.5).to_radians();
        let lat = ((b.south + b.north) * 0.5).to_radians();
        let up = (lat.cos() * lon.cos(), lat.cos() * lon.sin(), lat.sin());
        let dot = nx * up.0 + ny * up.1 + nz * up.2;
        assert!(dot > 0.999, "normal should point up, dot={dot}");
    }

    /// Inverse of [`encode_octahedral`] for tests.
    fn decode_octahedral(ox: i8, oy: i8) -> (f64, f64, f64) {
        let u = ox as f64 / 127.0;
        let v = oy as f64 / 127.0;
        let mut nx = u;
        let mut ny = v;
        let nz = 1.0 - u.abs() - v.abs();
        if nz < 0.0 {
            let old = nx;
            nx = (1.0 - ny.abs()) * if old >= 0.0 { 1.0 } else { -1.0 };
            ny = (1.0 - old.abs()) * if ny >= 0.0 { 1.0 } else { -1.0 };
        }
        let len = (nx * nx + ny * ny + nz * nz).sqrt();
        (nx / len, ny / len, nz / len)
    }

    #[test]
    fn elevated_mesh_normal_tilts_with_slope() {
        let b = Bounds::of_tile(10, 500, 500);
        // A west→east ramp produces normals tilted off vertical (not (0,0)).
        let (m, _, _) = elevated_mesh(8, &b, |lon, _| (lon - b.west) * 100_000.0);
        assert!(m.normals.iter().any(|&c| c != 0));
    }

    #[test]
    fn surface_height_matches_the_mesh_vertices_and_ramp() {
        for grid in [TERRAIN_GRID, TERRAIN_GRID_DETAIL] {
            let b = Bounds::of_tile(14, 8500, 5800);
            // A planar west→east + south→north ramp. surface_height linearly
            // interpolates the lattice, so on a planar field it is exact everywhere.
            let field = |lon: f64, lat: f64| (lon - b.west) * 1.0e5 + (lat - b.south) * 3.0e5;
            let cell_lon = b.width() / grid as f64;
            let cell_lat = b.height() / grid as f64;
            // At a grid vertex it equals the sample.
            let (vlon, vlat) = (b.west + 5.0 * cell_lon, b.south + 7.0 * cell_lat);
            let h = surface_height(&b, grid, vlon, vlat, &mut |lo, la| field(lo, la));
            assert!((h - field(vlon, vlat)).abs() < 1e-6);
            // Mid-cell, both above and below the diagonal, still exact on a plane.
            for (fx, fy) in [(0.7, 0.2), (0.2, 0.7)] {
                let lon = b.west + (3.0 + fx) * cell_lon;
                let lat = b.south + (3.0 + fy) * cell_lat;
                let got = surface_height(&b, grid, lon, lat, &mut |lo, la| field(lo, la));
                assert!((got - field(lon, lat)).abs() < 1e-6, "grid={grid} fx={fx} fy={fy} got {got}");
            }
        }
    }

    #[test]
    fn surface_height_is_continuous_across_tiles() {
        // A point sampled from two adjacent tiles' frames yields the same height,
        // because the lattice is global (a tile spans exactly `grid` cells).
        for grid in [TERRAIN_GRID, TERRAIN_GRID_DETAIL] {
            let west = Bounds::of_tile(14, 8500, 5800);
            let east = Bounds::of_tile(14, 8501, 5800);
            let lon = east.west + 0.3 * (east.width() / grid as f64);
            let lat = east.south + 0.4 * (east.height() / grid as f64);
            // A non-planar field so interpolation is frame-sensitive unless aligned.
            let field = |lo: f64, la: f64| (lo * 12.0).sin() * 100.0 + (la * 9.0).cos() * 60.0;
            let from_east = surface_height(&east, grid, lon, lat, &mut |a, b| field(a, b));
            let from_west = surface_height(&west, grid, lon, lat, &mut |a, b| field(a, b));
            assert!(
                (from_east - from_west).abs() < 1e-6,
                "grid={grid} east {from_east} west {from_west}"
            );
        }
    }

    #[test]
    fn grid_for_switches_at_the_reference_zoom() {
        // The ladder grades down a rung at a time instead of collapsing.
        assert_eq!(grid_for(15, 16), TERRAIN_GRID_DETAIL / 2);
        assert_eq!(grid_for(14, 16), TERRAIN_GRID_DETAIL / 4);
        assert_eq!(grid_for(13, 16), TERRAIN_GRID);
        assert_eq!(grid_for(4, 16), TERRAIN_GRID);
        for z in 0..=16u8 {
            assert_eq!(EXTENT as u32 % grid_for(z, 16), 0, "grid must divide the extent");
        }
        assert_eq!(grid_for(16, 16), TERRAIN_GRID_DETAIL);
        assert_eq!(grid_for(16, 14), TERRAIN_GRID_DETAIL);
        // Both resolutions divide the quantized extent exactly.
        assert_eq!(EXTENT as u32 % TERRAIN_GRID, 0);
        assert_eq!(EXTENT as u32 % TERRAIN_GRID_DETAIL, 0);
    }
}
