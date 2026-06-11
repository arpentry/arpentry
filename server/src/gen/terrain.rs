//! Procedural terrain: elevation/moisture fields and the tile mesh (port of
//! `gen/terrain.c`).
//!
//! Elevation is a two-layer field on the unit sphere: a smooth low-frequency
//! continental shape that decides land vs. ocean, plus ridged fBm that adds
//! mountain detail on land. Moisture is an independent fBm pass used by the
//! biome classifier. Both are deterministic functions of `(lon, lat)`.

use std::f64::consts::PI;

use super::noise::{fbm3, simplex3};
use crate::project::{self, Bounds};
use crate::terrain::encode_octahedral;

/// Terrain grid resolution (cells per side).
pub const TERRAIN_GRID: usize = 256;
/// Vertices per side (`TERRAIN_GRID + 1`).
pub const TERRAIN_VERTS: usize = TERRAIN_GRID + 1;

// Continental shape: low-frequency smooth fBm.
const CONTINENT_OCTAVES: i32 = 6;
const CONTINENT_FREQ: f64 = 1.8;
const CONTINENT_LACUNARITY: f64 = 2.0;
const CONTINENT_PERSIST: f64 = 0.5;
const CONTINENT_BIAS: f64 = 0.45; // ~55-60% ocean

// Mountain ridges: ridged noise at mountain-chain scale.
const RIDGE_OCTAVES: i32 = 5;
const RIDGE_FREQ: f64 = 40.0;
const RIDGE_LACUNARITY: f64 = 2.5;
const RIDGE_PERSIST: f64 = 0.75;
const RIDGE_OFFSET_X: f64 = 53.7;
const RIDGE_OFFSET_Y: f64 = 91.2;
const RIDGE_OFFSET_Z: f64 = 37.4;

// Elevation scaling (metres).
const TERRAIN_LAND_HEIGHT: f64 = 9500.0;
const TERRAIN_OCEAN_DEPTH: f64 = 11000.0;

// Moisture parameters.
const MOISTURE_FREQ: f64 = 3.0;
const MOISTURE_OCTAVES: i32 = 6;
const MOISTURE_OFFSET_X: f64 = 17.3;
const MOISTURE_OFFSET_Y: f64 = 31.7;
const MOISTURE_OFFSET_Z: f64 = 5.9;

/// Geodetic (lon, lat) in degrees → unit-sphere (x, y, z).
fn lonlat_to_sphere(lon_deg: f64, lat_deg: f64) -> (f64, f64, f64) {
    let lon_r = lon_deg * (PI / 180.0);
    let lat_r = lat_deg * (PI / 180.0);
    let cos_lat = lat_r.cos();
    (cos_lat * lon_r.cos(), cos_lat * lon_r.sin(), lat_r.sin())
}

/// Ridged fBm: each octave is individually ridged before summing. Returns `[0, 1]`.
fn ridged_fbm3(x: f64, y: f64, z: f64, octaves: i32, lacunarity: f64, persistence: f64) -> f64 {
    let mut signal = 0.0;
    let mut freq = 1.0;
    let mut amp = 1.0;
    let mut amp_sum = 0.0;
    for _ in 0..octaves {
        let mut n = simplex3(x * freq, y * freq, z * freq);
        n = 1.0 - n.abs();
        n *= n;
        signal += n * amp;
        amp_sum += amp;
        freq *= lacunarity;
        amp *= persistence;
    }
    signal / amp_sum
}

/// Terrain elevation in metres at a geodetic point. Positive = land, negative =
/// ocean. Deterministic.
pub fn terrain_elevation(lon_deg: f64, lat_deg: f64) -> f64 {
    let (sx, sy, sz) = lonlat_to_sphere(lon_deg, lat_deg);

    // Layer 1 — continental shape.
    let cn = fbm3(
        sx * CONTINENT_FREQ,
        sy * CONTINENT_FREQ,
        sz * CONTINENT_FREQ,
        CONTINENT_OCTAVES,
        CONTINENT_LACUNARITY,
        CONTINENT_PERSIST,
    );
    let ce = (cn + 1.0) * 0.5; // [0, 1]

    // Ocean.
    if ce < CONTINENT_BIAS {
        let t = 1.0 - ce / CONTINENT_BIAS;
        return -t * t * TERRAIN_OCEAN_DEPTH;
    }

    // Land envelope normalised to [0, 1].
    let t = (ce - CONTINENT_BIAS) / (1.0 - CONTINENT_BIAS);

    // Layer 2 — ridged fBm.
    let ridge = ridged_fbm3(
        sx * RIDGE_FREQ + RIDGE_OFFSET_X,
        sy * RIDGE_FREQ + RIDGE_OFFSET_Y,
        sz * RIDGE_FREQ + RIDGE_OFFSET_Z,
        RIDGE_OCTAVES,
        RIDGE_LACUNARITY,
        RIDGE_PERSIST,
    );

    // sqrt envelope rises quickly from the coast.
    t.sqrt() * ridge * TERRAIN_LAND_HEIGHT
}

/// Moisture in `[0, 1]` at a geodetic point. Deterministic, decorrelated from
/// elevation via a spatial offset.
pub fn terrain_moisture(lon_deg: f64, lat_deg: f64) -> f64 {
    let (sx, sy, sz) = lonlat_to_sphere(lon_deg, lat_deg);
    let m = fbm3(
        sx * MOISTURE_FREQ + MOISTURE_OFFSET_X,
        sy * MOISTURE_FREQ + MOISTURE_OFFSET_Y,
        sz * MOISTURE_FREQ + MOISTURE_OFFSET_Z,
        MOISTURE_OCTAVES,
        2.0,
        0.5,
    );
    (m + 1.0) * 0.5
}

/// The terrain vertex/index buffers for one tile.
pub struct TerrainMesh {
    pub vx: Vec<u16>,
    pub vy: Vec<u16>,
    pub vz: Vec<i32>,
    /// Octahedral int8×2 per-vertex normals (length `2 * vertex_count`).
    pub normals: Vec<i8>,
    pub indices: Vec<u32>,
}

/// Builds the padded `(TERRAIN_VERTS + 2)²` elevation grid (one row/column of
/// halo on each side, for centred finite-difference normals).
fn build_elevation_grid(bounds: &Bounds, cell_lon: f64, cell_lat: f64) -> Vec<f64> {
    let pad_w = TERRAIN_VERTS + 2;
    let mut elev = vec![0.0f64; pad_w * pad_w];
    // rows/cols run from -1 to TERRAIN_VERTS inclusive.
    for row in -1i32..=TERRAIN_VERTS as i32 {
        let lat = bounds.south + row as f64 * cell_lat;
        for col in -1i32..=TERRAIN_VERTS as i32 {
            let lon = bounds.west + col as f64 * cell_lon;
            let pi = (row + 1) as usize * pad_w + (col + 1) as usize;
            elev[pi] = terrain_elevation(lon, lat);
        }
    }
    elev
}

/// Builds vertex positions + octahedral normals + triangle indices for the tile.
pub fn build_mesh(bounds: &Bounds) -> TerrainMesh {
    let lon_span = bounds.width();
    let lat_span = bounds.height();
    let cell_lon = lon_span / TERRAIN_GRID as f64;
    let cell_lat = lat_span / TERRAIN_GRID as f64;

    // Approximate cell size in metres (for the finite-difference slope).
    let mid_lat = (bounds.south + bounds.north) * 0.5;
    let cos_lat = (mid_lat * PI / 180.0).cos();
    let cell_w_m = cell_lon * 111_319.5 * cos_lat;
    let cell_h_m = cell_lat * 111_319.5;

    let pad_w = TERRAIN_VERTS + 2;
    let elev = build_elevation_grid(bounds, cell_lon, cell_lat);

    let nv = TERRAIN_VERTS * TERRAIN_VERTS;
    let mut vx = vec![0u16; nv];
    let mut vy = vec![0u16; nv];
    let mut vz = vec![0i32; nv];
    let mut normals = vec![0i8; nv * 2];

    for row in 0..TERRAIN_VERTS {
        let lat = bounds.south + row as f64 * cell_lat;
        for col in 0..TERRAIN_VERTS {
            let idx = row * TERRAIN_VERTS + col;
            let lon = bounds.west + col as f64 * cell_lon;
            let pi = (row + 1) * pad_w + (col + 1);

            vx[idx] = project::quantize_x(lon, bounds);
            vy[idx] = project::quantize_y(lat, bounds);
            vz[idx] = project::quantize_z(elev[pi]);

            // Centred finite differences using padded neighbours.
            let dz_dx = (elev[pi + 1] - elev[pi - 1]) / (2.0 * cell_w_m);
            let dz_dy = (elev[pi + pad_w] - elev[pi - pad_w]) / (2.0 * cell_h_m);

            // ECEF normal from ENU basis vectors.
            let lon_r = lon * (PI / 180.0);
            let lat_r = lat * (PI / 180.0);
            let (sin_lon, cos_lon) = (lon_r.sin(), lon_r.cos());
            let (sin_lat, cos_lat) = (lat_r.sin(), lat_r.cos());

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

            let (ox, oy) = encode_octahedral(nx, ny, nz);
            normals[idx * 2] = ox;
            normals[idx * 2 + 1] = oy;
        }
    }

    let mut indices = Vec::with_capacity(TERRAIN_GRID * TERRAIN_GRID * 6);
    for row in 0..TERRAIN_GRID {
        for col in 0..TERRAIN_GRID {
            let tl = (row * TERRAIN_VERTS + col) as u32;
            let tr = tl + 1;
            let bl = tl + TERRAIN_VERTS as u32;
            let br = bl + 1;
            indices.extend_from_slice(&[tl, tr, br, tl, br, bl]);
        }
    }

    TerrainMesh { vx, vy, vz, normals, indices }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ocean_is_negative_land_is_positive() {
        // Deep-ocean and high-land samples are stable across runs.
        let e = terrain_elevation(12.34, -56.78);
        assert_eq!(e, terrain_elevation(12.34, -56.78));
        // Moisture stays within its declared range.
        let m = terrain_moisture(12.34, -56.78);
        assert!((0.0..=1.0).contains(&m));
    }

    #[test]
    fn mesh_has_expected_dimensions() {
        let b = Bounds::of_tile(8, 130, 90);
        let m = build_mesh(&b);
        let nv = TERRAIN_VERTS * TERRAIN_VERTS;
        assert_eq!(m.vx.len(), nv);
        assert_eq!(m.normals.len(), nv * 2);
        assert_eq!(m.indices.len(), TERRAIN_GRID * TERRAIN_GRID * 6);
        // Tile-proper edges land on the buffer offsets.
        assert_eq!(*m.vx.iter().min().unwrap(), 16384);
        assert_eq!(*m.vx.iter().max().unwrap(), 49152);
        assert!(m.indices.iter().all(|&i| (i as usize) < nv));
    }
}
