//! WGS84 ↔ tile-local quantized coordinate conversion (see FORMAT.md §5).
//!
//! Coordinate space: extent 32768 with a 16384-unit buffer per side, so the
//! uint16 range `[0, 65535]` is partitioned as `16384 + 32768 + 16384`. The
//! tile proper spans raw values `[16384, 49151]`; values outside are buffer
//! geometry beyond the tile edges.

/// Units the tile spans along each axis.
pub const EXTENT: f64 = 32768.0;
/// Buffer units of overflow on each side.
pub const BUFFER: f64 = 16384.0;

/// Geographic bounds of a tile, in WGS84 degrees.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bounds {
    pub west: f64,
    pub south: f64,
    pub east: f64,
    pub north: f64,
}

impl Bounds {
    /// The full world (the level-0 extent of the geographic quadtree).
    pub const WORLD: Bounds = Bounds { west: -180.0, south: -90.0, east: 180.0, north: 90.0 };

    /// Computes the bounds of tile `(z, x, y)`.
    ///
    /// The grid at level `z` is `2^z × 2^z` (one root tile at z0), matching the
    /// C client/server (`tiler/src/clip.c arpt_tile_bounds`): lon span
    /// `360/2^z`, lat span `180/2^z`. (FORMAT.md §2 says `2^(z+1)` columns but
    /// is out of date — the client uses this scheme.)
    pub fn of_tile(z: u8, x: u32, y: u32) -> Bounds {
        let n = (1u64 << z as u32) as f64;
        let tile_w = 360.0 / n;
        let tile_h = 180.0 / n;
        let west = -180.0 + x as f64 * tile_w;
        let south = -90.0 + y as f64 * tile_h;
        Bounds { west, south, east: west + tile_w, north: south + tile_h }
    }

    pub fn width(&self) -> f64 {
        self.east - self.west
    }

    pub fn height(&self) -> f64 {
        self.north - self.south
    }

    /// Expands the bounds outward by `frac` of its size on each side.
    ///
    /// The tile clip rect is the tile bounds expanded by [`BUFFER`]/[`EXTENT`]
    /// (= 0.5) per side, matching the format's buffer zone (FORMAT.md §5).
    pub fn expanded(&self, frac: f64) -> Bounds {
        let dx = self.width() * frac;
        let dy = self.height() * frac;
        Bounds {
            west: self.west - dx,
            south: self.south - dy,
            east: self.east + dx,
            north: self.north + dy,
        }
    }

    /// Whether a point lies within the bounds (inclusive of edges).
    pub fn contains(&self, x: f64, y: f64) -> bool {
        x >= self.west && x <= self.east && y >= self.south && y <= self.north
    }
}

/// Quantizes a longitude to a tile-local x value, clamped to `[0, 65535]`.
pub fn quantize_x(lon: f64, b: &Bounds) -> u16 {
    quantize(lon, b.west, b.width())
}

/// Quantizes a latitude to a tile-local y value, clamped to `[0, 65535]`.
pub fn quantize_y(lat: f64, b: &Bounds) -> u16 {
    quantize(lat, b.south, b.height())
}

fn quantize(v: f64, origin: f64, span: f64) -> u16 {
    let t = (v - origin) / span; // 0..1 across the tile proper
    let q = BUFFER + (t * EXTENT).round();
    q.clamp(0.0, 65535.0) as u16
}

/// Quantizes an already-normalized tile coordinate `t` (0 at the west/south
/// edge, 1 at the east/north edge) to a tile-local uint16, clamped to
/// `[0, 65535]`. Equivalent to the C `arpt_quantize`.
pub fn quantize_unit(t: f64) -> u16 {
    quantize(t, 0.0, 1.0)
}

/// Dequantizes a tile-local x back to longitude (FORMAT.md §5).
pub fn dequantize_x(qx: u16, b: &Bounds) -> f64 {
    b.west + ((qx as f64 - BUFFER) / EXTENT) * b.width()
}

/// Dequantizes a tile-local y back to latitude.
pub fn dequantize_y(qy: u16, b: &Bounds) -> f64 {
    b.south + ((qy as f64 - BUFFER) / EXTENT) * b.height()
}

/// Quantizes an elevation in metres to int32 millimetres above the ellipsoid.
pub fn quantize_z(altitude_m: f64) -> i32 {
    (altitude_m * 1000.0).round() as i32
}

/// Dequantizes int32 millimetres back to metres.
pub fn dequantize_z(z: i32) -> f64 {
    z as f64 * 0.001
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn root_tile_covers_world() {
        // Level 0 is a single tile covering the whole world (2^0 × 2^0).
        let root = Bounds::of_tile(0, 0, 0);
        assert_eq!(root, Bounds { west: -180.0, south: -90.0, east: 180.0, north: 90.0 });
        // Level 1 splits into 2×2 tiles of 180° × 90°.
        let nw = Bounds::of_tile(1, 0, 1);
        assert_eq!(nw, Bounds { west: -180.0, south: 0.0, east: 0.0, north: 90.0 });
    }

    #[test]
    fn tile_proper_endpoints_map_to_buffer_offsets() {
        let b = Bounds::of_tile(5, 10, 7);
        // West/south edge -> raw 16384; east/north edge -> raw 49152.
        assert_eq!(quantize_x(b.west, &b), 16384);
        assert_eq!(quantize_y(b.south, &b), 16384);
        assert_eq!(quantize_x(b.east, &b), 16384 + 32768);
        assert_eq!(quantize_y(b.north, &b), 16384 + 32768);
    }

    #[test]
    fn quantize_dequantize_roundtrip_within_one_unit() {
        let b = Bounds::of_tile(12, 2000, 1500);
        let lon = b.west + 0.37 * b.width();
        let lat = b.south + 0.62 * b.height();
        let qx = quantize_x(lon, &b);
        let qy = quantize_y(lat, &b);
        // One quant unit of tolerance.
        let tol_lon = b.width() / EXTENT;
        let tol_lat = b.height() / EXTENT;
        assert!((dequantize_x(qx, &b) - lon).abs() <= tol_lon);
        assert!((dequantize_y(qy, &b) - lat).abs() <= tol_lat);
    }

    #[test]
    fn out_of_range_clamps_not_panics() {
        let b = Bounds::of_tile(3, 0, 0);
        // Far west of the tile, well past the buffer -> clamps to 0.
        assert_eq!(quantize_x(b.west - 10.0 * b.width(), &b), 0);
        // Far east -> clamps to u16::MAX.
        assert_eq!(quantize_x(b.east + 10.0 * b.width(), &b), 65535);
    }

    #[test]
    fn elevation_roundtrip() {
        assert_eq!(quantize_z(123.456), 123456);
        assert!((dequantize_z(quantize_z(123.456)) - 123.456).abs() < 1e-9);
    }
}
