//! Digital-elevation sampling from a Mapterhorn PMTiles archive.
//!
//! Mapterhorn distributes global terrain as Terrarium-encoded RGB tiles (512 px
//! WebP) in a Web-Mercator PMTiles pyramid (`planet.pmtiles`, z0–12). This
//! module turns that into an `elevation(lon, lat)` query: it picks a source zoom
//! for the output tile, reprojects the WGS84 point into Web Mercator, reads the
//! covering DEM tile (cached, decoded once), and bilinearly samples it.
//!
//! Terrarium decode (tilezen/joerd): `elev_m = R*256 + G + B/256 − 32768`.
//!
//! The arpentry tiler uses a WGS84 geographic tiling while Mapterhorn is Web
//! Mercator, so sampling reprojects per point rather than reusing tile indices.

use std::collections::{HashMap, VecDeque};
use std::f64::consts::PI;
use std::io;
use std::path::Path;

use crate::pmtiles::Pmtiles;

/// Web Mercator latitude limit (the square Mercator extent).
const MERCATOR_LAT_LIMIT: f64 = 85.051_128_779_806_59;
/// Terrarium tiles are square; Mapterhorn uses 512 px.
const TILE_PX: usize = 512;
/// Decoded-tile cache capacity (each entry is `TILE_PX² f32` ≈ 1 MiB).
const CACHE_CAP: usize = 32;

/// A decoded Terrarium tile: row-major elevations in metres.
struct ElevTile {
    elev: Vec<f32>,
}

impl ElevTile {
    /// Samples the tile at fractional pixel `(px, py)` with bilinear
    /// interpolation, clamping to the tile edge.
    fn sample(&self, px: f64, py: f64) -> f64 {
        let max = (TILE_PX - 1) as f64;
        let fx = px.clamp(0.0, max);
        let fy = py.clamp(0.0, max);
        let x0 = fx.floor() as usize;
        let y0 = fy.floor() as usize;
        let x1 = (x0 + 1).min(TILE_PX - 1);
        let y1 = (y0 + 1).min(TILE_PX - 1);
        let tx = fx - x0 as f64;
        let ty = fy - y0 as f64;
        let at = |x: usize, y: usize| self.elev[y * TILE_PX + x] as f64;
        let top = at(x0, y0) * (1.0 - tx) + at(x1, y0) * tx;
        let bot = at(x0, y1) * (1.0 - tx) + at(x1, y1) * tx;
        top * (1.0 - ty) + bot * ty
    }
}

/// A DEM sampler over an opened Mapterhorn PMTiles archive, with a small
/// decoded-tile cache (output tiles are visited in spatial order, so a handful
/// of recently used DEM tiles serves most vertex queries).
pub struct Dem {
    archive: Pmtiles,
    cache: HashMap<(u8, u32, u32), Option<ElevTile>>,
    /// Insertion-order queue for cache eviction (front = oldest).
    order: VecDeque<(u8, u32, u32)>,
}

impl Dem {
    /// Opens a Terrarium PMTiles archive (WebP or PNG tile data).
    pub fn open(path: &Path) -> io::Result<Dem> {
        let archive = Pmtiles::open(path)?;
        Ok(Dem { archive, cache: HashMap::new(), order: VecDeque::new() })
    }

    /// Source zoom to sample for an output tile at zoom `out_zoom`: matched to
    /// the output zoom but clamped to what the archive actually contains.
    fn source_zoom(&self, out_zoom: u8) -> u8 {
        out_zoom.clamp(self.archive.min_zoom, self.archive.max_zoom)
    }

    /// Elevation in metres above the ellipsoid at `(lon, lat)`, sampling the DEM
    /// at the zoom appropriate for an output tile of zoom `out_zoom`. Returns 0
    /// where the archive has no coverage (e.g. beyond the Mercator latitude
    /// limit or in a gap), so callers get a flat sea-level surface there.
    pub fn elevation(&mut self, lon: f64, lat: f64, out_zoom: u8) -> f64 {
        if !(-MERCATOR_LAT_LIMIT..=MERCATOR_LAT_LIMIT).contains(&lat) {
            return 0.0;
        }
        let z = self.source_zoom(out_zoom);
        let n = (1u64 << z as u32) as f64;

        // Web Mercator pixel coordinates across the whole pyramid level.
        let world_x = (lon + 180.0) / 360.0 * n;
        let lat_r = lat.to_radians();
        let world_y = (1.0 - (lat_r.tan() + 1.0 / lat_r.cos()).ln() / PI) / 2.0 * n;

        let tx = (world_x.floor() as i64).clamp(0, n as i64 - 1) as u32;
        let ty = (world_y.floor() as i64).clamp(0, n as i64 - 1) as u32;
        let px = (world_x - tx as f64) * TILE_PX as f64;
        let py = (world_y - ty as f64) * TILE_PX as f64;

        match self.tile(z, tx, ty) {
            Some(t) => t.sample(px, py),
            None => 0.0,
        }
    }

    /// Returns the decoded tile for `(z, x, y)`, loading and caching it on first
    /// use. The cache stores `None` for missing tiles too, so repeated misses in
    /// an ocean tile don't re-hit the archive.
    fn tile(&mut self, z: u8, x: u32, y: u32) -> Option<&ElevTile> {
        let key = (z, x, y);
        if !self.cache.contains_key(&key) {
            let decoded = self
                .archive
                .tile(z, x, y)
                .ok()
                .flatten()
                .and_then(|bytes| decode_terrarium(&bytes));
            self.insert(key, decoded);
        }
        self.cache.get(&key).and_then(|o| o.as_ref())
    }

    fn insert(&mut self, key: (u8, u32, u32), value: Option<ElevTile>) {
        if self.cache.len() >= CACHE_CAP {
            if let Some(old) = self.order.pop_front() {
                self.cache.remove(&old);
            }
        }
        self.cache.insert(key, value);
        self.order.push_back(key);
    }
}

/// Decodes a Terrarium-encoded image (WebP or PNG) into per-pixel metres.
/// Returns `None` if the bytes don't decode or aren't a 512×512 tile.
fn decode_terrarium(bytes: &[u8]) -> Option<ElevTile> {
    let img = image::load_from_memory(bytes).ok()?.to_rgb8();
    if img.width() as usize != TILE_PX || img.height() as usize != TILE_PX {
        return None;
    }
    let mut elev = Vec::with_capacity(TILE_PX * TILE_PX);
    for p in img.pixels() {
        let [r, g, b] = p.0;
        elev.push((r as f32 * 256.0 + g as f32 + b as f32 / 256.0) - 32768.0);
    }
    Some(ElevTile { elev })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn terrarium_decodes_reference_pixel() {
        // rgb(137, 219, 68) -> 2523.265625 m (tilezen/joerd reference).
        let mut buf = vec![0u8; TILE_PX * TILE_PX * 3];
        for px in buf.chunks_mut(3) {
            px.copy_from_slice(&[137, 219, 68]);
        }
        let img = image::RgbImage::from_raw(TILE_PX as u32, TILE_PX as u32, buf).unwrap();
        let mut png = Vec::new();
        image::DynamicImage::ImageRgb8(img)
            .write_to(&mut std::io::Cursor::new(&mut png), image::ImageFormat::Png)
            .unwrap();
        let tile = decode_terrarium(&png).unwrap();
        assert!((tile.sample(10.0, 10.0) - 2523.265_625).abs() < 1e-3);
    }

    #[test]
    fn bilinear_interpolates_between_corners() {
        let mut elev = vec![0.0f32; TILE_PX * TILE_PX];
        elev[0] = 0.0; // (0,0)
        elev[1] = 100.0; // (1,0)
        let t = ElevTile { elev };
        // Halfway between the two known pixels on row 0.
        assert!((t.sample(0.5, 0.0) - 50.0).abs() < 1e-6);
    }
}
