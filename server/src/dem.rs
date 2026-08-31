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
//!
//! Decoding a 512² lossless-WebP tile costs milliseconds — orders of magnitude
//! more than sampling it — so the decoded tiles are held in one process-wide
//! LRU cache shared by every [`Dem`] handle [`fork`](Dem::fork)ed from the
//! same archive: parallel workers walk neighbouring output tiles and keep
//! needing the same source tiles, and sharing turns *threads × tiles* decodes
//! into *tiles*.

use std::collections::{HashMap, VecDeque};
use std::f64::consts::PI;
use std::io;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex, OnceLock};

use crate::pmtiles::Pmtiles;

/// Web Mercator latitude limit (the square Mercator extent).
const MERCATOR_LAT_LIMIT: f64 = 85.051_128_779_806_59;
/// Terrarium tiles are square; Mapterhorn uses 512 px.
const TILE_PX: usize = 512;
/// Shared decoded-tile cache capacity (each entry is `TILE_PX² f32` ≈ 1 MiB).
/// Sized so one output tile's working set — its own zoom's tiles plus the
/// reference lattice's, straddling up to four source tiles each — stays
/// resident across every worker while they walk neighbouring output tiles.
const CACHE_CAP: usize = 256;
/// Per-handle front cache of recently used slots, checked without taking the
/// shared lock. Queries cluster heavily (a building footprint, a road run),
/// so a handful of entries absorbs almost all lookups.
const RECENT_CAP: usize = 8;

/// Process-wide count of decode attempts (cache misses), for run stats.
pub static DECODES: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

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

/// One cache slot: decoded at most once however many handles race on it (the
/// `OnceLock` serializes only the racers on this one tile), `None` for a
/// missing or undecodable tile so repeated ocean misses stay cheap.
type Slot = Arc<OnceLock<Option<ElevTile>>>;

/// The shared decoded-tile cache: an LRU keyed by `(z, x, y)`. Slots are
/// handed out under a brief lock; decoding happens outside it. An evicted
/// slot stays alive for whoever still holds its `Arc`.
struct DemCache {
    inner: Mutex<DemCacheInner>,
}

#[derive(Default)]
struct DemCacheInner {
    map: HashMap<(u8, u32, u32), Slot>,
    /// Recency queue (front = coldest).
    order: VecDeque<(u8, u32, u32)>,
}

impl DemCache {
    fn slot(&self, key: (u8, u32, u32)) -> Slot {
        let mut inner = self.inner.lock().expect("dem cache poisoned");
        if let Some(slot) = inner.map.get(&key) {
            let slot = Arc::clone(slot);
            if let Some(pos) = inner.order.iter().position(|k| *k == key) {
                inner.order.remove(pos);
                inner.order.push_back(key);
            }
            return slot;
        }
        if inner.map.len() >= CACHE_CAP {
            if let Some(old) = inner.order.pop_front() {
                inner.map.remove(&old);
            }
        }
        let slot: Slot = Arc::new(OnceLock::new());
        inner.map.insert(key, Arc::clone(&slot));
        inner.order.push_back(key);
        slot
    }
}

/// A DEM sampler over an opened Mapterhorn PMTiles archive. Each handle owns
/// its file descriptor (PMTiles reads seek); the decoded tiles live in the
/// cache shared across all handles forked from the first.
pub struct Dem {
    path: PathBuf,
    archive: Pmtiles,
    cache: Arc<DemCache>,
    /// Lock-free MRU front cache of `(key, slot)` pairs (see [`RECENT_CAP`]).
    recent: Vec<((u8, u32, u32), Slot)>,
}

impl Dem {
    /// Opens a Terrarium PMTiles archive (WebP or PNG tile data).
    pub fn open(path: &Path) -> io::Result<Dem> {
        Ok(Dem {
            path: path.to_path_buf(),
            archive: Pmtiles::open(path)?,
            cache: Arc::new(DemCache { inner: Mutex::new(DemCacheInner::default()) }),
            recent: Vec::new(),
        })
    }

    /// Another handle onto the same archive — its own file descriptor,
    /// sharing this handle's decoded-tile cache. One handle per worker.
    pub fn fork(&self) -> io::Result<Dem> {
        Ok(Dem {
            path: self.path.clone(),
            archive: Pmtiles::open(&self.path)?,
            cache: Arc::clone(&self.cache),
            recent: Vec::new(),
        })
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
        self.imaged(lon, lat, out_zoom).unwrap_or(0.0)
    }

    /// Like [`Dem::elevation`], but `None` where the archive has no coverage
    /// instead of the flat 0 fallback. For callers deriving a *level* from a
    /// set of samples (a water surface read along a shoreline): a bbox-clipped
    /// extract images only part of a big lake's shore, and a gap mistaken for
    /// sea level drags a low-percentile statistic to 0 — a 372 m cliff drawn
    /// along the waterline.
    pub fn imaged(&mut self, lon: f64, lat: f64, out_zoom: u8) -> Option<f64> {
        if !(-MERCATOR_LAT_LIMIT..=MERCATOR_LAT_LIMIT).contains(&lat) {
            return None;
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

        let slot = self.slot((z, tx, ty));
        let archive = &mut self.archive;
        let tile = slot.get_or_init(|| {
            DECODES.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            archive.tile(z, tx, ty).ok().flatten().and_then(|bytes| decode_terrarium(&bytes))
        });
        tile.as_ref().map(|t| t.sample(px, py))
    }

    /// The cache slot for a source tile: the per-handle front cache first,
    /// then the shared LRU.
    fn slot(&mut self, key: (u8, u32, u32)) -> Slot {
        if let Some(pos) = self.recent.iter().position(|(k, _)| *k == key) {
            if pos != 0 {
                self.recent[..=pos].rotate_right(1);
            }
            return Arc::clone(&self.recent[0].1);
        }
        let slot = self.cache.slot(key);
        self.recent.insert(0, (key, Arc::clone(&slot)));
        self.recent.truncate(RECENT_CAP);
        slot
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
