//! Digital-elevation sampling from a Mapterhorn PMTiles archive.
//!
//! Mapterhorn distributes global terrain as Terrarium-encoded RGB tiles (512 px
//! WebP) in a Web-Mercator PMTiles pyramid (`planet.pmtiles`, z0–12). This
//! module turns that into an `elevation(lon, lat)` query: it picks a source zoom
//! for the output tile, reprojects the WGS84 point into Web Mercator, reads the
//! covering DEM tiles (cached, decoded once), and samples them bicubically in
//! the pixel grid of the whole pyramid level, so a tile boundary is invisible
//! in the result and the reconstructed ground is C1 (see [`bicubic`]).
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
//!
//! A `--terrain` value that is not a path is an analytic ground instead — see
//! [`field`], the isolation harness's terrain dial. [`Dem`] is the seam that
//! makes that free: it is one facade over two sources, so the fifteen places
//! that open a DEM never learn which one they got.

mod field;
pub use field::Field;

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
    /// The elevation of one pixel of this tile. Both indices must be in
    /// `0..TILE_PX`; the caller owns the clamping, because it is the only one
    /// that knows whether an out-of-range index belongs to a neighbouring tile
    /// (fetch it) or to the edge of the archive (hold the edge).
    fn at(&self, i: usize, j: usize) -> f64 {
        self.elev[j * TILE_PX + i] as f64
    }
}

/// The distinct tile indices covering global pixel range `lo..=hi`, each
/// clamped into the level's `0..=last`. At most two: a 4-wide support cannot
/// straddle more than one tile boundary.
fn tile_span(lo: i64, hi: i64, last: i64) -> Vec<u32> {
    let a = (lo.clamp(0, last) / TILE_PX as i64) as u32;
    let b = (hi.clamp(0, last) / TILE_PX as i64) as u32;
    if a == b {
        vec![a]
    } else {
        vec![a, b]
    }
}

/// The Catmull-Rom cubic through `p1` and `p2`, shaped by their outer
/// neighbours, at `t` in `0..1`.
fn catmull_rom(p0: f64, p1: f64, p2: f64, p3: f64, t: f64) -> f64 {
    0.5 * ((2.0 * p1)
        + (-p0 + p2) * t
        + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t * t
        + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t * t * t)
}

/// Bicubic Catmull-Rom over a 4×4 pixel neighbourhood, `p[row][col]`, at
/// `(tu, tv)` in `0..1` measured from the inner pixel `p[1][1]`.
///
/// **Why not bilinear.** Bilinear is C0: the gradient jumps across every
/// pixel boundary, so the reconstructed ground is a quilt of patches creased
/// along the DEM's own pixel grid. Measured on the montreux z14 archive the
/// crease is 2.8° of normal angle at the median and 24° at the worst. A
/// lattice as coarse as the pixels cannot resolve those creases and they go
/// unnoticed; refine the lattice to get a smoother world and they become
/// geometry, and the ground reads as blocky at 3.3 m however many triangles
/// it is given. Catmull-Rom is C1, which drops the same measurement to 0.11°
/// median / 0.82° worst — the residual being the tolerance of the probe, not
/// a crease.
///
/// It interpolates: at a pixel position it returns that pixel exactly, so
/// this is a smoother reading of the same data, not a smoothing of it. A
/// constant gradient comes back exactly too, so most ground — flat, or evenly
/// sloping — is untouched.
///
/// The price is overshoot: a cubic through a step rings past it. Measured
/// over 17.8 M sub-pixel positions on a 2 km block of the montreux z14
/// archive, the result leaves the range of its own 4×4 support at 0.125% of
/// them, by 2.2 cm at the median, 32 cm at p99 and 1.13 m at the worst —
/// 11.8% of the local relief where it bites. That is the right trade for
/// ground that is mostly smooth, and it is confined to the steps: a road cut,
/// a retaining wall, a cliff. Clamping the result to the support would remove
/// it, at the cost of putting a crease back exactly at those steps — a worse
/// bargain than it sounds, since 0.125% of positions creased is two orders of
/// magnitude better than bilinear's every pixel boundary.
fn bicubic(p: &[[f64; 4]; 4], tu: f64, tv: f64) -> f64 {
    let mut col = [0.0; 4];
    for (k, c) in col.iter_mut().enumerate() {
        *c = catmull_rom(p[k][0], p[k][1], p[k][2], p[k][3], tu);
    }
    catmull_rom(col[0], col[1], col[2], col[3], tv)
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

/// A ground the pipeline can sample: a Mapterhorn PMTiles archive, or an
/// analytic [`Field`].
///
/// **One facade, two sources.** The alternative — an `Option<Field>` beside
/// the `Option<PathBuf>` — would have reached every one of the fifteen places
/// that open a DEM and every struct that carries one, and each would have had
/// to decide what to do when both were set. Here the choice is made once, in
/// [`Dem::open`], from the string the run was given.
pub struct Dem {
    source: Source,
}

enum Source {
    Archive(Archive),
    Field(Field),
}

/// A DEM sampler over an opened Mapterhorn PMTiles archive. Each handle owns
/// its file descriptor (PMTiles reads seek); the decoded tiles live in the
/// cache shared across all handles forked from the first.
struct Archive {
    path: PathBuf,
    archive: Pmtiles,
    cache: Arc<DemCache>,
    /// Lock-free MRU front cache of `(key, slot)` pairs (see [`RECENT_CAP`]).
    recent: Vec<((u8, u32, u32), Slot)>,
}

impl Dem {
    /// Opens a Terrarium PMTiles archive, or builds the analytic ground a
    /// terrain spec names (`flat`, `ramp`, `hill`, `step` — see [`field`]).
    ///
    /// A spec is recognised by its kind word, before the filesystem is
    /// consulted, so a mistyped one fails here rather than reading as a
    /// missing archive — which the callers that `.ok()` this would have turned
    /// into a silent sea-level ground.
    pub fn open(path: &Path) -> io::Result<Dem> {
        if let Some(spec) = path.to_str().filter(|s| Field::is_spec(s)) {
            let field = Field::parse(spec)
                .map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;
            return Ok(Dem { source: Source::Field(field) });
        }
        Ok(Dem {
            source: Source::Archive(Archive {
                path: path.to_path_buf(),
                archive: Pmtiles::open(path)?,
                cache: Arc::new(DemCache { inner: Mutex::new(DemCacheInner::default()) }),
                recent: Vec::new(),
            }),
        })
    }

    /// Another handle onto the same ground — for an archive, its own file
    /// descriptor sharing this handle's decoded-tile cache. One per worker.
    pub fn fork(&self) -> io::Result<Dem> {
        let source = match &self.source {
            Source::Field(f) => Source::Field(*f),
            Source::Archive(a) => Source::Archive(Archive {
                path: a.path.clone(),
                archive: Pmtiles::open(&a.path)?,
                cache: Arc::clone(&a.cache),
                recent: Vec::new(),
            }),
        };
        Ok(Dem { source })
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
    ///
    /// A field has no gaps and no zoom: it answers everywhere, with the same
    /// height at every zoom, which is one fewer thing that can move under a
    /// measurement.
    pub fn imaged(&mut self, lon: f64, lat: f64, out_zoom: u8) -> Option<f64> {
        match &mut self.source {
            Source::Field(f) => Some(f.elevation(lon, lat)),
            Source::Archive(a) => a.imaged(lon, lat, out_zoom),
        }
    }

    /// The zoom an `out_zoom` query will actually be answered at: `out_zoom`
    /// clamped to the archive's own range, or `None` for a field, which has
    /// no zoom.
    ///
    /// Asking for a zoom the archive does not carry is not an error and must
    /// not be — a coarse extract is still a ground. But the clamp is silent,
    /// and a caller that believes it sampled at z16 while the archive stops
    /// at z14 has a ground four times coarser than it thinks and no way to
    /// tell: it surfaces only as a render that looks faceted. Callers that
    /// report what they built should report this.
    pub fn served_zoom(&self, out_zoom: u8) -> Option<u8> {
        match &self.source {
            Source::Field(_) => None,
            Source::Archive(a) => Some(a.source_zoom(out_zoom)),
        }
    }
}

impl Archive {
    /// Source zoom to sample for an output tile at zoom `out_zoom`: matched to
    /// the output zoom but clamped to what the archive actually contains.
    fn source_zoom(&self, out_zoom: u8) -> u8 {
        out_zoom.clamp(self.archive.min_zoom, self.archive.max_zoom)
    }

    /// Samples the pyramid level in **global pixel coordinates** — the pixel
    /// grid of the whole level, not of one tile.
    ///
    /// Which tile holds a pixel is a fact about storage, not about the ground,
    /// so the reconstruction is done here and the tile boundary disappears
    /// from the result. Sampling inside one tile and clamping its
    /// neighbourhood to its own edge, as this did before, flattened the last
    /// half-pixel of every tile and then stepped to the neighbour's first
    /// pixel: on the montreux z14 archive, a 3.3 m shelf and a 1.18 m cliff
    /// repeating on a 1686 m grid across the world. It hid inside one cell of
    /// a lattice as coarse as the pixels; it does not hide in a fine one.
    fn imaged(&mut self, lon: f64, lat: f64, out_zoom: u8) -> Option<f64> {
        if !(-MERCATOR_LAT_LIMIT..=MERCATOR_LAT_LIMIT).contains(&lat) {
            return None;
        }
        let z = self.source_zoom(out_zoom);
        let n = (1u64 << z as u32) as f64;

        // Web Mercator tile coordinates across the whole pyramid level, then
        // the pixel grid of that level.
        let world_x = (lon + 180.0) / 360.0 * n;
        let lat_r = lat.to_radians();
        let world_y = (1.0 - (lat_r.tan() + 1.0 / lat_r.cos()).ln() / PI) / 2.0 * n;
        let gx = world_x * TILE_PX as f64;
        let gy = world_y * TILE_PX as f64;
        let last = (n as i64) * TILE_PX as i64 - 1;
        let (i1, j1) = (gx.floor(), gy.floor());
        let (tu, tv) = (gx - i1, gy - j1);
        let (i1, j1) = (i1 as i64, j1 as i64);

        // The tile the point itself falls in must exist — no coverage there is
        // no answer, as before. Its neighbours are wanted but not required:
        // at the edge of a bbox-clipped extract they are absent, and holding
        // the covered tile's edge there is what the old clamp did anyway.
        let (ctx, cty) = ((i1.clamp(0, last) / TILE_PX as i64) as u32, (j1.clamp(0, last) / TILE_PX as i64) as u32);
        let centre = self.tile(z, ctx, cty);
        centre.get().and_then(|t| t.as_ref())?;

        // The 4×4 support spans at most two tiles in each direction. Fetch
        // those (usually one) up front rather than per pixel.
        let mut held: Vec<((u32, u32), Slot)> = Vec::with_capacity(4);
        let txs = tile_span(i1 - 1, i1 + 2, last);
        let tys = tile_span(j1 - 1, j1 + 2, last);
        for &ty in &tys {
            for &tx in &txs {
                if (tx, ty) == (ctx, cty) {
                    held.push(((tx, ty), Arc::clone(&centre)));
                } else {
                    let slot = self.tile(z, tx, ty);
                    held.push(((tx, ty), slot));
                }
            }
        }

        let pixel = |gi: i64, gj: i64| -> f64 {
            let (gi, gj) = (gi.clamp(0, last), gj.clamp(0, last));
            let (tx, ty) = ((gi / TILE_PX as i64) as u32, (gj / TILE_PX as i64) as u32);
            let found = held
                .iter()
                .find(|(k, _)| *k == (tx, ty))
                .and_then(|(_, s)| s.get())
                .and_then(|t| t.as_ref());
            match found {
                Some(t) => t.at((gi % TILE_PX as i64) as usize, (gj % TILE_PX as i64) as usize),
                // A missing neighbour: hold the centre tile's edge.
                None => {
                    let (lo_i, lo_j) = (ctx as i64 * TILE_PX as i64, cty as i64 * TILE_PX as i64);
                    let i = gi.clamp(lo_i, lo_i + TILE_PX as i64 - 1) - lo_i;
                    let j = gj.clamp(lo_j, lo_j + TILE_PX as i64 - 1) - lo_j;
                    let t = centre.get().and_then(|t| t.as_ref()).expect("checked above");
                    t.at(i as usize, j as usize)
                }
            }
        };

        let mut p = [[0.0f64; 4]; 4];
        for (dj, row) in p.iter_mut().enumerate() {
            for (di, v) in row.iter_mut().enumerate() {
                *v = pixel(i1 - 1 + di as i64, j1 - 1 + dj as i64);
            }
        }
        Some(bicubic(&p, tu, tv))
    }

    /// The decoded tile for `(z, tx, ty)`, reading through both caches and
    /// decoding at most once however many handles race on it.
    fn tile(&mut self, z: u8, tx: u32, ty: u32) -> Slot {
        let slot = self.slot((z, tx, ty));
        let archive = &mut self.archive;
        slot.get_or_init(|| {
            DECODES.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            archive.tile(z, tx, ty).ok().flatten().and_then(|bytes| decode_terrarium(&bytes))
        });
        slot
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
        assert!((tile.at(10, 10) - 2523.265_625).abs() < 1e-3);
    }

    /// The 4×4 neighbourhood of a field with constant gradient, centred on
    /// `p[1][1]`, for the reconstruction tests below.
    fn ramp(slope_u: f64, slope_v: f64) -> [[f64; 4]; 4] {
        let mut p = [[0.0; 4]; 4];
        for (j, row) in p.iter_mut().enumerate() {
            for (i, v) in row.iter_mut().enumerate() {
                *v = (i as f64 - 1.0) * slope_u + (j as f64 - 1.0) * slope_v;
            }
        }
        p
    }

    #[test]
    fn bicubic_returns_the_pixel_at_a_pixel() {
        // Catmull-Rom interpolates: this is a smoother reading of the data,
        // not a smoothing of it, so the samples themselves must survive.
        let mut p = ramp(3.0, 7.0);
        p[1][1] = 42.0;
        p[1][2] = -13.0;
        assert!((bicubic(&p, 0.0, 0.0) - 42.0).abs() < 1e-9);
        assert!((bicubic(&p, 1.0, 0.0) - -13.0).abs() < 1e-9);
    }

    #[test]
    fn bicubic_reproduces_a_plane() {
        // A cubic through four collinear points is that line, so a constant
        // gradient must come back exactly — no ringing on flat or evenly
        // sloping ground, which is most ground.
        let p = ramp(2.5, -1.5);
        for (u, v) in [(0.0, 0.0), (0.25, 0.75), (0.5, 0.5), (0.9, 0.1)] {
            let want = u * 2.5 + v * -1.5;
            assert!((bicubic(&p, u, v) - want).abs() < 1e-9, "at ({u}, {v})");
        }
    }

    #[test]
    fn bicubic_is_smooth_where_bilinear_creases() {
        // The property the whole change exists for: crossing from one pixel
        // cell into the next, the gradient must not jump. Compare the slope a
        // hair either side of the pixel line at `tu = 1` / `tu = 0`.
        let mut left = [[0.0; 4]; 4];
        let mut right = [[0.0; 4]; 4];
        // A ridge: rising, then falling. Bilinear creases hard along it.
        let h = [0.0, 10.0, 12.0, 4.0, -6.0, -20.0];
        for j in 0..4 {
            for i in 0..4 {
                left[j][i] = h[i];
                right[j][i] = h[i + 1];
            }
        }
        let d = 1e-6;
        let slope_before = (bicubic(&left, 1.0, 0.5) - bicubic(&left, 1.0 - d, 0.5)) / d;
        let slope_after = (bicubic(&right, d, 0.5) - bicubic(&right, 0.0, 0.5)) / d;
        assert!(
            (slope_before - slope_after).abs() < 1e-3,
            "gradient jumps across the pixel line: {slope_before} vs {slope_after}"
        );
    }

    #[test]
    fn tile_span_covers_at_most_two_tiles() {
        let last = 4 * TILE_PX as i64 - 1;
        // Wholly inside one tile.
        assert_eq!(tile_span(10, 13, last), vec![0]);
        // Straddling the first boundary.
        assert_eq!(tile_span(TILE_PX as i64 - 2, TILE_PX as i64 + 1, last), vec![0, 1]);
        // Off the west edge of the level, and off the east edge.
        assert_eq!(tile_span(-3, 0, last), vec![0]);
        assert_eq!(tile_span(last - 1, last + 2, last), vec![3]);
    }
}
