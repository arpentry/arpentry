//! `.arpa` tile archive — single-file container (see TILER.md §2).
//!
//! Layout (all multi-byte integers little-endian):
//! ```text
//! ┌──────────────────────────────┐
//! │ Header (128 bytes)           │  magic, version, zoom range, bounds,
//! │                              │  root_error, tile_count, dir/meta offsets
//! ├──────────────────────────────┤
//! │ Tile Data                    │  sequential Brotli-compressed .arpt blobs
//! ├──────────────────────────────┤
//! │ Directory                    │  sorted array of 40-byte entries,
//! │                              │  binary-searchable by Hilbert tile id
//! ├──────────────────────────────┤
//! │ Metadata                     │  Brotli-compressed .arpi blob
//! └──────────────────────────────┘
//! ```
//!
//! This module owns only the container. Tile blobs are stored opaquely — the
//! caller compresses them — and the metadata blob is likewise stored as-is
//! (the caller Brotli-compresses the `.arpi` FlatBuffer). Keeping compression
//! and FlatBuffers out of here lets the container build and test with no
//! external dependencies.

use std::io::{self, Seek, SeekFrom, Write};

use crate::hilbert;
use crate::project::Bounds;

/// `"arpa"` little-endian (`0x61727061`).
pub const MAGIC: u32 = 0x6172_7061;
/// Current archive format version.
pub const VERSION: u32 = 1;
/// Fixed header size in bytes.
pub const HEADER_SIZE: u64 = 128;
/// Fixed directory-entry size in bytes.
pub const DIR_ENTRY_SIZE: usize = 40;

// Header field offsets (see module docs). bounds is 8-byte aligned at 16.
const H_MAGIC: usize = 0;
const H_VERSION: usize = 4;
const H_MIN_ZOOM: usize = 8;
const H_MAX_ZOOM: usize = 9;
const H_BOUNDS: usize = 16; // 4 × f64
const H_ROOT_ERROR: usize = 48;
const H_TILE_COUNT: usize = 56;
const H_DIR_OFFSET: usize = 64;
const H_META_OFFSET: usize = 72;
const H_META_SIZE: usize = 80;

/// Errors surfaced when opening or reading an archive.
#[derive(Debug)]
pub enum ArchiveError {
    Io(io::Error),
    /// File too short to contain the structure it claims.
    Truncated,
    /// Magic number did not match `"arpa"`.
    BadMagic(u32),
    /// Version is not understood by this build.
    UnsupportedVersion(u32),
    /// Directory or a tile blob range falls outside the file.
    Corrupt(&'static str),
}

impl std::fmt::Display for ArchiveError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ArchiveError::Io(e) => write!(f, "io error: {e}"),
            ArchiveError::Truncated => write!(f, "archive truncated"),
            ArchiveError::BadMagic(m) => write!(f, "bad magic 0x{m:08x} (expected arpa)"),
            ArchiveError::UnsupportedVersion(v) => write!(f, "unsupported version {v}"),
            ArchiveError::Corrupt(why) => write!(f, "corrupt archive: {why}"),
        }
    }
}

impl std::error::Error for ArchiveError {}

impl From<io::Error> for ArchiveError {
    fn from(e: io::Error) -> Self {
        ArchiveError::Io(e)
    }
}

type Result<T> = std::result::Result<T, ArchiveError>;

/// A directory entry: a tile's address and its blob location in the file.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DirEntry {
    pub hilbert_id: u64,
    pub offset: u64,
    pub size: u64,
    pub z: u8,
    pub x: u32,
    pub y: u32,
}

impl DirEntry {
    fn write_to(&self, buf: &mut [u8; DIR_ENTRY_SIZE]) {
        buf.fill(0);
        buf[0..8].copy_from_slice(&self.hilbert_id.to_le_bytes());
        buf[8..16].copy_from_slice(&self.offset.to_le_bytes());
        buf[16..24].copy_from_slice(&self.size.to_le_bytes());
        buf[24] = self.z;
        // bytes 25..28 padding
        buf[28..32].copy_from_slice(&self.x.to_le_bytes());
        buf[32..36].copy_from_slice(&self.y.to_le_bytes());
        // bytes 36..40 padding
    }

    fn read_from(buf: &[u8]) -> DirEntry {
        DirEntry {
            hilbert_id: rd_u64(buf, 0),
            offset: rd_u64(buf, 8),
            size: rd_u64(buf, 16),
            z: buf[24],
            x: rd_u32(buf, 28),
            y: rd_u32(buf, 32),
        }
    }
}

/// Top-level archive parameters captured in the header.
#[derive(Debug, Clone, Copy)]
pub struct ArchiveMeta {
    pub min_zoom: u8,
    pub max_zoom: u8,
    pub bounds: Bounds,
    pub root_error: f64,
}

/// Streams tile blobs into a `.arpa` file, then finalizes the directory + header.
///
/// Tiles should be added in Hilbert order (as the pipeline emits them) so that
/// consecutive identical blobs — e.g. runs of empty ocean tiles — are
/// deduplicated automatically.
pub struct ArchiveWriter<W: Write + Seek> {
    w: W,
    meta: ArchiveMeta,
    dir: Vec<DirEntry>,
    offset: u64, // absolute write position (starts after the reserved header)
    last: Option<LastBlob>,
}

struct LastBlob {
    bytes: Vec<u8>,
    offset: u64,
    size: u64,
}

impl<W: Write + Seek> ArchiveWriter<W> {
    /// Creates a writer, reserving the header region at the start of the file.
    pub fn new(mut w: W, meta: ArchiveMeta) -> Result<Self> {
        w.write_all(&[0u8; HEADER_SIZE as usize])?;
        Ok(ArchiveWriter { w, meta, dir: Vec::new(), offset: HEADER_SIZE, last: None })
    }

    /// Appends one tile blob. Consecutive byte-identical blobs share storage.
    pub fn add_tile(&mut self, z: u8, x: u32, y: u32, blob: &[u8]) -> Result<()> {
        let hilbert_id = hilbert::tile_id(z, x, y);

        if let Some(last) = &self.last {
            if last.bytes == blob {
                // Reuse the previous blob's storage.
                self.dir.push(DirEntry {
                    hilbert_id,
                    offset: last.offset,
                    size: last.size,
                    z,
                    x,
                    y,
                });
                return Ok(());
            }
        }

        let offset = self.offset;
        let size = blob.len() as u64;
        self.w.write_all(blob)?;
        self.offset += size;
        self.dir.push(DirEntry { hilbert_id, offset, size, z, x, y });
        self.last = Some(LastBlob { bytes: blob.to_vec(), offset, size });
        Ok(())
    }

    /// Writes the directory and metadata, backfills the header, returns the
    /// inner writer for the caller to flush/close.
    pub fn finish(mut self, metadata: &[u8]) -> Result<W> {
        // Directory, sorted by Hilbert id for binary search.
        self.dir.sort_unstable_by_key(|e| e.hilbert_id);
        let dir_offset = self.offset;
        let mut entry = [0u8; DIR_ENTRY_SIZE];
        for e in &self.dir {
            e.write_to(&mut entry);
            self.w.write_all(&entry)?;
        }
        self.offset += (self.dir.len() * DIR_ENTRY_SIZE) as u64;

        // Metadata blob.
        let meta_offset = self.offset;
        self.w.write_all(metadata)?;

        // Backfill the header.
        let mut header = [0u8; HEADER_SIZE as usize];
        wr_u32(&mut header, H_MAGIC, MAGIC);
        wr_u32(&mut header, H_VERSION, VERSION);
        header[H_MIN_ZOOM] = self.meta.min_zoom;
        header[H_MAX_ZOOM] = self.meta.max_zoom;
        wr_f64(&mut header, H_BOUNDS, self.meta.bounds.west);
        wr_f64(&mut header, H_BOUNDS + 8, self.meta.bounds.south);
        wr_f64(&mut header, H_BOUNDS + 16, self.meta.bounds.east);
        wr_f64(&mut header, H_BOUNDS + 24, self.meta.bounds.north);
        wr_f64(&mut header, H_ROOT_ERROR, self.meta.root_error);
        wr_u64(&mut header, H_TILE_COUNT, self.dir.len() as u64);
        wr_u64(&mut header, H_DIR_OFFSET, dir_offset);
        wr_u64(&mut header, H_META_OFFSET, meta_offset);
        wr_u64(&mut header, H_META_SIZE, metadata.len() as u64);

        self.w.seek(SeekFrom::Start(0))?;
        self.w.write_all(&header)?;
        self.w.flush()?;
        Ok(self.w)
    }
}

/// A read-only view over an archive's bytes (e.g. an mmap or a read-in buffer).
///
/// `get` resolves a tile by binary-searching the directory on its Hilbert id;
/// returned slices borrow the backing bytes and are valid for its lifetime.
pub struct Archive<'a> {
    data: &'a [u8],
    min_zoom: u8,
    max_zoom: u8,
    bounds: Bounds,
    root_error: f64,
    tile_count: u64,
    dir_offset: u64,
    meta_offset: u64,
    meta_size: u64,
}

impl<'a> Archive<'a> {
    /// Parses and validates the header and directory bounds.
    pub fn open(data: &'a [u8]) -> Result<Self> {
        if (data.len() as u64) < HEADER_SIZE {
            return Err(ArchiveError::Truncated);
        }
        let magic = rd_u32(data, H_MAGIC);
        if magic != MAGIC {
            return Err(ArchiveError::BadMagic(magic));
        }
        let version = rd_u32(data, H_VERSION);
        if version != VERSION {
            return Err(ArchiveError::UnsupportedVersion(version));
        }
        let tile_count = rd_u64(data, H_TILE_COUNT);
        let dir_offset = rd_u64(data, H_DIR_OFFSET);
        let meta_offset = rd_u64(data, H_META_OFFSET);
        let meta_size = rd_u64(data, H_META_SIZE);

        let dir_end = dir_offset
            .checked_add(tile_count.checked_mul(DIR_ENTRY_SIZE as u64).ok_or(ArchiveError::Corrupt("directory size overflow"))?)
            .ok_or(ArchiveError::Corrupt("directory end overflow"))?;
        if dir_end > data.len() as u64 {
            return Err(ArchiveError::Corrupt("directory out of range"));
        }
        let meta_end = meta_offset
            .checked_add(meta_size)
            .ok_or(ArchiveError::Corrupt("metadata end overflow"))?;
        if meta_end > data.len() as u64 {
            return Err(ArchiveError::Corrupt("metadata out of range"));
        }

        Ok(Archive {
            data,
            min_zoom: data[H_MIN_ZOOM],
            max_zoom: data[H_MAX_ZOOM],
            bounds: Bounds {
                west: rd_f64(data, H_BOUNDS),
                south: rd_f64(data, H_BOUNDS + 8),
                east: rd_f64(data, H_BOUNDS + 16),
                north: rd_f64(data, H_BOUNDS + 24),
            },
            root_error: rd_f64(data, H_ROOT_ERROR),
            tile_count,
            dir_offset,
            meta_offset,
            meta_size,
        })
    }

    pub fn min_zoom(&self) -> u8 {
        self.min_zoom
    }
    pub fn max_zoom(&self) -> u8 {
        self.max_zoom
    }
    pub fn bounds(&self) -> Bounds {
        self.bounds
    }
    pub fn root_error(&self) -> f64 {
        self.root_error
    }
    pub fn tile_count(&self) -> u64 {
        self.tile_count
    }
    pub fn is_empty(&self) -> bool {
        self.tile_count == 0
    }

    /// Returns the (still-compressed) metadata blob.
    pub fn metadata(&self) -> &'a [u8] {
        let start = self.meta_offset as usize;
        &self.data[start..start + self.meta_size as usize]
    }

    /// Reads the i-th directory entry (in Hilbert-id order). Caller guarantees
    /// `i < tile_count`.
    fn entry(&self, i: u64) -> DirEntry {
        let start = (self.dir_offset + i * DIR_ENTRY_SIZE as u64) as usize;
        DirEntry::read_from(&self.data[start..start + DIR_ENTRY_SIZE])
    }

    /// Looks up a tile's blob by address, or `None` if absent.
    pub fn get(&self, z: u8, x: u32, y: u32) -> Option<&'a [u8]> {
        self.get_by_id(hilbert::tile_id(z, x, y))
    }

    /// Looks up a tile's blob by its Hilbert id, or `None` if absent.
    pub fn get_by_id(&self, id: u64) -> Option<&'a [u8]> {
        // Binary search the on-disk directory without materializing it.
        let mut lo = 0u64;
        let mut hi = self.tile_count;
        while lo < hi {
            let mid = lo + (hi - lo) / 2;
            let e = self.entry(mid);
            match e.hilbert_id.cmp(&id) {
                std::cmp::Ordering::Less => lo = mid + 1,
                std::cmp::Ordering::Greater => hi = mid,
                std::cmp::Ordering::Equal => {
                    let start = e.offset as usize;
                    let end = start.checked_add(e.size as usize)?;
                    return self.data.get(start..end);
                }
            }
        }
        None
    }

    /// Iterates directory entries in Hilbert-id order.
    pub fn entries(&self) -> impl Iterator<Item = DirEntry> + '_ {
        (0..self.tile_count).map(move |i| self.entry(i))
    }
}

// --- little-endian read/write helpers (no external deps) ---

fn rd_u32(d: &[u8], o: usize) -> u32 {
    u32::from_le_bytes(d[o..o + 4].try_into().unwrap())
}
fn rd_u64(d: &[u8], o: usize) -> u64 {
    u64::from_le_bytes(d[o..o + 8].try_into().unwrap())
}
fn rd_f64(d: &[u8], o: usize) -> f64 {
    f64::from_le_bytes(d[o..o + 8].try_into().unwrap())
}
fn wr_u32(d: &mut [u8], o: usize, v: u32) {
    d[o..o + 4].copy_from_slice(&v.to_le_bytes());
}
fn wr_u64(d: &mut [u8], o: usize, v: u64) {
    d[o..o + 8].copy_from_slice(&v.to_le_bytes());
}
fn wr_f64(d: &mut [u8], o: usize, v: f64) {
    d[o..o + 8].copy_from_slice(&v.to_le_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    fn sample_meta() -> ArchiveMeta {
        ArchiveMeta {
            min_zoom: 0,
            max_zoom: 14,
            bounds: Bounds { west: 6.0, south: 46.0, east: 7.0, north: 47.0 },
            root_error: 12345.678,
        }
    }

    fn build(tiles: &[(u8, u32, u32, Vec<u8>)], metadata: &[u8]) -> Vec<u8> {
        let mut w = ArchiveWriter::new(Cursor::new(Vec::new()), sample_meta()).unwrap();
        for (z, x, y, blob) in tiles {
            w.add_tile(*z, *x, *y, blob).unwrap();
        }
        w.finish(metadata).unwrap().into_inner()
    }

    #[test]
    fn roundtrip_header_and_tiles() {
        // Added out of Hilbert order on purpose.
        let tiles = vec![
            (5u8, 31u32, 17u32, b"tile-A".to_vec()),
            (0, 0, 0, b"tile-B-bigger".to_vec()),
            (14, 9000, 4096, b"tile-C".to_vec()),
        ];
        let meta = b"metadata-blob";
        let bytes = build(&tiles, meta);

        let a = Archive::open(&bytes).unwrap();
        assert_eq!(a.tile_count(), 3);
        assert_eq!(a.min_zoom(), 0);
        assert_eq!(a.max_zoom(), 14);
        assert_eq!(a.bounds(), sample_meta().bounds);
        assert!((a.root_error() - 12345.678).abs() < 1e-9);
        assert_eq!(a.metadata(), meta);

        for (z, x, y, blob) in &tiles {
            assert_eq!(a.get(*z, *x, *y), Some(blob.as_slice()), "z={z} x={x} y={y}");
        }
    }

    #[test]
    fn directory_is_sorted_by_hilbert_id() {
        let tiles = vec![
            (5u8, 31u32, 17u32, b"a".to_vec()),
            (0, 0, 0, b"b".to_vec()),
            (14, 9000, 4096, b"c".to_vec()),
        ];
        let bytes = build(&tiles, b"");
        let a = Archive::open(&bytes).unwrap();
        let ids: Vec<u64> = a.entries().map(|e| e.hilbert_id).collect();
        assert_eq!(ids.len(), 3);
        assert!(ids.windows(2).all(|w| w[0] < w[1]));
    }

    #[test]
    fn missing_tile_returns_none() {
        let bytes = build(&[(3u8, 1u32, 2u32, b"x".to_vec())], b"");
        let a = Archive::open(&bytes).unwrap();
        assert_eq!(a.get(3, 1, 2), Some(b"x".as_slice()));
        assert_eq!(a.get(3, 5, 5), None);
        assert_eq!(a.get(7, 0, 0), None);
    }

    #[test]
    fn consecutive_identical_blobs_are_deduplicated() {
        // Two identical empty-ocean-style blobs added back to back, then a
        // different one. Hilbert order is not required for the dedup to fire
        // because we add them consecutively.
        let empty = vec![0xEEu8; 64];
        let tiles = vec![
            (4u8, 0u32, 0u32, empty.clone()),
            (4, 1, 0, empty.clone()),
            (4, 2, 0, vec![0x11u8; 64]),
        ];
        let bytes = build(&tiles, b"");
        let a = Archive::open(&bytes).unwrap();

        // Both empty tiles resolve to identical content...
        assert_eq!(a.get(4, 0, 0), Some(empty.as_slice()));
        assert_eq!(a.get(4, 1, 0), Some(empty.as_slice()));

        // ...and share the same storage offset.
        let mut by_id = std::collections::HashMap::new();
        for e in a.entries() {
            by_id.insert(e.hilbert_id, e);
        }
        let id0 = hilbert::tile_id(4, 0, 0);
        let id1 = hilbert::tile_id(4, 1, 0);
        assert_eq!(by_id[&id0].offset, by_id[&id1].offset, "dedup should share offset");

        // Stored payload = one empty blob + one distinct blob (not two empties).
        let payload = bytes.len() as u64
            - HEADER_SIZE
            - (3 * DIR_ENTRY_SIZE) as u64; // no metadata
        assert_eq!(payload, 64 + 64, "duplicate blob must not be stored twice");
    }

    #[test]
    fn empty_archive_reports_empty() {
        let bytes = build(&[], b"");
        let a = Archive::open(&bytes).unwrap();
        assert!(a.is_empty());
        assert_eq!(a.tile_count(), 0);
        assert_eq!(a.get(0, 0, 0), None);
    }

    #[test]
    fn rejects_bad_magic() {
        let mut bytes = build(&[(0u8, 0u32, 0u32, b"x".to_vec())], b"");
        bytes[0] ^= 0xFF;
        assert!(matches!(Archive::open(&bytes), Err(ArchiveError::BadMagic(_))));
    }

    #[test]
    fn rejects_truncated() {
        let bytes = vec![0u8; 10];
        assert!(matches!(Archive::open(&bytes), Err(ArchiveError::Truncated)));
    }
}
