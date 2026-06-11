//! Minimal PMTiles v3 archive reader (spec: github.com/protomaps/PMTiles).
//!
//! Only the read path the DEM sampler needs is implemented: parse the 127-byte
//! header, walk the root (and any leaf) directory to resolve a `(z, x, y)` tile
//! to its byte range, and return the (decompressed) tile blob. Mapterhorn's
//! terrain archives store Terrarium WebP tiles with gzip-compressed directories
//! and uncompressed tile data; gzip, brotli, and "none" compression are handled
//! (zstd is rejected with a clear error rather than mis-decoded).
//!
//! Tile ids use PMTiles' Hilbert ordering, which is the same curve as
//! [`crate::hilbert`] plus a per-zoom base offset — so the existing `xy2d` is
//! reused instead of a second Hilbert implementation.

use std::fs::File;
use std::io::{self, Read, Seek, SeekFrom};
use std::path::Path;

use crate::hilbert;

/// Internal/tile compression codes (PMTiles header bytes 97/98).
const COMPRESSION_NONE: u8 = 1;
const COMPRESSION_GZIP: u8 = 2;
const COMPRESSION_BROTLI: u8 = 3;
const COMPRESSION_ZSTD: u8 = 4;

const HEADER_LEN: usize = 127;
const MAGIC: &[u8] = b"PMTiles";

/// Sanity caps on lengths read from the archive, so a corrupt header or
/// directory entry yields a clean error instead of a multi-GB allocation.
const MAX_DIR_LEN: u64 = 16 * 1024 * 1024;
const MAX_TILE_LEN: u64 = 64 * 1024 * 1024;

/// One directory entry: a tile run or a pointer to a leaf directory.
#[derive(Clone, Copy)]
struct Entry {
    tile_id: u64,
    /// Byte offset relative to the tile-data section (tiles) or leaf-dir section
    /// (leaf pointers).
    offset: u64,
    length: u32,
    /// Number of consecutive tile ids covered (>=1 for tiles); 0 = leaf pointer.
    run_length: u32,
}

/// An opened PMTiles archive. Holds the file handle and the in-memory root
/// directory; leaf directories are read on demand.
pub struct Pmtiles {
    file: File,
    root: Vec<Entry>,
    leaf_dirs_offset: u64,
    tile_data_offset: u64,
    internal_compression: u8,
    tile_compression: u8,
    /// Tile type code (4 = WebP, 2 = PNG) and zoom range, exposed for callers.
    pub tile_type: u8,
    pub min_zoom: u8,
    pub max_zoom: u8,
}

impl Pmtiles {
    /// Opens and parses the archive header and root directory.
    pub fn open(path: &Path) -> io::Result<Pmtiles> {
        let mut file = File::open(path)?;
        let mut header = [0u8; HEADER_LEN];
        file.read_exact(&mut header)?;
        if &header[0..7] != MAGIC {
            return Err(invalid("not a PMTiles archive (bad magic)"));
        }
        if header[7] != 3 {
            return Err(invalid(&format!("unsupported PMTiles version {}", header[7])));
        }

        let root_offset = le_u64(&header[8..16]);
        let root_length = le_u64(&header[16..24]);
        let leaf_dirs_offset = le_u64(&header[40..48]);
        let tile_data_offset = le_u64(&header[56..64]);
        let internal_compression = header[97];
        let tile_compression = header[98];
        let tile_type = header[99];
        let min_zoom = header[100];
        let max_zoom = header[101];

        let mut pm = Pmtiles {
            file,
            root: Vec::new(),
            leaf_dirs_offset,
            tile_data_offset,
            internal_compression,
            tile_compression,
            tile_type,
            min_zoom,
            max_zoom,
        };
        if root_length > MAX_DIR_LEN {
            return Err(invalid("root directory length exceeds sanity cap"));
        }
        let raw = pm.read_range(root_offset, root_length as usize)?;
        let dir = decompress(internal_compression, &raw)?;
        pm.root = parse_directory(&dir)?;
        Ok(pm)
    }

    /// Returns the decompressed bytes of tile `(z, x, y)`, or `None` if absent.
    pub fn tile(&mut self, z: u8, x: u32, y: u32) -> io::Result<Option<Vec<u8>>> {
        if z >= 32 {
            return Ok(None); // tile_id math overflows u64 beyond z=31
        }
        let id = tile_id(z, x, y);
        // Resolve through the root directory, following at most one leaf level
        // (PMTiles archives in practice never nest deeper than one).
        let mut entries = self.root.clone();
        for _ in 0..4 {
            let Some(e) = find(&entries, id) else {
                return Ok(None);
            };
            if e.run_length == 0 {
                // Leaf-directory pointer: load and continue the search there.
                if e.length as u64 > MAX_DIR_LEN {
                    return Err(invalid("leaf directory length exceeds sanity cap"));
                }
                let raw = self.read_range(self.leaf_dirs_offset + e.offset, e.length as usize)?;
                let dir = decompress(self.internal_compression, &raw)?;
                entries = parse_directory(&dir)?;
                continue;
            }
            if id - e.tile_id >= e.run_length as u64 {
                return Ok(None);
            }
            if e.length as u64 > MAX_TILE_LEN {
                return Err(invalid("tile length exceeds sanity cap"));
            }
            let raw = self.read_range(self.tile_data_offset + e.offset, e.length as usize)?;
            return Ok(Some(decompress(self.tile_compression, &raw)?));
        }
        Ok(None)
    }

    fn read_range(&mut self, offset: u64, len: usize) -> io::Result<Vec<u8>> {
        self.file.seek(SeekFrom::Start(offset))?;
        let mut buf = vec![0u8; len];
        self.file.read_exact(&mut buf)?;
        Ok(buf)
    }
}

/// PMTiles tile id for `(z, x, y)`: the count of all tiles in zooms `< z`
/// (`(4^z - 1) / 3`) plus the Hilbert distance of `(x, y)` within zoom `z`.
fn tile_id(z: u8, x: u32, y: u32) -> u64 {
    let base = ((1u64 << (2 * z as u32)) - 1) / 3;
    base + hilbert::xy2d(z as u32, x, y)
}

/// Finds the entry covering `id`: the last entry whose `tile_id <= id`.
fn find(entries: &[Entry], id: u64) -> Option<Entry> {
    if entries.is_empty() || entries[0].tile_id > id {
        return None;
    }
    // Binary search for the rightmost tile_id <= id.
    let mut lo = 0usize;
    let mut hi = entries.len(); // exclusive
    while lo + 1 < hi {
        let mid = (lo + hi) / 2;
        if entries[mid].tile_id <= id {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    Some(entries[lo])
}

/// Parses a serialized directory: a uvarint count followed by four
/// delta/run-length-encoded uvarint columns (tile_id, run_length, length,
/// offset). An offset of 0 means "immediately after the previous entry".
fn parse_directory(buf: &[u8]) -> io::Result<Vec<Entry>> {
    let mut pos = 0usize;
    let n = read_uvarint(buf, &mut pos)? as usize;
    let mut entries = vec![Entry { tile_id: 0, offset: 0, length: 0, run_length: 0 }; n];

    let mut last_id = 0u64;
    for e in entries.iter_mut() {
        last_id += read_uvarint(buf, &mut pos)?;
        e.tile_id = last_id;
    }
    for e in entries.iter_mut() {
        e.run_length = read_uvarint(buf, &mut pos)? as u32;
    }
    for e in entries.iter_mut() {
        e.length = read_uvarint(buf, &mut pos)? as u32;
    }
    for i in 0..n {
        let v = read_uvarint(buf, &mut pos)?;
        entries[i].offset = if v == 0 {
            // "Immediately after the previous entry" — meaningless for the
            // first entry, which has nothing to follow.
            if i == 0 {
                return Err(invalid("first directory entry has no explicit offset"));
            }
            entries[i - 1].offset + entries[i - 1].length as u64
        } else {
            v - 1
        };
    }
    Ok(entries)
}

/// Reads an unsigned LEB128 varint, advancing `pos`.
fn read_uvarint(buf: &[u8], pos: &mut usize) -> io::Result<u64> {
    let mut result = 0u64;
    let mut shift = 0u32;
    loop {
        let byte = *buf.get(*pos).ok_or_else(|| invalid("truncated varint"))?;
        *pos += 1;
        result |= ((byte & 0x7f) as u64) << shift;
        if byte & 0x80 == 0 {
            return Ok(result);
        }
        shift += 7;
        if shift >= 64 {
            return Err(invalid("varint overflow"));
        }
    }
}

/// Decompresses `data` according to a PMTiles compression code.
fn decompress(compression: u8, data: &[u8]) -> io::Result<Vec<u8>> {
    match compression {
        COMPRESSION_NONE => Ok(data.to_vec()),
        COMPRESSION_GZIP => {
            let mut out = Vec::new();
            flate2::read::GzDecoder::new(data).read_to_end(&mut out)?;
            Ok(out)
        }
        COMPRESSION_BROTLI => {
            let mut out = Vec::new();
            brotli::Decompressor::new(data, 4096).read_to_end(&mut out)?;
            Ok(out)
        }
        COMPRESSION_ZSTD => Err(invalid("zstd-compressed PMTiles are not supported")),
        other => Err(invalid(&format!("unknown PMTiles compression code {other}"))),
    }
}

fn le_u64(b: &[u8]) -> u64 {
    u64::from_le_bytes(b.try_into().unwrap())
}

fn invalid(msg: &str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, msg.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tile_id_matches_known_values() {
        // Root tile is id 0; zoom 1 begins at base 1 in Hilbert (z,x,y) order.
        assert_eq!(tile_id(0, 0, 0), 0);
        assert_eq!(tile_id(1, 0, 0), 1);
        assert_eq!(tile_id(1, 0, 1), 2);
        assert_eq!(tile_id(1, 1, 1), 3);
        assert_eq!(tile_id(1, 1, 0), 4);
        // Zoom 2 starts right after the four zoom-1 tiles.
        assert_eq!(tile_id(2, 0, 0), 5);
    }

    #[test]
    fn find_picks_covering_entry() {
        let entries = vec![
            Entry { tile_id: 0, offset: 0, length: 10, run_length: 1 },
            Entry { tile_id: 5, offset: 10, length: 10, run_length: 3 },
            Entry { tile_id: 20, offset: 20, length: 10, run_length: 1 },
        ];
        assert_eq!(find(&entries, 0).unwrap().tile_id, 0);
        assert_eq!(find(&entries, 6).unwrap().tile_id, 5); // inside run
        assert_eq!(find(&entries, 19).unwrap().tile_id, 5); // before next entry
        assert_eq!(find(&entries, 100).unwrap().tile_id, 20);
        assert!(find(&entries, 0).is_some());
    }

    #[test]
    fn parse_directory_rejects_auto_offset_on_first_entry() {
        // count=1; id delta [3]; run [1]; length [10]; offset 0 = "after previous",
        // which the first entry doesn't have — must error, not panic.
        let bytes = [1u8, 3, 1, 10, 0];
        assert!(parse_directory(&bytes).is_err());
    }

    #[test]
    fn parse_directory_roundtrips_offsets() {
        // count=2; ids delta [3,1]; runs [1,1]; lengths [10,20]; offsets [0(=>auto),0]
        // Build the byte stream by hand (all values < 0x80, single-byte varints).
        let bytes = [
            2u8, // count
            3, 1, // tile_id deltas -> 3, 4
            1, 1, // run lengths
            10, 20, // lengths
            1, 0, // offsets: first = 1-1 = 0; second = 0 -> prev.offset+prev.length = 10
        ];
        let dir = parse_directory(&bytes).unwrap();
        assert_eq!(dir.len(), 2);
        assert_eq!(dir[0].tile_id, 3);
        assert_eq!(dir[1].tile_id, 4);
        assert_eq!(dir[0].offset, 0);
        assert_eq!(dir[1].offset, 10);
    }
}
