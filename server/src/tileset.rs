//! Builds the `.arpi` tileset metadata blob (FORMAT.md §6.2).
//!
//! Deep module: the caller hands over plain descriptors and gets back the
//! finished, Brotli-compressed `.arpi` bytes — ready to pass to
//! [`crate::archive::ArchiveWriter::finish`]. FlatBuffers assembly and
//! compression are hidden inside. The build cannot fail in practice (in-memory
//! serialization + compression), so it returns the bytes directly.

use crate::fb::tileset::arpentry::tiles as fbt;
use crate::geom::GeometryType;
use crate::project::Bounds;

/// Per-layer descriptor for the tileset index.
#[derive(Debug, Clone)]
pub struct LayerInfo {
    pub name: String,
    /// Geometry topologies present in this layer.
    pub geometry_types: Vec<GeometryType>,
    /// First level where the layer appears.
    pub min_level: u8,
    /// Last level where the layer appears.
    pub max_level: u8,
}

/// Everything needed to describe a tileset.
#[derive(Debug, Clone)]
pub struct TilesetInfo {
    pub name: Option<String>,
    pub bounds: Bounds,
    /// Min/max elevation in metres.
    pub elevation_range: (f64, f64),
    pub min_level: u8,
    pub max_level: u8,
    /// Geometric error at level 0, in metres.
    pub root_error: f64,
    /// Layer descriptors in decode-priority order (FORMAT.md §9).
    pub layers: Vec<LayerInfo>,
}

/// Builds the Brotli-compressed `.arpi` blob for a tileset.
pub fn build(info: &TilesetInfo) -> Vec<u8> {
    compress(&build_uncompressed(info))
}

/// Builds the uncompressed `.arpi` FlatBuffer (file identifier `"arpi"` at
/// bytes 4..8, standard layout).
pub fn build_uncompressed(info: &TilesetInfo) -> Vec<u8> {
    let mut fbb = flatbuffers::FlatBufferBuilder::new();

    let name = info.name.as_deref().map(|s| fbb.create_string(s));

    // Nested vectors/strings must be built before the table that references them.
    let layer_offsets: Vec<_> = info
        .layers
        .iter()
        .map(|l| {
            let lname = fbb.create_string(&l.name);
            let gts: Vec<fbt::GeometryType> =
                l.geometry_types.iter().copied().map(to_fb_geom).collect();
            let gt_vec = fbb.create_vector(&gts);
            fbt::LayerInfo::create(
                &mut fbb,
                &fbt::LayerInfoArgs {
                    name: Some(lname),
                    geometry_types: Some(gt_vec),
                    min_level: l.min_level,
                    max_level: l.max_level,
                },
            )
        })
        .collect();
    let layers = fbb.create_vector(&layer_offsets);

    let bounds = fbt::Bounds::new(
        info.bounds.west,
        info.bounds.south,
        info.bounds.east,
        info.bounds.north,
    );
    let elevation = fbt::ElevationRange::new(info.elevation_range.0, info.elevation_range.1);

    let root = fbt::Tileset::create(
        &mut fbb,
        &fbt::TilesetArgs {
            version: 1,
            name,
            bounds: Some(&bounds),
            elevation_range: Some(&elevation),
            min_level: info.min_level,
            max_level: info.max_level,
            root_error: info.root_error,
            layers: Some(layers),
        },
    );
    fbt::finish_tileset_buffer(&mut fbb, root);
    fbb.finished_data().to_vec()
}

fn to_fb_geom(g: GeometryType) -> fbt::GeometryType {
    match g {
        GeometryType::Point => fbt::GeometryType::Point,
        GeometryType::Line => fbt::GeometryType::Line,
        GeometryType::Polygon => fbt::GeometryType::Polygon,
        GeometryType::Mesh => fbt::GeometryType::Mesh,
    }
}

/// Brotli-compresses an in-memory buffer.
fn compress(data: &[u8]) -> Vec<u8> {
    let params = brotli::enc::BrotliEncoderParams::default();
    let mut out = Vec::new();
    let mut input = data;
    brotli::BrotliCompress(&mut input, &mut out, &params).expect("brotli compress in-memory");
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> TilesetInfo {
        TilesetInfo {
            name: Some("Natural Earth".to_string()),
            bounds: Bounds { west: -180.0, south: -85.0, east: 180.0, north: 85.0 },
            elevation_range: (-10994.0, 8849.0),
            min_level: 0,
            max_level: 8,
            root_error: 256000.0,
            layers: vec![
                LayerInfo {
                    name: "terrain".to_string(),
                    geometry_types: vec![GeometryType::Mesh],
                    min_level: 0,
                    max_level: 8,
                },
                LayerInfo {
                    name: "water".to_string(),
                    geometry_types: vec![GeometryType::Polygon],
                    min_level: 0,
                    max_level: 8,
                },
            ],
        }
    }

    fn decompress(data: &[u8]) -> Vec<u8> {
        let mut out = Vec::new();
        let mut input = data;
        brotli::BrotliDecompress(&mut input, &mut out).expect("brotli decompress");
        out
    }

    #[test]
    fn has_standard_arpi_identifier() {
        // FORMAT.md §7: identifier at bytes 4..8 (standard FlatBuffers layout).
        let raw = build_uncompressed(&sample());
        assert_eq!(&raw[4..8], b"arpi");
    }

    #[test]
    fn flatbuffer_reads_back() {
        let raw = build_uncompressed(&sample());
        let ts = fbt::root_as_tileset(&raw).expect("read tileset root");

        assert_eq!(ts.version(), 1);
        assert_eq!(ts.name(), Some("Natural Earth"));
        assert_eq!(ts.min_level(), 0);
        assert_eq!(ts.max_level(), 8);
        assert_eq!(ts.root_error(), 256000.0);

        let bounds = ts.bounds().expect("bounds present");
        assert_eq!(bounds.west(), -180.0);
        assert_eq!(bounds.north(), 85.0);

        let layers = ts.layers().expect("layers present");
        assert_eq!(layers.len(), 2);
        assert_eq!(layers.get(0).name(), "terrain");
        assert_eq!(layers.get(1).name(), "water");
    }

    #[test]
    fn brotli_roundtrips() {
        let raw = build_uncompressed(&sample());
        let blob = build(&sample());
        assert!(!blob.is_empty());
        assert_eq!(decompress(&blob), raw, "brotli compress/decompress must round-trip");
    }
}
