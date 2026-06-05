//! Shared geometry vocabulary.
//!
//! For now this only carries the topology discriminator used across the tiler
//! (encoder layer descriptors, `.arpi` metadata). The full SoA geometry model
//! lands with the geometry-ops milestone.

/// Geometry topology. Ordinals match the FlatBuffers `Geometry` union member
/// order and the `GeometryType` enum in the tileset schema (FORMAT.md §6).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum GeometryType {
    Point = 0,
    Line = 1,
    Polygon = 2,
    Mesh = 3,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ordinals_match_schema() {
        assert_eq!(GeometryType::Point as u8, 0);
        assert_eq!(GeometryType::Line as u8, 1);
        assert_eq!(GeometryType::Polygon as u8, 2);
        assert_eq!(GeometryType::Mesh as u8, 3);
    }
}
