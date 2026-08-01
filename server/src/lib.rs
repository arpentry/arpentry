//! Arpentry tiler — generates `.arpa` tile archives from geographic data.
//!
//! Tile generation is framed as a sort problem: features are clipped to tiles,
//! sorted by a space-filling-curve key, grouped, encoded, and written to a
//! single archive file. See `docs/TILER.md` and `docs/FORMAT.md`.
//!
//! The pipeline is modelled on Planetiler's two-phase design:
//!   1. process — fan features out into per-tile clipped records, sorted on disk
//!      by a Hilbert-ordered tile id;
//!   2. emit — read records back in tile order, group, encode, write the archive.
//!
//! Only the foundational, dependency-free modules are wired up so far. Heavier
//! modules are added per milestone (see README.md).

// During scaffolding, modules expose their public API (e.g. the archive reader)
// ahead of the consumers — pipeline, server — that will exercise it. Remove this
// once the pipeline wires everything together (milestone 5).
#![allow(dead_code)]

pub mod archive;
pub mod assemble;
pub mod building_mesh;
pub mod clip;
pub mod dem;
pub mod dump;
pub mod fb;
pub mod gen;
pub mod geom;
pub mod geoparquet;
pub mod ground;
pub mod hilbert;
pub mod layers;
pub mod levels;
pub mod models;
pub mod pipeline;
pub mod pmtiles;
pub mod priors;
pub mod profile;
pub mod project;
pub mod record;
pub mod rules;
pub mod scene;
pub mod simplify;
pub mod solve;
pub mod sort;
pub mod style;
pub mod synth;
pub mod terrain;
pub mod terrain_cdt;
pub mod tile_build;
pub mod tileid;
pub mod tileset;
pub mod value;
pub mod verify;
pub mod wkb;

/// Re-export shim for `flatc`-generated code.
///
/// `model_generated.rs` includes `tile.fbs` and refers to the shared `Part` and
/// `Color` types through the absolute path `crate::tile_generated`. The checked-in
/// `tile` bindings live under [`fb::tile`], so this module re-exports that
/// namespace at the path the generated code expects.
pub(crate) mod tile_generated {
    pub use crate::fb::tile::arpentry::tiles::*;
}
