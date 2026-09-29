//! A tile-free 3D world for one bounding box.
//!
//! The tiler in `arpentry-server` builds its model once and cuts it into
//! tiles, and everything below its assemble stage speaks in tile-local
//! quantised coordinates, so every defect is first a question of *which* of
//! the ground, the extract, the tile border and the geometry is to blame.
//! This crate has none of that: one bounding box, plain `f64` metres in a
//! local frame, no tiles anywhere, and a pipeline of steps each of which adds
//! one layer — testable on synthetic ground where its output is an assertion
//! rather than a distribution.
//!
//! Each step is a plain function of the layers it reads
//! (`fn run(inputs…) -> (Layer, Summary)`), so its signature is the whole of
//! its interface: nothing reaches into a shared world, and nothing can start
//! depending on a neighbour without the change showing up in one place.
//! That one place is [`pipeline`], which owns the wiring. The steps and
//! their layers are listed once, in [`step`]'s table, and the order
//! ([`step::Step::ALL`]), the fields of [`world::World`] (each named after
//! its step) and the dump are all expanded from it. `World` is the record
//! the layers land in, read only by the two renderers. The result is written
//! as a binary glTF ([`gltf::write_glb`]) whose bytes are a function of the
//! inputs alone, so two runs can be compared with `cmp`.
//!
//! Only the source readers are borrowed from the server crate: the DEM, the
//! GeoParquet reader and the bbox type. When the world is good, the tiler
//! becomes one more caller of this crate; nothing here must learn about tiles
//! for that to stay true.

pub mod arrangement;
pub mod bench;
pub mod building;
pub mod copies;
pub mod crossing;
pub mod drape;
pub mod dump;
pub mod earthwork;
pub mod facade;
pub mod field;
#[cfg(test)]
pub(crate) mod fixture;
pub mod frame;
pub mod gap;
pub mod gltf;
pub mod grade;
pub mod grid;
pub mod ground;
pub mod lattice;
pub mod line;
pub mod kerb;
pub mod legs;
pub mod lift;
pub mod mesh;
pub mod net;
pub mod partition;
pub mod pipeline;
pub mod poly;
pub mod portal;
pub mod profile;
pub mod reference;
pub mod solve;
pub mod ribbon;
pub mod roads;
pub mod room;
pub mod sheet;
pub mod standard;
#[cfg(test)]
mod specs;
pub mod step;
pub mod structure;
pub mod surface;
pub mod svg;
pub mod terrain;
pub mod triangulate;
pub mod width;
pub mod world;
