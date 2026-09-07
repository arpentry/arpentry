//! A tile-free 3D world for one bounding box.
//!
//! The tiler in `arpentry-server` builds its model once and cuts it into
//! tiles, and everything below its assemble stage speaks in tile-local
//! quantised coordinates. That made every defect a question of *which* of the
//! ground, the extract, the tile border and the geometry was to blame before
//! it could be worked on. This crate takes the other road: one bounding box,
//! plain `f64` metres in a local frame, no tiles anywhere, and a pipeline of
//! steps each of which is a function from the world so far to the world with
//! one more layer — testable on synthetic ground where its output is an
//! assertion rather than a distribution.
//!
//! The steps run in a fixed order ([`step::Step::ALL`]) and the result is
//! written as a binary glTF ([`gltf::write_glb`]) whose bytes are a function of
//! the inputs alone, so two runs can be compared with `cmp`.
//!
//! Only the source readers are borrowed from the server crate: the DEM, the
//! GeoParquet reader and the bbox type. When the world is good, the tiler
//! becomes one more caller of this crate; nothing here must learn about tiles
//! for that to stay true.

pub mod drape;
pub mod facade;
pub mod fillet;
pub mod frame;
pub mod gltf;
pub mod grid;
pub mod kerb;
pub mod net;
pub mod poly;
pub mod ribbon;
pub mod roads;
pub mod room;
pub mod step;
pub mod surface;
pub mod svg;
pub mod terrain;
pub mod width;
pub mod world;
