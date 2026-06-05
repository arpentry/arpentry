//! Procedural world generation (port of the C server's `src/gen/`).
//!
//! When the server is launched without an `.arpa` archive, it synthesises tiles
//! on the fly from deterministic fractal noise: terrain elevation, biome
//! surfaces, a procedural town (roads + buildings), scattered trees, and a
//! handful of points of interest. [`world::generate_terrain`] is the whole
//! interface — it returns a Brotli-compressed `.arpt` blob for any `(z, x, y)`,
//! byte-for-byte equivalent to `arpt_generate_terrain` in `gen/world.c`.
//!
//! The submodules mirror the C files one-for-one:
//!   `noise`   ← `gen/noise.c`    simplex + fBm
//!   `terrain` ← `gen/terrain.c`  elevation/moisture fields + mesh
//!   `surface` ← `gen/surface.c`  biome classification + marching squares
//!   `town`    ← `gen/town.c`     procedural roads + buildings
//!   `tree`    ← `gen/tree.c`     forest point scatter
//!   `poi`     ← `gen/poi.c`      hardcoded points of interest
//!   `world`   ← `gen/world.c`    tile assembly + compression

pub mod noise;
pub mod poi;
pub mod surface;
pub mod terrain;
pub mod town;
pub mod tree;
pub mod world;
