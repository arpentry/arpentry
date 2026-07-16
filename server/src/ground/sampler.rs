//! Per-worker ground sampler: the one handle everything in the emit path reads
//! ground heights through.
//!
//! Wraps the worker's own DEM reader (its decoded-tile cache is not shareable)
//! around the shared, immutable [`GroundModel`]. Consumers ask either for the
//! raw engineered ground ([`GroundSampler::ground`]) or for the *rendered*
//! ground ([`GroundSampler::surface`]) — the triangulated terrain-lattice
//! height that matches what the client draws at that zoom, which is what
//! draped geometry must sit on (invariant 4).

use std::collections::HashMap;
use std::sync::Arc;

use crate::dem::Dem;
use crate::ground::GroundModel;
use crate::project::Bounds;
use crate::terrain;

/// Corner-memo capacity. Entries are ~50 B, so this is a few MiB per worker —
/// hundreds of tiles' worth of lattice corners before an (unlikely) reset.
const CORNER_CAP: usize = 262_144;

pub struct GroundSampler {
    dem: Option<Dem>,
    ground: Arc<GroundModel>,
    /// The run's reference zoom, keying the per-zoom lattice resolution
    /// ([`terrain::grid_for`]) `surface` reads through.
    z_ref: u8,
    /// Reusable earthwork-query buffer (grid hits per sample).
    scratch: Vec<u32>,
    /// Memoized engineered heights at rendered-lattice corners, keyed by the
    /// corner's exact coordinate bits and the sampled zoom. The lattice is
    /// global per zoom and a tile's width is a dyadic rational, so every
    /// frame computes bit-identical corner coordinates — one memo entry
    /// serves the terrain mesh and every draped road vertex that touches the
    /// corner, collapsing the 4-corner fan-out of `surface` into ~one DEM
    /// sample per distinct corner.
    corners: HashMap<(u8, u64, u64), f64>,
}

impl GroundSampler {
    pub fn new(dem: Option<Dem>, ground: Arc<GroundModel>, z_ref: u8) -> GroundSampler {
        GroundSampler { dem, ground, z_ref, scratch: Vec::new(), corners: HashMap::new() }
    }

    /// Whether the run has real elevation at all (a DEM was configured).
    pub fn has_elevation(&self) -> bool {
        self.dem.is_some()
    }

    /// The engineered ground height at `(lon, lat)`, sampling the DEM at zoom
    /// `z`. Zero without a DEM (the flat parity world).
    pub fn ground(&mut self, lon: f64, lat: f64, z: u8) -> f64 {
        let raw = match &mut self.dem {
            Some(d) => d.elevation(lon, lat, z),
            None => 0.0,
        };
        self.ground.height(lon, lat, raw, &mut self.scratch)
    }

    /// The engineered ground at a rendered-lattice corner, memoized (see
    /// [`GroundSampler::corners`]). The terrain mesh and `surface` both read
    /// corners through this, so each distinct corner costs one DEM sample per
    /// worker however many queries land on it.
    pub fn corner(&mut self, lon: f64, lat: f64, z: u8) -> f64 {
        let key = (z, lon.to_bits(), lat.to_bits());
        if let Some(&h) = self.corners.get(&key) {
            return h;
        }
        let h = self.ground(lon, lat, z);
        if self.corners.len() >= CORNER_CAP {
            self.corners.clear();
        }
        self.corners.insert(key, h);
        h
    }

    /// The *rendered* ground at `(lon, lat)`: the engineered ground evaluated
    /// through the global zoom-`z` terrain lattice anchored at `bounds`, so it
    /// matches the triangulated mesh the client draws exactly.
    pub fn surface(&mut self, bounds: &Bounds, lon: f64, lat: f64, z: u8) -> f64 {
        let grid = terrain::grid_for(z, self.z_ref);
        terrain::surface_height(bounds, grid, lon, lat, &mut |a, o| self.corner(a, o, z))
    }

    /// The exact engineered roadbed height under `(lon, lat)` when the point
    /// lies fully inside a road earthwork's held width — a corridor's roadbed
    /// or a street bench — else `None`. The drape rides this at the reference
    /// zoom, where the rendered lattice is too coarse to hold a street-wide
    /// bench (see `synth::road::surface_height`).
    pub fn bed_target(&mut self, lon: f64, lat: f64) -> Option<f64> {
        self.ground.earthworks().target_at(lon, lat, &mut self.scratch)
    }
}
