//! Per-worker ground sampler: the one handle everything in the emit path reads
//! ground heights through.
//!
//! Wraps the worker's own DEM reader (its decoded-tile cache is not shareable)
//! around the shared, immutable [`GroundStack`]. Consumers ask either for the
//! raw engineered ground ([`GroundSampler::ground`]) or for the *rendered*
//! ground ([`GroundSampler::surface`]) — the triangulated terrain-lattice
//! height that matches what the client draws at that zoom, which is what
//! draped geometry must sit on (invariant 4).

use std::collections::HashMap;
use std::sync::Arc;

use crate::dem::Dem;
use crate::ground::GroundStack;
use crate::project::Bounds;
use crate::synth::region::Region;
use crate::terrain::{self, TerrainMesh};
use crate::terrain_cdt;

/// Corner-memo capacity. Entries are ~50 B, so this is a few MiB per worker —
/// hundreds of tiles' worth of lattice corners before an (unlikely) reset.
const CORNER_CAP: usize = 262_144;

/// How the detail-zoom terrain mesh is built. Both are on in a normal run; the
/// flags exist so an A/B re-tile is a command-line switch rather than a patch.
///
/// Carried as one value and set once, at construction. Half a dozen places have
/// to agree on the answer — the terrain mesher that cuts the hole, the paver
/// that makes its rim opaque, the road that drops its raise-only clamp — and
/// a sampler that could be reconfigured after the fact is a sampler that can be
/// configured inconsistently.
#[derive(Debug, Clone, Copy)]
pub struct MeshOptions {
    /// Whether detail meshes are breakline-constrained (docs/GROUND.md §3);
    /// `--no-breaklines` turns it off.
    pub breaklines: bool,
    /// Whether the detail mesh stops at the kerb (docs/GROUND.md §3, "the
    /// hole"); `--no-hole` turns it off. Implied off without breaklines —
    /// there is no constrained mesh to cut.
    pub hole: bool,
}

impl Default for MeshOptions {
    fn default() -> MeshOptions {
        MeshOptions { breaklines: true, hole: true }
    }
}

pub struct GroundSampler {
    dem: Option<Dem>,
    ground: Arc<GroundStack>,
    /// The run's reference zoom, keying the per-zoom lattice resolution
    /// ([`terrain::grid_for`]) `surface` reads through.
    z_ref: u8,
    mesh: MeshOptions,
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
    /// The terrain meshes this worker has drawn, by tile, for the one-canvas
    /// rule: `surface` answers from the mesh the tile actually draws wherever
    /// one exists (`terrain::DrawnMesh`), so a building foot, a structure
    /// datum or a kerb reads the ground that is on screen rather than the
    /// mirror lattice it diverges from between constraints.
    drawn: HashMap<(u8, u32, u32), std::sync::Arc<terrain::DrawnMesh>>,
}

impl GroundSampler {
    pub fn new(
        dem: Option<Dem>,
        ground: Arc<GroundStack>,
        z_ref: u8,
        mesh: MeshOptions,
    ) -> GroundSampler {
        GroundSampler {
            dem,
            ground,
            z_ref,
            mesh,
            scratch: Vec::new(),
            corners: HashMap::new(),
            drawn: HashMap::new(),
        }
    }

    /// Whether zoom `z` is the detail rung — the only one that gets a
    /// breakline-constrained mesh, and so the only one that can cut a hole.
    fn is_detail(&self, z: u8) -> bool {
        if one_canvas() {
            // One canvas displacement (docs/GROUND.md §4, data/plans/
            // carmack-rewrite-plan-2026-08-29.md item ②): every rung that
            // draws asphalt cuts the hole under it and holds its benches in
            // the constrained mesh, so there is no rung on which ground is
            // drawn under the carriageway for the road to be clamped above.
            return z >= crate::priors::ROAD_SURFACE_MIN_ZOOM;
        }
        terrain::grid_for(z, self.z_ref) == terrain::TERRAIN_GRID_DETAIL
    }

    /// Whether the ground under the asphalt is cut away at zoom `z`.
    ///
    /// The single definition. [`GroundSampler::terrain_mesh`] cuts on it, the
    /// paver makes its rim opaque and builds its apron on it
    /// (`synth::pave_mesh`), and the road drops its raise-only clamp on it
    /// (`synth::road::on_ground`). Letting any of those spell the condition out
    /// for itself is how a plate ends up clamped while the roads meeting it are
    /// not, which is the disagreement `synth::height` exists to prevent.
    pub fn cuts_hole(&self, z: u8) -> bool {
        self.mesh.hole && self.mesh.breaklines && self.is_detail(z)
    }

    /// Whether the run has real elevation at all (a DEM was configured).
    pub fn has_elevation(&self) -> bool {
        self.dem.is_some()
    }

    /// The reference rung: the zoom whose lattice the solve was anchored to,
    /// and the one every per-zoom correction is stated against.
    pub fn z_ref(&self) -> u8 {
        self.z_ref
    }

    /// The engineered ground height at `(lon, lat)`, sampling the DEM at zoom
    /// `z`. Zero without a DEM (the flat parity world).
    pub fn ground(&mut self, lon: f64, lat: f64, z: u8) -> f64 {
        let raw = match &mut self.dem {
            Some(d) => d.elevation(lon, lat, z),
            None => 0.0,
        };
        let cell = self.cell_m(lat, z);
        self.ground.height(lon, lat, raw, cell, &mut self.scratch)
    }

    /// The metric width of one lattice cell at zoom `z` and this latitude —
    /// the resolution the ground is being asked at (see
    /// [`crate::ground::GroundStack::height`]). At the reference zoom the
    /// breakline-constrained mesh holds every bench exactly, so nothing is
    /// filtered out there; coarser rungs drop what they cannot draw.
    fn cell_m(&self, lat: f64, z: u8) -> f64 {
        // A rung whose mesh is constrained holds every bench exactly, so it
        // asks for the ground unfiltered; the resolution filter is only for
        // the plain lattice, which spikes on a bench narrower than its cell.
        if z >= self.z_ref || self.is_detail(z) {
            return 0.0;
        }
        let tile_deg = 360.0 / (1u64 << z) as f64;
        tile_deg * crate::scene::DEG_M * lat.to_radians().cos()
            / terrain::grid_for(z, self.z_ref) as f64
    }

    /// The engineered ground at a rendered-lattice corner, memoized (see
    /// [`GroundSampler::corners`]). The terrain mesh and `surface` both read
    /// corners through this, so each distinct corner costs one DEM sample per
    /// worker however many queries land on it.
    pub fn corner(&mut self, lon: f64, lat: f64, z: u8) -> f64 {
        let cell = self.cell_m(lat, z);
        corner_memo(
            &mut self.dem,
            &self.ground,
            &mut self.corners,
            &mut self.scratch,
            lon,
            lat,
            z,
            cell,
        )
    }

    /// The *rendered* ground at `(lon, lat)`: the engineered ground evaluated
    /// through the global zoom-`z` terrain lattice anchored at `bounds`, so it
    /// matches the triangulated mesh the client draws exactly.
    pub fn surface(&mut self, bounds: &Bounds, lon: f64, lat: f64, z: u8) -> f64 {
        let grid = terrain::grid_for(z, self.z_ref);
        terrain::surface_height(bounds, grid, lon, lat, &mut |a, o| self.corner(a, o, z))
    }

    /// The ground a thing standing on the drawn world reads: the mesh the
    /// tile draws where one is cached and covers the point (the one-canvas
    /// rule), the lattice mirror otherwise. For consumers whose neighbours
    /// are the *pixels* — a building foot — never for the datum field, whose
    /// stations must all reconstruct one basis along a span: mixing drawn
    /// stations off the asphalt with lattice stations inside the hole put a
    /// step back at every abutment (`seam.band_deck_step` 38 → 73 % over,
    /// measured 2026-08-30).
    pub fn surface_drawn(&mut self, bounds: &Bounds, lon: f64, lat: f64, z: u8) -> f64 {
        if one_canvas() && self.is_detail(z) {
            if let Some(h) = self.drawn_height(bounds, lon, lat, z) {
                return h;
            }
        }
        self.surface(bounds, lon, lat, z)
    }

    /// The drawn height at a point from this worker's cached mesh of the
    /// zoom-`z` tile containing it, if any.
    fn drawn_height(&self, bounds: &Bounds, lon: f64, lat: f64, z: u8) -> Option<f64> {
        let (tx, ty) = tile_xy(bounds, z);
        let m = self.drawn.get(&(z, tx, ty))?;
        let qx = crate::project::BUFFER + (lon - bounds.west) / bounds.width() * crate::project::EXTENT;
        let qy = crate::project::BUFFER + (lat - bounds.south) / bounds.height() * crate::project::EXTENT;
        m.height_at(qx, qy)
    }

    /// Records the mesh a tile draws, so [`GroundSampler::surface`] can read
    /// it (one canvas). Bounded: a worker keeps the last few tiles only.
    pub fn remember_drawn(&mut self, bounds: &Bounds, z: u8, mesh: &TerrainMesh) {
        if !one_canvas() {
            return;
        }
        const KEEP: usize = 8;
        if self.drawn.len() >= KEEP {
            self.drawn.clear();
        }
        let (tx, ty) = tile_xy(bounds, z);
        self.drawn.insert((z, tx, ty), std::sync::Arc::new(terrain::DrawnMesh::new(mesh.clone())));
    }

    /// The exact engineered roadbed height under `(lon, lat)` when the point
    /// lies fully inside a road earthwork's held width — a corridor's roadbed
    /// or a street bench — else `None`. The drape rides this at the reference
    /// zoom, where the rendered lattice is too coarse to hold a street-wide
    /// bench (see `synth::road::surface_height`).
    pub fn bed_target(&mut self, lon: f64, lat: f64) -> Option<f64> {
        self.ground.bed_target(lon, lat, &mut self.scratch)
    }

    /// The tile's terrain mesh: the engineered ground on the regular lattice,
    /// except at the detail resolution where bench contact lines cross the
    /// tile — there the mesh is the breakline-constrained triangulation that
    /// holds the benches exactly (docs/GROUND.md §3), falling back to the
    /// lattice when the triangulation abstains (invariant 6).
    pub fn terrain_mesh(
        &mut self,
        bounds: &Bounds,
        z: u8,
        regions: &[Region],
    ) -> (TerrainMesh, f64, f64) {
        let grid = terrain::grid_for(z, self.z_ref);
        if self.mesh.breaklines && self.is_detail(z) {
            // Pad the query by one cell so a line grazing the border still
            // constrains the edge cells it touches.
            let pad = bounds.width().max(bounds.height()) / grid as f64;
            let bbox =
                (bounds.west - pad, bounds.south - pad, bounds.east + pad, bounds.north + pad);
            let mut ids = Vec::new();
            let mut segments = Vec::new();
            self.ground.breaklines().query(bbox, &mut ids, &mut segments);
            let regions: &[Region] = if self.cuts_hole(z) { regions } else { &[] };
            if !segments.is_empty() || !regions.is_empty() {
                let (dem, ground, corners, scratch) =
                    (&mut self.dem, &self.ground, &mut self.corners, &mut self.scratch);
                if let Some(mesh) =
                    terrain_cdt::constrained_mesh(grid, bounds, &segments, regions, &mut |lon, lat| {
                        // The constrained mesh holds every bench exactly, so it
                        // asks for the ground unfiltered.
                        corner_memo(dem, ground, corners, scratch, lon, lat, z, 0.0)
                    })
                {
                    return mesh;
                }
            }
        }
        terrain::elevated_mesh(grid, bounds, |lon, lat| self.corner(lon, lat, z))
    }
}

/// Whether the one-canvas rule is on for this run (`ARPT_ONE_CANVAS=1`, an
/// opt-in until it is measured to hold): the hole is cut and the mesh
/// constrained at every asphalt rung, and the at-grade band reads its
/// profile plus the structure datum shift — the deck's own formula — with no
/// clamp. Read once: the sampler asks per vertex.
pub fn one_canvas() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("ARPT_ONE_CANVAS").is_some())
}

impl GroundSampler {
    /// S5 prototype falsifier: the one-mesh border of this tile, per class
    /// (see `terrain_cdt::one_mesh_border_probe`), built from the same
    /// breaklines and regions the drawn path uses.
    pub fn one_mesh_border_probe(
        &mut self,
        bounds: &Bounds,
        z: u8,
        regions: &[&Region],
        asphalt_edges: &[((u16, u16), (u16, u16))],
    ) -> Option<(Vec<(u16, u16, i32)>, Vec<(u16, u16)>)> {
        let grid = terrain::grid_for(z, self.z_ref);
        let pad = bounds.width().max(bounds.height()) / grid as f64;
        let bbox = (bounds.west - pad, bounds.south - pad, bounds.east + pad, bounds.north + pad);
        let mut ids = Vec::new();
        let mut segments = Vec::new();
        self.ground.breaklines().query(bbox, &mut ids, &mut segments);
        let (dem, ground, corners, scratch) =
            (&mut self.dem, &self.ground, &mut self.corners, &mut self.scratch);
        crate::terrain_cdt::one_mesh_border_probe(grid, bounds, &segments, regions, asphalt_edges, &mut |lon, lat| {
            corner_memo(dem, ground, corners, scratch, lon, lat, z, 0.0)
        })
    }
}

/// The tile grid index of `bounds` at zoom `z` (its south-west corner).
fn tile_xy(bounds: &Bounds, z: u8) -> (u32, u32) {
    let n = (1u64 << z) as f64;
    (
        ((bounds.west + 180.0) / 360.0 * n).round() as u32,
        ((bounds.south + 90.0) / 180.0 * n).round() as u32,
    )
}

/// [`GroundSampler::corner`] with the sampler's fields split apart, so a
/// closure holding the field borrows can call it (the borrow checker cannot
/// split `self` through a closure).
#[allow(clippy::too_many_arguments)]
fn corner_memo(
    dem: &mut Option<Dem>,
    ground: &GroundStack,
    corners: &mut HashMap<(u8, u64, u64), f64>,
    scratch: &mut Vec<u32>,
    lon: f64,
    lat: f64,
    z: u8,
    cell_m: f64,
) -> f64 {
    let key = (z, lon.to_bits(), lat.to_bits());
    if let Some(&h) = corners.get(&key) {
        return h;
    }
    let raw = match dem {
        Some(d) => d.elevation(lon, lat, z),
        None => 0.0,
    };
    let h = ground.height(lon, lat, raw, cell_m, scratch);
    if corners.len() >= CORNER_CAP {
        corners.clear();
    }
    corners.insert(key, h);
    h
}
