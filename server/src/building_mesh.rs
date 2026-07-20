//! Server-side 3D building mesh generation.
//!
//! Buildings ship as `MeshGeometry`: this module bakes the full 3D geometry —
//! extruded walls plus a roof whose shape comes from Overture's `roof_shape`
//! attribute (`flat` when absent). The client just renders the mesh (see
//! `client/src/tile/decode.c` `arpt_decode_building_mesh`); it has no footprint
//! extruder.
//!
//! Walls anchor at the highest ground under the footprint and sink past the
//! lowest ground (relief) plus a margin, so sloped terrain never reveals a gap.
//! Coordinates are tile-local quantized uint16 (x/y) and int32 millimetres (z),
//! matching `MeshGeometry`.
//!
//! Footprints with interior rings (courtyards, like EPFL's Rolex Learning
//! Centre) get walls around every ring and a roof cap triangulated with the
//! holes cut out, so the void reads through. The exterior is forced CCW and
//! holes CW so wall faces and the cap point the right way under back-face
//! culling.

use geo_types::{Coord, Geometry, Polygon};

use crate::project::{self, Bounds, BUFFER, EXTENT};
use crate::terrain::{encode_octahedral, TerrainMesh};

/// Extra depth, below the footprint's lowest ground, that walls sink to (2 m):
/// covers sub-metre relief rounding and DEM jitter on flat ground. The buried
/// part is hidden by the opaque terrain.
const FOUNDATION_MARGIN_MM: i32 = 2000;

/// Default roof rise (eave-to-ridge) when `roof_height` is absent, as a fraction
/// of the footprint's short side, capped by [`MAX_ROOF_RISE_M`]. Overture rarely
/// carries `roof_height`, so most pitched roofs use this.
const ROOF_RISE_FRACTION: f64 = 0.5;
const MAX_ROOF_RISE_M: f64 = 6.0;
const MIN_ROOF_RISE_M: f64 = 1.0;

/// Metres per degree, for the local-tangent normal computation. Latitude is
/// near-constant; longitude is scaled by the tile-centre latitude.
pub(crate) const M_PER_DEG_LAT: f64 = 110_540.0;
pub(crate) const M_PER_DEG_LON_EQUATOR: f64 = 111_320.0;

/// Roof geometry kind, parsed from Overture's `roof_shape`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RoofShape {
    Flat,
    Gabled,
    Pyramidal,
    Skillion,
}

impl RoofShape {
    /// Maps an Overture `roof_shape` string to a supported shape. Unsupported or
    /// missing values fall back to [`RoofShape::Flat`]. `hipped`/`half_hipped`
    /// are approximated as gabled (a true hip needs a straight skeleton).
    pub fn parse(s: &str) -> RoofShape {
        match s {
            "gabled" | "hipped" | "half_hipped" | "round" | "gambrel" | "mansard" => {
                RoofShape::Gabled
            }
            "pyramidal" | "dome" | "onion" | "cone" => RoofShape::Pyramidal,
            "skillion" | "lean_to" | "mono_pitch" | "shed" => RoofShape::Skillion,
            _ => RoofShape::Flat,
        }
    }
}

/// Roof attributes for one building (from its tile properties).
#[derive(Clone, Copy, Debug)]
pub struct RoofParams {
    pub shape: RoofShape,
    /// Eave-to-ridge height in metres, when known.
    pub roof_height_m: Option<f64>,
}

impl Default for RoofParams {
    fn default() -> Self {
        RoofParams { shape: RoofShape::Flat, roof_height_m: None }
    }
}

/// A unit ENU→ECEF basis at a point, plus the local metres-per-degree scale,
/// used to turn footprint geometry into ECEF surface normals. Shared with
/// `synth::structure`, which extrudes its box prisms in the same ENU frame.
pub(crate) struct Frame {
    pub(crate) clon: f64,
    pub(crate) clat: f64,
    pub(crate) east: [f64; 3],
    pub(crate) north: [f64; 3],
    pub(crate) up: [f64; 3],
    pub(crate) m_per_deg_lon: f64,
}

impl Frame {
    pub(crate) fn at_center(bounds: &Bounds) -> Frame {
        let clon = (bounds.west + bounds.east) * 0.5;
        let clat = (bounds.south + bounds.north) * 0.5;
        let (slon, coslon) = clon.to_radians().sin_cos();
        let (slat, coslat) = clat.to_radians().sin_cos();
        Frame {
            clon,
            clat,
            east: [-slon, coslon, 0.0],
            north: [-slat * coslon, -slat * slon, coslat],
            up: [coslat * coslon, coslat * slon, slat],
            m_per_deg_lon: M_PER_DEG_LON_EQUATOR * clat.to_radians().cos(),
        }
    }

    /// ENU-metre offset of a (lon, lat, z_mm) point from the tile centre.
    pub(crate) fn local_m(&self, lon: f64, lat: f64, z_mm: i32) -> [f64; 3] {
        [
            (lon - self.clon) * self.m_per_deg_lon,
            (lat - self.clat) * M_PER_DEG_LAT,
            z_mm as f64 / 1000.0,
        ]
    }

    /// Encodes an ENU-metre direction as an octahedral ECEF normal.
    pub(crate) fn encode_enu(&self, e: f64, n: f64, u: f64) -> (i8, i8) {
        let nx = e * self.east[0] + n * self.north[0] + u * self.up[0];
        let ny = e * self.east[1] + n * self.north[1] + u * self.up[1];
        let nz = e * self.east[2] + n * self.north[2] + u * self.up[2];
        let len = (nx * nx + ny * ny + nz * nz).sqrt();
        if len < 1e-12 {
            return encode_octahedral(self.up[0], self.up[1], self.up[2]);
        }
        encode_octahedral(nx / len, ny / len, nz / len)
    }
}

/// A footprint vertex in both quantized tile space and geographic space (the
/// latter drives normal computation).
#[derive(Clone, Copy)]
struct Vtx {
    qx: u16,
    qy: u16,
    lon: f64,
    lat: f64,
}

/// Accumulates mesh vertices and triangles (one `MeshGeometry` per feature).
#[derive(Default)]
struct Accum {
    x: Vec<u16>,
    y: Vec<u16>,
    z: Vec<i32>,
    indices: Vec<u32>,
    normals: Vec<i8>,
}

impl Accum {
    fn push(&mut self, qx: u16, qy: u16, z: i32, n: (i8, i8)) -> u32 {
        let i = self.x.len() as u32;
        self.x.push(qx);
        self.y.push(qy);
        self.z.push(z);
        self.normals.push(n.0);
        self.normals.push(n.1);
        i
    }

    fn tri(&mut self, a: u32, b: u32, c: u32) {
        self.indices.extend_from_slice(&[a, b, c]);
    }

    fn into_mesh(self) -> Option<TerrainMesh> {
        if self.indices.is_empty() {
            return None;
        }
        Some(TerrainMesh { x: self.x, y: self.y, z: self.z, indices: self.indices, normals: self.normals, edge_across: Vec::new() })
    }
}

/// Builds a building mesh for one feature's footprint, or `None` when there is
/// nothing to emit (no polygon, degenerate ring, or non-positive height).
///
/// `base_elev_m` is the highest ground under the footprint (the wall top is
/// measured from here); `relief_m` is the highest-minus-lowest spread (the wall
/// foot drops this far plus the margin). Both come from the DEM stamping in
/// `pipeline::stamp_elevations`.
pub fn build(
    geom: &Geometry,
    bounds: &Bounds,
    base_elev_m: f64,
    relief_m: f64,
    height_m: f64,
    roof: &RoofParams,
) -> Option<TerrainMesh> {
    if height_m <= 0.0 {
        return None;
    }
    let frame = Frame::at_center(bounds);
    let base_z = project::quantize_z(base_elev_m);
    let top_z = base_z + (height_m * 1000.0) as i32;
    let foot_z = base_z - (relief_m * 1000.0) as i32 - FOUNDATION_MARGIN_MM;

    let mut acc = Accum::default();
    match geom {
        Geometry::Polygon(p) => add_polygon(&mut acc, &frame, bounds, p, foot_z, base_z, top_z, roof),
        Geometry::MultiPolygon(mp) => {
            for p in &mp.0 {
                add_polygon(&mut acc, &frame, bounds, p, foot_z, base_z, top_z, roof);
            }
        }
        _ => return None,
    }
    acc.into_mesh()
}

/// Whether the ring centroid lies in the tile proper (not the buffer zone), so a
/// building straddling a tile boundary is meshed by exactly one tile. Mirrors
/// the client's `building_in_tile_proper`.
fn centroid_in_proper(ring: &[Vtx]) -> bool {
    let n = ring.len() as u64;
    if n == 0 {
        return false;
    }
    let sx: u64 = ring.iter().map(|v| v.qx as u64).sum();
    let sy: u64 = ring.iter().map(|v| v.qy as u64).sum();
    let cx = (sx / n) as f64;
    let cy = (sy / n) as f64;
    let lo = BUFFER;
    let hi = BUFFER + EXTENT;
    cx >= lo && cx < hi && cy >= lo && cy < hi
}

fn add_polygon(
    acc: &mut Accum,
    frame: &Frame,
    bounds: &Bounds,
    poly: &Polygon,
    foot_z: i32,
    base_z: i32,
    top_z: i32,
    roof: &RoofParams,
) {
    let mut outer = open_ring(poly.exterior().0.as_slice(), bounds);
    if outer.len() < 3 || !centroid_in_proper(&outer) {
        return;
    }
    // Force the exterior CCW so wall faces and the roof cap point outward/up;
    // courtyard rings are forced CW so their walls face into the courtyard and
    // earcut cuts them out of the cap. Quantized winding matches geographic
    // winding (qy grows northward), so this is consistent with the lon/lat
    // normals computed in `add_walls`.
    ensure_winding(&mut outer, Winding::Ccw);
    let mut holes: Vec<Vec<Vtx>> = Vec::new();
    for interior in poly.interiors() {
        let mut hole = open_ring(interior.0.as_slice(), bounds);
        if hole.len() < 3 {
            continue;
        }
        ensure_winding(&mut hole, Winding::Cw);
        holes.push(hole);
    }

    // Walls rise to the eave, and a pitched roof's rise sits *under* `top_z` so
    // the apex lands at the feature's height (the source `height`) instead of
    // stacking above it and overshooting. The rise can't exceed the building's
    // own height, so the eave stays at or above the ground anchor; a flat cap has
    // zero rise, so its eave is the top and the walls are unchanged.
    let (shape, rise_mm) = resolve_roof(frame, &outer, &holes, roof);
    let rise_mm = rise_mm.clamp(0, top_z - base_z);
    let eave_z = top_z - rise_mm;

    add_walls(acc, frame, bounds, &outer, foot_z, eave_z);
    for hole in &holes {
        add_walls(acc, frame, bounds, hole, foot_z, eave_z);
    }
    add_roof(acc, frame, &outer, &holes, eave_z, shape, rise_mm);
}

/// Ring orientation in tile space (`qy` grows northward, so this also matches
/// geographic winding).
#[derive(Clone, Copy, PartialEq, Eq)]
enum Winding {
    Ccw,
    Cw,
}

/// Twice the signed area of a ring in quantized space; positive is CCW.
fn signed_area2(ring: &[Vtx]) -> i64 {
    let n = ring.len();
    let mut a = 0i64;
    for i in 0..n {
        let j = (i + 1) % n;
        a += ring[i].qx as i64 * ring[j].qy as i64 - ring[j].qx as i64 * ring[i].qy as i64;
    }
    a
}

/// Reverses the ring in place if it does not already have the wanted winding.
fn ensure_winding(ring: &mut [Vtx], want: Winding) {
    let have = if signed_area2(ring) >= 0 { Winding::Ccw } else { Winding::Cw };
    if have != want {
        ring.reverse();
    }
}

/// Drops the repeated closing vertex and quantizes each ring coordinate.
fn open_ring(coords: &[Coord], bounds: &Bounds) -> Vec<Vtx> {
    let n = if coords.len() > 1 && coords.first() == coords.last() {
        coords.len() - 1
    } else {
        coords.len()
    };
    coords[..n]
        .iter()
        .map(|c| Vtx {
            qx: project::quantize_x(c.x, bounds),
            qy: project::quantize_y(c.y, bounds),
            lon: c.x,
            lat: c.y,
        })
        .collect()
}

/// Emits a wall quad per footprint edge (foot → top), with outward-perpendicular
/// normals.
fn add_walls(acc: &mut Accum, frame: &Frame, bounds: &Bounds, ring: &[Vtx], foot_z: i32, top_z: i32) {
    let n = ring.len();
    let (lon_span, lat_span) = (bounds.width(), bounds.height());
    for e in 0..n {
        let a = ring[e];
        let b = ring[(e + 1) % n];
        // Edge direction in normalized tile fractions.
        let dx = (b.lon - a.lon) / lon_span;
        let dy = (b.lat - a.lat) / lat_span;
        let len = (dx * dx + dy * dy).sqrt().max(1e-12);
        // Outward perpendicular (dy, -dx), scaled back to degree spans.
        let perp_e = (dy / len) * lon_span;
        let perp_n = (-dx / len) * lat_span;
        let plen = (perp_e * perp_e + perp_n * perp_n).sqrt().max(1e-12);
        let nrm = frame.encode_enu(perp_e / plen, perp_n / plen, 0.0);

        let v0 = acc.push(a.qx, a.qy, foot_z, nrm);
        let v1 = acc.push(a.qx, a.qy, top_z, nrm);
        let v2 = acc.push(b.qx, b.qy, top_z, nrm);
        let v3 = acc.push(b.qx, b.qy, foot_z, nrm);
        acc.tri(v0, v2, v1);
        acc.tri(v0, v3, v2);
    }
}

/// Resolves the roof actually built for a footprint after fallbacks: the
/// effective shape and its rise in millimetres. Pyramidal (centroid apex) and
/// gabled (ridge over a quad) have no sensible analogue when the footprint has
/// courtyards — and gabled only over a 4-vertex quad — so those degrade to a flat
/// cap, the only shape that triangulates holes. A flat cap has zero rise, so the
/// walls below it are not shortened. This is the single source of truth the wall
/// extrusion and [`add_roof`] both read, so the eave and the apex agree.
fn resolve_roof(frame: &Frame, outer: &[Vtx], holes: &[Vec<Vtx>], roof: &RoofParams) -> (RoofShape, i32) {
    let shape = match roof.shape {
        RoofShape::Skillion => RoofShape::Skillion,
        RoofShape::Pyramidal if holes.is_empty() => RoofShape::Pyramidal,
        RoofShape::Gabled if holes.is_empty() && outer.len() == 4 => RoofShape::Gabled,
        // Flat, or a complex/holed outline that can only be a flat cap.
        _ => RoofShape::Flat,
    };
    let rise_mm = if shape == RoofShape::Flat { 0 } else { roof_rise_mm(frame, outer, roof) };
    (shape, rise_mm)
}

/// Caps the walls with the roof: an eave-level flat cap, or a pitched roof rising
/// `rise_mm` from the eave to the apex. `shape`/`rise_mm` come pre-resolved by
/// [`resolve_roof`], so the arms here are exhaustive (pyramidal implies no holes,
/// gabled a quad) and the apex matches the eave the walls were extruded to. Flat
/// and skillion tessellate the cap directly and so carry holes through.
fn add_roof(acc: &mut Accum, frame: &Frame, outer: &[Vtx], holes: &[Vec<Vtx>], eave_z: i32, shape: RoofShape, rise_mm: i32) {
    match shape {
        RoofShape::Flat => add_flat_roof(acc, frame, outer, holes, eave_z),
        RoofShape::Skillion => add_skillion_roof(acc, frame, outer, holes, eave_z, rise_mm),
        RoofShape::Pyramidal => add_pyramidal_roof(acc, frame, outer, eave_z, rise_mm),
        RoofShape::Gabled => add_gabled_roof(acc, frame, outer, eave_z, rise_mm),
    }
}

/// Roof rise in millimetres: `roof_height` when present, else a fraction of the
/// footprint's short side, clamped.
fn roof_rise_mm(frame: &Frame, ring: &[Vtx], roof: &RoofParams) -> i32 {
    let rise_m = roof.roof_height_m.filter(|h| *h > 0.0).unwrap_or_else(|| {
        let (w, h) = footprint_extent_m(frame, ring);
        (w.min(h) * ROOF_RISE_FRACTION).clamp(MIN_ROOF_RISE_M, MAX_ROOF_RISE_M)
    });
    (rise_m.min(MAX_ROOF_RISE_M).max(0.0) * 1000.0) as i32
}

/// Width/height of the footprint's axis-aligned bounding box, in metres.
fn footprint_extent_m(frame: &Frame, ring: &[Vtx]) -> (f64, f64) {
    let (mut min_e, mut max_e) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut min_n, mut max_n) = (f64::INFINITY, f64::NEG_INFINITY);
    for v in ring {
        let p = frame.local_m(v.lon, v.lat, 0);
        min_e = min_e.min(p[0]);
        max_e = max_e.max(p[0]);
        min_n = min_n.min(p[1]);
        max_n = max_n.max(p[1]);
    }
    ((max_e - min_e).max(0.0), (max_n - min_n).max(0.0))
}

fn add_flat_roof(acc: &mut Accum, frame: &Frame, outer: &[Vtx], holes: &[Vec<Vtx>], z: i32) {
    let up = encode_octahedral(frame.up[0], frame.up[1], frame.up[2]);
    let (verts, hole_starts) = combine_rings(outer, holes);
    let base: Vec<u32> = verts.iter().map(|v| acc.push(v.qx, v.qy, z, up)).collect();
    for [a, b, c] in triangulate_cap(&verts, &hole_starts) {
        acc.tri(base[a], base[b], base[c]);
    }
}

/// A roof facet from three footprint/ridge points; computes a flat-shaded
/// outward normal from the ECEF cross product and emits the triangle.
fn facet(acc: &mut Accum, frame: &Frame, a: (Vtx, i32), b: (Vtx, i32), c: (Vtx, i32)) {
    let pa = frame.local_m(a.0.lon, a.0.lat, a.1);
    let pb = frame.local_m(b.0.lon, b.0.lat, b.1);
    let pc = frame.local_m(c.0.lon, c.0.lat, c.1);
    let u = [pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]];
    let v = [pc[0] - pa[0], pc[1] - pa[1], pc[2] - pa[2]];
    let mut nrm = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
    if nrm[2] < 0.0 {
        nrm = [-nrm[0], -nrm[1], -nrm[2]]; // keep roof normals pointing up
    }
    let n = frame.encode_enu(nrm[0], nrm[1], nrm[2]);
    let ia = acc.push(a.0.qx, a.0.qy, a.1, n);
    let ib = acc.push(b.0.qx, b.0.qy, b.1, n);
    let ic = acc.push(c.0.qx, c.0.qy, c.1, n);
    acc.tri(ia, ib, ic);
}

fn add_pyramidal_roof(acc: &mut Accum, frame: &Frame, ring: &[Vtx], eave_z: i32, rise_mm: i32) {
    let apex = centroid(ring);
    let apex_z = eave_z + rise_mm;
    let n = ring.len();
    for e in 0..n {
        let a = ring[e];
        let b = ring[(e + 1) % n];
        facet(acc, frame, (a, eave_z), (b, eave_z), (apex, apex_z));
    }
}

fn add_skillion_roof(acc: &mut Accum, frame: &Frame, outer: &[Vtx], holes: &[Vec<Vtx>], eave_z: i32, rise_mm: i32) {
    let (verts, hole_starts) = combine_rings(outer, holes);
    // Single tilted plane: ramp z linearly from the south edge (low) to the
    // north edge (high) of the footprint's lat span.
    let (mut min_lat, mut max_lat) = (f64::INFINITY, f64::NEG_INFINITY);
    for v in &verts {
        min_lat = min_lat.min(v.lat);
        max_lat = max_lat.max(v.lat);
    }
    let span = (max_lat - min_lat).max(1e-12);
    let zof = |v: &Vtx| eave_z + ((v.lat - min_lat) / span * rise_mm as f64) as i32;
    // Triangulate the cap (cutting any courtyards); each facet gets the plane
    // normal.
    for [a, b, c] in triangulate_cap(&verts, &hole_starts) {
        facet(
            acc,
            frame,
            (verts[a], zof(&verts[a])),
            (verts[b], zof(&verts[b])),
            (verts[c], zof(&verts[c])),
        );
    }
}

/// Flattens an outer ring plus its holes into one vertex list and the start
/// index of each hole (the form earcut expects).
fn combine_rings(outer: &[Vtx], holes: &[Vec<Vtx>]) -> (Vec<Vtx>, Vec<usize>) {
    let mut verts = outer.to_vec();
    let mut starts = Vec::with_capacity(holes.len());
    for hole in holes {
        starts.push(verts.len());
        verts.extend_from_slice(hole);
    }
    (verts, starts)
}

/// Triangulates a roof cap, returning index triples into `verts`. Without holes
/// this is the in-house [`earclip`] (so existing buildings are untouched); with
/// holes it bridges and ear-clips via earcut. The outer ring is forced CCW by
/// the caller, so both produce CCW (upward-front-facing) triangles.
fn triangulate_cap(verts: &[Vtx], hole_starts: &[usize]) -> Vec<[usize; 3]> {
    if hole_starts.is_empty() {
        return earclip(verts);
    }
    let mut coords: Vec<f64> = Vec::with_capacity(verts.len() * 2);
    for v in verts {
        coords.push(v.qx as f64);
        coords.push(v.qy as f64);
    }
    match earcutr::earcut(&coords, hole_starts, 2) {
        Ok(idx) => idx.chunks_exact(3).map(|t| [t[0], t[1], t[2]]).collect(),
        // Degenerate footprint (e.g. self-touching): cap the outer ring only
        // rather than emit nothing.
        Err(_) => earclip(&verts[..hole_starts[0]]),
    }
}

/// Gabled roof for a 4-vertex (quad) footprint: ridge along the long axis,
/// centred over the two short edges, with two sloped faces and two gable-end
/// triangles closing the shape up to the ridge.
fn add_gabled_roof(acc: &mut Accum, frame: &Frame, ring: &[Vtx], eave_z: i32, rise_mm: i32) {
    let (v0, v1, v2, v3) = (ring[0], ring[1], ring[2], ring[3]);
    let l01 = edge_len_m(frame, v0, v1);
    let l12 = edge_len_m(frame, v1, v2);
    let ridge_z = eave_z + rise_mm;

    // Ridge runs parallel to the longer pair of edges, over the midpoints of the
    // shorter pair.
    let (e_a0, e_a1, e_b0, e_b1, r0, r1) = if l01 >= l12 {
        // Long edges: v0v1 and v2v3. Ridge over mids of v1v2 and v3v0.
        (v0, v1, v2, v3, midpoint(v1, v2), midpoint(v3, v0))
    } else {
        // Long edges: v1v2 and v3v0. Ridge over mids of v0v1 and v2v3.
        (v1, v2, v3, v0, midpoint(v2, v3), midpoint(v0, v1))
    };

    // Two sloped faces (eave edge → ridge), each split into two triangles.
    facet(acc, frame, (e_a0, eave_z), (e_a1, eave_z), (r0, ridge_z));
    facet(acc, frame, (e_a0, eave_z), (r0, ridge_z), (r1, ridge_z));
    facet(acc, frame, (e_b0, eave_z), (e_b1, eave_z), (r1, ridge_z));
    facet(acc, frame, (e_b0, eave_z), (r1, ridge_z), (r0, ridge_z));

    // Gable-end triangles (vertical, eave edge mids → ridge).
    facet(acc, frame, (e_a1, eave_z), (e_b0, eave_z), (r0, ridge_z));
    facet(acc, frame, (e_b1, eave_z), (e_a0, eave_z), (r1, ridge_z));
}

fn edge_len_m(frame: &Frame, a: Vtx, b: Vtx) -> f64 {
    let pa = frame.local_m(a.lon, a.lat, 0);
    let pb = frame.local_m(b.lon, b.lat, 0);
    ((pb[0] - pa[0]).powi(2) + (pb[1] - pa[1]).powi(2)).sqrt()
}

fn midpoint(a: Vtx, b: Vtx) -> Vtx {
    Vtx {
        qx: ((a.qx as u32 + b.qx as u32) / 2) as u16,
        qy: ((a.qy as u32 + b.qy as u32) / 2) as u16,
        lon: (a.lon + b.lon) * 0.5,
        lat: (a.lat + b.lat) * 0.5,
    }
}

fn centroid(ring: &[Vtx]) -> Vtx {
    let n = ring.len() as f64;
    let (mut sx, mut sy, mut slon, mut slat) = (0u64, 0u64, 0.0, 0.0);
    for v in ring {
        sx += v.qx as u64;
        sy += v.qy as u64;
        slon += v.lon;
        slat += v.lat;
    }
    Vtx {
        qx: (sx / ring.len() as u64) as u16,
        qy: (sy / ring.len() as u64) as u16,
        lon: slon / n,
        lat: slat / n,
    }
}

/// Ear-clipping triangulation of a simple polygon, returning local index
/// triples — a port of the client's `earclip_triangulate`. Works in quantized
/// integer space.
fn earclip(ring: &[Vtx]) -> Vec<[usize; 3]> {
    let n = ring.len();
    let mut out = Vec::new();
    if n < 3 {
        return out;
    }
    if n == 3 {
        out.push([0, 1, 2]);
        return out;
    }

    let x = |i: usize| ring[i].qx as i64;
    let y = |i: usize| ring[i].qy as i64;
    let cross = |a: usize, b: usize, c: usize| -> i64 {
        (x(b) - x(a)) * (y(c) - y(a)) - (y(b) - y(a)) * (x(c) - x(a))
    };
    let in_tri = |p: usize, a: usize, b: usize, c: usize| -> bool {
        let d1 = cross(a, b, p);
        let d2 = cross(b, c, p);
        let d3 = cross(c, a, p);
        (d1 > 0 && d2 > 0 && d3 > 0) || (d1 < 0 && d2 < 0 && d3 < 0)
    };

    let mut area2 = 0i64;
    for i in 0..n {
        let j = (i + 1) % n;
        area2 += x(i) * y(j) - x(j) * y(i);
    }
    let ccw = area2 > 0;

    let mut prv: Vec<usize> = (0..n).map(|i| (i + n - 1) % n).collect();
    let mut nxt: Vec<usize> = (0..n).map(|i| (i + 1) % n).collect();

    let mut remaining = n;
    let mut ear = 0usize;
    let mut attempts = 0usize;
    while remaining > 3 && attempts < remaining * remaining {
        let p = prv[ear];
        let c = ear;
        let nx = nxt[ear];
        let cr = cross(p, c, nx);
        let convex = if ccw { cr >= 0 } else { cr <= 0 };
        if convex {
            let mut blocked = false;
            let mut test = nxt[nx];
            while test != p {
                if in_tri(test, p, c, nx) {
                    blocked = true;
                    break;
                }
                test = nxt[test];
            }
            if !blocked {
                out.push([p, c, nx]);
                nxt[p] = nx;
                prv[nx] = p;
                remaining -= 1;
                attempts = 0;
                ear = nx;
                continue;
            }
        }
        ear = nxt[ear];
        attempts += 1;
    }

    if remaining == 3 {
        let a = ear;
        let b = nxt[a];
        let c = nxt[b];
        out.push([a, b, c]);
    } else if remaining > 3 {
        // Degenerate fallback: fan the rest from the current ear.
        let a = ear;
        let mut b = nxt[a];
        while remaining > 2 {
            let c = nxt[b];
            out.push([a, b, c]);
            b = c;
            remaining -= 1;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo_types::{Coord, LineString, Polygon};

    /// A square footprint, centred in the tile proper, with the given side in
    /// degrees.
    fn square(bounds: &Bounds, side: f64) -> Geometry {
        let cx = (bounds.west + bounds.east) * 0.5;
        let cy = (bounds.south + bounds.north) * 0.5;
        let h = side * 0.5;
        let ring = vec![
            Coord { x: cx - h, y: cy - h },
            Coord { x: cx + h, y: cy - h },
            Coord { x: cx + h, y: cy + h },
            Coord { x: cx - h, y: cy + h },
            Coord { x: cx - h, y: cy - h },
        ];
        Geometry::Polygon(Polygon::new(LineString(ring), vec![]))
    }

    fn decode_oct(ox: i8, oy: i8) -> [f64; 3] {
        let mut u = ox as f64 / 127.0;
        let mut v = oy as f64 / 127.0;
        let nz = 1.0 - u.abs() - v.abs();
        if nz < 0.0 {
            let ou = u;
            u = (1.0 - v.abs()) * if ou >= 0.0 { 1.0 } else { -1.0 };
            v = (1.0 - ou.abs()) * if v >= 0.0 { 1.0 } else { -1.0 };
        }
        let len = (u * u + v * v + nz * nz).sqrt();
        [u / len, v / len, nz / len]
    }

    #[test]
    fn flat_roof_counts() {
        let b = Bounds::of_tile(16, 34000, 22000);
        let g = square(&b, b.width() * 0.2);
        let m = build(&g, &b, 100.0, 0.0, 10.0, &RoofParams::default()).unwrap();
        // 4 walls × 4 verts + 4 roof verts = 20 vertices.
        assert_eq!(m.x.len(), 20);
        assert_eq!(m.y.len(), 20);
        assert_eq!(m.z.len(), 20);
        assert_eq!(m.normals.len(), 40);
        // 4 walls × 6 + roof (4-gon → 2 tris × 3) = 24 + 6 = 30 indices.
        assert_eq!(m.indices.len(), 30);
        // Every index in range.
        assert!(m.indices.iter().all(|&i| (i as usize) < m.x.len()));
    }

    #[test]
    fn zero_height_emits_nothing() {
        let b = Bounds::of_tile(16, 34000, 22000);
        let g = square(&b, b.width() * 0.2);
        assert!(build(&g, &b, 100.0, 0.0, 0.0, &RoofParams::default()).is_none());
    }

    #[test]
    fn wall_top_and_foot_match_height_and_relief() {
        let b = Bounds::of_tile(16, 34000, 22000);
        let g = square(&b, b.width() * 0.2);
        let m = build(&g, &b, 50.0, 4.0, 12.0, &RoofParams::default()).unwrap();
        let base = project::quantize_z(50.0);
        let top = base + 12_000;
        let foot = base - 4_000 - FOUNDATION_MARGIN_MM;
        assert!(m.z.iter().any(|&z| z == top), "expected a wall-top vertex");
        assert!(m.z.iter().any(|&z| z == foot), "expected a sunk wall-foot vertex");
    }

    #[test]
    fn gabled_roof_raises_a_ridge_within_the_building_height() {
        let b = Bounds::of_tile(16, 34000, 22000);
        let g = square(&b, b.width() * 0.2);
        let roof = RoofParams { shape: RoofShape::Gabled, roof_height_m: Some(4.0) };
        let m = build(&g, &b, 0.0, 0.0, 10.0, &roof).unwrap();
        // The apex reaches the building height (10 m); the eaves sit a roof-rise
        // below it, so the roof fits *under* `height` instead of overshooting it.
        let ridge = 10_000;
        let eave = 10_000 - 4_000;
        assert!(m.z.iter().any(|&z| z == ridge), "ridge sits at the building height");
        assert!(m.z.iter().any(|&z| z == eave), "eaves sit a roof-rise below the apex");
        assert!(m.z.iter().all(|&z| z <= ridge), "nothing pokes above the building height");
    }

    #[test]
    fn normals_are_unit_length() {
        let b = Bounds::of_tile(16, 34000, 22000);
        let g = square(&b, b.width() * 0.2);
        let m = build(&g, &b, 0.0, 0.0, 10.0, &RoofParams::default()).unwrap();
        for i in 0..m.x.len() {
            let v = decode_oct(m.normals[i * 2], m.normals[i * 2 + 1]);
            let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
            assert!((len - 1.0).abs() < 1e-6, "normal {i} not unit: {len}");
        }
    }

    #[test]
    fn buffer_zone_building_is_skipped() {
        // A footprint centred well outside the tile proper (in the south-west
        // buffer) must not be meshed — the owning tile handles it.
        let b = Bounds::of_tile(16, 34000, 22000);
        let edge = b.west - b.width() * 0.4;
        let ring = vec![
            Coord { x: edge, y: b.south },
            Coord { x: edge + b.width() * 0.05, y: b.south },
            Coord { x: edge + b.width() * 0.05, y: b.south + b.height() * 0.05 },
            Coord { x: edge, y: b.south + b.height() * 0.05 },
            Coord { x: edge, y: b.south },
        ];
        let g = Geometry::Polygon(Polygon::new(LineString(ring), vec![]));
        assert!(build(&g, &b, 0.0, 0.0, 10.0, &RoofParams::default()).is_none());
    }

    /// A square footprint with a concentric square courtyard (hole), both
    /// centred in the tile proper. `outer`/`inner` are the side lengths in
    /// degrees. The exterior is CCW and the hole CW, matching OGC convention.
    fn square_with_hole(bounds: &Bounds, outer: f64, inner: f64) -> Geometry {
        let cx = (bounds.west + bounds.east) * 0.5;
        let cy = (bounds.south + bounds.north) * 0.5;
        let ring = |side: f64, ccw: bool| {
            let h = side * 0.5;
            let mut r = vec![
                Coord { x: cx - h, y: cy - h },
                Coord { x: cx + h, y: cy - h },
                Coord { x: cx + h, y: cy + h },
                Coord { x: cx - h, y: cy + h },
                Coord { x: cx - h, y: cy - h },
            ];
            if !ccw {
                r.reverse();
            }
            LineString(r)
        };
        Geometry::Polygon(Polygon::new(ring(outer, true), vec![ring(inner, false)]))
    }

    #[test]
    fn courtyard_adds_inner_walls_and_perforates_roof() {
        let b = Bounds::of_tile(16, 34000, 22000);
        let holed = square_with_hole(&b, b.width() * 0.3, b.width() * 0.15);
        let m = build(&holed, &b, 0.0, 0.0, 10.0, &RoofParams::default()).unwrap();

        // Outer wall loop: 4 quads × 4 verts = 16. Inner (courtyard) wall loop:
        // another 16. Flat roof over the 8-vertex annulus pushes those 8 verts.
        assert_eq!(m.x.len(), 40, "outer + inner walls + annulus cap");
        // Walls: 8 quads × 6 = 48 indices. The square-with-square-hole annulus
        // ear-cuts to 8 triangles = 24 indices (a covered roof would be 6, an
        // ignored hole would drop the inner walls entirely).
        assert_eq!(m.indices.len(), 72);
        assert!(m.indices.iter().all(|&i| (i as usize) < m.x.len()));

        // The footprint centre sits inside the courtyard, so no up-facing cap
        // triangle may contain it: the roof is perforated, not solid.
        let top_z = 10_000;
        let centre = ((BUFFER + EXTENT / 2.0) as i64, (BUFFER + EXTENT / 2.0) as i64);
        let in_tri = |a: usize, b: usize, c: usize| -> bool {
            let p = |i: usize| (m.x[i] as i64, m.y[i] as i64);
            let (ax, ay) = p(a);
            let (bx, by) = p(b);
            let (cx, cy) = p(c);
            let s = |(px, py): (i64, i64), (qx, qy): (i64, i64)| {
                (qx - px) * (centre.1 - py) - (qy - py) * (centre.0 - px)
            };
            let d1 = s((ax, ay), (bx, by));
            let d2 = s((bx, by), (cx, cy));
            let d3 = s((cx, cy), (ax, ay));
            (d1 >= 0 && d2 >= 0 && d3 >= 0) || (d1 <= 0 && d2 <= 0 && d3 <= 0)
        };
        for t in m.indices.chunks_exact(3) {
            let (i, j, k) = (t[0] as usize, t[1] as usize, t[2] as usize);
            if m.z[i] == top_z && m.z[j] == top_z && m.z[k] == top_z {
                assert!(!in_tri(i, j, k), "roof cap covers the courtyard centre");
            }
        }
    }
}
