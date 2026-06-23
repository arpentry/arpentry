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
const M_PER_DEG_LAT: f64 = 110_540.0;
const M_PER_DEG_LON_EQUATOR: f64 = 111_320.0;

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
/// used to turn footprint geometry into ECEF surface normals.
struct Frame {
    clon: f64,
    clat: f64,
    east: [f64; 3],
    north: [f64; 3],
    up: [f64; 3],
    m_per_deg_lon: f64,
}

impl Frame {
    fn at_center(bounds: &Bounds) -> Frame {
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
    fn local_m(&self, lon: f64, lat: f64, z_mm: i32) -> [f64; 3] {
        [
            (lon - self.clon) * self.m_per_deg_lon,
            (lat - self.clat) * M_PER_DEG_LAT,
            z_mm as f64 / 1000.0,
        ]
    }

    /// Encodes an ENU-metre direction as an octahedral ECEF normal.
    fn encode_enu(&self, e: f64, n: f64, u: f64) -> (i8, i8) {
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
        Some(TerrainMesh { x: self.x, y: self.y, z: self.z, indices: self.indices, normals: self.normals })
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
    let height_mm = base_z + (height_m * 1000.0) as i32;
    let foot_z = base_z - (relief_m * 1000.0) as i32 - FOUNDATION_MARGIN_MM;

    let mut acc = Accum::default();
    match geom {
        Geometry::Polygon(p) => add_polygon(&mut acc, &frame, bounds, p, foot_z, height_mm, roof),
        Geometry::MultiPolygon(mp) => {
            for p in &mp.0 {
                add_polygon(&mut acc, &frame, bounds, p, foot_z, height_mm, roof);
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
    height_mm: i32,
    roof: &RoofParams,
) {
    let ring = open_ring(poly.exterior().0.as_slice(), bounds);
    if ring.len() < 3 || !centroid_in_proper(&ring) {
        return;
    }
    add_walls(acc, frame, bounds, &ring, foot_z, height_mm);
    add_roof(acc, frame, &ring, height_mm, roof);
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

fn add_roof(acc: &mut Accum, frame: &Frame, ring: &[Vtx], eave_z: i32, roof: &RoofParams) {
    let rise_mm = roof_rise_mm(frame, ring, roof);
    match roof.shape {
        RoofShape::Flat => add_flat_roof(acc, frame, ring, eave_z),
        RoofShape::Pyramidal => add_pyramidal_roof(acc, frame, ring, eave_z, rise_mm),
        RoofShape::Skillion => add_skillion_roof(acc, frame, ring, eave_z, rise_mm),
        RoofShape::Gabled => {
            // A clean gable needs a rectangle-ish footprint; fall back to flat
            // for complex outlines rather than emit self-intersecting facets.
            if ring.len() == 4 {
                add_gabled_roof(acc, frame, ring, eave_z, rise_mm);
            } else {
                add_flat_roof(acc, frame, ring, eave_z);
            }
        }
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

fn add_flat_roof(acc: &mut Accum, frame: &Frame, ring: &[Vtx], z: i32) {
    let up = encode_octahedral(frame.up[0], frame.up[1], frame.up[2]);
    let base: Vec<u32> = ring.iter().map(|v| acc.push(v.qx, v.qy, z, up)).collect();
    for [a, b, c] in earclip(ring) {
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

fn add_skillion_roof(acc: &mut Accum, frame: &Frame, ring: &[Vtx], eave_z: i32, rise_mm: i32) {
    // Single tilted plane: ramp z linearly from the south edge (low) to the
    // north edge (high) of the footprint's lat span.
    let (mut min_lat, mut max_lat) = (f64::INFINITY, f64::NEG_INFINITY);
    for v in ring {
        min_lat = min_lat.min(v.lat);
        max_lat = max_lat.max(v.lat);
    }
    let span = (max_lat - min_lat).max(1e-12);
    let zof = |v: &Vtx| eave_z + ((v.lat - min_lat) / span * rise_mm as f64) as i32;
    // Triangulate the cap in footprint order; each facet gets the plane normal.
    for [a, b, c] in earclip(ring) {
        facet(
            acc,
            frame,
            (ring[a], zof(&ring[a])),
            (ring[b], zof(&ring[b])),
            (ring[c], zof(&ring[c])),
        );
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
    fn gabled_roof_raises_a_ridge() {
        let b = Bounds::of_tile(16, 34000, 22000);
        let g = square(&b, b.width() * 0.2);
        let roof = RoofParams { shape: RoofShape::Gabled, roof_height_m: Some(4.0) };
        let m = build(&g, &b, 0.0, 0.0, 10.0, &roof).unwrap();
        let ridge = 10_000 + 4_000;
        assert!(m.z.iter().any(|&z| z == ridge), "ridge vertices should sit above the eaves");
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
}
