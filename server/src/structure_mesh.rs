//! Server-side 3D geometry for road structures — bridges and tunnels — swept as
//! box prisms along the road's reconstructed surface profile (`crate::structures`).
//!
//! Overture marks a bridge or tunnel as a non-ground span of a road segment, and
//! `crate::pipeline` has already split a multi-level segment into constant-level
//! pieces, so each feature here is uniformly a bridge or a tunnel. They are
//! different solids — a bridge is open, a tunnel has a roof — that meet at the
//! road surface:
//!
//! - **Bridge** ([`sweep_deck`]): a thin open slab whose top *is* the road surface
//!   (the single model), a [`DECK_THICKNESS_M`] underside below. No roof.
//! - **Tunnel** ([`sweep_bore`]): a roofed bore — roof a [`TUNNEL_HEIGHT_M`] ramp
//!   above the road, floor on the *same plane as a deck's underside*
//!   (`road − [`DECK_THICKNESS_M`]`) so the bore's bottom aligns with an adjoining
//!   deck: a tunnel is just that deck plus a roof, not a box hanging below it. Each
//!   real portal mouth is **capped** with a portal face, so where a tunnel meets a
//!   bridge the bore ends solidly and the deck meets that face at the road surface
//!   — the two *touch* (at different heights: the tunnel taller for its roof)
//!   instead of the bore's open mouth reading as a gap above the thin deck.
//!
//! Heights are a function of the *global* road profile carried to every fragment,
//! so independent tile fragments line up at the seams. The box is clipped to the
//! tile *proper*, not the format's half-tile buffer: an opaque solid built in the
//! buffer is replicated across neighbours and the independently-clipped copies
//! overlap into a staircase; clipped to the proper tile each draws once and abuts
//! its neighbour at the shared edge. The `level` sign is kept on the mesh only so
//! the client colours bridges blue-grey and tunnels rust.
//!
//! Coordinates are tile-local quantized uint16 (x/y) and int32 millimetres (z),
//! matching `MeshGeometry`, with octahedral ECEF normals in the shared ENU
//! [`Frame`].

use geo_types::{Coord, Geometry, LineString};

use crate::building_mesh::{Frame, M_PER_DEG_LAT};
use crate::clip;
use crate::dem::Dem;
use crate::project::{self, Bounds};
use crate::structures::{self, RoadProfile};
use crate::terrain::TerrainMesh;
use crate::tile_build::{prop_str, EncoderFeature};

/// Thickness of a bridge deck slab in metres — deck surface to its underside.
const DECK_THICKNESS_M: f64 = 1.5;

/// Vertical clearance of a tunnel bore in metres — road floor to its flat roof.
const TUNNEL_HEIGHT_M: f64 = 6.0;

/// How far a tunnel bore is extended past each real portal mouth, so the mouth
/// pokes from the hillside and overlaps the adjoining deck or approach road
/// instead of ending flush at the run boundary. Only real portals (interior run
/// ends) get the buffer; a tile-boundary cut continues in the neighbour.
const PORTAL_BUFFER_M: f64 = 12.0;

/// Target spacing in metres for densifying the centerline, so the box follows
/// the road's curve and grade smoothly.
const SEGMENT_M: f64 = 8.0;

/// Cap on densified centerline vertices per line — a runaway guard.
const MAX_VERTS: usize = 4096;

/// Builds the structure box for a transportation feature and stores it in
/// `f.mesh`, stripping the carried segment properties. `level` selects the kind: a
/// bridge (`level > 0`) is a thin open deck on the road surface; a tunnel
/// (`level < 0`) is a roofed bore, capped at its real portal mouths so the deck of
/// an adjoining bridge meets the portal face. A no-op (leaving the line to drape)
/// when the feature carries no usable road profile or is not a line.
pub fn stamp(f: &mut EncoderFeature, dem: &mut Dem, z: u8, bounds: &Bounds, level: i64) {
    let profile = RoadProfile::from_feature(f, dem, z, bounds);
    structures::discard_run(f);
    let Some(profile) = profile else {
        return;
    };
    let frame = Frame::at_center(bounds);
    let half_w = half_width_m(prop_str(f, "class").as_deref());

    let mut acc = Accum::default();
    for line in lines(&f.geometry) {
        for piece in proper_pieces(&line.0, bounds) {
            if level < 0 {
                sweep_bore(&mut acc, &frame, bounds, &profile, &piece, half_w);
            } else {
                sweep_deck(&mut acc, &frame, bounds, &profile, &piece, half_w);
            }
        }
    }
    if let Some(mesh) = acc.into_mesh() {
        f.mesh = Some(mesh);
    }
}

/// Whether `c` sits on the tile-proper boundary — i.e. a clip cut where the box
/// continues into the neighbouring tile, *not* a real run end. A portal mouth is
/// an interior run end, so a bore is capped only where `!on_tile_edge`.
fn on_tile_edge(c: Coord, bounds: &Bounds) -> bool {
    let ex = bounds.width() * 1e-4;
    let ey = bounds.height() * 1e-4;
    (c.x - bounds.west).abs() < ex
        || (c.x - bounds.east).abs() < ex
        || (c.y - bounds.south).abs() < ey
        || (c.y - bounds.north).abs() < ey
}

/// The lines of a (multi)line geometry (empty for any other topology).
fn lines(g: &Geometry) -> Vec<&LineString> {
    match g {
        Geometry::LineString(ls) => vec![ls],
        Geometry::MultiLineString(mls) => mls.0.iter().collect(),
        _ => Vec::new(),
    }
}

/// Clips a span to the tile *proper*, returning the (in-order) pieces. An opaque
/// solid must not extend into the format's buffer or neighbouring tiles would each
/// rebuild and overlap it; clipped to the proper tile each draws once and abuts
/// its neighbour at the shared edge.
fn proper_pieces(span: &[Coord], bounds: &Bounds) -> Vec<Vec<Coord>> {
    let g = Geometry::LineString(LineString(span.to_vec()));
    match clip::clip_geometry(&g, bounds) {
        Some(Geometry::LineString(ls)) => vec![ls.0],
        Some(Geometry::MultiLineString(mls)) => mls.0.into_iter().map(|l| l.0).collect(),
        _ => Vec::new(),
    }
}

/// Half-width of the box in metres by road class — bigger roads, bigger structures.
fn half_width_m(class: Option<&str>) -> f64 {
    match class {
        Some("motorway") | Some("trunk") => 7.5,
        Some("primary") | Some("secondary") => 6.0,
        _ => 4.0,
    }
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

/// One swept cross-section: the (smoothed) centerline position, the box top and
/// bottom heights, and the unit left-perpendicular (ENU metres) it spans.
struct Section {
    lon: f64,
    lat: f64,
    top_mm: i32,
    bot_mm: i32,
    /// Left-perpendicular unit direction in ENU metres (east, north).
    left_e: f64,
    left_n: f64,
}

/// Sweeps a bridge deck along one (proper-clipped) span: a thin open slab on the
/// road surface — top *is* the road, a [`DECK_THICKNESS_M`] underside below. No
/// roof: a bridge is open, unlike a tunnel. Where the road sinks to/under grade
/// the slab passes under the terrain and is occluded, so the deck meets the ground
/// without a step.
fn sweep_deck(acc: &mut Accum, frame: &Frame, bounds: &Bounds, profile: &RoadProfile, span: &[Coord], half_w: f64) {
    let pts = densify(span, frame);
    if pts.len() < 2 {
        return;
    }
    let sections: Vec<Section> = profile
        .deck_nodes(&pts)
        .into_iter()
        .map(|d| Section {
            lon: d.lon,
            lat: d.lat,
            top_mm: project::quantize_z(d.height_m),
            bot_mm: project::quantize_z(d.height_m - DECK_THICKNESS_M),
            left_e: d.left_e,
            left_n: d.left_n,
        })
        .collect();
    sweep_prism(acc, frame, bounds, &sections, half_w);
}

/// Sweeps a tunnel bore along one (proper-clipped) span: a roof a clean
/// [`TUNNEL_HEIGHT_M`] ramp above the road, and a floor on the same plane as a
/// bridge deck's underside (`road − [`DECK_THICKNESS_M`]`) so the bore's bottom
/// aligns with an adjoining deck — the tunnel is just that deck plus a roof, not a
/// box hanging below it. Each *real* portal mouth (a run end inside the tile, not
/// a tile-boundary clip) is capped with a portal face, so the bore ends solidly
/// and the deck meets that face at the road surface — the two touch.
fn sweep_bore(acc: &mut Accum, frame: &Frame, bounds: &Bounds, profile: &RoadProfile, span: &[Coord], half_w: f64) {
    let pts = densify(span, frame);
    if pts.len() < 2 {
        return;
    }
    let mut sections: Vec<Section> = profile
        .deck_nodes(&pts)
        .into_iter()
        .map(|d| Section {
            lon: d.lon,
            lat: d.lat,
            top_mm: project::quantize_z(d.height_m + TUNNEL_HEIGHT_M),
            bot_mm: project::quantize_z(d.height_m - DECK_THICKNESS_M),
            left_e: d.left_e,
            left_n: d.left_n,
        })
        .collect();
    if sections.len() < 2 {
        return;
    }

    // A real portal mouth (an interior run end) is extended outward by
    // [`PORTAL_BUFFER_M`] and capped; a tile-boundary cut stays put and open (the
    // bore continues in the neighbour).
    let start_portal = !on_tile_edge(pts[0], bounds);
    let end_portal = !on_tile_edge(pts[pts.len() - 1], bounds);
    let n0 = sections.len();
    let start_stub = start_portal.then(|| portal_stub(&sections[0], &sections[1], frame));
    let end_stub = end_portal.then(|| portal_stub(&sections[n0 - 1], &sections[n0 - 2], frame));
    if let Some(s) = end_stub {
        sections.push(s);
    }
    if let Some(s) = start_stub {
        sections.insert(0, s);
    }

    sweep_prism(acc, frame, bounds, &sections, half_w);

    let n = sections.len();
    if start_portal {
        cap_end(acc, frame, bounds, &sections[0], &sections[1], half_w);
    }
    if end_portal {
        cap_end(acc, frame, bounds, &sections[n - 1], &sections[n - 2], half_w);
    }
}

/// A bore section [`PORTAL_BUFFER_M`] beyond `end`, continuing the roof's grade
/// straight out along the `inner → end` tangent (the floor stays on the deck
/// plane, so the protruding mouth keeps the bridge-aligned bottom).
fn portal_stub(end: &Section, inner: &Section, frame: &Frame) -> Section {
    let de = (end.lon - inner.lon) * frame.m_per_deg_lon;
    let dn = (end.lat - inner.lat) * M_PER_DEG_LAT;
    let len = (de * de + dn * dn).sqrt().max(1e-9);
    let lon = end.lon + (de / len) * PORTAL_BUFFER_M / frame.m_per_deg_lon;
    let lat = end.lat + (dn / len) * PORTAL_BUFFER_M / M_PER_DEG_LAT;
    // Extrapolate both faces straight (no kink at the mouth): the roof and the
    // floor each change by their per-section step (the road grade) over the buffer.
    let step = |end_mm: i32, inner_mm: i32| {
        end_mm + ((end_mm - inner_mm) as f64 * PORTAL_BUFFER_M / len).round() as i32
    };
    Section {
        lon,
        lat,
        top_mm: step(end.top_mm, inner.top_mm),
        bot_mm: step(end.bot_mm, inner.bot_mm),
        left_e: end.left_e,
        left_n: end.left_n,
    }
}

/// The (qx, qy) corner of a cross-section offset `side` half-widths to its left.
fn corner(s: &Section, side: f64, frame: &Frame, bounds: &Bounds, half_w: f64) -> (u16, u16) {
    let dlon = s.left_e * half_w * side / frame.m_per_deg_lon;
    let dlat = s.left_n * half_w * side / M_PER_DEG_LAT;
    (project::quantize_x(s.lon + dlon, bounds), project::quantize_y(s.lat + dlat, bounds))
}

/// Closes a bore's end cross-section `end` with a portal face, its outward normal
/// pointing away from the interior section `inner`.
fn cap_end(acc: &mut Accum, frame: &Frame, bounds: &Bounds, end: &Section, inner: &Section, half_w: f64) {
    let (el, er) = (corner(end, 1.0, frame, bounds, half_w), corner(end, -1.0, frame, bounds, half_w));
    let de = (end.lon - inner.lon) * frame.m_per_deg_lon;
    let dn = (end.lat - inner.lat) * M_PER_DEG_LAT;
    let len = (de * de + dn * dn).sqrt().max(1e-9);
    let nrm = frame.encode_enu(de / len, dn / len, 0.0);
    quad(acc, (el, end.bot_mm), (er, end.bot_mm), (er, end.top_mm), (el, end.top_mm), nrm);
}

/// Sweeps a box prism through the cross-sections: a top face (up), a bottom face
/// (down), and the two side walls. Cull mode is off on the client, so winding
/// only feeds lighting via the per-face normal.
fn sweep_prism(acc: &mut Accum, frame: &Frame, bounds: &Bounds, sections: &[Section], half_w: f64) {
    let n = sections.len();
    if n < 2 {
        return;
    }
    let n_up = frame.encode_enu(0.0, 0.0, 1.0);
    let n_down = frame.encode_enu(0.0, 0.0, -1.0);

    for i in 0..n - 1 {
        let (a, b) = (&sections[i], &sections[i + 1]);
        let (al, ar) = (corner(a, 1.0, frame, bounds, half_w), corner(a, -1.0, frame, bounds, half_w));
        let (bl, br) = (corner(b, 1.0, frame, bounds, half_w), corner(b, -1.0, frame, bounds, half_w));
        let (top_a, top_b) = (a.top_mm, b.top_mm);
        let (bot_a, bot_b) = (a.bot_mm, b.bot_mm);
        let n_left = frame.encode_enu(a.left_e, a.left_n, 0.0);
        let n_right = frame.encode_enu(-a.left_e, -a.left_n, 0.0);

        quad(acc, (al, bot_a), (ar, bot_a), (br, bot_b), (bl, bot_b), n_down);
        quad(acc, (al, top_a), (bl, top_b), (br, top_b), (ar, top_a), n_up);
        quad(acc, (al, bot_a), (bl, bot_b), (bl, top_b), (al, top_a), n_left);
        quad(acc, (ar, bot_a), (ar, top_a), (br, top_b), (br, bot_b), n_right);
    }
}

/// Emits a quad (two triangles) from four corners, each `((qx, qy), z_mm)`, all
/// carrying the same face normal.
fn quad(
    acc: &mut Accum,
    p0: ((u16, u16), i32),
    p1: ((u16, u16), i32),
    p2: ((u16, u16), i32),
    p3: ((u16, u16), i32),
    nrm: (i8, i8),
) {
    let v0 = acc.push((p0.0).0, (p0.0).1, p0.1, nrm);
    let v1 = acc.push((p1.0).0, (p1.0).1, p1.1, nrm);
    let v2 = acc.push((p2.0).0, (p2.0).1, p2.1, nrm);
    let v3 = acc.push((p3.0).0, (p3.0).1, p3.1, nrm);
    acc.tri(v0, v1, v2);
    acc.tri(v0, v2, v3);
}

/// Densifies a centerline to ~[`SEGMENT_M`] spacing in (lon, lat), so the swept
/// box follows the road's curve and grade.
fn densify(pts: &[Coord], frame: &Frame) -> Vec<Coord> {
    let mut out = Vec::new();
    if pts.is_empty() {
        return out;
    }
    out.push(pts[0]);
    for w in pts.windows(2) {
        let (p0, p1) = (w[0], w[1]);
        let de = (p1.x - p0.x) * frame.m_per_deg_lon;
        let dn = (p1.y - p0.y) * M_PER_DEG_LAT;
        let len = (de * de + dn * dn).sqrt();
        let steps = ((len / SEGMENT_M).ceil() as usize).clamp(1, MAX_VERTS);
        for i in 1..=steps {
            let t = i as f64 / steps as f64;
            out.push(Coord { x: p0.x + (p1.x - p0.x) * t, y: p0.y + (p1.y - p0.y) * t });
        }
        if out.len() >= MAX_VERTS {
            break;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A straight west→east line across the tile centre.
    fn centre_line(b: &Bounds) -> LineString {
        let cy = (b.south + b.north) * 0.5;
        LineString(vec![
            Coord { x: b.west + 0.3 * b.width(), y: cy },
            Coord { x: b.west + 0.7 * b.width(), y: cy },
        ])
    }

    /// Sweeps a bridge deck over a whole line.
    fn deck(line: &LineString, profile: &RoadProfile, b: &Bounds, class: Option<&str>) -> Option<TerrainMesh> {
        let frame = Frame::at_center(b);
        let half_w = half_width_m(class);
        let mut acc = Accum::default();
        for piece in proper_pieces(&line.0, b) {
            sweep_deck(&mut acc, &frame, b, profile, &piece, half_w);
        }
        acc.into_mesh()
    }

    /// Sweeps a tunnel bore over a whole line with an injected terrain sampler.
    fn bore(line: &LineString, profile: &RoadProfile, b: &Bounds, class: Option<&str>) -> Option<TerrainMesh> {
        let frame = Frame::at_center(b);
        let half_w = half_width_m(class);
        let mut acc = Accum::default();
        for piece in proper_pieces(&line.0, b) {
            sweep_bore(&mut acc, &frame, b, profile, &piece, half_w);
        }
        acc.into_mesh()
    }

    #[test]
    fn bridge_deck_is_a_thin_open_slab_on_the_road_surface() {
        // Top is the road surface, a slab hangs below, and there is no roof.
        let b = Bounds::of_tile(14, 8500, 5800);
        let line = centre_line(&b);
        let profile = RoadProfile::flat(&line.0, 100.0);
        let mesh = deck(&line, &profile, &b, Some("motorway")).expect("deck");

        let surface = project::quantize_z(100.0);
        assert!(mesh.z.iter().any(|&z| z == surface), "expected a deck-top vertex on the road surface");
        assert!(mesh.z.iter().all(|&z| z <= surface), "a bridge has no roof above the road");
        let underside = project::quantize_z(100.0 - DECK_THICKNESS_M);
        assert!(mesh.z.iter().any(|&z| z == underside), "expected a deck underside vertex");
        assert_eq!(mesh.indices.len() % 6, 0);
        assert!(mesh.indices.iter().all(|&i| (i as usize) < mesh.x.len()));
        assert_eq!(mesh.normals.len(), mesh.x.len() * 2);
        // The east→west road runs along x, so the cross-section spreads in y.
        let (lo, hi) = mesh.y.iter().fold((u16::MAX, u16::MIN), |(lo, hi), &y| (lo.min(y), hi.max(y)));
        assert!(hi > lo, "deck has no width across the road");
    }

    #[test]
    fn tunnel_bore_has_a_roof_above_the_road_and_a_deck_aligned_floor() {
        // Road at 100 m: a roof a bore height above the road, and a floor on the
        // deck-underside plane (road − DECK_THICKNESS) so the bottom aligns with a
        // bridge deck rather than hanging below it.
        let b = Bounds::of_tile(14, 8500, 5800);
        let line = centre_line(&b);
        let profile = RoadProfile::flat(&line.0, 100.0);
        let mesh = bore(&line, &profile, &b, Some("motorway")).expect("bore");

        let roof = project::quantize_z(100.0 + TUNNEL_HEIGHT_M);
        let floor = project::quantize_z(100.0 - DECK_THICKNESS_M);
        assert!(mesh.z.iter().any(|&z| z == roof), "expected a roof vertex a bore height above the road");
        assert!(mesh.z.iter().any(|&z| z == floor), "expected a floor vertex on the deck-underside plane");
        assert!(mesh.z.iter().all(|&z| z <= roof), "nothing above the roof");
        assert!(mesh.z.iter().all(|&z| z >= floor), "nothing below the floor");
    }

    #[test]
    fn bore_caps_its_interior_portals() {
        // Both ends are interior run ends (real portals), so both are capped. The
        // prism is 4 quads/segment (24 indices); two caps add 12, so the index
        // total is 12 beyond a whole number of segments.
        let b = Bounds::of_tile(14, 8500, 5800);
        let line = centre_line(&b);
        let profile = RoadProfile::flat(&line.0, 100.0);
        let mesh = bore(&line, &profile, &b, Some("motorway")).expect("bore");
        assert_eq!(mesh.indices.len() % 24, 12, "expected two portal caps (+12 indices)");
    }

    #[test]
    fn bore_leaves_tile_edge_cuts_open() {
        // A bore running off the west edge: that end is a clip (open, the bore
        // continues in the neighbour); only the interior east end is capped — one
        // cap, so the index total is 6 beyond a whole number of segments.
        let b = Bounds::of_tile(14, 8500, 5800);
        let cy = (b.south + b.north) * 0.5;
        let line = LineString(vec![
            Coord { x: b.west - 0.2 * b.width(), y: cy },
            Coord { x: b.west + 0.6 * b.width(), y: cy },
        ]);
        let profile = RoadProfile::flat(&centre_line(&b).0, 100.0);
        let mesh = bore(&line, &profile, &b, Some("motorway")).expect("bore");
        assert_eq!(mesh.indices.len() % 24, 6, "a tile-edge end stays open, only one cap");
    }

    #[test]
    fn bore_extends_a_buffer_past_interior_portals() {
        // Both ends are interior portals, so the bore pokes ~PORTAL_BUFFER_M past
        // each end of the centerline (the road runs east–west, so this shows in lon).
        let b = Bounds::of_tile(14, 8500, 5800);
        let line = centre_line(&b);
        let profile = RoadProfile::flat(&line.0, 100.0);
        let mesh = bore(&line, &profile, &b, Some("motorway")).expect("bore");

        let lons: Vec<f64> = mesh.x.iter().map(|&q| project::dequantize_x(q, &b)).collect();
        let mesh_w = lons.iter().cloned().fold(f64::INFINITY, f64::min);
        let mesh_e = lons.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let line_w = line.0[0].x.min(line.0[1].x);
        let line_e = line.0[0].x.max(line.0[1].x);
        let half_buf = 0.5 * PORTAL_BUFFER_M / Frame::at_center(&b).m_per_deg_lon;
        assert!(line_w - mesh_w > half_buf, "bore must poke past the west portal");
        assert!(mesh_e - line_e > half_buf, "bore must poke past the east portal");
    }

    #[test]
    fn bore_floor_is_independent_of_the_terrain() {
        // The floor rides the road profile (road − DECK_THICKNESS), not the terrain,
        // so a tunnel and a bridge share a bottom whatever the ground does beneath.
        let b = Bounds::of_tile(14, 8500, 5800);
        let line = centre_line(&b);
        let profile = RoadProfile::flat(&line.0, 100.0);
        let deck = deck(&line, &profile, &b, Some("motorway")).expect("deck");
        let bore = bore(&line, &profile, &b, Some("motorway")).expect("bore");
        let deck_floor = *deck.z.iter().min().expect("non-empty");
        let bore_floor = *bore.z.iter().min().expect("non-empty");
        assert_eq!(deck_floor, project::quantize_z(100.0 - DECK_THICKNESS_M));
        assert_eq!(bore_floor, deck_floor, "bore bottom must align with the deck bottom");
    }

    #[test]
    fn half_width_scales_with_class() {
        assert!(half_width_m(Some("motorway")) > half_width_m(Some("residential")));
        assert_eq!(half_width_m(None), half_width_m(Some("residential")));
    }
}
