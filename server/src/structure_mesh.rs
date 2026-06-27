//! Server-side 3D geometry for road structures — bridges and tunnels — as a box
//! prism swept along the road's reconstructed surface profile (`crate::structures`).
//!
//! Overture marks a bridge or tunnel as a non-ground span of a road segment (see
//! `crate::levels`). Both are a box following the road's gentle grade through the
//! terrain it crosses. The road runs at the *road surface* either way, but on a
//! bridge that is the deck *top* (you drive on top of the deck) and in a tunnel
//! it is the *floor* (you drive along the bore floor), so the solid sits on the
//! opposite side of the surface:
//!
//! - **Tunnel** (negative level): the bore rises *above* the road (floor =
//!   surface, ceiling = surface + bore height). Riding the gentle grade through
//!   the hill it pierces, the body stays buried; the terrain (drawn first, owning
//!   the depth buffer) occludes it, and the open ends read as the portals.
//! - **Bridge** (positive level): a thin slab hangs *below* the road (deck top =
//!   surface, underside = surface − slab thickness). Where the terrain dips below
//!   it (a ravine) the deck stands proud — the visible viaduct; where it rises
//!   above (a flank) the slab passes under ground and is occluded.
//!
//! Because the road surface is shared, a bridge's deck top meets a tunnel's floor
//! exactly where they join — the bridge aligns to the bottom of the tunnel.
//!
//! Heights come from the *global* segment carried on every tile fragment, so
//! independent fragments of one structure line up at the seams. The box is
//! open-ended (no caps): a clipped fragment's ends meet its neighbour's.
//! Coordinates are tile-local quantized uint16 (x/y) and int32 millimetres (z),
//! matching `MeshGeometry`, with octahedral ECEF normals in the shared ENU
//! [`Frame`].

use geo_types::{Coord, Geometry, LineString};

use crate::building_mesh::{Frame, M_PER_DEG_LAT};
use crate::clip;
use crate::dem::Dem;
use crate::project::{self, Bounds};
use crate::structures::{self, RoadProfile};
use crate::terrain::{self, TerrainMesh};
use crate::tile_build::{prop_str, EncoderFeature};

/// Vertical clearance of a tunnel bore in metres — floor to flat ceiling.
const TUNNEL_HEIGHT_M: f64 = 6.0;

/// Thickness of a bridge deck slab in metres — deck surface to its underside.
const DECK_THICKNESS_M: f64 = 1.5;

/// Target spacing in metres for densifying the centerline, so the box follows
/// the road's curve and grade smoothly.
const SEGMENT_M: f64 = 8.0;

/// Cap on densified centerline vertices per line — a runaway guard.
const MAX_VERTS: usize = 4096;

/// How far past the point its bore mouth clears the terrain a tunnel is pushed,
/// so the opening pokes from the hillside and reads as a portal instead of
/// ending flush with the slope.
const PORTAL_MARGIN_M: f64 = 16.0;

/// Step of the outward search that finds where a tunnel's bore ceiling rises
/// above the terrain, and the distance it reaches before giving up. The reach
/// caps the push so an end cut mid-bore at a tile edge (never exposed) can't run
/// away; a real portal clears the slope well within it.
const PORTAL_STEP_M: f64 = 2.0;
const PORTAL_REACH_M: f64 = 80.0;

/// Whether the box is lifted onto a bridge deck or sunk as a tunnel bore.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Kind {
    Bridge,
    Tunnel,
}

/// Builds the structure box for a transportation feature and stores it in
/// `f.mesh`, stripping the carried segment properties. `level` selects the kind
/// (positive = bridge, negative = tunnel). A no-op (the carry is just discarded,
/// leaving the line to drape) when the feature carries no usable road profile or
/// its geometry is not a line.
pub fn stamp(f: &mut EncoderFeature, dem: &mut Dem, z: u8, bounds: &Bounds, level: i64) {
    let kind = if level > 0 { Kind::Bridge } else { Kind::Tunnel };
    if let Some(profile) = RoadProfile::from_feature(f, dem, z, bounds) {
        let class = prop_str(f, "class");
        // The rendered terrain surface a tunnel mouth must clear — the same
        // surface the road profile is anchored to, so the portal opens exactly
        // where the bore would surface.
        let mut terrain =
            |c: Coord| terrain::surface_height(bounds, c.x, c.y, &mut |lon, lat| dem.elevation(lon, lat, z));
        // Clip to the tile *proper*, not the format's half-tile buffer. A draped
        // line wants the buffer so its stroke covers the seam, but a structure box
        // is an opaque 3D solid: built in the buffer it is replicated across every
        // neighbouring tile, and the independently-fit copies overlap into a
        // staircase. Clipped to the proper tile each box draws once and abuts its
        // neighbour at the shared edge (the cross-section there is identical — same
        // global smoothed line, fit, and width — so the seam is seamless).
        if let Some(geom) = clip::clip_geometry(&f.geometry, bounds) {
            if let Some(mesh) = build(&geom, &profile, bounds, class.as_deref(), kind, &mut terrain) {
                f.mesh = Some(mesh);
            }
        }
    }
    structures::discard_run(f);
}

/// Builds a box-prism mesh sweeping the (clipped) structure centerline, riding
/// the road profile. `None` for non-line or degenerate geometry.
fn build(
    geom: &Geometry,
    profile: &RoadProfile,
    bounds: &Bounds,
    class: Option<&str>,
    kind: Kind,
    terrain: &mut dyn FnMut(Coord) -> f64,
) -> Option<TerrainMesh> {
    let frame = Frame::at_center(bounds);
    let half_w = half_width_m(class);
    // Both kinds put their road face *on* the road surface (the single model); the
    // solid then extends to the far face. A bridge slab hangs *below* the deck
    // (negative rise); a tunnel bore rises *above* the floor (positive rise). The
    // shared road surface is what makes a bridge's deck top meet a tunnel's floor.
    let rise_mm = match kind {
        Kind::Bridge => -((DECK_THICKNESS_M * 1000.0) as i32),
        Kind::Tunnel => (TUNNEL_HEIGHT_M * 1000.0) as i32,
    };
    // A tunnel's ends are opened past the hillside (terrain-aware) so the mouths
    // read as portals; a bridge deck ends flush at its abutments.
    let open = kind == Kind::Tunnel;
    let mut acc = Accum::default();
    match geom {
        Geometry::LineString(ls) => {
            add_line(&mut acc, &frame, bounds, profile, ls, half_w, rise_mm, open, terrain)
        }
        Geometry::MultiLineString(mls) => {
            for ls in &mls.0 {
                add_line(&mut acc, &frame, bounds, profile, ls, half_w, rise_mm, open, terrain);
            }
        }
        _ => return None,
    }
    acc.into_mesh()
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

/// One densified centerline node with its box-top height (deck surface or road
/// surface) and the unit left-perpendicular direction (in ENU metres) the
/// cross-section spans.
struct Node {
    lon: f64,
    lat: f64,
    top_mm: i32,
    /// Left-perpendicular unit direction in ENU metres (east, north).
    left_e: f64,
    left_n: f64,
}

/// Sweeps a box prism along one clipped centerline. When `open`, the ends are
/// pushed past the hillside so a tunnel's mouths read as portals (see
/// [`open_portals`]).
#[allow(clippy::too_many_arguments)]
fn add_line(
    acc: &mut Accum,
    frame: &Frame,
    bounds: &Bounds,
    profile: &RoadProfile,
    line: &LineString,
    half_w: f64,
    rise_mm: i32,
    open: bool,
    terrain: &mut dyn FnMut(Coord) -> f64,
) {
    let base = if open { open_portals(&line.0, bounds, profile, terrain) } else { line.0.clone() };
    let pts = densify(&base, frame);
    if pts.len() < 2 {
        return;
    }
    // Each cross-section placed on the smoothed road line with a straight-ramp
    // deck height and a smoothed section direction, so the box is a regular prism
    // that follows the road instead of tracing every wiggle and dive (see
    // [`RoadProfile::deck_nodes`]). The road face sits on this surface (the single
    // model); the box extends `rise_mm` to the far face (a bridge slab below, a
    // bore above).
    let nodes: Vec<Node> = profile
        .deck_nodes(&pts)
        .into_iter()
        .map(|d| Node {
            lon: d.lon,
            lat: d.lat,
            top_mm: project::quantize_z(d.height_m),
            left_e: d.left_e,
            left_n: d.left_n,
        })
        .collect();
    let n = nodes.len();
    if n < 2 {
        return;
    }

    // Per-face octahedral normals (constant across the prism in ENU).
    let n_up = frame.encode_enu(0.0, 0.0, 1.0);
    let n_down = frame.encode_enu(0.0, 0.0, -1.0);

    // Corner (lon, lat) for a node offset by `s` half-widths to the left.
    let corner = |nd: &Node, s: f64| -> (u16, u16) {
        let dlon = nd.left_e * half_w * s / frame.m_per_deg_lon;
        let dlat = nd.left_n * half_w * s / M_PER_DEG_LAT;
        (
            project::quantize_x(nd.lon + dlon, bounds),
            project::quantize_y(nd.lat + dlat, bounds),
        )
    };

    for i in 0..n - 1 {
        let (a, b) = (&nodes[i], &nodes[i + 1]);
        let (al, ar) = (corner(a, 1.0), corner(a, -1.0));
        let (bl, br) = (corner(b, 1.0), corner(b, -1.0));
        // The road face is on the road surface (up-facing); the far face is
        // `rise_mm` away (a bridge deck's underside below, a tunnel bore's ceiling
        // above), down-facing. A bridge deck stands above the terrain (a viaduct);
        // a tunnel bore stays buried in the hill and is occluded.
        let (road_a, road_b) = (a.top_mm, b.top_mm);
        let (far_a, far_b) = (a.top_mm + rise_mm, b.top_mm + rise_mm);
        // Side-wall outward normals (left/right), horizontal in ENU.
        let n_left = frame.encode_enu(a.left_e, a.left_n, 0.0);
        let n_right = frame.encode_enu(-a.left_e, -a.left_n, 0.0);

        // Far face (down-facing), road face (up-facing), and the two side walls.
        // Each is a quad of four fresh flat-shaded vertices; cull mode is off on
        // the client, so winding only feeds lighting via the per-face normal.
        quad(acc, (al, far_a), (ar, far_a), (br, far_b), (bl, far_b), n_down);
        quad(acc, (al, road_a), (bl, road_b), (br, road_b), (ar, road_a), n_up);
        quad(acc, (al, far_a), (bl, far_b), (bl, road_b), (al, road_a), n_left);
        quad(acc, (ar, far_a), (ar, road_a), (br, road_b), (br, far_b), n_right);
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

/// Opens a tunnel's portals: moves each end outward along its tangent until the
/// bore ceiling rises above the rendered terrain — the mouth is clear — then a
/// fixed margin further so it pokes from the hillside. A fixed push (the old
/// scheme) cannot do this: the two ends sit on different slopes and need
/// different reach, so one constant either buries the steep mouth or overruns the
/// gentle one. The heights past the segment end hold the portal's grade
/// ([`RoadProfile::height_at`] clamps to the nearest on-segment point), so the
/// protruding stub keeps climbing the road's profile, not the terrain's.
///
/// An end lying on the tile boundary is a clip cut, not a real portal — extending
/// it would grow a phantom mouth at every tile seam the bore crosses — so it is
/// left in place to abut the neighbour tile's piece.
fn open_portals(
    pts: &[Coord],
    bounds: &Bounds,
    profile: &RoadProfile,
    terrain: &mut dyn FnMut(Coord) -> f64,
) -> Vec<Coord> {
    if pts.len() < 2 {
        return pts.to_vec();
    }
    let mean_lat = pts.iter().map(|c| c.y).sum::<f64>() / pts.len() as f64;
    let cos_lat = mean_lat.to_radians().cos();
    let mut out = pts.to_vec();
    let n = out.len();
    if !on_tile_edge(pts[0], bounds) {
        out[0] = portal_point(pts[1], pts[0], cos_lat, profile, terrain);
    }
    if !on_tile_edge(pts[n - 1], bounds) {
        out[n - 1] = portal_point(pts[n - 2], pts[n - 1], cos_lat, profile, terrain);
    }
    out
}

/// Whether `c` sits on the proper tile boundary (a clip cut), within a small
/// fraction of the tile size.
fn on_tile_edge(c: Coord, bounds: &Bounds) -> bool {
    let ex = bounds.width() * 1e-4;
    let ey = bounds.height() * 1e-4;
    (c.x - bounds.west).abs() < ex
        || (c.x - bounds.east).abs() < ex
        || (c.y - bounds.south).abs() < ey
        || (c.y - bounds.north).abs() < ey
}

/// The opened position of one tunnel end: stepping along the `inner` → `end`
/// direction, the first point whose bore ceiling (road surface + bore height)
/// clears the terrain, plus [`PORTAL_MARGIN_M`]; capped at [`PORTAL_REACH_M`].
/// An end already clear (terrain below the ceiling) just gets the margin — the
/// same small poke the fixed scheme gave every portal.
fn portal_point(
    inner: Coord,
    end: Coord,
    cos_lat: f64,
    profile: &RoadProfile,
    terrain: &mut dyn FnMut(Coord) -> f64,
) -> Coord {
    let de = (end.x - inner.x) * cos_lat * M_PER_DEG_LAT;
    let dn = (end.y - inner.y) * M_PER_DEG_LAT;
    let len = (de * de + dn * dn).sqrt().max(f64::MIN_POSITIVE);
    // Outward direction in lon/lat per metre stepped.
    let (ux, uy) = ((end.x - inner.x) / len, (end.y - inner.y) / len);
    let steps = (PORTAL_REACH_M / PORTAL_STEP_M) as usize;
    let exposed = (0..=steps).find_map(|k| {
        let d = k as f64 * PORTAL_STEP_M;
        let p = Coord { x: end.x + ux * d, y: end.y + uy * d };
        let ceiling = profile.height_at(p.x, p.y) + TUNNEL_HEIGHT_M;
        (terrain(p) <= ceiling).then_some(d)
    });
    let reach = (exposed.unwrap_or(PORTAL_REACH_M) + PORTAL_MARGIN_M).min(PORTAL_REACH_M);
    Coord { x: end.x + ux * reach, y: end.y + uy * reach }
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

    #[test]
    fn tunnel_bore_rises_above_the_road_surface() {
        let b = Bounds::of_tile(14, 8500, 5800);
        let line = centre_line(&b);
        let profile = RoadProfile::flat(&line.0, 100.0);
        let mesh =
            build(&Geometry::LineString(line), &profile, &b, Some("motorway"), Kind::Tunnel, &mut |_: Coord| f64::NEG_INFINITY).unwrap();

        let floor = project::quantize_z(100.0); // road surface = bore floor
        let ceiling = floor + (TUNNEL_HEIGHT_M * 1000.0) as i32;
        assert!(mesh.z.iter().any(|&z| z == floor), "expected floor vertices at the road level");
        assert!(mesh.z.iter().any(|&z| z == ceiling), "expected ceiling vertices a bore height above");
        // The road runs on the floor; nothing dips below it (so it can't poke out
        // under the terrain it passes beneath).
        assert!(mesh.z.iter().all(|&z| z >= floor), "no vertex below the road floor");
        // Every face is a quad → 6 indices; indices reference real vertices.
        assert_eq!(mesh.indices.len() % 6, 0);
        assert!(mesh.indices.iter().all(|&i| (i as usize) < mesh.x.len()));
        assert_eq!(mesh.normals.len(), mesh.x.len() * 2);
    }

    #[test]
    fn bridge_deck_sits_on_the_road_surface() {
        let b = Bounds::of_tile(14, 8500, 5800);
        let line = centre_line(&b);
        let profile = RoadProfile::flat(&line.0, 100.0);
        let mesh =
            build(&Geometry::LineString(line), &profile, &b, Some("motorway"), Kind::Bridge, &mut |_: Coord| f64::NEG_INFINITY).unwrap();

        // The deck top *is* the road surface (the single model); the slab hangs
        // below, so the deck never floats above the road.
        let surface = project::quantize_z(100.0);
        assert!(mesh.z.iter().any(|&z| z == surface), "expected a deck-top vertex on the road surface");
        assert!(mesh.z.iter().all(|&z| z <= surface), "no vertex above the deck top");
        let underside = surface - (DECK_THICKNESS_M * 1000.0) as i32;
        assert!(mesh.z.iter().any(|&z| z == underside), "expected a deck underside vertex");
    }

    #[test]
    fn bridge_deck_top_meets_tunnel_floor() {
        // The road is continuous at the road surface: on a bridge that is the deck
        // *top* (you drive on the deck), in a tunnel it is the *floor* (you drive
        // through the bore). So the bridge's highest face and the tunnel's lowest
        // face are the same height — the bridge aligns to the bottom of the tunnel,
        // not its top.
        let b = Bounds::of_tile(14, 8500, 5800);
        let line = centre_line(&b);
        let profile = RoadProfile::flat(&line.0, 100.0);
        let surface = project::quantize_z(100.0);
        let bridge =
            build(&Geometry::LineString(line.clone()), &profile, &b, Some("motorway"), Kind::Bridge, &mut |_: Coord| f64::NEG_INFINITY)
                .unwrap();
        let tunnel =
            build(&Geometry::LineString(line), &profile, &b, Some("motorway"), Kind::Tunnel, &mut |_: Coord| f64::NEG_INFINITY).unwrap();
        let deck_top = *bridge.z.iter().max().expect("non-empty");
        let tunnel_floor = *tunnel.z.iter().min().expect("non-empty");
        assert_eq!(deck_top, surface);
        assert_eq!(tunnel_floor, surface);
        assert_eq!(deck_top, tunnel_floor, "bridge deck top must meet the tunnel floor");
        // The bore rises above that shared road surface (it is not below it).
        assert!(tunnel.z.iter().any(|&z| z > surface), "tunnel bore must rise above the road");
    }

    #[test]
    fn box_has_width_across_the_road() {
        let b = Bounds::of_tile(14, 8500, 5800);
        let line = centre_line(&b);
        let profile = RoadProfile::flat(&line.0, 0.0);
        let mesh =
            build(&Geometry::LineString(line), &profile, &b, Some("motorway"), Kind::Tunnel, &mut |_: Coord| f64::NEG_INFINITY).unwrap();
        // The east→west road runs along x, so the cross-section spreads in y: the
        // box must occupy a band of qy, not a single line.
        let (lo, hi) = mesh.y.iter().fold((u16::MAX, u16::MIN), |(lo, hi), &y| (lo.min(y), hi.max(y)));
        assert!(hi > lo, "structure box has no width across the road");
    }

    #[test]
    fn portal_opens_past_a_buried_mouth() {
        // Flat road at height 0, so the bore ceiling sits at TUNNEL_HEIGHT_M
        // everywhere. The hillside buries the mouth for the first 10 m past the
        // end, then falls away. The portal must be pushed past the buried stretch
        // plus the margin — a steeper slope would push it further, which a fixed
        // distance could not follow.
        let inner = Coord { x: 6.0, y: 46.0 };
        let end = Coord { x: 6.001, y: 46.0 };
        let profile = RoadProfile::flat(&[inner, end], 0.0);
        let cos_lat = 46.0_f64.to_radians().cos();
        let east_m = |c: Coord| (c.x - end.x) * cos_lat * M_PER_DEG_LAT;
        let mut buried_10m = |c: Coord| if east_m(c) < 10.0 { 100.0 } else { -100.0 };
        let pushed = east_m(portal_point(inner, end, cos_lat, &profile, &mut buried_10m));
        assert!(
            (pushed - (10.0 + PORTAL_MARGIN_M)).abs() < PORTAL_STEP_M + 0.5,
            "portal pushed {pushed} m, expected ~{}",
            10.0 + PORTAL_MARGIN_M
        );

        // An already-clear mouth gets only the margin — the small poke the fixed
        // scheme gave every end.
        let mut clear = |_: Coord| -100.0;
        let poke = east_m(portal_point(inner, end, cos_lat, &profile, &mut clear));
        assert!((poke - PORTAL_MARGIN_M).abs() < 0.5, "clear mouth poked {poke} m");
    }

    #[test]
    fn non_line_geometry_yields_nothing() {
        let b = Bounds::of_tile(14, 8500, 5800);
        let profile = RoadProfile::flat(&centre_line(&b).0, 0.0);
        let pt = Geometry::Point(geo_types::Point::new(b.west, b.south));
        assert!(build(&pt, &profile, &b, None, Kind::Tunnel, &mut |_: Coord| f64::NEG_INFINITY).is_none());
    }

    #[test]
    fn half_width_scales_with_class() {
        assert!(half_width_m(Some("motorway")) > half_width_m(Some("residential")));
        assert_eq!(half_width_m(None), half_width_m(Some("residential")));
    }
}
