//! Structure generator — bridge decks and tunnel bores, swept as box prisms
//! along a corridor's solved profile (`crate::solve::profile`).
//!
//! The assemble stage resolved every structure into a constant-kind span, so
//! each feature here is uniformly a bridge or a tunnel. They are different
//! solids — a bridge is open, a tunnel has a roof — that meet at the road
//! surface:
//!
//! - **Bridge** ([`sweep_deck`]): a thin open slab whose top *is* the road
//!   surface (the single model), a [`DECK_THICKNESS_M`] underside below. No
//!   roof.
//! - **Tunnel** ([`sweep_bore`]): a roofed bore — roof a [`TUNNEL_HEIGHT_M`]
//!   ramp above the road, floor on the *same plane as a deck's underside*
//!   (`road − DECK_THICKNESS_M`) so the bore's bottom aligns with an adjoining
//!   deck. Each real portal mouth is **capped** with a portal face, so where a
//!   tunnel meets a bridge the bore ends solidly and the deck meets that face
//!   at the road surface.
//!
//! The portal is placed where the tunnel *actually emerges from the ground*,
//! not where the annotation happens to end (S5): the bore is the span where
//! the tube fits under the hill — the signed gap is the **roof's** daylight,
//! [`crate::solve::portals::roof_gap`], negative where the whole
//! constant-section solid is below the surface — and the cap sits on its zero
//! crossing, so by construction the mouth is always exactly flush with the
//! ground, never buried, never poking. A tunnel tagged over flat or shallow
//! ground (the gap never goes negative) has no bore at all; the caller drapes
//! it instead (the degradation ladder).
//!
//! Heights are a function of the solved global profile, so independent tile
//! fragments line up at the seams (invariant 5). The box is clipped to the
//! tile *proper*, not the format's half-tile buffer: an opaque solid built in
//! the buffer is replicated across neighbours and the independently-clipped
//! copies overlap into a staircase; clipped to the proper tile each draws once
//! and abuts its neighbour at the shared edge.
//!
//! Coordinates are tile-local quantized uint16 (x/y) and int32 millimetres
//! (z), matching `MeshGeometry`, with octahedral ECEF normals in the shared
//! ENU [`Frame`].

use geo_types::{Coord, Geometry, LineString};

use crate::building_mesh::Frame;
use crate::scene::DEG_M;
use crate::clip;
use crate::priors::{
    Kind, DECK_THICKNESS_M, PORTAL_CLEARANCE_M, PORTAL_MARCH_M, PORTAL_MAX_M,
    TUNNEL_HEIGHT_M,
};
use crate::project::{self, Bounds};
use crate::scene::SpanKind;
use crate::solve::Profile;
use crate::terrain::TerrainMesh;
use crate::tile_build::{prop_str, EncoderFeature};
use crate::value::Value;

/// Target spacing in metres for densifying the centerline, so the box follows
/// the road's curve and grade smoothly. Half the profile node spacing: the
/// deck rides a spline through the profile's smooth nodes, and sampling at
/// twice their density keeps the swept chords' sagitta invisible on the
/// curves a viaduct actually takes.
const SEGMENT_M: f64 = 4.0;

/// Cap on densified centerline vertices per line — a runaway guard.
const MAX_VERTS: usize = 4096;

/// Builds the structure box for a transportation feature and stores it in
/// `f.mesh`. A deck is the bare slab and a bore the bare tube — no supporting
/// geometry under either. Returns whether a solid was emitted — `false` (a
/// tunnel over flat ground, degenerate geometry) tells the caller to drape the
/// road instead.
pub fn stamp(f: &mut EncoderFeature, profile: &Profile, kind: SpanKind, bounds: &Bounds) -> bool {
    let frame = Frame::at_center(bounds);
    let feature_kind =
        Kind::parse(None, prop_str(f, "class").as_deref(), prop_str(f, "subclass").as_deref());
    let link = crate::priors::is_link(prop_str(f, "subclass").as_deref());
    // One cross-section function (docs/ROADS.md invariant 1): the deck spans the
    // same carriageway width the surface band and paint stroke use — the mapped
    // `width_m` the attribute profiler resolved (measured `width_rules` when
    // plausible, else the class prior; `synth::surface` reads the identical
    // property) — plus the structure shoulder, so the deck-top surface frames
    // the road ribbon and meets the approach band edge-to-edge with no width
    // step at the abutment. Falls back to the class half-width when a drivable
    // corridor carries no `width_m` (a non-P1 street). Any class with a
    // surface band takes this branch — a rail formation as much as a
    // carriageway, so a railway bridge is a rail-scale deck rather than the
    // footbridge slab it used to fall to. A structure carrying a class with
    // no surface of its own — a footbridge, cycleway or pedestrian bridge —
    // is pedestrian-scale instead: a narrow slab with no vehicle shoulder,
    // not the car-lane deck a street half-width would otherwise bake.
    let half_w = match (feature_kind.prior().surface != crate::priors::Surface::None)
        .then(|| feature_kind.prior().half_width_m(link))
        .flatten()
    {
        Some(prior_half) => {
            let half_carriageway = f
                .properties
                .iter()
                .find_map(|(k, v)| match (k.as_str(), v) {
                    ("width_m", Value::Double(w)) => Some(*w * 0.5),
                    _ => None,
                })
                .unwrap_or(prior_half);
            half_carriageway + crate::priors::STRUCTURE_SHOULDER_M
        }
        // No cross-section of its own: a footbridge, cycleway or pedestrian
        // bridge is pedestrian-scale — a narrow slab with no vehicle shoulder.
        None => crate::priors::PATH_STRUCTURE_HALF_WIDTH_M,
    };

    let mut acc = Accum::default();
    for line in lines(&f.geometry) {
        for piece in proper_pieces(&line.0, bounds) {
            match kind {
                SpanKind::Tunnel => sweep_bore(&mut acc, &frame, bounds, profile, &piece, half_w),
                _ => sweep_deck(&mut acc, &frame, bounds, profile, &piece, half_w),
            }
        }
    }
    match acc.into_mesh() {
        Some(mesh) => {
            f.mesh = Some(mesh);
            true
        }
        None => false,
    }
}

/// Whether `c` sits on the tile-proper boundary — i.e. a clip cut where the box
/// continues into the neighbouring tile, *not* a real run end. A portal mouth
/// is an interior run end, so a bore is capped only where `!on_tile_edge`.
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

/// Clips a span to the tile *proper*, returning the (in-order) pieces. An
/// opaque solid must not extend into the format's buffer or neighbouring tiles
/// would each rebuild and overlap it; clipped to the proper tile each draws
/// once and abuts its neighbour at the shared edge.
fn proper_pieces(span: &[Coord], bounds: &Bounds) -> Vec<Vec<Coord>> {
    let g = Geometry::LineString(LineString(span.to_vec()));
    match clip::clip_geometry(&g, bounds) {
        Some(Geometry::LineString(ls)) => vec![ls.0],
        Some(Geometry::MultiLineString(mls)) => mls.0.into_iter().map(|l| l.0).collect(),
        _ => Vec::new(),
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
    edge_across: Vec<i8>,
}

impl Accum {
    /// `across` is the signed across-carriageway coordinate (snorm ±127 at the
    /// paved edge, 0 at the centre / on a non-drivable face) driving the deck
    /// fragment's analytic edge AA. Only a bridge deck's top edges carry ±127.
    fn push(&mut self, qx: u16, qy: u16, z: i32, n: (i8, i8), across: i8) -> u32 {
        let i = self.x.len() as u32;
        self.x.push(qx);
        self.y.push(qy);
        self.z.push(z);
        self.normals.push(n.0);
        self.normals.push(n.1);
        self.edge_across.push(across);
        i
    }

    fn tri(&mut self, a: u32, b: u32, c: u32) {
        self.indices.extend_from_slice(&[a, b, c]);
    }

    fn into_mesh(self) -> Option<TerrainMesh> {
        if self.indices.is_empty() {
            return None;
        }
        Some(TerrainMesh {
            x: self.x,
            y: self.y,
            z: self.z,
            indices: self.indices,
            normals: self.normals,
            edge_across: self.edge_across,
        })
    }
}

/// One swept cross-section: the (smoothed) centerline position, the box top
/// and bottom heights, and the unit left-perpendicular (ENU metres) it spans.
struct Section {
    lon: f64,
    lat: f64,
    top_mm: i32,
    bot_mm: i32,
    /// Signed gap `road − terrain` in metres at this section: negative where
    /// the road runs under the ground (the buried bore), positive where it
    /// stands proud. The portal cap sits on the zero crossing. Only meaningful
    /// for a bore (a deck does not consult it).
    gap_m: f64,
    /// Left-perpendicular unit direction in ENU metres (east, north).
    left_e: f64,
    left_n: f64,
}

/// Sweeps a bridge deck along one (proper-clipped) span: a thin open slab on
/// the road surface — top *is* the road, a [`DECK_THICKNESS_M`] underside
/// below. No roof: a bridge is open, unlike a tunnel. Where the road sinks
/// to/under grade the slab passes under the terrain and is occluded, so the
/// deck meets the ground without a step. Each *real* run end (an interior
/// end, not a tile-boundary clip) is capped so the slab ends with a solid
/// face instead of an open cross-section.
fn sweep_deck(
    acc: &mut Accum,
    frame: &Frame,
    bounds: &Bounds,
    profile: &Profile,
    span: &[Coord],
    half_w: f64,
) {
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
            gap_m: 0.0, // a deck rides occlusion, not the gap
            left_e: d.left_e,
            left_n: d.left_n,
        })
        .collect();
    sweep_prism(acc, frame, bounds, &sections, half_w, true);
    let n = sections.len();
    if n >= 2 {
        if !on_tile_edge(pts[0], bounds) {
            cap_end(acc, frame, bounds, &sections[0], &sections[1], half_w);
        }
        if !on_tile_edge(pts[pts.len() - 1], bounds) {
            cap_end(acc, frame, bounds, &sections[n - 1], &sections[n - 2], half_w);
        }
    }
}

/// Sweeps a tunnel bore along one (proper-clipped) span: a roof a clean
/// [`TUNNEL_HEIGHT_M`] ramp above the road, and a floor on the same plane as a
/// bridge deck's underside so the bore's bottom aligns with an adjoining deck.
/// Each *real* portal mouth (a run end inside the tile, not a tile-boundary
/// clip) is capped with a portal face.
fn sweep_bore(
    acc: &mut Accum,
    frame: &Frame,
    bounds: &Bounds,
    profile: &Profile,
    span: &[Coord],
    half_w: f64,
) {
    let pts = densify(span, frame);
    if pts.len() < 2 {
        return;
    }
    // The cap criterion this run's own geometry elects (see
    // `solve::portals::tube_fit_majority`): a real bore trims and caps on the
    // line's crossings — its shallow mouths are the portal transition — while
    // a surface gallery caps on the roof's, so the tube is drawn only where
    // it fits. Same walk as the span reconciliation, so the mesh and the span
    // truth agree on where the mouth is.
    let mid = 0.5
        * (profile.arc_of(pts[0].x, pts[0].y)
            + profile.arc_of(pts[pts.len() - 1].x, pts[pts.len() - 1].y));
    let roof_caps = !crate::solve::portals::tube_fit_majority(profile, mid);
    let mut sections: Vec<Section> = profile
        .deck_nodes(&pts)
        .into_iter()
        .map(|d| bore_section(d.lon, d.lat, d.height_m, d.left_e, d.left_n, profile, roof_caps))
        .collect();
    if sections.len() < 2 {
        return;
    }

    // Resolve each end to the road/terrain crossing. Trimming an above-ground
    // end back to the crossing applies everywhere — even at a tile edge, an
    // end that has already emerged means the portal lies inside this tile.
    // Marching a *still-buried* end outward to where it surfaces is only for
    // an interior (real) portal; a buried tile-edge end stays open, the bore
    // continuing in the neighbour.
    let start_interior = !on_tile_edge(pts[0], bounds);
    let end_interior = !on_tile_edge(pts[pts.len() - 1], bounds);
    let cap_high = resolve_portal(&mut sections, frame, profile, End::High, end_interior, roof_caps);
    let cap_low = resolve_portal(&mut sections, frame, profile, End::Low, start_interior, roof_caps);

    // No buried span means the road never runs under the ground here: a tunnel
    // tagged over flat (or shallow) terrain. Drawing a bore would float a box
    // on the surface, so emit nothing and let the caller drape the road.
    if sections.len() < 2 || sections.iter().all(|s| s.gap_m >= 0.0) {
        return;
    }

    sweep_prism(acc, frame, bounds, &sections, half_w, false);

    let n = sections.len();
    if cap_low {
        cap_end(acc, frame, bounds, &sections[0], &sections[1], half_w);
    }
    if cap_high {
        cap_end(acc, frame, bounds, &sections[n - 1], &sections[n - 2], half_w);
    }
}

/// Which end of the ordered section list a portal sits on.
#[derive(Clone, Copy)]
enum End {
    Low,
    High,
}

/// A bore cross-section at `(lon, lat)` whose road surface is `road_m`: roof a
/// [`TUNNEL_HEIGHT_M`] above the road, floor a [`DECK_THICKNESS_M`] below (the
/// deck-aligned bottom), and the signed terrain gap sampled from the profile.
///
/// The section is the *same* everywhere along the bore: a constant tube riding
/// the profile, never squeezed by the cover overhead. `gap_m` carries the
/// criterion the run elected (`roof_caps`, from
/// [`crate::solve::portals::tube_fit_majority`], read against
/// [`Profile::surface_at`], pre-earthwork — the natural ground): a real
/// bore's gap is the road line's, so its mouth sits where the road emerges
/// and the roof pokes briefly through the thinning cover — the portal
/// transition it always had. A surface gallery's gap is the roof's
/// ([`crate::solve::portals::roof_gap`]), so the tube is drawn only where it
/// fits; swept by the line, the Territet gallery snaked its roof out of the
/// slope on both sides of the road that crossed it. Either way the trim, the
/// outward march and the portal cap sit on the same zero crossing the
/// [`crate::solve::portals`] solver places, so the mesh, the span truth and
/// the daylighting cut agree on where the mouth is.
fn bore_section(
    lon: f64,
    lat: f64,
    road_m: f64,
    left_e: f64,
    left_n: f64,
    profile: &Profile,
    roof_caps: bool,
) -> Section {
    let terrain = profile.surface_at(lon, lat);
    Section {
        lon,
        lat,
        top_mm: project::quantize_z(road_m + TUNNEL_HEIGHT_M),
        bot_mm: project::quantize_z(road_m - DECK_THICKNESS_M),
        gap_m: if roof_caps {
            crate::solve::portals::roof_gap(road_m, terrain)
        } else {
            road_m - terrain
        },
        left_e,
        left_n,
    }
}

/// Resolves one end of the bore onto the road/terrain crossing, returning
/// whether the end is capped. If it already stands above ground (`gap ≥ 0`)
/// the above-ground sections are trimmed and a cap is interpolated onto the
/// crossing (always, even at a tile edge — the portal is inside this tile). If
/// it is still buried (`gap < 0`): an `interior` end is marched outward to
/// where the road emerges and capped there; a tile-edge end stays open (the
/// bore continues in the neighbour).
fn resolve_portal(
    sections: &mut Vec<Section>,
    frame: &Frame,
    profile: &Profile,
    end: End,
    interior: bool,
    roof_caps: bool,
) -> bool {
    let last = sections.len() - 1;
    let edge = match end {
        End::Low => 0,
        End::High => last,
    };
    if sections[edge].gap_m >= 0.0 {
        trim_to_crossing(sections, end);
        return true;
    }
    if !interior {
        return false; // buried tile-edge cut: leave open
    }
    let inner = match end {
        End::Low => 1,
        End::High => last - 1,
    };
    if let Some(cap) =
        march_to_crossing(&sections[edge], &sections[inner], frame, profile, roof_caps)
    {
        match end {
            End::Low => sections.insert(0, cap),
            End::High => sections.push(cap),
        }
    }
    true
}

/// Trims an above-ground end down to the buried span: drops the sections that
/// stand proud of the terrain and interpolates a cap onto the `gap = 0`
/// crossing. A no-op when no buried section exists (the caller drops the
/// bore).
fn trim_to_crossing(sections: &mut Vec<Section>, end: End) {
    match end {
        End::Low => {
            if let Some(i) =
                (0..sections.len()).find(|&i| sections[i].gap_m < 0.0).filter(|&i| i > 0)
            {
                let cap = crossing_section(&sections[i], &sections[i - 1]);
                sections.drain(0..i);
                sections.insert(0, cap);
            }
        }
        End::High => {
            let last = sections.len() - 1;
            if let Some(i) = (0..sections.len())
                .rev()
                .find(|&i| sections[i].gap_m < 0.0)
                .filter(|&i| i < last)
            {
                let cap = crossing_section(&sections[i], &sections[i + 1]);
                sections.truncate(i + 1);
                sections.push(cap);
            }
        }
    }
}

/// Marches outward from a buried boundary section `edge` along the *corridor*
/// (not the straight `inner → edge` tangent), sampling the profile, and returns
/// a cap section at the first point where the road reaches the terrain (the
/// bore emerges), nudged out by [`PORTAL_CLEARANCE_M`] so the mouth sits just
/// clear. Falls back to a cap at [`PORTAL_MAX_M`] if the approach stays buried
/// that far.
///
/// Following the corridor arc rather than a straight extrapolation matters on a
/// curving tunnel: a straight tangent walks off the corridor within tens of
/// metres, so the road height and gap it samples — and the portal it places —
/// land beside the road instead of on it. Marching the arc keeps plan position,
/// height, and gap all on the corridor.
fn march_to_crossing(
    edge: &Section,
    inner: &Section,
    _frame: &Frame,
    profile: &Profile,
    roof_caps: bool,
) -> Option<Section> {
    let a_edge = profile.arc_of(edge.lon, edge.lat);
    let a_inner = profile.arc_of(inner.lon, inner.lat);
    let d = a_edge - a_inner;
    if d.abs() < 1e-9 {
        return None;
    }
    let outward = d.signum(); // +1 toward increasing arc, −1 toward decreasing
    let at = |dist: f64| -> Section {
        let a = a_edge + outward * dist;
        let p = profile.point_at_arc(a);
        bore_section(p.x, p.y, profile.road_at_arc(a), edge.left_e, edge.left_n, profile, roof_caps)
    };
    let mut prev = (0.0, edge.gap_m.min(-f64::MIN_POSITIVE)); // (dist, gap), buried
    let mut dist = PORTAL_MARCH_M;
    while dist <= PORTAL_MAX_M {
        let s = at(dist);
        if s.gap_m >= 0.0 {
            // Crossing between prev (buried) and here (emerged): interpolate,
            // then step out by the emergence clearance.
            let t = prev.1 / (prev.1 - s.gap_m);
            let cross = prev.0 + (dist - prev.0) * t;
            return Some(at(cross + PORTAL_CLEARANCE_M));
        }
        prev = (dist, s.gap_m);
        dist += PORTAL_MARCH_M;
    }
    Some(at(PORTAL_MAX_M)) // never emerged within reach: best-effort cap
}

/// Interpolates a cap section onto the `gap = 0` crossing between a buried
/// section `a` and an above-ground section `b`.
fn crossing_section(a: &Section, b: &Section) -> Section {
    let t = a.gap_m / (a.gap_m - b.gap_m); // a.gap < 0 ≤ b.gap, so t ∈ [0, 1)
    Section {
        lon: lerp(a.lon, b.lon, t),
        lat: lerp(a.lat, b.lat, t),
        top_mm: lerp(a.top_mm as f64, b.top_mm as f64, t).round() as i32,
        bot_mm: lerp(a.bot_mm as f64, b.bot_mm as f64, t).round() as i32,
        gap_m: 0.0,
        left_e: lerp(a.left_e, b.left_e, t),
        left_n: lerp(a.left_n, b.left_n, t),
    }
}

/// Linear interpolation between `a` and `b` at `t`.
fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

/// The (qx, qy) corner of a cross-section offset `side` half-widths to its left.
fn corner(s: &Section, side: f64, frame: &Frame, bounds: &Bounds, half_w: f64) -> (u16, u16) {
    let dlon = s.left_e * half_w * side / frame.m_per_deg_lon;
    let dlat = s.left_n * half_w * side / DEG_M;
    (project::quantize_x(s.lon + dlon, bounds), project::quantize_y(s.lat + dlat, bounds))
}

/// Closes a bore's end cross-section `end` with a portal face, its outward
/// normal pointing away from the interior section `inner`.
fn cap_end(
    acc: &mut Accum,
    frame: &Frame,
    bounds: &Bounds,
    end: &Section,
    inner: &Section,
    half_w: f64,
) {
    let (el, er) =
        (corner(end, 1.0, frame, bounds, half_w), corner(end, -1.0, frame, bounds, half_w));
    let de = (end.lon - inner.lon) * frame.m_per_deg_lon;
    let dn = (end.lat - inner.lat) * DEG_M;
    let len = (de * de + dn * dn).sqrt().max(1e-9);
    let nrm = frame.encode_enu(de / len, dn / len, 0.0);
    quad(acc, (el, end.bot_mm), (er, end.bot_mm), (er, end.top_mm), (el, end.top_mm), nrm);
}

/// Sweeps a box prism through the cross-sections: a top face, a bottom face,
/// and the two side walls, each a single smooth-shaded strip. Vertices are
/// shared between adjacent segments within a face and carry per-section
/// normals — the sides follow the turning perpendicular, the top and bottom
/// the deck's local tangent (grade included) — so a curving, ramping deck
/// shades continuously instead of as flat panels. Across faces vertices are
/// *not* shared: the deck's edges stay hard creases. Cull mode is off on the
/// client, so winding only feeds lighting via the normals.
fn sweep_prism(
    acc: &mut Accum,
    frame: &Frame,
    bounds: &Bounds,
    sections: &[Section],
    half_w: f64,
    top_is_road: bool,
) {
    let n = sections.len();
    if n < 2 {
        return;
    }
    // A bridge deck's top face IS the carriageway, so its two edge strips carry
    // the ±1 across-coord for analytic edge AA (0 at the interpolated centre).
    // A tunnel's box top is its roof, not a drivable surface, so it stays 0.
    let (a_left, a_right) = if top_is_road { (127i8, -127i8) } else { (0, 0) };
    // Vertex ids per section, per face: top left/right, bottom left/right,
    // left wall bottom/top, right wall bottom/top.
    let (mut tl, mut tr) = (Vec::with_capacity(n), Vec::with_capacity(n));
    let (mut bl, mut br) = (Vec::with_capacity(n), Vec::with_capacity(n));
    let (mut lb, mut lt) = (Vec::with_capacity(n), Vec::with_capacity(n));
    let (mut rb, mut rt) = (Vec::with_capacity(n), Vec::with_capacity(n));
    for i in 0..n {
        let s = &sections[i];
        let (p, q) = (&sections[i.saturating_sub(1)], &sections[(i + 1).min(n - 1)]);
        // Local tangent (ENU metres, grade included) over the neighbours.
        let fe = (q.lon - p.lon) * frame.m_per_deg_lon;
        let fnn = (q.lat - p.lat) * DEG_M;
        let fu = (q.top_mm - p.top_mm) as f64 / 1000.0;
        // Top normal = tangent × left, flipped upward when a clipped fragment
        // runs opposite the corridor (its left then opposes the travel
        // direction). encode_enu normalises.
        let (mut te, mut tn, mut tu) =
            (-fu * s.left_n, fu * s.left_e, fe * s.left_n - fnn * s.left_e);
        if tu < 0.0 {
            (te, tn, tu) = (-te, -tn, -tu);
        }
        let n_top = frame.encode_enu(te, tn, tu);
        let n_bot = frame.encode_enu(-te, -tn, -tu);
        let n_left = frame.encode_enu(s.left_e, s.left_n, 0.0);
        let n_right = frame.encode_enu(-s.left_e, -s.left_n, 0.0);
        let l = corner(s, 1.0, frame, bounds, half_w);
        let r = corner(s, -1.0, frame, bounds, half_w);
        tl.push(acc.push(l.0, l.1, s.top_mm, n_top, a_left));
        tr.push(acc.push(r.0, r.1, s.top_mm, n_top, a_right));
        bl.push(acc.push(l.0, l.1, s.bot_mm, n_bot, 0));
        br.push(acc.push(r.0, r.1, s.bot_mm, n_bot, 0));
        lb.push(acc.push(l.0, l.1, s.bot_mm, n_left, 0));
        lt.push(acc.push(l.0, l.1, s.top_mm, n_left, 0));
        rb.push(acc.push(r.0, r.1, s.bot_mm, n_right, 0));
        rt.push(acc.push(r.0, r.1, s.top_mm, n_right, 0));
    }
    for i in 0..n - 1 {
        acc.tri(tl[i], tr[i], tr[i + 1]);
        acc.tri(tl[i], tr[i + 1], tl[i + 1]);
        acc.tri(bl[i], br[i], br[i + 1]);
        acc.tri(bl[i], br[i + 1], bl[i + 1]);
        acc.tri(lb[i], lt[i], lt[i + 1]);
        acc.tri(lb[i], lt[i + 1], lb[i + 1]);
        acc.tri(rb[i], rt[i], rt[i + 1]);
        acc.tri(rb[i], rt[i + 1], rb[i + 1]);
    }
}

/// Emits a quad (two triangles) from four corners, each `((qx, qy), z_mm)`,
/// all carrying the same face normal.
fn quad(
    acc: &mut Accum,
    p0: ((u16, u16), i32),
    p1: ((u16, u16), i32),
    p2: ((u16, u16), i32),
    p3: ((u16, u16), i32),
    nrm: (i8, i8),
) {
    // A cap is a non-drivable face → across 0.
    let v0 = acc.push((p0.0).0, (p0.0).1, p0.1, nrm, 0);
    let v1 = acc.push((p1.0).0, (p1.0).1, p1.1, nrm, 0);
    let v2 = acc.push((p2.0).0, (p2.0).1, p2.1, nrm, 0);
    let v3 = acc.push((p3.0).0, (p3.0).1, p3.1, nrm, 0);
    acc.tri(v0, v1, v2);
    acc.tri(v0, v2, v3);
}

/// Densifies a centerline to ~[`SEGMENT_M`] spacing in (lon, lat), so the
/// swept box follows the road's curve and grade.
fn densify(pts: &[Coord], frame: &Frame) -> Vec<Coord> {
    let mut out = Vec::new();
    if pts.is_empty() {
        return out;
    }
    out.push(pts[0]);
    for w in pts.windows(2) {
        let (p0, p1) = (w[0], w[1]);
        let de = (p1.x - p0.x) * frame.m_per_deg_lon;
        let dn = (p1.y - p0.y) * DEG_M;
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
    use crate::priors::RoadClass;

    /// A straight west→east line across the tile centre.
    fn centre_line(b: &Bounds) -> LineString {
        let cy = (b.south + b.north) * 0.5;
        LineString(vec![
            Coord { x: b.west + 0.3 * b.width(), y: cy },
            Coord { x: b.west + 0.7 * b.width(), y: cy },
        ])
    }

    /// A straight west→east line spanning the tile-width fractions `u0..u1`.
    fn sub_line(b: &Bounds, u0: f64, u1: f64) -> LineString {
        let cy = (b.south + b.north) * 0.5;
        LineString(vec![
            Coord { x: b.west + u0 * b.width(), y: cy },
            Coord { x: b.west + u1 * b.width(), y: cy },
        ])
    }

    /// A west→east profile across the whole tile: the road flat at 100 m and a
    /// terrain hill that pierces it — a buried plateau in the centre flanked by
    /// ground that dips below the road. The road runs under the hill over
    /// `u ∈ (≈0.417, ≈0.583)` and emerges (crosses the terrain) at the flanks,
    /// so a bore's portals land on those crossings.
    fn hill_profile(b: &Bounds) -> Profile {
        let cy = (b.south + b.north) * 0.5;
        let n = 201;
        let nodes: Vec<Coord> = (0..n)
            .map(|i| Coord { x: b.west + b.width() * i as f64 / (n - 1) as f64, y: cy })
            .collect();
        let road_m = vec![100.0; n];
        let terrain_m: Vec<f64> = (0..n)
            .map(|i| {
                let d = (i as f64 / (n - 1) as f64 - 0.5).abs();
                if d < 0.05 {
                    140.0 // buried plateau (gap −40)
                } else if d < 0.10 {
                    140.0 - (d - 0.05) / 0.05 * 60.0 // 140 → 80, crosses 100 at d≈0.083
                } else {
                    80.0 // flanks below the road (gap +20)
                }
            })
            .collect();
        Profile::from_heights(&nodes, road_m, terrain_m)
    }

    /// Sweeps a bridge deck over a whole line.
    fn deck(line: &LineString, profile: &Profile, b: &Bounds) -> Option<TerrainMesh> {
        let frame = Frame::at_center(b);
        let half_w = Kind::Road(RoadClass::Motorway).prior().half_width_m(false).unwrap();
        let mut acc = Accum::default();
        for piece in proper_pieces(&line.0, b) {
            sweep_deck(&mut acc, &frame, b, profile, &piece, half_w);
        }
        acc.into_mesh()
    }

    /// Sweeps a tunnel bore over a whole line.
    fn bore(line: &LineString, profile: &Profile, b: &Bounds) -> Option<TerrainMesh> {
        let frame = Frame::at_center(b);
        let half_w = Kind::Road(RoadClass::Motorway).prior().half_width_m(false).unwrap();
        let mut acc = Accum::default();
        for piece in proper_pieces(&line.0, b) {
            sweep_bore(&mut acc, &frame, b, profile, &piece, half_w);
        }
        acc.into_mesh()
    }

    #[test]
    fn deck_width_follows_the_carriageway_width_property() {
        // The deck spans the mapped `width_m` (the property `synth::surface`
        // bands with), not the bare class prior, so deck and approach band meet
        // edge-to-edge with no width step at the abutment (ROADS.md invariant 1).
        let b = Bounds::of_tile(14, 8500, 5800);
        let line = centre_line(&b);
        let profile = Profile::flat(&line.0, 100.0);
        let y_spread = |width_m: Option<f64>| -> f64 {
            let mut props = vec![("class".to_string(), Value::String("residential".into()))];
            if let Some(w) = width_m {
                props.push(("width_m".to_string(), Value::Double(w)));
            }
            let mut f = EncoderFeature {
                id: 1,
                geometry: Geometry::LineString(line.clone()),
                properties: props,
                elevation: None,
                z: None,
                mesh: None,
                synth: crate::synth::Synth::None,
            };
            assert!(stamp(&mut f, &profile, SpanKind::Bridge, &b));
            let mesh = f.mesh.expect("deck mesh");
            // The west→east line spreads its cross-section in y.
            let (lo, hi) =
                mesh.y.iter().fold((u16::MAX, u16::MIN), |(lo, hi), &y| (lo.min(y), hi.max(y)));
            (hi - lo) as f64
        };
        let prior = y_spread(None); // residential class prior (~5.5 m carriageway)
        let wide = y_spread(Some(20.0)); // a mapped 20 m carriageway
        assert!(
            wide > prior * 1.5,
            "a 20 m mapped width must widen the deck well past the class prior \
             (prior spread {prior}, wide {wide})"
        );
    }

    #[test]
    fn bridge_deck_is_a_thin_open_slab_on_the_road_surface() {
        // Top is the road surface, a slab hangs below, and there is no roof.
        let b = Bounds::of_tile(14, 8500, 5800);
        let line = centre_line(&b);
        let profile = Profile::flat(&line.0, 100.0);
        let mesh = deck(&line, &profile, &b).expect("deck");

        let surface = project::quantize_z(100.0);
        assert!(
            mesh.z.iter().any(|&z| z == surface),
            "expected a deck-top vertex on the road surface"
        );
        assert!(mesh.z.iter().all(|&z| z <= surface), "a bridge has no roof above the road");
        let underside = project::quantize_z(100.0 - DECK_THICKNESS_M);
        assert!(mesh.z.iter().any(|&z| z == underside), "expected a deck underside vertex");
        assert_eq!(mesh.indices.len() % 6, 0);
        assert!(mesh.indices.iter().all(|&i| (i as usize) < mesh.x.len()));
        assert_eq!(mesh.normals.len(), mesh.x.len() * 2);
        // The east→west road runs along x, so the cross-section spreads in y.
        let (lo, hi) =
            mesh.y.iter().fold((u16::MAX, u16::MIN), |(lo, hi), &y| (lo.min(y), hi.max(y)));
        assert!(hi > lo, "deck has no width across the road");
    }

    #[test]
    fn tunnel_bore_has_a_roof_above_the_road_and_a_deck_aligned_floor() {
        // Road at 100 m under a hill: a roof a bore height above the road, and
        // a floor on the deck-underside plane (road − DECK_THICKNESS) so the
        // bottom aligns with a bridge deck rather than hanging below it.
        let b = Bounds::of_tile(14, 8500, 5800);
        let line = sub_line(&b, 0.45, 0.55); // within the buried plateau
        let profile = hill_profile(&b);
        let mesh = bore(&line, &profile, &b).expect("bore");

        let roof = project::quantize_z(100.0 + TUNNEL_HEIGHT_M);
        let floor = project::quantize_z(100.0 - DECK_THICKNESS_M);
        assert!(
            mesh.z.iter().any(|&z| z == roof),
            "expected a roof vertex a bore height above the road"
        );
        assert!(
            mesh.z.iter().any(|&z| z == floor),
            "expected a floor vertex on the deck-underside plane"
        );
        assert!(mesh.z.iter().all(|&z| z <= roof), "nothing above the roof");
        assert!(mesh.z.iter().all(|&z| z >= floor), "nothing below the floor");
    }

    #[test]
    fn bore_is_dropped_over_flat_terrain() {
        // A tunnel tagged where the ground sits on the road (no hill): the road
        // never runs underground, so there is no bore to draw — a box on the
        // grass is worse than nothing.
        let b = Bounds::of_tile(14, 8500, 5800);
        let line = centre_line(&b);
        let profile = Profile::flat(&line.0, 100.0);
        assert!(bore(&line, &profile, &b).is_none(), "no bore over flat ground");
    }

    #[test]
    fn stamp_reports_a_dropped_bore_so_the_caller_can_drape() {
        let b = Bounds::of_tile(14, 8500, 5800);
        let line = centre_line(&b);
        let profile = Profile::flat(&line.0, 100.0);
        let mut f = EncoderFeature {
            id: 1,
            geometry: Geometry::LineString(line),
            properties: vec![],
            elevation: None,
            z: None,
            mesh: None,
            synth: crate::synth::Synth::None,
        };
        assert!(!stamp(&mut f, &profile, SpanKind::Tunnel, &b));
        assert!(f.mesh.is_none());
    }

    #[test]
    fn a_high_deck_carries_no_supports_under_it() {
        // A deck 40 m over the valley floor: the slab is all there is — nothing
        // reaches down to the ground (no piers, no abutment seats).
        let b = Bounds::of_tile(14, 8500, 5800);
        let line = centre_line(&b);
        let n = 101;
        let nodes: Vec<Coord> = (0..n)
            .map(|i| Coord {
                x: line.0[0].x + (line.0[1].x - line.0[0].x) * i as f64 / (n - 1) as f64,
                y: line.0[0].y,
            })
            .collect();
        let profile = Profile::from_heights(&nodes, vec![100.0; n], vec![60.0; n]);

        let mesh = deck(&line, &profile, &b).expect("deck");
        assert_eq!(
            *mesh.z.iter().min().unwrap(),
            project::quantize_z(100.0 - DECK_THICKNESS_M),
            "nothing reaches below the deck underside"
        );
    }

    #[test]
    fn bore_caps_its_interior_portals() {
        // Both ends are interior run ends (real portals), so both are capped.
        // The prism is 4 quads/segment (24 indices); two caps add 12, so the
        // index total is 12 beyond a whole number of segments.
        let b = Bounds::of_tile(14, 8500, 5800);
        let line = sub_line(&b, 0.45, 0.55);
        let profile = hill_profile(&b);
        let mesh = bore(&line, &profile, &b).expect("bore");
        assert_eq!(mesh.indices.len() % 24, 12, "expected two portal caps (+12 indices)");
    }

    #[test]
    fn bore_marches_a_buried_end_out_to_the_crossing() {
        // Both ends of the line sit inside the buried plateau, so each is
        // marched outward to where the road emerges from the hill — the bore
        // pokes well past the centerline ends.
        let b = Bounds::of_tile(14, 8500, 5800);
        let line = sub_line(&b, 0.45, 0.55);
        let profile = hill_profile(&b);
        let mesh = bore(&line, &profile, &b).expect("bore");

        let lons: Vec<f64> = mesh.x.iter().map(|&q| project::dequantize_x(q, &b)).collect();
        let mesh_w = lons.iter().cloned().fold(f64::INFINITY, f64::min);
        let mesh_e = lons.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let (line_w, line_e) = (line.0[0].x.min(line.0[1].x), line.0[0].x.max(line.0[1].x));
        assert!(mesh_w < line_w, "west portal must reach back to the crossing");
        assert!(mesh_e > line_e, "east portal must reach out to the crossing");
    }

    #[test]
    fn bore_trims_an_end_that_already_stands_above_ground() {
        // The east end of the line runs out onto the flank, where the road
        // already stands above the (dipped) ground. That stretch is not a
        // tunnel, so the bore is trimmed back to the emergence crossing well
        // short of the line end.
        let b = Bounds::of_tile(14, 8500, 5800);
        let line = sub_line(&b, 0.50, 0.70); // centre (buried) out to the flank (above)
        let profile = hill_profile(&b);
        let mesh = bore(&line, &profile, &b).expect("bore");

        let lons: Vec<f64> = mesh.x.iter().map(|&q| project::dequantize_x(q, &b)).collect();
        let mesh_e = lons.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let line_e = line.0[0].x.max(line.0[1].x);
        // The crossing is near u≈0.583; the line end is u=0.70.
        let crossing = b.west + 0.60 * b.width();
        assert!(mesh_e < crossing, "bore must stop at the crossing, not run onto the flank");
        assert!(mesh_e < line_e, "bore must not reach the above-ground line end");
    }

    #[test]
    fn bore_leaves_tile_edge_cuts_open() {
        // A bore running off the west edge while still buried: that end is a
        // clip (open, the bore continues in the neighbour); only the interior
        // east end is resolved and capped — one cap, so the index total is 6
        // beyond a whole number of segments.
        let b = Bounds::of_tile(14, 8500, 5800);
        let cy = (b.south + b.north) * 0.5;
        // Buried (terrain above the road) west of u≈0.55, emerging to the east.
        let n = 201;
        let nodes: Vec<Coord> = (0..n)
            .map(|i| Coord { x: b.west + b.width() * i as f64 / (n - 1) as f64, y: cy })
            .collect();
        let road_m = vec![100.0; n];
        let terrain_m: Vec<f64> = (0..n)
            .map(|i| {
                let u = i as f64 / (n - 1) as f64;
                if u < 0.55 { 130.0 } else { 130.0 - (u - 0.55) / 0.10 * 60.0 } // crosses 100 at u=0.60
            })
            .collect();
        let profile = Profile::from_heights(&nodes, road_m, terrain_m);
        // West end off-tile (clipped, buried → open); east end interior
        // (buried, marched to the crossing → capped).
        let line = LineString(vec![
            Coord { x: b.west - 0.2 * b.width(), y: cy },
            Coord { x: b.west + 0.52 * b.width(), y: cy },
        ]);
        let mesh = bore(&line, &profile, &b).expect("bore");
        assert_eq!(mesh.indices.len() % 24, 6, "a tile-edge end stays open, only one cap");
    }

    #[test]
    fn a_tube_that_does_not_fit_is_not_drawn() {
        // A covered urban road where the ground rises only 2 m above the
        // carriageway: the constant tube is TUNNEL_HEIGHT_M tall, so there is
        // no room to drive through — the roof would stand 3 m proud of the
        // ground for the whole run. The old behaviour swept it anyway ("the
        // terrain is what gives"), which drew the Territet gallery snaking
        // out of its hillside; now the roof gap never goes negative, so no
        // bore is emitted at all and the span reconciles to open grade
        // upstream.
        let b = Bounds::of_tile(14, 8500, 5800);
        let cy = (b.south + b.north) * 0.5;
        let n = 201;
        let nodes: Vec<Coord> = (0..n)
            .map(|i| Coord { x: b.west + b.width() * i as f64 / (n - 1) as f64, y: cy })
            .collect();
        let terrain_m: Vec<f64> = (0..n)
            .map(|i| {
                let d = (i as f64 / (n - 1) as f64 - 0.5).abs();
                if d < 0.25 { 102.0 } else { 90.0 }
            })
            .collect();
        let profile = Profile::from_heights(&nodes, vec![100.0; n], terrain_m);
        assert!(
            bore(&sub_line(&b, 0.40, 0.60), &profile, &b).is_none(),
            "a 2 m cover fits no {TUNNEL_HEIGHT_M} m tube: nothing is drawn"
        );
    }

    #[test]
    fn bore_keeps_its_section_under_a_thin_but_sufficient_cover() {
        // Ground half a metre over the roof — the covered-bore ceiling's own
        // margin (TUNNEL_COVER_M): the tube is the same tube it is under a
        // mountain, full headroom, never squeezed by the cover.
        let b = Bounds::of_tile(14, 8500, 5800);
        let cy = (b.south + b.north) * 0.5;
        let n = 201;
        let nodes: Vec<Coord> = (0..n)
            .map(|i| Coord { x: b.west + b.width() * i as f64 / (n - 1) as f64, y: cy })
            .collect();
        let terrain_m: Vec<f64> = (0..n)
            .map(|i| {
                let d = (i as f64 / (n - 1) as f64 - 0.5).abs();
                if d < 0.25 { 100.0 + TUNNEL_HEIGHT_M + 0.5 } else { 90.0 }
            })
            .collect();
        let profile = Profile::from_heights(&nodes, vec![100.0; n], terrain_m);
        let mesh = bore(&sub_line(&b, 0.40, 0.60), &profile, &b).expect("bore");
        let roof = project::quantize_z(100.0 + TUNNEL_HEIGHT_M);
        let floor = project::quantize_z(100.0 - DECK_THICKNESS_M);
        assert!(mesh.z.iter().all(|&z| z == roof || z == floor), "the tube keeps one section");
        assert!(mesh.z.iter().any(|&z| z == roof), "full headroom under a thin cover too");
    }

    #[test]
    fn bore_floor_is_independent_of_the_terrain() {
        // The floor rides the road profile (road − DECK_THICKNESS), not the
        // terrain, so a tunnel and a bridge share a bottom whatever the ground
        // does beneath.
        let b = Bounds::of_tile(14, 8500, 5800);
        let deck_line = centre_line(&b);
        let deck = deck(&deck_line, &Profile::flat(&deck_line.0, 100.0), &b).expect("deck");
        let bore = bore(&sub_line(&b, 0.45, 0.55), &hill_profile(&b), &b).expect("bore");
        let deck_floor = *deck.z.iter().min().expect("non-empty");
        let bore_floor = *bore.z.iter().min().expect("non-empty");
        assert_eq!(deck_floor, project::quantize_z(100.0 - DECK_THICKNESS_M));
        assert_eq!(bore_floor, deck_floor, "bore bottom must align with the deck bottom");
    }
}
