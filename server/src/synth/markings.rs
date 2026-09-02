//! Longitudinal road markings — the painted lines on the carriageway
//! (docs/ROADS.md P3).
//!
//! Markings are baked server-side as ordinary narrow line features of class
//! `marking`, so the client's existing stroke pipeline draws them —
//! SDF-antialiased, at physical width, over the road paint they follow in
//! draw order — with no client change; the style file gives the class its
//! colour. The lane model never reaches the client (ROADS.md §6.2): which
//! lines a road paints is decided here from the class ladder, the one-way
//! verdict, and the P1 width — a dashed centre line between opposing flows,
//! solid edge lines on the motorway network.
//!
//! Generation runs in **phase 1**, on the pre-clip geometry — the whole
//! unclaimed segment, or a corridor piece cut at its global span boundaries —
//! never on a tile window. That is what makes the dash phase global
//! (ROADS.md H4, invariant 4): the dashes are cut once from arclength zero of
//! a global object and then clipped like any feature, so any two tiles carry
//! identical copies of every dash. The emitted features ride a
//! [`Synth::Road`] tag and bake their heights in phase 2 exactly like the
//! road paint they lie on.

use geo_types::{Coord, Geometry, LineString, MultiLineString};

use crate::assemble::grid::GridIndex;
use crate::building_mesh::Frame;
use crate::scene::DEG_M;
use crate::priors;
use crate::project::Bounds;
use crate::synth::area::Area;
use crate::value::Value;

/// Cap on the offset miter scale at a bend, matching the surface band's.
const MITER_MAX: f64 = 1.5;

/// Shortest painted solid line worth emitting, in metres: the stubs left
/// between close junctions read as noise, not as road markings.
const MIN_LINE_M: f64 = 8.0;

/// The centre guide line's dash pattern in metres — the Swiss Leitlinie runs
/// 3/3 m in town and 6/6 m outside; one compromise pattern keeps the phase
/// machinery simple.
const CENTRE_DASH_M: f64 = 4.0;
const CENTRE_GAP_M: f64 = 6.0;

/// The lane divider's dash pattern in metres — longer stride than the centre
/// guide line, as on a real carriageway.
const LANE_DASH_M: f64 = 5.0;
const LANE_GAP_M: f64 = 9.0;

/// How much clear asphalt a longitudinal line keeps before a zebra ladder, in
/// metres, beyond the ladder's own half-depth. The bars reach
/// `priors::CROSSING_WIDTH_M / 2` from the chord ([`crossing_bars`] strokes
/// them centred on it), and paint stopping flush with the outermost bar still
/// touches it once the client rounds the cap — so the stop keeps the margin
/// road paint actually holds before a crosswalk. Well over the bar's own
/// clearance, and the excess is not generosity: the filter runs on the
/// model-space line and the paint is drawn snapped to the smoothed sweep,
/// which displaces it laterally — a median half-metre at a junction mouth
/// (`street.kerb_join`), which is exactly where crossings live. A margin
/// sized to the ladder alone let displaced dash tips back inside it (0.33 to
/// 0.45 m from a bar at three Montreux sites at 0.6 m; one survived 1.0 m).
const CROSSING_STOP_M: f64 = 1.5;

/// One painted line to emit: its geometry and painted width. The caller
/// attaches the `marking` class and the synth tag of the road it lies on.
pub struct Marking {
    pub geometry: Geometry,
    pub width_m: f64,
    /// The style class the paint is emitted as: `"marking"` for road paint,
    /// `"rail_line"` for the rail heads riding the ballast band. Distinct
    /// classes because colour is keyed by class (paint is white, a rail is
    /// dark), and because the calibrated `paint.*` check populations filter
    /// on the literal `"marking"`.
    pub class: &'static str,
}

impl Marking {
    /// The tile properties of a marking feature.
    pub fn properties(&self) -> Vec<(String, Value)> {
        vec![
            ("class".to_string(), Value::String(self.class.to_string())),
            ("width_m".to_string(), Value::Double(self.width_m)),
        ]
    }
}

/// The painted lines for one road line: a dashed centre line between opposing
/// flows, dashed dividers between the same-direction lanes of a one-way
/// carriageway (their count inferred from the width — ROADS.md H2), and
/// solid edge lines on the motorway network, per the ladder
/// (`priors::has_centre_line` / `has_lane_lines` / `has_edge_lines`). `line`
/// must be pre-clip geometry — a whole segment or a corridor span piece — so
/// the dash phase anchors to a global arclength origin. `areas` are the paved
/// intersections near the line: solid lines stop at them, and any dash whose
/// midpoint falls inside one is dropped. `chords` are the registered crossing
/// chords: a dash reaching inside a zebra ladder's footprint is dropped too,
/// because the crossing owns that stretch of paint ([`ChordIndex`]).
pub fn for_line(
    line: &LineString,
    class: &str,
    oneway: bool,
    width_m: f64,
    areas: &[&Area],
    chords: &ChordIndex,
) -> Vec<Marking> {
    let centre = priors::has_centre_line(class, oneway);
    let lanes = if priors::has_lane_lines(class, oneway) {
        priors::lane_count(class, width_m)
    } else {
        1
    };
    let edges = priors::has_edge_lines(class);
    if (!centre && !edges && lanes < 2) || line.0.len() < 2 {
        return Vec::new();
    }
    let mut out = Vec::new();
    let mut push_dashes = |dashes: Vec<LineString>, width: f64| {
        let kept: Vec<LineString> = dashes
            .into_iter()
            .filter(|d| !midpoint_paved(d, areas) && !chords.blocks(d))
            .collect();
        if !kept.is_empty() {
            out.push(Marking {
                geometry: Geometry::MultiLineString(MultiLineString(kept)),
                width_m: width,
                class: "marking",
            });
        }
    };

    if centre {
        push_dashes(cut_dashes(line, CENTRE_DASH_M, CENTRE_GAP_M), priors::CENTRE_LINE_WIDTH_M);
    }
    // Dashed dividers at the n−1 interior lane boundaries of the carriageway.
    if lanes >= 2 {
        let frame = frame_at(line.0[0]);
        for k in 1..lanes {
            let off = -width_m * 0.5 + k as f64 * (width_m / lanes as f64);
            let boundary = if off.abs() < 1e-3 {
                Some(line.clone())
            } else {
                offset_line(line, off, &frame)
            };
            if let Some(b) = boundary {
                push_dashes(cut_dashes(&b, LANE_DASH_M, LANE_GAP_M), priors::CENTRE_LINE_WIDTH_M);
            }
        }
    }
    if edges {
        let inset = width_m * 0.5 - priors::EDGE_LINE_INSET_M;
        if inset > priors::EDGE_LINE_WIDTH_M {
            let frame = frame_at(line.0[0]);
            for side in [1.0, -1.0] {
                let Some(edge) = offset_line(line, inset * side, &frame) else {
                    continue;
                };
                let pieces: Vec<LineString> = trim_line(&edge, areas)
                    .into_iter()
                    .flat_map(|p| chords.cut(&p))
                    .filter(|p| line_len_m(p, &frame) >= MIN_LINE_M)
                    .collect();
                if !pieces.is_empty() {
                    out.push(Marking {
                        geometry: Geometry::MultiLineString(MultiLineString(pieces)),
                        width_m: priors::EDGE_LINE_WIDTH_M,
                        class: "marking",
                    });
                }
            }
        }
    }
    out
}

/// The zebra ladder of one registered crossing chord (docs/ROADS.md R7).
///
/// The chord runs kerb to kerb — the *walking* direction — and zebra stripes
/// are longitudinal to traffic, so a stripe sits every
/// [`priors::CROSSING_BAR_M`]-plus-gap along the chord. Each stripe is drawn
/// as a **transverse** line — [`priors::CROSSING_WIDTH_M`] long, along the
/// road — stroked at the bar width, *not* as a piece of the chord stroked to
/// the crossing depth: the client's stroke wears round caps that reach half
/// the stroke width past each end, so chord-wise dashes at a 2.8 m width grow
/// 1.4 m of cap into every gap and the ladder fuses into one slab. Stroked
/// this way the caps reach along the stripe's own length, where a rounded
/// stripe end is how the paint actually wears. The pattern is centred on the
/// chord — both kerbs get the same margin — and is a function of the chord
/// and its traffic direction alone (I5): every tile cuts identical bars.
///
/// `traffic` is the crossed street's own unit tangent (metric ENU) at the
/// crossing (`walkway::Chord`), and each stripe runs along it: real stripes
/// are longitudinal to traffic whatever the chord's obliquity, so an oblique
/// crossing draws a *sheared* ladder rather than one rotated against its
/// street — the drawn symptom `street.crossing_skew`'s faithful-obliquity
/// tail used to carry (38.9° at a curving side-roadway mouth). A degenerate
/// tangent — or `ARPT_NO_BAR_TRAFFIC`, the inertness A/B — falls back to
/// square-across the chord, the pre-R7 finish.
///
/// The shear is bounded at 45°. The stripes step [`priors::CROSSING_BAR_M`]
/// plus a gap *along the chord*, so their square-on spacing shrinks by the
/// shear's cosine — past 45° a ladder is on its way to a smear, and the worst
/// registered chord crosses its street 73° off square (a crosswalk mapped
/// nearly along a curving mouth), where stripes truly along traffic would
/// overlap themselves. Within the bound the stripe leans with traffic; at it,
/// the stripe holds 45° off square toward traffic's side.
///
/// Class `crossing`, not `marking`: colour is keyed by class, and the
/// calibrated `paint.*` populations filter on the literal `"marking"` — a
/// transverse ladder in them would poison the offset statistics.
pub fn crossing_bars(a: Coord, b: Coord, traffic: (f64, f64)) -> Vec<Marking> {
    let cos_lat = ((a.y + b.y) * 0.5).to_radians().cos().max(0.1);
    let m_lon = DEG_M * cos_lat;
    let (dx, dy) = ((b.x - a.x) * m_lon, (b.y - a.y) * DEG_M);
    let len = dx.hypot(dy);
    let period = priors::CROSSING_BAR_M + priors::CROSSING_GAP_M;
    let n = ((len + priors::CROSSING_GAP_M) / period).floor().max(0.0) as usize;
    if n == 0 || !(len > 0.0) {
        return Vec::new();
    }
    let pattern = n as f64 * priors::CROSSING_BAR_M + (n - 1) as f64 * priors::CROSSING_GAP_M;
    let start = (len - pattern) * 0.5;
    // Unit vectors in metres: along the chord for stripe placement, and the
    // stripe's own direction — traffic where the registration knows it.
    let (ux, uy) = (dx / len, dy / len);
    let tl = traffic.0.hypot(traffic.1);
    let (px, py) = if tl > 0.0 && std::env::var_os("ARPT_NO_BAR_TRAFFIC").is_none() {
        let (tx, ty) = (traffic.0 / tl, traffic.1 / tl);
        // The chord normal, oriented to traffic's side of the chord.
        let (mut nx, mut ny) = (-uy, ux);
        if nx * tx + ny * ty < 0.0 {
            (nx, ny) = (-nx, -ny);
        }
        let along = tx * ux + ty * uy; // sin of the shear, signed
        if along.abs() <= std::f64::consts::FRAC_1_SQRT_2 {
            (tx, ty)
        } else {
            let s = std::f64::consts::FRAC_1_SQRT_2.copysign(along);
            let c = std::f64::consts::FRAC_1_SQRT_2;
            (nx * c + ux * s, ny * c + uy * s)
        }
    } else {
        (-uy, ux)
    };
    let half = priors::CROSSING_WIDTH_M * 0.5;
    let bars: Vec<LineString> = (0..n)
        .map(|k| {
            let s = start + k as f64 * period + priors::CROSSING_BAR_M * 0.5;
            let c = (a.x + ux * s / m_lon, a.y + uy * s / DEG_M);
            LineString(vec![
                Coord { x: c.0 - px * half / m_lon, y: c.1 - py * half / DEG_M },
                Coord { x: c.0 + px * half / m_lon, y: c.1 + py * half / DEG_M },
            ])
        })
        .collect();
    vec![Marking {
        geometry: Geometry::MultiLineString(MultiLineString(bars)),
        width_m: priors::CROSSING_BAR_M,
        class: "crossing",
    }]
}

/// The registered crossing chords, indexed for the dash filter.
///
/// Longitudinal paint yields to the zebra: a centre line drawn through a
/// crosswalk is two painted systems contradicting each other, and the junction
/// areas `midpoint_paved` consults cannot say so — a crossing is not an area,
/// and most sit mid-leg where no area reaches. The chords are a phase-1
/// product (`synth::walkway::crossings`) and the dashes are cut per feature in
/// phase 2, so the index is built once on the world and every feature's
/// dashes are asked against every ladder near them, not just their own.
pub struct ChordIndex {
    grid: GridIndex,
    chords: Vec<(Coord, Coord)>,
}

impl ChordIndex {
    /// An index over no chords — nothing blocks. For worlds with no registered
    /// crossing, and for tests exercising the ladder alone.
    pub fn empty() -> ChordIndex {
        ChordIndex { grid: GridIndex::with_cell_m(64.0), chords: Vec::new() }
    }

    pub fn build<I: IntoIterator<Item = (Coord, Coord)>>(chords: I) -> ChordIndex {
        let mut idx = ChordIndex::empty();
        for (a, b) in chords {
            idx.grid.insert(
                (a.x.min(b.x), a.y.min(b.y), a.x.max(b.x), a.y.max(b.y)),
                idx.chords.len() as u32,
            );
            idx.chords.push((a, b));
        }
        idx
    }

    /// Whether the dash reaches inside a ladder's footprint plus the stop
    /// margin: within `CROSSING_WIDTH_M / 2 + CROSSING_STOP_M` of a chord.
    fn blocks(&self, dash: &LineString) -> bool {
        if self.chords.is_empty() || dash.0.len() < 2 {
            return false;
        }
        let reach = priors::CROSSING_WIDTH_M * 0.5 + CROSSING_STOP_M;
        let cosk = dash.0[0].y.to_radians().cos().max(0.1);
        let pad = reach / (DEG_M * cosk);
        let (mut w, mut s, mut e, mut n) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
        for c in &dash.0 {
            w = w.min(c.x);
            e = e.max(c.x);
            s = s.min(c.y);
            n = n.max(c.y);
        }
        let mut hits: Vec<u32> = Vec::new();
        self.grid.query((w - pad, s - pad, e + pad, n + pad), &mut hits);
        hits.iter().any(|&i| {
            let (a, b) = self.chords[i as usize];
            dash.0.windows(2).any(|d| seg_seg_m(d[0], d[1], a, b, cosk) < reach)
        })
    }

    /// The pieces of a solid line that lie clear of every ladder: the covered
    /// arc intervals are cut out, the way `trim_line` cuts junction areas. An
    /// edge line is interrupted across a crosswalk exactly as a centre line
    /// is — the difference is only that a solid line is cut where a dashed
    /// one is dropped whole.
    fn cut(&self, line: &LineString) -> Vec<LineString> {
        if self.chords.is_empty() || line.0.len() < 2 {
            return vec![line.clone()];
        }
        let reach = priors::CROSSING_WIDTH_M * 0.5 + CROSSING_STOP_M;
        let cosk = line.0[0].y.to_radians().cos().max(0.1);
        let pad = reach / (DEG_M * cosk);
        let (mut w, mut s, mut e, mut n) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
        for c in &line.0 {
            w = w.min(c.x);
            e = e.max(c.x);
            s = s.min(c.y);
            n = n.max(c.y);
        }
        let mut hits: Vec<u32> = Vec::new();
        self.grid.query((w - pad, s - pad, e + pad, n + pad), &mut hits);
        if hits.is_empty() {
            return vec![line.clone()];
        }
        // March the line and keep the maximal clear runs. A quarter metre
        // resolves the ladder edge well inside the stop margin.
        let arc = arc_lengths(&line.0);
        let total = *arc.last().expect("non-empty");
        let step = 0.25;
        let m = (total / step).ceil().max(1.0) as usize;
        let clear_at = |sm: f64| -> bool {
            let p = slice(&line.0, &arc, sm, sm)[0];
            let q = Coord { x: p.x + 1e-9, y: p.y };
            hits.iter().all(|&i| {
                let (a, b) = self.chords[i as usize];
                seg_seg_m(p, q, a, b, cosk) >= reach
            })
        };
        let mut out = Vec::new();
        let mut run: Option<f64> = None;
        for k in 0..=m {
            let sm = (k as f64 * step).min(total);
            match (clear_at(sm), run) {
                (true, None) => run = Some(sm),
                (false, Some(from)) => {
                    let piece = slice(&line.0, &arc, from, sm - step);
                    if piece.len() >= 2 {
                        out.push(LineString(piece));
                    }
                    run = None;
                }
                _ => {}
            }
        }
        if let Some(from) = run {
            let piece = slice(&line.0, &arc, from, total);
            if piece.len() >= 2 {
                out.push(LineString(piece));
            }
        }
        out
    }
}

/// Plan distance between two segments in metres (equirectangular at `cosk`),
/// zero where they cross.
fn seg_seg_m(a0: Coord, a1: Coord, b0: Coord, b1: Coord, cosk: f64) -> f64 {
    let m = |c: Coord| (c.x * DEG_M * cosk, c.y * DEG_M);
    let (a0, a1, b0, b1) = (m(a0), m(a1), m(b0), m(b1));
    let cross = |o: (f64, f64), p: (f64, f64), q: (f64, f64)| {
        (p.0 - o.0) * (q.1 - o.1) - (p.1 - o.1) * (q.0 - o.0)
    };
    if cross(a0, a1, b0) * cross(a0, a1, b1) < 0.0 && cross(b0, b1, a0) * cross(b0, b1, a1) < 0.0 {
        return 0.0;
    }
    let pt = |p: (f64, f64), a: (f64, f64), b: (f64, f64)| {
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let l2 = dx * dx + dy * dy;
        let t = if l2 > 0.0 { (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / l2).clamp(0.0, 1.0) } else { 0.0 };
        (p.0 - a.0 - t * dx).hypot(p.1 - a.1 - t * dy)
    };
    pt(a0, b0, b1).min(pt(a1, b0, b1)).min(pt(b0, a0, a1)).min(pt(b1, a0, a1))
}

/// A local ENU frame at a point (marking generation runs before any tile
/// exists, so there are no tile bounds to centre one on).
fn frame_at(c: Coord) -> Frame {
    frame_for(c.x, c.y)
}

fn frame_for(lon: f64, lat: f64) -> Frame {
    let b = Bounds { west: lon, south: lat, east: lon, north: lat };
    Frame::at_center(&b)
}

/// Cuts the line into dashes of `dash_m` every `dash_m + gap_m`, phase
/// anchored at the line's start (a global arclength origin — see the module
/// doc). Dashes clipped by the line's end shorter than half a dash are
/// dropped rather than painted as crumbs.
fn cut_dashes(line: &LineString, dash_m: f64, gap_m: f64) -> Vec<LineString> {
    let pts = &line.0;
    let arc = arc_lengths(pts);
    let total = *arc.last().expect("non-empty");
    let period = dash_m + gap_m;
    let mut out = Vec::new();
    let mut s = 0.0;
    while s < total {
        let e = (s + dash_m).min(total);
        if e - s >= dash_m * 0.5 {
            let piece = slice(pts, &arc, s, e);
            if piece.len() >= 2 {
                out.push(LineString(piece));
            }
        }
        s += period;
    }
    out
}

/// Whether a dash's midpoint falls on a paved intersection — no road marking
/// is painted through one.
fn midpoint_paved(dash: &LineString, areas: &[&Area]) -> bool {
    let pts = &dash.0;
    let arc = arc_lengths(pts);
    let mid = slice(pts, &arc, arc.last().copied().unwrap_or(0.0) * 0.5, f64::INFINITY);
    let Some(&m) = mid.first() else {
        return false;
    };
    areas.iter().any(|a| a.contains(m))
}

/// Cumulative arclength in metres at each vertex.
fn arc_lengths(pts: &[Coord]) -> Vec<f64> {
    let cosk = pts[0].y.to_radians().cos();
    let mut arc = Vec::with_capacity(pts.len());
    arc.push(0.0);
    for w in pts.windows(2) {
        let de = (w[1].x - w[0].x) * DEG_M * cosk;
        let dn = (w[1].y - w[0].y) * DEG_M;
        arc.push(arc.last().expect("non-empty") + (de * de + dn * dn).sqrt());
    }
    arc
}

/// The coordinates covering arclength `[a, b]`, interpolating the endpoints
/// and keeping the original vertices between them.
fn slice(pts: &[Coord], arc: &[f64], a: f64, b: f64) -> Vec<Coord> {
    let total = *arc.last().expect("non-empty");
    let (a, b) = (a.clamp(0.0, total), b.clamp(0.0, total));
    let at = |s: f64| -> Coord {
        let i = match arc.binary_search_by(|v| v.total_cmp(&s)) {
            Ok(i) => i,
            Err(i) => i - 1,
        };
        if i >= pts.len() - 1 {
            return pts[pts.len() - 1];
        }
        let len = arc[i + 1] - arc[i];
        let t = if len > 0.0 { ((s - arc[i]) / len).clamp(0.0, 1.0) } else { 0.0 };
        Coord {
            x: pts[i].x + (pts[i + 1].x - pts[i].x) * t,
            y: pts[i].y + (pts[i + 1].y - pts[i].y) * t,
        }
    };
    let mut out = vec![at(a)];
    for (i, &s) in arc.iter().enumerate() {
        if s > a && s < b {
            out.push(pts[i]);
        }
    }
    out.push(at(b));
    out.dedup_by(|p, q| (p.x - q.x).abs() < 1e-12 && (p.y - q.y).abs() < 1e-12);
    out
}

/// A line's length in metres (local equirectangular scale).
fn line_len_m(line: &LineString, frame: &Frame) -> f64 {
    line.0
        .windows(2)
        .map(|w| {
            let de = (w[1].x - w[0].x) * frame.m_per_deg_lon;
            let dn = (w[1].y - w[0].y) * DEG_M;
            (de * de + dn * dn).sqrt()
        })
        .sum()
}

/// The two rail heads of a track: the centerline offset ±half the class
/// gauge, each drawn as a thin dark line riding the ballast band the way
/// road paint rides the asphalt (docs/ROADS.md P3, extended to rail).
///
/// No dashes and no intersection trim, on purpose: a rail is continuous
/// steel, and at a level crossing it runs *through* the asphalt — trimming
/// it at the crossing's area would break exactly the place a rail is most
/// visible. The lateral offset survives the paint snap because
/// `Profile::smooth_at` re-applies the query point's offset about the raw
/// edge onto the smoothed curve (the two-curves fix; guarded by
/// `paint.edge_line_inset` for road paint).
pub fn rails_for_line(line: &LineString, gauge_m: f64) -> Vec<Marking> {
    if line.0.len() < 2 {
        return Vec::new();
    }
    let frame = frame_at(line.0[0]);
    [0.5, -0.5]
        .into_iter()
        .filter_map(|side| offset_line(line, gauge_m * side, &frame))
        .map(|rail| Marking {
            geometry: Geometry::LineString(rail),
            width_m: priors::RAIL_HEAD_WIDTH_M,
            class: "rail_line",
        })
        .collect()
}

/// The centerline offset sideways by `offset_m` (positive = left of travel),
/// with the same averaged-perpendicular miter the surface band uses, or
/// `None` for a degenerate line.
fn offset_line(line: &LineString, offset_m: f64, frame: &Frame) -> Option<LineString> {
    let pts = &line.0;
    if pts.len() < 2 {
        return None;
    }
    let m_lon = frame.m_per_deg_lon;
    let dir = |a: Coord, b: Coord| -> Option<(f64, f64)> {
        let (de, dn) = ((b.x - a.x) * m_lon, (b.y - a.y) * DEG_M);
        let len = (de * de + dn * dn).sqrt();
        (len > 1e-9).then(|| (de / len, dn / len))
    };
    let mut out = Vec::with_capacity(pts.len());
    for i in 0..pts.len() {
        let before = (i > 0).then(|| dir(pts[i - 1], pts[i])).flatten();
        let after = (i + 1 < pts.len()).then(|| dir(pts[i], pts[i + 1])).flatten();
        let (e, n, scale) = match (before, after) {
            (Some((e0, n0)), Some((e1, n1))) => {
                let (se, sn) = (e0 + e1, n0 + n1);
                let len = (se * se + sn * sn).sqrt();
                if len < 1e-9 {
                    continue;
                }
                (se / len, sn / len, (1.0 / (len * 0.5).min(1.0)).min(MITER_MAX))
            }
            (Some(d), None) | (None, Some(d)) => (d.0, d.1, 1.0),
            (None, None) => continue,
        };
        let (pe, pn) = (-n, e);
        let reach = offset_m * scale;
        out.push(Coord {
            x: pts[i].x + pe * reach / m_lon,
            y: pts[i].y + pn * reach / DEG_M,
        });
    }
    (out.len() >= 2).then(|| LineString(out))
}

/// Cuts the parts of a painted line that fall inside any intersection area,
/// returning the pieces that remain — the whole line when no area touches it.
/// Handles a line *ending* at an intersection and one passing through it alike,
/// and a line that leaves an area and re-enters it.
///
/// Longitudinal paint stops *at* the intersection, so the cut is exact: the
/// half-metre tuck this carried when it also trimmed surface bands existed only
/// to hide a band's end under a plate, and there are no plates now. Short
/// survivors are the caller's business — the marking ladder already drops stubs
/// under [`MIN_LINE_M`].
fn trim_line(line: &LineString, areas: &[&Area]) -> Vec<LineString> {
    let pts = &line.0;
    if pts.len() < 2 {
        return Vec::new();
    }
    // Cumulative arclength in metres (local equirectangular scale).
    let cosk = pts[0].y.to_radians().cos();
    let en = |from: Coord, to: Coord| {
        ((to.x - from.x) * DEG_M * cosk, (to.y - from.y) * DEG_M)
    };
    let mut arc = Vec::with_capacity(pts.len());
    arc.push(0.0);
    for w in pts.windows(2) {
        let (de, dn) = en(w[0], w[1]);
        arc.push(arc.last().expect("non-empty") + (de * de + dn * dn).sqrt());
    }
    let total = *arc.last().expect("non-empty");
    if total <= 0.0 {
        return Vec::new();
    }

    // The arc intervals the areas cover: per segment, clip the chord against
    // each area's leg rectangles.
    let mut cut: Vec<(f64, f64)> = Vec::new();
    let mut chord: Vec<(f64, f64)> = Vec::new();
    for area in areas {
        for (i, w) in pts.windows(2).enumerate() {
            chord.clear();
            area.clip_chord(w[0], w[1], &mut chord);
            let len = arc[i + 1] - arc[i];
            for &(t0, t1) in &chord {
                if t1 > t0 {
                    cut.push((arc[i] + t0 * len, arc[i] + t1 * len));
                }
            }
        }
    }
    if cut.is_empty() {
        return vec![line.clone()];
    }
    cut.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut merged: Vec<(f64, f64)> = Vec::new();
    for iv in cut {
        match merged.last_mut() {
            Some(last) if iv.0 <= last.1 => last.1 = last.1.max(iv.1),
            _ => merged.push(iv),
        }
    }
    // Walk the complement, rebuilding each kept span's coordinates.
    let mut keep: Vec<(f64, f64)> = Vec::new();
    let mut s = 0.0;
    for &(a, b) in &merged {
        if a > s {
            keep.push((s, a));
        }
        s = s.max(b);
    }
    if total > s {
        keep.push((s, total));
    }
    keep.iter()
        .map(|&(a, b)| LineString(slice_arc(pts, &arc, a, b)))
        .filter(|l| l.0.len() >= 2)
        .collect()
}

/// The coordinates of the sub-line covering arclength `[a, b]`, interpolating
/// the cut endpoints and keeping the original vertices between them.
fn slice_arc(pts: &[Coord], arc: &[f64], a: f64, b: f64) -> Vec<Coord> {
    let at = |s: f64| -> Coord {
        let i = match arc.binary_search_by(|v| v.total_cmp(&s)) {
            Ok(i) => i,
            Err(i) => i - 1, // arc[0] = 0 ≤ s, so i ≥ 1 here
        };
        if i >= pts.len() - 1 {
            return pts[pts.len() - 1];
        }
        let len = arc[i + 1] - arc[i];
        let t = if len > 0.0 { ((s - arc[i]) / len).clamp(0.0, 1.0) } else { 0.0 };
        Coord {
            x: pts[i].x + (pts[i + 1].x - pts[i].x) * t,
            y: pts[i].y + (pts[i + 1].y - pts[i].y) * t,
        }
    };
    let mut out = vec![at(a)];
    for (i, &s) in arc.iter().enumerate() {
        if s > a && s < b {
            out.push(pts[i]);
        }
    }
    out.push(at(b));
    // A cut landing exactly on a vertex would duplicate it.
    out.dedup_by(|p, q| (p.x - q.x).abs() < 1e-12 && (p.y - q.y).abs() < 1e-12);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn centre_dashes_yield_to_a_crossing_chord() {
        // A 100 m west→east secondary with a crossing chord across it at
        // x = +25 m: the dash whose run covers that station is dropped, the
        // rest of the ladder survives, and an empty index drops nothing.
        let cy: f64 = 46.0;
        let m_lon = DEG_M * cy.to_radians().cos();
        let at = |de: f64, dn: f64| Coord { x: 7.0 + de / m_lon, y: cy + dn / DEG_M };
        let line = LineString(vec![at(0.0, 0.0), at(100.0, 0.0)]);
        let clear = for_line(&line, "secondary", false, 6.0, &[], &ChordIndex::empty());
        assert_eq!(clear.len(), 1);
        let Geometry::MultiLineString(all) = &clear[0].geometry else { panic!("dashes") };

        let chord = ChordIndex::build([(at(25.0, -4.0), at(25.0, 4.0))]);
        let out = for_line(&line, "secondary", false, 6.0, &[], &chord);
        assert_eq!(out.len(), 1);
        let Geometry::MultiLineString(kept) = &out[0].geometry else { panic!("dashes") };
        assert!(kept.0.len() < all.0.len(), "a dash under the ladder is dropped");
        let reach = priors::CROSSING_WIDTH_M * 0.5 + CROSSING_STOP_M;
        for d in &kept.0 {
            for v in &d.0 {
                let de = ((v.x - 7.0) * m_lon - 25.0).abs();
                assert!(de > reach - 1e-6, "kept paint {de:.2} m from the chord");
            }
        }
        // A chord a street away blocks nothing.
        let far = ChordIndex::build([(at(25.0, 20.0), at(25.0, 28.0))]);
        let unharmed = for_line(&line, "secondary", false, 6.0, &[], &far);
        let Geometry::MultiLineString(u) = &unharmed[0].geometry else { panic!("dashes") };
        assert_eq!(u.0.len(), all.0.len());
    }

    #[test]
    fn edge_lines_are_cut_at_a_crossing_chord() {
        // A motorway's solid edge lines split at a chord across the road; a
        // clear run of the same road keeps one piece per side.
        let cy: f64 = 46.0;
        let m_lon = DEG_M * cy.to_radians().cos();
        let at = |de: f64, dn: f64| Coord { x: 7.0 + de / m_lon, y: cy + dn / DEG_M };
        let line = LineString(vec![at(0.0, 0.0), at(200.0, 0.0)]);
        let solid = |ms: &[Marking]| -> usize {
            ms.iter()
                .filter(|m| (m.width_m - priors::EDGE_LINE_WIDTH_M).abs() < 1e-6)
                .map(|m| match &m.geometry {
                    Geometry::MultiLineString(p) => p.0.len(),
                    _ => 0,
                })
                .sum()
        };
        let clear = for_line(&line, "motorway", true, 9.0, &[], &ChordIndex::empty());
        let chord = ChordIndex::build([(at(100.0, -6.0), at(100.0, 6.0))]);
        let cut = for_line(&line, "motorway", true, 9.0, &[], &chord);
        assert_eq!(solid(&cut), solid(&clear) * 2, "each edge line splits in two");
    }

    fn straight(len_m: f64) -> LineString {
        let cy: f64 = 46.0;
        let m_lon = DEG_M * cy.to_radians().cos();
        LineString(vec![
            Coord { x: 7.0, y: cy },
            Coord { x: 7.0 + len_m / m_lon, y: cy },
        ])
    }

    #[test]
    fn dashes_cut_on_the_global_period() {
        // 50 m at 4/6: dashes at [0,4], [10,14], [20,24], [30,34], [40,44].
        let dashes = cut_dashes(&straight(50.0), 4.0, 6.0);
        assert_eq!(dashes.len(), 5);
        let cy: f64 = 46.0;
        let m_lon = DEG_M * cy.to_radians().cos();
        for (k, d) in dashes.iter().enumerate() {
            let s = (d.0.first().expect("start").x - 7.0) * m_lon;
            let e = (d.0.last().expect("end").x - 7.0) * m_lon;
            assert!((s - 10.0 * k as f64).abs() < 0.01, "dash {k} starts at {s}");
            assert!((e - s - 4.0).abs() < 0.01, "dash {k} length {}", e - s);
        }
        // A tail crumb shorter than half a dash is dropped: 42 m ends mid-gap
        // for the 5th dash window at [40, 42] → kept (2 m = dash/2)…
        assert_eq!(cut_dashes(&straight(42.0), 4.0, 6.0).len(), 5);
        // …but 41 m leaves only 1 m of it → dropped.
        assert_eq!(cut_dashes(&straight(41.0), 4.0, 6.0).len(), 4);
    }

    #[test]
    fn centre_line_is_dashed_and_edge_lines_are_solid() {
        let line = straight(100.0);
        let centre = for_line(&line, "secondary", false, 6.0, &[], &ChordIndex::empty());
        assert_eq!(centre.len(), 1);
        assert!(matches!(&centre[0].geometry, Geometry::MultiLineString(m) if m.0.len() == 10));
        assert_eq!(centre[0].width_m, priors::CENTRE_LINE_WIDTH_M);
        // A one-way secondary of one lane's width paints nothing.
        assert!(for_line(&line, "secondary", true, 3.5, &[], &ChordIndex::empty()).is_empty());
        // A two-lane motorway carriageway paints one dashed lane divider
        // (down its middle) plus two solid edge lines — no centre line.
        let mw = for_line(&line, "motorway", true, 9.0, &[], &ChordIndex::empty());
        assert_eq!(mw.len(), 3);
        let dashed = mw
            .iter()
            .filter(|m| matches!(&m.geometry, Geometry::MultiLineString(g) if g.0.len() > 1))
            .count();
        assert_eq!(dashed, 1, "one dashed divider");
        let solid = mw.iter().filter(|m| m.width_m == priors::EDGE_LINE_WIDTH_M).count();
        assert_eq!(solid, 2, "two solid edge lines");
        // A wide (three-lane) carriageway paints two dividers.
        let wide = for_line(&line, "motorway", true, 12.0, &[], &ChordIndex::empty());
        assert_eq!(wide.len(), 4, "two dividers + two edges");
    }

    #[test]
    fn lane_count_infers_back_from_the_width() {
        assert_eq!(priors::lane_count("motorway", 9.0), 2);
        assert_eq!(priors::lane_count("motorway", 12.5), 3);
        assert_eq!(priors::lane_count("primary", 7.0), 2);
        assert_eq!(priors::lane_count("primary", 3.5), 1);
        // Untagged motorways still divide: one-way by construction.
        assert!(priors::has_lane_lines("motorway", false));
        assert!(!priors::has_lane_lines("primary", false));
        assert!(priors::has_lane_lines("primary", true));
    }

    #[test]
    fn dashes_inside_an_intersection_are_dropped() {
        let line = straight(50.0);
        let cy: f64 = 46.0;
        let m_lon = DEG_M * cy.to_radians().cos();
        // A crossroads over the middle: the [20,24] dash (mid 22) is on it.
        let centre = Coord { x: 7.0 + 22.0 / m_lon, y: cy };
        let legs = vec![
            crate::synth::area::Leg { e: 1.0, n: 0.0, half_w: 5.0 },
            crate::synth::area::Leg { e: -1.0, n: 0.0, half_w: 5.0 },
            crate::synth::area::Leg { e: 0.0, n: 1.0, half_w: 5.0 },
            crate::synth::area::Leg { e: 0.0, n: -1.0, half_w: 5.0 },
        ];
        let area = Area::new(centre, legs, 5.0).expect("an intersection");
        let out = for_line(&line, "secondary", false, 6.0, &[&area], &ChordIndex::empty());
        assert_eq!(out.len(), 1);
        let Geometry::MultiLineString(m) = &out[0].geometry else {
            panic!("a multiline of dashes");
        };
        assert_eq!(m.0.len(), 4, "the covered dash is dropped");
    }

    /// A square intersection of half-extent `half_m` centred on `c` — four
    /// equal legs at the compass points, the shape the trim tests cut against.
    fn square_area(c: Coord, half_m: f64) -> Area {
        let legs = vec![
            crate::synth::area::Leg { e: 1.0, n: 0.0, half_w: half_m },
            crate::synth::area::Leg { e: -1.0, n: 0.0, half_w: half_m },
            crate::synth::area::Leg { e: 0.0, n: 1.0, half_w: half_m },
            crate::synth::area::Leg { e: 0.0, n: -1.0, half_w: half_m },
        ];
        Area::new(c, legs, half_m).expect("a square area")
    }

    /// R7's finish: stripes are longitudinal to traffic whatever the chord's
    /// obliquity — an oblique crossing is a sheared ladder, not a rotated one.
    #[test]
    fn zebra_stripes_run_with_traffic_not_square_to_the_chord() {
        let cy: f64 = 46.0;
        let m_lon = DEG_M * cy.to_radians().cos();
        let at = |de: f64, dn: f64| Coord { x: 7.0 + de / m_lon, y: cy + dn / DEG_M };
        // A chord crossing a west→east street at ~34° off square.
        let (a, b) = (at(0.0, -4.0), at(5.5, 4.0));
        let east = (1.0, 0.0);
        let out = crossing_bars(a, b, east);
        assert_eq!(out.len(), 1);
        let Geometry::MultiLineString(bars) = &out[0].geometry else { panic!("bars") };
        assert!(bars.0.len() >= 2, "a ladder, not a stripe");
        for bar in &bars.0 {
            let (p, q) = (bar.0[0], bar.0[1]);
            let (dx, dy) = ((q.x - p.x) * m_lon, (q.y - p.y) * DEG_M);
            let len = dx.hypot(dy);
            assert!((len - priors::CROSSING_WIDTH_M).abs() < 0.02, "stripe length {len}");
            assert!(
                dy.abs() / len < 1e-6,
                "a stripe runs {:.1}° off traffic",
                (dy / dx).atan().to_degrees()
            );
        }
        // Past 45° of shear the stripe holds the bound: a chord crossing its
        // street 73° off square gets 45°-leaning stripes, not a smear of
        // near-chord-parallel ones.
        let (a2, b2) = (at(20.0, -1.5), at(28.0, 1.5)); // ~69° off square vs east
        let out = crossing_bars(a2, b2, east);
        let Geometry::MultiLineString(bars) = &out[0].geometry else { panic!("bars") };
        let (cx2, cy2) = ((b2.x - a2.x) * m_lon, (b2.y - a2.y) * DEG_M);
        let clen2 = cx2.hypot(cy2);
        for bar in &bars.0 {
            let (p, q) = (bar.0[0], bar.0[1]);
            let (dx, dy) = ((q.x - p.x) * m_lon, (q.y - p.y) * DEG_M);
            let cosn = ((dx * cx2 + dy * cy2) / (dx.hypot(dy) * clen2)).abs();
            assert!(
                (cosn - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-6,
                "sheared stripe should hold 45° off the chord, reads cos {cosn:.3}"
            );
        }

        // A degenerate tangent falls back to square-across the chord — the
        // pre-R7 finish, and what ARPT_NO_BAR_TRAFFIC restores wholesale.
        let out = crossing_bars(a, b, (0.0, 0.0));
        let Geometry::MultiLineString(bars) = &out[0].geometry else { panic!("bars") };
        let (cx, cyv) = ((b.x - a.x) * m_lon, (b.y - a.y) * DEG_M);
        let clen = cx.hypot(cyv);
        for bar in &bars.0 {
            let (p, q) = (bar.0[0], bar.0[1]);
            let (dx, dy) = ((q.x - p.x) * m_lon, (q.y - p.y) * DEG_M);
            let dot = (dx * cx + dy * cyv) / (dx.hypot(dy) * clen);
            assert!(dot.abs() < 1e-6, "fallback stripe not square to the chord");
        }
    }

    #[test]
    fn trim_splits_a_through_line_at_the_intersection() {
        // A 200 m west→east line through a 10 m intersection at its middle:
        // two pieces, each ending at the intersection edge (tucked under it).
        let cy: f64 = 46.0;
        let m_lon = DEG_M * cy.to_radians().cos();
        let c = Coord { x: 7.0, y: cy };
        let line = LineString(vec![
            Coord { x: c.x - 100.0 / m_lon, y: cy },
            Coord { x: c.x + 100.0 / m_lon, y: cy },
        ]);
        let area = square_area(c, 10.0);
        let pieces = trim_line(&line, &[&area]);
        assert_eq!(pieces.len(), 2, "the intersection splits the line");
        for p in &pieces {
            for v in &p.0 {
                let d = ((v.x - c.x) * m_lon).abs();
                // Up to the boundary less the tuck, never past it.
                assert!(d > 10.0 - 0.1, "piece vertex {d:.2} m inside the plate");
            }
        }
        // An end-of-line junction shortens rather than splits.
        let at_end = square_area(line.0[1], 10.0);
        let end = trim_line(&line, &[&at_end]);
        assert_eq!(end.len(), 1);
        let last = end[0].0.last().expect("non-empty");
        assert!(((last.x - line.0[1].x) * m_lon).abs() > 10.0 - 0.1);
        // No intersections → the whole line; one swallowing it → nothing.
        assert_eq!(trim_line(&line, &[]).len(), 1);
        let huge = square_area(c, 150.0);
        assert!(trim_line(&line, &[&huge]).is_empty());
    }

    #[test]
    fn trim_survives_a_line_that_leaves_and_re_enters() {
        // A line clipping the north arm of a cross, dipping out over the
        // corner notch and back in — two cuts, one kept piece between them.
        // The circular trim this replaced could only ever make one cut.
        let cy: f64 = 46.0;
        let m_lon = DEG_M * cy.to_radians().cos();
        let c = Coord { x: 7.0, y: cy };
        let legs = vec![
            crate::synth::area::Leg { e: 1.0, n: 0.0, half_w: 4.0 },
            crate::synth::area::Leg { e: -1.0, n: 0.0, half_w: 4.0 },
            crate::synth::area::Leg { e: 0.0, n: 1.0, half_w: 4.0 },
        ];
        // Reach 20 m: the three arms stick well out of the 8 m core, so a
        // line across them at 12 m north is inside, outside, inside.
        let area = Area::new(c, legs, 20.0).expect("a tee");
        let at = |de: f64, dn: f64| Coord { x: c.x + de / m_lon, y: c.y + dn / DEG_M };
        let line = LineString(vec![at(-30.0, 12.0), at(30.0, 12.0)]);
        let pieces = trim_line(&line, &[&area]);
        assert_eq!(pieces.len(), 2, "west and east of the north arm survive");
        for p in &pieces {
            for v in &p.0 {
                let de = (v.x - c.x) * m_lon;
                assert!(de.abs() > 4.0 - 0.1, "vertex {de:.2} m inside the arm");
            }
        }
    }
}
