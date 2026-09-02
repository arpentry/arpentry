//! The band a pedestrian way is drawn as (docs/ROADS.md invariant 1).
//!
//! A footway used to be a stroke and nothing else — `Surface::None`, no
//! corridor, no width to buffer — so at the detail zooms where every road
//! became a meshed surface a sidewalk stayed a cartographic line painted over
//! whatever it happened to cross. Here it becomes a surface like the rest: a
//! [`priors::Surface::Walkway`] band that joins the union, takes its own hole
//! out of the drawn ground, and wears an apron where it stands above it.
//!
//! **Two kinds of band, one material.**
//!
//! - *Attached* — the way runs alongside a street (`assemble::walks`). Its
//!   shape comes from the **host's** smoothed centerline, offset laterally, not
//!   from its own mapped polyline: that is what makes a sidewalk parallel to
//!   its kerb, constant in width, and free of the wobble a hand-drawn footway
//!   carries. The polyline decides *where a sidewalk is* and *which side*;
//!   it never decides what shape one has. Its height is the host's road
//!   surface plus [`priors::KERB_RISE_M`].
//! - *Unattached* — the rest of every pedestrian line, which is a path across
//!   open ground. It is buffered along its own polyline and stands on the
//!   ground, because there is nothing else for it to stand on.
//!
//! The two meet along one way with no seam: they are segments of one
//! `Surface::Walkway` run, the union merges them, and the height field blends
//! the kerb rise out over a band's width where one becomes the other — a
//! dropped kerb, which is what is really there.
//!
//! **The room is spent in order.** A band takes what is left between the drawn
//! kerb and [`priors::FACADE_CLEAR_M`] short of the facade, up to
//! [`priors::WALK_WIDTH_M`]. Where less than [`priors::WALK_MIN_WIDTH_M`] is
//! left there is no band, which is what a street too narrow for a sidewalk
//! looks like. That is the same room `synth::carriageway` allotted the asphalt
//! out of, read through the same `Room` (docs/ROADS.md invariant 1).

use geo_types::Coord;

use crate::assemble::facades::{Facades, Section};
use crate::priors::{self, Kind, RoadClass};
use crate::scene::{Corridor, CorridorId, SceneGraph, SpanKind, DEG_M};
use crate::solve::SolvedModel;

use std::collections::HashMap;

use super::carriageway::{level_runs, SourceSeg, RUN_EPS_M};

/// The corridor id a band with no host carries. Nothing resolves it to a
/// profile (`SolvedModel::profile` answers `None` past the end of its list),
/// which is exactly right: a path across a field has no road to ride, so its
/// height is the ground.
pub(crate) const NO_HOST: CorridorId = CorridorId::MAX;

/// Station spacing along an unattached path, in metres. The band follows the
/// way's own mapped line there, so this only bounds how coarsely a long
/// straight is buffered; an attached band is stationed by its host's profile
/// instead, like the carriageway.
const PATH_STATION_M: f64 = 8.0;

/// Largest ground step a hostless Walkway segment may span end to end, in
/// metres, before it is a wall rather than a link ([`fitted_half`]).
///
/// The only hostless Walkway left is a crossing's kerb stub — a corner is now
/// inside the strip its two claims merged into — and a stub is connective
/// tissue between a chord and a pavement. Where the ground jumps more than a
/// storey within one segment it is not bridging a kerb, it is draping across
/// the bench cliff between a switchback's two arms (6.9166,46.4338 is the type
/// specimen). Higher than any kerb or corner ramp, lower than the arm
/// separation of the shallowest switchback the extract holds (~5 m).
const CORNER_STEP_M: f64 = 1.5;

/// Lateral sampling pitch of the bench-face probe, metres — fine enough that
/// a wall inside the band's width cannot hide between two samples.
const FACE_STEP_M: f64 = 0.75;

/// Longest hostless Walkway segment [`CORNER_STEP_M`] applies to, in metres —
/// what counts as connective tissue rather than a way going somewhere.
const CORNER_LINK_MAX_M: f64 = 4.0;

/// Steepest end-to-end seat fall a hostless Walkway segment may carry, as a
/// grade — past it the segment is draping a wall, whatever its length.
///
/// [`CORNER_STEP_M`]'s length exemption was written so a sidewalk leaving its
/// street could legitimately fall a storey over a hundred metres — and it
/// exempted the type specimen with it: an 8 m segment at the trench mouth of
/// 6.8932,46.4435 whose stamped seat falls 5.6 m (a 70 % walkway, drawn as a
/// near-vertical curtain between two terraces; `slope.walk_crossfall`'s
/// worst, 908 %). The allowance therefore grows with length at this grade
/// instead of vanishing: a link keeps the absolute step, a longer segment is
/// allowed `len ×` this, and nothing keeps a wall.
///
/// Read off the census (`ARPT_DEBUG_WALK`, Montreux zone, 7,531 hostless
/// Walkway segments): p50 0.039, p90 0.182, p99 0.491, max 3.567. The
/// steepest sidewalks anywhere are ~0.3; past 0.5 is stair territory, and a
/// `steps` way is excluded from Walkway bands by design (it draws as a free
/// band until P5 gives it a profile). The ceiling refuses 215 m over the
/// whole zone — 0.05 % of the drawn sidewalk length — all of it wall.
const WALK_WALL_GRADE: f64 = 0.5;

/// How far a crossing's mapped line is extended past each end when it is
/// registered against the carriageways, in metres. A crosswalk is mapped by
/// hand across the road it crosses and its endpoints rarely lie on the kerbs
/// (docs/ROADS.md §2); the extension lets the registration find the asphalt
/// the mapped stub stops short of. It bounds search, not paint — the painted
/// extent is exactly the on-asphalt interval, wherever the mapped ends are.
pub(crate) const CROSSING_EXTEND_M: f64 = 8.0;

/// Registration sampling step along a crossing line, in metres — fine enough
/// that a kerb lands within a step of where the buffer test says.
const CROSSING_STEP_M: f64 = 0.25;

/// Two on-asphalt intervals closer than this merge, in metres: a lane
/// divider's worth of noise is not a refuge island.
pub(crate) const CROSSING_MERGE_M: f64 = 1.0;

/// Shortest on-asphalt interval that counts as a crossed carriageway, in
/// metres — under it the line grazed an edge rather than crossed a road.
const CROSSING_MIN_CHORD_M: f64 = 1.5;

/// Shortest kerb stub worth a band, in metres. Under it the strip is inside
/// the quantization of the kerb itself.
const CROSSING_MIN_STUB_M: f64 = 0.4;

/// How far a carriageway's solved height may lie from the crossing's own
/// ground level and still be the street this at-grade crosswalk crosses, in
/// metres.
///
/// Registration used to be plan-only, and Territet showed what that admits: a
/// crosswalk on a terrace plan-crosses the avenue 13 m below it, a hairpin's
/// upper arm lies 8 m over the lower arm the crossing was mapped on, and an
/// underpass runs 4–6 m under the surface street at its portal — each one
/// "asphalt under the line" to a plan test, and each one annexed into the
/// chord (a 20 m ladder climbing 6.4 m over the portal at 6.9097,46.4376;
/// `street.crossing_skew` reading 83–89° against streets the crossing never
/// touches). Three metres passes a kerb, benching and a clearance-lifted
/// approach on one street, and rejects every measured stacked-street
/// separation, which starts at four.
pub(crate) const CROSSING_LEVEL_M: f64 = 3.0;

/// The smallest incidence at which a proper plan intersection counts as
/// *crossing* a street, as the sine of the angle between the crossing line
/// and the street's local tangent (0.342 = 20°). Real crosswalks cross at
/// 60–90°; a mapped skew survives to ~40°. What this rejects is the
/// geometric accident: at a hairpin the extended line grazes the far arm
/// nearly parallel — a proper intersection by orientation signs, and a
/// street the pedestrian never crosses. Measured before the gate: a 22 m
/// ladder around the bend at 6.9118,46.4387, `street.crossing_skew` 88.8°.
pub(crate) const CROSSING_MIN_INCIDENCE_SIN: f64 = 0.342;

/// How far along a crossed street's own arc its asphalt answers the chord
/// march, in metres each way from the crossing point. A crossing crosses one
/// street at one station; the host is a whole spliced corridor, so without
/// this window the asphalt of the same street two bends away — a hairpin's
/// other arm, 80 m by road and 5 m in plan — answers as readily as the
/// asphalt underfoot. Wide enough for the widest junction mouth a chord may
/// legitimately traverse.
pub(crate) const CROSSING_ARC_WINDOW_M: f64 = 25.0;

/// One crosswalk, registered: the kerb-to-kerb chords its paint spans. The
/// stubs — the stretches of the *mapped* line outside every chord — are
/// returned by [`crossings`] as ordinary band segments instead, because they
/// are the strip of real sidewalk between the kerb and whatever the crossing
/// joins.
/// One kerb-to-kerb chord of a registered crossing, with the direction
/// traffic runs where it is crossed.
#[derive(Clone, Copy)]
pub struct Chord {
    pub a: Coord,
    pub b: Coord,
    /// Unit tangent (metric ENU) of the crossed centerline nearest the
    /// chord's midpoint — the direction a zebra's stripes run
    /// (docs/ROADS.md R7): stripes are longitudinal to traffic whatever the
    /// chord's obliquity, so an oblique crossing is a sheared ladder, not a
    /// rotated one. Falls back to square-across the chord where the hosts
    /// offer no tangent, which is the pre-R7 finish.
    pub traffic: (f64, f64),
}

pub struct CrossingPaint {
    /// Source hash of the crosswalk feature, for the phase-1 lookup.
    pub source: u64,
    /// Kerb-to-kerb chords over drawn asphalt, in walk order along the line.
    /// More than one where the crossing spans a divided carriageway: the gap
    /// between them is the refuge island, and it is not painted.
    pub chords: Vec<Chord>,
}

/// Registers every crosswalk line against the carriageways: the paint chords
/// per crossing, and the kerb stubs as Walkway band segments.
///
/// **One derivation, two readers** (the codebase's own sliver lesson): the
/// interval a crossing lies on asphalt is computed once, here; the paint
/// ladder is its inside and the stub bands are its outside, so the two meet
/// at the kerb by construction. Registration is against the corridors' raw
/// centerlines and drawn half-widths — the same cross-section the union
/// buffers — so the chord and the asphalt edge agree to the smoothing
/// displacement (a median half-metre at junction mouths), which the paint's
/// decal bias absorbs.
///
/// A crossing that registers no chord — mapped across a path, or floating in
/// data noise (R12) — contributes nothing here and keeps its cartographic
/// stroke, which is exactly the previous behaviour.
pub fn crossings(
    scene: &SceneGraph,
    solved: &SolvedModel,
    terrain: Option<&std::path::Path>,
) -> (Vec<CrossingPaint>, Vec<SourceSeg>) {
    let mut paints = Vec::new();
    let mut stubs = Vec::new();
    if std::env::var_os("ARPT_NO_CROSSING").is_some() {
        return (paints, stubs);
    }
    // The level gate's ground reference. Absent (a flat synthetic world, a
    // run without terrain) every level test passes and registration is the
    // plan-only one it always was.
    let mut dem = terrain.and_then(|p| crate::dem::Dem::open(p).ok());
    // Every drivable carriageway edge, indexed by plan position — the same
    // hosts `assemble::walks` attaches against, indexed the same way.
    let mut grid = crate::assemble::grid::GridIndex::with_cell_m(64.0);
    let mut edges: Vec<(u32, u32)> = Vec::new();
    for (ci, c) in scene.corridors.iter().enumerate() {
        if c.kind.prior().surface != priors::Surface::Asphalt {
            continue;
        }
        if super::carriageway::corridor_half_width_m(c).is_none() {
            continue;
        }
        for i in 0..c.nodes.len().saturating_sub(1) {
            let (a, b) = (c.nodes[i], c.nodes[i + 1]);
            grid.insert(
                (a.x.min(b.x), a.y.min(b.y), a.x.max(b.x), a.y.max(b.y)),
                edges.len() as u32,
            );
            edges.push((ci as u32, i as u32));
        }
    }
    let mut scratch: Vec<u32> = Vec::new();
    for (line, _) in scene.walks.lines() {
        if !line.crosswalk || line.line.len() < 2 {
            continue;
        }
        if !line.spans.is_empty() {
            continue; // partly elevated: not paint on an at-grade carriageway
        }
        let cos_lat = crate::scene::run_cos_lat(&line.line);
        let arc = crate::scene::cumulative_arc(&line.line);
        let total = *arc.last().unwrap_or(&0.0);
        if !(total > 0.0) {
            continue;
        }
        // The extended line: the mapped one, pushed out along its end
        // tangents. Positions are parameterized by extended arc `t` in
        // [-EXTEND, total + EXTEND].
        let at = |t: f64| -> Coord {
            if t < 0.0 {
                end_extension(&line.line, cos_lat, false, -t)
            } else if t > total {
                end_extension(&line.line, cos_lat, true, t - total)
            } else {
                point_on(&line.line, &arc, t)
            }
        };
        // **The carriageways this crossing actually crosses.** Without this the
        // registration asks only "is there asphalt under me", which a road the
        // crossing runs *beside* answers as readily as one it spans — so a
        // crossing at a station forecourt annexed the service roads flanking it
        // and painted a ladder over a railway.
        let anchor =
            dem.as_mut().map(|d| crossing_level_anchor(&line.line, d, solved.z_ref));
        let hosts =
            crossed_hosts(&at, total, scene, solved, anchor, &grid, &edges, &mut scratch);
        if hosts.is_empty() {
            continue; // crosses nothing: keep the stroke, draw nothing
        }
        // On-asphalt intervals of the extended line.
        let mut on: Vec<(f64, f64)> = Vec::new();
        let mut t = -CROSSING_EXTEND_M;
        let mut open: Option<f64> = None;
        while t <= total + CROSSING_EXTEND_M + 1e-9 {
            let p = at(t);
            let inside = on_asphalt(scene, solved, anchor, &hosts, p);
            match (inside, open) {
                (true, None) => open = Some(t),
                (false, Some(s)) => {
                    on.push((s, t - CROSSING_STEP_M));
                    open = None;
                }
                _ => {}
            }
            t += CROSSING_STEP_M;
        }
        if let Some(s) = open {
            on.push((s, total + CROSSING_EXTEND_M));
        }
        // Merge across noise, drop the grazes.
        let mut merged: Vec<(f64, f64)> = Vec::new();
        for (s, e) in on {
            match merged.last_mut() {
                Some(last) if s - last.1 < CROSSING_MERGE_M => last.1 = e,
                _ => merged.push((s, e)),
            }
        }
        merged.retain(|&(s, e)| e - s >= CROSSING_MIN_CHORD_M);
        if merged.is_empty() {
            continue; // nothing crossed: keep the stroke, draw nothing
        }
        // The stubs: the mapped line outside every chord. Only what the map
        // actually draws — the extension found the kerb, it is not sidewalk.
        let mut cursor = 0.0f64;
        for &(s, e) in &merged {
            stub_band(&line.line, &arc, cursor.min(total), s.clamp(0.0, total), cos_lat, &mut stubs);
            cursor = e.max(cursor);
        }
        stub_band(&line.line, &arc, cursor.clamp(0.0, total), total, cos_lat, &mut stubs);
        // The traffic direction per chord: the tangent of the crossed
        // centerline at the registered intersection nearest the chord's
        // midpoint. `hosts` carries the corridor arcs of every proper
        // intersection, so this is the registration's own answer re-read,
        // not a second derivation.
        let traffic_at = |mid: Coord| -> Option<(f64, f64)> {
            let mut best: Option<(f64, (f64, f64))> = None;
            for (ci, arcs) in &hosts {
                let c = &scene.corridors[*ci as usize];
                for &sc in arcs {
                    let k = c.arc.partition_point(|&x| x < sc).clamp(1, c.nodes.len() - 1);
                    let (na, nb) = (c.nodes[k - 1], c.nodes[k]);
                    let (a0, a1) = (c.arc[k - 1], c.arc[k]);
                    let t = if a1 > a0 { ((sc - a0) / (a1 - a0)).clamp(0.0, 1.0) } else { 0.0 };
                    let x = Coord { x: na.x + (nb.x - na.x) * t, y: na.y + (nb.y - na.y) * t };
                    let (dx, dy) = ((x.x - mid.x) * c.cos_lat, x.y - mid.y);
                    let d2 = dx * dx + dy * dy;
                    let (tx, ty) = ((nb.x - na.x) * c.cos_lat, nb.y - na.y);
                    let l = tx.hypot(ty);
                    if l > 0.0 && best.is_none_or(|(bd, _)| d2 < bd) {
                        best = Some((d2, (tx / l, ty / l)));
                    }
                }
            }
            best.map(|(_, t)| t)
        };
        paints.push(CrossingPaint {
            source: line.source,
            chords: merged
                .iter()
                .map(|&(s, e)| {
                    let (a, b) = (at(s), at(e));
                    let traffic = traffic_at(at(0.5 * (s + e))).unwrap_or_else(|| {
                        let (dx, dy) = ((b.x - a.x) * cos_lat, b.y - a.y);
                        let l = dx.hypot(dy).max(1e-12);
                        (-dy / l, dx / l)
                    });
                    Chord { a, b, traffic }
                })
                .collect(),
        });
    }
    (paints, stubs)
}

/// The at-grade level a crossing is drawn at: the reference terrain at its
/// mapped line, read as the median of three interior stations so one station
/// hanging over a terrace edge cannot move it. This is the anchor every
/// registration decision measures against — an at-grade crosswalk has one
/// seat, and a carriageway that is not near it is not crossed however much of
/// it lies under the line in plan.
pub(crate) fn crossing_level_anchor(
    line: &[Coord],
    dem: &mut crate::dem::Dem,
    z_ref: u8,
) -> f64 {
    let arc = crate::scene::cumulative_arc(line);
    let total = *arc.last().unwrap_or(&0.0);
    let mut hs: [f64; 3] = [0.25, 0.5, 0.75].map(|f| {
        let p = point_on(line, &arc, total * f);
        crate::solve::reference_surface(dem, z_ref, p.x, p.y)
    });
    hs.sort_by(f64::total_cmp);
    hs[1]
}

/// Whether the carriageway `ci` is at the crossing's own level at plan point
/// `p`. Permissive by construction where there is nothing to measure: no
/// anchor (no DEM) or no solved profile leaves the plan test in charge,
/// which is exactly the previous behaviour.
pub(crate) fn host_level_ok(
    solved: &SolvedModel,
    ci: CorridorId,
    p: Coord,
    anchor: Option<f64>,
) -> bool {
    let (Some(anchor), Some(prof)) = (anchor, solved.profile(ci)) else {
        return true;
    };
    (prof.height_at(p.x, p.y) - anchor).abs() <= CROSSING_LEVEL_M
}

/// Where two segments cross. Called only when [`segments_cross`] said they
/// do, so the denominator cannot vanish other than by degeneracy, which
/// falls back to an endpoint.
fn seg_intersection(a0: Coord, a1: Coord, b0: Coord, b1: Coord) -> Coord {
    let d = (a1.x - a0.x) * (b1.y - b0.y) - (a1.y - a0.y) * (b1.x - b0.x);
    if d.abs() < 1e-18 {
        return a0;
    }
    let t = ((b0.x - a0.x) * (b1.y - b0.y) - (b0.y - a0.y) * (b1.x - b0.x)) / d;
    Coord { x: a0.x + (a1.x - a0.x) * t, y: a0.y + (a1.y - a0.y) * t }
}

/// The carriageways whose centerline this crossing's line properly intersects.
///
/// **This is the predicate `verify::model::street::crossing_extent` scores
/// against**, and the two now make the same distinction rather than only the
/// check making it: a corridor a crossing merely runs beside is not crossed,
/// however much of it lies under the line.
///
/// The *extended* line is tested, not the mapped one. A crosswalk is mapped by
/// hand and its endpoints rarely reach the kerbs, let alone the centerline
/// (docs/ROADS.md §2), so a short stub across a wide road properly intersects
/// nothing — and the extension is the same 8 m of tangent the chord march
/// already walks. It cannot admit a parallel road: an extension is collinear
/// with the end it leaves, so a corridor running alongside the crossing is no
/// more crossed by the extension than by the line itself.
///
/// Plan intersection alone is not enough where streets stack: each candidate
/// must also hold [`host_level_ok`] at the intersection — its solved height
/// within [`CROSSING_LEVEL_M`] of the crossing's own ground — or a terrace
/// crosswalk registers the avenue below it (see the constant's comment for
/// the measured menagerie). The same gate runs per sample in [`on_asphalt`],
/// because a host is a whole spliced corridor: a hairpin's two arms share one
/// id, and only the sample-level test keeps the chord off the arm eight
/// metres up the slope.
fn crossed_hosts(
    at: &dyn Fn(f64) -> Coord,
    total: f64,
    scene: &SceneGraph,
    solved: &SolvedModel,
    anchor: Option<f64>,
    grid: &crate::assemble::grid::GridIndex,
    edges: &[(u32, u32)],
    scratch: &mut Vec<u32>,
) -> Vec<(u32, Vec<f64>)> {
    // The extended line as a polyline: both extensions plus the mapped nodes,
    // walked pairwise. `at` is the one parameterization the chord march uses,
    // so the two cannot disagree about where the crossing is.
    let mut pts: Vec<Coord> = vec![at(-CROSSING_EXTEND_M)];
    let mut t = 0.0;
    while t < total {
        pts.push(at(t));
        t += CROSSING_STEP_M;
    }
    pts.push(at(total));
    pts.push(at(total + CROSSING_EXTEND_M));

    let mut hosts: Vec<(u32, Vec<f64>)> = Vec::new();
    for w in pts.windows(2) {
        let bbox =
            (w[0].x.min(w[1].x), w[0].y.min(w[1].y), w[0].x.max(w[1].x), w[0].y.max(w[1].y));
        grid.query(bbox, scratch);
        for &e in scratch.iter() {
            let (ci, ni) = edges[e as usize];
            let c = &scene.corridors[ci as usize];
            let (a, b) = (c.nodes[ni as usize], c.nodes[ni as usize + 1]);
            if !segments_cross(w[0], w[1], a, b)
                || !incidence_ok(w[0], w[1], a, b, c.cos_lat)
            {
                continue;
            }
            let x = seg_intersection(w[0], w[1], a, b);
            if !host_level_ok(solved, ci, x, anchor) {
                continue;
            }
            let s = c.arc[ni as usize]
                + crate::scene::metric_len(c.nodes[ni as usize], x, c.cos_lat);
            match hosts.iter_mut().find(|(h, _)| *h == ci) {
                Some((_, arcs)) => arcs.push(s),
                None => hosts.push((ci, vec![s])),
            }
        }
    }
    hosts
}

/// Whether two crossing segments meet at a believable incidence for a
/// crosswalk: `|sin|` of the angle between them at least
/// [`CROSSING_MIN_INCIDENCE_SIN`].
pub(crate) fn incidence_ok(w0: Coord, w1: Coord, a: Coord, b: Coord, cos_lat: f64) -> bool {
    let (ux, uy) = ((w1.x - w0.x) * cos_lat, w1.y - w0.y);
    let (vx, vy) = ((b.x - a.x) * cos_lat, b.y - a.y);
    let (lu, lv) = (ux.hypot(uy), vx.hypot(vy));
    if !(lu > 0.0 && lv > 0.0) {
        return false;
    }
    ((ux * vy - uy * vx) / (lu * lv)).abs() >= CROSSING_MIN_INCIDENCE_SIN
}

/// Whether two segments properly cross — opposite orientations both ways.
///
/// The same test `verify::model::street` makes, in plan degrees. Scaling to
/// metres would not change a sign, and a sign is all this reads.
fn segments_cross(a0: Coord, a1: Coord, b0: Coord, b1: Coord) -> bool {
    let orient = |p: Coord, q: Coord, r: Coord| {
        let v = (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
        if v > 0.0 {
            1i8
        } else if v < 0.0 {
            -1
        } else {
            0
        }
    };
    let (d1, d2) = (orient(b0, b1, a0), orient(b0, b1, a1));
    let (d3, d4) = (orient(a0, a1, b0), orient(a0, a1, b1));
    d1 != d2 && d3 != d4
}

/// Whether a point lies on a carriageway **this crossing crosses**: within the
/// drawn half-width of the centerline of one of its registered hosts.
///
/// Corridor-wide in plan, gated by level: a crossing that spans a street is
/// entitled to that street's asphalt wherever the chord meets it *at the
/// crossing's own level* — a host is a whole spliced corridor, so without the
/// per-sample gate a hairpin's upper arm answers for the lower arm the
/// crossing was mapped on. `verify::model::street` scores with the same gate.
fn on_asphalt(
    scene: &SceneGraph,
    solved: &SolvedModel,
    anchor: Option<f64>,
    hosts: &[(u32, Vec<f64>)],
    p: Coord,
) -> bool {
    hosts.iter().any(|(ci, arcs)| {
        let c = &scene.corridors[*ci as usize];
        let Some(half) = super::carriageway::corridor_half_width_m(c) else { return false };
        (0..c.nodes.len().saturating_sub(1)).any(|i| {
            let (w0, w1) = (c.nodes[i], c.nodes[i + 1]);
            let near_station = arcs.iter().any(|&x| {
                c.arc[i] <= x + CROSSING_ARC_WINDOW_M
                    && c.arc[i + 1] >= x - CROSSING_ARC_WINDOW_M
            });
            near_station && probe_seg_dist_m(p, w0, w1, c.cos_lat) <= half
        }) && host_level_ok(solved, *ci, p, anchor)
    })
}

/// A point `over_m` past one end of a polyline, along that end's tangent.
pub(crate) fn end_extension(line: &[Coord], cos_lat: f64, at_end: bool, over_m: f64) -> Coord {
    let (p, q) = if at_end {
        (line[line.len() - 1], line[line.len() - 2])
    } else {
        (line[0], line[1])
    };
    let m_lon = DEG_M * cos_lat;
    let (dx, dy) = ((p.x - q.x) * m_lon, (p.y - q.y) * DEG_M);
    let len = dx.hypot(dy);
    if !(len > 0.0) {
        return p;
    }
    Coord { x: p.x + dx / len * over_m / m_lon, y: p.y + dy / len * over_m / DEG_M }
}

/// The point at arc `s` along a polyline (clamped to it).
fn point_on(line: &[Coord], arc: &[f64], s: f64) -> Coord {
    let i = arc.partition_point(|&a| a < s).clamp(1, line.len() - 1);
    let (a0, a1) = (arc[i - 1], arc[i]);
    let t = if a1 > a0 { ((s - a0) / (a1 - a0)).clamp(0.0, 1.0) } else { 0.0 };
    Coord {
        x: line[i - 1].x + (line[i].x - line[i - 1].x) * t,
        y: line[i - 1].y + (line[i].y - line[i - 1].y) * t,
    }
}

/// Bands the stretch `[s0, s1]` of a crossing's mapped line — a kerb stub:
/// the strip of real sidewalk between the carriageway edge and whatever the
/// crossing joins. Walkway material **on the ground**: the end of a crossing
/// is a dropped kerb by construction — it is where a person steps off — and a
/// rise here would float the band above the bench stratum D cuts for it,
/// which keys a hostless band's target to the ground along its centerline
/// (`contact.walk_rim` read the 0.12 m float on every stub, measured).
fn stub_band(
    line: &[Coord],
    arc: &[f64],
    s0: f64,
    s1: f64,
    cos_lat: f64,
    out: &mut Vec<SourceSeg>,
) {
    if s1 - s0 < CROSSING_MIN_STUB_M {
        return;
    }
    // The stub half of `ARPT_NO_CROSSING`, so the paint and the band it meets
    // at the kerb can be measured apart — they are one derivation with two
    // readers, and a metric that moves needs to say which reader moved it.
    if std::env::var_os("ARPT_NO_CROSSING_STUB").is_some() {
        return;
    }
    // A sidewalk's width, not the crossing's. `CROSSING_WIDTH_M` is how deep
    // the *paint* is along the road axis; the stub is the piece of pavement
    // the crossing lands on, and drawing it wider than the band it joins put
    // a bulge at every kerb.
    let half = priors::WALK_WIDTH_M * 0.5;
    let (a, b) = (point_on(line, arc, s0), point_on(line, arc, s1));
    out.push(SourceSeg {
        a,
        b,
        cos_lat,
        half_m: half,
        sect_a: Section::uniform(half),
        sect_b: Section::uniform(half),
        level: 0,
        layer: 0,
        cut_a: None,
        cut_b: None,
        height_a: 0.0,
        height_b: 0.0,
        corridor: NO_HOST,
        surface: priors::Surface::Walkway,
        rise_m: 0.0,
        arc0: s0,
    });
}

/// How far a kerb stub looks for the band it continues, in metres.
///
/// The stub ends on the pavement, so the pavement's *centerline* is within
/// half a band width of it — [`priors::WALK_WIDTH_M`] and half again covers a
/// band the street narrowed, without reaching across a road to the pavement
/// opposite.
const STUB_SEAT_REACH_M: f64 = priors::WALK_WIDTH_M * 1.5;

/// Seats each kerb stub on the band it continues.
///
/// **A stub was seated on the ground while the pavement beside it was seated
/// on its street.** [`fitted_half`] takes a `NO_HOST` segment's target from
/// the ground under its own ends and a hosted one's from `height_a`/`height_b`
/// — the height its street's cross-section draws it at, kerb included. A stub
/// was built hostless with both heights zero, so the two met at the kerb in
/// plan and nowhere in section: at 6.856580,46.457663 the band ran
/// 384.2 → 383.67 → 384.3, a 0.6 m notch a third of a metre wide, and
/// `slope.walk_crossfall` read the drop off the stub's own edge.
///
/// So a stub takes the corridor, the drawn height and the kerb rise of the
/// nearest hosted walkway band at each of its ends. It is the same claim the
/// stub's doc already makes — "the strip of real sidewalk between the kerb and
/// whatever the crossing joins" — made to the machinery that decides heights
/// rather than only to the reader. A stub that finds no band within
/// [`STUB_SEAT_REACH_M`] stays hostless and drapes as before: a crossing onto
/// a path, or onto a pavement the fit declined.
pub fn seat_stubs(bands: &mut [SourceSeg], stub_from: usize) {
    if stub_from >= bands.len() {
        return;
    }
    let mut grid = crate::assemble::grid::GridIndex::with_cell_m(64.0);
    let mut hosted: Vec<u32> = Vec::new();
    for (i, s) in bands[..stub_from].iter().enumerate() {
        if s.surface != priors::Surface::Walkway || s.corridor == NO_HOST {
            continue;
        }
        grid.insert(
            (s.a.x.min(s.b.x), s.a.y.min(s.b.y), s.a.x.max(s.b.x), s.a.y.max(s.b.y)),
            hosted.len() as u32,
        );
        hosted.push(i as u32);
    }
    if hosted.is_empty() {
        return;
    }

    let mut scratch: Vec<u32> = Vec::new();
    for si in stub_from..bands.len() {
        let stub = bands[si];
        let seat = |end: Coord, scratch: &mut Vec<u32>| -> Option<(CorridorId, f64, f64)> {
            let (dx, dy) = (
                STUB_SEAT_REACH_M / (DEG_M * stub.cos_lat),
                STUB_SEAT_REACH_M / DEG_M,
            );
            scratch.clear();
            grid.query((end.x - dx, end.y - dy, end.x + dx, end.y + dy), scratch);
            let mut best: Option<(f64, CorridorId, f64, f64)> = None;
            for &h in scratch.iter() {
                let s = &bands[hosted[h as usize] as usize];
                let (d, t) = point_to_seg(end, s.a, s.b, stub.cos_lat);
                if d > STUB_SEAT_REACH_M {
                    continue;
                }
                if best.is_none_or(|(bd, ..)| d < bd) {
                    let h = s.height_a + (s.height_b - s.height_a) * t;
                    best = Some((d, s.corridor, h, s.rise_m));
                }
            }
            best.map(|(_, c, h, r)| (c, h, r))
        };
        let (sa, sb) = (seat(stub.a, &mut scratch), seat(stub.b, &mut scratch));
        // One end is enough: a stub is a couple of metres long, and the end
        // that found a band is the one that is *on* the pavement — the other
        // is at the kerb, over the carriageway's own hole.
        let Some((corridor, _, rise)) = sa.or(sb) else { continue };
        let s = &mut bands[si];
        s.corridor = corridor;
        s.rise_m = rise;
        s.height_a = sa.or(sb).map(|(_, h, _)| h).expect("one end seated");
        s.height_b = sb.or(sa).map(|(_, h, _)| h).expect("one end seated");
    }
}

/// Distance in metres from `p` to segment `a`–`b`, and the fraction along it
/// of the closest point.
fn point_to_seg(p: Coord, a: Coord, b: Coord, cos_lat: f64) -> (f64, f64) {
    let (ax, ay) = ((b.x - a.x) * cos_lat * DEG_M, (b.y - a.y) * DEG_M);
    let (px, py) = ((p.x - a.x) * cos_lat * DEG_M, (p.y - a.y) * DEG_M);
    let len2 = ax * ax + ay * ay;
    let t = if len2 > 0.0 { ((px * ax + py * ay) / len2).clamp(0.0, 1.0) } else { 0.0 };
    let (dx, dy) = (px - ax * t, py - ay * t);
    ((dx * dx + dy * dy).sqrt(), t)
}

/// Builds every walkway band in the extract: the geometry, the seat and the
/// material, with no grade layer yet.
///
/// **Model, not drawing, and that is why the grade layer comes later.** The
/// band is derived once, before stage 3, because the ground under a walkway
/// is that walkway (`ground::walk_earthworks`) and a bench derived from a
/// *second* construction of the same band would be a bench that does not fit
/// it — the sliver family this codebase keeps re-learning. One derivation,
/// two readers: the ground benches it, and the union draws it.
///
/// The layer is stamped in `synth::carriageway::bake`, by running the sheet
/// layering (`synth::sheets`) over the walk bands *among themselves* — never
/// mixed into the carriageway assignment: a sidewalk must not vote on the
/// grade-separation layering of the street it stands beside, and the walk
/// sheet is its own namespace (`height::Sheet::walk`).
/// Builds every walkway band, and — in step with it — the **source each
/// segment came from**.
///
/// The second vector is what lets the drawing keep its promise of graceful
/// degradation (docs/GENERATION.md I6). A pedestrian way's cartographic
/// stroke is deleted at the walk zooms because the band *is* the surface
/// (`pipeline::paves_via_walkway`), and if that test were the class and
/// nothing else, a way whose band the ground fit declined to build would lose
/// its stroke too and vanish from the map entirely. On a steep flank that is
/// exactly where it happens: the Territet switchback at 6.9189,46.4304 came
/// out as a handful of disjoint slabs with nothing between them. Carrying the
/// source per segment lets phase 1 ask *was anything actually drawn for this
/// feature* rather than *is this feature the kind of thing we draw*.
/// Every pedestrian surface the extract draws, and the way each segment came
/// from (0 where nothing owns it).
///
/// **A pavement is a side of a street, not a feature of its own**
/// (docs/ROADS.md invariant 1). Two producers, in that order:
///
/// - [`street_bands`] — one continuous strip per corridor side that the data
///   says carries a pavement, seated on the kerb, sized by the one allotment
///   in [`super::cross`].
/// - [`free_bands`] — the ways that witness no street: a path across a field,
///   a farm track, a footway leaving the network. These follow their own
///   polyline, because there is no cross-section for them to be part of.
///
/// **What this replaced, and why.** The band used to be built per *attachment*:
/// `assemble::walks::runs` breaks an attachment wherever the way turns across
/// its host, so one mapped pavement arrived as an alternating chain of claimed
/// and unclaimed stretches, and the two were then drawn as different materials
/// on different curves — `Walkway` on the host centerline where a claim held,
/// `Path` on the mapped polyline where it did not, with `Path` junior to
/// `Walkway` so the overlaps were subtracted. Every transition was a lateral
/// jump, every sub-`MIN_BAND_M` stretch a hole, and every seat that ran out of
/// room a silent 43 m of nothing. `street.strip_continuity` measured a third of
/// all claimed pavement drawn bare, with one unbroken 103 m hole — while the
/// attach census read 98.1 % built, because a census counts arc that produced a
/// segment and cannot see that the segments do not join.
pub fn bands(
    scene: &SceneGraph,
    solved: &SolvedModel,
    facades: &Facades,
) -> (Vec<SourceSeg>, Vec<u64>) {
    let mut out = Vec::new();
    let mut sources: Vec<u64> = Vec::new();
    if std::env::var_os("ARPT_NO_WALK_BAND").is_some() {
        return (out, sources);
    }
    let mut census = AttachCensus::default();
    let claims = claims_by_side(scene);
    // **What was built, not what was claimed.** The free bands are the
    // complement of the strip, and the strip can decline a stretch its claim
    // covers — the room ran out. Subtracting the *claim* deleted the path band
    // there too and left bare ground with neither: `street.strip_continuity`
    // 34.2 % -> 39.4 % on the first cut of this, which is the same
    // attached-is-not-drawn trap the corner rule fell into before it.
    let mut built: HashMap<(u32, u8), Vec<(f64, f64)>> = HashMap::new();
    street_bands(scene, solved, facades, &claims, &mut built, &mut out, &mut sources, &mut census);
    free_bands(scene, &built, &mut out, &mut sources);
    census.report();
    probe(scene, &claims, &built);
    (out, sources)
}

/// What the data says about the pavement on one side of one street.
struct SideClaims {
    /// Where the strip is drawn, in host arc metres: the claims **merged**
    /// across the gaps an attachment breaks at by design.
    ///
    /// `assemble::walks::runs` ends a run wherever the way turns across its
    /// host — right, a band must not bridge the mouth of a side street — which
    /// puts a break at every corner and every driveway. The stretch between two
    /// claims on one side of one street is pavement the data asserts as plainly
    /// as the claims themselves, so the extent the drawing is held to is the
    /// merged one. It is the same judgement [`priors::WALK_CORNER_MAX_M`] was
    /// introduced for, made once here instead of as a special case downstream.
    spans: Vec<(f64, f64)>,
    /// The claims themselves, in arc order, so a segment can be attributed to
    /// the way that actually witnessed it rather than to whichever way happened
    /// to dominate the merge. What keeps a way's stroke depends on this.
    parts: Vec<(f64, f64, u64)>,
}

impl SideClaims {
    /// Whether the strip is drawn at this arc.
    fn covers(&self, arc: f64) -> bool {
        self.spans.iter().any(|&(a, b)| arc >= a && arc <= b)
    }

    /// The way a segment starting at this arc belongs to: the claim containing
    /// it, or — inside a filled gap — the nearer of the two claims it links.
    fn owner(&self, arc: f64) -> u64 {
        let mut best: Option<(f64, u64)> = None;
        for &(a, b, src) in &self.parts {
            if arc >= a && arc <= b {
                return src;
            }
            let d = (a - arc).abs().min((b - arc).abs());
            if best.is_none_or(|(bd, _)| d < bd) {
                best = Some((d, src));
            }
        }
        best.map_or(0, |(_, src)| src)
    }
}

/// Every corridor side the data attaches a pedestrian way to.
///
/// A `steps` is excluded: `assemble::walks` attaches one to the street it
/// climbs from — it is that street's stair — but a staircase's whole purpose is
/// to change height relative to what is beside it, so it is not a strip of that
/// street's cross-section. It draws as a free band on its own line until P5
/// gives it a stepped profile of its own.
fn claims_by_side(scene: &SceneGraph) -> HashMap<(u32, u8), SideClaims> {
    let mut raw: HashMap<(u32, u8), Vec<(f64, f64, u64)>> = HashMap::new();
    for (line, attached) in scene.walks.lines() {
        if line.crosswalk || matches!(line.kind, Kind::Road(RoadClass::Steps)) {
            continue;
        }
        for a in attached {
            raw.entry((a.host, a.side)).or_default().push((a.arc0, a.arc1, line.source));
        }
    }
    raw.into_iter()
        .map(|(key, mut parts)| {
            parts.sort_by(|x, y| x.0.total_cmp(&y.0));
            let spans = merge_spans(&parts);
            (key, SideClaims { spans, parts })
        })
        .collect()
}

/// Arc-sorted claims merged across the gaps an attachment breaks at by design
/// — see [`SideClaims::spans`] for why the gap is pavement too.
fn merge_spans(sorted: &[(f64, f64, u64)]) -> Vec<(f64, f64)> {
    let mut spans: Vec<(f64, f64)> = Vec::new();
    for &(a, b, _) in sorted {
        match spans.last_mut() {
            Some(last) if a - last.1 <= priors::WALK_CORNER_MAX_M => last.1 = last.1.max(b),
            _ => spans.push((a, b)),
        }
    }
    spans
}

/// Which sides of which corridors carry a pavement — the same question
/// [`street_bands`] answers when it builds one, asked early enough for the
/// **ground** to hear it.
///
/// `ground::derive_seniors` runs before the strips exist (the seniors imprint,
/// the band is fitted to what it finds, and stratum D benches the result), so
/// the road bench cannot read the strips themselves. It can read the same two
/// inputs they come from: what the data claims, and what the synthesis prior
/// would add. That is enough for a street's bench to be as wide as its own
/// cross-section, which is what stops the pavement from cutting a second
/// terrace beside the first.
pub(crate) fn pavement_sides(
    scene: &SceneGraph,
    solved: &SolvedModel,
    facades: &Facades,
) -> std::collections::HashSet<(u32, u8)> {
    let mut out = std::collections::HashSet::new();
    if std::env::var_os("ARPT_NO_WALK_BAND").is_some() {
        return out;
    }
    for (line, attached) in scene.walks.lines() {
        if line.crosswalk || matches!(line.kind, Kind::Road(RoadClass::Steps)) {
            continue;
        }
        for a in attached {
            out.insert((a.host, a.side));
        }
    }
    if std::env::var_os("ARPT_NO_WALK_SYNTH").is_none() {
        let mut scratch: Vec<u32> = Vec::new();
        for c in &scene.corridors {
            if priors::synthesizes_pavement(c.kind) && built_up(c, solved, facades, &mut scratch) {
                out.insert((c.id, 0));
                out.insert((c.id, 1));
            }
        }
    }
    out
}


/// How far a wall may stand from a street's centerline and still make it a
/// built-up street, in metres.
///
/// Read off the sizing census rather than guessed: over Montreux the share of
/// side-length with walls on both sides moves 59.7 / 68.4 / 75.1 % for
/// residential at 15 / 25 / 40 m, so the answer is not sharp and the middle
/// value is the honest one. What the number must do is separate a town street
/// from the same class winding up a hillside, and at 25 m it does: motorway
/// comes out 0.00 km built-up against 18.9 km not, and unclassified — mostly
/// rural lanes here — 5.0 against 88.5.
const BUILT_UP_REACH_M: f64 = 25.0;

/// Share of a corridor's at-grade length that must have walls on both sides
/// for the corridor to be a built-up street.
///
/// A whole-corridor verdict rather than a per-station one, deliberately: a
/// pavement that switched on and off as a street left and re-entered a terrace
/// of houses is the fragmentation this model was rebuilt to stop.
const BUILT_UP_SHARE: f64 = 0.5;

/// Station spacing for the built-up walk, in metres — coarse, because the
/// question is what a whole street is, not what one metre of it is.
const BUILT_UP_STEP_M: f64 = 10.0;

/// Whether this corridor is a street with rooms either side of it: walls within
/// [`BUILT_UP_REACH_M`] on **both** sides over [`BUILT_UP_SHARE`] of its
/// at-grade length.
///
/// **Two walls, not one.** A single facade within reach is a barn beside a
/// mountain road; two facing each other across the carriageway is a street. The
/// test is the load-bearing half of the synthesis prior — the class table says
/// only which classes *could* carry a pavement — and it is what stops
/// `priors::synthesizes_pavement` from paving the countryside.
///
/// A corridor outside the building input's coverage answers **false**, which is
/// the right way round: assemble admits whole parquet row groups, so the scene
/// runs far past the extract into ground no footprint covers, and a synthesis
/// prior that fired there would invent pavement precisely where it knows least.
pub(crate) fn built_up(
    c: &Corridor,
    solved: &SolvedModel,
    facades: &Facades,
    scratch: &mut Vec<u32>,
) -> bool {
    if facades.is_empty() {
        return false;
    }
    let Some(half_m) = super::carriageway::corridor_half_width_m(c) else { return false };
    let profile = solved.profile(c.id);
    let point = |arc: f64| match profile {
        Some(p) => p.smooth_at_arc(arc),
        None => super::carriageway::raw_point_at_arc(c, arc),
    };
    let reach = half_m + BUILT_UP_REACH_M;
    let (mut total, mut walled) = (0.0f64, 0.0f64);
    for (r0, r1, _, kind) in level_runs(c) {
        if kind != SpanKind::Grade {
            continue;
        }
        let mut a = r0;
        while a < r1 {
            let b = (a + BUILT_UP_STEP_M).min(r1);
            let mid = 0.5 * (a + b);
            let step = b - a;
            let (p, q) = (point(mid), point((mid + 1.0).min(r1)));
            // Only where the building input was actually read. A corridor runs
            // far past the extract, and counting its unsurveyed tail as "no
            // walls" put a town street at a few per cent built-up — the verdict
            // then said countryside about the middle of Montreux, and the whole
            // synthesis fired on 259 earthwork edges.
            if !facades.covered(p) {
                a = b;
                continue;
            }
            total += step;
            let m_lon = DEG_M * c.cos_lat;
            let (dx, dy) = ((q.x - p.x) * m_lon, (q.y - p.y) * DEG_M);
            let len = dx.hypot(dy);
            if len > 0.0 {
                let room = facades.room(
                    p,
                    c.cos_lat,
                    (dx / len, dy / len),
                    reach,
                    super::carriageway::ROOM_WINDOW_MAX_M,
                    scratch,
                );
                if room.left < reach && room.right < reach {
                    walled += step;
                }
            }
            a = b;
        }
    }
    total > 0.0 && walled >= BUILT_UP_SHARE * total
}


/// The pavements: one continuous strip per corridor side, over the whole extent
/// the data claims.
///
/// **Continuity is structural here, where it used to be luck.** The strip is
/// emitted along the corridor's own stations, at a *constant* `half_m`, with
/// every width variation in `sect_a`/`sect_b` — which is the idiom the
/// carriageway has always used and the pavement never did. `pavement::runs`
/// chains segments into one buffered polyline only while `half_m` matches, and
/// a boolean union keeps merely-touching shapes apart, so a band that varied
/// `half_m` was drawn as one slab per width rung. Consecutive segments share an
/// endpoint bit-for-bit because both compute it from the same station, the same
/// normal and the same allotment.
#[allow(clippy::too_many_arguments)]
fn street_bands(
    scene: &SceneGraph,
    solved: &SolvedModel,
    facades: &Facades,
    claims: &HashMap<(u32, u8), SideClaims>,
    built: &mut HashMap<(u32, u8), Vec<(f64, f64)>>,
    out: &mut Vec<SourceSeg>,
    sources: &mut Vec<u64>,
    census: &mut AttachCensus,
) {
    let no_room = std::env::var_os("ARPT_NO_FACADE_ROOM").is_some();
    let mut scratch: Vec<u32> = Vec::new();
    // **Where the data maps no pavement, the class and the facades decide.**
    // Overture carries no `sidewalk=*` on a road (docs/SOURCES.md §7), so a
    // street whose pavement nobody drew is indistinguishable from one with
    // none, and taking the data's silence for absence leaves a town's
    // residential streets bare. `priors::synthesizes_pavement` says which
    // classes could carry one and [`built_up`] says whether this street
    // actually is one. `ARPT_NO_WALK_SYNTH=1` withholds it, for the A/B.
    let synthesize = std::env::var_os("ARPT_NO_WALK_SYNTH").is_none();
    for c in &scene.corridors {
        let sides: [Option<&SideClaims>; 2] =
            [claims.get(&(c.id, 0)), claims.get(&(c.id, 1))];
        let synth = synthesize
            && priors::synthesizes_pavement(c.kind)
            && built_up(c, solved, facades, &mut scratch);
        if sides[0].is_none() && sides[1].is_none() && !synth {
            continue;
        }
        let claimed: f64 = sides
            .iter()
            .flatten()
            .flat_map(|s| s.spans.iter())
            .map(|&(a, b)| b - a)
            .sum();
        census.claimed_m += claimed;
        let Some(half_m) = super::carriageway::corridor_half_width_m(c) else {
            census.no_width_m += claimed;
            continue; // the host paves nothing, so there is no kerb to sit on
        };
        let profile = solved.profile(c.id);
        for (r0, r1, level, kind) in level_runs(c) {
            // **Only where the host is on the ground.** Over a bridge or in a
            // bore the pavement is carried by the structure (`synth::carried`),
            // and a strip drawn there would be a second one floating beside the
            // deck.
            if kind != SpanKind::Grade {
                census.non_grade_m += overlap(&sides, r0, r1);
                continue;
            }
            // The host's own stations, so the strip is sampled on the same
            // curve the asphalt is buffered around and the two stay parallel by
            // construction.
            let stations: Vec<f64> = match profile {
                Some(p) => p.arc().iter().copied().filter(|&s| s > r0 && s < r1).collect(),
                None => c.arc.iter().copied().filter(|&s| s > r0 && s < r1).collect(),
            };
            // **The claim's own boundaries are stations.** Without them a
            // segment straddling the end of a span has one station inside and
            // one outside, so it can be neither drawn nor honestly refused —
            // the first cut charged every such segment to `narrow` and dropped
            // it, which on profile stations a few metres apart is two segments
            // per span and was 16.5 % of the claimed extent all by itself. Cut
            // here and every segment is wholly inside a claim or wholly outside
            // it, and the strip ends exactly where the data says it does.
            let mut stations: Vec<f64> = stations;
            for cl in sides.iter().flatten() {
                for &(a, b) in &cl.spans {
                    stations.extend([a, b].into_iter().filter(|&s| s > r0 && s < r1));
                }
            }
            stations.sort_by(f64::total_cmp);
            let mut stops: Vec<f64> = vec![r0];
            for s in stations.into_iter().chain(std::iter::once(r1)) {
                if s - stops[stops.len() - 1] > RUN_EPS_M {
                    stops.push(s);
                }
            }
            if stops.len() < 2 {
                continue;
            }
            let point = |arc: f64| match profile {
                Some(p) => p.smooth_at_arc(arc),
                None => super::carriageway::raw_point_at_arc(c, arc),
            };
            let pts: Vec<Coord> = stops.iter().map(|&s| point(s)).collect();
            // What each side asks for at each station — the class nominal where
            // the data claims a pavement, nothing where it does not.
            let want: Vec<[f64; 2]> = stops
                .iter()
                .map(|&s| {
                    [0usize, 1].map(|side| {
                        let mapped = sides[side].is_some_and(|cl| cl.covers(s));
                        if mapped || synth { priors::WALK_WIDTH_M } else { 0.0 }
                    })
                })
                .collect();
            // **One allotment for the whole street** (`synth::cross`): the same
            // call, the same query, the same room the asphalt was cut from.
            let sections = super::cross::sections_along(
                c, &stops, &pts, half_m, &want, facades, no_room, &mut scratch,
            );
            let height = |arc: f64| profile.map_or(0.0, |p| p.road_at_arc(arc));
            for side in 0..2 {
                let cl = sides[side];
                if cl.is_none() && !synth {
                    continue;
                }
                for i in 0..stops.len() - 1 {
                    let len = stops[i + 1] - stops[i];
                    // Read at the midpoint: the boundaries are stations now, so
                    // a segment lies wholly inside one claim or wholly outside
                    // every claim, and its midpoint says which. A synthesized
                    // street claims its whole at-grade length.
                    let mid = 0.5 * (stops[i] + stops[i + 1]);
                    if !synth && !cl.is_some_and(|c| c.covers(mid)) {
                        continue; // outside the claimed extent: nothing owed
                    }
                    let (wa, wb) = (sections[i].walk[side], sections[i + 1].walk[side]);
                    if !(wa > 0.0) || !(wb > 0.0) {
                        // The room left less than a strip worth drawing —
                        // which is the same sentence invariant 1 speaks about a
                        // street too narrow for a sidewalk, and the only
                        // legitimate way for a pavement to stop.
                        census.narrow_m += len;
                        continue;
                    }
                    census.built_m += len;
                    let na = normal_at(&pts, i, c.cos_lat, side);
                    let nb = normal_at(&pts, i + 1, c.cos_lat, side);
                    out.push(SourceSeg {
                        a: offset(pts[i], na, sections[i].walk_centre(side), c.cos_lat),
                        b: offset(pts[i + 1], nb, sections[i + 1].walk_centre(side), c.cos_lat),
                        cos_lat: c.cos_lat,
                        // **Constant along the run**, so `pavement::runs`
                        // chains it. Everything the room did to the strip is in
                        // the sections below.
                        half_m: priors::WALK_WIDTH_M * 0.5,
                        sect_a: Section::uniform(wa * 0.5),
                        sect_b: Section::uniform(wb * 0.5),
                        level,
                        layer: 0,
                        cut_a: None,
                        cut_b: None,
                        height_a: height(stops[i]) + priors::KERB_RISE_M,
                        height_b: height(stops[i + 1]) + priors::KERB_RISE_M,
                        corridor: c.id,
                        surface: priors::Surface::Walkway,
                        rise_m: priors::KERB_RISE_M,
                        arc0: stops[i],
                    });
                    // A synthesized stretch belongs to no mapped way, so it
                    // owns no source: nothing loses a stroke for it, and
                    // `banded_walks` is untouched.
                    sources.push(match cl {
                        Some(c) if c.covers(mid) => c.owner(stops[i]),
                        _ => 0,
                    });
                    // The extent that actually became pavement, so the free
                    // bands can be its complement rather than the claim's.
                    let spans = built.entry((c.id, side as u8)).or_default();
                    match spans.last_mut() {
                        Some(last) if stops[i] - last.1 <= RUN_EPS_M => last.1 = stops[i + 1],
                        _ => spans.push((stops[i], stops[i + 1])),
                    }
                }
            }
        }
    }
}

/// How much of the claimed extent on either side falls in `[r0, r1]`.
fn overlap(sides: &[Option<&SideClaims>; 2], r0: f64, r1: f64) -> f64 {
    sides
        .iter()
        .flatten()
        .flat_map(|s| s.spans.iter())
        .map(|&(a, b)| (b.min(r1) - a.max(r0)).max(0.0))
        .sum()
}

/// The bands for the ways that witness no street: a path across a field, a farm
/// track, a footway leaving the network — and the stretches of an attached way
/// that run away from the street it belongs to.
///
/// These follow their own mapped polyline, because there is no cross-section
/// for them to be a side of. Two rules that used to live here are gone with the
/// per-feature model: there is no minimum length, since a short link is still a
/// link; and there is no corner case, because a corner is now inside the strip
/// its two claims were merged into.
fn free_bands(
    scene: &SceneGraph,
    built: &HashMap<(u32, u8), Vec<(f64, f64)>>,
    out: &mut Vec<SourceSeg>,
    sources: &mut Vec<u64>,
) {
    for (line, attached) in scene.walks.lines() {
        if line.crosswalk || line.line.len() < 2 || !priors::earns_walk_band(line.kind) {
            continue;
        }
        let cos_lat = crate::scene::run_cos_lat(&line.line);
        let arc = crate::scene::cumulative_arc(&line.line);
        let total = *arc.last().unwrap_or(&0.0);
        if !(total > 0.0) {
            continue;
        }
        // What the strip already covers, in this way's own arc: the stretches
        // it was attached over, merged exactly as the strip merged them so the
        // two partitions agree, plus the stretches that are not on the ground.
        let mut taken = covered_by_strip(attached, built);
        let strip_m: f64 = taken.iter().map(|&(a, b)| b - a).sum();
        taken.extend(line.spans.iter().map(|&(s, e)| (s * total, e * total)));
        taken.sort_by(|x, y| x.0.total_cmp(&y.0));
        let mut merged: Vec<(f64, f64)> = Vec::new();
        for &(a, b) in &taken {
            match merged.last_mut() {
                Some(last) if a - last.1 <= priors::WALK_CORNER_MAX_M => last.1 = last.1.max(b),
                _ => merged.push((a, b)),
            }
        }
        // A track is drawn at a vehicle's width, everything else at a walker's.
        let nominal = if matches!(line.kind, Kind::Road(RoadClass::Track)) {
            priors::TRACK_WIDTH_M
        } else {
            priors::WALK_WIDTH_M
        };
        // **A way that is a street's pavement somewhere is its pavement
        // everywhere.** Where a strip was built for this way, the stretches that
        // leave the street are the same object continuing — so they take the
        // same material and merge into one region with one rim rather than
        // drawing as a second kind of thing beside the first. They still stand
        // on the ground and carry no kerb rise: a hostless band is benched along
        // its own centerline, and a rise there would be a float above its own
        // bench. The height field ramps the neighbouring kerb down into it,
        // which is what a dropped kerb is.
        //
        // Deliberately *not* a blanket merge of the two materials. A hillside
        // path that merely passes a road has no cross-section relation to it,
        // and giving it the sidewalk's material starts `contact.sidewalk_grade`
        // scoring hillsides against roads they pass near — measured, 0.31 % to
        // 2.70 %. Merging only where the relation exists is the same change
        // where it is true and none of it where it is not.
        let continues_a_strip = strip_m > 0.0;
        let surface = if continues_a_strip {
            priors::Surface::Walkway
        } else {
            priors::Surface::Path
        };
        let half = nominal * 0.5;
        let mut cursor = 0.0f64;
        for (w0, w1) in merged.into_iter().chain(std::iter::once((total, total))) {
            let lo = cursor.min(total);
            let hi = w0.clamp(lo, total);
            cursor = cursor.max(w1);
            if hi - lo <= MIN_FREE_M {
                continue;
            }
            let (stops, pts) = resample(&line.line, &arc, lo, hi, PATH_STATION_M);
            for (i, w) in pts.windows(2).enumerate() {
                out.push(SourceSeg {
                    a: w[0],
                    b: w[1],
                    cos_lat,
                    half_m: half,
                    sect_a: Section::uniform(half),
                    sect_b: Section::uniform(half),
                    level: 0,
                    layer: 0,
                    cut_a: None,
                    cut_b: None,
                    // A free band stands on the ground. The height field reads
                    // that off `NO_HOST`'s absent profile.
                    height_a: 0.0,
                    height_b: 0.0,
                    corridor: NO_HOST,
                    surface,
                    rise_m: 0.0,
                    arc0: stops[i],
                });
                sources.push(line.source);
            }
        }
    }
}

/// The stretches of one way, in its own arc, that the street strip already
/// drew.
///
/// **Per *linked group* of attachments, not per attachment.** The strip is
/// built over the *merged* claim (`SideClaims::spans`), so it covers the gaps
/// an attachment breaks at — a corner, a driveway — as well as the claims
/// themselves. Subtracting only each attachment's own range therefore leaves
/// the gap uncovered, and the way draws a second ribbon there, on its own
/// mapped polyline, a metre or two from the strip that is already covering it.
/// That is the parallel-ribbons defect reported from Territet.
///
/// Two attachments are linked when they are close **in both parameterizations**
/// — near in host arc *and* near in the way's own arc. Host arc alone is not
/// enough: a way can leave its street, wander two hundred metres, and come back
/// to the same kerb thirty metres along, and interpolating across that gap
/// would claim the whole excursion as drawn when none of it is.
fn covered_by_strip(
    attached: &[crate::assemble::walks::Attachment],
    built: &HashMap<(u32, u8), Vec<(f64, f64)>>,
) -> Vec<(f64, f64)> {
    let mut out: Vec<(f64, f64)> = Vec::new();
    let mut by_host: HashMap<(u32, u8), Vec<&crate::assemble::walks::Attachment>> = HashMap::new();
    for a in attached {
        by_host.entry((a.host, a.side)).or_default().push(a);
    }
    let mut keys: Vec<(u32, u8)> = by_host.keys().copied().collect();
    keys.sort_unstable();
    for key in keys {
        let Some(spans) = built.get(&key) else { continue };
        let mut atts = by_host.remove(&key).unwrap_or_default();
        atts.sort_by(|x, y| x.arc0.total_cmp(&y.arc0));
        // The knots of a monotone host-arc -> walk-arc map, one group at a time.
        let mut group: Vec<(f64, f64)> = Vec::new();
        let flush = |group: &mut Vec<(f64, f64)>, out: &mut Vec<(f64, f64)>| {
            if group.len() < 2 {
                group.clear();
                return;
            }
            let (lo_arc, hi_arc) = (group[0].0, group[group.len() - 1].0);
            for &(b0, b1) in spans {
                let (lo, hi) = (b0.max(lo_arc), b1.min(hi_arc));
                if hi <= lo {
                    continue;
                }
                let (w0, w1) = (lerp_knots(group, lo), lerp_knots(group, hi));
                out.push((w0.min(w1), w0.max(w1)));
            }
            group.clear();
        };
        for a in atts {
            if let Some(&(prev_arc, prev_walk)) = group.last() {
                let linked = a.arc0 - prev_arc <= priors::WALK_CORNER_MAX_M
                    && (a.walk0 - prev_walk).abs() <= priors::WALK_CORNER_MAX_M;
                if !linked {
                    flush(&mut group, &mut out);
                }
            }
            group.push((a.arc0, a.walk0));
            group.push((a.arc1, a.walk1));
        }
        flush(&mut group, &mut out);
    }
    out
}

/// A monotone piecewise-linear lookup over `(host arc, walk arc)` knots,
/// clamped outside their range.
fn lerp_knots(knots: &[(f64, f64)], at: f64) -> f64 {
    if at <= knots[0].0 {
        return knots[0].1;
    }
    for w in knots.windows(2) {
        let ((a0, w0), (a1, w1)) = (w[0], w[1]);
        if at <= a1 {
            let t = if a1 > a0 { (at - a0) / (a1 - a0) } else { 0.0 };
            return w0 + (w1 - w0) * t;
        }
    }
    knots[knots.len() - 1].1
}

/// Shortest free band worth a segment, in metres — a guard against zero-length
/// slivers where a claim ends a hair short of a vertex, not a claim about how
/// long a path has to be to exist.
const MIN_FREE_M: f64 = 0.4;

/// `ARPT_PROBE_WALK="lon,lat[,r_m]"`: for every corridor side claiming a
/// pavement within `r_m` (default 30) of the point, print what the data claimed
/// and what was built, so a bare spot in the render can be traced to the rule
/// that made it.
fn probe(
    scene: &SceneGraph,
    claims: &HashMap<(u32, u8), SideClaims>,
    built: &HashMap<(u32, u8), Vec<(f64, f64)>>,
) {
    let Some((lon, lat, r)) = std::env::var("ARPT_PROBE_WALK").ok().and_then(|s| {
        let v: Vec<f64> = s.split(',').filter_map(|t| t.trim().parse().ok()).collect();
        match v.as_slice() {
            [lon, lat] => Some((*lon, *lat, 30.0)),
            [lon, lat, r] => Some((*lon, *lat, *r)),
            _ => None,
        }
    }) else {
        return;
    };
    let p = Coord { x: lon, y: lat };
    let mut keys: Vec<(u32, u8)> = claims.keys().copied().collect();
    keys.sort_unstable();
    for key in keys {
        let (host, side) = key;
        let Some(c) = scene.corridors.get(host as usize) else { continue };
        let near = c
            .nodes
            .windows(2)
            .any(|w| probe_seg_dist_m(p, w[0], w[1], c.cos_lat) <= r);
        if !near {
            continue;
        }
        let cl = &claims[&key];
        let none = Vec::new();
        let made = built.get(&key).unwrap_or(&none);
        let want: f64 = cl.spans.iter().map(|&(a, b)| b - a).sum();
        let got: f64 = made.iter().map(|&(a, b)| b - a).sum();
        eprintln!(
            "[walk probe] corridor {host} ({}) side {side}: {} claims -> {} spans, \
             {want:.0} m claimed, {got:.0} m built in {} runs\n              \
             claimed {:?}\n              built   {:?}\n              at grade {:?}",
            c.class_key,
            cl.parts.len(),
            cl.spans.len(),
            made.len(),
            cl.spans.iter().map(|&(a, b)| (a.round(), b.round())).collect::<Vec<_>>(),
            made.iter().map(|&(a, b)| (a.round(), b.round())).collect::<Vec<_>>(),
            // The level runs, because "the strip stops here" and "the host is
            // on a bridge here" look identical from the outside.
            level_runs(c)
                .into_iter()
                .filter(|&(_, _, _, k)| k == SpanKind::Grade)
                .map(|(a, b, _, _)| (a.round(), b.round()))
                .collect::<Vec<_>>(),
        );
    }
}

/// What became of the extent the data claimed — the census that says how much
/// pavement the drawing lost after the relation was already won, and to which
/// rule. Under `ARPT_DEBUG_WALK`, printed by [`bands`].
#[derive(Default)]
struct AttachCensus {
    /// Claimed extent, merged, in metres of host arc.
    claimed_m: f64,
    /// …whose host has no drawable width at all.
    no_width_m: f64,
    /// …where the host is not on the ground (the structure carries any
    /// pavement there — `synth::carried`).
    non_grade_m: f64,
    /// …where the room left less than a strip worth drawing.
    narrow_m: f64,
    /// …that produced a strip segment.
    built_m: f64,
}

impl AttachCensus {
    fn report(&self) {
        if std::env::var_os("ARPT_DEBUG_WALK").is_none() || !(self.claimed_m > 0.0) {
            return;
        }
        let pct = |v: f64| 100.0 * v / self.claimed_m;
        eprintln!(
            "[walk] claimed {:>8.2} km of host arc:   built {:>5.1} %   no-width {:>4.1} %   \
             non-grade {:>4.1} %   narrow {:>4.1} %",
            self.claimed_m / 1000.0,
            pct(self.built_m),
            pct(self.no_width_m),
            pct(self.non_grade_m),
            pct(self.narrow_m),
        );
    }
}

/// How much of the face allowance a narrowed band aims at, so it lands inside
/// it rather than on it.
///
/// The width whose face is *exactly* the cap is what a straight line through
/// two samples predicts, and a hillside between them is not straight: aiming at
/// the cap itself recovered a sixth of the length the estimate said it would,
/// because every bit of roughness put the re-probe back over.
const FIT_MARGIN: f64 = 0.85;

/// Narrows every at-grade band to the width the ground under it can actually
/// carry, and drops the ones left under [`priors::WALK_MIN_WIDTH_M`].
///
/// **The third bound on a seat.** [`seat`] already allots a band out of the room
/// between its kerb and its facade; this is the same allotment against a third
/// constraint — the earthwork the material may plausibly build — and it is the
/// one bound that cannot be read from the plan, because it is a fact about the
/// ground. Hence the split in `ground::derive`: the seniors imprint first, the
/// band is fitted to them here, and stratum D then benches the band it was
/// given (`ground::walk_earthworks`). One cross-section, decided once.
///
/// **Why it is not the bench's job to narrow itself.** Measured both ways. A
/// bench that narrows while its band stays wide keeps the surface flat — the
/// refused length falls 16.8 % → 4.6 % and `slope.walk_crossfall` 22.5 → 8.5 %
/// — but it spends its verge doing it, and the verge is what guarantees the
/// drawn terrain's hole rim lands on flat ground rather than where the batter
/// starts: `contact.walk_rim` went 2.8 → 3.8 % for it. Narrowing the *band*
/// keeps the verge by construction, because the bench is derived from the band
/// after the fact and is always [`priors::EARTHWORK_MARGIN_M`] wider than it.
///
/// **A path on a flank is genuinely narrower**, which is what makes this a
/// model and not a dodge: two metres of promenade across a 45° hillside is a
/// claim about the world that a footpath there does not support, and where the
/// allowance leaves less than a band's minimum there is no band — the same rule
/// [`seat`] already applies to a street too narrow for a sidewalk, said once
/// against a different bound.
pub fn fit_to_ground(
    bands: &mut Vec<SourceSeg>,
    sources: &mut Vec<u64>,
    seniors: &[crate::ground::GroundLayer],
    terrain_path: Option<&std::path::Path>,
    z_ref: u8,
) {
    // The source list is per-segment and is dropped in lockstep below, so a
    // way whose every segment the fit declines leaves the set entirely — which
    // is exactly the question `pipeline::paves_via_walkway` asks of it.
    sources.resize(bands.len(), 0);
    // The A/B control: bands sized from the plan alone, as before. The seat
    // heights below are still read and stamped — they are a fact about the
    // model, not a product of the fit.
    let no_fit = std::env::var_os("ARPT_NO_WALK_FIT").is_some();
    // Only what is drawn at grade, matching the population the bench serves: a
    // band over a bridge is carried by the structure and has no ground under it
    // to be fitted to.
    let seats: Vec<usize> = (0..bands.len()).filter(|&i| bands[i].level == 0).collect();
    if seats.is_empty() {
        return;
    }
    let fitted = crate::ground::over_senior_ground(
        seats.len(),
        seniors,
        terrain_path,
        z_ref,
        |k, sample| {
            let s = &bands[seats[k]];
            // A hostless band's seat: the senior ground under its own two ends
            // — the same target the fit checks its faces against and stratum D
            // benches to. Read here, once, and stamped onto the band below: it
            // is built with `height_a`/`height_b` zero, and everything that
            // compares band heights — the walk sheet layering
            // (`synth::sheets`), the trench yields — needs the real seat, not
            // the zero.
            let ends = (s.corridor == NO_HOST).then(|| (sample(s.a), sample(s.b)));
            let half = if no_fit { None } else { fitted_half(s, ends, sample) };
            (half, ends)
        },
    );
    let mut drop: Vec<bool> = vec![false; bands.len()];
    let census = std::env::var_os("ARPT_DEBUG_WALK").is_some();
    // Length by what the fit did to it, so the cost of the rule is reported
    // rather than inferred: kept as it was, narrowed, or given up on.
    let mut by = [[0.0f64; 4]; 2]; // [path][kept, narrowed, sliver, dropped]
    // The end-to-end fall of every hostless band's seat, as a grade — the
    // population the wall-drape guard's ceiling is read off (see
    // [`fitted_half`]'s link check and the census report below).
    let mut falls: Vec<(f64, f64)> = Vec::new(); // (grade, len_m), Walkway only
    for (k, (half, ends)) in fitted.into_iter().enumerate() {
        let i = seats[k];
        if let Some((ha, hb)) = ends {
            bands[i].height_a = ha;
            bands[i].height_b = hb;
            if census && bands[i].surface == priors::Surface::Walkway {
                let len = crate::scene::metric_len(bands[i].a, bands[i].b, bands[i].cos_lat);
                if len > 1.0 {
                    falls.push(((ha - hb).abs() / len, len));
                }
            }
        }
        if no_fit {
            continue; // seats stamped, widths untouched, nothing dropped
        }
        let len = crate::scene::metric_len(bands[i].a, bands[i].b, bands[i].cos_lat);
        let path = usize::from(bands[i].corridor == NO_HOST);
        match half {
            Some(half) => {
                // A band whose interior is narrower than one rim reads
                // as a hairline rather than a surface (`PAVE_RIM_M`), and
                // `slope.walk_crossfall` cannot probe a metre across it. Counted
                // separately because that is the cost this floor is trading.
                let bucket = if half >= bands[i].drawn_half() - 1e-9 {
                    0
                } else if 2.0 * half >= 3.0 * priors::PAVE_RIM_M {
                    1
                } else {
                    2
                };
                by[path][bucket] += len;
                let half = priors::quantize_walk_width(half * 2.0) * 0.5;
                // **The section, never `half_m`.** The earthwork's narrowing is
                // a fact about the drawn edge, and `half_m` is what
                // `pavement::runs` chains a polyline on: writing it here is what
                // used to break a pavement into one buffered slab per width
                // rung. Same asymmetry as before — a band may give width up and
                // may never take it — said now in the field that carries it.
                bands[i].sect_a = Section::uniform(half.min(bands[i].sect_a.reach_m()));
                bands[i].sect_b = Section::uniform(half.min(bands[i].sect_b.reach_m()));
            }
            None => {
                by[path][3] += len;
                drop[i] = true;
            }
        }
    }
    if no_fit {
        return;
    }
    let mut i = 0;
    bands.retain(|_| {
        i += 1;
        !drop[i - 1]
    });
    let mut j = 0;
    sources.retain(|_| {
        j += 1;
        !drop[j - 1]
    });
    unify_width_along_ways(bands, sources);
    taper_along_runs(bands);
    // The joint weld is the walk graph's predecessor (`synth::walkgraph`
    // stamps every joint from one shared graph); it runs only under the
    // graph's revert switch so the two never fight over one seat.
    if std::env::var_os("ARPT_NO_WALK_GRAPH").is_some() {
        weld_joints(bands);
    }
    width_census(bands, sources);
    if census {
        for (path, name) in [(0usize, "sidewalk"), (1, "path")] {
            let t: f64 = by[path].iter().sum();
            if t <= 0.0 {
                continue;
            }
            eprintln!(
                "[walk] {name:<9} fit: {:>8.2} km   full {:>5.1} %   narrowed {:>5.1} %   \
                 hairline {:>4.1} %   dropped {:>4.1} %",
                t / 1000.0,
                100.0 * by[path][0] / t,
                100.0 * by[path][1] / t,
                100.0 * by[path][2] / t,
                100.0 * by[path][3] / t,
            );
        }
        if !falls.is_empty() {
            falls.sort_by(|a, b| a.0.total_cmp(&b.0));
            let q = |f: f64| falls[((falls.len() - 1) as f64 * f) as usize].0;
            let steep_m: f64 = falls.iter().filter(|(g, _)| *g > 0.5).map(|(_, l)| l).sum();
            eprintln!(
                "[walk] hostless walkway seat fall  n={}  p50 {:.3}  p90 {:.3}  p99 {:.3}  \
                 p999 {:.3}  max {:.3}   over 50 %: {:.0} m",
                falls.len(),
                q(0.50),
                q(0.90),
                q(0.99),
                q(0.999),
                q(1.0),
                steep_m,
            );
        }
    }
}

/// Makes the drawn width **continuous** along a run: a shared vertex takes the
/// narrower of the two segments' widths, so the ribbon tapers between them
/// instead of stepping.
///
/// **Bounded by construction, which is the whole point.** [`fit_to_ground`]
/// decides one width per *segment* — the widest the earthwork under that
/// segment can carry — and applying it as a flat value made the two sides of
/// every shared vertex disagree. A path across a flank therefore stepped
/// between 1.2 m and 2.0 m every station, which reads as a different object
/// each time (`street.width_step`). Taking the **min** at each vertex means the
/// width interpolated across a segment lies between two values that are both at
/// most that segment's own allowance, so it never exceeds it at any station: a
/// band may give width up and may never take it, said in a third place after
/// [`fit_to_ground`] and [`unify_width_along_ways`].
///
/// Run membership is the union's own chaining test (`synth::pavement::runs`)
/// less `half_m`, so exactly the vertices that will become interior vertices of
/// one buffered polyline are the ones made continuous.
fn taper_along_runs(bands: &mut [SourceSeg]) {
    if std::env::var_os("ARPT_NO_WALK_TAPER").is_some() {
        return; // the A/B control: one flat width per segment, as before
    }
    // Resolved first, applied after: assigning in place would let a narrowed
    // vertex propagate down the run and thin a whole path to its worst segment.
    let joins: Vec<Option<f64>> = (0..bands.len())
        .map(|i| {
            if i + 1 >= bands.len() {
                return None;
            }
            let (a, b) = (&bands[i], &bands[i + 1]);
            let chains = matches!(a.surface, priors::Surface::Walkway | priors::Surface::Path)
                && a.surface == b.surface
                && a.level == b.level
                && a.layer == b.layer
                && a.corridor == b.corridor
                && a.b == b.a;
            chains.then(|| a.sect_b.reach_m().min(b.sect_a.reach_m()))
        })
        .collect();
    for i in 0..bands.len() {
        if let Some(h) = joins[i] {
            bands[i].sect_b = Section::uniform(h);
            bands[i + 1].sect_a = Section::uniform(h);
        }
    }
}

/// **A way is drawn at one width — by narrowing to it, never by widening.**
/// The width most of its length carries; stretches drawn wider than that come
/// down to it, and stretches the ground holds *below* it keep what the ground
/// allows.
///
/// A street strip does not need this — its width is the room its facades leave,
/// which varies for a reason a viewer can see, and it reads as a taper. A
/// **free band** is the opposite case: nothing crowds a path across a hillside,
/// so the only thing varying its width is [`fit_to_ground`] resolving the
/// earthwork per *segment*, and a ribbon that pulses between 1.2 m and 2.0 m
/// every few metres reads as a different object each time. Measured at
/// Territet, which is where it was reported from: `path/track` p10 1.20 against
/// p50 2.00, and **26.9 % of ways varying along themselves** by up to 1.2 m.
///
/// **Widening was built, measured and rejected**, and the number is worth
/// keeping: letting a pinched stretch borrow one ladder rung (0.4 m) to match
/// its way took the varying share only 27.1 % → 25.8 %, and cost
/// `contact.walk_rim` 0.381 → 0.764 % with its worst 3.19 → **7.07 m**. The
/// bench is derived from these same segments, so a band drawn wider than the
/// ground was measured to carry gets a deeper batter face and a bigger step at
/// its own rim. A band may always give width up and may never take it.
///
/// **The section, never `half_m`.** That is the whole reason this could come
/// back after the cross-section rewrite deleted it: `pavement::runs` chains a
/// polyline on `half_m`, so the old version — which wrote the unified width
/// there — was itself breaking a way into one slab per rung while trying to
/// stop exactly that.
///
/// Chosen by *length*, not by segment count, so a long uniform stretch is not
/// outvoted by a scatter of short pinched ones.
fn unify_width_along_ways(bands: &mut [SourceSeg], sources: &[u64]) {
    if std::env::var_os("ARPT_NO_WALK_UNIFORM").is_some() {
        return; // the A/B control: width resolved per segment, as before
    }
    let key = |w: f64| (w * 1000.0).round() as u64;
    let mut by: HashMap<u64, HashMap<u64, f64>> = HashMap::new();
    for (s, &src) in bands.iter().zip(sources) {
        // Free bands only. A strip's width is the room its street leaves it,
        // and flattening that would draw a pavement through a wall.
        if src == 0 || s.corridor != NO_HOST {
            continue;
        }
        let len = crate::scene::metric_len(s.a, s.b, s.cos_lat);
        *by.entry(src).or_default().entry(key(s.drawn_half())).or_insert(0.0) += len;
    }
    // The width that carries the most length wins the way; ties go to the
    // wider, so a way split evenly does not thin for nothing.
    let target: HashMap<u64, f64> = by
        .into_iter()
        .filter_map(|(src, hist)| {
            let best = hist.into_iter().max_by(|a, b| a.1.total_cmp(&b.1).then(a.0.cmp(&b.0)))?;
            Some((src, best.0 as f64 / 1000.0))
        })
        .collect();
    for (s, &src) in bands.iter_mut().zip(sources) {
        if s.corridor != NO_HOST {
            continue;
        }
        let Some(&want) = target.get(&src) else { continue };
        s.sect_a = Section::uniform(want.min(s.sect_a.reach_m()));
        s.sect_b = Section::uniform(want.min(s.sect_b.reach_m()));
    }
}

/// Under `ARPT_DEBUG_WIDTH`, what the drawn pedestrian network's width
/// actually looks like — the diagnostic behind "why are these all different
/// sizes".
///
/// Two questions, because there are two kinds of non-uniformity and they have
/// different fixes. **Across** classes: what width does each material come
/// out at, which is the nominal ladder (`WALK_WIDTH_M`, `TRACK_WIDTH_M`,
/// `CROSSING_WIDTH_M`) plus whatever the seat and the fit took off it.
/// **Along** one way: how much a single mapped way's own width varies from
/// end to end, which is the seat and the fit deciding per *segment* — a way
/// that pulses between 0.8 m and 2.0 m along its length reads as a different
/// object every few metres however uniform the class table is.
/// Welds the pedestrian network's joints in height.
///
/// A hostless band seats on the ground under its own two ends; an attached
/// band rides its host street's kerb. Nothing reconciled the two where the
/// data joins them, so a path meeting a sidewalk on an embankment kept the
/// hillside's height while the sidewalk kept the street's, and the drawn band
/// cliffed the whole difference in the metre their buffers overlap
/// (`network.walk_joint`: 13.4 % of the zone's joints past 0.3 m, worst
/// 2.54 m — the user-visible "connected paths that do not join",
/// 2026-08-31). The junction weld is what streets get; this is the walk
/// network's, at its own scale: every free end takes the joint's
/// authoritative height — the attached bands' where any stands there, the
/// ends' mean otherwise — and the correction rides the band's own station to
/// its far end, which is the ramp a real path climbs an embankment with.
///
/// `ARPT_NO_WALK_WELD=1` withholds it.
fn weld_joints(bands: &mut [SourceSeg]) {
    if std::env::var_os("ARPT_NO_WALK_WELD").is_some() {
        return;
    }
    /// How far past a band's own drawn edge a free end still stands on it, in
    /// metres: the boolean kernel, quantization, and an endpoint a vertex
    /// short of the kerb line (`network.walk_joint` uses the same slack).
    const ON_M: f64 = 0.25;
    /// Past this the disagreement is a structure or a mismap, not a joint to
    /// close — the same boundary `crossings::SEPARATION_M` draws.
    const WELD_MAX_M: f64 = 3.0;
    use crate::assemble::grid::GridIndex;
    let mut grid = GridIndex::new();
    for (i, s) in bands.iter().enumerate() {
        if s.level != 0 {
            continue;
        }
        let pad_lat = (s.half_m + ON_M) / DEG_M;
        let pad_lon = pad_lat / s.cos_lat.max(1e-6);
        grid.insert(
            (
                s.a.x.min(s.b.x) - pad_lon,
                s.a.y.min(s.b.y) - pad_lat,
                s.a.x.max(s.b.x) + pad_lon,
                s.a.y.max(s.b.y) + pad_lat,
            ),
            i as u32,
        );
    }
    // The free ends, gathered first: the weld reads heights while it decides,
    // so it must not see its own writes (one pass, decisions from the
    // pre-weld state, deterministic in band order).
    struct Weld {
        band: u32,
        which: u8,
        target: f64,
    }
    let mut welds: Vec<Weld> = Vec::new();
    let mut cand: Vec<u32> = Vec::new();
    for (i, s) in bands.iter().enumerate() {
        if s.level != 0 || s.corridor != NO_HOST {
            continue; // the street's side of the joint is the authority
        }
        for (which, p, h) in [(0u8, s.a, s.height_a), (1u8, s.b, s.height_b)] {
            grid.query((p.x, p.y, p.x, p.y), &mut cand);
            let (mut att_sum, mut att_n) = (0.0f64, 0u32);
            let (mut oth_sum, mut oth_n) = (0.0f64, 0u32);
            for &j in cand.iter() {
                if j as usize == i {
                    continue;
                }
                let t = &bands[j as usize];
                // A chain-mate shares this exact endpoint bit-for-bit AND the
                // height there — both computed it from the same station, so
                // its vote is a no-op that dilutes the real neighbour's. A
                // band at the same point with a *different* height is the
                // opposite of a chain-mate: it is the joint itself. The
                // coordinate-only test excluded exactly the partner whenever
                // the map was clean — an attached walkway and a hostless path
                // meeting at one mapped node — which is how a 0.35 m step
                // survived the weld at 6.90932,46.43744 with att_n 0.
                if (t.a == p && (t.height_a - h).abs() <= 0.02)
                    || (t.b == p && (t.height_b - h).abs() <= 0.02)
                {
                    continue;
                }
                let (d, tt) = point_to_seg(p, t.a, t.b, s.cos_lat);
                let half =
                    t.sect_a.reach_m() + (t.sect_b.reach_m() - t.sect_a.reach_m()) * tt;
                if d > half + ON_M {
                    continue; // the end does not stand on this band
                }
                let th = t.height_at(tt);
                if t.corridor != NO_HOST {
                    att_sum += th;
                    att_n += 1;
                } else {
                    oth_sum += th;
                    oth_n += 1;
                }
            }
            if let Some(at) = std::env::var_os("ARPT_WELD_AT") {
                if let Some((plon, plat)) = at
                    .to_str()
                    .and_then(|v| v.split_once(','))
                    .and_then(|(a, b)| Some((a.trim().parse::<f64>().ok()?, b.trim().parse::<f64>().ok()?)))
                {
                    let d = crate::scene::metric_len(p, Coord { x: plon, y: plat }, s.cos_lat);
                    if d < 3.0 {
                        eprintln!(
                            "[weld] end {which} of band {i} at {:.6},{:.6} h {h:.2} att_n {att_n} oth_n {oth_n}",
                            p.x, p.y
                        );
                    }
                }
            }
            // The joint's authority: the attached bands where any stands
            // here — they ride their host street and the street is senior —
            // the other bands' mean otherwise (a through band has no end
            // here, keeps its height, and so becomes the authority of every
            // T-joint by construction).
            let target = if att_n > 0 {
                att_sum / att_n as f64
            } else if oth_n > 0 {
                oth_sum / oth_n as f64
            } else {
                continue; // alone: a true dead end owes nothing
            };
            let delta = target - h;
            if delta.abs() <= 0.02 || delta.abs() > WELD_MAX_M {
                continue; // already agreed, or not a joint at all
            }
            welds.push(Weld { band: i as u32, which, target });
        }
    }
    for w in &welds {
        let b = &mut bands[w.band as usize];
        // Every co-located end of the same chain must move with this one, or
        // the chain steps against itself where the weld begins — handled by
        // the caller order: consecutive segments share the endpoint
        // bit-for-bit, so both ends produce the same weld independently.
        if w.which == 0 {
            b.height_a = w.target;
        } else {
            b.height_b = w.target;
        }
    }
    if std::env::var_os("ARPT_DEBUG_WALK").is_some() {
        eprintln!("[walk] joint weld: {} free ends welded", welds.len());
    }
}

fn width_census(bands: &[SourceSeg], sources: &[u64]) {
    if std::env::var_os("ARPT_DEBUG_WIDTH").is_none() {
        return;
    }
    let q = |v: &mut Vec<f64>, f: f64| -> f64 {
        if v.is_empty() {
            return 0.0;
        }
        v.sort_by(f64::total_cmp);
        v[(((v.len() - 1) as f64) * f) as usize]
    };
    for (label, want_walk, hosted) in
        [("sidewalk", true, Some(true)), ("walk (hostless)", true, Some(false)), ("path/track", false, None)]
    {
        let mut w: Vec<f64> = bands
            .iter()
            .filter(|s| (s.surface == priors::Surface::Walkway) == want_walk)
            .filter(|s| hosted.is_none_or(|h| (s.corridor != NO_HOST) == h))
            .map(|s| s.drawn_half() * 2.0)
            .collect();
        if w.is_empty() {
            continue;
        }
        eprintln!(
            "[width] {label:<16} n={:<7} p10 {:.2}  p50 {:.2}  p90 {:.2}  min {:.2}  max {:.2}",
            w.len(),
            q(&mut w, 0.10),
            q(&mut w, 0.50),
            q(&mut w, 0.90),
            q(&mut w, 0.0),
            q(&mut w, 1.0),
        );
    }
    // **Is one way drawn as one kind of thing?** The reported defect at
    // Territet is "different kinds of path and track that are not consistent",
    // and the first question that answers is whether a single mapped way comes
    // out as one material at one height. A way that is part street strip
    // (`Walkway`, riding its host's kerb) and part free band (`Path`, on the
    // ground) draws as two objects with a rim between them, however
    // closely their colours are matched in the style.
    let mut kinds: HashMap<u64, [f64; 2]> = HashMap::new();
    for (s, &src) in bands.iter().zip(sources) {
        if src == 0 {
            continue;
        }
        let len = crate::scene::metric_len(s.a, s.b, s.cos_lat);
        // By *drawn material*, which is what the eye sees: a stretch that leaves
        // its street keeps the sidewalk's material and merges into its region, so
        // it is not a second kind of thing however hostless it is.
        let slot = usize::from(s.surface == priors::Surface::Path);
        kinds.entry(src).or_default()[slot] += len;
    }
    let mixed: Vec<&[f64; 2]> =
        kinds.values().filter(|v| v[0] > 1.0 && v[1] > 1.0).collect();
    let total_ways = kinds.len();
    let (strip_m, free_m): (f64, f64) =
        kinds.values().fold((0.0, 0.0), |acc, v| (acc.0 + v[0], acc.1 + v[1]));
    let mixed_m: f64 = mixed.iter().map(|v| v[0] + v[1]).sum();
    eprintln!(
        "[width] one way one kind  ways={total_ways:<6} walkway {:.1} km  path {:.1} km   \
         drawn as BOTH: {} ways ({:.1} %), {:.1} km ({:.1} % of drawn length)",
        strip_m / 1000.0,
        free_m / 1000.0,
        mixed.len(),
        100.0 * mixed.len() as f64 / total_ways.max(1) as f64,
        mixed_m / 1000.0,
        100.0 * mixed_m / (strip_m + free_m).max(1.0),
    );

    // Along one way: the spread of a single source's own widths.
    let mut by: std::collections::HashMap<u64, (f64, f64, u32)> = std::collections::HashMap::new();
    for (s, &src) in bands.iter().zip(sources) {
        if src == 0 {
            continue;
        }
        let e = by.entry(src).or_insert((f64::MAX, f64::MIN, 0));
        e.0 = e.0.min(s.sect_a.reach_m().min(s.sect_b.reach_m()) * 2.0);
        e.1 = e.1.max(s.drawn_half() * 2.0);
        e.2 += 1;
    }
    let mut spread: Vec<f64> = by.values().filter(|v| v.2 >= 3).map(|v| v.1 - v.0).collect();
    let n = spread.len();
    let varying = spread.iter().filter(|&&d| d > 0.05).count();
    eprintln!(
        "[width] along one way   n={n:<7} p50 {:.2}  p90 {:.2}  max {:.2}   varying by >5 cm: {:.1} %",
        q(&mut spread, 0.50),
        q(&mut spread, 0.90),
        q(&mut spread, 1.0),
        100.0 * varying as f64 / n.max(1) as f64,
    );
}

/// The narrowest band the fit may narrow *to*, in metres.
///
/// **A band narrower than its own two rims has no surface left to
/// draw.** `synth::pave_mesh` insets the silhouette by [`priors::PAVE_RIM_M`]
/// on each side and meshes the interior as the surface, so under 0.70 m a band
/// is pure rim and under about 1.05 m the interior is a hairline — which is
/// what [`priors::WALK_MIN_WIDTH_M`] at 0.8 m already permits, from the facade
/// room, and the first cut of this fit made common.
///
/// `ARPT_WALK_FIT_MIN` overrides it, which is how the number below was chosen
/// rather than guessed: `slope.walk_crossfall` reads the band's height a metre
/// inward from its own surface edge, so it is **blind to any band under
/// 1.70 m** — narrow enough bands leave the metric's population rather than its
/// offender set, and a fit that narrowed freely would have scored itself by
/// deleting the evidence.
fn min_width_m() -> f64 {
    std::env::var_os("ARPT_WALK_FIT_MIN")
        .and_then(|v| v.to_str()?.parse().ok())
        .unwrap_or(priors::WALK_MIN_WIDTH_M)
}

/// The half-width this segment's band can be given: the widest that keeps the
/// bench under it inside its material's face allowance, or `None` where even
/// the narrowest band worth drawing does not fit.
///
/// `ends` is a hostless band's seat — the senior ground under its two ends,
/// sampled once by the caller (which also stamps it onto the band); a hosted
/// band passes `None` and seats on `height_a`/`height_b`, the height its
/// street's cross-section draws it at.
///
/// Two probes at most, and the first answers for the great majority. The face
/// grows with the bench's width, so where the nominal verge already fits there
/// is nothing to decide; where it does not, the width whose face is the cap is
/// the cap's share of the one just measured, and the second probe reads the
/// ground there rather than trusting that estimate.
fn fitted_half(
    s: &SourceSeg,
    ends: Option<(f64, f64)>,
    sample: &mut dyn FnMut(Coord) -> f64,
) -> Option<f64> {
    let cos_lat = s.cos_lat;
    let (dx, dy) = ((s.b.x - s.a.x) * cos_lat, s.b.y - s.a.y);
    let len = (dx * dx + dy * dy).sqrt();
    if !(len > 0.0) {
        return Some(s.drawn_half()); // degenerate: the bench declines it anyway
    }
    let (px, py) = (-dy / len, dx / len); // lateral unit, metric (left)
    let mid = Coord { x: (s.a.x + s.b.x) * 0.5, y: (s.a.y + s.b.y) * 0.5 };
    // The same seat the bench will take, read at both ends
    // (`ground::walk_edge` says why never at the middle).
    let target = if let Some((ha, hb)) = ends {
        // **A link between two bands must be a link, not a wall.** A hostless
        // Walkway piece is a corner or a crossing stub — connective tissue
        // between two claimed stretches — and where the ground jumps more
        // than a storey within one segment it is not wrapping a corner, it
        // is draping across the bench cliff between a switchback's two arms
        // (6.9166,46.4338 is the type specimen: `slope.walk_crossfall`'s
        // worst read a 6.3 m step across 0.25 m of band there). No drawn
        // piece beats a wall.
        //
        // The allowance grows with length rather than vanishing past a link
        // ([`WALK_WALL_GRADE`]): a sidewalk leaving its street may fall a
        // storey over a hundred metres, and may not fall one over eight —
        // the length exemption that said otherwise approved an 8 m segment
        // draping 5.6 m down a trench-mouth wall (6.8932,46.4435).
        let len = crate::scene::metric_len(s.a, s.b, cos_lat);
        let allow = CORNER_STEP_M.max(len * WALK_WALL_GRADE);
        if s.surface == priors::Surface::Walkway && (ha - hb).abs() > allow {
            return None;
        }
        (ha + hb) * 0.5
    } else {
        (s.height_a + s.height_b) * 0.5
    };
    let cap = priors::bench_face_cap_m(s.surface);
    // The deepest deviation anywhere across the bench a half-width `w` would
    // carry — sampled at sub-metre steps, not at the two edges alone. The
    // edge pair jumps a wall standing *inside* the width: at a trench mouth
    // the outer probe lands on the neighbouring bench (the trench road's own
    // carriageway, at road height) and the inner on the strip's seat, both
    // near the target, and the strip is approved straight across the wall
    // between them — drawn, it falls eight metres across a quarter-metre of
    // band (6.8932,46.4435). On a monotone cross-slope the edge *is* the
    // deepest sample, so this only ever measures more, never less.
    let face = |w: f64, sample: &mut dyn FnMut(Coord) -> f64| -> f64 {
        let steps = (w / FACE_STEP_M).ceil().max(1.0) as i32;
        let mut worst = 0.0f64;
        for k in 1..=steps {
            let d = w * k as f64 / steps as f64;
            for side in [-1.0f64, 1.0] {
                let at = Coord {
                    x: mid.x + side * px * d / (DEG_M * cos_lat),
                    y: mid.y + side * py * d / DEG_M,
                };
                worst = worst.max((target - sample(at)).abs());
            }
        }
        worst
    };
    // The width the band is *drawn* at, not `half_m` — that is the run's
    // chaining key now and is the class nominal whatever the room left
    // (`SourceSeg::drawn_half_at`). Sizing the bench off it would bench a strip
    // wider than the one drawn, which is the widening trade this file already
    // measured and rejected.
    let nominal = s.drawn_half() + priors::EARTHWORK_MARGIN_M;
    let rise = face(nominal, sample);
    if rise <= cap {
        return Some(s.drawn_half());
    }
    // Never wider than it was asked to be, and never narrower than a band worth
    // drawing — below that the answer is no band at all rather than a sliver.
    //
    // **The floor never widens a band.** A band that arrives already narrower
    // than the floor was narrowed by [`seat`], out of the room its facades left
    // it; this fit only ever takes width away, so for such a band the floor is
    // its own width and the only question left is whether it benches at all.
    let floor = (min_width_m() * 0.5 + priors::EARTHWORK_MARGIN_M).min(nominal);
    let w = (nominal * cap * FIT_MARGIN / rise).clamp(floor, nominal);
    if face(w, sample) > cap {
        return None;
    }
    Some(w - priors::EARTHWORK_MARGIN_M)
}

/// The unit normal at station `i`, pointing to `side` (0 left, 1 right) — the
/// same handedness `assemble::walks` measured the offset with.
fn normal_at(pts: &[Coord], i: usize, cos_lat: f64, side: usize) -> (f64, f64) {
    let m_lon = DEG_M * cos_lat;
    let (j, k) = (i.saturating_sub(1), (i + 1).min(pts.len() - 1));
    let (dx, dy) = ((pts[k].x - pts[j].x) * m_lon, (pts[k].y - pts[j].y) * DEG_M);
    let len = dx.hypot(dy);
    if !(len > 0.0) {
        return (0.0, 0.0);
    }
    // Left of the direction of travel is `(-dy, dx)`; side 1 is the other.
    let s = if side == 0 { 1.0 } else { -1.0 };
    (-s * dy / len, s * dx / len)
}

/// `p` moved `d` metres along a local east/north unit vector.
fn offset(p: Coord, n: (f64, f64), d: f64, cos_lat: f64) -> Coord {
    let m_lon = DEG_M * cos_lat;
    Coord { x: p.x + n.0 * d / m_lon, y: p.y + n.1 * d / DEG_M }
}

/// Plan distance from `p` to the segment `a`–`b`, in metres — the probe's own,
/// so it needs nothing from its neighbours.
fn probe_seg_dist_m(p: Coord, a: Coord, b: Coord, cos_lat: f64) -> f64 {
    let m_lon = DEG_M * cos_lat;
    let (ex, ey) = ((b.x - a.x) * m_lon, (b.y - a.y) * DEG_M);
    let (qx, qy) = ((p.x - a.x) * m_lon, (p.y - a.y) * DEG_M);
    let len2 = ex * ex + ey * ey;
    let u = if len2 > 0.0 { ((qx * ex + qy * ey) / len2).clamp(0.0, 1.0) } else { 0.0 };
    (qx - ex * u).hypot(qy - ey * u)
}

/// The stretch `[from_m, to_m]` of a polyline, stationed at most `step_m`
/// apart and keeping every mapped vertex in between: the stations' arcs along
/// the way, and the points they fall at.
fn resample(
    line: &[Coord],
    arc: &[f64],
    from_m: f64,
    to_m: f64,
    step_m: f64,
) -> (Vec<f64>, Vec<Coord>) {
    let at = |s: f64| -> Coord {
        let i = arc.partition_point(|&a| a < s).clamp(1, line.len() - 1);
        let (a0, a1) = (arc[i - 1], arc[i]);
        let t = if a1 > a0 { (s - a0) / (a1 - a0) } else { 0.0 };
        Coord {
            x: line[i - 1].x + (line[i].x - line[i - 1].x) * t,
            y: line[i - 1].y + (line[i].y - line[i - 1].y) * t,
        }
    };
    let mut stops: Vec<f64> = vec![from_m];
    for (i, &a) in arc.iter().enumerate() {
        if a <= from_m || a >= to_m {
            continue;
        }
        // Keep the vertex, and fill the straight leading to it.
        let prev = stops[stops.len() - 1];
        let n = ((a - prev) / step_m).floor() as usize;
        for k in 1..=n {
            stops.push(prev + step_m * k as f64);
        }
        if a - stops[stops.len() - 1] > RUN_EPS_M {
            stops.push(a);
        }
        let _ = i;
    }
    let prev = stops[stops.len() - 1];
    let n = ((to_m - prev) / step_m).floor() as usize;
    for k in 1..=n {
        stops.push(prev + step_m * k as f64);
    }
    if to_m - stops[stops.len() - 1] > RUN_EPS_M {
        stops.push(to_m);
    }
    let pts = stops.iter().map(|&s| at(s)).collect();
    (stops, pts)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scene::Corridor;

    #[test]
    fn level_gate_is_permissive_only_when_there_is_nothing_to_measure() {
        // No anchor, or no profile: the plan test stays in charge. With both
        // present the gate is a hard CROSSING_LEVEL_M band.
        let solved = SolvedModel::empty(16);
        let p = Coord { x: 6.9, y: LAT };
        assert!(host_level_ok(&solved, 0, p, None));
        assert!(host_level_ok(&solved, 0, p, Some(390.0)), "no profile: permissive");
    }

    #[test]
    fn seg_intersection_finds_the_crossing_point() {
        let a0 = Coord { x: 0.0, y: -1.0 };
        let a1 = Coord { x: 0.0, y: 1.0 };
        let b0 = Coord { x: -1.0, y: 0.5 };
        let b1 = Coord { x: 1.0, y: 0.5 };
        let x = seg_intersection(a0, a1, b0, b1);
        assert!((x.x - 0.0).abs() < 1e-12 && (x.y - 0.5).abs() < 1e-12);
    }

    const LAT: f64 = 46.44;

    fn east(m: f64) -> f64 {
        m / (DEG_M * LAT.to_radians().cos())
    }

    fn line(from_m: f64, to_m: f64, north_m: f64) -> Vec<Coord> {
        vec![
            Coord { x: 6.9 + east(from_m), y: LAT + north_m / DEG_M },
            Coord { x: 6.9 + east(to_m), y: LAT + north_m / DEG_M },
        ]
    }

    /// A `SourceSeg` at the given plan positions and drawn heights.
    fn seg(from_m: f64, to_m: f64, north_m: f64, h: f64, corridor: CorridorId) -> SourceSeg {
        let pts = line(from_m, to_m, north_m);
        let half = priors::WALK_WIDTH_M * 0.5;
        SourceSeg {
            a: pts[0],
            b: pts[1],
            cos_lat: LAT.to_radians().cos(),
            half_m: half,
            sect_a: Section::uniform(half),
            sect_b: Section::uniform(half),
            level: 0,
            layer: 0,
            cut_a: None,
            cut_b: None,
            height_a: h,
            height_b: h,
            corridor,
            surface: priors::Surface::Walkway,
            rise_m: if corridor == NO_HOST { 0.0 } else { priors::KERB_RISE_M },
            arc0: 0.0,
        }
    }

    /// A wall standing *inside* the band's width must fail the face probe.
    /// The old two-point probe read only the edges: one landed on the strip's
    /// own seat and the other on the bench beyond the wall, both near the
    /// target, and the strip was approved straight across the drop.
    #[test]
    fn a_wall_inside_the_width_fails_the_face() {
        let s = seg(0.0, 8.0, 0.0, 400.0, 5);
        // Ground: at the target on the seat line, a 6 m trench from 1.2 m to
        // 2.2 m north, and the neighbouring bench at the target again beyond —
        // the exact geometry that blinded the two-point probe: both edges land
        // on bench-height ground and the wall between them goes unseen.
        let mut sample = |c: Coord| -> f64 {
            let north_m = (c.y - LAT) * crate::scene::DEG_M;
            if north_m > 1.2 && north_m <= 2.2 {
                394.0
            } else {
                400.0
            }
        };
        let got = fitted_half(&s, None, &mut sample);
        // The nominal reaches past 1.2 m, so the wall is inside the width and
        // the fit must either refuse or narrow to keep the bench off it.
        match got {
            None => {}
            Some(half) => assert!(
                half <= 1.2,
                "a 6 m wall 1.2 m out must not be inside the approved half ({half:.2})"
            ),
        }
    }

    /// **A walkway may fall a storey over a hundred metres and may not fall
    /// one over eight.** The wall-drape guard ([`WALK_WALL_GRADE`]): an 8 m
    /// hostless segment whose seat falls 5.6 m is a wall and is refused; the
    /// same fall over 100 m is a hillside sidewalk and stands.
    #[test]
    fn a_seat_falling_a_storey_over_eight_metres_is_a_wall() {
        let mut sample = |_: Coord| 400.0;
        let steep = seg(0.0, 8.0, 0.0, 0.0, NO_HOST);
        assert!(
            fitted_half(&steep, Some((400.0, 394.4)), &mut sample).is_none(),
            "a 70 % walkway must be refused"
        );
        let long = seg(0.0, 100.0, 0.0, 0.0, NO_HOST);
        assert!(
            fitted_half(&long, Some((400.0, 394.4)), &mut sample).is_some(),
            "a 5.6 % hillside sidewalk must stand"
        );
    }

    /// The defect this fixes: `fitted_half` seats a `NO_HOST` band on the
    /// ground and a hosted one on `height_a`/`height_b`, so an unseated stub
    /// met the pavement in plan and nowhere in section.
    #[test]
    fn a_kerb_stub_takes_the_height_of_the_band_it_continues() {
        // A pavement at 384.3 running east, and a stub running north off its
        // far end — the strip between the kerb and the crossing.
        let mut bands = vec![seg(0.0, 20.0, 0.0, 384.3, 5), seg(20.0, 21.5, 0.0, 0.0, NO_HOST)];
        seat_stubs(&mut bands, 1);
        assert_eq!(bands[1].corridor, 5, "the stub joins its pavement's street");
        assert!((bands[1].height_a - 384.3).abs() < 1e-6);
        assert!((bands[1].height_b - 384.3).abs() < 1e-6);
        assert!((bands[1].rise_m - priors::KERB_RISE_M).abs() < 1e-9, "and rides its kerb");
    }

    /// A crossing onto a path, or onto a pavement the fit declined, finds
    /// nothing to seat on and must drape exactly as it did before.
    #[test]
    fn a_stub_with_no_band_in_reach_stays_hostless() {
        let mut bands = vec![
            seg(0.0, 20.0, 0.0, 384.3, 5),
            seg(0.0, 1.5, STUB_SEAT_REACH_M + 5.0, 0.0, NO_HOST),
        ];
        seat_stubs(&mut bands, 1);
        assert_eq!(bands[1].corridor, NO_HOST);
        assert_eq!(bands[1].height_a, 0.0);
    }

    /// The reach must not step across a carriageway to the pavement opposite.
    #[test]
    fn a_stub_seats_on_the_nearer_of_two_pavements() {
        let mut bands = vec![
            seg(0.0, 20.0, 0.0, 384.3, 5),
            seg(0.0, 20.0, 2.0, 390.0, 9),
            seg(10.0, 10.0 + 1.5, 0.4, 0.0, NO_HOST),
        ];
        seat_stubs(&mut bands, 2);
        assert_eq!(bands[2].corridor, 5, "0.4 m away beats 1.6 m away");
        assert!((bands[2].height_a - 384.3).abs() < 1e-6);
    }

    #[test]
    fn two_claims_a_corner_apart_are_one_pavement() {
        // The corner rule, now the merge rather than a special case: a break
        // at a side street's mouth does not end the pavement.
        let parts = vec![(0.0, 49.0, 1u64), (51.0, 100.0, 2u64)];
        assert_eq!(merge_spans(&parts), vec![(0.0, 100.0)]);
        // Past the corner allowance they are two pavements with a real gap.
        let far = vec![(0.0, 49.0, 1u64), (49.0 + priors::WALK_CORNER_MAX_M + 1.0, 100.0, 2u64)];
        assert_eq!(merge_spans(&far).len(), 2);
    }

    #[test]
    fn a_segment_is_attributed_to_the_way_that_witnessed_it() {
        let cl = SideClaims {
            spans: vec![(0.0, 100.0)],
            parts: vec![(0.0, 40.0, 7), (60.0, 100.0, 9)],
        };
        assert_eq!(cl.owner(10.0), 7, "inside the first claim");
        assert_eq!(cl.owner(80.0), 9, "inside the second");
        // Inside the filled gap, the nearer of the two it links — so a corner
        // costs neither way its stroke.
        assert_eq!(cl.owner(45.0), 7);
        assert_eq!(cl.owner(55.0), 9);
        assert!(cl.covers(50.0), "the merged span is drawn end to end");
    }

    #[test]
    fn the_normal_points_to_the_side_the_way_is_on() {
        // An east-running line: side 0 (left) is north.
        let pts = vec![Coord { x: 6.9, y: LAT }, Coord { x: 6.9 + east(10.0), y: LAT }];
        let n = normal_at(&pts, 0, LAT.to_radians().cos(), 0);
        assert!(n.1 > 0.9, "left of east is north: {n:?}");
        let n = normal_at(&pts, 0, LAT.to_radians().cos(), 1);
        assert!(n.1 < -0.9, "right of east is south: {n:?}");
    }

    #[test]
    fn the_strip_keeps_one_half_m_however_much_the_room_takes() {
        // The whole of P2 as a property. `pavement::runs` chains a polyline
        // only while `half_m` matches, so a strip that varied it there was
        // drawn as one slab per width rung.
        let c = corridor();
        let stops = vec![0.0, 10.0, 20.0, 30.0];
        let pts: Vec<Coord> = stops.iter().map(|&s| Coord { x: 6.9 + east(s), y: LAT }).collect();
        // A wall stepping in over the run, so the allotment genuinely varies.
        let wall = Facades::from_edges([
            [
                Coord { x: 6.9 + east(12.0), y: LAT + 4.6 / DEG_M },
                Coord { x: 6.9 + east(40.0), y: LAT + 4.6 / DEG_M },
            ],
        ]);
        let want = vec![[priors::WALK_WIDTH_M, 0.0]; stops.len()];
        let mut scratch = Vec::new();
        let sections = super::super::cross::sections_along(
            &c, &stops, &pts, 3.0, &want, &wall, false, &mut scratch,
        );
        let widths: Vec<f64> = sections.iter().map(|s| s.walk[0]).collect();
        assert!(
            widths.iter().any(|&w| w > 0.0) && widths.windows(2).any(|w| w[0] != w[1]),
            "the fixture must actually vary: {widths:?}"
        );
        // Whatever the room did, the strip's inner edge is the kerb.
        for s in &sections {
            assert_eq!(s.walk_centre(0) - s.walk[0] * 0.5, s.carriage.on(0));
        }
    }

    /// A crossing spans the street it crosses; one drawn beside a street
    /// crosses nothing, however much of that street lies under it.
    ///
    /// This is the distinction `street.crossing_extent` scores and the
    /// registration used not to make: `on_asphalt` asked only whether there was
    /// asphalt underfoot, so a crossing along a station forecourt's service
    /// roads annexed them and painted a ladder the length of the line.
    #[test]
    fn a_crossing_crosses_a_street_it_does_not_run_beside_one() {
        // A street running east-west along y = LAT.
        let mut scene = SceneGraph::default();
        scene.corridors = vec![corridor()];
        let mut grid = crate::assemble::grid::GridIndex::with_cell_m(64.0);
        let mut edges: Vec<(u32, u32)> = Vec::new();
        for i in 0..scene.corridors[0].nodes.len() - 1 {
            let (a, b) = (scene.corridors[0].nodes[i], scene.corridors[0].nodes[i + 1]);
            grid.insert(
                (a.x.min(b.x), a.y.min(b.y), a.x.max(b.x), a.y.max(b.y)),
                edges.len() as u32,
            );
            edges.push((0, i as u32));
        }
        let mut scratch: Vec<u32> = Vec::new();

        // Across it: a north-south line through x = 25 m, from 6 m south to
        // 6 m north. It properly intersects the centerline.
        let across = vec![
            Coord { x: 6.9 + east(25.0), y: LAT - 6.0 / DEG_M },
            Coord { x: 6.9 + east(25.0), y: LAT + 6.0 / DEG_M },
        ];
        let arc = crate::scene::cumulative_arc(&across);
        let total = *arc.last().expect("two points");
        let at = |t: f64| -> Coord {
            if t < 0.0 {
                end_extension(&across, LAT.to_radians().cos(), false, -t)
            } else if t > total {
                end_extension(&across, LAT.to_radians().cos(), true, t - total)
            } else {
                point_on(&across, &arc, t)
            }
        };
        let hosts = crossed_hosts(&at, total, &scene, &SolvedModel::empty(16), None, &grid, &edges, &mut scratch);
        assert_eq!(hosts.len(), 1, "a line across the street crosses it: {hosts:?}");
        assert_eq!(hosts[0].0, 0);
        // The intersection sits mid-street: 25 m along a 50 m corridor.
        assert!((hosts[0].1[0] - 25.0).abs() < 1.0, "arc {:?}", hosts[0].1);

        // Beside it: a line parallel to the street, one metre off its
        // centerline, so every point of it is on the asphalt and none of it
        // crosses anything.
        let beside = vec![
            Coord { x: 6.9 + east(10.0), y: LAT + 1.0 / DEG_M },
            Coord { x: 6.9 + east(40.0), y: LAT + 1.0 / DEG_M },
        ];
        let arc = crate::scene::cumulative_arc(&beside);
        let total = *arc.last().expect("two points");
        let at = |t: f64| -> Coord {
            if t < 0.0 {
                end_extension(&beside, LAT.to_radians().cos(), false, -t)
            } else if t > total {
                end_extension(&beside, LAT.to_radians().cos(), true, t - total)
            } else {
                point_on(&beside, &arc, t)
            }
        };
        assert!(
            on_asphalt(&scene, &SolvedModel::empty(16), None, &[(0, vec![25.0])], at(total * 0.5)),
            "the fixture must lie on the asphalt, or it proves nothing"
        );
        let hosts = crossed_hosts(&at, total, &scene, &SolvedModel::empty(16), None, &grid, &edges, &mut scratch);
        assert!(hosts.is_empty(), "a line beside the street crosses nothing: {hosts:?}");
    }

    fn corridor() -> Corridor {
        let nodes = vec![Coord { x: 6.9, y: LAT }, Coord { x: 6.9 + east(50.0), y: LAT }];
        Corridor {
            id: 0,
            arc: crate::scene::cumulative_arc(&nodes),
            nodes,
            cos_lat: LAT.to_radians().cos(),
            kind: Kind::Road(RoadClass::Residential),
            class_key: "residential".into(),
            link: false,
            width_m: Some(5.5),
            spans: Vec::new(),
            segments: Vec::new(),
            connectors: Vec::new(),
        }
    }
}
