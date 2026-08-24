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
use crate::assemble::walks::{Attachment, WalkLine};
use crate::priors;
use crate::scene::{Corridor, CorridorId, SceneGraph, SpanKind, DEG_M};
use crate::solve::SolvedModel;

use super::carriageway::{level_runs, sections_along, GradeRun, SourceSeg, RUN_EPS_M};

/// The corridor id a band with no host carries. Nothing resolves it to a
/// profile (`SolvedModel::profile` answers `None` past the end of its list),
/// which is exactly right: a path across a field has no road to ride, so its
/// height is the ground.
const NO_HOST: CorridorId = CorridorId::MAX;

/// Station spacing along an unattached path, in metres. The band follows the
/// way's own mapped line there, so this only bounds how coarsely a long
/// straight is buffered; an attached band is stationed by its host's profile
/// instead, like the carriageway.
const PATH_STATION_M: f64 = 8.0;

/// Shortest band worth building, in metres. Below this the buffer is a blob
/// whose casing is most of it.
const MIN_BAND_M: f64 = 4.0;

/// How far a crossing's mapped line is extended past each end when it is
/// registered against the carriageways, in metres. A crosswalk is mapped by
/// hand across the road it crosses and its endpoints rarely lie on the kerbs
/// (docs/ROADS.md §2); the extension lets the registration find the asphalt
/// the mapped stub stops short of. It bounds search, not paint — the painted
/// extent is exactly the on-asphalt interval, wherever the mapped ends are.
const CROSSING_EXTEND_M: f64 = 8.0;

/// Registration sampling step along a crossing line, in metres — fine enough
/// that a kerb lands within a step of where the buffer test says.
const CROSSING_STEP_M: f64 = 0.25;

/// Two on-asphalt intervals closer than this merge, in metres: a lane
/// divider's worth of noise is not a refuge island.
const CROSSING_MERGE_M: f64 = 1.0;

/// Shortest on-asphalt interval that counts as a crossed carriageway, in
/// metres — under it the line grazed an edge rather than crossed a road.
const CROSSING_MIN_CHORD_M: f64 = 1.5;

/// Shortest kerb stub worth a band, in metres. Under it the strip is inside
/// the quantization of the kerb itself.
const CROSSING_MIN_STUB_M: f64 = 0.4;

/// One crosswalk, registered: the kerb-to-kerb chords its paint spans. The
/// stubs — the stretches of the *mapped* line outside every chord — are
/// returned by [`crossings`] as ordinary band segments instead, because they
/// are the strip of real sidewalk between the kerb and whatever the crossing
/// joins.
pub struct CrossingPaint {
    /// Source hash of the crosswalk feature, for the phase-1 lookup.
    pub source: u64,
    /// Kerb-to-kerb chords over drawn asphalt, in walk order along the line.
    /// More than one where the crossing spans a divided carriageway: the gap
    /// between them is the refuge island, and it is not painted.
    pub chords: Vec<(Coord, Coord)>,
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
pub fn crossings(scene: &SceneGraph) -> (Vec<CrossingPaint>, Vec<SourceSeg>) {
    let mut paints = Vec::new();
    let mut stubs = Vec::new();
    if std::env::var_os("ARPT_NO_CROSSING").is_some() {
        return (paints, stubs);
    }
    // Every drivable carriageway edge, indexed by plan position — the same
    // hosts `assemble::walks` attaches against, indexed the same way.
    let mut grid = crate::assemble::grid::GridIndex::with_cell_m(64.0);
    let mut edges: Vec<(u32, u32)> = Vec::new();
    let mut reach_m = 0.0f64;
    for (ci, c) in scene.corridors.iter().enumerate() {
        if c.kind.prior().surface != priors::Surface::Asphalt {
            continue;
        }
        let Some(half) = super::carriageway::corridor_half_width_m(c) else { continue };
        reach_m = reach_m.max(half);
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
        // On-asphalt intervals of the extended line.
        let mut on: Vec<(f64, f64)> = Vec::new();
        let mut t = -CROSSING_EXTEND_M;
        let mut open: Option<f64> = None;
        while t <= total + CROSSING_EXTEND_M + 1e-9 {
            let p = at(t);
            let inside = on_asphalt(scene, &grid, &edges, reach_m, p, cos_lat, &mut scratch);
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
        paints.push(CrossingPaint {
            source: line.source,
            chords: merged.iter().map(|&(s, e)| (at(s), at(e))).collect(),
        });
    }
    (paints, stubs)
}

/// Whether a point lies on a drawn carriageway: within some corridor's own
/// drawn half-width of its centerline.
fn on_asphalt(
    scene: &SceneGraph,
    grid: &crate::assemble::grid::GridIndex,
    edges: &[(u32, u32)],
    reach_m: f64,
    p: Coord,
    cos_lat: f64,
    scratch: &mut Vec<u32>,
) -> bool {
    let (rx, ry) = (reach_m / (DEG_M * cos_lat), reach_m / DEG_M);
    grid.query((p.x - rx, p.y - ry, p.x + rx, p.y + ry), scratch);
    for &e in scratch.iter() {
        let (ci, ni) = edges[e as usize];
        let c = &scene.corridors[ci as usize];
        let Some(half) = super::carriageway::corridor_half_width_m(c) else { continue };
        let (a, b) = (c.nodes[ni as usize], c.nodes[ni as usize + 1]);
        if probe_seg_dist_m(p, a, b, c.cos_lat) <= half {
            return true;
        }
    }
    false
}

/// A point `over_m` past one end of a polyline, along that end's tangent.
fn end_extension(line: &[Coord], cos_lat: f64, at_end: bool, over_m: f64) -> Coord {
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

/// Builds every walkway band in the extract: the geometry, the seat and the
/// material, with no grade layer yet.
///
/// **Model, not drawing, and that is why it is split from [`stamp_layers`].**
/// The band is derived once, before stage 3, because the ground under a
/// walkway is that walkway (`ground::walk_earthworks`) and a bench derived
/// from a *second* construction of the same band would be a bench that does
/// not fit it — the sliver family this codebase keeps re-learning. One
/// derivation, two readers: the ground benches it, and the union draws it.
///
/// They are *not* handed to `synth::sheets`: a band's sheet is its host's, and
/// letting a sidewalk vote on the grade-separation layering would let the thing
/// standing on a surface decide what that surface is.
/// Builds every walkway band, and — in step with it — the **source each
/// segment came from**.
///
/// The second vector is what lets the drawing keep its promise of graceful
/// degradation (docs/GENERATION.md I6). A pedestrian way's cartographic
/// stroke is deleted at the walk zooms because the band *is* the surface
/// (`pipeline::paves_via_walkway`), and that test was the class and nothing
/// else — so a way whose band the ground fit declined to build lost its
/// stroke too and vanished from the map entirely. On a steep flank that is
/// exactly where it happens: the Territet switchback at 6.9189,46.4304 came
/// out as a handful of disjoint slabs with nothing between them. Carrying the
/// source per segment lets phase 1 ask *was anything actually drawn for this
/// feature* rather than *is this feature the kind of thing we draw*.
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
    let mut scratch: Vec<u32> = Vec::new();
    let mut census = AttachCensus::default();
    // `ARPT_PROBE_WALK="lon,lat[,r_m]"`: for every pedestrian line passing
    // within `r_m` (default 30) of the point, print what the model made of it
    // — the attachments it won and the band segments that actually came out —
    // so a bare spot in the render can be traced to the rule that made it.
    let probe = std::env::var("ARPT_PROBE_WALK").ok().and_then(|s| {
        let v: Vec<f64> = s.split(',').filter_map(|t| t.trim().parse().ok()).collect();
        match v.as_slice() {
            [lon, lat] => Some((*lon, *lat, 30.0)),
            [lon, lat, r] => Some((*lon, *lat, *r)),
            _ => None,
        }
    });
    for (li, (line, attached)) in scene.walks.lines().enumerate() {
        if line.crosswalk || line.line.len() < 2 {
            continue; // a crossing is paint on the carriageway, not a band
        }
        let near_probe = probe.map(|(lon, lat, r)| {
            let cos_lat = crate::scene::run_cos_lat(&line.line);
            line.line.windows(2).any(|w| {
                probe_seg_dist_m(Coord { x: lon, y: lat }, w[0], w[1], cos_lat) <= r
            })
        });
        if near_probe == Some(true) {
            eprintln!(
                "[walk probe] line {li} source {:x} kind {:?} tagged {} len {:.0} m  \
                 spans {:?}",
                line.source,
                line.kind,
                line.tagged,
                crate::scene::cumulative_arc(&line.line).last().unwrap_or(&0.0),
                line.spans,
            );
        }
        let before = out.len();
        // Which attachments produced a band at all. A gap is only a *corner*
        // when what bounds it is drawn: a claim that built nothing — the host
        // on a structure, the seat out of room — is no claim, and the stretch
        // between two of those is the middle of a bare run, not a link (the
        // Glion gallery sidewalk grew ground-level "corners" under a host
        // 13 m overhead this way).
        let mut built: Vec<(f64, f64)> = Vec::new();
        for a in attached {
            let b0 = out.len();
            attached_band(scene, solved, facades, a, &mut out, &mut scratch, &mut census);
            if out.len() > b0 {
                built.push((a.walk0, a.walk1));
            }
            if near_probe == Some(true) {
                eprintln!(
                    "  attach host {} side {} walk {:.0}..{:.0} arc {:.0}..{:.0} \
                     offset {:.1} spread {:.1} {:?} -> {} segs",
                    a.host,
                    a.side,
                    a.walk0,
                    a.walk1,
                    a.arc0,
                    a.arc1,
                    a.offset_m,
                    a.spread_m,
                    a.evidence,
                    out.len() - b0,
                );
            }
        }
        let banded = out.len();
        path_bands(line, attached, &built, &mut out);
        if near_probe == Some(true) {
            eprintln!(
                "  built: {} attached segs, {} path segs",
                banded - before,
                out.len() - banded
            );
        }
        sources.resize(out.len(), line.source);
    }
    census.report();
    (out, sources)
}

/// What became of the host arc the attachments claimed — the census that says
/// how much sidewalk the drawing lost after the relation was already won, and
/// to which rule. Under `ARPT_DEBUG_WALK`, printed by [`bands`].
#[derive(Default)]
struct AttachCensus {
    /// Host arc claimed by attachments, in metres.
    claimed_m: f64,
    /// …whose host has no drawable width at all.
    no_width_m: f64,
    /// …where the host is not on the ground (the structure carries any
    /// sidewalk there — `synth::carried`).
    non_grade_m: f64,
    /// …in stretches under [`MIN_BAND_M`].
    short_m: f64,
    /// …where the seat found no room between kerb and facade.
    no_room_m: f64,
    /// …where the room left less than a band worth drawing.
    narrow_m: f64,
    /// …that produced a band segment.
    built_m: f64,
}

impl AttachCensus {
    fn report(&self) {
        if std::env::var_os("ARPT_DEBUG_WALK").is_none() || !(self.claimed_m > 0.0) {
            return;
        }
        let pct = |v: f64| 100.0 * v / self.claimed_m;
        eprintln!(
            "[walk] attached {:>8.2} km of host arc:   built {:>5.1} %   no-width {:>4.1} %   \
             non-grade {:>4.1} %   short {:>4.1} %   no-room {:>4.1} %   narrow {:>4.1} %",
            self.claimed_m / 1000.0,
            pct(self.built_m),
            pct(self.no_width_m),
            pct(self.non_grade_m),
            pct(self.short_m),
            pct(self.no_room_m),
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
    if std::env::var_os("ARPT_NO_WALK_FIT").is_some() {
        return; // the A/B control: bands sized from the plan alone, as before
    }
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
        |k, sample| fitted_half(&bands[seats[k]], sample),
    );
    let mut drop: Vec<bool> = vec![false; bands.len()];
    let census = std::env::var_os("ARPT_DEBUG_WALK").is_some();
    // Length by what the fit did to it, so the cost of the rule is reported
    // rather than inferred: kept as it was, narrowed, or given up on.
    let mut by = [[0.0f64; 4]; 2]; // [path][kept, narrowed, sliver, dropped]
    for (k, half) in fitted.into_iter().enumerate() {
        let i = seats[k];
        let len = crate::scene::metric_len(bands[i].a, bands[i].b, bands[i].cos_lat);
        let path = usize::from(bands[i].corridor == NO_HOST);
        match half {
            Some(half) => {
                // A band whose interior is narrower than one casing rim reads
                // as a dark hairline rather than a surface (`PAVE_RIM_M`), and
                // `slope.walk_crossfall` cannot probe a metre across it. Counted
                // separately because that is the cost this floor is trading.
                let bucket = if half >= bands[i].half_m - 1e-9 {
                    0
                } else if 2.0 * half >= 3.0 * priors::PAVE_RIM_M {
                    1
                } else {
                    2
                };
                by[path][bucket] += len;
                let half = priors::quantize_walk_width(half * 2.0) * 0.5;
                bands[i].half_m = half;
                bands[i].sect_a = Section::uniform(half);
                bands[i].sect_b = Section::uniform(half);
            }
            None => {
                by[path][3] += len;
                drop[i] = true;
            }
        }
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
    }
}

/// **A way is drawn at one width — by narrowing to it, never by widening.**
/// The width most of its length carries; stretches drawn wider than that come
/// down to it, and stretches the ground holds *below* it keep what the ground
/// allows.
///
/// This is the last of the three sources of "why is this path a different
/// size every few metres". The class nominal is one number
/// ([`priors::WALK_WIDTH_M`]) and the ladder
/// ([`priors::quantize_walk_width`]) removes the fine jitter, but the room a
/// band is allotted still varies along a street — a facade steps in, a flank
/// steepens — and resolved per station that draws as a ribbon that keeps
/// changing size for reasons a viewer cannot see. Measured before any of
/// this: **31.9 % of ways varied along themselves, p90 by 1.23 m.**
///
/// **Widening was built, measured and rejected**, and the number is worth
/// keeping: letting a pinched stretch borrow one ladder rung (0.4 m) to match
/// its way took the varying share only 27.1 % → 25.8 %, and cost
/// `contact.walk_rim` 0.381 → 0.764 % with its worst 3.19 → **7.07 m**. The
/// mechanism is not subtle — the bench is derived from these same segments,
/// so a band drawn wider than the ground was measured to carry gets a deeper
/// batter face and a bigger step at its own rim. A band may always give width
/// up and may never take it, which is the same asymmetry
/// [`fit_to_ground`] is built on.
///
/// So the residual variation is the ground fit doing its job: a path on a
/// steep flank is genuinely narrower than a promenade, and forcing it wide
/// gives back the fix that took `slope.walk_crossfall` from 22.5 % to 6.3 %.
///
/// Chosen by *length*, not by segment count, so a long uniform stretch is not
/// outvoted by a scatter of short pinched ones.
fn unify_width_along_ways(bands: &mut [SourceSeg], sources: &[u64]) {
    if std::env::var_os("ARPT_NO_WALK_UNIFORM").is_some() {
        return; // the A/B control: width resolved per station, as before
    }
    // Length carried at each quantized width, per way.
    let mut by: std::collections::HashMap<u64, std::collections::HashMap<u64, f64>> =
        std::collections::HashMap::new();
    let key = |w: f64| (w * 1000.0).round() as u64;
    for (s, &src) in bands.iter().zip(sources) {
        if src == 0 {
            continue;
        }
        let len = crate::scene::metric_len(s.a, s.b, s.cos_lat);
        *by.entry(src).or_default().entry(key(s.half_m)).or_insert(0.0) += len;
    }
    // The width that carries the most length wins the way; ties go to the
    // wider, so a way split evenly does not thin for nothing.
    let target: std::collections::HashMap<u64, f64> = by
        .into_iter()
        .filter_map(|(src, hist)| {
            let best = hist
                .into_iter()
                .max_by(|a, b| a.1.total_cmp(&b.1).then(a.0.cmp(&b.0)))?;
            Some((src, best.0 as f64 / 1000.0))
        })
        .collect();
    for (s, &src) in bands.iter_mut().zip(sources) {
        let Some(&want) = target.get(&src) else { continue };
        if want >= s.half_m - 1e-9 {
            continue; // never take width, only give it
        }
        s.half_m = want;
        s.sect_a = Section::uniform(want);
        s.sect_b = Section::uniform(want);
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
            .map(|s| s.half_m * 2.0)
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
    // Along one way: the spread of a single source's own widths.
    let mut by: std::collections::HashMap<u64, (f64, f64, u32)> = std::collections::HashMap::new();
    for (s, &src) in bands.iter().zip(sources) {
        if src == 0 {
            continue;
        }
        let e = by.entry(src).or_insert((f64::MAX, f64::MIN, 0));
        e.0 = e.0.min(s.half_m * 2.0);
        e.1 = e.1.max(s.half_m * 2.0);
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
/// **A band narrower than its own two casing rims has no surface left to
/// draw.** `synth::pave_mesh` insets the silhouette by [`priors::PAVE_RIM_M`]
/// on each side and meshes the interior as the surface, so under 0.70 m a band
/// is pure casing and under about 1.05 m the interior is a hairline — which is
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
/// Two probes at most, and the first answers for the great majority. The face
/// grows with the bench's width, so where the nominal verge already fits there
/// is nothing to decide; where it does not, the width whose face is the cap is
/// the cap's share of the one just measured, and the second probe reads the
/// ground there rather than trusting that estimate.
fn fitted_half(s: &SourceSeg, sample: &mut dyn FnMut(Coord) -> f64) -> Option<f64> {
    let cos_lat = s.cos_lat;
    let (dx, dy) = ((s.b.x - s.a.x) * cos_lat, s.b.y - s.a.y);
    let len = (dx * dx + dy * dy).sqrt();
    if !(len > 0.0) {
        return Some(s.half_m); // degenerate: the bench declines it anyway
    }
    let (px, py) = (-dy / len, dx / len); // lateral unit, metric (left)
    let mid = Coord { x: (s.a.x + s.b.x) * 0.5, y: (s.a.y + s.b.y) * 0.5 };
    // The same seat the bench will take: a sidewalk's is the height it is drawn
    // at, a path's is the ground along its own centerline, read at both ends
    // (`ground::walk_edge` says why never at the middle).
    let target = if s.corridor == NO_HOST {
        let (ha, hb) = (sample(s.a), sample(s.b));
        // **A link between two bands must be a link, not a wall.** A hostless
        // Walkway piece is a corner or a crossing stub — connective tissue
        // between two claimed stretches — and where the ground jumps more
        // than a storey within one segment it is not wrapping a corner, it
        // is draping across the bench cliff between a switchback's two arms
        // (6.9166,46.4338 is the type specimen: `slope.walk_crossfall`'s
        // worst read a 6.3 m step across 0.25 m of band there). No drawn
        // piece beats a wall.
        if s.surface == priors::Surface::Walkway && (ha - hb).abs() > CORNER_STEP_M {
            return None;
        }
        (ha + hb) * 0.5
    } else {
        (s.height_a + s.height_b) * 0.5
    };
    let cap = priors::bench_face_cap_m(s.surface);
    // The deepest of the two verge faces a bench of half-width `w` would carry.
    let face = |w: f64, sample: &mut dyn FnMut(Coord) -> f64| -> f64 {
        let at = |side: f64| Coord {
            x: mid.x + side * px * w / (DEG_M * cos_lat),
            y: mid.y + side * py * w / DEG_M,
        };
        (target - sample(at(1.0))).abs().max((target - sample(at(-1.0))).abs())
    };
    let nominal = s.half_m + priors::EARTHWORK_MARGIN_M;
    let rise = face(nominal, sample);
    if rise <= cap {
        return Some(s.half_m);
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

/// Stamps each band with the grade-separation layer of the carriageway stretch
/// it rides, once `synth::sheets` has settled them.
///
/// A band belongs to its host's sheet by definition, so the lookup is the run
/// containing the segment's own arc. A path has no host and no sheet to ride:
/// it stays on layer 0, where everything at grade that nothing stacks over
/// lives.
pub fn stamp_layers(bands: &mut [SourceSeg], grade_runs: &[GradeRun]) {
    for s in bands.iter_mut().filter(|s| s.corridor != NO_HOST) {
        s.layer = grade_runs
            .iter()
            .find(|g| {
                g.corridor == s.corridor
                    && g.arc0 <= s.arc0 + RUN_EPS_M
                    && g.arc1 >= s.arc0 - RUN_EPS_M
            })
            .map_or(0, |g| g.layer);
    }
}

/// The band of one attachment: the host's own centerline, offset to the side
/// the way is on, over the stretch of it the way covers.
#[allow(clippy::too_many_arguments)]
fn attached_band(
    scene: &SceneGraph,
    solved: &SolvedModel,
    facades: &Facades,
    a: &Attachment,
    out: &mut Vec<SourceSeg>,
    scratch: &mut Vec<u32>,
    census: &mut AttachCensus,
) {
    census.claimed_m += a.len_m();
    let Some(c) = scene.corridors.get(a.host as usize) else { return };
    let Some(half_m) = super::carriageway::corridor_half_width_m(c) else {
        census.no_width_m += a.len_m();
        return;
    };
    let profile = solved.profile(c.id);
    for (r0, r1, level, kind) in level_runs(c) {
        // **Only where the host is on the ground.** Over a bridge or in a bore
        // the sidewalk is carried by the structure itself (`synth::carried`),
        // and a band drawn there would be a second one floating beside the
        // deck — 1.5 % of attached host arc, measured in phase 3.
        if kind != SpanKind::Grade {
            census.non_grade_m += (a.arc1.min(r1) - a.arc0.max(r0)).max(0.0);
            continue;
        }
        let (lo, hi) = (a.arc0.max(r0), a.arc1.min(r1));
        if hi - lo < MIN_BAND_M {
            census.short_m += (hi - lo).max(0.0);
            continue;
        }
        // The host's own stations, so the band is sampled on the same curve the
        // asphalt is buffered around and the two stay parallel by construction.
        let stations: Vec<f64> = match profile {
            Some(p) => p.arc().iter().copied().filter(|&s| s > lo && s < hi).collect(),
            None => c.arc.iter().copied().filter(|&s| s > lo && s < hi).collect(),
        };
        let mut stops: Vec<f64> = vec![lo];
        for s in stations.into_iter().chain(std::iter::once(hi)) {
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
        // The asphalt's own cross-section here — the kerb the band starts at.
        let sections = sections_along(c, &stops, &pts, half_m, facades, false, scratch);
        let side = a.side as usize;
        // Where each station's band sits: its centre offset from the host
        // centerline, and its half-width. `None` where the room ran out.
        let seats: Vec<Option<(f64, f64)>> = (0..stops.len())
            .map(|i| seat(c, &stops, &pts, i, sections[i], side, a.offset_m, facades, scratch))
            .collect();
        let height = |arc: f64| profile.map_or(0.0, |p| p.road_at_arc(arc));
        for i in 0..stops.len() - 1 {
            let (Some((oa, ha)), Some((ob, hb))) = (seats[i], seats[i + 1]) else {
                census.no_room_m += stops[i + 1] - stops[i];
                continue; // the band stops where the room does
            };
            // One half-width per segment, so the run chains: the union strokes
            // a polyline at a constant width, and a taper would be a stack of
            // one-segment runs. The narrower of the two ends keeps the band
            // clear of the facade at both.
            let half = ha.min(hb);
            if 2.0 * half < priors::WALK_MIN_WIDTH_M {
                census.narrow_m += stops[i + 1] - stops[i];
                continue;
            }
            census.built_m += stops[i + 1] - stops[i];
            let normal_a = normal_at(&pts, i, c.cos_lat, side);
            let normal_b = normal_at(&pts, i + 1, c.cos_lat, side);
            out.push(SourceSeg {
                a: offset(pts[i], normal_a, oa, c.cos_lat),
                b: offset(pts[i + 1], normal_b, ob, c.cos_lat),
                cos_lat: c.cos_lat,
                half_m: half,
                sect_a: Section::uniform(half),
                sect_b: Section::uniform(half),
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
        }
    }
}

/// Where one station's band sits: `(offset from the centerline, half-width)`,
/// or `None` where there is not [`priors::WALK_MIN_WIDTH_M`] of room for one.
///
/// **Seated on the mapped way, pushed clear of the kerb, clipped at the
/// facade.** Three rules, in that priority:
///
/// - The band is [`priors::WALK_WIDTH_M`] wide, narrowing only where the strip
///   between kerb and facade is narrower than that.
/// - It sits where the way was measured (`Attachment::offset_m`). A band pinned
///   to the kerb regardless would be right for the median sidewalk — phase 3
///   measured p50 1.1 m clear of the kerb — and wrong for the p90 at 3.1 m and
///   the tail past 5 m, drawing the sidewalk metres from where the mapper put
///   it while its own stroke is gone.
/// - It never overlaps the carriageway and never crosses
///   [`priors::FACADE_CLEAR_M`] short of a wall. Those two are the bounds the
///   seat is clamped into, so a way mapped *inside* its own street's kerb — a
///   footway digitized down the middle of a lane — still comes out beside it
///   rather than on it.
#[allow(clippy::too_many_arguments)]
fn seat(
    c: &Corridor,
    stops: &[f64],
    pts: &[Coord],
    i: usize,
    section: Section,
    side: usize,
    want_m: f64,
    facades: &Facades,
    scratch: &mut Vec<u32>,
) -> Option<(f64, f64)> {
    let kerb = section.on(side);
    // Look far enough out to seat the band where the way is *and* clear its
    // far edge — never less than a kerb-hugging band's own reach, or open
    // ground would report exactly the room a band needs and the seat would
    // have nowhere to slide.
    let want = want_m.max(kerb);
    let reach = (want + priors::WALK_WIDTH_M * 0.5 + priors::FACADE_CLEAR_M)
        .max(kerb + priors::WALK_WIDTH_M + priors::FACADE_CLEAR_M);
    let room = room_at(c, stops, pts, i, side, reach, facades, scratch);
    let avail = room - priors::FACADE_CLEAR_M - kerb;
    if avail < priors::WALK_MIN_WIDTH_M {
        return None;
    }
    // Snapped down to the width ladder, so a strip that merely varies along
    // the street draws one width instead of a new one at every station.
    let half = priors::quantize_walk_width(avail.min(priors::WALK_WIDTH_M)) * 0.5;
    Some((want.clamp(kerb + half, kerb + avail - half), half))
}

/// The room on `side` at station `i`, read exactly as the carriageway read it
/// (`synth::carriageway::sections_along`) so the two allocations are one
/// measurement of one street.
#[allow(clippy::too_many_arguments)]
fn room_at(
    c: &Corridor,
    stops: &[f64],
    pts: &[Coord],
    i: usize,
    side: usize,
    reach: f64,
    facades: &Facades,
    scratch: &mut Vec<u32>,
) -> f64 {
    if facades.is_empty() {
        return reach;
    }
    let m_lon = DEG_M * c.cos_lat;
    let (j, k) = (i.saturating_sub(1), (i + 1).min(stops.len() - 1));
    if j == k {
        return reach;
    }
    let (dx, dy) = ((pts[k].x - pts[j].x) * m_lon, (pts[k].y - pts[j].y) * DEG_M);
    let len = dx.hypot(dy);
    if !(len > 0.0) {
        return reach;
    }
    let window = (stops[k] - stops[j])
        .max(super::carriageway::ROOM_WINDOW_MIN_M)
        .min(super::carriageway::ROOM_WINDOW_MAX_M);
    facades.room(pts[i], c.cos_lat, (dx / len, dy / len), reach, window, scratch).on(side)
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

/// The bands for the stretches of a way that attached to nothing: paths
/// across open ground, and — where a stretch is *pinched between two claimed
/// ones* — the corner a sidewalk wraps between its two streets.
///
/// **The corner is a sidewalk, not a path.** `assemble::walks::runs` breaks
/// an attachment where the way turns across its host, correctly — a band must
/// not bridge the mouth of a side street — so the stretch that wraps a corner
/// attaches to nothing by construction. Left to the path rule it came out the
/// wrong feature twice over: `Path` material on the ground where its two
/// neighbours are `Walkway` on a kerb, and *nothing at all* under
/// [`MIN_BAND_M`] — and a junction's corners are exactly where sub-4 m
/// stretches arise, between the two crossing connectors of a corner. So a
/// bounded gap under [`priors::WALK_CORNER_MAX_M`] keeps the material, the
/// kerb rise and the width of the sidewalk it continues, at any length worth
/// a segment at all: it is the link between two bands, and a link has no
/// minimum worth existing.
fn path_bands(
    line: &WalkLine,
    attached: &[Attachment],
    built: &[(f64, f64)],
    out: &mut Vec<SourceSeg>,
) {
    let cos_lat = crate::scene::run_cos_lat(&line.line);
    let arc = crate::scene::cumulative_arc(&line.line);
    let total = *arc.last().unwrap_or(&0.0);
    // The complement of the attached stretches, in the way's own arc. They come
    // in walk-arc order along one line, so one sweep is enough.
    // What the band must skip: the stretches a sidewalk already covers, and the
    // stretches that are not on the ground at all. Both are "something else is
    // the walkway here", and both are cut out the same way.
    let mut taken: Vec<(f64, f64)> = attached.iter().map(|a| (a.walk0, a.walk1)).collect();
    taken.extend(line.spans.iter().map(|&(s, e)| (s * total, e * total)));
    taken.sort_by(|a, b| a.0.total_cmp(&b.0));
    // A gap's end is *drawn* when the claim it borders produced a band —
    // matched by the claim's own walk range, so a span or an empty claim
    // bounds a path, never a corner.
    let drawn_left = |lo: f64| built.iter().any(|&(_, w1)| (w1 - lo).abs() < 1e-6);
    let drawn_right = |hi: f64| built.iter().any(|&(w0, _)| (w0 - hi).abs() < 1e-6);
    let mut cursor = 0.0f64;
    // `(from, to, corner)`: a gap, and whether both of its ends are drawn.
    let mut gaps: Vec<(f64, f64, bool)> = Vec::new();
    for (w0, w1) in taken.into_iter().chain(std::iter::once((total, total))) {
        // The ranges come from two measurements of the same line — station
        // counts and level-run fractions — so neither is guaranteed to land
        // inside `total`. Order the clamp so a range past the end closes the
        // sweep rather than inverting it.
        let lo = cursor.min(total);
        let hi = w0.clamp(lo, total);
        cursor = cursor.max(w1);
        let corner = hi - lo <= priors::WALK_CORNER_MAX_M
            && drawn_left(lo)
            && drawn_right(hi);
        if hi - lo > if corner { MIN_CORNER_M } else { MIN_BAND_M } {
            gaps.push((lo, hi, corner));
        }
    }
    // A track is drawn at a vehicle's width, everything else at a walker's.
    let nominal = if matches!(line.kind, crate::priors::Kind::Road(crate::priors::RoadClass::Track))
    {
        priors::TRACK_WIDTH_M
    } else {
        priors::WALK_WIDTH_M
    };
    for (g0, g1, corner) in gaps {
        let half = nominal * 0.5;
        // A corner keeps the sidewalk's *material* but stands on the ground:
        // a hostless band's bench targets the ground along its own centerline,
        // so a rise here is exactly a float above its own bench — the height
        // field ramps the neighbouring kerbs down into it, which is what a
        // corner's dropped kerbs are.
        let surface = if corner { priors::Surface::Walkway } else { priors::Surface::Path };
        let (stops, pts) = resample(&line.line, &arc, g0, g1, PATH_STATION_M);
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
                // A path stands on the ground. The height field reads that off
                // `NO_HOST`'s absent profile, so these are only what
                // `synth::sheets` would have compared — and it never sees them.
                height_a: 0.0,
                height_b: 0.0,
                corridor: NO_HOST,
                surface,
                rise_m: 0.0,
                arc0: stops[i],
            });
        }
    }
}

/// Shortest corner link worth a segment, in metres — a guard against
/// zero-length slivers, not a claim about sidewalks: the whole point of the
/// corner rule is that a two-metre link still draws.
const MIN_CORNER_M: f64 = 0.4;

/// Largest ground step a hostless Walkway segment may span end to end, in
/// metres, before it is a wall rather than a link ([`fitted_half`]). Higher
/// than any kerb or corner ramp, lower than the arm separation of the
/// shallowest switchback the extract holds (~5 m).
const CORNER_STEP_M: f64 = 1.5;

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
    use crate::priors::{Kind, RoadClass};

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

    fn walk_line(pts: Vec<Coord>) -> WalkLine {
        WalkLine {
            source: 1,
            line: pts,
            kind: Kind::Road(RoadClass::Footway),
            tagged: true,
            crosswalk: false,
            connectors: Vec::new(),
            spans: Vec::new(),
        }
    }

    #[test]
    fn a_path_with_no_attachment_is_banded_along_its_whole_length() {
        let w = walk_line(line(0.0, 40.0, 0.0));
        let mut out = Vec::new();
        path_bands(&w, &[], &[], &mut out);
        assert!(!out.is_empty());
        assert!(out.iter().all(|s| s.corridor == NO_HOST && s.rise_m == 0.0));
        let span: f64 = out
            .iter()
            .map(|s| crate::scene::metric_len(s.a, s.b, s.cos_lat))
            .sum();
        assert!((span - 40.0).abs() < 0.5, "{span}");
    }

    #[test]
    fn the_attached_stretch_is_left_to_the_sidewalk() {
        // 100 m of way, attached from 20 to 60: the path bands are the two ends.
        let w = walk_line(line(0.0, 100.0, 0.0));
        let a = Attachment {
            walk: 1,
            line: 0,
            walk0: 20.0,
            walk1: 60.0,
            kind: Kind::Road(RoadClass::Footway),
            host: 0,
            side: 0,
            arc0: 20.0,
            arc1: 60.0,
            offset_m: 5.0,
            spread_m: 0.0,
            evidence: crate::assemble::walks::Evidence::Tag,
        };
        let mut out = Vec::new();
        path_bands(&w, &[a], &[], &mut out);
        let span: f64 = out
            .iter()
            .map(|s| crate::scene::metric_len(s.a, s.b, s.cos_lat))
            .sum();
        assert!((span - 60.0).abs() < 0.5, "20 + 40 metres of path, got {span}");
    }

    #[test]
    fn a_gap_pinched_between_two_claims_is_the_sidewalk_wrapping_its_corner() {
        let w = walk_line(line(0.0, 100.0, 0.0));
        let stub = |w0: f64, w1: f64| Attachment {
            walk: 1,
            line: 0,
            walk0: w0,
            walk1: w1,
            kind: Kind::Road(RoadClass::Footway),
            host: 0,
            side: 0,
            arc0: w0,
            arc1: w1,
            offset_m: 5.0,
            spread_m: 0.0,
            evidence: crate::assemble::walks::Evidence::Tag,
        };
        // Two attachments 2 m apart: the gap is the link between two bands —
        // kept, in the sidewalk's own material at the kerb rise, under any
        // path-band minimum.
        let mut out = Vec::new();
        path_bands(
            &w,
            &[stub(0.0, 49.0), stub(51.0, 100.0)],
            &[(0.0, 49.0), (51.0, 100.0)],
            &mut out,
        );
        assert!(!out.is_empty(), "the corner link must be banded");
        assert!(out
            .iter()
            .all(|s| s.surface == crate::priors::Surface::Walkway && s.rise_m == 0.0));
        let span: f64 =
            out.iter().map(|s| crate::scene::metric_len(s.a, s.b, s.cos_lat)).sum();
        assert!((span - 2.0).abs() < 0.5, "just the gap: {span}");
        // A sliver under MIN_CORNER_M is still nothing.
        let mut none = Vec::new();
        path_bands(
            &w,
            &[stub(0.0, 49.9), stub(50.2, 100.0)],
            &[(0.0, 49.9), (50.2, 100.0)],
            &mut none,
        );
        assert!(none.is_empty(), "{}", none.len());
        // And an *unbounded* stretch keeps the path rule: the leading 30 m
        // before a single attachment is a path, not a corner.
        let mut open = Vec::new();
        path_bands(&w, &[stub(30.0, 100.0)], &[(30.0, 100.0)], &mut open);
        assert!(open.iter().all(|s| s.surface == crate::priors::Surface::Path));
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
    fn the_band_takes_the_room_it_has_and_slides_to_meet_the_way() {
        let sect = Section::uniform(3.0);
        let c = corridor();
        let stops = vec![0.0, 50.0];
        let pts = vec![c.nodes[0], c.nodes[1]];
        let mut scratch = Vec::new();
        // Open ground: the full width, seated where the way was measured.
        let (off, half) =
            seat(&c, &stops, &pts, 0, sect, 0, 5.0, &Facades::empty(), &mut scratch).unwrap();
        assert!((half - 1.0).abs() < 1e-9, "the nominal 2 m band: {half}");
        assert!((off - 5.0).abs() < 1e-9, "seated on the mapped offset: {off}");
        // A way mapped right against the kerb — or inside it — is pushed out
        // to make room for its own width, so the band never sits on asphalt.
        for want in [3.0, 1.0, -2.0] {
            let (off, half) =
                seat(&c, &stops, &pts, 0, sect, 0, want, &Facades::empty(), &mut scratch).unwrap();
            assert!(
                (off - half - 3.0).abs() < 1e-9,
                "the band's inner edge is the kerb, for want={want}: {off} ± {half}"
            );
        }
    }

    #[test]
    fn a_facade_too_close_to_the_kerb_leaves_no_sidewalk() {
        let c = corridor();
        let stops = vec![0.0, 50.0];
        let pts = vec![c.nodes[0], c.nodes[1]];
        let mut scratch = Vec::new();
        // A wall 3.6 m out from a centerline whose asphalt already reaches
        // 3.0 m: 0.1 m of strip after the facade clearance, under the minimum.
        let wall = Facades::from_edges([[
            Coord { x: 6.89, y: LAT + 3.6 / DEG_M },
            Coord { x: 6.91, y: LAT + 3.6 / DEG_M },
        ]]);
        let seated =
            seat(&c, &stops, &pts, 0, Section::uniform(3.0), 0, 4.0, &wall, &mut scratch);
        assert!(seated.is_none(), "{seated:?}");
        // And the other side of the same street is untouched.
        assert!(seat(&c, &stops, &pts, 0, Section::uniform(3.0), 1, 4.0, &wall, &mut scratch)
            .is_some());
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
