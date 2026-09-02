//! Whether the street is drawn as a cross-section.
//!
//! `network` asks whether the mapped pedestrian *graph* survives into the drawn
//! world. This module asks the question one level down, about the object that
//! graph is supposed to be drawn as: **a street is a room between facades,
//! tiled edge to edge — carriageway, kerb, footway, verge — and a pavement is a
//! side of a corridor, not a feature of its own** (docs/ROADS.md invariant 1).
//! Every metric here measures a property the cross-section model holds *by
//! construction* and the per-feature model can only hold by luck.
//!
//! **Why the existing instruments read green while the picture is confetti.**
//! `ARPT_DEBUG_WALK` reports 98.1 % of claimed host arc built and 96.6 % of it
//! at full width; the render at Montreux Rue du Centre (6.9092, 46.4373) is
//! three disjoint rectangles across four mapped sidewalks totalling 285 m. The
//! census counts host arc that produced *a segment*. It cannot see that the
//! segments do not join — and nothing else could either, because no object in
//! the model owns a whole pavement. That is the gap these three fill.
//!
//! - **`street.strip_continuity`** — the headline. Along every corridor side
//!   the data says carries a pavement, the share of that extent with no drawn
//!   footway anywhere in the strip between the kerb and the seat's own reach.
//!   Continuity is the property; a ribbon interrupted every few metres reads as
//!   slabs however much total length was built.
//! - **`street.kerb_join`** — the plan gap between the carriageway's drawn edge
//!   and the pavement's inner edge. Zero once the two are one number read twice
//!   (§3.2 of the plan); today the band is seated at the mapped way's own
//!   offset, so bare ground between road and pavement is produced by the rule
//!   rather than by accident.
//! - **`street.crossing_extent`** — zebra painted where no crossed carriageway
//!   accounts for it. `crossings` builds its chord by marching for *any*
//!   corridor within its prior half-width and merging across 1 m gaps, so a
//!   crossing that runs *alongside* a forecourt's service roads annexes them:
//!   at Rue de la Gare an 18 m crosswalk became a ~34 m ladder over a railway.
//!
//! **Model-side, for `network`'s reason and one more.** From
//! `WALK_SURFACE_MIN_ZOOM` the pedestrian strokes are deleted, so the mapped
//! line exists only in the model. And two of these three measure a *relation
//! between two allotments of one street* — what the carriageway took and what
//! the footway got — which the archive has already dissolved into one boolean
//! union with no memory of which input paid for which metre.

use std::collections::HashMap;

use geo_types::Coord;

use crate::assemble::facades::Section;
use crate::priors::Surface;
use crate::scene::{Corridor, DEG_M, SpanKind};
use crate::synth::carriageway::{SourceSeg, corridor_half_width_m};
use crate::synth::walkway;
use crate::verify::dist::Dist;
use crate::verify::{Invariant, Metric, Offender, Sense, Worst};

use super::Model;

/// Sample spacing along a corridor side, in metres. One metre is what
/// `assemble::walks` stations at and finer than any gap a person would call
/// continuous.
const STATION_M: f64 = 1.0;

/// How far out from the drawn kerb the strip probe looks for pavement, in
/// metres.
///
/// The same number `network::walk_material` calls `STANDIN_M`, for the same
/// reason: the seat re-plots a pavement anywhere between the kerb and the
/// facade clearance, at the run's mean offset, so a band found anywhere in that
/// window *is* this street's pavement. Looking only at the kerb would measure
/// the seat's offset rather than continuity — that is `kerb_join`'s question,
/// asked separately on purpose.
const STRIP_REACH_M: f64 = 8.0;

/// Bare strip past this counts as an interruption, in metres.
///
/// Reasoned, not read, and it is deliberately below the station spacing: an
/// interruption is at least one station wide, so **every** uncovered station
/// counts and the rate is exactly the share of the evidence extent drawn bare.
/// The threshold's only job is to keep the covered stations — which score
/// exactly zero — out of the tally.
const BARE_M: f64 = 0.5;

/// Gap between the carriageway edge and the pavement's inner edge past which
/// the strip between them reads as bare ground rather than as a kerb, in
/// metres.
///
/// The cross-section model makes this identically zero: the pavement's inner
/// edge *is* the carriageway's outer edge, one number read twice. Until then
/// the tolerance has to admit what the current seat legitimately produces —
/// the profile smoothing displaces a band from the raw centerline by a median
/// half-metre at junction mouths — so half a metre is the machinery, and
/// anything past it is the rule putting a strip of hillside between a road and
/// its pavement.
const JOIN_M: f64 = 0.5;

/// Step along a crossing chord when asking what carriageway is under it, in
/// metres. A quarter metre resolves a kerb line; the value being measured is a
/// length of misplaced paint, not a position.
const CHORD_STEP_M: f64 = 0.25;

/// Misplaced chord past this is a violation, in metres. Same order as
/// [`BARE_M`] and for the same reason: a chord end may legitimately overrun the
/// carriageway edge by the smoothing displacement, and nothing beyond that.
const CHORD_BARE_M: f64 = 0.5;

/// A crossing chord further off square than this is skewed, in degrees.
///
/// The zebra's bars are longitudinal to traffic by construction
/// (`synth::markings::crossing_bars` draws them perpendicular to the chord),
/// so the chord's angle to the crossed centerline *is* the bars' angle to the
/// traffic axis, and a square crossing reads 0 here whatever direction the
/// street runs. Real crosswalks are mapped a few degrees oblique where a
/// refuge island or a bent kerb line skews the walking line; twenty degrees
/// is past what any of that explains, and the first offender measured — a
/// Territet junction mouth whose chord picked kerb points that are not
/// opposite each other — read 26°.
const CHORD_SKEW_DEG: f64 = 20.0;

/// Half-size of a source query box, in metres — the widest half-width any band
/// carries, plus the strip reach, so a probe finds every source that could
/// cover it.
const QUERY_M: f64 = 32.0;

/// Two evidence intervals on one corridor side separated by less than this are
/// one claim, in metres.
///
/// `assemble::walks::runs` breaks an attachment wherever the way turns across
/// its host, which is correct for attachment and produces a break at every
/// corner and every driveway. The stretch between two claims on one side of one
/// street is pavement the data asserts as plainly as the claims themselves —
/// `priors::WALK_CORNER_MAX_M` is the same judgement made in `synth::walkway`
/// — so the extent this check holds the drawing to is the merged one.
const EVIDENCE_MERGE_M: f64 = crate::priors::WALK_CORNER_MAX_M;

pub fn check(m: &Model<'_>) -> Vec<Metric> {
    synth_census(m);
    vec![strip_continuity(m), kerb_join(m), crossing_extent(m), crossing_skew(m), width_step(m)]
}

// ---------------------------------------------------------------- continuity

/// Every corridor side the data says carries a pavement, marched for the
/// stretches where none is drawn.
fn strip_continuity(m: &Model<'_>) -> Metric {
    let mut dist = Dist::metres();
    let mut worst = Worst::new(Sense::HigherIsWorse, 8);
    let debug = std::env::var_os("ARPT_DEBUG_STRIP").is_some();

    // What the data claims, per (corridor, side), in host arc metres.
    let mut claims: HashMap<(u32, u8), Vec<(f64, f64)>> = HashMap::new();
    for (_, attached) in m.scene.walks.lines() {
        for a in attached {
            claims.entry((a.host, a.side)).or_default().push((a.arc0, a.arc1));
        }
    }
    // Sorted, so the walk order — and therefore the worst list — is a function
    // of the model rather than of hashing.
    let mut keys: Vec<(u32, u8)> = claims.keys().copied().collect();
    keys.sort_unstable();

    let mut scratch: Vec<u32> = Vec::new();
    let mut sides = 0u32;
    let mut breaks = 0u64;
    let mut extent_m = 0.0f64;
    for key in keys {
        let (host, side) = key;
        let Some(c) = m.scene.corridors.get(host as usize) else { continue };
        let Some(half) = corridor_half_width_m(c) else { continue };
        let mut ranges = claims[&key].clone();
        ranges.sort_by(|a, b| a.0.total_cmp(&b.0));
        let merged = merge_ranges(&ranges, EVIDENCE_MERGE_M);
        let mut side_seen = false;
        for (lo, hi) in merged {
            // Only where the host is on the ground. Over a bridge or in a bore
            // the pavement is carried by the structure (`synth::carried`) and
            // no band is owed here — the same exclusion `attached_band` makes.
            for (g0, g1) in grade_ranges(c, lo, hi) {
                let mut covered: Vec<bool> = Vec::new();
                let mut at: Vec<Coord> = Vec::new();
                let mut s = g0;
                while s <= g1 {
                    let p = point_at_arc(m, c, s);
                    // A station outside the extract does not merely go
                    // unmeasured — it *ends* the stretch being measured.
                    // Carrying on across it would join the gaps either side of
                    // the boundary into one hole that exists in no tile.
                    if !m.bounds.contains(p.x, p.y) {
                        side_seen |= score_stretch(
                            &covered, &at, c, side, m.solved.z_ref, &mut dist, &mut worst,
                            &mut breaks, &mut extent_m,
                        );
                        covered.clear();
                        at.clear();
                        s += STATION_M;
                        continue;
                    }
                    let n = outward_normal(m, c, s, side);
                    let a = offset(p, n, half, c.cos_lat);
                    let b = offset(p, n, half + STRIP_REACH_M, c.cos_lat);
                    covered.push(strip_covered(m, a, b, c.cos_lat, &mut scratch));
                    at.push(p);
                    s += STATION_M;
                }
                side_seen |= score_stretch(
                    &covered, &at, c, side, m.solved.z_ref, &mut dist, &mut worst, &mut breaks,
                    &mut extent_m,
                );
            }
        }
        if side_seen {
            sides += 1;
        }
    }
    if debug {
        eprintln!(
            "[strip] {sides} corridor sides claim a pavement over {:.2} km; \
             {breaks} interruptions",
            extent_m / 1000.0,
        );
    }

    Metric {
        id: "street.strip_continuity".into(),
        invariant: Invariant::I4,
        title: "Claimed pavement with none drawn beside the street".into(),
        population: format!(
            "Every {STATION_M:.0} m of every (corridor, side) that `assemble::walks` attached a \
             pedestrian way to, over the merged extent of its claims — two claims within \
             {EVIDENCE_MERGE_M:.0} m are one, since the attachment breaks at every corner and \
             driveway by design. At-grade stretches inside the bbox only. A station scores zero \
             when any walk or path band lies within the strip from the drawn kerb to \
             {STRIP_REACH_M:.0} m out (the seat's own play, so a legitimately re-seated pavement \
             counts), and otherwise the length of the interruption it belongs to. The rate is \
             therefore the share of claimed pavement drawn bare, and the worst is the longest \
             single hole. `ARPT_DEBUG_STRIP` prints the sides, the extent and the break count."
        ),
        detail: format!(
            "A pavement is a side of a street, and its realism is continuity: a ribbon \
             interrupted every few metres reads as floating slabs however much total length was \
             built. The per-feature model cannot hold this — a drawn metre must survive a mapped \
             line, an attachment, a level run, `MIN_BAND_M`, a seat with room, a ground fit and a \
             run chain that breaks on any width change — and no rule anywhere asserts the result \
             is connected. Past {BARE_M:.1} m a person sees a hole in the pavement."
        ),
        sense: Sense::HigherIsWorse,
        threshold: BARE_M,
        skipped: (extent_m <= 0.0)
            .then(|| "no pedestrian way attached to a street in this extract".to_string()),
        dist,
        worst: worst.into_vec(),
    }
}

/// Scores one uninterrupted run of stations: a sample per station, valued at
/// the length of the interruption it belongs to. The rate is then the share of
/// claimed pavement drawn bare, and the worst is the longest single hole rather
/// than the deepest of many. Returns whether anything was measured.
#[allow(clippy::too_many_arguments)]
fn score_stretch(
    covered: &[bool],
    at: &[Coord],
    c: &Corridor,
    side: u8,
    zoom: u8,
    dist: &mut Dist,
    worst: &mut Worst,
    breaks: &mut u64,
    extent_m: &mut f64,
) -> bool {
    if covered.is_empty() {
        return false;
    }
    *extent_m += covered.len() as f64 * STATION_M;
    let mut i = 0;
    while i < covered.len() {
        if covered[i] {
            dist.push(0.0);
            i += 1;
            continue;
        }
        let start = i;
        while i < covered.len() && !covered[i] {
            i += 1;
        }
        let run_m = (i - start) as f64 * STATION_M;
        *breaks += 1;
        for _ in start..i {
            dist.push(run_m);
        }
        let mid = at[start + (i - start) / 2];
        worst.offer(Offender {
            lon: mid.x,
            lat: mid.y,
            zoom,
            value: run_m,
            note: format!(
                "{run_m:.0} m of {} side {} carries a mapped pavement and draws none",
                c.class_key,
                if side == 0 { "left" } else { "right" },
            ),
        });
    }
    true
}

/// Whether any walkable band covers the strip segment `a`–`b`.
fn strip_covered(
    m: &Model<'_>,
    a: Coord,
    b: Coord,
    cos_lat: f64,
    scratch: &mut Vec<u32>,
) -> bool {
    let (rx, ry) = (QUERY_M / (DEG_M * cos_lat), QUERY_M / DEG_M);
    let bbox = (
        a.x.min(b.x) - rx,
        a.y.min(b.y) - ry,
        a.x.max(b.x) + rx,
        a.y.max(b.y) + ry,
    );
    m.junctions.sources_near(bbox, scratch);
    scratch.iter().any(|&i| {
        let s = m.junctions.source(i);
        if !matches!(s.surface, Surface::Walkway | Surface::Path) {
            return false;
        }
        // The width the band is *drawn* at, at the point on it nearest the
        // strip. `half_m` is the run's chaining key now and stays at the class
        // nominal however much the room took off, so measuring against it would
        // report a narrowed strip as covering ground it does not — this check's
        // own headline, read optimistically.
        let (d, t) = segment_distance_m(a, b, s.a, s.b, cos_lat);
        d <= s.drawn_half_at(t)
    })
}

// ----------------------------------------------------------------- kerb join

/// Every attached pavement segment, scored by the strip of ground left between
/// its inner edge and the carriageway it belongs to.
fn kerb_join(m: &Model<'_>) -> Metric {
    let mut dist = Dist::metres();
    let mut worst = Worst::new(Sense::HigherIsWorse, 8);
    let mut scratch: Vec<u32> = Vec::new();
    let mut measured = 0u64;

    for i in 0..m.junctions.source_count() as u32 {
        let band = m.junctions.source(i);
        // Attached pavement only. A hostless band — a path, a corner link, a
        // crossing's kerb stub — has no kerb to join, and asking it this
        // question would score the distance to whatever street it happens to
        // pass.
        if band.surface != Surface::Walkway || band.corridor == crate::synth::walkway::NO_HOST {
            continue;
        }
        let mid = Coord { x: 0.5 * (band.a.x + band.b.x), y: 0.5 * (band.a.y + band.b.y) };
        if !m.bounds.contains(mid.x, mid.y) {
            continue;
        }
        let (rx, ry) = (QUERY_M / (DEG_M * band.cos_lat), QUERY_M / DEG_M);
        m.junctions
            .sources_near((mid.x - rx, mid.y - ry, mid.x + rx, mid.y + ry), &mut scratch);
        // Its own host's asphalt, never the nearest asphalt: a pavement on a
        // corner stands close to the side street it does not belong to, and
        // measuring against that would report a join it does not owe.
        let mut best: Option<f64> = None;
        for &j in scratch.iter() {
            let road = m.junctions.source(j);
            if road.surface != Surface::Asphalt || road.corridor != band.corridor {
                continue;
            }
            let (d, u) = project(mid, road.a, road.b, road.cos_lat);
            let side = side_of(mid, road.a, road.b, road.cos_lat);
            let gap = d - band.drawn_half_at(0.5) - lerp_section(road, u).on(side);
            if best.is_none_or(|b| gap < b) {
                best = Some(gap);
            }
        }
        let Some(gap) = best else { continue };
        measured += 1;
        dist.push(gap.max(0.0));
        if gap > JOIN_M {
            worst.offer(Offender {
                lon: mid.x,
                lat: mid.y,
                zoom: m.solved.z_ref,
                value: gap,
                note: format!("a pavement stands {gap:.2} m clear of the carriageway edge it belongs to"),
            });
        }
    }

    Metric {
        id: "street.kerb_join".into(),
        invariant: Invariant::I4,
        title: "Bare ground between a carriageway and its own pavement".into(),
        population: format!(
            "Every walk band segment attached to a host corridor, inside the bbox ({measured} \
             measured), scored as the plan distance from the band's inner edge to the drawn edge \
             of *that same corridor's* asphalt — never the nearest asphalt, since a pavement on a \
             corner stands close to a side street it does not belong to. Overlap is clamped to \
             zero: a band the room could not narrow far enough is `order.at_grade_overlap`'s \
             question, and the union already subtracts it. Hostless bands — paths, corner links, \
             crossing stubs — are out of the population, having no kerb to join."
        ),
        detail: format!(
            "The cross-section model makes this identically zero, because the pavement's inner \
             edge and the carriageway's outer edge are one number read twice. Today they are two \
             independent allotments of one street — `carriageway::sections_along` spends the \
             facade room on asphalt and `walkway::seat` re-measures the same facades for what it \
             thinks is left — and the band is then seated at the mapped way's own offset, which \
             is a fact about where a mapper drew a line and not about where a kerb is. Past \
             {JOIN_M:.1} m the strip between road and pavement is drawn hillside, and on a grade \
             it is the small cliff `contact.kerb_lip` scores from the other side."
        ),
        sense: Sense::HigherIsWorse,
        threshold: JOIN_M,
        skipped: (measured == 0)
            .then(|| "no pavement is attached to a carriageway in this extract".to_string()),
        dist,
        worst: worst.into_vec(),
    }
}

// ------------------------------------------------------------ crossing extent

/// Every registered zebra chord, scored by the length of it that no carriageway
/// the crossing actually *crosses* accounts for.
/// The crossing's mapped polyline pushed out along its end tangents by the
/// same 8 m the registration extends it (`walkway::CROSSING_EXTEND_M`). Hosts
/// must be derived from this line, not the mapped one: a crosswalk mapped
/// short of the second roadway of a divided street finds it only through the
/// extension, and a check that refuses the extension scores that roadway's
/// chord as bare — 4.7 m of a 5 m chord at 6.90847,46.43794, on a drawing
/// that is right.
fn extended_line(line: &[Coord], cos_lat: f64) -> Vec<Coord> {
    let mut ext = Vec::with_capacity(line.len() + 2);
    ext.push(walkway::end_extension(line, cos_lat, false, walkway::CROSSING_EXTEND_M));
    ext.extend_from_slice(line);
    ext.push(walkway::end_extension(line, cos_lat, true, walkway::CROSSING_EXTEND_M));
    ext
}

fn crossing_extent(m: &Model<'_>) -> Metric {
    let mut dist = Dist::metres();
    let mut worst = Worst::new(Sense::HigherIsWorse, 8);
    let debug = std::env::var_os("ARPT_DEBUG_STRIP").is_some();
    let mut dem = m.terrain.and_then(|p| crate::dem::Dem::open(p).ok());

    // Every drivable centerline segment, indexed by plan position — the same
    // index `synth::walkway::crossings` scans, so the two disagree only about
    // the predicate and never about the data.
    let mut grid = crate::assemble::grid::GridIndex::with_cell_m(64.0);
    let mut edges: Vec<(u32, u32)> = Vec::new();
    for (ci, c) in m.scene.corridors.iter().enumerate() {
        if c.kind.prior().surface != Surface::Asphalt || corridor_half_width_m(c).is_none() {
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
    let mut chords = 0u64;
    let mut hostless = 0u64;
    for (line, _) in m.scene.walks.lines() {
        if !line.crosswalk || line.line.len() < 2 {
            continue;
        }
        let Some(painted) = m.crossings.get(&line.source) else { continue };
        let cos_lat = crate::scene::run_cos_lat(&line.line);
        let anchor = dem
            .as_mut()
            .map(|d| walkway::crossing_level_anchor(&line.line, d, m.solved.z_ref));
        // The streets this crossing *crosses*: those whose centerline its own
        // mapped polyline properly intersects at the crossing's own level. A
        // corridor it merely runs beside is not crossed, and a plan-crossed
        // one a terrace away is not either — that distinction is the whole
        // check, and the registration now makes both
        // (`walkway::crossed_hosts`), so the two share one derivation.
        let ext = extended_line(&line.line, cos_lat);
        let mut hosts: Vec<(u32, Vec<f64>)> = Vec::new();
        for w in ext.windows(2) {
            let bbox = (
                w[0].x.min(w[1].x),
                w[0].y.min(w[1].y),
                w[0].x.max(w[1].x),
                w[0].y.max(w[1].y),
            );
            grid.query(bbox, &mut scratch);
            for &e in scratch.iter() {
                let (ci, ni) = edges[e as usize];
                let c = &m.scene.corridors[ci as usize];
                let (t0, t1) = (c.nodes[ni as usize], c.nodes[ni as usize + 1]);
                if !segments_cross(w[0], w[1], t0, t1)
                    || !walkway::incidence_ok(w[0], w[1], t0, t1, c.cos_lat)
                {
                    continue;
                }
                let mid = Coord { x: (w[0].x + w[1].x) * 0.5, y: (w[0].y + w[1].y) * 0.5 };
                if !walkway::host_level_ok(m.solved, ci, mid, anchor) {
                    continue;
                }
                let s = c.arc[ni as usize]
                    + crate::scene::metric_len(c.nodes[ni as usize], mid, c.cos_lat);
                match hosts.iter_mut().find(|(h, _)| *h == ci) {
                    Some((_, arcs)) => arcs.push(s),
                    None => hosts.push((ci, vec![s])),
                }
            }
        }
        if hosts.is_empty() {
            hostless += 1;
            continue; // crosses nothing: no carriageway owes it a chord
        }
        for &crate::synth::walkway::Chord { a, b, .. } in painted.iter() {
            if !m.bounds.contains(a.x, a.y) && !m.bounds.contains(b.x, b.y) {
                continue;
            }
            let total = crate::scene::metric_len(a, b, cos_lat);
            if !(total > 0.0) {
                continue;
            }
            chords += 1;
            // Evenly spaced cell centres, each standing for its own slice of
            // the chord: the measured bare length is then bounded by the chord
            // it was measured on, which a march to `total` inclusive is not.
            let n = (total / CHORD_STEP_M).ceil().max(1.0) as usize;
            let cell = total / n as f64;
            let mut bare = 0.0f64;
            let mut worst_at = a;
            let (mut run, mut best_run, mut run_start) = (0usize, 0usize, 0usize);
            for k in 0..n {
                let f = (k as f64 + 0.5) / n as f64;
                let p = Coord { x: a.x + (b.x - a.x) * f, y: a.y + (b.y - a.y) * f };
                if over_hosts(m, &hosts, anchor, p) {
                    run = 0;
                    continue;
                }
                bare += cell;
                if run == 0 {
                    run_start = k;
                }
                run += 1;
                // The place to look is the middle of the *longest* stretch of
                // misplaced paint, not wherever the last one happened to end.
                if run > best_run {
                    best_run = run;
                    let g = (run_start as f64 + run as f64 * 0.5) / n as f64;
                    worst_at = Coord { x: a.x + (b.x - a.x) * g, y: a.y + (b.y - a.y) * g };
                }
            }
            dist.push(bare);
            if bare > CHORD_BARE_M {
                worst.offer(Offender {
                    lon: worst_at.x,
                    lat: worst_at.y,
                    zoom: m.solved.z_ref,
                    value: bare,
                    note: format!(
                        "{bare:.1} m of a {total:.0} m zebra chord lies off the {} carriageway{} \
                         it crosses",
                        hosts.len(),
                        if hosts.len() == 1 { "" } else { "s" },
                    ),
                });
            }
        }
    }
    if debug {
        eprintln!(
            "[strip] {chords} zebra chords scored; {hostless} crossings registered a chord \
             while crossing no carriageway centerline"
        );
    }

    Metric {
        id: "street.crossing_extent".into(),
        invariant: Invariant::I4,
        title: "Zebra painted off the carriageway it crosses".into(),
        population: format!(
            "Every registered crossing chord ({chords} scored) with an end inside the bbox, \
             marched at {CHORD_STEP_M} m and scored as the total length lying further than its \
             own drawn half-width from every corridor the *mapped crosswalk polyline actually \
             intersects*. Crossings whose polyline crosses no drivable centerline are out of the \
             population ({hostless} of them) — nothing owes them a width — and so is a chord \
             wholly outside the bbox."
        ),
        detail: format!(
            "A crossing crosses one street, so its chord is that street's cross-section at that \
             station. `synth::walkway::crossings` instead extends the mapped stub \
             {:.0} m at each end and marches for *any* corridor within its prior half-width, \
             merging intervals across {:.0} m — so a crossing running alongside a forecourt's \
             service roads annexes them into one chord. Measuring with the same predicate would \
             read zero by construction; this asks the question the fix will answer. Past \
             {CHORD_BARE_M:.1} m the ladder is painted on something that is not the road being \
             crossed, and `paint.buried` finds the same site from the other side when that \
             something is a hillside.",
            crate::synth::walkway::CROSSING_EXTEND_M,
            crate::synth::walkway::CROSSING_MERGE_M,
        ),
        sense: Sense::HigherIsWorse,
        threshold: CHORD_BARE_M,
        skipped: (chords == 0)
            .then(|| "no crossing registered a chord in this extract".to_string()),
        dist,
        worst: worst.into_vec(),
    }
}

/// How far each registered chord lies from square across the street it
/// crosses.
fn crossing_skew(m: &Model<'_>) -> Metric {
    let mut dist = Dist::new(0.0, 90.0);
    let mut worst = Worst::new(Sense::HigherIsWorse, 8);
    let mut dem = m.terrain.and_then(|p| crate::dem::Dem::open(p).ok());

    // The same centerline index crossing_extent scans, for the same reason.
    let mut grid = crate::assemble::grid::GridIndex::with_cell_m(64.0);
    let mut edges: Vec<(u32, u32)> = Vec::new();
    for (ci, c) in m.scene.corridors.iter().enumerate() {
        if c.kind.prior().surface != Surface::Asphalt || corridor_half_width_m(c).is_none() {
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

    /// How far from a centerline intersection a chord's midpoint may lie and
    /// still be the chord that paints that crossing, in metres. Registration
    /// moves chords *along* the mapped line, so the right chord is close;
    /// pairing by "anything within reach" instead scored corner crossings
    /// against the *other* street of their junction, which reads ~90° however
    /// square both ladders are.
    const PAIR_REACH_M: f64 = 15.0;

    let mut scratch: Vec<u32> = Vec::new();
    let mut chords = 0u64;
    for (line, _) in m.scene.walks.lines() {
        if !line.crosswalk || line.line.len() < 2 {
            continue;
        }
        let Some(painted) = m.crossings.get(&line.source) else { continue };
        let cos_lat = crate::scene::run_cos_lat(&line.line);
        let anchor = dem
            .as_mut()
            .map(|d| walkway::crossing_level_anchor(&line.line, d, m.solved.z_ref));
        // Every (tangent, place) where the mapped polyline properly crosses a
        // drivable centerline at the crossing's own level: the streets this
        // crossing crosses, each with the direction traffic runs where it is
        // crossed. The level gate is what keeps a terrace crosswalk from
        // being scored against the avenue below it, which read as 80–89° of
        // skew on ladders that are square to their own street.
        let ext = extended_line(&line.line, cos_lat);
        let mut hosts: Vec<(f64, f64, Coord)> = Vec::new();
        for w in ext.windows(2) {
            let bbox = (
                w[0].x.min(w[1].x),
                w[0].y.min(w[1].y),
                w[0].x.max(w[1].x),
                w[0].y.max(w[1].y),
            );
            grid.query(bbox, &mut scratch);
            for &e in scratch.iter() {
                let (ci, ni) = edges[e as usize];
                let c = &m.scene.corridors[ci as usize];
                let (t0, t1) = (c.nodes[ni as usize], c.nodes[ni as usize + 1]);
                if !segments_cross(w[0], w[1], t0, t1) {
                    continue;
                }
                if !walkway::incidence_ok(w[0], w[1], t0, t1, c.cos_lat) {
                    continue;
                }
                {
                    let mid = Coord { x: (w[0].x + w[1].x) * 0.5, y: (w[0].y + w[1].y) * 0.5 };
                    if !walkway::host_level_ok(m.solved, ci, mid, anchor) {
                        continue;
                    }
                }
                let (tx, ty) = ((t1.x - t0.x) * cos_lat, t1.y - t0.y);
                let len = tx.hypot(ty);
                if len > 0.0 {
                    let mid = Coord { x: (w[0].x + w[1].x) * 0.5, y: (w[0].y + w[1].y) * 0.5 };
                    hosts.push((tx / len, ty / len, mid));
                }
            }
        }
        // One chord answers for each crossed street: the one whose midpoint
        // lies nearest the crossing point, with an end in the bbox.
        let mut scored: std::collections::HashSet<usize> = std::collections::HashSet::new();
        for &(tx, ty, at) in &hosts {
            let mut best: Option<(f64, f64, f64, Coord)> = None; // (dist, ux, uy, mid)
            for &crate::synth::walkway::Chord { a, b, .. } in painted.iter() {
                if !m.bounds.contains(a.x, a.y) && !m.bounds.contains(b.x, b.y) {
                    continue;
                }
                let (ux, uy) = ((b.x - a.x) * cos_lat, b.y - a.y);
                let len = ux.hypot(uy);
                if !(len > 0.0) {
                    continue;
                }
                let mid = Coord { x: (a.x + b.x) * 0.5, y: (a.y + b.y) * 0.5 };
                let d = crate::scene::metric_len(mid, at, cos_lat);
                if d <= PAIR_REACH_M && best.as_ref().is_none_or(|(bd, ..)| d < *bd) {
                    best = Some((d, ux / len, uy / len, mid));
                }
            }
            let Some((_, ux, uy, mid)) = best else { continue };
            scored.insert((mid.x.to_bits() as usize) ^ (mid.y.to_bits() as usize));
            let skew = (ux * tx + uy * ty).abs().min(1.0).asin().to_degrees();
            if std::env::var_os("ARPT_DEBUG_STRIP").is_some() {
                eprintln!("[skew] {skew:6.1} deg, chord mid {:.6},{:.6}", mid.x, mid.y);
            }
            dist.push(skew);
            if skew > CHORD_SKEW_DEG {
                worst.offer(Offender {
                    lon: mid.x,
                    lat: mid.y,
                    zoom: m.solved.z_ref,
                    value: skew,
                    note: format!(
                        "the chord crosses its street {skew:.0}° off square — every \
                         zebra bar is drawn {skew:.0}° off the traffic axis"
                    ),
                });
            }
        }
        chords += scored.len() as u64;
    }

    Metric {
        id: "street.crossing_skew".into(),
        invariant: Invariant::I4,
        title: "Crossing chord off square to the street it crosses".into(),
        population: format!(
            "One sample per (crossed centerline, chord) pair ({chords} chords scored): \
             every place a crossing's own mapped polyline properly intersects a drivable \
             centerline, scored against the crossing's *nearest* registered chord within \
             15 m — the chord that paints that crossing point. Pairing by proximity alone \
             instead scored corner crossings against the other street of their junction, \
             which reads ~90° however square both ladders are. A chord near no \
             intersection yields nothing — what it crosses is `street.crossing_extent`'s \
             question — and a crossing whose polyline crosses no centerline is out of the \
             population entirely."
        ),
        detail: format!(
            "The angle between the chord and square-across the crossed centerline, in \
             degrees. A registration metric, not a drawn one, since the R7 finish: the \
             bars run along the crossed street's own tangent whatever the chord does \
             (`synth::markings::crossing_bars`, shear bounded at 45°; \
             `ARPT_NO_BAR_TRAFFIC` restores chord-square bars), so an oblique chord \
             now draws a *sheared* ladder whose stripes still lie with traffic rather \
             than a rotated one. Obliquity can be real \
             (a refuge island, a bent kerb), so the {CHORD_SKEW_DEG:.0}° gate is loose; \
             what it catches is the chord *derivation* pairing kerb points that are not \
             opposite each other — which still misplaces the ladder in plan, however \
             correctly its stripes now lean."
        ),
        sense: Sense::HigherIsWorse,
        threshold: CHORD_SKEW_DEG,
        skipped: (chords == 0)
            .then(|| "no registered chord crosses a centerline in this extract".to_string()),
        dist,
        worst: worst.into_vec(),
    }
}

/// Whether `p` lies on the drawn carriageway of one of `hosts`, at the
/// crossing's own level — the same per-sample gate the registration marches
/// with (`walkway::on_asphalt`): a host is a whole spliced corridor, and
/// without the gate a hairpin's upper arm answers for the lower one.
fn over_hosts(m: &Model<'_>, hosts: &[(u32, Vec<f64>)], anchor: Option<f64>, p: Coord) -> bool {
    hosts.iter().any(|(ci, arcs)| {
        let ci = *ci;
        if !walkway::host_level_ok(m.solved, ci, p, anchor) {
            return false;
        }
        let c = &m.scene.corridors[ci as usize];
        let Some(half) = corridor_half_width_m(c) else { return false };
        (0..c.nodes.len().saturating_sub(1)).any(|i| {
            arcs.iter().any(|&x| {
                c.arc[i] <= x + walkway::CROSSING_ARC_WINDOW_M
                    && c.arc[i + 1] >= x - walkway::CROSSING_ARC_WINDOW_M
            }) && point_to_segment_m(p, c.nodes[i], c.nodes[i + 1], c.cos_lat) <= half
        })
    })
}

// ---------------------------------------------------------------- width step

/// Change in drawn width across a shared vertex, past which a ribbon reads as
/// a different object either side of it, in metres.
///
/// Zero by construction once the fit tapers instead of stepping, so this is an
/// epsilon on the arithmetic rather than a tolerance on the model.
const WIDTH_STEP_M: f64 = 0.01;

/// Every shared vertex of a pedestrian run, scored by how much the drawn width
/// jumps across it.
///
/// **Continuity of a ribbon is two properties, not one.** `strip_continuity`
/// asks whether the surface is there; this asks whether it is the *same
/// surface* along its length. `fit_to_ground` resolves the earthwork per
/// *segment*, so before the taper a path across a flank stepped between 1.2 m
/// and 2.0 m at every station — measured at Territet, which is where it was
/// reported from: `path/track` p10 1.20 against p50 2.00, and 26 % of ways
/// varying along themselves.
///
/// The taper is bounded by construction and this is what says so: a shared
/// vertex takes the **narrower** of the two segments' allowances, so the width
/// interpolated across either segment never exceeds what the fit allowed for
/// that segment at any station. A band may give width up and may never take it,
/// said once more in a third place.
fn width_step(m: &Model<'_>) -> Metric {
    let mut dist = Dist::metres();
    let mut worst = Worst::new(Sense::HigherIsWorse, 8);
    let mut pairs = 0u64;
    for i in 1..m.junctions.source_count() as u32 {
        let (prev, next) = (m.junctions.source(i - 1), m.junctions.source(i));
        if !matches!(prev.surface, Surface::Walkway | Surface::Path) {
            continue;
        }
        // The same test `synth::pavement::runs` chains on, less `half_m` —
        // which is exactly the field this metric exists because the pavement
        // stopped varying.
        let chains = prev.surface == next.surface
            && prev.level == next.level
            && prev.layer == next.layer
            && prev.corridor == next.corridor
            && prev.b.x == next.a.x
            && prev.b.y == next.a.y;
        if !chains || !m.bounds.contains(prev.b.x, prev.b.y) {
            continue;
        }
        pairs += 1;
        // Full width, not half: it is the ribbon a person sees.
        let step = (prev.sect_b.reach_m() - next.sect_a.reach_m()).abs() * 2.0;
        dist.push(step);
        if step > WIDTH_STEP_M {
            worst.offer(Offender {
                lon: prev.b.x,
                lat: prev.b.y,
                zoom: m.solved.z_ref,
                value: step,
                note: format!(
                    "a {} ribbon changes width by {step:.2} m across one vertex ({:.2} m to {:.2} m)",
                    if prev.corridor == crate::synth::walkway::NO_HOST { "free" } else { "street" },
                    prev.sect_b.reach_m() * 2.0,
                    next.sect_a.reach_m() * 2.0,
                ),
            });
        }
    }

    Metric {
        id: "street.width_step".into(),
        invariant: Invariant::I6,
        title: "Pedestrian ribbon changing width mid-run".into(),
        population: format!(
            "Every shared vertex of two consecutive walk or path band segments that \
             `synth::pavement` will chain into one buffered polyline ({pairs} inside the bbox), \
             scored as the change in *drawn* width across it. The chaining test is the union's \
             own, less `half_m` — which is the field the pavement stopped varying, and the \
             reason a ribbon can now taper at all instead of breaking into a slab per rung."
        ),
        detail: format!(
            "A pavement's width may vary for a reason a viewer can see — a facade steps in, a \
             flank steepens — and reads as a taper. What reads as a different object every few \
             metres is a width resolved per *segment* and applied as a step: `fit_to_ground` \
             decides one number per segment, so before the taper the two sides of every shared \
             vertex disagreed. Past {WIDTH_STEP_M:.2} m the ribbon is drawn with a visible \
             shoulder mid-run."
        ),
        sense: Sense::HigherIsWorse,
        threshold: WIDTH_STEP_M,
        skipped: (pairs == 0).then(|| "no pedestrian run in this extract".to_string()),
        dist,
        worst: worst.into_vec(),
    }
}

// ------------------------------------------------------------ sizing the prior

/// How far a wall may stand from a street's centerline and still make it a
/// built-up street, in metres. Three values, so the sensitivity to this number
/// is visible rather than assumed.
const BUILT_UP_REACH_M: [f64; 3] = [15.0, 25.0, 40.0];

/// Station spacing for the built-up walk, in metres — coarse, because the
/// question is what a whole street is, not what one metre of it is.
const BUILT_UP_STEP_M: f64 = 10.0;

/// Share of a corridor's measured at-grade length that must have walls on both
/// sides for the corridor to count as built-up.
const BUILT_UP_SHARE: f64 = 0.5;

/// **Sizing the synthesis prior, before writing it.** Under `ARPT_DEBUG_SYNTH`,
/// per class: how much street side-length there is inside the extract, how much
/// of it the data already claims a pavement for, and how much of it a
/// "built-up" test would hand one to.
///
/// The question is not whether synthesizing pavement is a good idea but *how
/// much of the world it invents*. Handing a sidewalk to every residential street
/// is only defensible where the mapped data is sparse **and** the street is
/// genuinely urban, and the second half is the load-bearing claim: facade
/// proximity on one side would pave a mountain road that passes a single barn.
/// So the test is walls on **both** sides — a street is a room, and a room needs
/// two walls.
///
/// **Clipped to the bbox, and that is not a detail.** `assemble` admits whole
/// parquet row groups, so the corridor set runs far past the extract into ground
/// no building input covers; measured unclipped, every rural kilometre reads
/// "no walls" and the built-up share of *residential* came out at 0.2 %, which
/// is a fact about where the footprints were loaded and not about Montreux.
fn synth_census(m: &Model<'_>) {
    if std::env::var_os("ARPT_DEBUG_SYNTH").is_none() {
        return;
    }
    // Per class, split by whether the corridor is built-up:
    // [side-km, mapped side-km] for built-up and for not, plus the walled
    // share at each reach for the sensitivity column.
    let mut by: HashMap<&'static str, [f64; 7]> = HashMap::new();
    let mut scratch: Vec<u32> = Vec::new();
    // **Merged**, not summed: several ways claim one stretch of kerb routinely,
    // and adding their lengths reported 322 % of a primary as mapped.
    let mut raw: HashMap<(u32, u8), Vec<(f64, f64, u64)>> = HashMap::new();
    for (line, attached) in m.scene.walks.lines() {
        for a in attached {
            raw.entry((a.host, a.side)).or_default().push((a.arc0, a.arc1, line.source));
        }
    }
    let claimed_m: HashMap<(u32, u8), Vec<(f64, f64)>> = raw
        .into_iter()
        .map(|(k, mut v)| {
            v.sort_by(|x, y| x.0.total_cmp(&y.0));
            (k, merge_ranges(&v.iter().map(|&(a, b, _)| (a, b)).collect::<Vec<_>>(), 0.0))
        })
        .collect();

    for c in &m.scene.corridors {
        let Some(half_m) = corridor_half_width_m(c) else { continue };
        if c.kind.prior().surface != crate::priors::Surface::Asphalt {
            continue;
        }
        let mut grade: Vec<(f64, f64)> = Vec::new();
        let mut walled = [0.0f64; 3];
        let mut grade_m = 0.0f64;
        for (r0, r1, _, kind) in crate::synth::carriageway::level_runs(c) {
            if kind != SpanKind::Grade {
                continue;
            }
            let mut a = r0;
            let mut seen0 = f64::NAN;
            while a < r1 {
                let b = (a + BUILT_UP_STEP_M).min(r1);
                let mid = 0.5 * (a + b);
                let p = point_at_arc(m, c, mid);
                if !m.bounds.contains(p.x, p.y) {
                    a = b;
                    continue;
                }
                if seen0.is_nan() {
                    seen0 = a;
                }
                let step = b - a;
                grade_m += step;
                grade.push((a, b));
                let q = point_at_arc(m, c, (mid + 1.0).min(r1));
                let m_lon = DEG_M * c.cos_lat;
                let (dx, dy) = ((q.x - p.x) * m_lon, (q.y - p.y) * DEG_M);
                let len = dx.hypot(dy);
                if len > 0.0 {
                    for (k, &reach) in BUILT_UP_REACH_M.iter().enumerate() {
                        let r = half_m + reach;
                        let room = m.facades.room(
                            p,
                            c.cos_lat,
                            (dx / len, dy / len),
                            r,
                            crate::synth::carriageway::ROOM_WINDOW_MAX_M,
                            &mut scratch,
                        );
                        // A street is a room, and a room needs two walls.
                        if room.left < r && room.right < r {
                            walled[k] += step;
                        }
                    }
                }
                a = b;
            }
        }
        if !(grade_m > 0.0) {
            continue;
        }
        // Mapped length, clipped to the same stretches the walk measured, so
        // the two shares are over one denominator.
        let mut mapped = 0.0f64;
        for side in [0u8, 1] {
            let Some(spans) = claimed_m.get(&(c.id, side)) else { continue };
            for &(s0, s1) in spans {
                for &(g0, g1) in &grade {
                    mapped += (s1.min(g1) - s0.max(g0)).max(0.0);
                }
            }
        }
        let built_up = walled[1] >= BUILT_UP_SHARE * grade_m;
        let e = by.entry(class_name(c.kind)).or_default();
        let base = usize::from(!built_up) * 2;
        e[base] += 2.0 * grade_m;
        e[base + 1] += mapped;
        for k in 0..3 {
            e[4 + k] += 2.0 * walled[k];
        }
    }
    let mut names: Vec<&'static str> = by.keys().copied().collect();
    names.sort_unstable();
    eprintln!(
        "[synth]                  BUILT-UP (>={:.0} % both-walled at {} m)      NOT BUILT-UP        \
         walled % at {:?} m",
        100.0 * BUILT_UP_SHARE,
        BUILT_UP_REACH_M[1],
        BUILT_UP_REACH_M,
    );
    eprintln!(
        "[synth] class             side-km  mapped %  UNMAPPED km   side-km  mapped %"
    );
    for n in names {
        let e = by[n];
        if e[0] + e[2] < 200.0 {
            continue;
        }
        let pct = |num: f64, den: f64| if den > 0.0 { 100.0 * num / den } else { 0.0 };
        eprintln!(
            "[synth] {n:<16} {:>8.2} {:>8.1}  {:>10.2}  {:>8.2} {:>8.1}   {:>5.1} {:>5.1} {:>5.1}",
            e[0] / 1000.0,
            pct(e[1], e[0]),
            (e[0] - e[1]) / 1000.0,
            e[2] / 1000.0,
            pct(e[3], e[2]),
            pct(e[4], e[0] + e[2]),
            pct(e[5], e[0] + e[2]),
            pct(e[6], e[0] + e[2]),
        );
    }
}

/// The class name the synthesis prior is keyed on.
fn class_name(kind: crate::priors::Kind) -> &'static str {
    use crate::priors::{Kind, RoadClass};
    match kind {
        Kind::Road(RoadClass::Motorway) => "motorway",
        Kind::Road(RoadClass::Trunk) => "trunk",
        Kind::Road(RoadClass::Primary) => "primary",
        Kind::Road(RoadClass::Secondary) => "secondary",
        Kind::Road(RoadClass::Tertiary) => "tertiary",
        Kind::Road(RoadClass::Unclassified) => "unclassified",
        Kind::Road(RoadClass::Residential) => "residential",
        Kind::Road(RoadClass::LivingStreet) => "living_street",
        Kind::Road(RoadClass::Service) => "service",
        Kind::Road(RoadClass::Unknown) => "unknown",
        Kind::Road(RoadClass::Track) => "track",
        _ => "other",
    }
}

// -------------------------------------------------------------------- shared

/// Intervals merged where they overlap or come within `slack` of each other.
fn merge_ranges(sorted: &[(f64, f64)], slack: f64) -> Vec<(f64, f64)> {
    let mut out: Vec<(f64, f64)> = Vec::new();
    for &(lo, hi) in sorted {
        match out.last_mut() {
            Some(last) if lo - last.1 <= slack => last.1 = last.1.max(hi),
            _ => out.push((lo, hi)),
        }
    }
    out
}

/// The parts of `[lo, hi]` where the corridor is on the ground.
fn grade_ranges(c: &Corridor, lo: f64, hi: f64) -> Vec<(f64, f64)> {
    if c.spans.is_empty() {
        return vec![(lo, hi)];
    }
    c.spans
        .iter()
        .filter(|s| s.kind == SpanKind::Grade)
        .filter_map(|s| {
            let (a, b) = (s.arc0.max(lo), s.arc1.min(hi));
            (b > a).then_some((a, b))
        })
        .collect()
}

/// A corridor's point at arc `s` — the smoothed profile where the solve gave it
/// one, exactly as `walkway::attached_band` stations a band, so the probe and
/// the band it is looking for stand on the same curve.
fn point_at_arc(m: &Model<'_>, c: &Corridor, s: f64) -> Coord {
    match m.solved.profile(c.id) {
        Some(p) => p.smooth_at_arc(s),
        None => crate::synth::carriageway::raw_point_at_arc(c, s),
    }
}

/// The unit normal at arc `s` pointing to `side` — 0 left of the direction of
/// travel, 1 right, the handedness `assemble::walks` measured the offset with.
fn outward_normal(m: &Model<'_>, c: &Corridor, s: f64, side: u8) -> (f64, f64) {
    let total = c.arc.last().copied().unwrap_or(0.0);
    let d = STATION_M.min(total.max(1e-6));
    let a = point_at_arc(m, c, (s - d).max(0.0));
    let b = point_at_arc(m, c, (s + d).min(total));
    let m_lon = DEG_M * c.cos_lat;
    let (dx, dy) = ((b.x - a.x) * m_lon, (b.y - a.y) * DEG_M);
    let len = dx.hypot(dy);
    if !(len > 0.0) {
        return (0.0, 0.0);
    }
    let sgn = if side == 0 { 1.0 } else { -1.0 };
    (-sgn * dy / len, sgn * dx / len)
}

/// `p` moved `d` metres along a local east/north unit vector.
fn offset(p: Coord, n: (f64, f64), d: f64, cos_lat: f64) -> Coord {
    let m_lon = DEG_M * cos_lat;
    Coord { x: p.x + n.0 * d / m_lon, y: p.y + n.1 * d / DEG_M }
}

/// `(distance in metres, parameter along a→b)` of the closest point to `p`.
fn project(p: Coord, a: Coord, b: Coord, cos_lat: f64) -> (f64, f64) {
    let m_lon = DEG_M * cos_lat;
    let (ex, ey) = ((b.x - a.x) * m_lon, (b.y - a.y) * DEG_M);
    let (qx, qy) = ((p.x - a.x) * m_lon, (p.y - a.y) * DEG_M);
    let len2 = ex * ex + ey * ey;
    let u = if len2 > 0.0 { ((qx * ex + qy * ey) / len2).clamp(0.0, 1.0) } else { 0.0 };
    ((qx - ex * u).hypot(qy - ey * u), u)
}

fn point_to_segment_m(p: Coord, a: Coord, b: Coord, cos_lat: f64) -> f64 {
    project(p, a, b, cos_lat).0
}

/// Which side of the directed edge `a`→`b` the point lies on: 0 left, 1 right.
fn side_of(p: Coord, a: Coord, b: Coord, cos_lat: f64) -> usize {
    let m_lon = DEG_M * cos_lat;
    let (ex, ey) = ((b.x - a.x) * m_lon, (b.y - a.y) * DEG_M);
    let (qx, qy) = ((p.x - a.x) * m_lon, (p.y - a.y) * DEG_M);
    usize::from(ex * qy - ey * qx <= 0.0)
}

/// The drawn cross-section at parameter `u` along a segment.
fn lerp_section(s: &SourceSeg, u: f64) -> Section {
    Section {
        left_m: s.sect_a.left_m + (s.sect_b.left_m - s.sect_a.left_m) * u,
        right_m: s.sect_a.right_m + (s.sect_b.right_m - s.sect_a.right_m) * u,
    }
}

/// `(plan distance between two segments, parameter along b₀→b₁ of the closest
/// point on it)`, in metres — zero distance where they cross.
///
/// The parameter comes back because the band being measured against is not one
/// width: it is drawn at `sect_a` at one end and `sect_b` at the other, and the
/// question is whether *this* place on it covers the strip.
fn segment_distance_m(
    a0: Coord,
    a1: Coord,
    b0: Coord,
    b1: Coord,
    cos_lat: f64,
) -> (f64, f64) {
    if segments_cross(a0, a1, b0, b1) {
        // Crossing: the contact is somewhere inside both, and either end's
        // width would do. Take the midpoint rather than an arbitrary end.
        return (0.0, 0.5);
    }
    let mut best = (f64::MAX, 0.0);
    // The closest point on b to either end of a…
    for p in [a0, a1] {
        let (d, u) = project(p, b0, b1, cos_lat);
        if d < best.0 {
            best = (d, u);
        }
    }
    // …and the closest point on a to either end of b, whose parameter on b is
    // that end's own.
    for (p, u) in [(b0, 0.0), (b1, 1.0)] {
        let d = point_to_segment_m(p, a0, a1, cos_lat);
        if d < best.0 {
            best = (d, u);
        }
    }
    best
}

/// Whether the two segments properly intersect. Degree space throughout: the
/// orientation of four points is a sign, and an anisotropic scaling of the
/// plane does not change it.
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn two_claims_within_the_corner_slack_are_one_extent() {
        let got = merge_ranges(&[(0.0, 20.0), (28.0, 51.0), (400.0, 410.0)], 25.0);
        assert_eq!(got, vec![(0.0, 51.0), (400.0, 410.0)]);
    }

    #[test]
    fn a_grade_range_is_clipped_to_the_spans_on_the_ground() {
        let c = Corridor {
            spans: vec![
                crate::scene::Span { arc0: 0.0, arc1: 40.0, level: 0, kind: SpanKind::Grade },
                crate::scene::Span { arc0: 40.0, arc1: 90.0, level: 1, kind: SpanKind::Bridge },
                crate::scene::Span { arc0: 90.0, arc1: 150.0, level: 0, kind: SpanKind::Grade },
            ],
            ..corridor()
        };
        assert_eq!(grade_ranges(&c, 10.0, 120.0), vec![(10.0, 40.0), (90.0, 120.0)]);
    }

    #[test]
    fn the_side_is_the_handedness_the_attachment_measured_with() {
        // A due-east edge; north of it is left of the direction of travel.
        let a = Coord { x: 0.0, y: 46.0 };
        let b = Coord { x: 0.001, y: 46.0 };
        let north = Coord { x: 0.0005, y: 46.0001 };
        let south = Coord { x: 0.0005, y: 45.9999 };
        assert_eq!(side_of(north, a, b, 0.7), 0);
        assert_eq!(side_of(south, a, b, 0.7), 1);
    }

    #[test]
    fn a_chord_running_beside_a_road_does_not_cross_it() {
        let a = Coord { x: 0.0, y: 46.0 };
        let b = Coord { x: 0.001, y: 46.0 };
        // Parallel, 10 m north.
        let c = Coord { x: 0.0, y: 46.0001 };
        let d = Coord { x: 0.001, y: 46.0001 };
        assert!(!segments_cross(a, b, c, d));
        // Perpendicular, through it.
        let e = Coord { x: 0.0005, y: 45.9999 };
        let f = Coord { x: 0.0005, y: 46.0001 };
        assert!(segments_cross(a, b, e, f));
    }

    fn corridor() -> Corridor {
        Corridor {
            id: 0,
            nodes: vec![Coord { x: 0.0, y: 46.0 }, Coord { x: 0.001, y: 46.0 }],
            arc: vec![0.0, 150.0],
            cos_lat: 0.7,
            kind: crate::priors::Kind::Road(crate::priors::RoadClass::Residential),
            class_key: "residential".into(),
            link: false,
            width_m: Some(6.0),
            spans: Vec::new(),
            segments: Vec::new(),
            connectors: Vec::new(),
        }
    }
}
