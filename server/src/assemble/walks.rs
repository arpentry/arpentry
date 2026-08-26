//! Stage 1 — which street a pedestrian way belongs to.
//!
//! A sidewalk is not a feature; it is a *relation*. Overture maps the footway
//! beside Rue du Marché as an independent draped line with its own id, and
//! nothing in the data says the two are one street — so the tiler drapes the
//! footway on whatever the hillside does while the carriageway rides its bench,
//! and 34.6 % of tagged sidewalk length ends up more than 2.5 m outside the
//! bench of the street it serves (`data/plans`, the sidewalks-and-facades
//! study).
//!
//! This module states the relation and nothing else: a draped pedestrian line
//! that runs alongside a street, close enough to be part of its cross-section,
//! is **attached** to that street over an [arc range](Attachment) on one side.
//! What is done with the relation — the band, its material, its kerb rise —
//! belongs to the consumers.
//!
//! **A relation, not a promotion.** No pedestrian feature enters the scene
//! graph, gets a corridor, or solves; GENERATION.md §4.2 is untouched.
//! Attachment says which street's finished cross-section a draped way rides,
//! which is a question about the *drawn* world and is settled long after the
//! solve has decided the heights.
//!
//! **The definition lives here, and the check reads it back.** The archive-side
//! `contact.sidewalk_grade` already had to decide what a sidewalk was in order
//! to score one, and it decided in archive space with its own constants. Those
//! constants are now [`priors::WALK_ATTACH_M`] and friends, read by both: a
//! model that attached one population while the check scored another would
//! report a metric about nothing.

use std::collections::{HashMap, HashSet};

use geo_types::Coord;

use crate::priors::{self, Kind, Surface};
use crate::scene::{Corridor, CorridorId, DEG_M};

use super::columns::{SubclassRun, SPAN_EPS};
use super::grid::GridIndex;

/// Sample spacing along a pedestrian line, in metres. The same metre the
/// archive-side check resamples at, and finer than any cross-section feature
/// the relation is decided by.
const STATION_M: f64 = 1.0;

/// How far either side of a station the direction it is *running* is read
/// from, in metres.
///
/// Not the direction of the metre it stands on. A mapped footway is a dense
/// polyline and its per-vertex direction wanders by tens of degrees, so a
/// local tangent fails the [`priors::WALK_ALONG`] test at every kink — read
/// off the segment it stood on, 64 % of run breaks were a way momentarily
/// "turning across" a street it was running dead straight beside, which
/// shredded the relation into stubs. A five-metre chord is still short
/// compared with anything a sidewalk does relative to its street, and it is
/// the same reasoning `synth::carriageway::sections_along` gives for reading a
/// station's tangent as a central difference.
const TANGENT_HALF_M: f64 = 5.0;

/// Grid cell size for the host index, in metres. Sized to the query, which
/// reaches a carriageway half-width plus [`priors::WALK_ATTACH_M`].
const CELL_M: f64 = 64.0;

/// Coverage a way needs when it also shares a connector with a crosswalk — a
/// lower bar, because the crossing is independent evidence that the way is
/// street furniture. The plan-space study measured 57.8 % of tagged sidewalks
/// crosswalk-joined against 3.5 % of untagged ways, and cut the relaxed bar at
/// half a way's length.
///
/// It is worth 26 lines of 863 untagged detections in the Montreux read —
/// small, and kept: each is a sidewalk that would otherwise drape, and the
/// whole clause is one set of connector ids.
const WALK_COVER_JOINED: f64 = 0.5;

/// One draped pedestrian line, as the transportation read saw it — the input
/// to [`attach`]. Its geometry is kept only until the relation is resolved.
#[derive(Clone)]
pub struct WalkLine {
    /// Hash of the source feature id ([`crate::scene::source_hash`]).
    pub source: u64,
    pub line: Vec<Coord>,
    /// The §9 prior key. Kept so a consumer can tell a `steps` from a `footway`
    /// without re-parsing the class.
    pub kind: Kind,
    /// `subclass = 'sidewalk'` — the tag, the first of the two evidences.
    pub tagged: bool,
    /// `subclass = 'crosswalk'`. A crosswalk is paint on a carriageway
    /// (phase 6), never a band beside one, so it attaches to nothing — but the
    /// connectors it shares are evidence about the ways it joins.
    pub crosswalk: bool,
    /// The connectors the line touches — id and `at` fraction along the way,
    /// as Overture maps them. The ids are the shared-connector evidence; the
    /// fractions say *where* a way joins the network, which is what lets an
    /// endpoint know it is supposed to connect to something
    /// (`verify::model::network`).
    pub connectors: Vec<super::columns::Connector>,
    /// The stretches of the way that are *not* on the ground, as fractions of
    /// its length — a footbridge, a subway, a passage under a building.
    ///
    /// Carrying a span is still not a promotion (§4.2): nothing here solves.
    /// But a band is a piece of drawn ground, and there is no ground under a
    /// footbridge — the way carries its own fitted deck there
    /// (`synth::draped`), which *is* the walkway over that stretch. Banding it
    /// as well would draw a second one lying in the river.
    pub spans: Vec<(f64, f64)>,
    /// The stretches the way spends **indoors**, as fractions — Overture's
    /// `is_indoor` (`crate::levels::indoor_runs`).
    ///
    /// These are also in [`spans`], because indoors is one of the ways a
    /// stretch is not on the ground and that is the field the band suppression
    /// reads. They are kept separately as well because the two are not
    /// interchangeable *here*: a footbridge is fully spanned too and must keep
    /// the cartographic stroke that is all it has, while a way that is wholly
    /// indoors must not — its stroke would be a line drawn through a
    /// building's floor ([`WalkLine::is_wholly_indoor`]).
    ///
    /// [`spans`]: Self::spans
    pub indoor: Vec<(f64, f64)>,
}

impl WalkLine {
    /// Whether the way spends effectively all of its length indoors.
    ///
    /// **The one case where losing the band must also cost the feature.** I6's
    /// fallback — a way the surface model declined keeps its cartographic
    /// stroke — exists for a band there was no room for. It does not license
    /// drawing a footway through a terminal: that line *is* the spectacle I6
    /// forbids, and `order.walk_indoors` is what measures it. A way that is
    /// only partly indoors is not silenced, because the stretch outside is
    /// real and gets a band, which deletes the stroke by the ordinary rule.
    pub fn is_wholly_indoor(&self) -> bool {
        if self.indoor.is_empty() {
            return false;
        }
        let mut runs: Vec<(f64, f64)> = self.indoor.clone();
        runs.sort_by(|a, b| a.0.partial_cmp(&b.0).expect("finite fractions"));
        let (mut covered, mut reach) = (0.0, 0.0);
        for (s, e) in runs {
            let s = s.max(reach);
            if e > s {
                covered += e - s;
                reach = e;
            }
        }
        covered >= 1.0 - INDOOR_WHOLE_EPS
    }
}

/// How much of a way may lie outside its indoor runs and still count as wholly
/// indoors. A way is cut at connectors, not at doors, so the last fraction of
/// a percent is registration noise rather than a stretch of outdoor pavement.
const INDOOR_WHOLE_EPS: f64 = 0.02;

/// Cuts a pedestrian line into pieces of one subclass each.
///
/// **A mixed way cannot be one line.** The two subclasses that matter here are
/// answered in opposite directions: a sidewalk attaches to the street it runs
/// beside and is drawn as a band; a crosswalk attaches to nothing, because it
/// is paint on a carriageway ([`attach`] skips it outright). A single Overture
/// segment that is a sidewalk to the corner, a crossing over the road, and a
/// sidewalk again is all three, and one [`WalkLine`] can only be one of them.
/// So the line is cut where the subclass changes, and each piece answers for
/// itself.
///
/// The common segment does not move: where `subclass_rules` is absent, or
/// holds a single full-length run, this returns `base` untouched. Cutting only
/// happens for the segments `docs/SOURCES.md` §2 counts — the ones whose
/// scalar `subclass` Overture nulled because a rule was partial.
///
/// Fractions are of the segment's **length**, and are read through
/// [`crate::scene::cumulative_arc`] — the same parameterisation `connectors.at`
/// and `level_rules.between` are resolved with, so a cut lands where the source
/// says it does.
pub fn split_by_subclass(base: WalkLine, runs: &[SubclassRun]) -> Vec<WalkLine> {
    if runs.is_empty() || base.line.len() < 2 {
        return vec![base];
    }
    let intervals = subclass_intervals(runs);
    // One interval spanning the whole way is what a uniform segment looks like
    // from here: the scalar `subclass` said the same thing, and `base` already
    // carries it.
    if intervals.len() < 2 {
        return vec![base];
    }

    let arc = crate::scene::cumulative_arc(&base.line);
    let total = *arc.last().expect("a line with two nodes has an arc");
    if total <= 0.0 {
        return vec![base];
    }
    let intervals = absorb_slivers(intervals, total);
    if intervals.len() < 2 {
        // Every cut was a sliver: the way is one thing after all, and it is
        // whatever survived.
        let mut whole = base;
        let value = intervals.first().and_then(|iv| iv.2.as_deref());
        whole.tagged = value == Some("sidewalk");
        whole.crosswalk = value == Some("crosswalk");
        return vec![whole];
    }

    let mut out = Vec::with_capacity(intervals.len());
    for (t0, t1, value) in intervals {
        let line = cut(&base.line, &arc, t0 * total, t1 * total);
        if line.len() < 2 {
            continue;
        }
        let width = t1 - t0;
        out.push(WalkLine {
            source: base.source,
            line,
            kind: base.kind,
            tagged: value.as_deref() == Some("sidewalk"),
            crosswalk: value.as_deref() == Some("crosswalk"),
            // A connector on a cut boundary belongs to **both** neighbours: it
            // is the node they meet at, and the "joined to a crossing" evidence
            // is about what a way touches, not which side of it.
            connectors: base
                .connectors
                .iter()
                .filter(|c| c.at >= t0 - SPAN_EPS && c.at <= t1 + SPAN_EPS)
                .map(|c| super::columns::Connector {
                    id: c.id,
                    at: ((c.at - t0) / width).clamp(0.0, 1.0),
                })
                .collect(),
            spans: clip_runs(&base.spans, t0, t1, width),
            indoor: clip_runs(&base.indoor, t0, t1, width),
        });
    }
    if out.is_empty() { vec![base] } else { out }
}

/// The runs resolved into a partition of `0..1`: consecutive, non-overlapping,
/// each with the subclass in force there (`None` where no rule reaches).
///
/// Overlapping rules resolve last-wins, mirroring source order — the same rule
/// `assemble::corridors::resolve_spans` uses for overlapping level runs.
fn subclass_intervals(runs: &[SubclassRun]) -> Vec<(f64, f64, Option<String>)> {
    let mut breaks = vec![0.0, 1.0];
    for r in runs {
        breaks.push(r.start);
        breaks.push(r.end);
    }
    breaks.sort_by(|a, b| a.partial_cmp(b).expect("finite fractions"));
    breaks.dedup_by(|a, b| (*a - *b).abs() <= SPAN_EPS);

    let mut out: Vec<(f64, f64, Option<String>)> = Vec::new();
    for w in breaks.windows(2) {
        let (t0, t1) = (w[0], w[1]);
        if t1 - t0 <= SPAN_EPS {
            continue;
        }
        let mid = 0.5 * (t0 + t1);
        let value = runs
            .iter()
            .rev()
            .find(|r| mid >= r.start && mid <= r.end)
            .map(|r| r.value.clone());
        match out.last_mut() {
            Some(last) if last.2 == value => last.1 = t1,
            _ => out.push((t0, t1, value)),
        }
    }
    out
}

/// Merges away runs too short to be the thing they claim to be.
///
/// **A band shorter than it is wide is a joint, not a stretch of walkway.**
/// Overture references a crossing along a longer way, and the sidewalk left
/// either side of it is often the few centimetres between the kerb and the
/// crossing's endpoint — in the Montreux zone the sidewalk runs have a p25 of
/// 1.61 m and a minimum of 0.59 m. Cut on those and the walk band shatters
/// into pieces that each fit their own bench, which is a cross-fall step at
/// every joint; the first cut of this change cost `slope.walk_crossfall`
/// +0.086 pp for exactly that.
///
/// So a run under [`priors::WALK_WIDTH_M`] is absorbed into its longer
/// neighbour and takes that neighbour's subclass. A **crosswalk is exempt**:
/// it is paint, never a band, so "shorter than the band is wide" says nothing
/// about it — and the crossings Overture maps here are 2.03 m at their
/// shortest, well clear of the floor anyway.
///
/// Mirrors `corridors::resolve_spans`, which drops sub-`SNAP_RUN_M` grade
/// slivers wedged between two structures for the same reason.
fn absorb_slivers(
    mut intervals: Vec<(f64, f64, Option<String>)>,
    total_m: f64,
) -> Vec<(f64, f64, Option<String>)> {
    let min_frac = (priors::WALK_WIDTH_M / total_m).min(1.0);
    loop {
        let sliver = intervals
            .iter()
            .position(|iv| iv.1 - iv.0 < min_frac && iv.2.as_deref() != Some("crosswalk"));
        let Some(i) = sliver.filter(|_| intervals.len() > 1) else { break };
        // Into the longer neighbour, so a sliver between two runs joins the
        // one that can carry it.
        let take_prev = match (i > 0, i + 1 < intervals.len()) {
            (true, false) => true,
            (false, true) => false,
            _ => intervals[i - 1].1 - intervals[i - 1].0 >= intervals[i + 1].1 - intervals[i + 1].0,
        };
        let (lo, hi) = (intervals[i].0, intervals[i].1);
        if take_prev {
            intervals[i - 1].1 = hi;
        } else {
            intervals[i + 1].0 = lo;
        }
        intervals.remove(i);
        // Absorbing can leave two same-valued runs adjacent.
        intervals.dedup_by(|b, a| (a.2 == b.2).then(|| a.1 = b.1).is_some());
    }
    intervals
}

/// The runs of `runs` that fall inside `[t0, t1]`, renormalised onto it.
fn clip_runs(runs: &[(f64, f64)], t0: f64, t1: f64, width: f64) -> Vec<(f64, f64)> {
    runs.iter()
        .filter_map(|&(s, e)| {
            let (s, e) = (s.max(t0), e.min(t1));
            (e - s > SPAN_EPS).then(|| ((s - t0) / width, (e - t0) / width))
        })
        .collect()
}

/// The sub-polyline between two arc positions, with interpolated ends.
fn cut(line: &[Coord], arc: &[f64], a0: f64, a1: f64) -> Vec<Coord> {
    let mut out = vec![point_at_arc(line, arc, a0)];
    for (i, &a) in arc.iter().enumerate() {
        if a > a0 + CUT_EPS_M && a < a1 - CUT_EPS_M {
            out.push(line[i]);
        }
    }
    out.push(point_at_arc(line, arc, a1));
    out
}

/// The line's point at arc `s`, linearly interpolated between the bracketing
/// nodes (clamped to the ends).
fn point_at_arc(line: &[Coord], arc: &[f64], s: f64) -> Coord {
    let i = arc.partition_point(|&a| a < s).clamp(1, arc.len() - 1);
    let (a0, a1) = (arc[i - 1], arc[i]);
    let t = if a1 > a0 { ((s - a0) / (a1 - a0)).clamp(0.0, 1.0) } else { 0.0 };
    Coord {
        x: line[i - 1].x + (line[i].x - line[i - 1].x) * t,
        y: line[i - 1].y + (line[i].y - line[i - 1].y) * t,
    }
}

/// A source vertex closer than this to a cut lands on the cut instead of
/// spawning a millimetre-long first edge.
const CUT_EPS_M: f64 = 1e-6;

/// Why a way is attached. The tag and the geometry are independent evidences
/// and neither substitutes for the other: 34 % of tagged sidewalks fail the
/// geometric test (long runs beside wide roads), and the geometric test finds
/// a third again as much length as the tag does.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Evidence {
    /// `subclass='sidewalk'`, but under [`priors::WALK_COVER`] of its length
    /// runs with the street.
    Tag,
    /// Untagged, and running with the street for its length.
    Alongside,
    Both,
}

/// One stretch of one pedestrian way, riding one side of one street.
///
/// The arc range is in **host** corridor metres, because that is the space the
/// band will be built in: a sidewalk's shape comes from the centerline it
/// parallels, never from its own polyline, which is what makes the band
/// constant-width and kerb-aligned by construction. The way's *own* range
/// rides along ([`walk0`](Self::walk0)) so the stretches that attached to
/// nothing can be recovered as the complement — a path is drawn along its
/// whole length whether or not a street claimed part of it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Attachment {
    /// Source hash of the pedestrian feature.
    pub walk: u64,
    /// Its index in [`Walks::lines`].
    pub line: u32,
    /// The stretch of the way's *own* length this covers, in metres from its
    /// first vertex.
    pub walk0: f64,
    pub walk1: f64,
    /// Its class. A `steps` beside a street is attached to it — it is that
    /// street's stair — but it is not a band candidate on the same terms as a
    /// footway, since a staircase's whole purpose is to change height relative
    /// to what is beside it.
    pub kind: Kind,
    pub host: CorridorId,
    /// 0 left of the host's direction of travel, 1 right — indexed as
    /// [`crate::assemble::facades::Section::on`] and `ground::modifiers` index
    /// a side.
    pub side: u8,
    pub arc0: f64,
    pub arc1: f64,
    /// Mean distance from the host centerline over the range, metres.
    pub offset_m: f64,
    /// Half the spread of that distance — how much the way wanders relative to
    /// the centerline it parallels. A consumer that must decide whether a
    /// constant-width band can stand in for the mapped line reads this.
    pub spread_m: f64,
    pub evidence: Evidence,
}

impl Attachment {
    pub fn len_m(&self) -> f64 {
        self.arc1 - self.arc0
    }
}

/// What [`attach`] saw, so a census can be taken without re-reading the input.
/// Lengths are pedestrian-line metres except where they say otherwise.
#[derive(Debug, Default, Clone, Copy)]
pub struct Census {
    /// Draped pedestrian lines considered, and their total length.
    pub lines: u32,
    pub line_m: f64,
    /// Of those, the ones carrying `subclass='sidewalk'`.
    pub tagged_lines: u32,
    pub tagged_m: f64,
    /// Length whose nearest street is within reach — the geometric evidence
    /// before the along-versus-across test.
    pub covered_m: f64,
    pub tagged_covered_m: f64,
    /// Length running *alongside* a street, before the per-feature gate.
    pub alongside_m: f64,
    /// Lines that ended up with at least one attachment, and the pedestrian
    /// length and host arc those attachments cover.
    pub attached_lines: u32,
    pub attached_m: f64,
    pub host_arc_m: f64,
    /// Attached lines by which evidence carried them.
    pub tag_only: u32,
    pub alongside_only: u32,
    pub both: u32,
    /// Tagged sidewalks with no street within reach anywhere along them —
    /// misattached in the source, or genuinely separate paths.
    pub tagged_unhosted: u32,
    /// Untagged lines that only cleared the gate because they share a
    /// connector with a crosswalk — what that evidence is worth on its own.
    pub joined_only: u32,
    /// Attachment ranges dropped for being shorter than
    /// [`priors::WALK_ATTACH_MIN_M`], and their length.
    pub dropped_short: u32,
    pub dropped_short_m: f64,
    /// Why the runs end, counted at each break. A band is drawn per run, so
    /// what breaks a run decides how fragmented the drawn sidewalk is — and
    /// these are different findings. Crossing a *side* street is a real end to
    /// a band and wants a crosswalk, not a longer band; the way turning across
    /// its own host is the way leaving the street; and a change of host with
    /// no turn is a divided carriageway handing a sidewalk over.
    pub broke_lost: u32,
    pub broke_crossed: u32,
    pub broke_host: u32,
    pub broke_side: u32,
    pub broke_turned: u32,
}

/// Every pedestrian way's relation to the street it belongs to.
///
/// A side table, keyed both ways: the band is built per host corridor, while
/// the tiling phase meets the way itself and needs to know it is attached.
///
/// **The lines are kept, not consumed.** A way that attached to nothing is
/// still drawn — as a path across the ground rather than as a sidewalk — so
/// `synth::walkway` needs every pedestrian line and the stretches of it that
/// found a street, not only the second.
#[derive(Default)]
pub struct Walks {
    lines: Vec<WalkLine>,
    /// Attachments of `lines[i]`, as a half-open range into `attachments`.
    /// They are pushed line by line and in walk-arc order within a line, so
    /// the range is contiguous by construction.
    line_ranges: Vec<(u32, u32)>,
    attachments: Vec<Attachment>,
    by_host: HashMap<CorridorId, Vec<u32>>,
    by_source: HashMap<u64, Vec<u32>>,
    census: Census,
}

impl Walks {
    pub fn is_empty(&self) -> bool {
        self.attachments.is_empty()
    }

    pub fn len(&self) -> usize {
        self.attachments.len()
    }

    /// Every draped pedestrian line the run read, with what attached along it.
    /// A crosswalk is in the list and has no attachments — it is paint on a
    /// carriageway, never a band beside one.
    pub fn lines(&self) -> impl Iterator<Item = (&WalkLine, &[Attachment])> {
        self.lines.iter().zip(self.line_ranges.iter()).map(|(l, &(a, b))| {
            (l, &self.attachments[a as usize..b as usize])
        })
    }

    pub fn line_count(&self) -> usize {
        self.lines.len()
    }

    /// Every attachment, in a stable order (input order of the pedestrian
    /// lines, then along each line).
    pub fn all(&self) -> &[Attachment] {
        &self.attachments
    }

    /// What rides this corridor, in arc order.
    pub fn on_host(&self, host: CorridorId) -> impl Iterator<Item = &Attachment> {
        self.by_host
            .get(&host)
            .map(|v| v.as_slice())
            .unwrap_or(&[])
            .iter()
            .map(|&i| &self.attachments[i as usize])
    }

    /// Where this pedestrian feature is attached, if anywhere — what the
    /// tiling phase asks when it meets the way itself.
    pub fn of_walk(&self, walk: u64) -> impl Iterator<Item = &Attachment> {
        self.by_source
            .get(&walk)
            .map(|v| v.as_slice())
            .unwrap_or(&[])
            .iter()
            .map(|&i| &self.attachments[i as usize])
    }

    pub fn census(&self) -> &Census {
        &self.census
    }

    fn push(&mut self, a: Attachment) {
        let i = self.attachments.len() as u32;
        self.by_host.entry(a.host).or_default().push(i);
        self.by_source.entry(a.walk).or_default().push(i);
        self.attachments.push(a);
    }
}

/// Resolves every pedestrian line against the streets, and returns the
/// relation.
///
/// Hosts are the **asphalt** corridors. Rail is out for the reason it is out of
/// `order.building_overlap`: a platform beside a track is not a sidewalk beside
/// a street, and the formation's cross-section is not a street's.
pub fn attach(corridors: &[Corridor], walks: Vec<WalkLine>) -> Walks {
    let mut out = Walks::default();
    let hosts = Hosts::build(corridors);
    // The shared-connector evidence: which connectors a crosswalk touches. A
    // way joined to a crossing at its end is a way people step off a kerb from.
    let crossing_nodes: HashSet<u64> = walks
        .iter()
        .filter(|w| w.crosswalk)
        .flat_map(|w| w.connectors.iter().map(|c| c.id))
        .collect();
    let mut scratch: Vec<u32> = Vec::new();
    for (li, w) in walks.iter().enumerate() {
        // Every line gets a range, in step with `lines`, whether or not
        // anything attaches to it — the empty ones are the paths.
        let first = out.attachments.len() as u32;
        out.line_ranges.push((first, first));
        // A crossing is paint on a carriageway, and a **track** is not street
        // furniture — "a farm track beside a road is not its sidewalk". Both
        // stay in `lines`, so `synth::walkway` still bands them along their
        // own polylines; neither may take a street's cross-section.
        if w.crosswalk || !priors::is_pedestrian(w.kind) || w.line.len() < 2 {
            continue;
        }
        let cos_lat = crate::scene::run_cos_lat(&w.line);
        let stations = stations(&w.line, cos_lat, STATION_M);
        if stations.len() < 2 {
            continue;
        }
        let walk_m = length_m(&w.line, cos_lat);
        out.census.lines += 1;
        out.census.line_m += walk_m;
        if w.tagged {
            out.census.tagged_lines += 1;
            out.census.tagged_m += walk_m;
        }
        // One pass down the way: for each station, the street it is nearest and
        // whether it is running with that street or across it.
        let hits: Vec<Option<Hit>> =
            stations.iter().map(|s| hosts.nearest(s, cos_lat, &mut scratch)).collect();
        let covered = hits.iter().filter(|h| h.is_some()).count();
        let alongside = hits.iter().filter(|h| h.is_some_and(|h| h.along)).count();
        let covered_m = covered as f64 * STATION_M;
        out.census.covered_m += covered_m;
        out.census.alongside_m += alongside as f64 * STATION_M;
        if w.tagged {
            out.census.tagged_covered_m += covered_m;
            if covered == 0 {
                out.census.tagged_unhosted += 1;
            }
        }
        // The per-feature gate. The tag admits a way whose geometry falls
        // short; the geometry admits an untagged one; a way joined to a
        // crosswalk clears a lower bar, since the crossing is independent
        // evidence that it is street furniture.
        let cover = covered as f64 / hits.len() as f64;
        let joined = w.connectors.iter().any(|c| crossing_nodes.contains(&c.id));
        let by_cover = cover >= priors::WALK_COVER;
        let geometric = by_cover || (joined && cover >= WALK_COVER_JOINED);
        if !w.tagged && !geometric {
            continue;
        }
        if !w.tagged && !by_cover {
            out.census.joined_only += 1;
        }
        let evidence = match (w.tagged, geometric) {
            (true, true) => Evidence::Both,
            (true, false) => Evidence::Tag,
            (false, _) => Evidence::Alongside,
        };
        let before = out.len();
        let mut attached_m = 0.0;
        for run in runs(&hits, &mut out.census) {
            let hit = hits[run.0].expect("a run starts on a hit");
            let (mut lo, mut hi) = (f64::MAX, f64::MIN);
            let (mut near, mut far) = (f64::MAX, f64::MIN);
            let mut sum = 0.0;
            for h in hits[run.0..=run.1].iter().flatten() {
                lo = lo.min(h.arc);
                hi = hi.max(h.arc);
                near = near.min(h.offset_m);
                far = far.max(h.offset_m);
                sum += h.offset_m;
            }
            let n = (run.1 - run.0 + 1) as f64;
            // The host arc a way covers is not the way's own length: a
            // sidewalk cutting a corner is shorter than the street it rides,
            // and one wandering out and back is longer.
            if hi - lo < priors::WALK_ATTACH_MIN_M {
                out.census.dropped_short += 1;
                out.census.dropped_short_m += hi - lo;
                continue;
            }
            attached_m += n * STATION_M;
            out.push(Attachment {
                walk: w.source,
                line: li as u32,
                walk0: run.0 as f64 * STATION_M,
                walk1: run.1 as f64 * STATION_M,
                kind: w.kind,
                host: hit.host,
                side: hit.side,
                arc0: lo,
                arc1: hi,
                offset_m: sum / n,
                spread_m: (far - near) * 0.5,
                evidence,
            });
            out.census.host_arc_m += hi - lo;
        }
        out.line_ranges[li].1 = out.attachments.len() as u32;
        if out.len() > before {
            out.census.attached_lines += 1;
            out.census.attached_m += attached_m;
            match evidence {
                Evidence::Tag => out.census.tag_only += 1,
                Evidence::Alongside => out.census.alongside_only += 1,
                Evidence::Both => out.census.both += 1,
            }
        }
    }
    out.lines = walks;
    out
}

/// One station's answer: the street it is nearest, where on it, and whether it
/// is running with it.
#[derive(Debug, Clone, Copy)]
struct Hit {
    host: CorridorId,
    side: u8,
    arc: f64,
    offset_m: f64,
    along: bool,
}

/// A station on a pedestrian line: where it is and which way the line runs
/// through it.
type Station = (Coord, (f64, f64));

/// Maximal runs of consecutive stations attached to the same side of the same
/// street, as inclusive index ranges, counting why each one ended.
///
/// **No gap tolerance.** A break is a station that found no street, or found a
/// different one, or turned away from the one it had — each of which is a real
/// end to a band, and bridging them would draw a sidewalk across the mouth of
/// the side road it stops at.
fn runs(hits: &[Option<Hit>], census: &mut Census) -> Vec<(usize, usize)> {
    let mut out: Vec<(usize, usize)> = Vec::new();
    let mut start: Option<usize> = None;
    for i in 0..=hits.len() {
        let raw = hits.get(i).copied().flatten();
        let here = raw.filter(|h| h.along);
        let open = start.and_then(|s| hits[s]);
        let breaks = match (open, here) {
            (Some(a), Some(b)) => a.host != b.host || a.side != b.side,
            _ => true,
        };
        if !breaks {
            continue;
        }
        if let Some(s) = start.take() {
            out.push((s, i - 1));
            let was = hits[s].expect("a run starts on a hit");
            match raw {
                None => census.broke_lost += 1,
                Some(h) if h.host != was.host && !h.along => census.broke_crossed += 1,
                Some(h) if h.host != was.host => census.broke_host += 1,
                Some(h) if !h.along => census.broke_turned += 1,
                Some(_) => census.broke_side += 1,
            }
        }
        if here.is_some() {
            start = Some(i);
        }
    }
    out
}

/// Stations every `step_m` along a line, each with the direction the line is
/// running through it — a chord over [`TANGENT_HALF_M`] either side, falling
/// back to the local one where the chord is too short to have a direction (a
/// hairpin, or the ends).
///
/// The last vertex is always a station, so a way shorter than one step still
/// has two and is measured rather than skipped.
fn stations(line: &[Coord], cos_lat: f64, step_m: f64) -> Vec<Station> {
    let m_lon = DEG_M * cos_lat;
    let mut pts: Vec<Coord> = Vec::new();
    let mut carry = 0.0;
    for e in line.windows(2) {
        let (dx, dy) = ((e[1].x - e[0].x) * m_lon, (e[1].y - e[0].y) * DEG_M);
        let len = dx.hypot(dy);
        if !(len > 0.0) {
            continue;
        }
        let mut t = carry;
        while t < len {
            let u = t / len;
            pts.push(Coord {
                x: e[0].x + (e[1].x - e[0].x) * u,
                y: e[0].y + (e[1].y - e[0].y) * u,
            });
            t += step_m;
        }
        carry = t - len;
    }
    if pts.is_empty() {
        return Vec::new();
    }
    pts.push(line[line.len() - 1]);
    let w = (TANGENT_HALF_M / step_m).round().max(1.0) as usize;
    let dir_between = |a: Coord, b: Coord| {
        let (dx, dy) = ((b.x - a.x) * m_lon, (b.y - a.y) * DEG_M);
        let len = dx.hypot(dy);
        (len > 1e-6).then(|| (dx / len, dy / len))
    };
    (0..pts.len())
        .map(|i| {
            let (j, k) = (i.saturating_sub(w), (i + w).min(pts.len() - 1));
            let dir = dir_between(pts[j], pts[k])
                .or_else(|| dir_between(pts[i.saturating_sub(1)], pts[(i + 1).min(pts.len() - 1)]))
                .unwrap_or((1.0, 0.0));
            (pts[i], dir)
        })
        .collect()
}

/// Plan length of a line in metres.
fn length_m(line: &[Coord], cos_lat: f64) -> f64 {
    line.windows(2).map(|e| crate::scene::metric_len(e[0], e[1], cos_lat)).sum()
}

/// Every street centerline edge, indexed by plan position, with the reach each
/// one hosts out to.
struct Hosts<'a> {
    corridors: &'a [Corridor],
    /// `(corridor index, node index)` — the edge from `nodes[i]` to
    /// `nodes[i + 1]`.
    edges: Vec<(u32, u32)>,
    /// Drawn half-width of each corridor, metres — the kerb the reach is
    /// measured from.
    half_m: Vec<f64>,
    grid: GridIndex,
    /// The widest `half_m + WALK_ATTACH_M` any host has, so one query box
    /// suffices whatever it finds.
    reach_m: f64,
}

impl<'a> Hosts<'a> {
    fn build(corridors: &'a [Corridor]) -> Hosts<'a> {
        let mut out = Hosts {
            corridors,
            edges: Vec::new(),
            half_m: vec![0.0; corridors.len()],
            grid: GridIndex::with_cell_m(CELL_M),
            reach_m: 0.0,
        };
        for (ci, c) in corridors.iter().enumerate() {
            if c.kind.prior().surface != Surface::Asphalt {
                continue;
            }
            let Some(width) = c.width_m else { continue };
            out.half_m[ci] = width * 0.5;
            out.reach_m = out.reach_m.max(width * 0.5 + priors::WALK_ATTACH_M);
            for i in 0..c.nodes.len().saturating_sub(1) {
                let (a, b) = (c.nodes[i], c.nodes[i + 1]);
                out.grid.insert(
                    (a.x.min(b.x), a.y.min(b.y), a.x.max(b.x), a.y.max(b.y)),
                    out.edges.len() as u32,
                );
                out.edges.push((ci as u32, i as u32));
            }
        }
        out
    }

    /// The street this station belongs to, if any: the nearest by *clear*
    /// distance from the kerb.
    ///
    /// Clear distance, not centre distance, because the question is how far
    /// the way is from the street's edge. Between two carriageways — a divided
    /// road, or a service road beside a main one — a centre-distance rule
    /// would hand the sidewalk to whichever centerline happened to be closer
    /// even when the wide road's kerb is right beside it.
    fn nearest(&self, s: &Station, cos_lat: f64, scratch: &mut Vec<u32>) -> Option<Hit> {
        if self.edges.is_empty() {
            return None;
        }
        let (p, dir) = *s;
        let m_lon = DEG_M * cos_lat;
        let (rx, ry) = (self.reach_m / m_lon, self.reach_m / DEG_M);
        self.grid.query((p.x - rx, p.y - ry, p.x + rx, p.y + ry), scratch);
        let mut best: Option<(f64, Hit)> = None;
        for &e in scratch.iter() {
            let (ci, ni) = self.edges[e as usize];
            let c = &self.corridors[ci as usize];
            let (a, b) = (c.nodes[ni as usize], c.nodes[ni as usize + 1]);
            let (ex, ey) = ((b.x - a.x) * m_lon, (b.y - a.y) * DEG_M);
            let len = ex.hypot(ey);
            if !(len > 0.0) {
                continue;
            }
            let (qx, qy) = ((p.x - a.x) * m_lon, (p.y - a.y) * DEG_M);
            let u = ((qx * ex + qy * ey) / (len * len)).clamp(0.0, 1.0);
            let (fx, fy) = (qx - ex * u, qy - ey * u);
            let offset = fx.hypot(fy);
            let clear = offset - self.half_m[ci as usize];
            if clear > priors::WALK_ATTACH_M || best.is_some_and(|(c, _)| clear >= c) {
                continue;
            }
            // Left of the direction of travel is side 0, as everywhere else.
            let lateral = -qx * (ey / len) + qy * (ex / len);
            let along = ((dir.0 * ex + dir.1 * ey) / len).abs() > priors::WALK_ALONG;
            best = Some((
                clear,
                Hit {
                    host: c.id,
                    side: u8::from(lateral <= 0.0),
                    arc: c.arc[ni as usize] + u * len,
                    offset_m: offset,
                    along,
                },
            ));
        }
        best.map(|(_, h)| h)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::priors::RoadClass;
    use crate::scene::cumulative_arc;

    const LAT: f64 = 46.44;

    fn east(m: f64) -> f64 {
        m / (DEG_M * LAT.to_radians().cos())
    }

    fn north(m: f64) -> f64 {
        m / DEG_M
    }

    /// A straight east-running residential street, `len_m` long, through
    /// (6.9, 46.44).
    fn street(len_m: f64) -> Corridor {
        let nodes =
            vec![Coord { x: 6.9, y: LAT }, Coord { x: 6.9 + east(len_m), y: LAT }];
        let cos_lat = crate::scene::run_cos_lat(&nodes);
        Corridor {
            id: 0,
            arc: cumulative_arc(&nodes),
            nodes,
            cos_lat,
            kind: Kind::Road(RoadClass::Residential),
            class_key: "residential".into(),
            link: false,
            width_m: Some(5.5),
            spans: Vec::new(),
            segments: Vec::new(),
            connectors: Vec::new(),
        }
    }

    /// A line parallel to that street, `north_m` to its north, running from
    /// `from_m` to `to_m` along it.
    fn beside(north_m: f64, from_m: f64, to_m: f64) -> Vec<Coord> {
        vec![
            Coord { x: 6.9 + east(from_m), y: LAT + north(north_m) },
            Coord { x: 6.9 + east(to_m), y: LAT + north(north_m) },
        ]
    }

    fn clone_of(w: &WalkLine) -> WalkLine {
        WalkLine {
            source: w.source,
            line: w.line.clone(),
            kind: w.kind,
            tagged: w.tagged,
            crosswalk: w.crosswalk,
            indoor: w.indoor.clone(),
            connectors: w.connectors.clone(),
            spans: w.spans.clone(),
        }
    }

    fn walk(line: Vec<Coord>, tagged: bool) -> WalkLine {
        WalkLine {
            source: 1,
            line,
            kind: Kind::Road(RoadClass::Footway),
            tagged,
            crosswalk: false,
            connectors: Vec::new(),
            spans: Vec::new(),
            indoor: Vec::new(),
        }
    }

    /// A straight east–west line of `n` metres from the origin, so an arc
    /// fraction and a metre offset are the same number over `DEG_M`.
    fn straight(n: f64) -> Vec<Coord> {
        vec![Coord { x: 0.0, y: 0.0 }, Coord { x: n / DEG_M, y: 0.0 }]
    }

    fn rule(value: &str, start: f64, end: f64) -> SubclassRun {
        SubclassRun { start, end, value: value.to_string() }
    }

    #[test]
    fn a_way_with_no_subclass_rules_is_not_cut() {
        let out = split_by_subclass(walk(straight(100.0), true), &[]);
        assert_eq!(out.len(), 1);
        assert!(out[0].tagged, "the scalar subclass still speaks for the way");
    }

    /// The uniform segment — the overwhelming majority — must come through
    /// byte-identical, or every metric moves for a reason that is not the fix.
    #[test]
    fn a_full_length_run_leaves_the_way_whole() {
        let out = split_by_subclass(walk(straight(100.0), true), &[rule("sidewalk", 0.0, 1.0)]);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].line.len(), 2);
        assert!(out[0].tagged);
        assert!(!out[0].crosswalk);
    }

    /// The case `docs/SOURCES.md` §2 is about: one segment, two identities,
    /// and today the scalar `subclass` is null so it reads as neither.
    #[test]
    fn a_sidewalk_that_becomes_a_crossing_is_cut_in_two() {
        let base = walk(straight(100.0), false);
        let out = split_by_subclass(
            base,
            &[rule("sidewalk", 0.0, 0.6), rule("crosswalk", 0.6, 1.0)],
        );
        assert_eq!(out.len(), 2);
        assert!(out[0].tagged && !out[0].crosswalk);
        assert!(out[1].crosswalk && !out[1].tagged);
        // The cut lands where the source put it, in metres along the way.
        let cos_lat = crate::scene::run_cos_lat(&out[0].line);
        assert!((length_m(&out[0].line, cos_lat) - 60.0).abs() < 0.01);
        assert!((length_m(&out[1].line, cos_lat) - 40.0).abs() < 0.01);
        // …and the pieces meet, rather than leaving a gap at the boundary.
        assert_eq!(out[0].line.last(), out[1].line.first());
    }

    /// A rule reaching only part way leaves the rest genuinely unlabelled —
    /// a plain footway, which is what it is. It must not inherit the tag.
    #[test]
    fn the_stretch_no_rule_reaches_is_untagged() {
        let out =
            split_by_subclass(walk(straight(100.0), false), &[rule("sidewalk", 0.0, 0.4)]);
        assert_eq!(out.len(), 2);
        assert!(out[0].tagged);
        assert!(!out[1].tagged && !out[1].crosswalk);
    }

    /// The crossing's connectors are the evidence that the pavement either
    /// side of it is street furniture (`WALK_COVER_JOINED`). A cut must not
    /// throw that away: a connector on the boundary belongs to both pieces.
    #[test]
    fn a_cut_remaps_connectors_and_shares_the_boundary_node() {
        let mut base = walk(straight(100.0), false);
        base.connectors = vec![
            super::super::columns::Connector { id: 7, at: 0.0 },
            super::super::columns::Connector { id: 8, at: 0.5 },
            super::super::columns::Connector { id: 9, at: 1.0 },
        ];
        let out = split_by_subclass(
            base,
            &[rule("sidewalk", 0.0, 0.5), rule("crosswalk", 0.5, 1.0)],
        );
        assert_eq!(out.len(), 2);
        let ids = |w: &WalkLine| w.connectors.iter().map(|c| c.id).collect::<Vec<_>>();
        assert_eq!(ids(&out[0]), vec![7, 8]);
        assert_eq!(ids(&out[1]), vec![8, 9]);
        // Fractions are renormalised onto the piece they now sit on.
        assert!((out[0].connectors[1].at - 1.0).abs() < 1e-9);
        assert!((out[1].connectors[0].at - 0.0).abs() < 1e-9);
    }

    /// A structure span is a fraction of the way it was read from; after a cut
    /// it must be a fraction of the piece, or a footbridge suppresses the band
    /// on the wrong stretch (`WalkLine::spans`).
    #[test]
    fn a_cut_renormalises_structure_spans_onto_the_piece() {
        let mut base = walk(straight(100.0), false);
        base.spans = vec![(0.5, 0.75)];
        let out = split_by_subclass(
            base,
            &[rule("sidewalk", 0.0, 0.5), rule("crosswalk", 0.5, 1.0)],
        );
        assert_eq!(out.len(), 2);
        assert!(out[0].spans.is_empty(), "the span is wholly in the second piece");
        assert_eq!(out[1].spans.len(), 1);
        assert!((out[1].spans[0].0 - 0.0).abs() < 1e-9);
        assert!((out[1].spans[0].1 - 0.5).abs() < 1e-9);
    }

    /// The kerb-to-crossing stub: a sidewalk run too short to be a band joins
    /// the run beside it rather than shattering the walkway at the joint.
    #[test]
    fn a_sidewalk_sliver_is_absorbed_into_its_neighbour() {
        let out = split_by_subclass(
            walk(straight(100.0), false),
            // 0.5 m of sidewalk against 99.5 m of crossing-free way.
            &[rule("sidewalk", 0.0, 0.005)],
        );
        assert_eq!(out.len(), 1, "a half-metre sidewalk is a joint, not a stretch");
        assert!(!out[0].tagged);
    }

    /// …but a crossing is paint, not a band, so the width floor does not apply
    /// to it. Losing a short crossing would lose the crossing.
    #[test]
    fn a_short_crossing_is_never_absorbed() {
        let out = split_by_subclass(
            walk(straight(100.0), false),
            &[rule("sidewalk", 0.0, 0.99), rule("crosswalk", 0.99, 1.0)],
        );
        assert_eq!(out.len(), 2);
        assert!(out[1].crosswalk, "a 1 m crossing is still a crossing");
    }

    /// A way through a terminal gets no band *and* no stroke: the line would
    /// be drawn through the building's floor, which is the spectacle I6 exists
    /// to forbid rather than the detail it protects.
    #[test]
    fn a_wholly_indoor_way_is_silent() {
        let mut w = walk(straight(100.0), false);
        w.indoor = vec![(0.0, 1.0)];
        assert!(w.is_wholly_indoor());
        // Reported in pieces, as Overture often maps it.
        let mut split = walk(straight(100.0), false);
        split.indoor = vec![(0.0, 0.4), (0.4, 0.75), (0.75, 1.0)];
        assert!(split.is_wholly_indoor());
    }

    /// A way only partly indoors keeps its voice: the stretch outside is real,
    /// it gets a band, and the band deletes the stroke by the ordinary rule.
    #[test]
    fn a_partly_indoor_way_is_not_silenced() {
        let mut w = walk(straight(100.0), false);
        w.indoor = vec![(0.0, 0.5)];
        assert!(!w.is_wholly_indoor());
        // Overlaps must not be double-counted into full coverage.
        let mut overlapped = walk(straight(100.0), false);
        overlapped.indoor = vec![(0.0, 0.5), (0.1, 0.6), (0.2, 0.7)];
        assert!(!overlapped.is_wholly_indoor(), "0.7 of a way is not all of it");
    }

    /// A footbridge is fully *spanned* and must not be mistaken for silence —
    /// which is exactly why `indoor` is kept apart from `spans`.
    #[test]
    fn a_fully_spanned_footbridge_is_not_indoors() {
        let mut w = walk(straight(100.0), false);
        w.spans = vec![(0.0, 1.0)];
        assert!(!w.is_wholly_indoor());
    }

    /// A cut piece inherits the indoor runs that fall on it, so silence
    /// survives `split_by_subclass` and does not leak onto its neighbour.
    #[test]
    fn a_cut_keeps_indoor_runs_on_the_piece_they_fall_on() {
        let mut base = walk(straight(100.0), false);
        base.indoor = vec![(0.5, 1.0)];
        base.spans = vec![(0.5, 1.0)];
        let out = split_by_subclass(
            base,
            &[rule("sidewalk", 0.0, 0.5), rule("crosswalk", 0.5, 1.0)],
        );
        assert_eq!(out.len(), 2);
        assert!(out[0].indoor.is_empty());
        assert!(out[1].is_wholly_indoor(), "the indoor half is wholly indoor once cut");
    }

    /// Overlapping rules are last-wins, matching `corridors::resolve_spans`.
    #[test]
    fn overlapping_rules_resolve_in_source_order() {
        let out = split_by_subclass(
            walk(straight(100.0), false),
            &[rule("sidewalk", 0.0, 1.0), rule("crosswalk", 0.4, 0.6)],
        );
        assert_eq!(out.len(), 3);
        assert!(out[0].tagged);
        assert!(out[1].crosswalk);
        assert!(out[2].tagged);
    }

    #[test]
    fn a_footway_running_beside_a_street_attaches_to_its_side() {
        let w = attach(&[street(100.0)], vec![walk(beside(5.0, 10.0, 90.0), false)]);
        assert_eq!(w.len(), 1, "{:?}", w.all());
        let a = w.all()[0];
        assert_eq!(a.host, 0);
        assert_eq!(a.side, 0, "north of an east-running street is its left");
        assert!((a.offset_m - 5.0).abs() < 0.1, "offset {}", a.offset_m);
        assert!((a.len_m() - 80.0).abs() < 2.0, "arc range {:?}", (a.arc0, a.arc1));
        assert_eq!(a.evidence, Evidence::Alongside);
    }

    #[test]
    fn the_far_side_is_the_other_side() {
        let w = attach(&[street(100.0)], vec![walk(beside(-5.0, 10.0, 90.0), false)]);
        assert_eq!(w.all()[0].side, 1);
    }

    #[test]
    fn a_path_out_of_reach_attaches_to_nothing() {
        // 14 m out is 11.25 m clear of a 5.5 m-wide street's kerb.
        let w = attach(&[street(100.0)], vec![walk(beside(14.0, 10.0, 90.0), false)]);
        assert!(w.is_empty(), "{:?}", w.all());
        assert_eq!(w.census().covered_m, 0.0);
    }

    #[test]
    fn the_reach_is_measured_from_the_kerb_not_the_centerline() {
        // 12 m out: outside the reach of a 5.5 m street (9.25 m clear), inside
        // that of a 20 m one (2 m clear) whose kerb is right beside it.
        let narrow = attach(&[street(100.0)], vec![walk(beside(12.0, 10.0, 90.0), false)]);
        assert!(narrow.is_empty(), "{:?}", narrow.all());
        let mut wide = street(100.0);
        wide.width_m = Some(20.0);
        assert_eq!(attach(&[wide], vec![walk(beside(12.0, 10.0, 90.0), false)]).len(), 1);
    }

    #[test]
    fn a_way_crossing_a_street_is_not_a_sidewalk_on_it() {
        let across = vec![
            Coord { x: 6.9 + east(50.0), y: LAT - north(6.0) },
            Coord { x: 6.9 + east(50.0), y: LAT + north(6.0) },
        ];
        let w = attach(&[street(100.0)], vec![walk(across, false)]);
        assert!(w.is_empty(), "{:?}", w.all());
        assert!(w.census().covered_m > 0.0, "it was in reach — it just ran across");
        assert_eq!(w.census().alongside_m, 0.0);
    }

    #[test]
    fn a_hillside_path_that_only_grazes_a_street_is_not_attached() {
        // 200 m of path, 20 m of which runs beside the street: 10 % coverage,
        // well under WALK_COVER, and untagged.
        let mut line = beside(5.0, 0.0, 20.0);
        line.push(Coord { x: 6.9 + east(30.0), y: LAT + north(180.0) });
        let w = attach(&[street(100.0)], vec![walk(line, false)]);
        assert!(w.is_empty(), "{:?}", w.all());
    }

    #[test]
    fn the_tag_admits_the_stretch_that_does_run_with_the_street() {
        let mut line = beside(5.0, 0.0, 20.0);
        line.push(Coord { x: 6.9 + east(30.0), y: LAT + north(180.0) });
        let w = attach(&[street(100.0)], vec![walk(line, true)]);
        assert_eq!(w.len(), 1, "the tagged way keeps its attached stretch");
        assert_eq!(w.all()[0].evidence, Evidence::Tag);
        assert!(w.all()[0].len_m() < 25.0, "and only that stretch");
    }

    #[test]
    fn a_tagged_sidewalk_nowhere_near_a_street_is_still_refused() {
        let w = attach(&[street(100.0)], vec![walk(beside(40.0, 0.0, 90.0), true)]);
        assert!(w.is_empty(), "the tag is evidence, not authority");
        assert_eq!(w.census().tagged_unhosted, 1);
    }

    #[test]
    fn a_way_that_changes_sides_attaches_twice() {
        // Down the north side, across the street, back along the south side.
        let line = vec![
            Coord { x: 6.9 + east(5.0), y: LAT + north(5.0) },
            Coord { x: 6.9 + east(45.0), y: LAT + north(5.0) },
            Coord { x: 6.9 + east(50.0), y: LAT - north(5.0) },
            Coord { x: 6.9 + east(95.0), y: LAT - north(5.0) },
        ];
        let w = attach(&[street(100.0)], vec![walk(line, true)]);
        assert_eq!(w.len(), 2, "{:?}", w.all());
        assert_eq!(w.all()[0].side, 0);
        assert_eq!(w.all()[1].side, 1);
    }

    #[test]
    fn a_nick_at_a_corner_is_too_short_to_be_a_band() {
        let w = attach(&[street(100.0)], vec![walk(beside(5.0, 50.0, 56.0), true)]);
        assert!(w.is_empty(), "{:?}", w.all());
        assert_eq!(w.census().dropped_short, 1);
    }

    #[test]
    fn a_crosswalk_attaches_to_nothing_but_vouches_for_what_it_joins() {
        let joint = |id| super::super::columns::Connector { id, at: 1.0 };
        let mut sidewalk = walk(beside(5.0, 0.0, 30.0), false);
        sidewalk.connectors = vec![joint(77)];
        // Half of a 60 m way runs with the street: under WALK_COVER on its own.
        sidewalk.line.push(Coord { x: 6.9 + east(40.0), y: LAT + north(35.0) });
        let mut zebra = walk(
            vec![
                Coord { x: 6.9 + east(30.0), y: LAT + north(5.0) },
                Coord { x: 6.9 + east(30.0), y: LAT - north(5.0) },
            ],
            false,
        );
        zebra.source = 2;
        zebra.crosswalk = true;
        zebra.connectors = vec![joint(77)];
        let alone = attach(&[street(100.0)], vec![clone_of(&sidewalk)]);
        assert!(alone.is_empty(), "half its length is not enough on its own");
        let joined = attach(&[street(100.0)], vec![sidewalk, zebra]);
        assert_eq!(joined.len(), 1, "the crossing is the other half of the evidence");
        assert!(joined.of_walk(2).next().is_none(), "and the crossing itself is paint");
    }

    #[test]
    fn a_railway_hosts_nothing() {
        let mut rail = street(100.0);
        rail.kind = Kind::Road(RoadClass::Residential);
        rail.kind = Kind::Rail(crate::priors::RailClass::StandardGauge);
        let w = attach(&[rail], vec![walk(beside(5.0, 10.0, 90.0), true)]);
        assert!(w.is_empty(), "a platform is not a sidewalk");
    }

    #[test]
    fn the_nearest_kerb_wins_between_two_streets() {
        // A service road 9 m north of a main street, and a footway between
        // them: 3.5 m clear of the service road's kerb, 4.75 m of the main
        // one's.
        let main = street(100.0);
        let mut service = street(100.0);
        service.id = 1;
        service.nodes.iter_mut().for_each(|n| n.y += north(9.0));
        service.width_m = Some(3.0);
        let w = attach(&[main, service], vec![walk(beside(4.0, 10.0, 90.0), true)]);
        assert_eq!(w.len(), 1, "{:?}", w.all());
        assert_eq!(w.all()[0].host, 0, "the main street's kerb is nearer");
    }

    #[test]
    fn the_census_counts_what_it_saw() {
        let w = attach(&[street(100.0)], vec![walk(beside(5.0, 10.0, 90.0), true)]);
        let c = w.census();
        assert_eq!(c.lines, 1);
        assert_eq!(c.tagged_lines, 1);
        assert_eq!(c.attached_lines, 1);
        assert_eq!(c.both, 1);
        assert!((c.line_m - 80.0).abs() < 2.0, "{c:?}");
        assert!((c.covered_m - 80.0).abs() < 2.0, "{c:?}");
    }
}
