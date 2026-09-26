//! The reference: the surface a way is solved against.
//!
//! Every height in the model is solved against *something*, and until this
//! step that something was the raw DEM. A surface DEM is not a ground: it
//! images a culvert as a slot the road dives through, canopy shadow and
//! upsampling ripple as crests on the carriageway, and a viaduct as ground
//! the road is already lying on. Solved against it a street dives into a
//! ditch it was engineered across, and a bridge over a gorge has no gorge to
//! be over.
//!
//! So the reference is the terrain with three things done to it, in order:
//!
//! 1. **Blind runs bridged.** Where the ground under the axis stands more
//!    than [`crate::grade::DECK_STANDOFF_M`] above the ground [`FLANK_M`] to
//!    both sides, the DEM under the axis is a structure's own top. It is not
//!    evidence, and the reference is carried straight across it from the two
//!    rims. This is the one thing no reader downstream can recover: on a
//!    `shelf` specimen every instrument in the run reads zero, because a road
//!    solved onto its own deck has no defect to show.
//!
//!    **A mapped span is bridged for the same reason** ([`spanned_mask`]).
//!    The blind mask asks the DEM whether it is standing on the way's deck;
//!    the span table says the way has left the ground here, and a terrain
//!    model that has had its bridges taken out answers with the slot
//!    underneath — the railway in its cutting, the stream in its bed. Read as
//!    ground it is the far shoulder of the approach embankment's crest, and
//!    the opening then shaves the embankment away as a false bump.
//! 2. **Notches filled** ([`close_notches`]). A valley narrower than
//!    [`NOTCH_SPAN_M`] under a road is ground continuity that was
//!    engineered — a culvert, an embankment, a small retaining structure —
//!    and the road existed first.
//! 3. **Bumps shaved** ([`open_bumps`]). The opening dual, tighter, because
//!    a false crest is shallow noise while a false notch can be a gorge.
//!
//! Both morphological passes are **bounded**: a run whose fill or shave
//! would exceed its cap is a genuine feature, not an artifact, so that run
//! keeps the raw terrain — and *the refusal is the signal*. A notch too deep
//! to fill under a line mapped level across it is a slot the way is carried
//! over; a crest too high to shave under a class that cannot climb it is a
//! mass the way goes through. [`Axis::refused_notch`] and
//! [`Axis::refused_crest`] are those two lists, and they are the terrain's
//! own bridge and tunnel priors
//! (`data/plans/spans-are-derived-2026-09-09.md` §4).
//!
//! **Nothing reads this yet.** The step computes the reference, reports what
//! it came to, and is consumed by no one: migration step 2 of the plan, whose
//! whole job is to put a number on how far the conditioned surface stands
//! from the raw one before anything depends on the answer.
//!
//! **A piece is not a corridor.** The reader cuts every way at every
//! annotation edge, so a 40 m bridge piece is conditioned with a 60 m window
//! and its two ends are the whole of it. `short` counts the pieces this is
//! true of, and that count is the argument for R1 — the reader keeping whole
//! ways — rather than a defect of the conditioning.

use std::collections::HashMap;

use crate::grade::{self, DECK_STANDOFF_M, NODE_M};
use crate::profile::densify_at;
use crate::step::{Residual, Summary};
use crate::terrain::height_at;
use crate::width;
use crate::world::{connector, Kind, Reference, Roads, Span, Terrain, Way};

/// Widest DEM notch, in metres of arc, that a road is assumed to span on
/// engineered fill rather than dive through. Gullies, stream cuts and shadow
/// artifacts under real roads image as narrow V's in a surface DEM; the road
/// existed first, and ground continuity across it was engineered. Wider
/// valleys are genuine descents and keep the terrain.
pub const NOTCH_SPAN_M: f64 = 60.0;

/// Deepest per-notch fill the closing may build, in metres. Deeper than this
/// under a line mapped level across it is a data contradiction — a gorge owed
/// a structure, or a DEM blunder — so the terrain is trusted, the raw profile
/// is kept, and the run is *reported* as a bridge prior.
pub const NOTCH_FILL_MAX_M: f64 = 15.0;

/// Widest convex bump, in metres of arc, shaved as noise rather than climbed.
/// The opening dual of [`NOTCH_SPAN_M`], sized under it: filling across an
/// engineered culvert is cheaper to assume than cutting through a crest that
/// is really there.
pub const BUMP_SPAN_M: f64 = 50.0;

/// Deepest per-bump shave the opening may take, in metres. Far tighter than
/// [`NOTCH_FILL_MAX_M`], because false crests are shallow while false notches
/// can be deep. A crest that would need a deeper cut is genuine relief, and
/// is reported as a tunnel prior for the classes that cannot climb it.
pub const BUMP_SHAVE_MAX_M: f64 = 4.0;

/// How far to the side of an axis the ground is asked whether the ground
/// *under* the axis is really ground, in metres. Wide enough to step off a
/// deck as a surface DEM rasterised it, narrow enough to stay inside the
/// gorge that deck spans.
pub const FLANK_M: f64 = 25.0;

/// How far past a mapped bridge's own edge the DEM's slot is allowed to
/// reach, in metres of arc.
///
/// A span boundary is where a mapper clicked, and the slot the deck spans is
/// where the ground actually falls away; the two are metres apart, and the
/// metres between them are the abutment, which a terrain model images as a
/// cliff. Read as ground they are the far shoulder of every crest the
/// approach embankment makes, so the opening shaves the embankment as a
/// false bump — measured at the Montreux rail overbridge, **3.00 m** of it
/// over the last 40 m of approach, the road's reference flattened to 398.20
/// where the DEM climbs to 401.20.
///
/// Three DEM cells, which is as far as a raster smears an edge.
pub const ABUTMENT_M: f64 = 10.0;

/// How steeply the terrain must fall into a mapped span, in metres per
/// metre, before the fall is read as the span's own wall rather than as
/// ground the way descends.
///
/// Half. No road in the model is built at 50 % — the steepest street
/// Switzerland has is under 30 %, and [`crate::bench::EARTHWORK_BATTER`]
/// battens at 40 % — so a drop that steep at a span's edge is the abutment
/// the DEM could not resolve. Grown on the fall alone the rule ate ten
/// metres of honest approach wherever a deck springs from the *inside* of a
/// bowl, where the ground beyond the abutment is lower still:
/// `a_span_that_already_clears_asks_for_nothing` is the check that said so.
pub const ABUTMENT_GRADE: f64 = 0.5;

/// Slack on a morphological comparison, in metres: the passes are a max and
/// a min of the same numbers, so "unchanged" has to mean unchanged to the
/// float rather than bit-for-bit.
const EPS_M: f64 = 1e-6;

/// Which of `ways` get a reference, and so a profile: the carriageway and
/// rail ways of a class that solves at all, as indices into `ways`.
///
/// **This is the only place the question is asked.** It used to be asked
/// again by every step that held a profile and wanted the way behind it —
/// the crossing, the partition and the sheet each re-ran the filter and two
/// of them inverted it into a `HashMap` — so a predicate in one module was
/// an invariant in four, with a runtime `assert_eq!` standing in for the
/// type that should have said it. Now the answer is recorded where it is
/// used: [`Axis::way`] and [`crate::world::Profile::way`] carry it, and a
/// caller holding either knows the way it belongs to without asking.
pub fn solving_of(ways: &[Way]) -> Vec<usize> {
    (0..ways.len())
        .filter(|&i| {
            let w = &ways[i];
            width::family(&w.class).solves() && grade::of(&w.class).solves()
        })
        .collect()
}

/// Builds the reference surface along every solving axis of the world, and
/// the span tables the terrain's own priors imply.
///
/// The tables are **returned, not written**. A refused notch is a slot the
/// closing will not fill under a line mapped level across it, and until
/// something acts on it nothing does: a street is not grade-limited, so it
/// follows the reference into the gorge exactly, departs from it nowhere, and
/// the derivation that reads departures finds no bridge to derive. So the
/// refusal becomes a **prior** in the way's span table — an annotation the
/// source did not make — and the profile chords across it like any other. It
/// is a prior and not a command: §4.5 still decides what is built, and a
/// chord that never leaves the ground still degrades.
///
/// Installing them on [`Roads::ways`] is [`crate::pipeline`]'s, one arm of a
/// `match` away from the order that makes it safe. This step used to take
/// `&mut Roads` and do it here, which made the way's span table a thing three
/// steps wrote to and no signature admitted.
pub fn run(terrain: &Terrain, roads: &Roads) -> (Reference, Vec<Vec<Span>>, Summary) {
    let solving = solving_of(&roads.ways);
    let Reference { axes } = of(&roads.ways, &solving, terrain);
    let (tables, promoted) = promote(&roads.ways, &axes);
    // The tables changed, so the axes are rebuilt over them: a boundary is a
    // station ([`Axis::of`]), and a chord has to start on one.
    let Reference { mut axes } = over(&roads.ways, &tables, &solving, terrain);
    // One value per junction, before any profile solves.
    let was = disagreement(&axes);
    agree(&mut axes);
    let summary = measure(&axes, was, promoted);
    (Reference { axes }, tables, summary)
}

/// What the conditioning came to, as the run's one line: a function of the
/// finished axes, `was` (the junction disagreement before [`agree`]) and how
/// many priors were promoted.
///
/// Apart from [`run`] for the same reason the profile's is: seventeen
/// counters against six lines of work, and a step's entry point should read
/// as what it makes.
fn measure(axes: &[Axis], was: f64, promoted: usize) -> Summary {
    let mut stations = 0usize;
    let (mut blind, mut spanned) = (0usize, 0usize);
    let mut m = Moved::default();
    let mut residual = Residual::new();
    let (mut blind_m, mut notch_m, mut crest_m) = (0.0f64, 0.0f64, 0.0f64);
    let (mut notches, mut crests, mut short) = (0usize, 0usize, 0usize);
    for a in axes {
        stations += a.s.len();
        short += (a.len() < NOTCH_SPAN_M) as usize;
        blind += a.blind.iter().filter(|b| **b).count();
        spanned += a.spanned.iter().filter(|b| **b).count();
        m.bridged += a.moved.bridged;
        m.filled += a.moved.filled;
        m.shaved += a.moved.shaved;
        m.bridge_m = m.bridge_m.max(a.moved.bridge_m);
        m.fill_m = m.fill_m.max(a.moved.fill_m);
        m.shave_m = m.shave_m.max(a.moved.shave_m);
        // The **same population** the steps after it report over: stations
        // on the ground. A station inside a mapped span is one where the
        // conditioning deliberately carries the pass across the slot
        // underneath ([`spanned_mask`]), so its residual is the height of the
        // structure and not a departure at all — and the steps after this one
        // exclude it too (`Solved::Grade`, `crossing::residual_of`). Counting
        // it here reported 17.09 m against the profile's 8.45 and read as a
        // conditioning that moves the surface twice as far as the road
        // follows it. There was no such thing: it was this loop, comparing
        // one population against another.
        for k in 0..a.s.len() {
            if !a.spanned[k] {
                residual.push(a.h[k], a.ground[k]);
            }
        }
        blind_m += a.blind_m();
        notches += a.refused_notch.len();
        crests += a.refused_crest.len();
        notch_m += a.refused_notch.iter().map(|(a0, a1)| a1 - a0).sum::<f64>();
        crest_m += a.refused_crest.iter().map(|(a0, a1)| a1 - a0).sum::<f64>();
    }
    Summary::new()
        .with("axes", axes.len())
        .with("stations", stations)
        .with_share("short", short, axes.len())
        .with_share("blind", blind, stations)
        .with("blind_m", format!("{blind_m:.0}"))
        .with_share("spanned", spanned, stations)
        .with_share("bridged", m.bridged, stations)
        .with("bridge", format!("{:.2}", m.bridge_m))
        .with_share("filled", m.filled, stations)
        .with("fill", format!("{:.2}", m.fill_m))
        .with_share("shaved", m.shaved, stations)
        .with("shave", format!("{:.2}", m.shave_m))
        .with("notch", format!("{notches}/{notch_m:.0}m"))
        .with("crest", format!("{crests}/{crest_m:.0}m"))
        .with("junction", format!("{was:.3}->{:.3}", disagreement(axes)))
        .with("promoted", promoted)
        .with_residual(residual)
}

/// The terrain's refused notches written into the ways as bridge priors, and
/// how many were taken.
///
/// A notch is promoted only where it lies **strictly inside one ground span**
/// of the way: a refusal overlapping a mapped structure is that structure's
/// business, and one reaching a way's end has no rim on that side to land on.
/// The span table stays a partition — the ground span is split in three and
/// the prior takes the middle.
fn promote(ways: &[Way], axes: &[Axis]) -> (Vec<Vec<Span>>, usize) {
    let mut tables: Vec<Vec<Span>> = ways.iter().map(|w| w.spans.clone()).collect();
    let mut taken = 0usize;
    for a in axes {
        let w = a.way;
        let g = grade::of(&ways[w].class);
        // **The mirror, and its gate.** A crest the opening refuses is a mass
        // the way would have to cut through — but only a class whose ladder
        // cannot climb it is owed a bore. A street may climb anything: the
        // DEM under a street *is* the street, and a lane mapped at 26 % up
        // the old town really climbs 26 % (S9). A motorway held to 6 % inside
        // an 8 m box cannot, and where the box cannot reach the crest's own
        // rise, what is there is a mass the road goes through.
        //
        // **The gate is on inventing a bore, not on extending one.** Where
        // the source has already said *tunnel*, the mapper has settled that
        // the road goes through rather than over, and the terrain is only
        // saying how far. So a refused crest that overlaps a mapped structure
        // is painted whatever the class; one in open ground needs the ladder
        // test. Gated both ways, a street's tunnel mapped forty metres under a
        // hundred-and-twenty-metre ridge kept the forty and climbed the rest.
        // **A blind run is not promoted, and it was worth finding out why.**
        // Where the DEM under the axis is a structure's own top, what it has
        // imaged is a deck — but saying so in the span table buys nothing,
        // because the consequence rule reads `h − ground` and on a causeway
        // the ground *is* the deck: the chord and the DEM agree exactly and
        // no departure exists to find. Tried on the loop box, seventeen
        // promotions produced eight spans that built no geometry at all
        // (`grounded` 20 → 28) and counted as bores, for half a percent of
        // wall. What the case actually needs is the **terrain** to lose the
        // causeway, which is the ground stage's and not a span table's.
        let sites: Vec<((f64, f64), Kind, bool)> = a
            .refused_notch
            .iter()
            .map(|&r| (r, Kind::Bridge(0), true))
            .chain(a.refused_crest.iter().map(|&(c0, c1)| {
                ((c0, c1), Kind::Tunnel(0), g.limited() && a.rise(c0, c1) > g.deviation_m)
            }))
            .collect();
        // Level 0 on a promoted site: the terrain says a structure is needed,
        // not that it is above anything. Promoted at level 1 instead, two
        // roads bridging one valley became peers at the same ordinal and
        // their crossing was filed as a data error, when the source had said
        // plainly which of them was over the other.
        let len = ways[w].len();
        for ((n0, n1), kind, may_invent) in sites {
            // One station of margin each side lands the edge on the rim
            // rather than on the last refused sample.
            let (lo, hi) = ((n0 - NODE_M).max(0.0), (n1 + NODE_M).min(len));
            if paint(&mut tables[w], lo, hi, kind, len, may_invent) {
                taken += 1;
            }
        }
    }
    (tables, taken)
}

/// Paints one terrain prior over a way's span table, and says whether it
/// changed anything.
///
/// A site that falls in open ground splits it in three and takes the middle.
/// A site that **overlaps a structure the source already mapped extends that
/// structure** to cover it, keeping the mapper's own kind and ordinal: the
/// annotation says *there is a bridge here* and the terrain says *the slot is
/// this wide*, and together they say how long the bridge is. Skipped instead
/// — the first rule, which took only sites strictly inside a ground span — a
/// gorge mapped with twenty metres of bridge over its middle kept the twenty
/// metres, and its two approaches dived thirty metres to the floor and
/// climbed out again.
///
/// The table stays a partition: the painted extent is one span, and ground
/// fills what is left either side of it.
fn paint(
    spans: &mut Vec<Span>,
    lo: f64,
    hi: f64,
    kind: Kind,
    len: f64,
    may_invent: bool,
) -> bool {
    let overlaps = |s: &Span| s.a1 > lo && s.a0 < hi;
    let mapped: Vec<Span> = spans.iter().filter(|s| s.kind != Kind::Ground).copied().collect();
    let hit: Vec<Span> = mapped.iter().copied().filter(overlaps).collect();
    // A site already inside a structure adds nothing.
    if hit.iter().any(|s| s.a0 <= lo + 1e-9 && s.a1 >= hi - 1e-9) {
        return false;
    }
    if hit.is_empty() && !may_invent {
        return false;
    }
    let (a0, a1, kind) = match hit.first() {
        Some(first) => (
            hit.iter().map(|s| s.a0).fold(lo, f64::min),
            hit.iter().map(|s| s.a1).fold(hi, f64::max),
            first.kind,
        ),
        None => (lo, hi, kind),
    };
    let mut kept: Vec<Span> = mapped.into_iter().filter(|s| !overlaps(s)).collect();
    kept.push(Span { a0, a1, kind });
    kept.sort_by(|x, y| x.a0.total_cmp(&y.a0));
    let mut out: Vec<Span> = Vec::with_capacity(kept.len() * 2 + 1);
    let mut at = 0.0f64;
    for s in kept {
        if s.a0 - at > 1e-9 {
            out.push(Span { a0: at, a1: s.a0, kind: Kind::Ground });
        }
        out.push(s);
        at = s.a1;
    }
    if len - at > 1e-9 {
        out.push(Span { a0: at, a1: len, kind: Kind::Ground });
    }
    *spans = out;
    true
}

/// How far apart two axes' references stand at a junction they share, at
/// worst — the instrument that found the defect [`agree`] closes.
pub fn disagreement(axes: &[Axis]) -> f64 {
    let mut at: HashMap<(i64, i64), (f64, f64)> = HashMap::new();
    for a in axes.iter().filter(|a| !a.is_empty()) {
        for k in [0, a.s.len() - 1] {
            let e = at.entry(connector(a.p[k])).or_insert((a.h[k], a.h[k]));
            e.0 = e.0.min(a.h[k]);
            e.1 = e.1.max(a.h[k]);
        }
    }
    at.values().map(|(lo, hi)| hi - lo).fold(0.0f64, f64::max)
}

/// **One reference at a junction.**
///
/// The conditioning is per axis, and two ways meeting at a point can fill a
/// notch there differently — one closes it, its neighbour *refuses* the same
/// notch as too deep and keeps the raw terrain, and the two then stand a
/// whole [`NOTCH_FILL_MAX_M`] apart at a point they share. Measured on the
/// loop box: **15.103 m**.
///
/// Nothing downstream survives that. The profile anchors a junction on
/// whichever way it reaches first, so the others are pinned that far off
/// their own target — a step the limiter cannot see, an abutment that does
/// not meet the ground way beside it (`structure abutment` 5.380), and a
/// clearance demand of 68 m where a bridge read sixty metres below a bore.
///
/// So the ends are agreed before any profile solves: every axis at one
/// connector takes the **mean** of what they each made of it — neither
/// reading is privileged, the disagreement being an artifact of where each
/// window happened to fall — and each axis is then corrected by a linear
/// taper that is the full difference at its end and nothing at
/// [`NOTCH_SPAN_M`]/2 in, which is the reach of the pass that caused it. On
/// an axis shorter than that reach the taper runs its whole length, so the
/// two ends stay exact and the two corrections cannot fight.
pub fn agree(axes: &mut [Axis]) {
    let mut at: HashMap<(i64, i64), (f64, usize)> = HashMap::new();
    for a in axes.iter().filter(|a| !a.is_empty()) {
        for k in [0, a.s.len() - 1] {
            let e = at.entry(connector(a.p[k])).or_insert((0.0, 0));
            e.0 += a.h[k];
            e.1 += 1;
        }
    }
    for a in axes.iter_mut().filter(|a| !a.is_empty()) {
        let (n, len) = (a.s.len(), a.len());
        let reach = (NOTCH_SPAN_M * 0.5).min(len);
        if !(reach > 0.0) {
            continue;
        }
        // Both deltas from the *uncorrected* heights, so neither end reads
        // the other's correction.
        let delta = |k: usize| -> f64 {
            at.get(&connector(a.p[k])).map_or(0.0, |&(sum, count)| sum / count as f64 - a.h[k])
        };
        let (d0, d1) = (delta(0), delta(n - 1));
        if d0 == 0.0 && d1 == 0.0 {
            continue;
        }
        for k in 0..n {
            let w0 = (1.0 - a.s[k] / reach).clamp(0.0, 1.0);
            let w1 = (1.0 - (len - a.s[k]) / reach).clamp(0.0, 1.0);
            a.h[k] += d0 * w0 + d1 * w1;
        }
    }
}

/// The reference along each way of `solving`, in that order.
///
/// `solving` indexes `ways` ([`solving_of`]), and each axis keeps the index
/// it was built for in [`Axis::way`] — so the correspondence
/// [`crate::profile::solve_on`] relies on is carried in the data rather than
/// re-derived by every caller.
pub fn of(ways: &[Way], solving: &[usize], terrain: &Terrain) -> Reference {
    let tables: Vec<Vec<Span>> = ways.iter().map(|w| w.spans.clone()).collect();
    over(ways, &tables, solving, terrain)
}

/// The same, reading `tables[w]` as way `w`'s span table instead of the
/// way's own.
///
/// A span boundary is a station ([`Axis::of`]), so the axes have to be built
/// again once the priors are promoted — and this is how that second pass
/// sees them **without anything having been written to the world**. The step
/// hands the tables back to [`crate::pipeline`] instead, which is the only
/// place that may install a layer.
fn over(ways: &[Way], tables: &[Vec<Span>], solving: &[usize], terrain: &Terrain) -> Reference {
    Reference { axes: solving.iter().map(|&w| Axis::of(w, &ways[w], &tables[w], terrain)).collect() }
}

/// The reference along one axis, at the stations the profile will solve it
/// at ([`NODE_M`] apart, the piece's own vertices kept).
#[derive(Debug, Clone, PartialEq)]
pub struct Axis {
    /// The way this axis is of: an index into [`Roads::ways`]. Every step
    /// that holds a profile needs it, and it is known here and nowhere
    /// cheaper.
    pub way: usize,
    /// Arc length at each station, from the piece's start.
    pub s: Vec<f64>,
    /// The plan position of each station.
    pub p: Vec<[f64; 2]>,
    /// The raw terrain there: what the bench still owes its earthwork
    /// against, and what a departure is still measured from.
    pub ground: Vec<f64>,
    /// The conditioned surface: blind runs bridged, notches filled, bumps
    /// shaved. What a profile is solved against.
    pub h: Vec<f64>,
    /// Where the ground under the axis is a structure's own top.
    pub blind: Vec<bool>,
    /// Where the ground under the axis is a mapped structure's slot or mass
    /// rather than the way's own ground ([`spanned_mask`]).
    pub spanned: Vec<bool>,
    /// Notches the closing refused, as `(arc, arc)`: the terrain's bridge
    /// priors.
    pub refused_notch: Vec<(f64, f64)>,
    /// Crests the opening refused: the terrain's tunnel priors, for the
    /// classes whose ladder cannot climb them.
    pub refused_crest: Vec<(f64, f64)>,
    /// What each of the three passes moved, separately.
    pub moved: Moved,
}

/// How far each pass moved the surface, and at how many stations.
///
/// Kept per pass rather than as one composite, because the composite cannot
/// be read: the bridging lowers a causeway onto its rims, the closing lifts a
/// notch and the opening shaves a crest, and a station that has been through
/// two of them reports a number belonging to neither. Measured against the
/// caps, a composite `shave` of 13.26 m looked like the opening exceeding its
/// own 4 m budget when it was the bridging doing its job.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Moved {
    pub bridged: usize,
    pub bridge_m: f64,
    pub filled: usize,
    pub fill_m: f64,
    pub shaved: usize,
    pub shave_m: f64,
}

impl Moved {
    /// The ledger of the three passes over one axis.
    fn of(raw: &[f64], bridged: &[f64], closed: &[f64], opened: &[f64]) -> Moved {
        let mut m = Moved::default();
        for k in 0..raw.len() {
            let (b, f, o) = (bridged[k] - raw[k], closed[k] - bridged[k], opened[k] - closed[k]);
            if b.abs() > EPS_M {
                m.bridged += 1;
                m.bridge_m = m.bridge_m.max(b.abs());
            }
            if f > EPS_M {
                m.filled += 1;
                m.fill_m = m.fill_m.max(f);
            }
            if o < -EPS_M {
                m.shaved += 1;
                m.shave_m = m.shave_m.max(-o);
            }
        }
        m
    }
}

impl Axis {
    /// The reference along `w` over `terrain`.
    ///
    /// Stationed at [`NODE_M`], the way's own vertices kept, **and a station
    /// at every span boundary**: a boundary is where the model changes hands,
    /// so a chord that starts there has to start on a station rather than
    /// within four metres of one.
    pub fn of(way: usize, w: &Way, spans: &[Span], terrain: &Terrain) -> Axis {
        let cuts: Vec<f64> = spans.iter().flat_map(|s| [s.a0, s.a1]).collect();
        let pts = densify_at(&w.pts, NODE_M, &cuts);
        let mut s = Vec::with_capacity(pts.len());
        let mut at = 0.0;
        for (i, p) in pts.iter().enumerate() {
            if i > 0 {
                at += (p[0] - pts[i - 1][0]).hypot(p[1] - pts[i - 1][1]);
            }
            s.push(at);
        }
        let ground: Vec<f64> = pts.iter().map(|p| height_at(terrain, p[0], p[1])).collect();
        let blind = blind_mask(&pts, &ground, terrain);
        let (inside, spanned) = spanned_mask(&s, &ground, spans);
        // The detectors read the *raw* profile: a bridged plateau is neither
        // a notch nor a crest, and running them on the reference would let
        // one pass reshape what the next one saw.
        let refused_notch = close_bounded_runs(&s, &ground, NOTCH_SPAN_M, NOTCH_FILL_MAX_M).1;
        let refused_crest = {
            let neg: Vec<f64> = ground.iter().map(|v| -v).collect();
            close_bounded_runs(&s, &neg, BUMP_SPAN_M, BUMP_SHAVE_MAX_M).1
        };
        // The three passes, kept apart so each can be measured against its
        // own cap: the bridging first (a plateau is neither notch nor crest),
        // then the closing, then the opening.
        //
        // **Twice, and glued at the span boundaries.** The pass over the
        // span-bridged profile is the one every ground station takes, so no
        // window reaches into a slot ([`spanned_mask`]); the pass over the
        // blind mask alone is the one a station *inside* a mapped span keeps,
        // because there the reference is not a target — the profile chords
        // across and solves against nothing — but it is still the evidence
        // `partition::derive` reads to decide whether the mapper's span is a
        // structure at all. Carried across there too, a deck over a 30 m
        // gorge reads no departure from the surface it is 30 m above, and an
        // 80 m annotation over a 40 m slot stopped being trimmed to it
        // (`spans::a_generous_annotation_is_trimmed_to_the_gorge`).
        let pass = |mask: &[bool]| {
            let bridged = bridge_blind(&s, &ground, mask);
            let closed = close_notches(&s, &bridged);
            let opened = open_bumps(&s, &closed);
            (bridged, closed, opened)
        };
        let plain = pass(&blind);
        // Most ways map no span, and then the two masks are one.
        let carried = if spanned.iter().any(|&x| x) {
            pass(&(0..s.len()).map(|k| blind[k] || spanned[k]).collect::<Vec<_>>())
        } else {
            plain.clone()
        };
        let glue = |a: &Vec<f64>, b: &Vec<f64>| -> Vec<f64> {
            (0..s.len()).map(|k| if inside[k] { a[k] } else { b[k] }).collect()
        };
        let bridged = glue(&plain.0, &carried.0);
        let closed = glue(&plain.1, &carried.1);
        let h = glue(&plain.2, &carried.2);
        let moved = Moved::of(&ground, &bridged, &closed, &h);
        Axis { way, s, p: pts, ground, h, blind, spanned, refused_notch, refused_crest, moved }
    }

    /// The axis's own length, in metres.
    pub fn len(&self) -> f64 {
        self.s.last().copied().unwrap_or(0.0)
    }

    /// Whether the axis has no stations at all.
    pub fn is_empty(&self) -> bool {
        self.s.is_empty()
    }

    /// How far the ground between arcs `a0` and `a1` stands above the higher
    /// of the two shoulders bracketing it — a refused crest's own rise, and
    /// what a class's deviation box has to reach to cut through it.
    pub fn rise(&self, a0: f64, a1: f64) -> f64 {
        let lo = self.s.partition_point(|x| *x < a0);
        let hi = self.s.partition_point(|x| *x <= a1);
        if lo == 0 || hi >= self.s.len() || hi <= lo {
            return 0.0;
        }
        let shoulder = self.ground[lo - 1].max(self.ground[hi]);
        let peak = self.ground[lo..hi].iter().copied().fold(f64::NEG_INFINITY, f64::max);
        peak - shoulder
    }

    /// How many metres of the axis stand on ground the DEM cannot vouch for.
    pub fn blind_m(&self) -> f64 {
        let mut m = 0.0;
        for k in 1..self.s.len() {
            if self.blind[k] && self.blind[k - 1] {
                m += self.s[k] - self.s[k - 1];
            }
        }
        m
    }
}

/// Where the ground under an axis is a structure's own top: it stands more
/// than [`DECK_STANDOFF_M`] above the ground [`FLANK_M`] to *both* sides.
///
/// Both sides, and therefore the **higher** of them. One flank below the axis
/// is a hillside — a road benched into a slope has a bank above it and a fall
/// below — and only a surface proud of the ground on both hands is standing
/// on something. Read against the *lower* flank instead, which is what the
/// server's own guard does inside its bridge trim, the mask fired on 70.6 %
/// of the loop box's stations: every contour road on the flank above
/// Montreux, whose downhill side is metres below it by construction. Bridging
/// those runs then drew the reference straight across the valleys they
/// overlook and lifted it by 145 m. The server can afford the loose reading
/// because it only ever asks inside an already-annotated bridge span; a mask
/// the whole pipeline leans on cannot.
///
/// The road's own height is not consulted — it does not exist yet, and it
/// does not need to: the question is about the DEM.
fn blind_mask(pts: &[[f64; 2]], ground: &[f64], terrain: &Terrain) -> Vec<bool> {
    let n = pts.len();
    let mut out = vec![false; n];
    for i in 0..n {
        let (a, b) = (pts[i.saturating_sub(1)], pts[(i + 1).min(n - 1)]);
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let len = dx.hypot(dy);
        if !(len > 0.0) {
            continue;
        }
        let (nx, ny) = (-dy / len, dx / len);
        let side = |sign: f64| {
            height_at(terrain, pts[i][0] + sign * nx * FLANK_M, pts[i][1] + sign * ny * FLANK_M)
        };
        out[i] = ground[i] - side(1.0).max(side(-1.0)) > DECK_STANDOFF_M;
    }
    out
}

/// Where the terrain under the axis is a **mapped structure's** slot or
/// mass rather than the way's own ground, as two masks: the stations inside
/// a bridge or tunnel span the source annotated, and those plus the
/// abutment either side of a bridge.
///
/// The first says where the reference is not a target and the second says
/// what no window may read; [`Axis::of`] needs both.
///
/// The rule is the blind mask's, one level up. [`blind_mask`] asks the DEM
/// whether it is standing on the way's *deck*; this asks the **source**
/// whether the way has left the ground at all. Both answers mean the same
/// thing to the passes that follow — the height here is not evidence about
/// where the way's ground is — and both are carried across by
/// [`bridge_blind`]. Read as ground instead, a 15 m bridge span put a 6 m
/// slot at the end of its own way's profile, and the opening then shaved
/// 3 m off the approach embankment beside it as a false crest.
///
/// **A bridge's mask grows to the rim and a bore's does not.** Under a deck
/// the annotation's edge is short of the slot by an abutment, and the
/// abutment is a cliff in a terrain model: from each end the mask walks out
/// while the terrain still falls into the span at more than
/// [`ABUTMENT_GRADE`], for at most [`ABUTMENT_M`]. It stops where the fall
/// becomes one a road could be built on, which on a bridge approach is the
/// embankment's own crest and is exactly the station the reference should be
/// carried from. Over a bore the mouth is not a smear to be cleaned up but a
/// place the model elects for itself (`partition::bore_bounds`), and the open
/// cutting in front of a portal is real ground that a growth would eat.
fn spanned_mask(arc: &[f64], ground: &[f64], spans: &[Span]) -> (Vec<bool>, Vec<bool>) {
    let n = arc.len();
    let mut inside = vec![false; n];
    let mut out = vec![false; n];
    // **The source's spans, and not the terrain's own priors.** A promoted
    // span ([`promote`]) is a notch the closing *refused* — the terrain said
    // the slot is real and too deep to fill — and it claims level 0 for
    // exactly that reason, where a mapper's flag never does. Masked like an
    // annotation it would be filled after all, by the bridging, and the cap
    // the refusal is made of would buy nothing.
    let mapped = |k: Kind| matches!(k, Kind::Bridge(n) | Kind::Tunnel(n) if n != 0);
    for span in spans.iter().filter(|s| mapped(s.kind)) {
        // Closed at both ends, and to [`EPS_M`]: a boundary station stands on
        // the abutment, which is the slot's wall and not the way's ground —
        // and the station *is* the boundary, put there by [`Axis::of`], so
        // the comparison is against a float the arc was summed to rather than
        // against a distinct place. Read exactly, the last station of a way
        // that ends on its own deck fell outside its span by half an ulp,
        // stayed sighted six metres down in the cutting, and the carry ran
        // down to meet it — which is the whole defect, unfixed.
        let lo = arc.partition_point(|&a| a < span.a0 - EPS_M);
        let hi = arc.partition_point(|&a| a <= span.a1 + EPS_M);
        if lo >= hi {
            continue;
        }
        inside[lo..hi].fill(true);
        out[lo..hi].fill(true);
        if !matches!(span.kind, Kind::Bridge(_)) {
            continue;
        }
        // The abutment is the wall between the span's edge and the rim above
        // it, so the rim is found first and the wall is what lies between:
        // the highest ground within [`ABUTMENT_M`] of the edge, taken as a
        // rim only where the climb to it is steeper than [`ABUTMENT_GRADE`].
        // The rim itself stays sighted — it is what the carry reads from.
        //
        // Found by the highest rather than walked station by station, because
        // a span boundary puts *two* stations within millimetres of each
        // other (the way's own vertex and the cut) and the grade across that
        // pair is whatever the DEM's last two digits say.
        let rim = |window: &mut dyn Iterator<Item = usize>, edge: usize| -> Option<usize> {
            // Highest first, and of equals the one nearest the edge, so the
            // mask over a plateau is the shortest that reaches a rim.
            let near = |k: usize| (arc[edge] - arc[k]).abs();
            let r = window.max_by(|&a, &b| {
                ground[a].total_cmp(&ground[b]).then(near(a).total_cmp(&near(b)).reverse())
            })?;
            (ground[r] - ground[edge] > ABUTMENT_GRADE * near(r)).then_some(r)
        };
        if let Some(r) = rim(&mut (0..lo).filter(|&k| arc[lo] - arc[k] <= ABUTMENT_M), lo) {
            out[r + 1..lo].fill(true);
        }
        if let Some(r) = rim(&mut (hi..n).filter(|&k| arc[k] - arc[hi - 1] <= ABUTMENT_M), hi - 1) {
            out[hi..r].fill(true);
        }
    }
    (inside, out)
}

/// The heights with every blind run replaced by a straight line between the
/// last sighted station before it and the first after: the DEM said "ground"
/// where it was looking at a deck, so the reference is carried across rather
/// than believed. A run that reaches an end of the axis holds the nearest
/// sighted height, there being no second rim to reach for.
fn bridge_blind(arc: &[f64], h: &[f64], blind: &[bool]) -> Vec<f64> {
    let n = h.len();
    let mut out = h.to_vec();
    let mut i = 0;
    while i < n {
        if !blind[i] {
            i += 1;
            continue;
        }
        let start = i;
        while i < n && blind[i] {
            i += 1;
        }
        let (before, after) = (start.checked_sub(1), (i < n).then_some(i));
        for k in start..i {
            out[k] = match (before, after) {
                (Some(a), Some(b)) if arc[b] > arc[a] => {
                    let t = (arc[k] - arc[a]) / (arc[b] - arc[a]);
                    h[a] + (h[b] - h[a]) * t
                }
                (Some(a), _) => h[a],
                (_, Some(b)) => h[b],
                // The whole axis is blind: nothing to carry across from, so
                // the raw heights stand and `blind` says they are not to be
                // trusted. A caller that needs a rim has to look wider than
                // this piece, which is R1's business.
                (None, None) => h[k],
            };
        }
    }
    out
}

/// The terrain profile with its narrow notches filled: a bounded
/// morphological closing along the arc — a running max then a running min
/// over ±[`NOTCH_SPAN_M`]/2 — which lifts every valley narrower than the
/// span to its rims and passes bumps, ramps and wide valleys through
/// untouched. A notch whose fill would exceed [`NOTCH_FILL_MAX_M`] keeps the
/// raw terrain, per contiguous run; the closing already meets the terrain at
/// a run's edges, so no step appears where it reverts.
pub fn close_notches(arc: &[f64], h: &[f64]) -> Vec<f64> {
    close_bounded_runs(arc, h, NOTCH_SPAN_M, NOTCH_FILL_MAX_M).0
}

/// The same with the narrow convex bumps shaved — the opening dual. Opening
/// is closing under negation (`open(h) = −close(−h)`), so it is the same
/// machinery on the same numbers upside down, and the two cannot drift.
pub fn open_bumps(arc: &[f64], h: &[f64]) -> Vec<f64> {
    let neg: Vec<f64> = h.iter().map(|v| -v).collect();
    let mut opened = close_bounded_runs(arc, &neg, BUMP_SPAN_M, BUMP_SHAVE_MAX_M).0;
    for v in &mut opened {
        *v = -*v;
    }
    opened
}

/// The conditioned surface: notches filled, then bumps shaved. Symmetric by
/// construction, so DEM noise enters the profile in neither direction and
/// genuine relief passes through in both. The closing runs first, so a
/// notch-and-bump pair — one signal ringing both ways — resolves toward the
/// engineered fill rather than toward the artifact.
pub fn condition(arc: &[f64], h: &[f64]) -> Vec<f64> {
    open_bumps(arc, &close_notches(arc, h))
}

/// Bounded closing, plus the runs it refused: the closed heights, and one
/// `(arc_first, arc_last)` per contiguous run whose fill exceeded `cap`.
///
/// Each refused interval covers exactly the stations the closing wanted to
/// lift — the notch's interior. The bracketing rim stations, where the
/// closing already meets the terrain, lie outside it, which is what makes
/// the interval a place a structure can land.
fn close_bounded_runs(arc: &[f64], h: &[f64], span: f64, cap: f64) -> (Vec<f64>, Vec<(f64, f64)>) {
    let n = h.len();
    if n == 0 {
        return (Vec::new(), Vec::new());
    }
    let r = span * 0.5;
    // One edge-replicated station at ±r, so the erosion pass has a *dilation*
    // to read in the extension rather than a copy of the first real one.
    // Extending the dilated array by its own edge value instead says the
    // dilation is already `h(r)` at `−r`, and the erosion can then never come
    // back down: the head of a rising axis lifts by `r · grade`, which on a
    // 5 % ramp is 1.5 m of invented fill at the first station.
    //
    // One node is enough *because* the fold reads its window's edges
    // ([`window_fold`]): between the pad and the first real station the
    // interpolation is the true dilation wherever the ground is linear, which
    // is what the extension of a linear head is.
    let mut pa = Vec::with_capacity(n + 2);
    let mut ph = Vec::with_capacity(n + 2);
    pa.push(arc[0] - r);
    ph.push(h[0]);
    pa.extend_from_slice(arc);
    ph.extend_from_slice(h);
    pa.push(arc[n - 1] + r);
    ph.push(h[n - 1]);
    let dilated = window_fold(&pa, &ph, r, f64::max);
    let eroded = window_fold(&pa, &dilated, r, f64::min);
    let mut closed: Vec<f64> = eroded[1..=n].to_vec();
    let mut refused: Vec<(f64, f64)> = Vec::new();
    let mut i = 0;
    while i < n {
        if closed[i] - h[i] <= EPS_M {
            closed[i] = h[i];
            i += 1;
            continue;
        }
        let start = i;
        let mut deepest = 0.0f64;
        while i < n && closed[i] - h[i] > EPS_M {
            deepest = deepest.max(closed[i] - h[i]);
            i += 1;
        }
        if deepest > cap {
            for k in start..i {
                closed[k] = h[k];
            }
            if two_rimmed(h, start, i, cap) {
                refused.push((arc[start], arc[i - 1]));
            }
        }
    }
    (closed, refused)
}

/// Whether a refused run `[start, i)` is a **notch** rather than the fringe
/// of something wider: the ground comes back up on *both* sides of it, by
/// more than the fill the closing refused to build.
///
/// A valley wider than the window is not refused at its floor — there the
/// window lies wholly inside it and the closing meets the terrain — but it is
/// refused along each flank, where the window reaches over the rim and wants
/// to fill down to it. Those two fringes are not slots the road spans; they
/// are the sides of a valley the road descends into. Told otherwise, a road
/// across a 120 m bowl was given two short decks on its shoulders and none
/// over the middle.
///
/// The rims are the stations just outside the run, where the closing already
/// meets the terrain. A run touching either end of the axis has no rim on
/// that side and is not a notch: its far side is off the profile, so nothing
/// about it is provable.
fn two_rimmed(h: &[f64], start: usize, end: usize, cap: f64) -> bool {
    if start == 0 || end >= h.len() {
        return false;
    }
    let rim = h[start - 1].min(h[end]);
    let floor = h[start..end].iter().copied().fold(f64::INFINITY, f64::min);
    rim - floor > cap
}

/// `fold` of `h` over the arc window ±`r` around each station — **including
/// the interpolated values at the window's two edges**, and with the signal
/// extended by its edge value beyond the ends.
///
/// The edges are what make it exact. A profile is piecewise linear, so the
/// true extremum over `[s−r, s+r]` is the fold of the two edge values with
/// the samples strictly inside; taking only the samples makes the answer
/// depend on whether a station happens to land at `s − r`. It usually does
/// not — the stations are `NODE_M` apart *except* where a span boundary or a
/// way vertex sits between two of them — and then the erosion cannot undo the
/// dilation: **closing stops being the identity on monotone ground and lifts
/// it by about one station spacing times the slope.** Measured on a 50 % ramp
/// with a boundary station 0.77 m from its neighbour, that was 0.385 m of
/// invented fill, which the profile then chorded from.
///
/// Reading the edges also removes the need to pad: the extension is constant,
/// so `at()` answers for it directly.
fn window_fold(arc: &[f64], h: &[f64], r: f64, fold: fn(f64, f64) -> f64) -> Vec<f64> {
    let n = h.len();
    let at = |x: f64| -> f64 {
        if x <= arc[0] {
            return h[0];
        }
        if x >= arc[n - 1] {
            return h[n - 1];
        }
        let j = arc.partition_point(|&a| a < x).clamp(1, n - 1);
        let (a0, a1) = (arc[j - 1], arc[j]);
        let t = if a1 > a0 { (x - a0) / (a1 - a0) } else { 0.0 };
        h[j - 1] + (h[j] - h[j - 1]) * t
    };
    let mut out = Vec::with_capacity(n);
    let (mut lo, mut hi) = (0usize, 0usize);
    for i in 0..n {
        let (a, b) = (arc[i] - r, arc[i] + r);
        while lo < n && arc[lo] <= a {
            lo += 1;
        }
        while hi < n && arc[hi] < b {
            hi += 1;
        }
        let mut v = fold(at(a), at(b));
        for &x in &h[lo..hi] {
            v = fold(v, x);
        }
        out.push(v);
    }
    out
}

#[cfg(test)]
mod tests {
    use crate::pipeline::tests::{built, upto};
    use crate::step::Step;
    use crate::world::World;

    use super::*;

    /// A world on `ground` with the network of `net`, referenced.
    fn world(ground: &str, net: &str) -> (World, Summary) {
        let (w, ran) = built(ground, net, None, 5.0, &upto(Step::Reference));
        (w, ran.last())
    }

    /// The one axis of a single-way specimen.
    fn axis(w: &World) -> &Axis {
        let r = w.reference.as_ref().expect("the reference step ran");
        assert_eq!(r.axes.len(), 1, "one axis");
        &r.axes[0]
    }

    /// A profile `n` stations apart at `step` metres, from a function of arc.
    fn ramp_h(n: usize, step: f64, f: impl Fn(f64) -> f64) -> (Vec<f64>, Vec<f64>) {
        let arc: Vec<f64> = (0..n).map(|i| i as f64 * step).collect();
        let h = arc.iter().map(|&a| f(a)).collect();
        (arc, h)
    }

    // ------------------------------------------------- the two passes

    /// **Closing is the identity on monotone ground, right up to the ends
    /// and at any sampling.**
    ///
    /// Two boundary bugs meet here. Padded with a single virtual station —
    /// the server's construction — the erosion window loses the pad from the
    /// second station on, and the head of every rising axis is lifted by up
    /// to `NOTCH_SPAN_M / 2 · grade`: 1.2 m over the first 30 m of a 5 %
    /// ramp, out of nothing but the edge. And reading only the *samples*
    /// inside the window, the erosion can undo the dilation only where a
    /// station happens to land at `arc − r`; it usually does not, and the
    /// lift is then about one spacing times the slope.
    ///
    /// The stations are deliberately uneven below, because that is what a
    /// real axis is: `NODE_M` apart except where a span boundary or a way
    /// vertex sits between two of them.
    #[test]
    fn closing_is_the_identity_on_a_ramp() {
        let (arc, h) = ramp_h(51, 4.0, |a| 400.0 + 0.05 * a);
        let closed = close_notches(&arc, &h);
        for k in 0..h.len() {
            assert!((closed[k] - h[k]).abs() < 1e-9, "station {k}: {} vs {}", closed[k], h[k]);
        }
        // And the opening likewise, so the conditioned surface is the ramp.
        let cond = condition(&arc, &h);
        for k in 0..h.len() {
            assert!((cond[k] - h[k]).abs() < 1e-9, "station {k}: {} vs {}", cond[k], h[k]);
        }
        // Unevenly stationed — a boundary 0.77 m from its neighbour, on a
        // 50 % ramp, which is where this was found — it is still the
        // identity.
        let mut arc: Vec<f64> = (0..14).map(|i| i as f64 * 50.0 / 13.0).collect();
        arc.push(20.0);
        arc.sort_by(f64::total_cmp);
        let h: Vec<f64> = arc.iter().map(|a| 390.0 + 0.5 * a).collect();
        let cond = condition(&arc, &h);
        for k in 0..h.len() {
            assert!((cond[k] - h[k]).abs() < 1e-9, "uneven station {k}: {} vs {}", cond[k], h[k]);
        }
    }

    /// A notch inside the budget is filled to its rims, and only the notch:
    /// the ground either side of it is untouched, so no step appears where
    /// the fill ends.
    #[test]
    fn a_narrow_shallow_notch_is_filled_to_its_rims() {
        // 4 m deep, 20 m across, on a 200 m flat profile.
        let (arc, h) = ramp_h(51, 4.0, |a| if (90.0..=110.0).contains(&a) { 396.0 } else { 400.0 });
        let closed = close_notches(&arc, &h);
        for k in 0..h.len() {
            assert!(closed[k] >= h[k] - 1e-9, "the closing may not cut: station {k}");
            assert!((closed[k] - 400.0).abs() < 1e-9, "station {k} at {}", closed[k]);
        }
        assert!(close_bounded_runs(&arc, &h, NOTCH_SPAN_M, NOTCH_FILL_MAX_M).1.is_empty());
    }

    /// A notch past the budget keeps the raw terrain and is *reported*: the
    /// closing's refusal is the terrain's own bridge prior.
    #[test]
    fn a_deep_notch_is_refused_and_reported() {
        // 30 m deep, 40 m across: past NOTCH_FILL_MAX_M, so not fillable.
        let (arc, h) = ramp_h(51, 4.0, |a| if (80.0..=120.0).contains(&a) { 370.0 } else { 400.0 });
        let (closed, refused) = close_bounded_runs(&arc, &h, NOTCH_SPAN_M, NOTCH_FILL_MAX_M);
        assert_eq!(refused.len(), 1, "one refusal: {refused:?}");
        let (a0, a1) = refused[0];
        assert!((a0 - 80.0).abs() < 1e-9 && (a1 - 120.0).abs() < 1e-9, "{refused:?}");
        for k in 0..h.len() {
            assert!((closed[k] - h[k]).abs() < 1e-9, "the raw terrain is kept: station {k}");
        }
    }

    /// A valley wider than the span is relief, not an artifact: the closing
    /// passes it through and refuses nothing, so no prior is invented.
    #[test]
    fn a_wide_valley_is_neither_filled_nor_refused() {
        // 200 m across — well past NOTCH_SPAN_M — and only 10 m deep.
        let (arc, h) = ramp_h(101, 4.0, |a| if (100.0..=300.0).contains(&a) { 390.0 } else { 400.0 });
        let (closed, refused) = close_bounded_runs(&arc, &h, NOTCH_SPAN_M, NOTCH_FILL_MAX_M);
        assert!(refused.is_empty(), "{refused:?}");
        // The rims are lifted by the window, but the valley's floor is not.
        assert!((closed[50] - 390.0).abs() < 1e-9, "the floor at {}", closed[50]);
    }

    /// Opening is closing under negation, and that is asserted rather than
    /// trusted: the two passes cannot drift apart if they are one pass.
    #[test]
    fn opening_is_closing_upside_down() {
        let (arc, h) = ramp_h(51, 4.0, |a| 400.0 + 6.0 * (a / 30.0).sin());
        let neg: Vec<f64> = h.iter().map(|v| -v).collect();
        let opened = open_bumps(&arc, &h);
        let closed_neg = close_bounded_runs(&arc, &neg, BUMP_SPAN_M, BUMP_SHAVE_MAX_M).0;
        for k in 0..h.len() {
            assert!((opened[k] + closed_neg[k]).abs() < 1e-9, "station {k}");
            assert!(opened[k] <= h[k] + 1e-9, "the opening may not lift: station {k}");
        }
    }

    // --------------------------------------------------- the blind mask

    /// The blindness rung: a causeway across a trench reads as ground on the
    /// axis and as a deck from the side, and the reference is carried across
    /// it from the two rims rather than believed.
    #[test]
    fn a_causeway_is_blind_and_the_reference_crosses_it() {
        let (w, s) = world("shelf?drop=30&width=40&flank=8", "net:straight");
        let a = axis(&w);
        assert!(a.blind.iter().any(|b| *b), "nothing read as blind: {s}");
        // Blind over the trench and sighted past its rims.
        let at = |m: f64| a.s.iter().position(|&x| x >= m + a.len() / 2.0).expect("a station");
        assert!(a.blind[at(0.0)], "the middle of the causeway is sighted: {s}");
        assert!(!a.blind[at(60.0)], "the ground past the rim reads blind: {s}");
        // The DEM says the axis is level; the reference agrees, because the
        // rims it is carried between are level too. What has changed is that
        // the stretch is now *marked*, which is what the derivation needs.
        assert!(a.blind_m() > 30.0, "blind_m {} of a 40 m trench", a.blind_m());
    }

    /// And a road on open ground is not blind, whatever the ground does
    /// along it: one flank below the axis is a hillside, not a structure.
    #[test]
    fn a_road_on_a_slope_is_not_blind() {
        for ground in ["flat?h=400", "ramp?grade=0.20", "hill?amp=60&radius=400"] {
            let (w, s) = world(ground, "net:straight");
            let a = axis(&w);
            assert!(a.blind.iter().all(|b| !*b), "{ground} read as blind: {s}");
        }
    }

    // ------------------------------------------------------- the priors

    /// The terrain's bridge prior, end to end through the step: a slot the
    /// closing will not fill, under a way mapped level across it.
    #[test]
    fn a_gorge_is_reported_as_a_bridge_prior() {
        let (w, s) = world("gorge?depth=30&width=40", "net:straight");
        let a = axis(&w);
        assert_eq!(a.refused_notch.len(), 1, "one notch: {s}");
        assert!(a.refused_crest.is_empty(), "a gorge is not a crest: {s}");
        let (a0, a1) = a.refused_notch[0];
        assert!((a1 - a0 - 40.0).abs() < 8.0, "the notch is the gorge: {a0}..{a1}");
        // And the raw terrain is kept across it — a refusal fills nothing.
        assert_eq!(s.num("filled"), 0.0, "{s}");
    }

    /// Its mirror. The prior is the terrain's alone; which classes may act
    /// on it is the partition step's business, not this one's.
    #[test]
    fn a_ridge_is_reported_as_a_tunnel_prior() {
        let (w, s) = world("ridge?height=40&width=120", "net:straight?len=400");
        let a = axis(&w);
        assert_eq!(a.refused_crest.len(), 1, "one crest: {s}");
        assert!(a.refused_notch.is_empty(), "a ridge is not a notch: {s}");
        assert_eq!(s.num("shaved"), 0.0, "a refusal shaves nothing: {s}");
    }

    /// A culvert is filled and reported as nothing: it is ground continuity
    /// that was engineered, and the road runs over it.
    #[test]
    fn a_culvert_is_filled_and_is_no_prior() {
        let (w, s) = world("gorge?depth=4&width=20", "net:straight");
        let a = axis(&w);
        assert!(a.refused_notch.is_empty(), "a culvert is not a bridge: {s}");
        assert!(s.num("filled") > 0.0, "the culvert was not filled: {s}");
        // The reference is level across it, so nothing downstream can dive.
        let mid = a.s.len() / 2;
        assert!((a.h[mid] - a.h[0]).abs() < 1e-6, "the reference dips: {}", a.h[mid]);
        assert!(a.ground[mid] < a.h[mid] - 3.0, "the raw ground did not dip");
    }

    /// Flat ground is left exactly alone: the step's own no-op.
    #[test]
    fn flat_ground_is_its_own_reference() {
        let (w, s) = world("flat?h=400", "net:cross");
        for a in &w.reference.as_ref().unwrap().axes {
            for k in 0..a.s.len() {
                assert!((a.h[k] - a.ground[k]).abs() < 1e-9, "station {k}");
            }
        }
        assert_eq!(s.num("dem_residual"), 0.0, "{s}");
        assert_eq!(s.num("notch"), 0.0, "{s}");
        assert_eq!(s.num("crest"), 0.0, "{s}");
    }

    /// **Each pass is measured against its own cap.** The three of them move
    /// the surface for three different reasons, and a composite number
    /// belongs to none of them: on the loop box the composite read a 13.26 m
    /// "shave" against a 4 m budget, which was the bridging setting a
    /// causeway down on its rims.
    #[test]
    fn each_pass_stays_inside_its_own_budget() {
        for ground in [
            "flat?h=400",
            "ramp?grade=0.20",
            "hill?amp=60&radius=400",
            "gorge?depth=30&width=40",
            "gorge?depth=4&width=20",
            "ridge?height=40&width=120",
            "shelf?drop=30&width=40&flank=8",
        ] {
            let (w, s) = world(ground, "net:straight?len=400");
            for a in &w.reference.as_ref().unwrap().axes {
                assert!(a.moved.fill_m <= NOTCH_FILL_MAX_M + 1e-9, "{ground} fill: {s}");
                assert!(a.moved.shave_m <= BUMP_SHAVE_MAX_M + 1e-9, "{ground} shave: {s}");
                // The bridging has no cap of its own: it is bounded by the
                // rims it reaches between, and by the mask
                // (`the_bridging_touches_only_blind_stations`).
            }
        }
    }

    /// Only a blind station is ever bridged: the pass may not touch ground
    /// the DEM can vouch for.
    #[test]
    fn the_bridging_touches_only_blind_stations() {
        let (w, _) = world("shelf?drop=30&width=40&flank=8", "net:straight");
        let a = axis(&w);
        let bridged = bridge_blind(&a.s, &a.ground, &a.blind);
        for k in 0..a.s.len() {
            if !a.blind[k] {
                assert!((bridged[k] - a.ground[k]).abs() < 1e-12, "sighted station {k} moved");
            }
        }
    }

    // ------------------------------------------- a mapped span is not ground

    /// One way, in the shape the Montreux rail overbridge has: a 5 %
    /// embankment climbing 160 m to a bridge, under which a terrain model
    /// with its bridges taken out shows the railway six metres down in its
    /// cutting — and the way ends on the deck, at the connector where the
    /// street on the far side takes over.
    ///
    /// Two things about it are the case that matters, and both are the
    /// source's doing rather than the terrain's. **The annotation starts at
    /// the foot of the abutment**, not at its head: a span boundary is where
    /// a mapper clicked. And **the way ends inside the slot**, so the only
    /// rim its own profile has is the one behind it.
    fn overbridge() -> (Vec<f64>, Vec<f64>, Vec<Span>) {
        let arc: Vec<f64> = (0..=46).map(|k| k as f64 * 4.0).collect();
        let ground: Vec<f64> = arc
            .iter()
            .map(|&s| match s {
                s if s <= 160.0 => 392.0 + 0.05 * s,
                s if s < 168.0 => 400.0 - 6.0 * (s - 160.0) / 8.0,
                _ => 394.0,
            })
            .collect();
        (arc, ground, vec![Span { a0: 168.0, a1: 184.0, kind: Kind::Bridge(1) }])
    }

    /// **The terrain under a mapped span is not the far shoulder of a
    /// crest.** The DEM dives into the slot the deck spans, so the last
    /// forty metres of the approach stand above their own surroundings and
    /// the opening shaves the embankment away as a false bump — 3.00 m of it
    /// at the site this specimen is drawn from, the reference flattened to
    /// 398.20 where the terrain climbs to 401.20. Every consumer paid: the
    /// road solved three metres into its own embankment, the crossing step
    /// then bought the clearance back as a ramp the service roads beside it
    /// dropped five metres off, and the partition derived a bore out of the
    /// hill the shave had invented.
    #[test]
    fn the_terrain_under_a_mapped_span_is_not_a_crest_s_shoulder() {
        let (arc, ground, spans) = overbridge();
        let none = vec![false; arc.len()];
        let bare = open_bumps(&arc, &close_notches(&arc, &bridge_blind(&arc, &ground, &none)));
        let crest = arc.iter().position(|&s| s == 160.0).expect("the embankment's own crest");
        let shave = ground[crest] - bare[crest];
        assert!(shave > 2.0, "no shave to fix: {shave:.2}");

        let (_, mask) = spanned_mask(&arc, &ground, &spans);
        let kept = open_bumps(&arc, &close_notches(&arc, &bridge_blind(&arc, &ground, &mask)));
        assert!(
            (kept[crest] - ground[crest]).abs() < 0.1,
            "the crest still moved: {:.2} against {:.2}",
            kept[crest],
            ground[crest]
        );
    }

    /// The mask reaches past the annotation's edge, because the edge is
    /// where a mapper clicked and the abutment is a cliff — but only down
    /// the cliff. The station at the embankment's crest is the one the
    /// reference has to be carried *from*, so it stays sighted.
    #[test]
    fn a_mapped_span_s_mask_reaches_down_the_abutment_and_stops_at_the_rim() {
        let (arc, ground, spans) = overbridge();
        let (inside, mask) = spanned_mask(&arc, &ground, &spans);
        let at = |s: f64| arc.iter().position(|x| *x == s).expect("a station there");
        assert!(!mask[at(160.0)], "the rim is masked, and it is what the carry reads from");
        assert!(!inside[at(164.0)] && mask[at(164.0)], "the abutment is not reached");
        assert!(inside[at(168.0)] && mask[at(168.0)], "the span itself is not masked");
        assert!(!mask[at(60.0)], "open ground short of the span is masked");
    }

    /// **The terrain's own priors are not annotations.** A promoted span is
    /// a notch the closing *refused* — it said the slot is real and too deep
    /// to fill, and claimed level 0 saying so. Carried across as if a mapper
    /// had drawn it, the refusal would be spent twice and the cap
    /// [`NOTCH_FILL_MAX_M`] is made of would buy nothing.
    #[test]
    fn a_promoted_span_is_not_masked() {
        let (arc, ground, _) = overbridge();
        let promoted = vec![Span { a0: 168.0, a1: 184.0, kind: Kind::Bridge(0) }];
        let (inside, mask) = spanned_mask(&arc, &ground, &promoted);
        let any = inside.iter().chain(&mask).any(|b| *b);
        assert!(!any, "the terrain's own prior was masked");
    }

    // ------------------------------------------------ one value at a junction

    /// **Two ways meeting at a point make one ground of it.** The
    /// conditioning is per axis, so a notch at a junction can be closed by
    /// one way and refused by its neighbour, and the two then stand a whole
    /// `NOTCH_FILL_MAX_M` apart at a point they share. On the loop box that
    /// was 15.103 m, and it reached every consumer: the profile anchors a
    /// junction on whichever way it sees first, so the others were pinned
    /// that far off their own target.
    #[test]
    fn ways_meeting_at_a_junction_agree_about_the_ground() {
        for (ground, net) in [
            ("gorge?depth=30&width=40", "net:cross?len=200"),
            ("gorge?depth=4&width=20", "net:tee?len=200"),
            ("hill?amp=60&radius=400", "net:cross?len=400"),
            ("shelf?drop=30&width=40&flank=8", "net:cross?len=200"),
        ] {
            let (w, s) = world(ground, net);
            let axes = &w.reference.as_ref().unwrap().axes;
            assert!(disagreement(axes) < 1e-9, "{ground} {net}: {s}");
        }
    }

    /// And the agreement is not bought by flattening the conditioning: away
    /// from the junctions the reference is what the passes made of it, and
    /// the correction dies within the window that caused it.
    #[test]
    fn the_agreement_is_local_to_the_junction() {
        let (w, _) = world("gorge?depth=4&width=20", "net:cross?len=400");
        let axes = &w.reference.as_ref().unwrap().axes;
        let mut raw = axes.clone();
        // Undo the agreement by rebuilding without it.
        let terrain = w.terrain.as_ref().unwrap();
        let ways = &w.roads.as_ref().unwrap().ways;
        raw.clone_from(&of(ways, &solving_of(ways), terrain).axes);
        for (a, b) in axes.iter().zip(raw.iter()) {
            for k in 0..a.s.len() {
                let from_end = a.s[k].min(a.len() - a.s[k]);
                if from_end > NOTCH_SPAN_M * 0.5 + 1e-9 {
                    assert!(
                        (a.h[k] - b.h[k]).abs() < 1e-9,
                        "station {k} at {from_end:.1} m from an end moved"
                    );
                }
            }
        }
    }

    /// The reference is a function of the world: two runs over one world
    /// agree to the bit, so a difference downstream is a change and not a
    /// reordering.
    #[test]
    fn the_reference_is_a_function_of_the_world() {
        let (w, _) = world("hill?amp=60&radius=400", "net:cross");
        let first = w.reference.clone().expect("built");
        let terrain = w.terrain.clone().expect("built");
        let (again, _, _) = run(&terrain, w.roads.as_ref().expect("built"));
        assert_eq!(first, again);
    }
}
