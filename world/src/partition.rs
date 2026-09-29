//! The partition: where a way is a deck, a bore, or on the ground,
//! and the one place the geometry is cut.
//!
//! The reader hands on whole ways with their spans as an attribute
//! ([`crate::world::Way`]), and the reference, the profile and the crossing
//! steps all work on those whole ways — so the reference has a corridor to
//! condition and the profile has no pin at a mapper's split point. This step
//! runs after them, with the solved heights in hand, and does two things:
//!
//! - **It decides the spans** ([`spans`]): the annotation trimmed and grown
//!   to the runs the solved heights imply ([`derive()`]), kept whole where the
//!   heights have nothing to say, and with runs the annotation never had
//!   added. That table is written back into the ways *and* the profiles, so
//!   it is the one span truth every later step reads.
//! - **It cuts the geometry**: one [`Polyline2`] per span, the ground pieces
//!   into [`Network::plan`] — which every surface step builds from — and the
//!   rest into [`Network::spans`], grouped into the surfaces that may merge
//!   ([`groups`]).
//!
//! The rules the spans follow on the structure specimens are checked in
//! `specs::spans`.

use crate::standard::{bore_cover_m, DECK_STANDOFF_M};
use std::collections::{HashMap, HashSet};

use crate::line;
use crate::poly::Pt;
use crate::step::Summary;
use crate::world::{Kind, Polyline2, Profile, Profiles, Network, Span, Way};
use crate::world::{Groups, Partition};

/// A grade sliver shorter than this, between two runs of one kind, is an edge
/// mismatch rather than real at-grade road: one pier's ground reading coming
/// up does not make a viaduct into two viaducts.
const SNAP_RUN_M: f64 = 10.0;

/// Shortest run that is a structure on its length alone, in metres.
const MIN_STRUCTURE_M: f64 = 40.0;

/// Smallest mid-run departure that makes a shorter run a structure anyway: a
/// 25 m span over a 30 m stream cut is a real bridge, and demoting it blindly
/// dives the road through the gorge it crosses.
const SHORT_STRUCTURE_DIP_M: f64 = 3.0;

/// Cuts every way of `ways` at the span boundaries the `solved` heights
/// imply, and writes that table back into the profiles.
pub fn run(ways: &[Way], solved: &Profiles) -> (Partition, Summary) {
    let mut ways = ways.to_vec();
    let annotated: Vec<Vec<Span>> = ways.iter().map(|w| w.spans.clone()).collect();

    // **The cut follows the geometry.** Every solved way's span table is
    // replaced by what the solved heights and the annotation together say it
    // is, and the pieces are cut from that. After this there is one span
    // truth and every consumer reads it. A way that solved no profile keeps
    // the table the source gave it: this step has no heights for it.
    let (mut found, mut lost, mut moved) = (0usize, 0usize, 0.0f64);
    let (mut deck_m, mut bore_m) = (0.0f64, 0.0f64);
    let mut portal_m = 0.0f64;
    let mut runs = 0usize;
    for p in &solved.profiles {
        let w = p.way;
        let derived = derive(p);
        runs += derived.len();
        for r in &derived {
            if r.kind.is_deck() {
                deck_m += r.len();
            } else {
                bore_m += r.len();
            }
        }
        let len = ways[w].len();
        let (out, opened) = open_portals(spans(&ways[w], p, &derived, len), p);
        portal_m += opened;
        let structure = |list: &[Span]| -> Vec<Span> {
            list.iter().filter(|s| s.kind.is_structure()).copied().collect()
        };
        let (was, now) = (structure(&annotated[w]), structure(&out));
        let overlaps = |x: &Span, list: &[Span]| list.iter().any(|y| y.a1 > x.a0 && y.a0 < x.a1);
        found += now.iter().filter(|x| !overlaps(x, &was)).count();
        lost += was.iter().filter(|x| !overlaps(x, &now)).count();
        moved += divergence(&annotated[w], &out);
        ways[w].spans = out;
    }

    // **One span truth.** The profiles carry the annotation they were solved
    // against; from here they carry the partition instead, so the structure
    // step, the lift and the plan view cut the same thing the surface steps
    // do. Correcting the cut and leaving one consumer on the annotation's table would
    // put a joint that cannot meet at every trimmed bridge: the paving and
    // the solids would disagree about where a deck is.
    //
    // The per-station verdict is recomputed with them: a station the trim
    // freed is at grade, and one a grow took in is a deck or a bore by
    // the same rule the solve applied (§4.5).
    let mut profiles = solved.clone();
    for p in &mut profiles.profiles {
        p.spans = ways[p.way].spans.clone();
        p.classify();
    }

    let (mut plan, mut spans) = (Vec::new(), Vec::new());
    let (mut ground_m, mut span_m) = (0.0f64, 0.0f64);
    for (i, way) in ways.iter().enumerate() {
        for piece in cut_at(way, i) {
            let m = line::length(&piece.pts);
            if piece.kind == Kind::Ground {
                ground_m += m;
                plan.push(piece);
            } else {
                span_m += m;
                spans.push(piece);
            }
        }
    }
    let (grouping, linkage) = groups(&plan, &spans);
    let group = &grouping.of;
    let grouped: std::collections::HashSet<usize> = group.iter().copied().collect();
    // **How many groups hold both kinds of piece.** Those are the ones with
    // a handover inside them — where a ground piece and a span of one
    // surface meet at an abutment — and where the seam between them lies.
    let mixed = {
        let mut ground: std::collections::HashSet<usize> = std::collections::HashSet::new();
        let mut over: std::collections::HashSet<usize> = std::collections::HashSet::new();
        for (i, g) in group.iter().enumerate() {
            if i < plan.len() { ground.insert(*g); } else { over.insert(*g); }
        }
        ground.intersection(&over).count()
    };
    let summary = Summary::new()
        .with("ways", ways.len())
        .with("structures", ways.iter().filter(|w| w.has_structure()).count())
        .with("plan", plan.len())
        .with("spans", spans.len())
        .with("ground_m", format!("{ground_m:.0}"))
        .with("span_m", format!("{span_m:.0}"))
        .with("derived", runs)
        .with("deck_m", format!("{deck_m:.0}"))
        .with("bore_m", format!("{bore_m:.0}"))
        .with("found", found)
        .with("lost", lost)
        .with("moved_m", format!("{moved:.0}"))
        .with("portal_m", format!("{portal_m:.0}"))
        .with("linked", format!("{}/{}", linkage.components, linkage.largest))
        .with("crossings", linkage.crossings)
        .with("groups", grouped.len())
        .with("mixed", mixed);
    (Partition { network: Network { ways, plan, spans }, profiles, groups: grouping }, summary)
}

/// How far the profiles stand off the raw DEM, at grade, once the cut is
/// written back into them.
///
/// The step rewrites the span table and the profiles with it, so a station
/// that was a chord can become at-grade and the population itself changes.
/// That is exactly what wants reporting: a residual that jumps here is a
/// span the partition gave back to the ground.
pub fn check(partition: &Partition) -> Summary {
    Summary::new().with_residual(partition.profiles.residual())
}

/// The pieces of one way: its polyline cut at every span boundary, each
/// piece carrying that span's [`Kind`] and `index`, the way's own place in
/// [`Network::ways`] — how a consumer holding a piece finds the profile its
/// way was solved into. Two neighbouring pieces share the boundary vertex
/// exactly.
pub fn cut_at(way: &Way, index: usize) -> Vec<Polyline2> {
    let mut out = Vec::with_capacity(way.spans.len());
    for span in &way.spans {
        let pts = line::between(&way.pts, span.a0, span.a1);
        if pts.len() < 2 {
            continue;
        }
        out.push(Polyline2 {
            id: way.id.clone(),
            class: way.class.clone(),
            subclass: way.subclass.clone(),
            width_m: way.width_m,
            kind: span.kind,
            way: index,
            a0: span.a0,
            a1: span.a1,
            pts,
        });
    }
    out
}

/// `spans` with every bore's portal cutting given back to the ground, and
/// how many metres of it there were.
///
/// **A portal is where the tube goes into the hill, not where the road
/// does.** Between the two the road runs under the ground by less than its
/// tube is tall: nothing fits over it, and if the span piece carried
/// it on, no surface step would pave it and the bench would cut nothing —
/// the terrain would lie on the roadway, and the mouth would be hill. That
/// stretch is an open cutting, and a cutting is ground: cut here, the
/// surface steps pave it, the bench cuts it into the terrain and walls it
/// where it is deeper than a face, and the tube starts where it fits.
///
/// The bore itself is not moved. Where it is — which runs are under the
/// ground at all — is [`spans`]'s and [`bore_bounds`]'s, elected on
/// the line; this only says where along it the cut between a cutting and a
/// tube falls, and it falls where the tube's roof first goes under the raw
/// ground ([`roof_gap`]), interpolated between stations. Only an end that
/// meets a ground piece is opened: a bore running on into a deck or off the
/// way's end has no cutting in front of it.
fn open_portals(spans: Vec<Span>, p: &Profile) -> (Vec<Span>, f64) {
    let st = &p.stations;
    let tube = crate::standard::tube_m(&p.class);
    let gap = |k: usize| roof_gap(st[k].h, st[k].ground, tube);
    // Where the roof crosses the ground between stations `i` (clear of it)
    // and `j` (under it).
    let fit = |i: usize, j: usize| {
        let (gi, gj) = (gap(i), gap(j));
        let t = if (gi - gj).abs() < f64::EPSILON { 0.0 } else { (gi / (gi - gj)).clamp(0.0, 1.0) };
        st[i].s + (st[j].s - st[i].s) * t
    };
    let mut out: Vec<Span> = Vec::with_capacity(spans.len() + 2);
    let mut opened = 0.0;
    for (i, sp) in spans.iter().enumerate() {
        if !matches!(sp.kind, Kind::Tunnel(_)) {
            out.push(*sp);
            continue;
        }
        let inside: Vec<usize> = (0..st.len()).filter(|&k| st[k].s > sp.a0 && st[k].s < sp.a1).collect();
        let (Some(&f), Some(&l)) = (inside.iter().find(|&&k| gap(k) < 0.0), inside.iter().rev().find(|&&k| gap(k) < 0.0))
        else {
            out.push(*sp);
            continue;
        };
        let before = i > 0 && spans[i - 1].kind == Kind::Ground;
        let after = i + 1 < spans.len() && spans[i + 1].kind == Kind::Ground;
        let t0 = if before && f > 0 { fit(f - 1, f).max(sp.a0) } else { sp.a0 };
        let t1 = if after && l + 1 < st.len() { fit(l + 1, l).min(sp.a1) } else { sp.a1 };
        let t0 = if t0 - sp.a0 > MIN_SPAN_M { t0 } else { sp.a0 };
        let t1 = if sp.a1 - t1 > MIN_SPAN_M { t1 } else { sp.a1 };
        if t1 - t0 < MIN_SPAN_M {
            out.push(*sp);
            continue;
        }
        if t0 > sp.a0 {
            out.push(Span { a0: sp.a0, a1: t0, kind: Kind::Ground });
        }
        out.push(Span { a0: t0, a1: t1, kind: sp.kind });
        if t1 < sp.a1 {
            out.push(Span { a0: t1, a1: sp.a1, kind: Kind::Ground });
        }
        opened += (t0 - sp.a0) + (sp.a1 - t1);
    }
    // The cutting joins the approach it opens onto: one ground span.
    let mut merged: Vec<Span> = Vec::with_capacity(out.len());
    for sp in out {
        match merged.last_mut() {
            Some(last) if last.kind == Kind::Ground && sp.kind == Kind::Ground => last.a1 = sp.a1,
            _ => merged.push(sp),
        }
    }
    (merged, opened)
}

/// The signed daylight of the drawn tube at one station: how far a bore's
/// **roof** stands above the raw ground. Negative is buried — the whole
/// constant-section tube fits under the ground — and the zero crossing is
/// where a portal can be.
///
/// This is a different question from [`derive`]'s, and both are needed. The
/// derivation asks how deep the road runs, and decides whether a bore is what
/// is there at all. This asks whether the *thing that would be drawn* fits,
/// and decides where it stops. A run can be deep enough to be a tunnel and
/// still have tails the tube pokes out of.
fn roof_gap(h: f64, ground: f64, tube: f64) -> f64 {
    h + tube - ground
}

/// The bounds of a bore inside `[a0, a1]`: **its interior by the line, its
/// ends by whichever criterion the run itself elects.**
///
/// The interior is where the road runs below the reference. A gully crossing
/// the alignment or a shore gallery whose cover thins mid-run never splits
/// it — the line never daylights there, so it is a covered stretch of one
/// tunnel and not two with a trench between them.
///
/// The **ends** follow the majority. A real bore holds its tube almost
/// everywhere and grazes only at its mouths, so its shallow ends are the
/// portal transition and the bounds are the line's own crossings. A run
/// that fits the tube only in a *minority* is not a bore with shallow mouths
/// but a surface gallery with one deep spot: its ends pull back to where the
/// tube fits, and the freed tails are the open cutting they are. Judged by
/// the roof alone every real portal would move into the hill; judged by the
/// line alone a gallery would draw its roof proud of the hillside.
///
/// The seed is the run of greatest **integrated burial**, so a half-metre
/// graze of DEM noise on the approach cannot capture the solve from the deep
/// run beside it.
///
/// `None` when nothing in the window is buried at all, or when a minority-fit
/// run holds the tube nowhere: a tunnel tagged over ground that never covers
/// it has no bore, and the open cutting is what is there.
fn bore_bounds(p: &Profile, a0: f64, a1: f64) -> Option<(f64, f64)> {
    // **Against the raw ground, not the reference.** The derivation asks
    // whether the road runs deep enough to be a bore, and asks it of the
    // surface the road was solved to. This asks whether the drawn tube fits
    // under the drawn ground, and the ground that is drawn is the terrain
    // itself — which is also what `structure`'s own `open` and `cover`
    // measure the tube against. Elected against the reference instead, the
    // two would disagree wherever the conditioning moved the surface, and a
    // tunnel would be pulled back for a fit that is never the one checked.
    let st = &p.stations;
    let line = |k: usize| st[k].h - st[k].ground;
    let tube = crate::standard::tube_m(&p.class);
    let roof = |k: usize| roof_gap(st[k].h, st[k].ground, tube);
    let inside = |k: usize| st[k].s >= a0 - 1e-9 && st[k].s <= a1 + 1e-9;

    // The dominant buried run touching the window, scored over its whole
    // extent but seeded inside it.
    let (mut best, mut best_score) = (None, 0.0f64);
    let mut i = 0;
    while i < st.len() {
        if line(i) >= 0.0 {
            i += 1;
            continue;
        }
        let (start, mut score, mut seeded) = (i, 0.0f64, false);
        while i < st.len() && line(i) < 0.0 {
            score -= line(i);
            seeded |= inside(i);
            i += 1;
        }
        if seeded && (best.is_none() || score > best_score) {
            best = Some((start, i - 1));
            best_score = score;
        }
    }
    let (mut f, mut l) = best?;

    // Does the tube fit over the majority of that run?
    let fits = (f..=l).filter(|&k| roof(k) < 0.0).count();
    let cross = |a: usize, b: usize, at: &dyn Fn(&crate::world::Station) -> f64| {
        let (ga, gb) = (at(&st[a]), at(&st[b]));
        if (ga - gb).abs() < f64::EPSILON {
            return st[a].s;
        }
        st[a].s + (st[b].s - st[a].s) * (ga / (ga - gb)).clamp(0.0, 1.0)
    };
    let line_of = |x: &crate::world::Station| x.h - x.ground;
    let roof_of = |x: &crate::world::Station| roof_gap(x.h, x.ground, tube);
    if 2 * fits >= l - f + 1 {
        let lo = if f > 0 { cross(f, f - 1, &line_of) } else { st[f].s };
        let hi = if l + 1 < st.len() { cross(l, l + 1, &line_of) } else { st[l].s };
        return Some((lo, hi));
    }
    // A surface gallery: pull each end back to the tube's own fit.
    while f <= l && roof(f) >= 0.0 {
        f += 1;
    }
    while l > f && roof(l) >= 0.0 {
        l -= 1;
    }
    if f > l || roof(f) >= 0.0 {
        return None;
    }
    let lo = if f > 0 && roof(f - 1) >= 0.0 { cross(f, f - 1, &roof_of) } else { st[f].s };
    let hi = if l + 1 < st.len() && roof(l + 1) >= 0.0 { cross(l, l + 1, &roof_of) } else { st[l].s };
    Some((lo, hi))
}

/// What the grouping had to work with: the pieces' connected components by
/// shared connector, the biggest of them, and the pairs whose interiors cross
/// with no connector between them.
///
/// `linked` says whether *connectivity* alone could be the grouping: it
/// cannot where the largest component is most of the network, because then a
/// viaduct shares a component with the street it flies over. `crossings` says
/// how much work the split has to do.
pub struct Linkage {
    pub components: usize,
    pub largest: usize,
    pub crossings: usize,
}

/// The group of every piece, in [`Network::pieces`]'s order — `plan` then
/// `spans` — and what it took to find it.
///
/// **The group is the surface a piece belongs to.** Two pieces may be unioned
/// into one region exactly when they share a group, and the rule is the
/// plan's: they merge where they *meet* and stay apart where they *cross*.
/// So the group starts as the connected component and is then split wherever
/// a component holds a crossing pair — the structure side of it, with the run
/// of structure pieces it is joined to, moves out on its own.
///
/// That the split is over the *structure* side is not a preference. A
/// crossing is a grade separation, which is to say one of the two is carried
/// over or under the other, and the one that is carried is the one that left
/// the ground. On a junction standing on a deck nothing crosses, and its
/// ground legs and its spans come out as one group — which is the whole
/// point.
///
/// It is a pure function of the pieces, computed once here and carried in
/// the partition's layer so no later step re-derives it.
pub fn groups(plan: &[Polyline2], spans: &[Polyline2]) -> (Groups, Linkage) {
    let pieces: Vec<&Polyline2> = plan.iter().chain(spans.iter()).collect();
    let fam = |p: &Polyline2| crate::width::family(&p.class) as usize;

    // Path-halved union-find over pieces joined at a shared connector.
    let mut parent: Vec<usize> = (0..pieces.len()).collect();
    fn root(parent: &mut [usize], mut x: usize) -> usize {
        while parent[x] != x {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        x
    }
    // **Every vertex, not only the two ends.** Overture does not cut a way
    // at every connector: a way can carry connectors at interior vertices,
    // and another way can end on one of them — on an interior vertex of a
    // bridge piece, say. Unioned by ends alone those fall in different
    // groups, and a junction is drawn as separate sheets that never merge.
    //
    // A shared vertex is a shared connector: two ways that cross without
    // one do not share a point. So the key is every vertex of every piece,
    // and the cost is the vertices rather than the pieces.
    let mut at: HashMap<(usize, (i64, i64)), usize> = HashMap::new();
    for (i, p) in pieces.iter().enumerate() {
        if p.pts.len() < 2 {
            continue;
        }
        for e in p.pts.iter().copied() {
            let key = (fam(p), crate::world::connector(e));
            match at.entry(key) {
                std::collections::hash_map::Entry::Occupied(o) => {
                    let (a, b) = (root(&mut parent, i), root(&mut parent, *o.get()));
                    if a != b {
                        parent[a] = b;
                    }
                }
                std::collections::hash_map::Entry::Vacant(v) => {
                    v.insert(i);
                }
            }
        }
    }
    let mut size: HashMap<usize, usize> = HashMap::new();
    for i in 0..pieces.len() {
        *size.entry(root(&mut parent, i)).or_default() += 1;
    }

    // Interiors that cross with no connector between them, counted once per
    // pair of pieces of one family.
    let ends = |p: &Polyline2| {
        [crate::world::connector(p.pts[0]), crate::world::connector(p.pts[p.pts.len() - 1])]
    };
    let lines: Vec<&[Pt]> = pieces.iter().map(|p| p.pts.as_slice()).collect();
    let mut crossings: HashSet<(usize, usize)> = HashSet::new();
    for c in line::crossings(&lines) {
        let (a, b) = (pieces[c.a], pieces[c.b]);
        // Sharing a connector makes them a junction, whatever their axes do
        // near it.
        if fam(a) == fam(b) && !ends(a).iter().any(|x| ends(b).contains(x)) {
            crossings.insert((c.a, c.b));
        }
    }
    let (components, largest) = (size.len(), size.values().copied().max().unwrap_or(0));

    // The split. A component holding a crossing pair cannot be one surface:
    // its two sides overlap in plan at heights that differ by the whole
    // clearance, and a union would merge them into one sheet at one height.
    let mut group: Vec<usize> = (0..pieces.len()).map(|i| root(&mut parent, i)).collect();
    let mut next = pieces.len();
    // **Sorted, because the group ids are an output.** Walking the set in
    // its hash order would number the groups differently from run to run,
    // and a `HashMap`'s order is reseeded per process — the world is
    // byte-deterministic and every id downstream would jitter with it.
    let mut pairs: Vec<(usize, usize)> = crossings.iter().copied().collect();
    pairs.sort_unstable();
    let mut split: Vec<(usize, usize)> = Vec::new();
    for &(a, b) in &pairs {
        if group[a] != group[b] {
            continue;
        }
        // The carried side leaves: of two pieces that cross, the structure
        // is the one that left the ground. Two ground pieces crossing share
        // no connector and are a data error the `crossing` step counts as
        // `same`; neither moves, and the union draws what the data says.
        let Some(off) = [a, b].into_iter().find(|&i| pieces[i].kind.is_structure()) else {
            continue;
        };
        // With the run it is joined to, so a viaduct does not come apart at
        // its own piece boundaries.
        let mut stack = vec![off];
        let was = group[off];
        split.push((next, was));
        while let Some(i) = stack.pop() {
            if group[i] != was {
                continue;
            }
            group[i] = next;
            for (j, p) in pieces.iter().enumerate() {
                if group[j] != was || !p.kind.is_structure() || p.pts.len() < 2 {
                    continue;
                }
                let (u, v) = (pieces[i], *p);
                if ends(u).iter().any(|e| ends(v).contains(e)) {
                    stack.push(j);
                }
            }
        }
        next += 1;
    }
    // **Which groups may never be one surface.** Every pair whose interiors
    // cross with no connector between them, named by the groups they ended
    // in. The [`crate::sheet`] step needs this and cannot re-derive it: it
    // merges groups that share a region of paving, and that merge is what
    // loses the split — a viaduct's approaches are welded to the street it
    // flies over wherever the two meet at grade somewhere else.
    //
    // Recorded for *every* crossing pair rather than only the ones that
    // needed a split, because a pair already in two components needed none
    // and must still never be joined.
    let rivals: Vec<(usize, usize)> =
        pairs.iter().map(|&(a, b)| (group[a].min(group[b]), group[a].max(group[b]))).collect();
    let linkage = Linkage { components, largest, crossings: crossings.len() };
    (Groups { of: group, rivals, parent: split }, linkage)
}

/// The spans of one way as the model believes them: **the annotation where
/// the geometry cannot see, the geometry everywhere else**.
///
/// - An annotated structure the heights bear out is **trimmed and grown to
///   the run they imply**: the mapper's edge is where a segment was split,
///   and the run's edge is where the road actually leaves the ground.
/// - An annotated structure **no** derived run overlaps is **kept whole**.
///   This is the whole-span guard, and it is the difference between the
///   geometry *contradicting* a tag and the geometry having nothing to say:
///   a 25 m bridge over a stream is below the resolution of a DEM of a few
///   metres, so the heights depart by nothing and no derivation could ever
///   see it. Degraded for want of evidence, such a bridge becomes an
///   earthwork the ground must wall — worse than the bridge it replaces.
///   **Absence of evidence is not evidence of absence**, and a tag is
///   overruled only where the geometry positively says otherwise
///   (`unstacked`, `clamped`, and the trim above).
/// - A run the heights imply that no annotation covers is **added**.
///
/// The result is a partition of `[0, len]`: the arcs are collected, sorted,
/// and one span emitted per gap, so nothing overlaps and nothing is dropped
/// however the three sources overlap.
pub fn spans(way: &Way, profile: &Profile, derived: &[Span], len: f64) -> Vec<Span> {
    let overlaps = |a: &Span, b: &Span| b.a1 > a.a0 && b.a0 < a.a1;
    let mut kept: Vec<Span> = Vec::new();
    for a in way.spans.iter().filter(|s| s.kind != Kind::Ground) {
        let hits: Vec<&Span> = derived.iter().filter(|d| overlaps(a, d)).collect();
        let mut out = if hits.is_empty() {
            *a
        } else {
            // Trimmed *and* grown: the union of what the heights imply here,
            // carrying the annotation's own kind and ordinal.
            let a0 = hits.iter().map(|d| d.a0).fold(f64::INFINITY, f64::min);
            let a1 = hits.iter().map(|d| d.a1).fold(f64::NEG_INFINITY, f64::max);
            Span { a0: a0.max(0.0), a1: a1.min(len), kind: a.kind }
        };
        // **A bore's ends are the run's own** (`bore_bounds`), between the
        // annotation and the proof, within three bounds:
        //
        // - It applies to a **proven** bore only. Over a span the whole-span
        //   guard is holding, it would pull back or degrade a tunnel for a
        //   fit the DEM was never going to show.
        // - It is read from the **annotation's** window, not the trimmed
        //   span: clamped to the trim it could only shrink, and leave a
        //   tube short of its ridge with both portals buried.
        // - It may not grow **past** the annotation. A flat-ground underpass
        //   whose ramps are cut into the ground reads as one line-buried run
        //   that fits the tube over its majority, and a free reach would let
        //   the bore swallow both approaches. Growing past a mapper's edge
        //   needs a *reason*; what grows a span here is the terrain, in
        //   `reference::paint`, which has one.
        if matches!(out.kind, Kind::Tunnel(_)) && !hits.is_empty() {
            let proved = (out.a0, out.a1);
            if let Some((e0, e1)) = bore_bounds(profile, a.a0, a.a1) {
                // The derived run may itself reach past the annotation, so
                // the bounds are ordered before they are clamped between.
                out.a0 = e0.clamp(a.a0.min(proved.0), proved.0);
                out.a1 = e1.clamp(proved.1, a.a1.max(proved.1));
            }
        }
        if out.a1 - out.a0 > MIN_SPAN_M {
            kept.push(out);
        }
    }
    for d in derived {
        if !way.spans.iter().any(|a| a.kind != Kind::Ground && overlaps(a, d)) {
            kept.push(*d);
        }
    }
    partition_of(kept, len)
}

/// A set of possibly-overlapping structure spans, resolved into a partition
/// of `[0, len]` with ground between them. Where two overlap the earlier one
/// holds the overlap, exactly as the reader's `crate::roads::pieces_of` does.
fn partition_of(mut kept: Vec<Span>, len: f64) -> Vec<Span> {
    kept.sort_by(|a, b| a.a0.total_cmp(&b.a0).then(a.a1.total_cmp(&b.a1)));
    let mut out: Vec<Span> = Vec::new();
    let mut at = 0.0f64;
    for s in kept {
        let (a0, a1) = (s.a0.max(at), s.a1.min(len));
        if a0 - at > MIN_SPAN_M {
            out.push(Span { a0: at, a1: a0, kind: Kind::Ground });
        }
        if a1 - a0.max(at) > MIN_SPAN_M {
            out.push(Span { a0: a0.max(at), a1, kind: s.kind });
            at = a1;
        } else {
            at = at.max(a0);
        }
    }
    if len - at > MIN_SPAN_M {
        out.push(Span { a0: at, a1: len, kind: Kind::Ground });
    }
    if out.is_empty() {
        out.push(Span { a0: 0.0, a1: len, kind: Kind::Ground });
    }
    out
}

/// Shortest span worth emitting, in metres: below this the piece quantizes
/// away and its neighbours meet instead.
const MIN_SPAN_M: f64 = 0.25;

/// The runs of one way the **solved heights** imply, whatever the source
/// said (docs/GENERATION.md §4.5).
///
/// One signed gap is the whole signal: past [`DECK_STANDOFF_M`] over the
/// reference is a deck, past [`bore_cover_m`] under it is a bore, and the
/// threshold crossing *is* the abutment or the portal — the mouth sits where
/// the road actually leaves the ground, not where a mapper split the way.
///
/// The gap is measured against the **reference**, not the raw DEM, and that
/// is what makes the two thresholds compose: the closing has already decided
/// which notches a road spans on engineered fill and which are gorges it
/// cannot ([`crate::reference::NOTCH_FILL_MAX_M`]), so a filled culvert reads
/// no departure at all and stays the embankment it is, while a refused gorge
/// reads its full depth.
///
/// Edges are interpolated onto the threshold rather than snapped to a
/// station: a portal placed on the nearest sample is up to `NODE_M` wrong.
pub fn derive(p: &Profile) -> Vec<Span> {
    let st = &p.stations;
    if st.len() < 2 {
        return Vec::new();
    }
    let gap = |k: usize| st[k].h - st[k].reference;
    let cover = bore_cover_m(&p.class);
    let kind_at = |k: usize| {
        if gap(k) > DECK_STANDOFF_M {
            Some(Kind::Bridge(1))
        } else if gap(k) <= -cover {
            Some(Kind::Tunnel(-1))
        } else {
            None
        }
    };
    // Where a run's edge crosses its threshold, between the last station
    // inside it and the first outside.
    let edge = |i: usize, j: usize, level: f64| -> f64 {
        let (gi, gj) = (gap(i) - level, gap(j) - level);
        if (gi - gj).abs() < f64::EPSILON {
            return st[i].s;
        }
        let t = (gi / (gi - gj)).clamp(0.0, 1.0);
        st[i].s + (st[j].s - st[i].s) * t
    };

    let mut runs: Vec<Span> = Vec::new();
    let mut i = 0;
    while i < st.len() {
        let Some(kind) = kind_at(i) else {
            i += 1;
            continue;
        };
        let start = i;
        while i + 1 < st.len() && kind_at(i + 1) == Some(kind) {
            i += 1;
        }
        let level = if kind == Kind::Bridge(1) { DECK_STANDOFF_M } else { -cover };
        let a0 = if start == 0 { st[0].s } else { edge(start, start - 1, level) };
        let a1 = if i + 1 == st.len() { st[i].s } else { edge(i, i + 1, level) };
        if a1 > a0 {
            runs.push(Span { a0, a1, kind });
        }
        i += 1;
    }

    // A sub-`SNAP_RUN_M` gap between two runs of one kind is an edge
    // mismatch, not two structures.
    let mut k = 1;
    while k < runs.len() {
        if runs[k].kind == runs[k - 1].kind && runs[k].a0 - runs[k - 1].a1 < SNAP_RUN_M {
            runs[k - 1].a1 = runs[k].a1;
            runs.remove(k);
        } else {
            k += 1;
        }
    }
    runs.retain(|r| plausible(p, r));
    runs
}

/// Whether a run is long enough to be a structure of its class, or short but
/// over ground that genuinely falls away beneath it.
fn plausible(p: &Profile, r: &Span) -> bool {
    if r.len() >= MIN_STRUCTURE_M {
        return true;
    }
    (1..=3).any(|k| {
        let a = r.a0 + r.len() * k as f64 / 4.0;
        let (h, ground) = at_arc(p, a);
        let depart = match r.kind {
            Kind::Tunnel(_) => ground - h,
            _ => h - ground,
        };
        depart > SHORT_STRUCTURE_DIP_M
    })
}

/// The solved height and the reference at one arc of a profile.
fn at_arc(p: &Profile, a: f64) -> (f64, f64) {
    let st = &p.stations;
    let k = st.partition_point(|s| s.s < a).clamp(1, st.len().saturating_sub(1));
    let (lo, hi) = (&st[k - 1], &st[k]);
    let t = if hi.s > lo.s { ((a - lo.s) / (hi.s - lo.s)).clamp(0.0, 1.0) } else { 0.0 };
    (lo.h + (hi.h - lo.h) * t, lo.reference + (hi.reference - lo.reference) * t)
}

/// Where two partitions of one way name different kinds, in metres.
fn divergence(a: &[Span], b: &[Span]) -> f64 {
    let mut cuts: Vec<f64> = a.iter().chain(b.iter()).flat_map(|s| [s.a0, s.a1]).collect();
    cuts.sort_by(f64::total_cmp);
    cuts.dedup_by(|x, y| (*x - *y).abs() < 1e-9);
    let kind_at = |spans: &[Span], at: f64| {
        spans.iter().find(|s| s.a0 <= at && at < s.a1).map(|s| s.kind)
    };
    let mut m = 0.0;
    for w in cuts.windows(2) {
        let mid = 0.5 * (w[0] + w[1]);
        let (x, y) = (kind_at(a, mid), kind_at(b, mid));
        let structure = |k: Option<Kind>| k.is_some_and(|k| k.is_structure());
        if structure(x) != structure(y) {
            m += w[1] - w[0];
        }
    }
    m
}

#[cfg(test)]
mod tests {
    use crate::world::World;
    use crate::pipeline::tests::{built, upto};
    use crate::step::Step;

    use super::*;

    fn world(net: &str) -> (World, Summary) {
        // Flat ground: nothing departs from it, so the cut is the
        // annotation's own, which is what a flat specimen is about.
        let (w, ran) = built("flat?h=400", net, None, 10.0, &upto(Step::Partition));
        (w, ran.last())
    }

    /// A world built to the partition step, heights and all.
    fn solved(ground: &str, net: &str) -> (World, Summary) {
        let (w, ran) = built(ground, net, None, 5.0, &upto(Step::Partition));
        (w, ran.last())
    }

    /// The one derived run of a single-way specimen.
    fn only_run(w: &World) -> Span {
        let p = &w.solved().unwrap().profiles;
        assert_eq!(p.len(), 1, "one way");
        let runs = derive(&p[0]);
        assert_eq!(runs.len(), 1, "one run, got {runs:?}");
        runs[0]
    }

    /// **A bridge from the terrain.** The source says nothing; the closing
    /// refuses the slot as too deep to fill; the profile chords across it;
    /// and the gap against the reference says deck over the whole crossing.
    #[test]
    fn a_gorge_the_closing_refused_derives_a_deck() {
        let (w, s) = solved("gorge?depth=30&width=40", "net:straight");
        let r = only_run(&w);
        assert_eq!(r.kind, Kind::Bridge(1), "{s}");
        // The rims are 40 m apart and the deck starts where the gap crosses
        // 3 m, a little inside each of them.
        assert!(r.len() > 20.0 && r.len() < 40.0, "deck {:.1} m: {s}", r.len());
        // `found` is 0 because by this step the way *is* annotated — by the terrain
        // itself, in the reference step. What the source said is nothing.
        assert_eq!(s.num("found"), 0.0, "{s}");
        assert_eq!(s.num("spans"), 1.0, "one span piece, cut from the prior: {s}");
    }

    /// **A culvert derives nothing.** The closing filled it, so the road sits
    /// on the reference and the gap is zero — the embankment it is, not a
    /// deck. This is why the departure is measured against the reference
    /// rather than the raw DEM: the two thresholds compose.
    #[test]
    fn a_filled_culvert_derives_nothing() {
        let (w, s) = solved("gorge?depth=4&width=20", "net:straight");
        let p = &w.solved().unwrap().profiles;
        assert!(derive(&p[0]).is_empty(), "{s}");
        assert_eq!(s.num("derived"), 0.0, "{s}");
    }

    /// **Absence of evidence is not evidence of absence.** Flat ground under
    /// a mapped bridge derives nothing — and the span is kept whole anyway.
    ///
    /// This is the whole-span guard, and it is the difference between the
    /// geometry contradicting a tag and the geometry having nothing to say. A
    /// 25 m bridge over a stream is below what a DEM of a few metres
    /// resolves, so no derivation could ever see it.
    #[test]
    fn a_mapped_bridge_that_never_departs_keeps_its_annotation() {
        let (_, s) = solved("flat?h=400", "net:straight?span=0.35,0.65&kind=bridge");
        assert_eq!(s.num("derived"), 0.0, "nothing to see: {s}");
        assert_eq!(s.num("lost"), 0.0, "the span was degraded: {s}");
        assert_eq!(s.num("moved_m"), 0.0, "the partition moved it: {s}");
        assert_eq!(s.num("spans"), 1.0, "the piece is still cut: {s}");
    }

    /// A way with no span of its own comes out as one ground piece, whole.
    #[test]
    fn an_unannotated_way_is_one_ground_piece() {
        let (w, s) = world("net:straight");
        let r = w.network().unwrap();
        assert_eq!(r.ways.len(), 1, "{s}");
        assert_eq!(r.plan.len(), 1, "{s}");
        assert!(r.spans.is_empty(), "{s}");
        assert_eq!(r.plan[0].kind, Kind::Ground);
        assert!((line::length(&r.plan[0].pts) - 200.0).abs() < 1e-9, "{s}");
    }

    /// A mapped span cuts its way into three, and the pieces meet exactly at
    /// the two boundaries.
    #[test]
    fn a_mapped_span_cuts_its_way_into_three() {
        let (w, s) = world("net:straight?span=0.35,0.65&kind=bridge");
        let r = w.network().unwrap();
        assert_eq!(r.ways.len(), 1, "still one way: {s}");
        assert_eq!(r.plan.len(), 2, "{s}");
        assert_eq!(r.spans.len(), 1, "{s}");
        let deck = &r.spans[0];
        assert_eq!(deck.kind, Kind::Bridge(1));
        assert!((line::length(&deck.pts) - 60.0).abs() < 1e-6, "{s}");
        // The abutments are shared vertices, not merely nearby ones.
        let ends: Vec<[f64; 2]> = r.plan.iter().flat_map(|p| [p.pts[0], p.pts[p.pts.len() - 1]]).collect();
        assert!(ends.contains(&deck.pts[0]), "the west abutment is not shared");
        assert!(ends.contains(&deck.pts[deck.pts.len() - 1]), "the east abutment is not shared");
    }

    /// The pieces partition the way: their lengths sum to its own, so the cut
    /// neither loses nor duplicates a metre.
    #[test]
    fn the_pieces_partition_the_way() {
        for net in [
            "net:straight?span=0.35,0.65&kind=bridge",
            "net:straight?span=0.1,0.9&kind=tunnel",
            "net:cross",
            "net:overpass",
            "net:roundabout?r=15&d=5",
        ] {
            let (w, s) = world(net);
            let r = w.network().unwrap();
            for way in &r.ways {
                let cut: f64 = cut_at(way, 0).iter().map(|p| line::length(&p.pts)).sum();
                assert!((cut - way.len()).abs() < 1e-6, "{net}: {cut} vs {} — {s}", way.len());
            }
        }
    }

    /// A span table is a partition of the way, whatever the source said.
    #[test]
    fn the_span_table_is_a_partition() {
        for net in ["net:straight?span=0.35,0.65&kind=bridge", "net:underpass", "net:sidewalk?d=6"] {
            let (w, _) = world(net);
            for way in &w.network().unwrap().ways {
                assert!(!way.spans.is_empty(), "{net}: a way with no spans");
                assert!((way.spans[0].a0).abs() < 1e-9, "{net}: does not start at 0");
                let last = way.spans[way.spans.len() - 1].a1;
                assert!((last - way.len()).abs() < 1e-6, "{net}: ends at {last} of {}", way.len());
                for pair in way.spans.windows(2) {
                    assert!((pair[0].a1 - pair[1].a0).abs() < 1e-9, "{net}: a gap in the table");
                }
            }
        }
    }
}
