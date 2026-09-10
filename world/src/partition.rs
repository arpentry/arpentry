//! Step 3: the partition — where a way is a deck, a bore, or on the ground,
//! and the one place the geometry is cut.
//!
//! The reader hands on whole ways with their spans as an attribute
//! ([`crate::world::Way`]). This step turns that attribute into geometry: one
//! [`Polyline2`] per span, the ground pieces into [`Roads::plan`] — which
//! every surface step builds from — and the rest into [`Roads::spans`].
//!
//! **Today it cuts exactly what the source said**, which is what makes the
//! move safe: the pieces are the ones the reader used to emit, so nothing
//! downstream can tell that the cut has moved. What it buys is that the cut
//! is now a *step*, with one caller and one place to change, and that the
//! way it cuts from is still whole — so the reference has a corridor to
//! condition and the profile has no pin at a mapper's split point.
//!
//! **What it will be.** The spans it cuts are to become a function of the
//! solved heights rather than a copy of the annotation
//! (`data/plans/spans-are-derived-2026-09-09.md`):
//!
//! ```text
//! spans(profile, annotated, licenses, prior) -> Vec<Span>
//! ```
//!
//! computed once, never mutated, and the step then moves after `crossing` so
//! it has heights to read. The checks that specification has to pass are in
//! [`crate::spans`], written before it.

use crate::grade::{DECK_STANDOFF_M, STRUCTURE_MIN_M};
use std::collections::{HashMap, HashSet};

use crate::poly;
use crate::step::Summary;
use crate::structure::TUNNEL_HEIGHT_M;
use crate::world::{Kind, Polyline2, Profile, Profiles, Roads, Solved, Span, Way};

/// Ground cover a bore keeps between its roof and the surface above it, in
/// metres: enough that what rides over it has something to ride on.
pub const TUNNEL_COVER_M: f64 = 0.5;

/// How far a surface must run **below** the reference before a bore is the
/// honest answer rather than a cutting: the road, the tube over it, and the
/// cover over that. Shallower than their sum there is nothing to drive
/// through, and a cutting is what is there.
///
/// The mirror of [`DECK_STANDOFF_M`], and asymmetric with it *for a reason*
/// rather than by calibration: a fill becomes a wall at the tallest face the
/// ground stage will build, while a cut stays a cutting until a tube fits
/// under it.
pub const BORE_COVER_M: f64 = TUNNEL_HEIGHT_M + TUNNEL_COVER_M;

/// A grade sliver shorter than this, between two runs of one kind, is an edge
/// mismatch rather than real at-grade road: one pier's ground reading coming
/// up does not make a viaduct into two viaducts.
pub const SNAP_RUN_M: f64 = 10.0;

/// Shortest run that is a structure on its length alone, in metres.
pub const MIN_STRUCTURE_M: f64 = 40.0;

/// Smallest mid-run departure that makes a shorter run a structure anyway: a
/// 25 m span over a 30 m stream cut is a real bridge, and demoting it blindly
/// dives the road through the gorge it crosses.
pub const SHORT_STRUCTURE_DIP_M: f64 = 3.0;

/// Cuts every way of the world at its span boundaries.
pub fn run(roads: &mut Roads, mut solved: Option<&mut Profiles>) -> Summary {
    let mut ways = std::mem::take(&mut roads.ways);
    let annotated: Vec<Vec<Span>> = ways.iter().map(|w| w.spans.clone()).collect();

    // **The cut follows the geometry.** Every way's span table is replaced by
    // what the solved heights, the annotation and the plan evidence together
    // say it is, and the pieces are cut from that. After this there is one
    // span truth and every consumer reads it.
    let over = crossed(&ways);
    let solving = crate::reference::solving_of(&ways);
    let profiles = solved.as_ref().map(|s| s.profiles.as_slice());
    let derived: Vec<Vec<Span>> =
        profiles.map(|ps| ps.iter().map(derive).collect()).unwrap_or_default();
    let (mut found, mut lost, mut moved) = (0usize, 0usize, 0.0f64);
    let mut witnessed = 0usize;
    let (mut deck_m, mut bore_m) = (0.0f64, 0.0f64);
    let mut runs = 0usize;
    for (n, &w) in solving.iter().enumerate() {
        let Some(d) = derived.get(n) else {
            continue;
        };
        runs += d.len();
        for r in d {
            if matches!(r.kind, Kind::Bridge(_)) {
                deck_m += r.len();
            } else {
                bore_m += r.len();
            }
        }
        let len = ways[w].len();
        witnessed += ways[w]
            .spans
            .iter()
            .filter(|a| {
                a.kind.is_structure()
                    && over.get(w).is_some_and(|x| x.iter().any(|&p| p > a.a0 && p < a.a1))
            })
            .count();
        let out = spans(&ways[w], profiles.and_then(|ps| ps.get(n)), d, len);
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
    // step, the bench and the plan view cut the same thing the surface steps
    // do. Correcting the cut and leaving one consumer on the old table is the
    // failure the server measured and withdrew a release for: the paint and
    // the solids disagreed about where a deck was, and every trimmed bridge
    // gained a joint that could not meet.
    //
    // The per-station verdict is recomputed with them: a station the trim
    // freed is at grade now, and one a grow took in is a deck or a bore by
    // the same rule the solve applied (§4.5).
    if let Some(solved) = solved.as_deref_mut() {
        for (n, &w) in solving.iter().enumerate() {
            let Some(p) = solved.profiles.get_mut(n) else {
                continue;
            };
            p.spans = ways[w].spans.clone();
            for st in p.stations.iter_mut() {
                st.solved = Solved::Grade;
            }
            for (k0, k1, kind) in p.runs() {
                if !kind.is_structure() {
                    continue;
                }
                for st in &mut p.stations[k0..=k1] {
                    st.solved = if st.h - st.ground >= STRUCTURE_MIN_M {
                        Solved::Deck
                    } else if st.ground - st.h >= STRUCTURE_MIN_M {
                        Solved::Bore
                    } else {
                        Solved::Grade
                    };
                }
            }
        }
    }

    let (mut plan, mut spans) = (Vec::new(), Vec::new());
    let (mut ground_m, mut span_m) = (0.0f64, 0.0f64);
    for (i, way) in ways.iter().enumerate() {
        for piece in cut_at(way, i) {
            let m = crate::roads::length(&piece.pts);
            if piece.kind == Kind::Ground {
                ground_m += m;
                plan.push(piece);
            } else {
                span_m += m;
                spans.push(piece);
            }
        }
    }
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
        .with("witnessed", witnessed)
        .with("moved_m", format!("{moved:.0}"));
    roads.ways = ways;
    roads.plan = plan;
    roads.spans = spans;
    summary
}

/// The pieces of one way: its polyline cut at every span boundary, each
/// piece carrying that span's [`Kind`].
///
/// Two neighbouring pieces share the boundary vertex exactly, so the pieces
/// still meet where they used to — the cut moved steps, not places.
pub fn cut(way: &Way) -> Vec<Polyline2> {
    cut_at(way, usize::MAX)
}

/// The same, stamping each piece with `way` as its index into
/// [`crate::world::Roads::ways`] — how a consumer holding a piece finds the
/// profile its way was solved into.
pub fn cut_at(way: &Way, index: usize) -> Vec<Polyline2> {
    let mut out = Vec::with_capacity(way.spans.len());
    for span in &way.spans {
        let pts = between(&way.pts, span.a0, span.a1);
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

/// The signed daylight of the drawn tube at one station: how far a bore's
/// **roof** stands above the reference. Negative is buried — the whole
/// constant-section tube fits under the ground — and the zero crossing is
/// where a portal can be.
///
/// This is a different question from [`derive`]'s, and both are needed. The
/// derivation asks how deep the road runs, and decides whether a bore is what
/// is there at all. This asks whether the *thing that would be drawn* fits,
/// and decides where it stops. A run can be deep enough to be a tunnel and
/// still have tails the tube pokes out of.
fn roof_gap(h: f64, ground: f64) -> f64 {
    h + TUNNEL_HEIGHT_M - ground
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
/// portal transition they have always been and the bounds are the line's own
/// crossings. A run that fits the tube only in a *minority* is not a bore
/// with shallow mouths but a surface gallery with one deep spot: its ends
/// pull back to where the tube fits, and the freed tails are the open cutting
/// they are. Judged by the roof alone every real portal moved into the hill;
/// judged by the line alone a gallery drew its roof proud of the hillside.
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
    // two disagreed wherever the conditioning had moved the surface, and
    // eleven tunnels were pulled back for a fit that was never the one being
    // checked.
    let st = &p.stations;
    let line = |k: usize| st[k].h - st[k].ground;
    let roof = |k: usize| roof_gap(st[k].h, st[k].ground);
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
    let cross = |a: usize, b: usize, at: fn(&crate::world::Station) -> f64| {
        let (ga, gb) = (at(&st[a]), at(&st[b]));
        if (ga - gb).abs() < f64::EPSILON {
            return st[a].s;
        }
        st[a].s + (st[b].s - st[a].s) * (ga / (ga - gb)).clamp(0.0, 1.0)
    };
    let line_of = |x: &crate::world::Station| x.h - x.ground;
    let roof_of = |x: &crate::world::Station| roof_gap(x.h, x.ground);
    if 2 * fits >= l - f + 1 {
        let lo = if f > 0 { cross(f, f - 1, line_of) } else { st[f].s };
        let hi = if l + 1 < st.len() { cross(l, l + 1, line_of) } else { st[l].s };
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
    let lo = if f > 0 && roof(f - 1) >= 0.0 { cross(f, f - 1, roof_of) } else { st[f].s };
    let hi = if l + 1 < st.len() && roof(l + 1) >= 0.0 { cross(l, l + 1, roof_of) } else { st[l].s };
    Some((lo, hi))
}

/// Per way, the arcs at which **another mapped way's line crosses it** — the
/// evidence that a span is a structure whatever the ground says.
///
/// A short bridge over a stream, a footpath or a service road is the case no
/// DEM settles: at 3.29 m a pixel the cut beneath it is not there to be seen,
/// so the heights depart by nothing and the derivation finds no structure.
/// What makes it one is the thing underneath, and the annotation is the only
/// thing in the data that says which of the two is on top. So a span that
/// passes over another alignment keeps its annotation, whatever the
/// derivation says (docs/GENERATION.md §4.5).
///
/// Every way is a witness, not only the ones that solve: a footway crossing
/// under a road bridge is exactly the evidence wanted. A meeting at a shared
/// end is a junction, not a passage, and [`crate::crossing::cross`] reports
/// proper crossings only.
pub fn crossed(ways: &[Way]) -> Vec<Vec<f64>> {
    let mut cells: HashMap<(i32, i32), Vec<(usize, usize)>> = HashMap::new();
    for (i, w) in ways.iter().enumerate() {
        for k in 1..w.pts.len() {
            let (p, q) = (w.pts[k - 1], w.pts[k]);
            let box_ = [p[0].min(q[0]), p[1].min(q[1]), p[0].max(q[0]), p[1].max(q[1])];
            for cell in poly::cells_over(box_, poly::CELL_M) {
                cells.entry(cell).or_default().push((i, k - 1));
            }
        }
    }
    let mut arc: Vec<Vec<f64>> = vec![Vec::new(); ways.len()];
    let mut seen: HashSet<(usize, usize, usize, usize)> = HashSet::new();
    for bucket in cells.values() {
        for x in 0..bucket.len() {
            for y in x + 1..bucket.len() {
                let (mut a, mut b) = (bucket[x], bucket[y]);
                if a.0 == b.0 {
                    continue;
                }
                if a.0 > b.0 {
                    std::mem::swap(&mut a, &mut b);
                }
                if !seen.insert((a.0, a.1, b.0, b.1)) {
                    continue;
                }
                let (u, v) = (&ways[a.0].pts, &ways[b.0].pts);
                let Some(p) = crate::crossing::cross(u[a.1], u[a.1 + 1], v[b.1], v[b.1 + 1]) else {
                    continue;
                };
                for (w, seg) in [(a.0, a.1), (b.0, b.1)] {
                    let at = &ways[w].pts[seg];
                    let mut s = 0.0;
                    for k in 1..=seg {
                        s += (ways[w].pts[k][0] - ways[w].pts[k - 1][0])
                            .hypot(ways[w].pts[k][1] - ways[w].pts[k - 1][1]);
                    }
                    arc[w].push(s + (p[0] - at[0]).hypot(p[1] - at[1]));
                }
            }
        }
    }
    for list in &mut arc {
        list.sort_by(f64::total_cmp);
    }
    arc
}

/// The spans of one way as the model believes them: **the annotation where
/// the geometry cannot see, the geometry everywhere else**
/// (`spans-are-derived-2026-09-09.md` R2/R3).
///
/// - An annotated structure the heights bear out is **trimmed and grown to
///   the run they imply**: the mapper's edge is where a segment was split,
///   and the run's edge is where the road actually leaves the ground.
/// - An annotated structure **no** derived run overlaps is **kept whole**.
///   This is the whole-span guard, and it is the difference between the
///   geometry *contradicting* a tag and the geometry having nothing to say:
///   a 25 m bridge over a stream is below the resolution of a 3.29 m DEM, so
///   the heights depart by nothing and no derivation could ever see it.
///   Degraded for want of evidence, 37 of the loop box's annotated structures
///   became earthworks instead, and the ground stage answered them with
///   19.8 m of fill and 28 902 m² of wall where 24 643 m² had been — worse
///   than the bridges they replaced. **Absence of evidence is not evidence of
///   absence**, and a tag is overruled only where the geometry positively
///   says otherwise (`unstacked`, `clamped`, and the trim above).
/// - A run the heights imply that no annotation covers is **added**.
///
/// The result is a partition of `[0, len]`: the arcs are collected, sorted,
/// and one span emitted per gap, so nothing overlaps and nothing is dropped
/// however the three sources overlap.
pub fn spans(way: &Way, profile: Option<&Profile>, derived: &[Span], len: f64) -> Vec<Span> {
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
        // **A bore's ends are the run's own** ([`bore_bounds`]), between the
        // annotation and the proof.
        //
        // Three bounds, each of which cost a measurement to find. It applies
        // to a **proven** bore only: run over spans the whole-span guard is
        // holding, it pulled back or degraded eleven of the loop box's
        // tunnels for a fit the DEM was never going to show. It is read from
        // the **annotation's** window, not the trimmed span: clamped to the
        // trim it can only shrink, and a 120 m ridge came out with 96 m of
        // tube and both portals buried in the hillside. And it may not grow
        // **past** the annotation: given a free reach instead, a flat-ground
        // underpass whose ramps are cut into the ground read as one
        // line-buried run that fits the tube over its majority, and the bore
        // swallowed both approaches whole — `cut` 6.5 m of honest open
        // cutting became 0. Growing past a mapper's edge needs a *reason*
        // (the server's licence: a crossing the buried tail passes beneath),
        // and this model has none yet. What grows a span here is the terrain,
        // in `reference::paint`, which has one.
        if matches!(out.kind, Kind::Tunnel(_)) && !hits.is_empty() {
            let proved = (out.a0, out.a1);
            if let Some((e0, e1)) = profile.and_then(|p| bore_bounds(p, a.a0, a.a1)) {
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
/// holds the overlap, exactly as the reader's own `pieces_of` does.
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
/// said (docs/GENERATION.md §4.5, and `spans-are-derived-2026-09-09.md` R2).
///
/// One signed gap is the whole signal: past [`DECK_STANDOFF_M`] over the
/// reference is a deck, past [`BORE_COVER_M`] under it is a bore, and the
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
    let kind_at = |k: usize| {
        if gap(k) > DECK_STANDOFF_M {
            Some(Kind::Bridge(1))
        } else if gap(k) <= -BORE_COVER_M {
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
        let level = if kind == Kind::Bridge(1) { DECK_STANDOFF_M } else { -BORE_COVER_M };
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
pub fn divergence(a: &[Span], b: &[Span]) -> f64 {
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

/// The part of `pts` between arc `a0` and arc `a1`, the interior vertices
/// kept and the two ends interpolated onto the polyline.
pub fn between(pts: &[[f64; 2]], a0: f64, a1: f64) -> Vec<[f64; 2]> {
    if pts.len() < 2 || !(a1 > a0) {
        return Vec::new();
    }
    let mut out: Vec<[f64; 2]> = Vec::new();
    let mut at = 0.0f64;
    for pair in pts.windows(2) {
        let (p, q) = (pair[0], pair[1]);
        let len = (q[0] - p[0]).hypot(q[1] - p[1]);
        let next = at + len;
        if next < a0 - 1e-12 {
            at = next;
            continue;
        }
        if at > a1 + 1e-12 {
            break;
        }
        if out.is_empty() {
            let t = if len > 0.0 { ((a0 - at) / len).clamp(0.0, 1.0) } else { 0.0 };
            out.push(lerp(p, q, t));
        }
        if next <= a1 + 1e-12 {
            if out.last() != Some(&q) {
                out.push(q);
            }
        } else {
            let t = if len > 0.0 { ((a1 - at) / len).clamp(0.0, 1.0) } else { 1.0 };
            let end = lerp(p, q, t);
            if out.last() != Some(&end) {
                out.push(end);
            }
            break;
        }
        at = next;
    }
    if out.len() < 2 {
        Vec::new()
    } else {
        out
    }
}

fn lerp(p: [f64; 2], q: [f64; 2], t: f64) -> [f64; 2] {
    [p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t]
}

/// Every piece of `roads`, on the ground or off it — the order the reference
/// and the profile take them in.
pub fn pieces(roads: &Roads) -> impl Iterator<Item = &Polyline2> {
    roads.pieces()
}

#[cfg(test)]
mod tests {
    use crate::world::World;
    use crate::pipeline::tests::{built, plan, upto};
    use crate::step::Step;
    

    
    

    use super::*;

    fn world(net: &str) -> (World, Summary) {
        // No profile: the cut is then the annotation's own, which is what a
        // flat specimen is about.
        let (w, ran) = built("flat?h=400", net, None, 10.0, &plan(Step::Partition));
        (w, ran.last())
    }

    /// A world built to the partition step, heights and all.
    fn solved(ground: &str, net: &str) -> (World, Summary) {
        let (w, ran) = built(ground, net, None, 5.0, &upto(Step::Partition));
        (w, ran.last())
    }

    /// The one derived run of a single-way specimen.
    fn only_run(w: &World) -> Span {
        let p = &w.profile.as_ref().unwrap().profiles;
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
        // `found` is 0 because by now the way *is* annotated — by the terrain
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
        let p = &w.profile.as_ref().unwrap().profiles;
        assert!(derive(&p[0]).is_empty(), "{s}");
        assert_eq!(s.num("derived"), 0.0, "{s}");
    }

    /// **Absence of evidence is not evidence of absence.** Flat ground under
    /// a mapped bridge derives nothing — and the span is kept whole anyway.
    ///
    /// This is the whole-span guard, and it is the difference between the
    /// geometry contradicting a tag and the geometry having nothing to say. A
    /// 25 m bridge over a stream is below what a 3.29 m DEM resolves, so no
    /// derivation could ever see it. Degraded for want of evidence, 37 of the
    /// loop box's annotated structures became earthworks and the ground stage
    /// answered them with 19.8 m of fill and 28 902 m² of wall against
    /// 24 643 — worse than the bridges they replaced.
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
        let r = w.roads.as_ref().unwrap();
        assert_eq!(r.ways.len(), 1, "{s}");
        assert_eq!(r.plan.len(), 1, "{s}");
        assert!(r.spans.is_empty(), "{s}");
        assert_eq!(r.plan[0].kind, Kind::Ground);
        assert!((crate::roads::length(&r.plan[0].pts) - 200.0).abs() < 1e-9, "{s}");
    }

    /// A mapped span cuts its way into three, and the pieces meet exactly at
    /// the two boundaries: the cut moved steps, not places.
    #[test]
    fn a_mapped_span_cuts_its_way_into_three() {
        let (w, s) = world("net:straight?span=0.35,0.65&kind=bridge");
        let r = w.roads.as_ref().unwrap();
        assert_eq!(r.ways.len(), 1, "still one way: {s}");
        assert_eq!(r.plan.len(), 2, "{s}");
        assert_eq!(r.spans.len(), 1, "{s}");
        let deck = &r.spans[0];
        assert_eq!(deck.kind, Kind::Bridge(1));
        assert!((crate::roads::length(&deck.pts) - 60.0).abs() < 1e-6, "{s}");
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
            let r = w.roads.as_ref().unwrap();
            for way in &r.ways {
                let cut: f64 = cut(way).iter().map(|p| crate::roads::length(&p.pts)).sum();
                assert!((cut - way.len()).abs() < 1e-6, "{net}: {cut} vs {} — {s}", way.len());
            }
        }
    }

    /// A span table is a partition of the way, whatever the source said.
    #[test]
    fn the_span_table_is_a_partition() {
        for net in ["net:straight?span=0.35,0.65&kind=bridge", "net:underpass", "net:sidewalk?d=6"] {
            let (w, _) = world(net);
            for way in &w.roads.as_ref().unwrap().ways {
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
