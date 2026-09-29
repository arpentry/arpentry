//! The sheets: which paved regions may merge, and which may not.
//!
//! The ground pieces go through `ribbon → surface → kerb → legs → room` and
//! come out as one unioned, refined region set per family; the deck spans
//! are ribbons of their own ([`crate::ribbon::run`]). Paved and
//! lifted apart, the two would meet neither in plan nor in height, and a
//! junction whose legs stand on a structure would be separate objects with
//! vertical lips between them. This step joins them where they are one
//! surface, so a deck and the road running onto it are one polygon lifted
//! by one field, and keeps them apart where they are not.
//!
//! The rule:
//!
//! > **Regions merge where they meet. They stay apart where they cross.**
//!
//! [`crate::partition::groups`] is that rule as a pure function of the
//! pieces — the connected components of the piece graph by shared
//! connector, split wherever a component holds a crossing pair.
//!
//! **A sheet is not a group.** "One region per group", run against the
//! refined asphalt, is not a partition at all: the asphalt has far fewer
//! connected regions than there are carriageway and rail groups. It is
//! coarser than the piece graph, and it has to be — a driveway that shares
//! no connector with the street it runs onto is its own component of the
//! piece graph, and its ribbon still overlaps the street's once both are
//! buffered. They are one polygon. One polygon is one surface, and one
//! surface is lifted by one field or it steps inside itself.
//!
//! So a sheet is a **connected region of paving**, and the groups are how
//! the spans find it: every group whose pieces lie in a region names that
//! region's sheet, and a span joins the sheet its own group's ground
//! pieces are in. Two sheets are never unioned, which is what keeps a
//! viaduct off the street it flies over — and where a viaduct's approaches
//! are welded to that street on the ground, the two are one sheet already
//! and the span has to be kept out of it by the overlap test rather than
//! by the grouping. That residual is counted, never assumed away.
//!
//! **Why the grouping is applied here and not in the chain.** Group ids
//! partition *per family* ([`crate::partition::groups`] keys on `(family,
//! connector)`): a sidewalk is never in the group of the road it is the
//! pavement of, and relating the two is the job of `kerb`, `legs` and
//! `room`. Each of the three is deliberately cross-region as well — a kerb
//! rung stops at *any* road rather than its own, the legs pave a gap
//! between two carriageways that never share a vertex, and a room's pocket
//! is a hole of asphalt *and* pavement *and* buildings taken together. Run
//! per group, all three would answer differently and worse.
//!
//! So the chain runs family-wide over the ground pieces, and the grouping is
//! applied once, after it. What a span forgoes by arriving late is the rest
//! of the chain — a deck gets no kerb ladder and no room — and that is the
//! right answer anyway: a bridge has a parapet.
//!
//! **Only the families that solve** ([`crate::width::Family::solves`]) get
//! sheets. A footbridge has no profile, so a walk span has no field to be
//! lifted by; the pavement is left whole and the structure step paves the
//! walk spans.

use std::collections::{BTreeMap, HashMap};

use crate::poly::{self, Shapes};
use crate::step::Summary;
use crate::width::{self, Family};
use crate::world::{Groups, Network, Polyline2, Profiles, Ribbons, Sheet, Sheets, Surface};

/// How many points along a piece are asked which region they fell in.
///
/// One would do for a piece whose whole length is paved, and most are. The
/// ones that are not are the reason for more: a piece that runs into a
/// building has its middle bitten out by the facade, and a piece cut short
/// by the bbox has its far end outside every region. Five samples at the
/// piece's own vertices cost nothing against an indexed query and leave
/// almost nothing unattributed.
const SAMPLES: usize = 5;

/// How much of a span may lie over the sheet's own ground ribbons away from
/// a connector, in square metres, before it is flying rather than joining.
///
/// Tiny on purpose: away from a junction a span's ribbon should not touch
/// the ground's at all, and a *graze* is what welds two polygons into one —
/// it need not have area. All this absorbs is the polygon kernel's own
/// lattice ([`poly::GRID_M`], 0.1 mm).
const FOREIGN_M2: f64 = 0.01;

/// How far from a connector on a span, in metres, paving under it is a
/// junction rather than something the span flies over.
///
/// A span's ribbon and its approach's are both capped round where they
/// meet, so each laps half its width — at most 4.5 m — over the other, and
/// a leg joining the span laps it by as much again. Twelve metres covers
/// that with room to spare.
///
/// **Every connector on the span, not only its two ends.** Overture does not
/// cut a way at its connectors, so a way's bridge piece can carry junctions
/// in its interior — a service road joining a way inside its deck — and
/// measured from the ends alone such a junction would read as paving the
/// deck flies over.
const ABUTMENT_REACH_M: f64 = 12.0;

/// A disc of [`ABUTMENT_REACH_M`] about each point, unioned: where a span
/// may lie over paving that is already there.
fn discs(at: &[[f64; 2]]) -> Shapes {
    if at.is_empty() {
        return Vec::new();
    }
    let seeds: Shapes = at
        .iter()
        .map(|p| poly::rect(p[0] - 0.05, p[1] - 0.05, p[0] + 0.05, p[1] + 0.05))
        .collect();
    poly::dilate(&poly::union_all(&seeds), ABUTMENT_REACH_M)
}

/// Orders a sheet's axes or chords by profile and then by station, and
/// drops the duplicates that ordering brings together: a piece contributes
/// its station to every group it touches, so a shared one arrives twice.
fn tidy(v: &mut Vec<(usize, f64, f64)>) {
    v.sort_by(|a, b| a.0.cmp(&b.0).then(a.1.total_cmp(&b.1)));
    v.dedup();
}

/// The sheets of the paved surface: one per connected region of paving.
pub fn run(
    roads: &Network,
    profiles: &Profiles,
    paving: &Surface,
    groups: &Groups,
    ribbons: &Ribbons,
) -> (Sheets, Summary) {
    let Groups { of: group, rivals, parent, .. } = groups;
    let pieces: Vec<&Polyline2> = roads.pieces().collect();
    // A piece knows the way it was cut from and a profile knows the way it
    // is of ([`Profile::way`]), so the grouping — which is on the pieces —
    // meets the heights — which are on the ways — through one index.
    //
    // The profiles are an argument for that reason: this step writes profile
    // indices into every sheet ([`Sheet::axes`]), so it depends on which ways
    // solved, and a signature that did not say so would hide a dependency
    // the pipeline exists to make visible.
    let of_way: HashMap<usize, usize> =
        profiles.profiles.iter().enumerate().map(|(i, p)| (p.way, i)).collect();

    // The spans to merge in — and, separately, the mask that says which
    // paving is over one. A round cap welds the span's ribbon to the
    // approach; a square one is the abutment the terrain's hole must end
    // on ([`crate::ribbon::run`]).
    let (spans, masks) = (&ribbons.spans, &ribbons.masks);

    // Which groups may never be one surface: the pairs whose interiors
    // cross with no connector between them.
    let mut rival: HashMap<usize, std::collections::HashSet<usize>> = HashMap::new();
    for &(a, b) in rivals {
        rival.entry(a).or_default().insert(b);
        rival.entry(b).or_default().insert(a);
    }

    // Which connectors more than one piece of a family meets: a span may
    // lie over paving at any of them, because that is a junction. The same
    // count the ribbons are capped by ([`crate::ribbon::joints`]), asked
    // once so the two cannot drift.
    let shared = &ribbons.joints;

    let mut sheets: Vec<Sheet> = Vec::new();
    let (mut orphans, mut merged, mut grouped) = (0usize, 0usize, 0usize);
    let (mut joined, mut apart) = (0usize, 0usize);
    for family in Family::ALL.into_iter().filter(|f| f.solves()) {
        let regions = paving.of(family);
        if regions.is_empty() {
            continue;
        }
        // Which sheet each region is, and which groups name it. A region is
        // claimed by the pieces that lie in it, and a region claimed by two
        // groups is the normal case rather than a smell: they are one
        // polygon of paving, and one polygon is one surface however the
        // piece graph grouped it. `merged` counts how often that happened,
        // which is the number that says a sheet is not a group.
        let index = poly::Indexed::new(regions);
        let mut claim: Vec<Option<usize>> = vec![None; regions.len()];
        let mut union = Union::new();
        // **A structure the split carried away still belongs with its own
        // approach.** The split gives a crossing structure run a group of
        // its own so it is never unioned with what it crosses — but that
        // group holds no ground piece, so nothing here would ever claim it
        // on its own. `parent` is the split's own record of which group a
        // structure's pieces were carried out of, which is exactly the
        // group its approach's ground still has, so joining the two here
        // gives the span back the sheet it lands on without ever touching
        // what it flies over: `rival` still keeps that apart below,
        // computed independently of this union.
        for &(child, was) in parent {
            union.join(child, was);
        }
        let mut seen: std::collections::BTreeSet<usize> = std::collections::BTreeSet::new();
        for (i, p) in pieces.iter().enumerate() {
            if width::family(&p.class) != family {
                continue;
            }
            seen.insert(group[i]);
            if i >= roads.plan.len() {
                continue;
            }
            for r in sample(p).filter_map(|q| index.which(q)) {
                match claim[r] {
                    Some(g) if union.same(g, group[i]) => {}
                    Some(g) => {
                        union.join(g, group[i]);
                        merged += 1;
                    }
                    None => claim[r] = Some(group[i]),
                }
            }
        }
        grouped += seen.len();
        // A region no piece fell in: a kerb return left standing on its
        // own, or asphalt whose every sample landed in a building. It still
        // has to be meshed and lifted, so it joins the sheet of the nearest
        // piece rather than being dropped.
        let mut by: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        for (r, region) in regions.iter().enumerate() {
            let g = match claim[r] {
                Some(g) => union.root(g),
                None => {
                    orphans += 1;
                    let Some(g) = nearest(region, &pieces, &group, &roads.plan, family) else {
                        continue;
                    };
                    union.root(g)
                }
            };
            by.entry(g).or_default().push(r);
        }
        for (g, rs) in by {
            // **Passed through, not re-unioned.** The regions of one family
            // are already disjoint, so a union over them would be the
            // identity — but it would run the polygon kernel over every
            // vertex of the box and move them onto its lattice. A sheet
            // with nothing to merge keeps the geometry the chain gave it.
            let mut shapes: Shapes = rs.iter().map(|&r| regions[r].clone()).collect();
            let mut over: Shapes = Vec::new();
            let mut axes: Vec<(usize, f64, f64)> = Vec::new();
            // The chords of whichever of this sheet's decks joined it.
            let mut mine: Vec<(usize, f64, f64)> = Vec::new();
            // The groups this sheet's paving came from, so a span can be
            // asked whether it flies over any of them.
            let mut held: Vec<usize> = Vec::new();
            // The sheet's ground pieces as raw ribbons, and where each
            // group's spans meet something.
            let mut raw: Shapes = Vec::new();
            let mut hands: BTreeMap<usize, Vec<[f64; 2]>> = BTreeMap::new();
            // The span axes of each group this sheet holds, kept apart so
            // that merging one group's spans brings in that group's chords
            // and not another's.
            let mut aside: BTreeMap<usize, Vec<(usize, f64, f64)>> = BTreeMap::new();
            // Every vertex of each group's spans, and the vertices and axis
            // of every ground piece: which ground a span hands over to is
            // the ground that shares a vertex with it.
            let mut touch: BTreeMap<usize, std::collections::HashSet<(i64, i64)>> = BTreeMap::new();
            let mut grounds: Vec<(&Polyline2, (usize, f64, f64))> = Vec::new();
            let mut spanning = false;
            for (i, p) in pieces.iter().enumerate() {
                if width::family(&p.class) != family || union.root(group[i]) != g {
                    continue;
                }
                if !held.contains(&group[i]) {
                    held.push(group[i]);
                }
                // **An axis is in a sheet's field exactly when its surface
                // is in that sheet.** A span's paving is the sheet's only
                // once the span's region has joined it — and included
                // before that, a bore's chord would go into the field of the
                // ground around its portal and cut the approach deeper than
                // a cutting goes.
                if i >= roads.plan.len() {
                    // A bore is not a sheet's: the ribbon step unions decks only.
                    if !p.kind.is_deck() {
                        continue;
                    }
                    spanning = true;
                    if let Some(&profile) = of_way.get(&p.way) {
                        aside.entry(group[i]).or_default().push((profile, p.a0, p.a1));
                    }
                    touch.entry(group[i]).or_default().extend(p.pts.iter().map(|&e| crate::world::connector(e)));
                    let fam = width::family(&p.class);
                    let ends = hands.entry(group[i]).or_default();
                    for e in p.pts.iter().copied() {
                        if shared.get(&(fam, crate::world::connector(e))).copied().unwrap_or(0) > 1 {
                            ends.push(e);
                        }
                    }
                    continue;
                }
                raw.extend(poly::buffer_line(&p.pts, p.width_m));
                if let Some(&profile) = of_way.get(&p.way) {
                    axes.push((profile, p.a0, p.a1));
                    grounds.push((p, (profile, p.a0, p.a1)));
                }
            }
            // **The spans join the sheet their own group's ground is in.**
            // That is the whole point of the step: a junction standing on a
            // structure comes out as one polygon, with no boundary between
            // the deck and the approaches and so no seam to step across.
            //
            // Unless the span would land on paving that is not its own.
            // A sheet is a connected region of paving, so a viaduct whose
            // approaches are welded to the street it flies over shares a
            // sheet with that street, and the grouping cannot keep them
            // apart — the overlap has to. What a span may legitimately
            // cover is its own group's ground ribbons, which is the
            // abutment margin `structure` adds to carry a deck out to its
            // rims; anything past that is a grade separation and gets a
            // sheet of its own, seam and all, counted in `apart`.
            // Unioned once for every span below, and only if there is one.
            let raw = if aside.is_empty() { Vec::new() } else { poly::union_all(&raw) };
            for (gg, chords) in &aside {
                let Some(span) = region_of(&spans, family, *gg) else {
                    continue;
                };
                // **What a span may legitimately cover is its own
                // abutments.** Tested against its whole group's ground
                // instead, a mountain road that tunnels under its own
                // hairpin passes — same group, so the hairpin counts as the
                // span's own — and the union welds two roadways a hundred
                // metres apart in height that lie a few metres apart in
                // plan.
                //
                // **Two questions, and either one keeps a span out.**
                //
                // The first is the model's own: `partition` finds every
                // pair of interiors that cross with no connector between
                // them, and a span may not join a sheet holding a group it
                // crosses that way. Not "does it overlap" — it must overlap,
                // at every junction it opens onto, and an area threshold
                // loose enough to let two service roads join a deck also
                // lets a viaduct weld to the road beneath it.
                //
                // The second is geometric, and it catches what the first
                // cannot — a way that flies over *itself*: a funicular
                // climbing its own slope, whose derived deck stands metres
                // over its own bed, is one group, so no crossing pair names
                // the two. Measured against the sheet's **raw ribbons**
                // rather than its finished surface, because the surface
                // carries kerb returns the ribbons do not and a sliver of a
                // return is not a fly-over.
                let flies = held.iter().any(|h| rival.get(gg).is_some_and(|r| r.contains(h)));
                let ends = hands.get(gg).cloned().unwrap_or_default();
                let lies = poly::difference(&poly::intersect(span, &raw), &discs(&ends));
                if flies || poly::area(&lies) > FOREIGN_M2 {
                    // Its own sheet, and a seam at its abutment. It still
                    // takes its approaches' ground axes as well as its
                    // chords, so the two agree where they part (invariant
                    // I2) even though the polygons do not meet.
                    //
                    // **Its approaches', not the sheet's.** Given every
                    // ground axis of the sheet it was kept out of, the field
                    // would hold the very roads the span flies over, and
                    // `copies::Fields` believes a chord only where it is no
                    // further off in plan than the nearest ground axis: over
                    // every street passing under the deck that street's axis
                    // is the nearer, and the asphalt would hang down onto it
                    // in a curtain. An approach is a ground piece sharing a
                    // vertex with the span; a road that passes under shares
                    // none.
                    apart += 1;
                    let mut own = chords.clone();
                    tidy(&mut own);
                    let near = touch.get(gg);
                    let approaches: Vec<&(&Polyline2, (usize, f64, f64))> = grounds
                        .iter()
                        .filter(|(p, _)| {
                            near.is_some_and(|t| p.pts.iter().any(|&e| t.contains(&crate::world::connector(e))))
                        })
                        .collect();
                    let mut approach: Vec<(usize, f64, f64)> = approaches.iter().map(|&&(_, a)| a).collect();
                    tidy(&mut approach);
                    // **The deck stops where its approach's paving begins.**
                    // Its ribbon is capped round to weld to the approach, so
                    // its end lies over the approach's own ground paving —
                    // and a second surface there, at the same height, is a
                    // pair of coplanar sheets that flicker. The approach is on
                    // the ground and keeps it; the deck gives it up, along the
                    // approach's own boundary, so the two meet on one line.
                    // Only near the approach: the street the deck flies over
                    // is in this sheet too, and the deck over it stays.
                    let reach: Shapes = approaches
                        .iter()
                        .flat_map(|(p, _)| poly::buffer_line(&p.pts, p.width_m + 2.0 * crate::standard::ROOM_REACH_M))
                        .collect();
                    let landing = poly::intersect(&shapes, &poly::union_all(&reach));
                    let deck = if landing.is_empty() { span.clone() } else { poly::difference(span, &landing) };
                    sheets.push(Sheet {
                        family,
                        group: *gg,
                        shapes: deck,
                        spans: span.clone(),
                        axes: approach,
                        chords: own,
                        spanning: true,
                    });
                    continue;
                }
                joined += 1;
                let mut parts = shapes;
                parts.extend(span.iter().cloned());
                shapes = poly::union_all(&parts);
                over.extend(region_of(&masks, family, *gg).unwrap_or(span).iter().cloned());
                mine.extend(chords.iter().copied());
            }
            tidy(&mut axes);
            // **The junction is rounded upstream, not here.** A span's
            // paving arrives as a raw ribbon with a round cap, so a junction
            // all of whose legs are decks has a notch between every pair.
            // The `legs` step builds that junction from its spans, and the
            // pavement is laid back outside its kerb and cut by `senior` as
            // for every other return. Closed here instead, the asphalt would
            // grow *after* `room` has finished and overlap the pavement,
            // which nothing downstream can take back (`on_walk`).
            //
            // **And the mask is the span ribbons, nothing more.** The return
            // wedges beside a deck are not in it: a return stands at a
            // junction, and a junction on a deck is at its abutment, where
            // the standoff is nothing. Nor does `copies::Fields` believe the
            // mask for the *height* — a chord answers only where it is no
            // further off than the ground the sheet also holds, so a sliver
            // of mask cannot stand the asphalt up in a fin. What a wrong mask
            // can cost is a vertex's `cut`/`fill`, and that is worth less
            // than another boolean between two polygons that share a
            // boundary.
            let spans_over = poly::union_all(&over);
            tidy(&mut mine);
            sheets.push(Sheet {
                family,
                group: g,
                shapes,
                spans: spans_over,
                axes,
                chords: mine,
                spanning,
            });
        }
    }

    let spanning = sheets.iter().filter(|s| s.spanning).count();
    let summary = Summary::new()
        .with("spanning", spanning)
        // `groups` against `sheets` is how much coarser the paving is than
        // the piece graph, and `merged` is how many joins it took to get
        // there. Neither is a defect: both are the measurement that says a
        // sheet is a region of paving and not a group. `orphan` is a region
        // no piece fell in at all, and that one should stay near zero.
        .with("groups", grouped)
        .with("orphan", orphans)
        .with("merged", merged)
        // `joined` is a group whose spans became part of the paving they
        // run onto — the whole point of the step — and `apart` one whose
        // spans would have landed on somebody else's paving and kept a
        // sheet, and a seam, of their own. `apart` is the residual this
        // step does not fix, and it is meant to stay small.
        .with("joined", joined)
        .with("apart", apart);
    (Sheets { sheets }, summary)
}

/// The sheets' size, and the invariant of the whole plan chain.
///
/// **The walk is under the asphalt this step adds, and `on_walk` says how
/// much.** Every step before this one leaves the three families' paving
/// mutually disjoint — `surface` cuts the walk to the carriageway and the
/// ballast, `legs` to its own junctions, `room` to both. This step then adds
/// asphalt none of them ever saw, and nothing takes the pavement back from
/// under it: a sidewalk inside the road, and 0.12 m over it once the lift
/// has put the kerb's rise on it. It is measured where the asphalt is
/// finally complete rather than assumed anywhere earlier, and **zero is the
/// only acceptable reading**.
pub fn check(sheets: &Sheets, paving: &Surface) -> Summary {
    let sheets = &sheets.sheets;
    let on_walk = poly::area(&poly::intersect(&poly::union_all(&crate::world::shapes(sheets)), &paving.walk));
    Summary::new()
        .with("sheets", sheets.len())
        .with("carriageway", sheets.iter().filter(|s| s.family == Family::Carriageway).count())
        .with("ballast", sheets.iter().filter(|s| s.family == Family::Rail).count())
        .with("regions", sheets.iter().map(|s| s.shapes.len()).sum::<usize>())
        .with("axes", sheets.iter().map(|s| s.axes.len()).sum::<usize>())
        .with_m2("sheet_m2", sheets.iter().map(|s| poly::area(&s.shapes)).sum::<f64>())
        .with_m2("on_walk", on_walk)
}

/// One group's regions out of what [`crate::ribbon::run`]
/// returns.
fn region_of(of: &[(Family, usize, Shapes)], family: Family, group: usize) -> Option<&Shapes> {
    of.iter().find(|(f, g, _)| *f == family && *g == group).map(|(.., s)| s)
}

/// Up to [`SAMPLES`] of a piece's own vertices, ends first — an end is where
/// a piece meets its neighbours and is the likeliest to be paved, and the
/// middle is the likeliest to have been bitten by a wall — each with the
/// midpoint of the segment after it.
///
/// **A vertex alone can lie on its own region's boundary**, and a parity
/// test there is a coin toss. A piece with a square end stops exactly where
/// its ribbon does: `net:underpass`'s approach is two vertices, one on the
/// butt end at the rect's edge and one on the butt end at the portal, and
/// which side of each the lattice rounds decides whether the region has any
/// piece in it at all. A midpoint is on the axis half a segment from either
/// end, so strictly inside the ribbon unless a wall has bitten it there.
fn sample(p: &Polyline2) -> impl Iterator<Item = [f64; 2]> + '_ {
    let n = p.pts.len();
    let step = n.div_ceil(SAMPLES).max(1);
    let mid = move |k: usize| {
        p.pts.get(k + 1).map(|b| [(p.pts[k][0] + b[0]) / 2.0, (p.pts[k][1] + b[1]) / 2.0])
    };
    (0..n)
        .step_by(step)
        .chain(std::iter::once(n.saturating_sub(1)))
        .flat_map(move |k| std::iter::once(p.pts[k]).chain(mid(k)))
}

/// The group of the piece nearest to `region`, by the distance from the
/// region's first vertex to a piece's own vertices. Approximate on purpose:
/// this decides only where a stray corner of asphalt is lifted from, and
/// the answer is wanted for a handful of regions out of hundreds.
fn nearest(
    region: &[Vec<[f64; 2]>],
    pieces: &[&Polyline2],
    group: &[usize],
    plan: &[Polyline2],
    family: Family,
) -> Option<usize> {
    let at = *region.first()?.first()?;
    let mut best: Option<(f64, usize)> = None;
    for (i, p) in pieces.iter().enumerate() {
        if i >= plan.len() || width::family(&p.class) != family {
            continue;
        }
        for q in &p.pts {
            let d = (q[0] - at[0]).hypot(q[1] - at[1]);
            if best.is_none_or(|(b, _)| d < b) {
                best = Some((d, group[i]));
            }
        }
    }
    best.map(|(_, g)| g)
}

/// Path-halved union-find over group ids, for the groups one region makes
/// into one. Sparse, because the ids are the partition's and most of them
/// never appear here.
#[derive(Default)]
struct Union {
    parent: HashMap<usize, usize>,
}

impl Union {
    fn new() -> Union {
        Union::default()
    }

    fn root(&mut self, mut x: usize) -> usize {
        while let Some(&p) = self.parent.get(&x) {
            if p == x {
                break;
            }
            let up = self.parent.get(&p).copied().unwrap_or(p);
            self.parent.insert(x, up);
            x = up;
        }
        x
    }

    fn same(&mut self, a: usize, b: usize) -> bool {
        self.root(a) == self.root(b)
    }

    fn join(&mut self, a: usize, b: usize) {
        let (a, b) = (self.root(a), self.root(b));
        if a != b {
            // The lower id wins, so the groups a merge produces are a
            // function of the ids and not of the order they were met in.
            let (lo, hi) = (a.min(b), a.max(b));
            self.parent.insert(hi, lo);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::tests::{built, upto};
    use crate::step::Step;
    use crate::world::World;

    fn world(ground: &str, net: &str) -> (World, Summary) {
        let (w, ran) = built(ground, net, None, 5.0, &upto(Step::Sheet));
        (w, ran.last())
    }

    /// The sheets hold every paved region of a family and no region twice:
    /// whatever the grouping does, the surface is neither lost nor doubled.
    #[test]
    fn the_sheets_are_the_paving_partitioned() {
        let (w, s) = world("flat", "net:cross?len=200");
        let sheets = w.sheet.as_ref().expect("the sheet step ran");
        let paved = poly::area(&w.room.as_ref().unwrap().surface.carriageway);
        let sheeted: f64 =
            sheets.of(Family::Carriageway).map(|sh| poly::area(&sh.shapes)).sum();
        assert!((paved - sheeted).abs() < 1e-6, "{paved} vs {sheeted}: {s}");
        assert_eq!(s.num("orphan"), 0.0, "{s}");
        assert_eq!(s.num("merged"), 0.0, "{s}");
    }

    /// A cross is one group, so its four legs are one sheet — the same
    /// answer `surface::run` gives, arrived at by the grouping.
    #[test]
    fn a_cross_is_one_sheet() {
        let (w, s) = world("flat", "net:cross?len=200");
        let sheets = w.sheet.as_ref().unwrap();
        let car: Vec<&Sheet> = sheets.of(Family::Carriageway).collect();
        assert_eq!(car.len(), 1, "{s}");
        assert_eq!(car[0].shapes.len(), 1, "one region: {s}");
        assert!(!car[0].spanning, "nothing here leaves the ground: {s}");
        assert_eq!(car[0].axes.len(), 4, "four legs at the origin: {s}");
    }

    /// Two carriageways that share no connector are two sheets, whatever
    /// their plan positions do. This is the rule the whole step exists for:
    /// an overpass and the road beneath it may not be merged.
    #[test]
    fn an_overpass_and_the_road_under_it_are_two_sheets() {
        let (w, s) = world("flat", "net:overpass?span=0.35,0.65&level=1&len=200");
        let sheets = w.sheet.as_ref().unwrap();
        let car: Vec<&Sheet> = sheets.of(Family::Carriageway).collect();
        assert!(car.len() >= 2, "the two must not be one sheet: {s}");
        for a in 0..car.len() {
            for b in a + 1..car.len() {
                assert_ne!(car[a].group, car[b].group, "{s}");
            }
        }
    }

    /// A junction standing on a structure is one group, and every one of
    /// its pieces — the ground legs and the spans — names the same sheet.
    #[test]
    fn a_junction_on_a_structure_is_one_sheet() {
        let (w, s) = world("gorge?depth=30&width=40", "net:tee?span=0.3&kind=bridge");
        let sheets = w.sheet.as_ref().unwrap();
        let car: Vec<&Sheet> = sheets.of(Family::Carriageway).collect();
        assert_eq!(car.len(), 1, "the tee is one sheet: {s}");
        assert!(car[0].spanning, "and it stands on a structure: {s}");
    }
}
