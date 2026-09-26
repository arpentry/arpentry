//! The sheets: which paved regions may merge, and which may not.
//!
//! The paved surface has been built twice. The ground pieces go through
//! `ribbon → surface → kerb → fillet → room` and come out as one unioned,
//! refined region set per family; the span pieces go through `structure`,
//! which sweeps each run on its own and lifts it afterwards by a field of
//! its own. The two meet neither in plan nor in height, and where they hand
//! over the surface steps 0.43 m at p90 and 2.79 m at worst. A junction
//! whose legs stand on a structure is drawn as separate objects with
//! vertical lips between them
//! (`data/plans/one-surface-at-a-junction-2026-09-14.md`).
//!
//! The rule that makes the two into one:
//!
//! > **Regions merge where they meet. They stay apart where they cross.**
//!
//! [`crate::partition::groups`] is that rule as a pure function of the
//! pieces — the connected components of the piece graph by shared
//! connector, split wherever a component holds a crossing pair.
//!
//! **A sheet is not a group, and the measurement is why.** The plan reads
//! "one region per group", and run against the refined asphalt that is not
//! a partition at all: the loop box has **521** carriageway and rail groups
//! and the asphalt they lie in has **39** connected regions. The asphalt is
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
//! **Why the grouping lands here and not in the chain.** The plan's step 4
//! reads "run the refinement chain per group". It is wrong by those steps'
//! own design, and the measurement that says so is that group ids partition
//! *per family* ([`crate::partition::groups`] keys on `(family,
//! connector)`): a sidewalk is never in the group of the road it is the
//! pavement of, and relating the two is the whole job of `kerb`, `fillet`
//! and `room`. Each of the three is deliberately cross-region as well — a
//! kerb rung stops at *any* road rather than its own, the fillet's closing
//! is bucketed by radius and reaches other regions on purpose, and a room's
//! pocket is a hole of asphalt *and* pavement *and* buildings taken
//! together. Run per group, all three would answer differently and worse.
//!
//! So the chain stays family-wide over the ground pieces, exactly as it
//! was, and the grouping is applied once, after it. What a span group
//! forgoes by arriving late is the chain itself — a bridge gets no kerb
//! return — and that is the right answer anyway: a bridge has a parapet.
//!
//! **Only the families that solve** ([`crate::width::Family::solves`]) get
//! sheets. A footbridge has no profile, so a walk span has no field to be
//! lifted by; the pavement is left whole and the structure step keeps
//! sweeping the walk spans until there is a field they can be lifted by.

use std::collections::{BTreeMap, HashMap};

use crate::poly::{self, Shapes};
use crate::step::Summary;
use crate::width::{self, Family};
use crate::world::{Polyline2, Profiles, Roads, Sheet, Sheets, Surface};

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
/// `structure::runs_of` widens every structure run by one station each side
/// — [`crate::grade::NODE_M`], 4 m — so a span's footprint reaches that far
/// back onto the ground piece it hands over to, and the cap adds its own
/// half-disc past that. Twelve metres covers both with room to spare.
///
/// **Every connector on the span, not only its two ends.** Overture does not
/// cut a way at its connectors, so a way's bridge piece can carry junctions
/// in its interior — south of the Clarens railway two service roads join a
/// residential way inside its deck, and measured from the ends alone those
/// two junctions read as paving the deck flew over.
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
pub fn run(roads: &Roads, profiles: &Profiles, paving: &Surface) -> (Sheets, Summary) {
    let crate::partition::Groups { of: group, rivals, parent, .. } =
        crate::partition::groups(&roads.plan, &roads.spans);
    let pieces: Vec<&Polyline2> = roads.pieces().collect();
    // A piece knows the way it was cut from and a profile knows the way it
    // is of ([`Profile::way`]), so the grouping — which is on the pieces —
    // meets the heights — which are on the ways — through one index.
    //
    // The profiles are an argument for that reason: this step writes profile
    // indices into every sheet ([`Sheet::axes`]), so it depends on which ways
    // solved, and a signature that did not say so was the dependency the
    // pipeline exists to make visible.
    let of_way: HashMap<usize, usize> =
        profiles.profiles.iter().enumerate().map(|(i, p)| (p.way, i)).collect();

    // The spans to merge in — and, separately, the mask that says which
    // paving is over one. A round cap welds the span's ribbon to the
    // approach; a square one is the abutment the terrain's hole must end
    // on (`surface::spans_masked`).
    let spans = crate::surface::spans_grouped(roads);
    let masks = crate::surface::spans_masked(roads);

    // Which groups may never be one surface: the pairs whose interiors
    // cross with no connector between them.
    let mut rival: HashMap<usize, std::collections::HashSet<usize>> = HashMap::new();
    for &(a, b) in &rivals {
        rival.entry(a).or_default().insert(b);
        rival.entry(b).or_default().insert(a);
    }

    // Which connectors more than one piece of a family meets: a span may
    // lie over paving at any of them, because that is a junction. The same
    // count `surface` caps on, asked once so the two cannot drift.
    let shared = crate::surface::joints(roads);

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
        for &(child, was) in &parent {
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
        // A region no piece fell in: a fillet's corner left standing on its
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
                // before that, a bore's chord went into the field of the
                // ground around its portal and cut the approach 0.25 m
                // deeper than a cutting goes.
                if i >= roads.plan.len() {
                    // A bore is not a sheet's: see `surface::spans_grouped`.
                    if !matches!(p.kind, crate::world::Kind::Bridge(_)) {
                        continue;
                    }
                    spanning = true;
                    if let Some(&profile) = of_way.get(&p.way) {
                        aside.entry(group[i]).or_default().push((profile, p.a0, p.a1));
                    }
                    touch.entry(group[i]).or_default().extend(p.pts.iter().map(|&e| crate::world::connector(e)));
                    let fam = width::family(&p.class) as usize;
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
                // hairpin passed — same group, so the hairpin counted as
                // the span's own — and the union welded a roadway at 514 m
                // to one at 669 m, two metres apart in plan.
                //
                // **Two questions, and either one keeps a span out.**
                //
                // The first is the model's own: `partition` finds every
                // pair of interiors that cross with no connector between
                // them, and a span may not join a sheet holding a group it
                // crosses that way. Not "does it overlap" — it must overlap,
                // at every junction it opens onto, and on the loop box the
                // threshold that let two service roads join a deck also let
                // a viaduct weld to the road beneath it, 29.5 m down.
                //
                // The second is geometric, and it catches what the first
                // cannot — a way that flies over *itself*. The Territet
                // funicular climbs its own slope at 60 %, its derived deck
                // stands 29.5 m over its own bed, and the two are one group
                // so no crossing pair names them. Measured against the
                // sheet's **raw ribbons** rather than its finished surface,
                // because the surface carries kerb returns the ribbons do
                // not and a sliver of filleted corner is not a fly-over.
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
                    // held the very roads the span flies over — 973 profiles
                    // for the Viaduc de Chillon, the whole connected paving
                    // of the town — and `bench::by_sheet` believes a chord
                    // only where it is no further off in plan than the
                    // nearest ground axis. Over every street passing under
                    // the deck that street's axis was the nearer, and the
                    // asphalt hung 44 m down onto it in a curtain. An
                    // approach is a ground piece sharing a vertex with the
                    // span; a road that passes under shares none.
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
                        .flat_map(|(p, _)| poly::buffer_line(&p.pts, p.width_m + 2.0 * crate::bench::ROOM_REACH_M))
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
            // **The junction is rounded by the fillet step, not here.** A
            // span's paving arrives as a raw ribbon with a round cap, so a
            // junction all of whose legs are decks has a notch between
            // every pair — and this step used to close them itself, with
            // the fillet's own closing over the merged region.
            //
            // It was the wrong place, and the measurement is `on_walk`: an
            // asphalt that grows *after* `room` has finished overlaps the
            // pavement, and nothing downstream can take it back. 25 of the
            // junction model's 30 m² came from here. `fillet` now does the
            // same closing per group with the span ribbons merged in, where
            // `laid_back` re-lays the pavement outside the new kerb and
            // `senior` cuts it, exactly as for every other return.
            //
            // **And the mask is the span ribbons, full stop.** It used to
            // take in the return wedges beside a deck as well, so the
            // earthwork would not read a deck's standoff as an embankment
            // under them. Two things retired that. A return stands at a
            // junction, and a junction on a deck is at its abutment, where
            // the standoff is nothing — so there was little to protect. And
            // `bench::by_sheet` no longer believes this mask for the
            // *height*: a chord answers only where it is no further off
            // than the ground the sheet also holds, which is why a 0.8 m²
            // sliver of it stopped standing the asphalt up in a 2.4 m fin.
            // What a wrong mask can still cost is a vertex's `cut`/`fill`,
            // and that is worth less than another boolean between two
            // polygons that share a boundary.
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
    // **The walk is under the asphalt this step adds, and the number says
    // how much.** Every step before this one leaves the three families'
    // paving mutually disjoint — `surface` cuts the walk to the
    // carriageway and the ballast, `fillet` to its own returns, `room` to
    // both, and `fillet.carriageway ∩ room.pavement` reads 0.00 m². This
    // step then adds asphalt none of them ever saw, and nothing takes the
    // pavement back from under it: a sidewalk inside the road, and 0.12 m
    // over it once the bench has put the kerb's rise on it.
    //
    // It is the invariant of the whole chain, so it is measured where the
    // asphalt is finally complete rather than assumed anywhere earlier.
    // **Zero is the only acceptable reading**; it is 29.9 m² on the
    // junction model today (`one-surface-at-a-junction-2026-09-14.md` §6).
    let on_walk = poly::area(&poly::intersect(
        &poly::union_all(&crate::world::shapes(&sheets)),
        &paving.walk,
    ));
    let summary = Summary::new()
        .with("sheets", sheets.len())
        .with("carriageway", sheets.iter().filter(|s| s.family == Family::Carriageway).count())
        .with("ballast", sheets.iter().filter(|s| s.family == Family::Rail).count())
        .with("spanning", spanning)
        .with("regions", sheets.iter().map(|s| s.shapes.len()).sum::<usize>())
        .with("axes", sheets.iter().map(|s| s.axes.len()).sum::<usize>())
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
        .with("apart", apart)
        .with_m2("sheet_m2", sheets.iter().map(|s| poly::area(&s.shapes)).sum::<f64>())
        .with_m2("on_walk", on_walk);
    (Sheets { sheets }, summary)
}


/// One group's regions out of what [`crate::surface::spans_grouped`]
/// returns.
fn region_of(of: &[(Family, usize, Shapes)], family: Family, group: usize) -> Option<&Shapes> {
    of.iter().find(|(f, g, _)| *f == family && *g == group).map(|(.., s)| s)
}

/// Up to [`SAMPLES`] points along a piece, its own vertices, ends first:
/// an end is where a piece meets its neighbours and is the likeliest to be
/// paved, and the middle is the likeliest to have been bitten by a wall.
fn sample(p: &Polyline2) -> impl Iterator<Item = [f64; 2]> + '_ {
    let n = p.pts.len();
    let step = n.div_ceil(SAMPLES).max(1);
    (0..n).step_by(step).chain(std::iter::once(n.saturating_sub(1))).map(move |k| p.pts[k])
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
        let sheets = w.sheets.as_ref().expect("the sheet step ran");
        let paved = poly::area(&w.fillet.as_ref().unwrap().surface.carriageway);
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
        let sheets = w.sheets.as_ref().unwrap();
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
        let sheets = w.sheets.as_ref().unwrap();
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
    /// The regions do not merge until step 3; the grouping does now.
    #[test]
    fn a_junction_on_a_structure_is_one_sheet() {
        let (w, s) = world("gorge?depth=30&width=40", "net:tee?span=0.3&kind=bridge");
        let sheets = w.sheets.as_ref().unwrap();
        let car: Vec<&Sheet> = sheets.of(Family::Carriageway).collect();
        assert_eq!(car.len(), 1, "the tee is one sheet: {s}");
        assert!(car[0].spanning, "and it stands on a structure: {s}");
    }
}
