//! The structures: decks and bores, the consequence of the profile.
//!
//! The profile solved a height along every carriageway axis and said, at
//! every station, whether that height stands off the ground ([`Solved::Deck`]),
//! runs under it ([`Solved::Bore`]) or lies on it. This step builds what
//! those answers imply and nothing else: **a structure is never built from
//! an annotation** (docs/GENERATION.md §4.5). A mapped bridge whose chord
//! never left the ground got no deck in the profile and gets no solid here.
//!
//! **It does not pave a road's or a railway's span.** The `sheet` step does,
//! in one polygon with the ground the span runs onto, so the handover at an
//! abutment is a place inside one surface rather than a boundary between
//! two. What this step still paves is a bore's floor — a sheet's field
//! reaches far enough in plan that a hairpin over its own tunnel read the
//! road above — and the walk span, which has no profile to be lifted by; and
//! it builds the solid under or over everything else.
//!
//! **A deck is a continuous surface in the air, nothing more.** Over a
//! deck run this step adds a soffit [`DECK_THICKNESS_M`] straight below the
//! roadway, its two sides and its two end faces — the same slab, full
//! length, whether the run is landing at its abutments or flying over a
//! gorge. There is no abutment block and no pier: the surface steps read
//! the ground pieces only, so the earthwork under a shallow deck is not
//! this step's to build, and a slab drawn straight through where it runs
//! close to the ground is a simplification this step accepts rather than a
//! defect it hides. Over a bore run the soffit becomes a crown
//! [`TUNNEL_HEIGHT_M`] above the roadway instead, open at the portals. The
//! roadway is the deck's top and the bore's floor, once, so no two
//! surfaces of this step are coplanar.
//!
//! **The abutment is continuous by construction.** A span's end height is
//! the anchor the profile step took from the ground at the connector, and
//! the ground pieces meeting there took the same one, so the deck lands on
//! the bench exactly. `abutment` measures it anyway.
//!
//! **A pedestrian span is fitted, not solved** (§4.2). Footways, paths and
//! steps never solve a profile, and most of the box's spans are theirs: 31
//! footways below the ground and 29 above it, against 14 motorway bridges.
//! Each is given a chord between the ground at its own two ends — no
//! ceiling, no deviation box — and then reads as a deck or a bore by the
//! same rule as everything else. *A path cannot descend a cliff* is named
//! in the plan and waits for a site.
//!
//! **Unless the road is already carrying it.** A separated sidewalk over a
//! road bridge is mapped as its own bridge — 22.7 % of the extract's
//! footbridges are drawn that way — and it is not a second structure but
//! the same one. A walk span every station of which lies within the room's
//! reach of a road's deck is *carried*: its height is that deck's plus the
//! kerb's rise, exactly as a pavement's is over the ground, and it builds
//! no solid of its own. `carried` counts them.
//!
//! **The portal is the bench's.** The partition gives the stretch between a
//! bore's line crossing and its roof's fit back to the ground, the bench cuts
//! it as a cutting and draws the headwall ([`crate::bench::Mouth`]), and this
//! step reaches the tube [`PORTAL_M`] out over it. `covered` measures tunnel
//! roadway still under the terrain with no tube over it, and `clear` the
//! least a slab clears the ground: it goes negative where a deck runs close
//! to its abutment, since there is no seat under a slab any more.

use std::collections::HashMap;

use crate::bench::{Field, KERB_RISE_M, ROOM_REACH_M};
use crate::grade::{NODE_M, STRUCTURE_MIN_M};
use crate::poly::{self, Pt, Shapes};
use crate::profile::densify;
use crate::step::Summary;
use crate::terrain::height_at;
use crate::width::{self, Family};
use crate::world::{connector, Kind, Profiles, Roads, Sheets, Solved, Station, Structure, Terrain, Tri};

/// How thick a road deck is, in metres
/// (`data/plans/surface-leaves-the-plane-2026-09-08.md` §5): the slab, its
/// beams and its bearings, as one number.
pub const DECK_THICKNESS_M: f64 = 1.5;

/// How far a bench-mesh lookup may disagree with this run's own chord and
/// still be believed, in metres. Wide enough to absorb a real kerb/pavement
/// seam (centimetres) or the lattice's own numerical noise; narrow enough
/// that no genuine grade separation (metres, by [`crate::grade::STRUCTURE_MIN_M`]
/// at the very least) is ever mistaken for one.
const TOP_AGREE_M: f64 = 1.0;

/// How thick a footbridge is. The plan carries one deck thickness, and it
/// is a road bridge's: 1.5 m of beam under a 3 m footway is not a
/// footbridge but a wall with a path on it, and most of the box's spans
/// are pedestrian. A prior of this step's own, named here.
pub const WALK_DECK_M: f64 = 0.4;

/// How high a bore is inside, in metres, from the roadway to the crown.
pub const TUNNEL_HEIGHT_M: f64 = 5.0;

/// The same for a way a person walks through: a subway, a covered stair, a
/// passage under a building. The mirror of [`WALK_DECK_M`], and needed for
/// the same reason — most of the loop box's tunnel spans are footways, steps
/// and paths, and five metres of tube is not what any of them is.
pub const WALK_TUNNEL_M: f64 = 2.5;

/// How high a standard-gauge railway's bore is inside, in metres, from the
/// track to the crown: the loading gauge and the overhead line over it,
/// which a road's [`TUNNEL_HEIGHT_M`] does not hold. The same headroom
/// [`crate::crossing::RAIL_CLEARANCE_M`] asks of a road over the rails, less
/// the margin a road bridge's soffit keeps.
pub const RAIL_TUNNEL_M: f64 = 6.0;

/// The same for metre gauge and a funicular: a smaller car under a lower
/// wire, as high inside as a road tunnel.
pub const NARROW_RAIL_TUNNEL_M: f64 = 5.0;

/// How far a railway's structure reaches past its track zone on each side,
/// in metres: the edge beam, the cable trough and the walkway a real deck
/// carries, and the same clearance inside a bore. The server's
/// `STRUCTURE_SHOULDER_M`, which its rail comments say the structure sweep
/// adds back and its code never did. Without it a metre-gauge viaduct on a
/// hillside is 2.6 m wide and 1.5 m deep, and reads as a wall rather than a
/// bridge. The track bed over the structure is swept to the same width: on
/// a deck the ballast runs to the parapet.
pub const RAIL_SHOULDER_M: f64 = 1.0;

/// How high a bore is inside for a way of `class`, in metres: a walk's
/// passage, a railway's tube, or a road tunnel. One answer for the
/// structure step, which draws the tube, the partition, which decides
/// where a tube fits, and the crossing, which clears one.
pub fn tube_m(class: &str) -> f64 {
    match (width::family(class), class) {
        (Family::Walk, _) => WALK_TUNNEL_M,
        (Family::Rail, "narrow_gauge" | "funicular") => NARROW_RAIL_TUNNEL_M,
        (Family::Rail, _) => RAIL_TUNNEL_M,
        (Family::Carriageway, _) => TUNNEL_HEIGHT_M,
    }
}

/// Whether a tunnel run of a way of `class`, over `stations`, is a
/// **gallery**: the road goes under the ground somewhere, by more than
/// [`STRUCTURE_MIN_M`], and its tube fits under it nowhere.
///
/// There is no hill to bore through, and still the source says the road is
/// covered: a gallery against a slope, a covered cutting, a road under a
/// deck or a building the terrain model does not have. Drawn as a bore it
/// drew nothing — the tube never fitted, so no tube, and the terrain lay on
/// the road end to end: a road that vanished into the ground with no
/// entrance and no exit (`mouths`). Drawn as a gallery it is what it is: the
/// ground is opened over it ([`crate::bench`]) and the tube stands in the
/// trench, its roof out in the open where the ground is lower than it.
///
/// One rule for the bench, which opens the ground, and for this step, which
/// draws the tube — over the same stations, the run and its abutments.
pub fn is_gallery(class: &str, stations: &[Station]) -> bool {
    let tube = tube_m(class);
    stations.iter().any(|st| st.ground - st.h > STRUCTURE_MIN_M)
        && stations.iter().all(|st| st.ground - st.h - tube < 0.0)
}

/// The station ranges of `p`'s galleries, abutments included: the tunnel
/// runs [`is_gallery`] says are galleries. A walk's spans are not solved and
/// are not here.
pub fn gallery_runs(p: &crate::world::Profile) -> Vec<(usize, usize)> {
    let last = p.stations.len().saturating_sub(1);
    p.runs()
        .into_iter()
        .filter(|r| matches!(r.2, Kind::Tunnel(_)))
        .map(|(k0, k1, _)| (k0.saturating_sub(1), (k1 + 1).min(last)))
        .filter(|&(a, b)| b > a && is_gallery(&p.class, &p.stations[a..=b]))
        .collect()
}

/// Half the width of the structure a way of `class` and `width_m` stands
/// on or runs through: the way's own for a road or a walk, the track zone
/// and [`RAIL_SHOULDER_M`] for a railway.
pub fn half_width_m(class: &str, width_m: f64) -> f64 {
    width_m / 2.0 + if width::family(class) == Family::Rail { RAIL_SHOULDER_M } else { 0.0 }
}

/// How far a tube reaches out of the hill past its portal, in metres: over
/// the cutting in front of it, so the edge where the terrain was cut lies
/// inside the tube rather than across its mouth, and the portal reads as a
/// short hood under its headwall.
pub const PORTAL_M: f64 = 2.0;

/// One span, with a height at every station: a carriageway piece as the
/// profile solved it, or a pedestrian piece as this step fitted it.
struct Span {
    class: String,
    width_m: f64,
    mapped: Kind,
    /// The arc range the span itself covers along its way. The stations
    /// reach one further each side, to its abutments.
    a0: f64,
    a1: f64,
    /// Whether each end meets more of its way — ground, or another span —
    /// rather than being where the way itself ends (the bbox cutting it, a
    /// span mapped to the way's end). Only such an end has an abutment, and
    /// only it is a portal.
    open_ends: [bool; 2],
    stations: Vec<Station>,
    /// Fitted here rather than solved in the profile step: a draped class.
    fitted: bool,
    /// Carried on a road's own deck: paved, but no structure of its own.
    carried: bool,
}

impl Span {
    fn thickness(&self) -> f64 {
        if width::family(&self.class) == Family::Walk {
            WALK_DECK_M
        } else {
            DECK_THICKNESS_M
        }
    }

    /// How high the bore is inside. A subway under a street is not a road
    /// tunnel: a footway given the full [`TUNNEL_HEIGHT_M`] draws five metres
    /// of tube for a passage a person walks through, and on the loop box most
    /// of the tunnel spans are footways, steps and paths. Nor is a railway's
    /// bore: a standard-gauge one has a wire over the train ([`tube_m`]).
    fn height(&self) -> f64 {
        tube_m(&self.class)
    }

    /// Half the width of the structure: [`half_width_m`].
    fn half_w(&self) -> f64 {
        half_width_m(&self.class, self.width_m)
    }
}

/// What the structures came to.
#[derive(Debug, Default, Clone, Copy)]
pub struct Stats {
    pub spans: usize,
    pub fitted: usize,
    /// Walk spans a road's own deck already carries.
    pub carried: usize,
    pub decks: usize,
    pub bores: usize,
    /// The plan area, in square metres, of the spans whose surface the
    /// [`crate::sheet`] step now paves — the roads' and the railways'.
    /// This step collects the footprints for `plan` and measures them; it
    /// meshes none of them.
    ///
    /// The walk's share is the exception and is still a sweep, so for a
    /// footbridge this is area this step really did lay.
    pub span_m2: f64,
    /// What the span regions' own meshing came to: the area, in square
    /// metres, by which their triangles disagree with the regions they were
    /// cut from, and the regions the ear clipper misread. A union is only
    /// worth having if it meshes honestly — double-covered triangles are
    /// indistinguishable from an un-unioned overlap in anything drawn.
    pub span_lost_m2: f64,
    pub span_lossy: usize,
    /// The least a deck's soffit clears the ground under it, between its
    /// abutments: a plain measurement now, since the deck is a slab of
    /// constant thickness under the roadway rather than a shape adapted to
    /// clear it.
    pub clear: f64,
    /// Runs whose face never left the ground anywhere: a bore that never
    /// got under.
    pub grounded: usize,
    /// The least a bore's crown lies under the ground over it, away from
    /// its portals, and how many stations it stands above it: there the
    /// bore has degraded to a cutting the terrain does not have.
    pub cover: f64,
    pub open: usize,
    /// The largest disagreement, in metres, between a span's end height
    /// and the ground piece it lands on.
    pub abutment: f64,
    /// Of the decks and bores, the railways'; and the track bed, in square
    /// metres, the structures lay.
    pub rail_decks: usize,
    pub rail_bores: usize,
    pub bed_m2: f64,
    /// Length of tunnel roadway, in metres, under the terrain with no tube
    /// over it: the terrain lying on the road at a portal, where the mouth
    /// should be.
    pub covered_m: f64,
    /// Road and rail tunnel ends the source mapped, and those a tube's
    /// mouth stands at; and the same for the walks' underpasses.
    pub ends: usize,
    pub mouths: usize,
    pub walk_ends: usize,
    pub walk_mouths: usize,
    /// Tunnel runs drawn as galleries ([`is_gallery`]).
    pub galleries: usize,
}

/// How far under the terrain, in metres, a stretch of tunnel roadway with no
/// tube over it has to lie to count as `covered`: past the lattice's own
/// rounding, short of anything a camera would miss.
const COVERED_EPS_M: f64 = 0.1;

/// Builds every structure the profile implies.
pub fn run(
    terrain: &Terrain,
    roads: &Roads,
    profiles: &Profiles,
    sheets: &Sheets,
    bench: &crate::world::Bench,
) -> (Structure, Summary) {
    let (structure, stats) = {
        // One span per *structure run* of a way, not per profile: a way is
        // one object now and may carry several decks along its length.
        // A run **plus its abutments**: the boundary stations belong to the
        // at-grade solve, and a deck runs from abutment to abutment. Without
        // them the roadway stops one station short at each end and the ground
        // pieces — cut at the annotation edge — do not reach it, so a station
        // of paving is laid by nobody.
        //
        // **The margin is also where two surfaces claim the same ground, and
        // it may not be trimmed away.** Where a ground piece *does* reach,
        // the swept ribbon lies on the unioned carriageway flush to the
        // 2 mm `abutment` reports, and two coincident sheets are what reads
        // from above as overlapping objects: on the loop box 1.17 % of the
        // drawn roadway sits within 0.2 m of carriageway and 0.69 % within
        // 0.05 m. Dropping the margin wherever `carriageway` already covers
        // it takes those to 0.20 % and 0.08 % — and takes the gorge
        // specimen's deck from 40 m to **32 m**, because the derived span is
        // narrower than the slot and the margin is what carries the deck out
        // to the rims (`a_gorge_is_a_bridge_without_being_told`). The
        // roadway then ends over the void while the soffit spans on, which
        // is a worse defect than the overlap it removes.
        //
        // Neither surface is the one to yield: the deck must reach its rims
        // and the ground piece is cut at `a0`. They need a *shared*
        // boundary — the partition cutting the ground piece where the deck's
        // drawn edge falls, rather than one station inside it — or the spans
        // brought into the ribbon/union path so the handover is an internal
        // edge of one region.
        let runs_of = |p: &crate::world::Profile| -> Vec<(usize, usize, Kind, [bool; 2])> {
            let last = p.stations.len().saturating_sub(1);
            p.runs()
                .into_iter()
                .filter(|r| r.2.is_structure())
                .map(|(k0, k1, kind)| (k0.saturating_sub(1), (k1 + 1).min(last), kind, [k0 > 0, k1 < last]))
                .collect()
        };
        let mut spans: Vec<Span> = profiles
            .profiles
            .iter()
            .flat_map(|p| {
                runs_of(p).into_iter().map(move |(k0, k1, kind, open_ends)| {
                    // The span the run's first station of its own lies in.
                    let own = p.stations[(k0 + 1).min(k1)].s;
                    let (a0, a1) = p
                        .spans
                        .iter()
                        .find(|sp| sp.kind.is_structure() && own >= sp.a0 && own <= sp.a1)
                        .map_or((p.stations[k0].s, p.stations[k1].s), |sp| (sp.a0, sp.a1));
                    Span {
                        class: p.class.clone(),
                        width_m: p.width_m,
                        mapped: kind,
                        a0,
                        a1,
                        open_ends,
                        stations: p.stations[k0..=k1].to_vec(),
                        fitted: false,
                        carried: false,
                    }
                })
            })
            .collect();
        let decks = Field::of_stations(profiles.profiles.iter().map(|p| {
            (p, runs_of(p).into_iter().map(|(k0, k1, _, _)| (k0, k1)).collect::<Vec<_>>())
        }));
        spans.extend(
            roads
                .spans
                .iter()
                .filter(|w| width::family(&w.class) == Family::Walk && w.kind != Kind::Indoor)
                .map(|w| fit(w, terrain, &decks)),
        );

        // Every connector a ground piece ends at, and the height it took
        // there: what a span's own end must equal.
        let mut ends: HashMap<(i64, i64), f64> = HashMap::new();
        // A way end that is on the ground: what a span's own end must equal
        // where it lands on one.
        for p in &profiles.profiles {
            let last = p.stations.len().saturating_sub(1);
            for (k, high) in [(0usize, false), (last, true)] {
                if !p.end_kind(high).is_structure() {
                    if let Some(st) = p.stations.get(k) {
                        ends.entry(connector(st.p)).or_insert(st.h);
                    }
                }
            }
        }

        // **Where the asphalt actually is, over every span.** The sheet
        // step already built this from the paving itself — buffered,
        // unioned, kerbed, filleted, cut on the lattice — so a deck's
        // solid built from it is watertight against the road it carries by
        // construction, rather than by a second sweep that lands close but
        // not on it (see the deck run below). Empty before the sheet step
        // has run, same as `road` above.
        let spanned_paving = sheets.spanned();

        // **What height the asphalt actually stands at.** A deck run's own
        // field ([`decks`] below) is built from the profile alone and
        // agrees with the bench's lift everywhere the two ask the same
        // question — but the bench blends at a junction over `BLEND_M`
        // and asks whichever sheet a vertex belongs to, and a field built
        // fresh here has neither: near a junction the two can read apart
        // by more than a kerb's rise, which a deck this step draws is far
        // enough to open a step between its own top and the road actually
        // sitting there. Read from the bench's own mesh instead, the
        // solid's rim is not a second guess at the road's height, the
        // same reason its plan is not a second guess at the road's edge.
        let paved_h = crate::bench::seam(&[&bench.carriageway, &bench.ballast]);

        // How many span ends of each family meet at each connector, which is
        // [`crate::ribbon`]'s rule for a ground piece applied here: a round
        // cap is a disc that closes a *joint* whatever the angle, and a free
        // end is square. Capped round at both ends instead, every deck grew
        // a half-disc of its own half-width past each abutment — `roadway_m2`
        // 72 131 → 73 647 on the loop box, area no road has.
        let mut span_ends: HashMap<(usize, (i64, i64)), usize> = HashMap::new();
        // **Spans that share a connector are one surface, whatever their
        // level ordinals say.** A level orders what *crosses*: the crossing
        // step reads it for two interiors that cross with no connector
        // between them, which is the only place a mapper's `layer` is a
        // claim about who is over whom. Two pieces that share a connector
        // meet at one point in the ground truth, and no tag makes them
        // otherwise — grouped by ordinal instead, the legs of a junction
        // whose mapper tagged them differently never merged, which is the
        // defect this replaced.
        let mut parent: Vec<usize> = (0..spans.len()).collect();
        let mut meet: HashMap<(usize, (i64, i64)), usize> = HashMap::new();
        for (i, span) in spans.iter().enumerate() {
            let n = span.stations.len();
            if n < 2 || width::family(&span.class) == Family::Walk {
                continue;
            }
            let fam = width::family(&span.class) as usize;
            for st in [&span.stations[0], &span.stations[n - 1]] {
                *span_ends.entry((fam, connector(st.p))).or_default() += 1;
                match meet.entry((fam, connector(st.p))) {
                    std::collections::hash_map::Entry::Occupied(e) => {
                        let (a, b) = (root(&mut parent, i), root(&mut parent, *e.get()));
                        if a != b {
                            parent[a] = b;
                        }
                    }
                    std::collections::hash_map::Entry::Vacant(e) => {
                        e.insert(i);
                    }
                }
            }
        }

        let mut s = Structure::default();
        // The spans' footprints, by family and connected group, unioned
        // after the loop. Keyed in a `BTreeMap` rather than a hash so the
        // order the regions are built in is the world's and not the
        // hasher's.
        let mut foot: std::collections::BTreeMap<(usize, usize), Shapes> = std::collections::BTreeMap::new();
        // The bores, kept apart: their roadway is still this step's.
        let mut bores: std::collections::BTreeMap<(usize, usize), Shapes> = std::collections::BTreeMap::new();
        let mut stats = Stats { spans: spans.len(), clear: f64::INFINITY, cover: f64::INFINITY, ..Stats::default() };
        for (i, span) in spans.iter().enumerate() {
            stats.fitted += span.fitted as usize;
            stats.carried += span.carried as usize;
            if span.stations.len() < 2 {
                continue;
            }
            let (l, r) = edges(&span.stations, span.half_w());
            // A railway's span lays its track bed, not a roadway: the same
            // surface in its own material, out to the structure's shoulders.
            let fam = width::family(&span.class);
            let rail = fam == Family::Rail;
            if fam == Family::Walk {
                // A walk span keeps the sweep. The `decks` field the union
                // below takes its heights from is built from the profiles,
                // and a footbridge is not one of them — asked about itself
                // it would answer with the nearest *road* deck.
                strip(&mut s.roadway, &l, &r);
                stats.span_m2 +=
                    poly::length(&span.stations.iter().map(|st| st.p).collect::<Vec<Pt>>()) * 2.0 * span.half_w();
            } else {
                // **A span's surface is a region, not a ribbon.** Swept one
                // span at a time and unioned with nothing, the legs of a
                // junction that happens to stand on a structure overlap each
                // other instead of merging, and leave notches at the corners
                // where the ribbons cross. Measured on the loop box:
                // `roadway` covered 1.43 % of its own area twice, half of
                // that within half a metre — while the unioned `carriageway`
                // covered 0.00 % of its 637 000 m² twice. The union is the
                // whole difference.
                //
                // Buffered the way a ground ribbon is, so the silhouette on
                // a bend is the same arc the carriageway would draw rather
                // than the chord `edges` steps through.
                let axis: Vec<Pt> = span.stations.iter().map(|st| st.p).collect();
                let n = span.stations.len();
                let joint = |p: Pt| span_ends.get(&(fam as usize, connector(p))).copied().unwrap_or(0) > 1;
                let caps = [joint(span.stations[0].p), joint(span.stations[n - 1].p)];
                let into = if matches!(span.mapped, Kind::Tunnel(_)) { &mut bores } else { &mut foot };
                into.entry((fam as usize, root(&mut parent, i)))
                    .or_default()
                    .extend(poly::buffer_line_capped(&axis, 2.0 * span.half_w(), caps));
            }
            for st in [&span.stations[0], &span.stations[span.stations.len() - 1]] {
                if let Some(h) = ends.get(&connector(st.p)) {
                    stats.abutment = stats.abutment.max((st.h - h).abs());
                }
            }
            if span.carried {
                continue;
            }
            // The arcs a tube is drawn over, for `mouths` and `covered` below.
            let mut tubes: Vec<(f64, f64)> = Vec::new();
            // **A gallery's tube stands over the whole of it**, abutment to
            // abutment: it fits under the ground nowhere, and the bench has
            // opened the ground over it for exactly this footprint.
            let gallery = !span.fitted
                && matches!(span.mapped, Kind::Tunnel(_))
                && is_gallery(&span.class, &span.stations);
            if gallery {
                stats.galleries += 1;
                stats.rail_bores += rail as usize;
                tube_over(&mut s.bore, &l, &r, span.height());
                tubes.push((span.stations[0].s, span.stations[span.stations.len() - 1].s));
            }
            let t = span.thickness();
            for (a, b) in runs(&span.stations, Solved::Deck) {
                stats.decks += 1;
                stats.rail_decks += rail as usize;
                // **The solid's plan is the sheet's, not a second sweep.**
                // `l`/`r` still find the run's own paving in `spanned_paving`
                // — a mask around them, generous by a pavement's own reach so
                // a kerb or a fillet at a junction is not clipped off, capped
                // square so it does not reach into a neighbouring run — but
                // what gets meshed is the intersection, the polygon the
                // asphalt above was actually cut to. Its sides therefore rise
                // to exactly the road they carry rather than to an
                // independent guess at its edge (see [`solid_under`]).
                // Widened directly rather than by a `dilate` afterward:
                // `dilate` grows a shape in *every* direction, including
                // past a square cap, so a margin meant to reach past the
                // ribbon's sides reached just as far past its ends —
                // `ROOM_REACH_M` into ground the run does not cover, where
                // the field answers with whatever axis is nearest rather
                // than with this run's own chord. Built into the buffer's
                // own width instead, the caps stay exactly at the run's own
                // ends.
                let sub_axis: Vec<Pt> = span.stations[a..=b].iter().map(|st| st.p).collect();
                let mask = poly::buffer_line_capped(&sub_axis, 2.0 * (span.half_w() + ROOM_REACH_M), [false, false]);
                let mut region = poly::intersect(&mask, &spanned_paving);
                // **A railway's deck is wider than its bed, on purpose.**
                // The sheet only ever paves the track zone — the same
                // surface a level crossing has — so intersecting against it
                // narrows the deck to the rails, losing the edge beam and
                // walkway [`RAIL_SHOULDER_M`] gives a real single-track
                // structure each side. Grown back out by exactly that
                // margin, same as [`half_width_m`] already grows a
                // railway's own half-width for everything else this step
                // builds.
                if rail && !region.is_empty() {
                    // Re-clipped to `mask` after: `dilate` grows in every
                    // direction, so it would reach the same `ROOM_REACH_M`
                    // past the run's own ends the mask above was just built
                    // to avoid. `mask`'s lateral margin is six times this
                    // one, so the shoulder it is meant to add is never what
                    // the re-clip trims.
                    region = poly::intersect(&poly::dilate(&region, RAIL_SHOULDER_M), &mask);
                }
                if region.is_empty() {
                    // No sheet reaches this run — a bare `--until structure`
                    // specimen, or a synthetic world with no paving at all.
                    // The old sweep is still the honest answer there.
                    box_under(&mut s.deck, &l[a..=b], &r[a..=b], t);
                } else {
                    // **`bench::at` can answer confidently and wrongly, not
                    // only miss.** `paved_h` (`bench::seam`) keys purely by
                    // (x, y) and, where two vertices share a key, keeps the
                    // *lower* — right for a kerb a few centimetres above the
                    // carriageway beside it, wrong at a grade separation:
                    // the viaduct's roadway and the road passing under it
                    // legitimately share an (x, y) metres apart in z, and
                    // the seam silently keeps the ground road's height at
                    // that key. A hit is trusted only where it agrees with
                    // this run's own chord within `TOP_AGREE_M` —
                    // comfortably wider than any real kerb/pavement seam,
                    // narrower than any real grade separation — and
                    // `axis_height` is believed otherwise, the same rule
                    // covering the plain miss: `region`'s ring is `mask ∩
                    // spanned_paving`, an intersection that invents vertices
                    // of its own where the local mask's own end cap cuts
                    // across the paving (typically at a junction), and those
                    // are never vertices of the bench's own mesh at all.
                    // Either way `axis_height` cannot answer wrong, because
                    // it knows nothing but this run's own stations — unlike
                    // `paved_h` or the old world-wide `decks` field, which
                    // is exactly the defect `bench::by_sheet` guards against
                    // for the asphalt
                    // (`a_sliver_in_the_span_mask_does_not_lift_the_asphalt`).
                    let top = |p: Pt| {
                        let local = axis_height(&span.stations[a..=b], p);
                        crate::bench::at(&paved_h, p).filter(|h| (h - local).abs() <= TOP_AGREE_M).unwrap_or(local)
                    };
                    solid_under(&mut s.deck, &region, &terrain.grid, &top, t);
                }
                // The least the slab clears the ground under it: a plain
                // measurement, since the slab is a constant thickness under
                // the roadway rather than a shape that adapts to the ground.
                for st in &span.stations[a..=b] {
                    stats.clear = stats.clear.min((st.h - t) - st.ground);
                }
            }
            for (a, b) in runs(&span.stations, Solved::Bore).into_iter().filter(|_| !gallery) {
                stats.bores += 1;
                stats.rail_bores += rail as usize;
                let high = span.height();
                let face: Vec<f64> =
                    span.stations[a..=b].iter().map(|st| st.ground - st.h - high).collect();
                if let Some((f, g)) = spanning(&face) {
                    tubes.push((span.stations[a + f].s, span.stations[a + g].s));
                }
                // **The tube is drawn between its portals and nowhere else.**
                // A bore run is a stretch the road runs *under* the ground by
                // `STRUCTURE_MIN_M`; the tube is a solid `high` tall, and
                // half a metre of burial does not fit five metres of tunnel.
                // Swept over the whole run regardless — which is what this
                // did — 23 of the loop box's 29 bores drew a tube standing
                // proud of the hillside end to end, and the rest poked out at
                // their mouths. The portal is where the crown goes under, and
                // that is exactly what `spanning` has always returned; it
                // only fed the counters.
                match spanning(&face) {
                    None => stats.grounded += 1,
                    Some((f, g)) => {
                        // The tube reaches `PORTAL_M` out over the cutting at
                        // a portal: where it starts at the span's first
                        // station of its own, the span meets ground there and
                        // the partition cut it where the tube fits.
                        let n = span.stations.len();
                        let (from, to) = (a + f, a + g);
                        let out0 = (from == 1 && span.open_ends[0]).then_some(span.a0 - PORTAL_M);
                        let out1 = (to + 2 == n && span.open_ends[1]).then_some(span.a1 + PORTAL_M);
                        let mut hood: Vec<Station> = Vec::with_capacity(to - from + 5);
                        if let Some(s0) = out0 {
                            hood.push(station_at(&span.stations, s0));
                            hood.extend(span.stations[..from].iter().filter(|st| st.s > s0).copied());
                        }
                        hood.extend_from_slice(&span.stations[from..=to]);
                        if let Some(s1) = out1 {
                            hood.extend(span.stations[to + 1..].iter().filter(|st| st.s < s1).copied());
                            hood.push(station_at(&span.stations, s1));
                        }
                        let (hl, hr) = edges(&hood, span.half_w());
                        tube_over(&mut s.bore, &hl, &hr, high);
                        stats.cover = stats.cover.min(face[f..=g].iter().copied().fold(f64::INFINITY, f64::min));
                        stats.open += face[f..=g].iter().filter(|c| **c < 0.0).count();
                    }
                }
            }
            // **A bore too short to have a run still has a tube.** A tunnel
            // span of a few metres — a stub the portal cutting left, a way
            // cut into short pieces — has one station of its own or none, so
            // no run of two stations is solved a bore and nothing above drew
            // anything, though the road is metres under the hill: a road
            // that vanished into it. Such a span's tube stands over the whole
            // of it, abutment to abutment, as a gallery's does; the hill
            // covers it, so the ground is not opened.
            if tubes.is_empty()
                && !span.fitted
                && matches!(span.mapped, Kind::Tunnel(_))
                && span.stations.iter().any(|st| st.ground - st.h > STRUCTURE_MIN_M)
            {
                stats.bores += 1;
                stats.rail_bores += rail as usize;
                tube_over(&mut s.bore, &l, &r, span.height());
                tubes.push((span.stations[0].s, span.stations[span.stations.len() - 1].s));
            }
            // **Every end of a tunnel is a mouth.** An end the source mapped
            // — not one the bbox cut — where no tube stands is a road that
            // disappears into the hill: the tube never fitted anywhere, or it
            // starts somewhere inside and the terrain lies on the road
            // before it. `mouths` counts the ends that do have one.
            if matches!(span.mapped, Kind::Tunnel(_)) && !span.carried && span.stations.len() >= 2 {
                let n = span.stations.len();
                // A solved span carries its abutments, one station outside
                // each end it meets ground at; a fitted one starts on its own
                // ends, which are always a walk's meeting with the ground.
                let (own0, own1) = if span.fitted {
                    (span.stations[0].s, span.stations[n - 1].s)
                } else {
                    (span.stations[1.min(n - 1)].s, span.stations[n.saturating_sub(2)].s)
                };
                for (real, start) in [(span.open_ends[0], true), (span.open_ends[1], false)] {
                    if !real {
                        continue;
                    }
                    let open = tubes.iter().any(|&(t0, t1)| if start { t0 <= own0 + 1e-6 } else { t1 >= own1 - 1e-6 });
                    // A walk's underpass and a road's or a railway's tunnel
                    // are counted apart: the first passes under a way or a
                    // building the terrain has not got, the second under the
                    // hill.
                    let (ends, mouths) =
                        if span.fitted { (&mut stats.walk_ends, &mut stats.walk_mouths) } else { (&mut stats.ends, &mut stats.mouths) };
                    *ends += 1;
                    *mouths += open as usize;
                }
            }
            // **A tunnel's roadway with the terrain on it.** Between a span's
            // own stations — its abutments are the ground pieces' — any
            // stretch the road runs under the terrain with no tube drawn over
            // it is a road the hill lies on: the portal where the mouth
            // should be, shut. Nothing else in the world covers a road, so
            // this reads zero wherever the portals are open.
            if matches!(span.mapped, Kind::Tunnel(_)) {
                let n = span.stations.len();
                for w in span.stations[1.min(n - 1)..n.saturating_sub(1).max(1)].windows(2) {
                    let mid = (w[0].s + w[1].s) / 2.0;
                    let under = w.iter().all(|st| st.ground - st.h > COVERED_EPS_M);
                    if under && !tubes.iter().any(|&(t0, t1)| mid >= t0 && mid <= t1) {
                        stats.covered_m += w[1].s - w[0].s;
                    }
                }
            }
        }
        // **A deck's surface is not this step's; a bore's still is.**
        //
        // A deck and the road that runs onto it share a connector, the
        // profile is continuous through it, and the [`crate::sheet`] step
        // paves both as one polygon lifted by one field — which is what
        // makes the handover a place inside one surface rather than a
        // boundary between two.
        //
        // A bore is the opposite case. Its roadway is under the hill and
        // meets the surface only at its portal, which `open_portals` has
        // already given back to the ground as an open cutting. Merged into
        // a sheet anyway, its chord went into that sheet's field and
        // answered for the road *above* it — a field reaches
        // `FIELD_LIMIT_M` in plan and a hairpin over its own tunnel is
        // nearer than that, so a vertex read 155 m below its neighbour
        // 1.8 m away. So a bore keeps its own sweep, and so does a walk
        // span, which has no profile to be lifted by at all.
        for ((fam, _group), parts) in &foot {
            let region = poly::union_all(parts);
            if region.is_empty() {
                continue;
            }
            let m2 = poly::area(&region);
            if *fam == Family::Rail as usize {
                stats.bed_m2 += m2;
            } else {
                stats.span_m2 += m2;
            }
        }
        // The bores' roadway, meshed and lifted the way it always was.
        for ((fam, _group), parts) in &bores {
            let region = poly::union_all(parts);
            if region.is_empty() {
                continue;
            }
            let height = |p: Pt| decks.at(p).map_or_else(|| height_at(terrain, p[0], p[1]), |f| f.h);
            let (tri, ms) = crate::mesh::triangulate(&region, &terrain.grid, &height);
            stats.span_lost_m2 += ms.lost_m2;
            stats.span_lossy += ms.failed + ms.lossy;
            let rail = *fam == Family::Rail as usize;
            let bed = if rail { &mut s.track } else { &mut s.roadway };
            bed.append(tri);
            let m2 = poly::area(&region);
            if rail {
                stats.bed_m2 += m2;
            } else {
                stats.span_m2 += m2;
            }
        }
        s.plan = spans.iter().filter(|s| s.stations.len() > 1).map(|s| plan(s)).collect();
        (s, stats)
    };
    let finite = |v: f64| if v.is_finite() { format!("{v:.2}") } else { "-".into() };
    let summary = Summary::new()
        .with("spans", stats.spans)
        .with("fitted", stats.fitted)
        .with("carried", stats.carried)
        .with("decks", stats.decks)
        .with("bores", stats.bores)
        .with("galleries", stats.galleries)
        .with("rail", format!("{}/{}", stats.rail_decks, stats.rail_bores))
        .with_m2("span_m2", stats.span_m2)
        .with("span_lost_m2", format!("{:.1e}", stats.span_lost_m2))
        .with("span_lossy", stats.span_lossy)
        .with_m2("bed_m2", stats.bed_m2)
        .with(
            "triangles",
            (structure.roadway.indices.len()
                + structure.track.indices.len()
                + structure.deck.indices.len()
                + structure.bore.indices.len())
                / 3,
        )
        .with("clear", finite(stats.clear))
        .with("grounded", stats.grounded)
        .with("cover", finite(stats.cover))
        .with("open", stats.open)
        .with("covered", format!("{:.1}", stats.covered_m))
        .with("mouths", format!("{}/{}", stats.mouths, stats.ends))
        .with("walk_mouths", format!("{}/{}", stats.walk_mouths, stats.walk_ends))
        .with("abutment", format!("{:.3}", stats.abutment));
    (structure, summary)
}

/// A pedestrian span, fitted: a chord between the ground at its two ends,
/// or the deck of the road that is already carrying it.
///
/// No ceiling and no box — a draped class holds nothing (stratum D) — so
/// the chord is the whole of the construction, and what it turns out to be
/// is read from the ground the same way a solved piece's is.
fn fit(w: &crate::world::Polyline2, terrain: &Terrain, decks: &Field) -> Span {
    let pts = densify(&w.pts, NODE_M);
    let mut stations: Vec<Station> = Vec::with_capacity(pts.len());
    let mut s = 0.0;
    for (i, p) in pts.iter().enumerate() {
        if i > 0 {
            s += (p[0] - pts[i - 1][0]).hypot(p[1] - pts[i - 1][1]);
        }
        let ground = height_at(terrain, p[0], p[1]);
        // A draped class samples the finished ground exactly (stratum D), so
        // its reference is the ground: there is no surface it is solved
        // toward, because it is not solved.
        stations.push(Station { s, p: *p, ground, reference: ground, h: ground, solved: Solved::Grade });
    }
    // Carried, if a road's own span runs within the room's reach of every
    // one of its stations: the same structure, drawn twice by the source.
    // The test is proximity and nothing else — asking also that the road
    // stand off the ground there would fail at the abutments, where it is
    // at grade by definition, and no span would ever be carried.
    let carried =
        stations.iter().all(|st| decks.at(st.p).is_some_and(|foot| foot.d <= foot.half_w + ROOM_REACH_M));
    if carried {
        // It rides the road's cross-section: the road's own height plus
        // the kerb's rise, wherever it stands, exactly as a pavement does
        // over the ground. What it is — deck, bore or neither — is the
        // road's answer and not a second one.
        for st in stations.iter_mut() {
            let road = decks.at(st.p).expect("every station has a road's span beside it");
            st.h = road.h + KERB_RISE_M;
            st.solved = Solved::Grade;
        }
        let a1 = stations.last().map_or(0.0, |st| st.s);
        return Span {
            class: w.class.clone(),
            width_m: w.width_m,
            mapped: w.kind,
            a0: 0.0,
            a1,
            open_ends: [true, true],
            stations,
            fitted: true,
            carried,
        };
    }
    let len = stations.last().map(|st| st.s).unwrap_or(0.0);
    let (h0, h1) = (stations[0].ground, stations[stations.len() - 1].ground);
    for st in stations.iter_mut() {
        st.h = if len > 0.0 { h0 + (h1 - h0) * st.s / len } else { h0 };
        st.solved = if st.h - st.ground >= STRUCTURE_MIN_M {
            Solved::Deck
        } else if st.ground - st.h >= STRUCTURE_MIN_M {
            Solved::Bore
        } else {
            Solved::Grade
        };
    }
    Span {
        class: w.class.clone(),
        width_m: w.width_m,
        mapped: w.kind,
        a0: 0.0,
        a1: len,
        open_ends: [true, true],
        stations,
        fitted: true,
        carried,
    }
}

/// The stretch of a run over which the structure has actually left the
/// ground: from the first station whose face clears to the last. Outside
/// it a deck is landing and a bore is surfacing, and that is what an
/// abutment and a portal *are* — a deck 1.5 m thick cannot have its
/// soffit out of the ground until its roadway is 1.5 m over it, so the
/// stations either side of the stretch carry no check. `None` if the face
/// never cleared at all.
fn spanning(face: &[f64]) -> Option<(usize, usize)> {
    let f = face.iter().position(|c| *c >= 0.0)?;
    let g = face.iter().rposition(|c| *c >= 0.0)?;
    (g > f).then_some((f, g))
}

/// The station at arc `s` along `st`: interpolated between the two either
/// side of it, or carried straight on from the nearest end segment where
/// `s` lies past the ends — a hood a couple of metres out from its last
/// station runs on along the road's own line.
fn station_at(st: &[Station], s: f64) -> Station {
    let n = st.len();
    let k = st.partition_point(|x| x.s < s).clamp(1, n - 1);
    let (a, b) = (st[k - 1], st[k]);
    let t = if b.s > a.s { (s - a.s) / (b.s - a.s) } else { 0.0 };
    let lerp = |x: f64, y: f64| x + (y - x) * t;
    Station {
        s,
        p: [lerp(a.p[0], b.p[0]), lerp(a.p[1], b.p[1])],
        ground: lerp(a.ground, b.ground),
        reference: lerp(a.reference, b.reference),
        h: lerp(a.h, b.h),
        solved: a.solved,
    }
}

/// The runs of consecutive stations solved `kind`, as inclusive index
/// pairs. A run of one station is no run: a structure needs a length.
fn runs(stations: &[Station], kind: Solved) -> Vec<(usize, usize)> {
    let mut out = Vec::new();
    let mut start: Option<usize> = None;
    for (i, st) in stations.iter().enumerate() {
        match (st.solved == kind, start) {
            (true, None) => start = Some(i),
            (false, Some(a)) => {
                if i - 1 > a {
                    out.push((a, i - 1));
                }
                start = None;
            }
            _ => {}
        }
    }
    if let Some(a) = start {
        if stations.len() - 1 > a {
            out.push((a, stations.len() - 1));
        }
    }
    out
}

/// Union-find root, path-halved: which group of connected spans `x` is in.
fn root(parent: &mut [usize], mut x: usize) -> usize {
    while parent[x] != x {
        parent[x] = parent[parent[x]];
        x = parent[x];
    }
    x
}

/// The height of `stations`' own chord at the point nearest `p`: a
/// run-local stand-in for the roadway height, used only where the paved
/// mesh has no vertex to ask. It cannot answer with a different run's
/// height, unlike a world-wide field, because it knows nothing but these
/// stations.
fn axis_height(stations: &[Station], p: Pt) -> f64 {
    let mut best = (f64::INFINITY, stations[0].h);
    for w in stations.windows(2) {
        let (a, b) = (w[0].p, w[1].p);
        let d = [b[0] - a[0], b[1] - a[1]];
        let len2 = d[0] * d[0] + d[1] * d[1];
        let t = if len2 > 0.0 { (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / len2).clamp(0.0, 1.0) } else { 0.0 };
        let q = [a[0] + d[0] * t, a[1] + d[1] * t];
        let dist2 = (p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2);
        if dist2 < best.0 {
            best = (dist2, w[0].h + (w[1].h - w[0].h) * t);
        }
    }
    best.1
}

/// The left and right edge of a piece `half_w` from its axis, at the
/// solved height of every station. The direction at a station is the
/// centred difference of its neighbours, so a bend is mitred by the mean
/// of the two headings rather than stepped.
fn edges(st: &[Station], half_w: f64) -> (Vec<[f64; 3]>, Vec<[f64; 3]>) {
    let n = st.len();
    let (mut l, mut r) = (Vec::with_capacity(n), Vec::with_capacity(n));
    for i in 0..n {
        let (a, b) = (st[i.saturating_sub(1)].p, st[(i + 1).min(n - 1)].p);
        let mut t = poly::unit([b[0] - a[0], b[1] - a[1]]);
        if t == [0.0, 0.0] {
            t = [1.0, 0.0];
        }
        let (p, h) = (st[i].p, st[i].h);
        l.push([p[0] - t[1] * half_w, p[1] + t[0] * half_w, h]);
        r.push([p[0] + t[1] * half_w, p[1] - t[0] * half_w, h]);
    }
    (l, r)
}

/// Quads between two polylines of equal length, wound counter-clockwise
/// seen from the side `a` is left of: `strip(left, right)` faces up.
fn strip(tri: &mut Tri, a: &[[f64; 3]], b: &[[f64; 3]]) {
    let base = tri.positions.len() as u32;
    for i in 0..a.len() {
        tri.positions.push(a[i]);
        tri.positions.push(b[i]);
    }
    for i in 0..a.len().saturating_sub(1) {
        let (a0, b0, a1, b1) = (base + 2 * i as u32, base + 2 * i as u32 + 1, base + 2 * i as u32 + 2, base + 2 * i as u32 + 3);
        tri.indices.extend_from_slice(&[a0, b0, b1, a0, b1, a1]);
    }
}

/// `l`/`r` lowered by `t`, straight down: a deck's underside, the same
/// constant thickness under the roadway everywhere. No seating on the
/// ground — a shallow run's slab is drawn straight through, and any
/// earthwork it implies is left to a later pass.
fn lower(v: &[[f64; 3]], t: f64) -> Vec<[f64; 3]> {
    v.iter().map(|p| [p[0], p[1], p[2] - t]).collect()
}

/// The solid between a deck's roadway and its underside: the underside
/// itself, the two sides and the two end faces. The roadway above it was
/// drawn once already and is not drawn again.
fn box_under(tri: &mut Tri, l: &[[f64; 3]], r: &[[f64; 3]], t: f64) {
    let (lo_l, lo_r) = (lower(l, t), lower(r, t));
    strip(tri, &lo_r, &lo_l);
    strip(tri, &lo_l, l);
    strip(tri, r, &lo_r);
    let n = l.len() - 1;
    tri.quad([l[0], r[0], lo_r[0], lo_l[0]]);
    tri.quad([r[n], l[n], lo_l[n], lo_r[n]]);
}

/// The solid under a deck, built from `region` — the sheet's own paved
/// polygon for this run — instead of [`box_under`]'s sweep of `edges()`.
///
/// `top` is the roadway height at a point of `region`: the same field the
/// asphalt above was lifted by, so a vertex of this solid's rim is the
/// same point the road surface holds, not a second construction's guess
/// at where that was. The floor is a deck's thickness `t` below it,
/// straight down, everywhere — a plain slab, not a shape adapted to clear
/// the ground.
fn solid_under(tri: &mut Tri, region: &Shapes, grid: &crate::grid::Grid, top: &dyn Fn(Pt) -> f64, t: f64) {
    let floor = |p: Pt| -> f64 { top(p) - t };
    // The underside, facing down: `mesh::triangulate` winds a surface to
    // face up (counter-clockwise seen from above), so its trailing two
    // indices swap per triangle to turn it the other way.
    let (mut under, _) = crate::mesh::triangulate(region, grid, &floor);
    for t3 in under.indices.chunks_exact_mut(3) {
        t3.swap(1, 2);
    }
    tri.append(under);
    // The sides, swept along every ring `region` has: the same device
    // `bench::wall` closes a step between two surfaces with. A region's
    // outer ring runs counter-clockwise and a hole the other way, so this
    // faces outward in both cases without a case of its own.
    for ring in region.iter().flatten() {
        for i in 0..ring.len() {
            let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
            let (ta, tb) = (top(a), top(b));
            let (fa, fb) = (floor(a), floor(b));
            tri.quad([[a[0], a[1], ta], [a[0], a[1], fa], [b[0], b[1], fb], [b[0], b[1], tb]]);
        }
    }
}

/// `l`/`r` raised by `dz` and walled: the crown and the two walls of a
/// bore, seen from inside. The portals stay open — the terrain has no
/// face cut into it for them yet.
fn tube_over(tri: &mut Tri, l: &[[f64; 3]], r: &[[f64; 3]], dz: f64) {
    let raise = |v: &[[f64; 3]]| -> Vec<[f64; 3]> { v.iter().map(|p| [p[0], p[1], p[2] + dz]).collect() };
    let (up_l, up_r) = (raise(l), raise(r));
    strip(tri, &up_r, &up_l);
    strip(tri, l, &up_l);
    strip(tri, &up_r, r);
}

/// A span's outline in plan, for the plan view: its two edges, closed.
fn plan(span: &Span) -> (Kind, Shapes) {
    let (l, r) = edges(&span.stations, span.half_w());
    let mut ring: Vec<Pt> = l.iter().map(|p| [p[0], p[1]]).collect();
    ring.extend(r.iter().rev().map(|p| [p[0], p[1]]));
    (span.mapped, poly::ccw(ring).map(|ring| vec![vec![ring]]).unwrap_or_default())
}

#[cfg(test)]
mod tests {
    use crate::world::World;
    use crate::pipeline::tests::{built, upto};
    use crate::step::Step;
    

    
    

    use super::*;

    /// A world on `ground` with the network of `net`, built to the end.
    fn world(ground: &str, net: &str) -> (World, Summary) {
        let (w, ran) = built(ground, net, None, 5.0, &upto(Step::Structure));
        (w, ran.last())
    }

    fn structure(w: &World) -> &Structure {
        w.structure.as_ref().expect("the structure step ran")
    }

    /// **The paving over a span, where it now lives.** This step stopped
    /// owning a surface: a carriageway's or a railway's deck is paved by
    /// the [`crate::sheet`] step, in one polygon with the ground it runs
    /// onto, and lifted by the bench. So the checks that used to read
    /// `roadway` read the bench's own mesh, masked to the span's plan —
    /// which is the same geometry asked for in the place it is built.
    ///
    /// `rail` picks the ballast rather than the carriageway. Both are
    /// restricted to the span footprints the structure step still collects
    /// for `plan`, so the mask is the step's own answer about where its
    /// spans are and not a number retyped from the specimen.
    fn paved(w: &World, rail: bool) -> Vec<[f64; 3]> {
        let bench = w.bench.as_ref().expect("the bench step ran");
        let tri = if rail { &bench.ballast } else { &bench.carriageway };
        let over: crate::poly::Shapes =
            structure(w).plan.iter().flat_map(|(_, s)| s.iter().cloned()).collect();
        let index = crate::poly::Indexed::new(&poly::dilate(&over, crate::bench::OVER_RIM_M));
        tri.positions.iter().copied().filter(|p| index.contains([p[0], p[1]])).collect()
    }

    /// The highest paving over a span, which is a deck's roadway.
    fn paved_top(w: &World, rail: bool) -> f64 {
        paved(w, rail).iter().map(|p| p[2]).fold(f64::NEG_INFINITY, f64::max)
    }

    #[test]
    fn runs_need_a_length() {
        let st = |solved: Solved| {
            Station { s: 0.0, p: [0.0, 0.0], ground: 0.0, reference: 0.0, h: 0.0, solved }
        };
        let row = [Solved::Grade, Solved::Deck, Solved::Deck, Solved::Grade, Solved::Deck, Solved::Grade, Solved::Deck, Solved::Deck];
        let stations: Vec<Station> = row.iter().map(|s| st(*s)).collect();
        // The lone deck station at 4 is no run; the pair at 1..2 and the
        // pair the row ends on are.
        assert_eq!(runs(&stations, Solved::Deck), vec![(1, 2), (6, 7)]);
        assert!(runs(&stations, Solved::Bore).is_empty());
    }

    #[test]
    fn a_valley_gets_a_viaduct() {
        // The plan's specimen: a 60 m valley with a mapped bridge over the
        // middle of it. The chord runs between the ground at the two
        // abutments, so it flies; the deck's soffit is 1.5 m under the
        // roadway and clears the ground everywhere between them.
        let (w, s) = world("hill?amp=-60&radius=300", "net:straight?len=400&span=0.35,0.65");
        assert_eq!(s.num("decks"), 1.0, "{s}");
        assert_eq!(s.num("bores"), 0.0, "{s}");
        assert_eq!(s.num("grounded"), 0.0, "{s}");
        // The soffit clears all the way; least where it is landing, most
        // over the valley floor, which is 60 m under the rim the chord
        // runs between.
        assert!(s.num("clear") > 0.0, "{s}");
        let t = w.terrain.as_ref().expect("the terrain step ran");
        let mid = crate::terrain::height_at(t, 0.0, 0.0);
        assert_eq!(s.num("abutment"), 0.0, "the deck does not land where the road is: {s}");
        let b = structure(&w);
        assert!(b.roadway.indices.is_empty(), "the span's paving is the sheet's now: {s}");
        assert!(!b.deck.indices.is_empty() && b.bore.indices.is_empty());
        let road = paved(&w, false);
        assert!(!road.is_empty(), "the bench paves the deck: {s}");
        let deck_z = paved_top(&w, false);
        assert!(deck_z - DECK_THICKNESS_M - mid > 4.0, "over the floor it clears {}", deck_z - mid);
        // The roadway is the chord: level over the span, since the two
        // abutments stand at the same height on a radial hill. Read off
        // the middle of it, because the mask reaches into the abutment
        // where the paving is the approach coming up to meet it.
        let z: Vec<f64> = road.iter().filter(|p| p[0].abs() < 50.0).map(|p| p[2]).collect();
        let (lo, hi) = (z.iter().cloned().fold(f64::INFINITY, f64::min), z.iter().cloned().fold(f64::NEG_INFINITY, f64::max));
        assert!((hi - lo).abs() < 1e-6, "{lo} {hi}");
        // And it is one road's width across.
        let y: Vec<f64> = road.iter().filter(|p| p[0].abs() < 50.0).map(|p| p[1]).collect();
        let w_m = y.iter().cloned().fold(f64::NEG_INFINITY, f64::max) - y.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!((w_m - 5.5).abs() < 1e-6, "{w_m}");
        // The deck's soffit is exactly a deck's thickness under it.
        let deck_z: Vec<f64> = b.deck.positions.iter().map(|p| p[2]).collect();
        let bottom = deck_z.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!((hi - bottom - DECK_THICKNESS_M).abs() < 1e-9, "{hi} {bottom}");
    }

    #[test]
    fn a_hill_gets_a_bore() {
        // The mirror: a hill with a mapped tunnel through it. The chord
        // runs under the ground, the crown 5 m over the roadway, and the
        // ground covers it everywhere between the portals.
        let (w, s) = world("hill?amp=60&radius=300", "net:straight?len=400&span=0.1,0.9&kind=tunnel");
        assert_eq!(s.num("bores"), 1.0, "{s}");
        assert_eq!(s.num("decks"), 0.0, "{s}");
        assert_eq!(s.num("open"), 0.0, "the crown breaks surface between the portals: {s}");
        assert_eq!(s.num("grounded"), 0.0, "{s}");
        assert!(s.num("cover") > 0.0, "{s}");
        assert_eq!(s.num("abutment"), 0.0, "{s}");
        let b = structure(&w);
        assert!(!b.bore.indices.is_empty() && b.deck.indices.is_empty());
        let road = b.roadway.positions.iter().map(|p| p[2]).fold(f64::NEG_INFINITY, f64::max);
        let crown = b.bore.positions.iter().map(|p| p[2]).fold(f64::NEG_INFINITY, f64::max);
        assert!((crown - road - TUNNEL_HEIGHT_M).abs() < 1e-9, "{crown} {road}");
        // The portals stand 160 m out on the flank, where the hill is
        // 26.9 m up; the crest is 60 m up. So the chord runs level under
        // 33 m of hill and the crown is some 28 m under the crest.
        let t = w.terrain.as_ref().expect("the terrain step ran");
        let cover = crate::terrain::height_at(t, 0.0, 0.0) - crown;
        assert!((cover - 28.1).abs() < 0.2, "{cover}");
    }

    #[test]
    fn a_span_that_never_left_the_ground_is_no_structure() {
        // A mapped bridge over flat ground: the chord is the ground, so
        // the profile solved every station at grade and this step builds no
        // solid at all — plain, not wrong (invariant 6). The roadway is
        // still laid, because the road is still a road.
        let (w, s) = world("flat", "net:straight?len=400&span=0.35,0.65");
        assert_eq!(s.num("decks"), 0.0, "{s}");
        assert_eq!(s.num("bores"), 0.0, "{s}");
        assert!((s.num("span_m2") - 660.0).abs() < 1.0, "{s}");
        let b = structure(&w);
        assert!(b.deck.indices.is_empty() && b.bore.indices.is_empty());
        assert!(b.roadway.positions.iter().all(|p| (p[2] - 400.0).abs() < 1e-9));
    }

    #[test]
    fn a_sidewalk_over_a_bridge_is_not_a_second_bridge() {
        // The plan's specimen, and 22.7 % of the extract's footbridges: a
        // road bridge whose separated sidewalk is mapped as its own
        // bridge. It is one structure. The walk is *carried* — its height
        // is the road's plus the kerb's rise — and it builds no deck, so
        // the count of decks is the road's one.
        let (w, s) = world("hill?amp=-60&radius=300", "net:sidewalk?d=6&span=0.35,0.65");
        assert_eq!(s.num("carried"), 1.0, "{s}");
        assert_eq!(s.num("decks"), 1.0, "the walk built a second bridge: {s}");
        let b = structure(&w);
        // Both are paved, and the walk's paving stands one kerb over the
        // road's, all the way along.
        // The road's paving is the sheet's now and the walk's is still
        // this step's sweep — a footbridge has no profile and so no field
        // to be lifted by. That the two are laid by different steps is
        // exactly what this check is about, so it reads each where it is.
        let of = |src: &Tri, lo: f64, hi: f64| -> Vec<f64> {
            let mut z: Vec<f64> =
                src.positions.iter().filter(|p| p[1] > lo && p[1] < hi).map(|p| p[2]).collect();
            z.sort_by(|a, b| a.partial_cmp(b).unwrap());
            z.dedup();
            z
        };
        let road = of(&w.bench.as_ref().unwrap().carriageway, -3.0, 3.0);
        let walk = of(&b.roadway, 4.0, 8.0);
        assert!(!road.is_empty() && !walk.is_empty(), "both are paved");
        assert!((walk[0] - road[0] - KERB_RISE_M).abs() < 1e-6, "{:?} vs {:?}", &walk[..1], &road[..1]);
    }

    #[test]
    fn a_footbridge_of_its_own_is_fitted() {
        // A footway whose span runs nowhere near a road carries no profile
        // — a draped class holds nothing — so this step fits it a chord
        // between the ground at its own two ends, and it reads as a deck
        // by the same rule as a road. Its deck is a footbridge's, not a
        // road bridge's.
        // The 400 m road is what makes this a slab rather than a graze:
        // over a 200 m one the chord's abutments sit inside the bowl and
        // the soffit comes down onto the floor exactly, where the abutment
        // block — rightly — seats it on the ground and there is no slab
        // left to measure.
        let (w, s) = world("hill?amp=-60&radius=300", "net:sidewalk?d=40&span=0.35,0.65&len=400");
        assert_eq!(s.num("carried"), 0.0, "{s}");
        assert_eq!(s.num("decks"), 2.0, "the road and the footbridge both fly: {s}");
        let b = structure(&w);
        // Two thicknesses in one mesh: 1.5 m under the road, 0.4 m under
        // the walk. Each chord tilts by a millimetre over its 120 m,
        // because the lattice is not symmetric about the origin and the
        // ground it reads at the two abutments is not exactly equal, so
        // the depth of a deck is read to the centimetre and not the ulp.
        let under = |lo: f64, hi: f64| -> f64 {
            let z: Vec<f64> = b.deck.positions.iter().filter(|p| p[1] > lo && p[1] < hi).map(|p| p[2]).collect();
            z.iter().cloned().fold(f64::NEG_INFINITY, f64::max) - z.iter().cloned().fold(f64::INFINITY, f64::min)
        };
        assert!((under(-3.0, 3.0) - DECK_THICKNESS_M).abs() < 0.01, "{}", under(-3.0, 3.0));
        assert!((under(38.0, 42.0) - WALK_DECK_M).abs() < 0.01, "{}", under(38.0, 42.0));
    }

    #[test]
    fn an_overpass_gets_its_embankment_and_its_deck_from_the_steps_it_already_had() {
        // On flat ground the mapped span degrades in the profile — a chord at
        // grade is not a bridge — and the crossing step's floor is what
        // makes it one. Neither the bench nor this step learns a rule:
        // the fill is the embankment the approach now stands on, and the
        // 5 m the soffit clears is exactly the headroom that was asked
        // for, the 1.5 m slab having been the rest of the demand.
        let (w, s) = world("flat", "net:overpass?len=300");
        assert_eq!(s.num("decks"), 1.0, "{s}");
        assert_eq!(s.num("bores"), 0.0, "{s}");
        // The deck reaches down the approach to where the road stands
        // `DECK_STANDOFF_M` off the ground, so what its soffit clears at its
        // own ends is that less the slab — the invariant a *derived* deck
        // has, where a deck cut to the annotation had the full headroom.
        let least = crate::grade::DECK_STANDOFF_M - DECK_THICKNESS_M;
        assert!(s.num("clear") >= least - 1e-6, "clear {} < {least}: {s}", s.num("clear"));
        assert!(s.num("clear") <= crate::crossing::ROAD_CLEARANCE_M + 1e-6, "{s}");
        // **The approach and the deck are one surface.** They used to be
        // two: the bench laid the road up to the deck's foot at
        // `DECK_STANDOFF_M` and the structure step swept the rest, and the
        // joint between them was the seam this whole change is about. Now
        // the sheet holds both, so the bench's own carriageway runs all the
        // way up to the chord — 6.5 m, the clearance the crossing asked for
        // — and there is no step anywhere across it.
        let b = w.bench.as_ref().unwrap();
        let top = b.carriageway.positions.iter().map(|p| p[2]).fold(f64::NEG_INFINITY, f64::max);
        assert!(
            (top - (400.0 + crate::crossing::ROAD_CLEARANCE_M + DECK_THICKNESS_M)).abs() < 0.5,
            "the bench stopped short of the deck it now paves: {top}"
        );
        assert!(structure(&w).roadway.indices.is_empty(), "and this step laid none of it: {s}");
        // The mirror: the leg is cut into the ground and the bore's crown
        // carries the road above it on the slab's thickness.
        let (_, s) = world("flat", "net:underpass?len=300");
        assert_eq!(s.num("bores"), 1.0, "{s}");
        assert_eq!(s.num("decks"), 0.0, "{s}");
        assert_eq!(s.num("open"), 0.0, "{s}");
        // The bore starts where the road runs `BORE_COVER_M` under the
        // ground, so the ground over its crown at that end is that less the
        // tube — the cover a *derived* bore has at its portal, where one cut
        // to the annotation had the slab's own thickness.
        let least = crate::partition::BORE_COVER_M - TUNNEL_HEIGHT_M;
        assert!(s.num("cover") >= least - 1e-6, "cover {} < {least}: {s}", s.num("cover"));
        assert!(s.num("cover") <= DECK_THICKNESS_M + 1e-6, "{s}");
    }

    #[test]
    fn a_deck_is_a_plain_slab_even_where_it_cannot_clear() {
        // A 3 m dip: the chord flies over the middle of it but only just,
        // so the slab clears in the centre and cannot at the two ends. No
        // abutment block seats the underside on the ground any more — the
        // deck is a constant [`DECK_THICKNESS_M`] under the roadway,
        // full length, so `clear` can read negative there. That is the
        // simplification this step accepts (see the module docs).
        let (_, s) = world("hill?amp=-3&radius=40", "net:straight?len=200&span=0.35,0.65");
        assert_eq!(s.num("decks"), 1.0, "{s}");
        assert!(s.num("clear") < 0.0, "the ends should read short of clearing now: {s}");
    }

    #[test]
    fn a_bridge_that_never_clears_is_still_one_continuous_slab() {
        // Half a metre off the ground makes a station a deck and the slab
        // is three times that thick, so a mapped bridge over a shallow
        // ditch is a slab buried end to end. It is still drawn as one
        // continuous slab under the roadway rather than as an embankment —
        // building the ground under it is left to a later pass.
        let (_, s) = world("hill?amp=-1&radius=40", "net:straight?len=200&span=0.35,0.65");
        assert_eq!(s.num("decks"), 1.0, "{s}");
        assert!(s.num("clear") < 0.0, "{s}");
    }

    /// A railway's span lays its track bed, not a roadway, and stands on
    /// the same deck a road's would — wider than its track by a shoulder
    /// each side, as a real single-track deck is.
    #[test]
    fn a_railway_bridge_lays_track_on_a_deck() {
        let (w, s) = world("gorge?depth=30&width=40", "net:straight?len=400&span=0.4,0.6&class=standard_gauge");
        let st = structure(&w);
        // The bed is the sheet's now, and this step lays neither surface.
        assert!(st.track.indices.is_empty() && st.roadway.indices.is_empty(), "{s}");
        let bed = paved(&w, true);
        assert!(!bed.is_empty(), "the bench lays the bed over the span: {s}");
        assert!(w.bench.as_ref().unwrap().carriageway.positions.is_empty(), "no road here: {s}");
        assert!(s.num("decks") >= 1.0, "{s}");
        assert_eq!(s.get("rail"), Some("1/0"), "{s}");
        // **The bed is its own width and the deck under it is wider.**
        // The bed is the track zone, the same on a deck as on the ground —
        // it is one surface now, and a railway that narrowed where it left
        // the ground would be the seam in another form. The *deck* is what
        // carries a shoulder each side, as a real single-track deck does.
        let y: Vec<f64> = bed.iter().filter(|p| p[0].abs() < 30.0).map(|p| p[1]).collect();
        let across = y.iter().cloned().fold(f64::NEG_INFINITY, f64::max) - y.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!((across - crate::width::RAIL_M).abs() < 1e-6, "the bed is the track zone: {across}");
        let dy: Vec<f64> = st.deck.positions.iter().map(|p| p[1]).collect();
        let deck_w = dy.iter().cloned().fold(f64::NEG_INFINITY, f64::max) - dy.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!(
            (deck_w - (crate::width::RAIL_M + 2.0 * RAIL_SHOULDER_M)).abs() < 1e-6,
            "the deck carries a shoulder each side: {deck_w}"
        );
        // And the deck under it is a deck's thickness deep.
        let top = paved_top(&w, true);
        let bottom = st.deck.positions.iter().map(|p| p[2]).fold(f64::INFINITY, f64::min);
        assert!(top - bottom >= DECK_THICKNESS_M - 1e-6, "{top} {bottom}");
    }

    /// **A railway's bore has room for the wire.** A standard-gauge line
    /// through a ridge, flat either side so the railway's own grade has
    /// nothing to do: the crown stands [`RAIL_TUNNEL_M`] over the track, a
    /// metre more than a road tunnel's, and the ridge still covers it
    /// between the portals. Metre gauge runs in a road tunnel's section.
    #[test]
    fn a_railway_bore_has_room_for_the_wire() {
        let (w, s) = world("ridge?height=40&width=120", "net:straight?len=400&span=0.3,0.7&kind=tunnel&class=standard_gauge");
        assert_eq!(s.num("bores"), 1.0, "{s}");
        assert_eq!(s.get("rail"), Some("0/1"), "{s}");
        assert_eq!(s.num("open"), 0.0, "the crown breaks surface between the portals: {s}");
        let b = structure(&w);
        // A bore's bed is still this step's: it is under the hill, and the
        // only place it meets the surface is its portal.
        assert!(!b.track.indices.is_empty(), "{s}");
        let track = b.track.positions.iter().map(|p| p[2]).fold(f64::NEG_INFINITY, f64::max);
        let crown = b.bore.positions.iter().map(|p| p[2]).fold(f64::NEG_INFINITY, f64::max);
        assert!((crown - track - RAIL_TUNNEL_M).abs() < 1e-6, "{crown} {track}");
        assert_eq!(tube_m("narrow_gauge"), NARROW_RAIL_TUNNEL_M);
        assert_eq!(tube_m("residential"), TUNNEL_HEIGHT_M);
        assert_eq!(tube_m("footway"), WALK_TUNNEL_M);
        // The partition asks the same tube to fit before it calls a run a
        // bore: a metre deeper for the railway than for a road.
        assert!((crate::partition::bore_cover_m("standard_gauge") - crate::partition::BORE_COVER_M - 1.0).abs() < 1e-12);
    }

    /// **A tunnel too shallow for its tube is a gallery, with two mouths.**
    /// A low hill over a mapped tunnel: the chord between the portals runs
    /// 3 m under the crest at most, and a 5 m tube (6 m for a mainline) fits
    /// nowhere. As a bore it drew no tube and the terrain lay on the road
    /// end to end; as a gallery the tube stands over the whole span, both
    /// ends are mouths, and the ground is opened for it.
    #[test]
    fn a_shallow_tunnel_is_a_gallery_with_two_mouths() {
        for class in ["residential", "standard_gauge"] {
            let (w, s) = world("hill?amp=3&radius=100", &format!("net:straight?len=400&span=0.3,0.7&kind=tunnel&class={class}"));
            assert_eq!(s.num("galleries"), 1.0, "{class}: {s}");
            assert_eq!(s.num("bores"), 0.0, "{class}: {s}");
            assert_eq!(s.get("mouths"), Some("2/2"), "{class}: {s}");
            assert_eq!(s.num("covered"), 0.0, "{class}: the terrain lies on the road: {s}");
            assert!(!structure(&w).bore.indices.is_empty(), "{class}: no tube");
            // The ground is opened under the tube: no terrain triangle has
            // its centroid on the gallery's axis.
            let g = &w.bench.as_ref().expect("the bench step ran").ground;
            let on_axis = g.indices.chunks_exact(3).any(|t| {
                let c = t.iter().fold([0.0, 0.0], |c, &i| {
                    let p = g.positions[i as usize];
                    [c[0] + p[0] / 3.0, c[1] + p[1] / 3.0]
                });
                c[0].abs() < 50.0 && c[1].abs() < 1.0
            });
            assert!(!on_axis, "{class}: terrain over the gallery");
        }
    }

    /// **A portal is open.** A road and a railway through a ridge, flat
    /// either side: the road meets the flank at grade, and between there and
    /// where the tube first fits under the ground it runs under the hill with
    /// nothing over it but the terrain. That stretch is the portal's cutting,
    /// and no tunnel roadway may lie under the terrain uncovered by a tube.
    #[test]
    fn a_portal_is_open() {
        for class in ["residential", "standard_gauge"] {
            let (_, s) = world("ridge?height=40&width=120", &format!("net:straight?len=400&span=0.3,0.7&kind=tunnel&class={class}"));
            assert_eq!(s.num("bores"), 1.0, "{class}: {s}");
            assert_eq!(s.num("covered"), 0.0, "{class}: the terrain lies on the road at a portal: {s}");
        }
    }

    #[test]
    fn the_structures_are_a_function_of_the_world() {
        let (a, _) = world("hill?amp=-60&radius=300", "net:straight?len=400&span=0.35,0.65");
        let (b, _) = world("hill?amp=-60&radius=300", "net:straight?len=400&span=0.35,0.65");
        assert_eq!(structure(&a).roadway.positions, structure(&b).roadway.positions);
        assert_eq!(structure(&a).deck.indices, structure(&b).deck.indices);
    }
}
