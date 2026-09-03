//! Invariant 4 at the kerb: "nothing floats and nothing is buried by accident",
//! measured where the asphalt and the drawn ground part company.
//!
//! **Why the kerb and not under the road.** This check began by sampling the
//! carriageway's interior against the terrain's triangles, which is where the
//! defect lived while ground was drawn under the asphalt: not a wrong *height*
//! — the road height field and the ground function agree to the centimetre
//! wherever a bench exists — but a wrong *surface*, two triangulations of the
//! same intent crossing each other between their shared vertices. Cutting the
//! terrain back to the kerb (docs/GROUND.md §3) removed the drawn ground under
//! the asphalt, and with it that instrument's whole population: `road − ground`
//! went from ~400 k samples to eighteen, not because the heights came right but
//! because there was nothing left to compare against.
//!
//! The gap did not vanish, it moved to the boundary, so that is where it is
//! measured. Two metrics, from one walk each:
//!
//! - [`contact.kerb_lip`] — how tall a wall the model implies, taken at every
//!   carriageway silhouette edge against the ground a metre outside it. Not a
//!   defect to drive to zero: a road on a real embankment has a real drop at its
//!   edge. It is the honest size of the earthwork the profile asked for, and the
//!   only thing left that can see a road standing on an embankment nobody built.
//! - [`contact.kerb_unwalled`] — how much of that wall is missing, walked along
//!   the terrain's own hole rim. This is the gate.

use crate::verify::dist::Dist;
use crate::verify::mesh::SurfaceMesh;
use crate::verify::scene::{RoadMesh, TileScene};
use crate::verify::{Invariant, Metric, Offender, Sense, Worst};

use super::{Check, Options};

/// A kerb standing this far above the ground beside it is not a kerb. Real
/// ones run to about a quarter-metre and the boundary carries quantization and
/// a metre of probe offset on a cross-slope, so half a metre is comfortably
/// past what kerb-ness explains and far short of the metres a missing
/// retaining wall costs.
const LIP_M: f64 = 0.5;

/// How far outside the kerb, in metres, the ground is asked for. Far enough to
/// clear the rounding on the shared boundary vertices, near enough that it is
/// still the ground *at* the kerb and not the next thing along.
const LIP_PROBE_M: f64 = 1.0;

/// How far past an abutment the ground is followed, and in what steps, looking
/// for the wall the deck should have started on top of. Matched to
/// `synth::draped`'s own reach and step so the two see the same wall.
const SEAT_PROBE_M: f64 = 20.0;
const SEAT_STEP_M: f64 = 2.0;

/// The climb that makes ground beside an abutment a wall rather than a slope
/// the path walks up. `synth::draped`'s `WALL_GRADE`, restated here because
/// this check must be able to disagree with the generator: sharing the constant
/// by import would make a change to the rule silently move the measurement too.
const SEAT_WALL_GRADE: f64 = 0.6;

/// How far a fitted deck's abutment may stand below the wall beside it before
/// it reads as seated in the notch rather than on its bank. The seating rule
/// declines corrections under a metre, and one probe step of a 60 % wall is
/// another 1.2 m, so anything under about that is the rule working rather than
/// failing.
const SEAT_M: f64 = 2.0;

/// How far, in plan, a footway may run from a solved deck's own *surface* and
/// still be a sidewalk on that bridge rather than a bridge beside it.
///
/// The probe's finding is that this threshold is not delicate, and the reason
/// to trust it: searching out to 25 m, the centerline offset of every carried
/// path in the extract lies between 2.0 m and 9.5 m, and
/// **nothing at all between 9.5 m and 25 m**. Any cut through that empty band
/// claims the same spans (a 4 m gap and a 16 m gap both claim 25 of 110).
///
/// Ten metres reads that finding into archive space, where the distance
/// available is to the carrier's drawn *surface* rather than to its
/// centerline. Surface distance is the smaller of the two by the deck's own
/// half-width, so it cannot push a carried span past 9.5 m, and it cannot pull
/// an uncarried one — over 25 m away by centerline — nearer than about 20 m.
/// The two instruments agree on the count it produces: 25 candidate carriers
/// from the scene model, 25 from the emitted archive.
const CARRY_LATERAL_M: f64 = 10.0;

/// How much of a fitted deck must run alongside one solved deck before that
/// deck counts as carrying it — the test that separates a sidewalk *on* a
/// bridge from a footway that merely reaches one.
///
/// Measured over the 13 archive pairs that clear [`CARRY_LATERAL_M`] and
/// [`CARRY_JOIN_M`], coverage is sharply bimodal — p0 0.08, then p25 0.90, p50
/// 1.00 — and the gate sits in the empty middle: 0.3 through 0.9 all keep 10 or
/// 11 of the 13. The same split read as a shared run in metres is cleaner
/// still, with the rejected three sharing 1.7 m, 11.7 m and 14.7 m of their
/// span and every accepted one sharing 15 m or more.
///
/// The three it rejects are real, and rejecting them is the point: they are
/// long walkways that run beside a road bridge for part of their length and
/// carry themselves for the rest. A rule that seats a whole span on a carrier
/// has nothing to say about a span that is mostly not on one, and a check
/// should claim what its rule can fix.
const CARRY_COVER: f64 = 0.7;

/// How closely the two decks must meet at one of the footway's ends.
///
/// This is the half of the rule that keeps a genuine footbridge out of the
/// population, and it is not optional: 3 of the 25 spans the lateral test alone
/// claims are paths passing *underneath* a structure, including a 12 m
/// footbridge under a motorway viaduct whose deck is 68 m overhead. A sidewalk
/// **joins** its road bridge — it arrives at the same abutment — so wherever
/// the annotation and the DEM agree at one end the fitted chord already lands
/// on the deck (p50 0.49 m). A path passing under one never touches it at
/// either end.
///
/// Sorted, the population separates itself and leaves this threshold a band to
/// sit in rather than a value to tune: seventeen candidates within 1.91 m, then
/// nothing until 4.69 m, then the three passing underneath. Every ceiling from
/// 2.0 m to 4.6 m claims the same spans.
const CARRY_JOIN_M: f64 = 3.0;

/// How nearly parallel the two decks' axes must be — |cos| of the angle
/// between them, so 0.87 is 30°.
///
/// A sidewalk is parallel to its bridge by construction, and this is the test
/// that says *along* rather than *across*. Coverage cannot do it on its own:
/// with a [`CARRY_LATERAL_M`] reach of ten metres, a twenty-metre footbridge
/// crossing a road bridge at right angles still has most of its samples within
/// reach of it, and reads as 79 % covered. Thirty degrees leaves room for a
/// bridge that curves — both axes are end-to-end chords, which a curving deck
/// and its sidewalk strike at slightly different angles.
const CARRY_ALONG: f64 = 0.87;

/// Sample spacing along a fitted deck's axis, and how far off that axis the
/// deck's own surface may be and still answer for it. A fitted deck is
/// pedestrian-scale (`priors::PATH_STRUCTURE_HALF_WIDTH_M` = 1.25 m), so two
/// metres reaches its far edge on a span that curves away from its own chord.
const CARRY_STEP_M: f64 = 2.0;
const CARRY_NEAR_M: f64 = 2.0;

/// How far a footway may sink below the deck carrying it before the two read
/// as separate structures. A sidewalk sits within a kerb and a parapet of its
/// carriageway; a metre is past both and far short of the p50 3.84 m that the
/// duplicate-bridge defect costs.
const CARRY_M: f64 = 1.0;

/// How far a drawn railway may stand off the ground directly beneath it before
/// the gap is air rather than the mesh's own resolution.
///
/// This is not the kerb's question. `contact.kerb_lip` probes a metre *outside*
/// the road, where a real embankment has a real drop; this asks the ground
/// *under* the formation, which the bench is supposed to have brought up to
/// meet it. There is no legitimate positive answer — a railway on an embankment
/// stands on the embankment.
///
/// So the threshold is only the width of what the mesh cannot help. Read off
/// the classes that bench: `standard_gauge` benches at 98.9 % of its at-grade
/// nodes and its standoff runs p95 0.80 m, p98 1.45 m, p99 1.62 m, then jumps
/// to 5.52 m at p999 — and the level-0 *road* lines, which share the drape
/// path, reach only 1.27 m at p999. The 2–4 m bin holds 0.12 % of standard
/// gauge. Two metres sits in that empty band: past the ~3 m detail cell's worth
/// of relief a profile can disagree with the lattice by on a steep flank, and
/// far short of the metres the float costs.
const RAIL_STANDOFF_M: f64 = 2.0;

/// How close a drawn structure surface must be to a rail stroke, vertically,
/// for the stroke to be *on* it.
///
/// The population trap this first closed was an emit-order bug: a structure
/// span emitted its paint stroke *before* the level ordinal was attached
/// (`pipeline.rs`), so the stroke over a viaduct arrived in the archive at
/// level 0 and metres above the ground — 19,993 of the level-0 vertices on the
/// Montreux extract. That bug is fixed (the ordinal is attached first), and the
/// guard was flagged for removal on the assumption its population had gone
/// empty with it.
///
/// **Measured, it has not, and the guard stays.** Turned off, the coarse rungs
/// gain 47 vertices at z12 and 15 at z11 — strokes riding their own deck
/// solids, which is not a formation floating over the ground however far above
/// it they are. The rate barely moves (6.674 → 6.781 % at z12) and the worst
/// not at all, so this is a correctness guard rather than a number: what it
/// removes is a different class, not a tail.
const RAIL_ON_DECK_M: f64 = 1.0;

pub struct Contact {
    lip: Dist,
    lip_worst: Worst,
    unwalled: Dist,
    unwalled_worst: Worst,
    seat: Dist,
    seat_worst: Worst,
    carried: Dist,
    carried_worst: Worst,
    rail: Dist,
    rail_worst: Worst,
}

impl Contact {
    pub fn new(opt: &Options) -> Contact {
        Contact {
            lip: Dist::metres(),
            lip_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            unwalled: Dist::metres(),
            unwalled_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            seat: Dist::metres(),
            seat_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            carried: Dist::metres(),
            carried_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            rail: Dist::metres(),
            rail_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
        }
    }

    /// Invariant 4 under a railway: how far the drawn formation stands off the
    /// drawn ground beneath it.
    ///
    /// **A coarse-rung instrument now.** This walk was written when a railway
    /// laid no surface and was drawn as a cartographic stroke riding its
    /// solved profile, straight over ground that was never asked to come up
    /// and meet it (rail strokes reached 5.25 m at p95 where road strokes
    /// held half a metre at 97.5 %). The formation band closed most of that,
    /// leaving this the residue: rail whose ballast band failed to mesh. Then
    /// the stroke itself left the detail zooms — from
    /// `priors::ROAD_SURFACE_MIN_ZOOM` the union paves the formation and
    /// `pipeline::paves_via_union` deletes the rail stroke with the
    /// carriageway's — so at the surface zooms this population is empty by
    /// construction and the walk only measures pre-surface rungs. The residue
    /// class it used to catch is unmeasured until the formation-coverage
    /// check (roadmap) lands.
    fn visit_rail(&mut self, tile: &TileScene, terrain: &SurfaceMesh) {
        let decks: Vec<&RoadMesh> = tile.roads.iter().filter(|r| r.is_deck()).collect();
        for line in tile.lines.iter().filter(|l| l.level == 0 && l.is_rail()) {
            for part in &line.parts {
                for &(px, py, h) in part {
                    if !tile.owns(px, py) {
                        continue;
                    }
                    let Some(gz) = terrain.height_at(px, py) else {
                        continue; // no drawn ground here to stand off from
                    };
                    // Paint on a deck, not a formation in the air. Guards an
                    // empty class at the surface zooms — the deck stroke is
                    // deleted with the rest — and, on the coarse rungs this
                    // walk still measures, the viaduct strokes that ride
                    // their solids.
                    if decks.iter().any(|d| {
                        d.mesh.height_at(px, py).is_some_and(|z| (h - z).abs() < RAIL_ON_DECK_M)
                    }) {
                        continue;
                    }
                    let v = h - gz;
                    self.rail.push(v);
                    if v > RAIL_STANDOFF_M {
                        let (lon, lat) = tile.lonlat(px, py);
                        self.rail_worst.offer(Offender {
                            lon,
                            lat,
                            zoom: tile.z,
                            value: v,
                            note: format!(
                                "{} formation {h:.2} m, drawn ground {gz:.2} m, nothing between them",
                                line.class
                            ),
                        });
                    }
                }
            }
        }
    }

    /// Invariant 4 alongside a road bridge: how far a footway's own deck sinks
    /// below the solved deck that is actually carrying it.
    ///
    /// Overture maps a road bridge's separated sidewalk as an independently
    /// `bridge`-tagged footway, and a footway is a **D** feature, so
    /// `synth::draped` fits it a deck of its own chorded to the ground at the
    /// two ends of its span. Where the path runs *along* a road bridge that
    /// assumption is wrong twice over: the ground it reads is the ground under
    /// the bridge, and the result is a second, smaller structure hanging
    /// beneath the real one — joined to it at the abutment where the two
    /// happen to agree, and diving under it at the other end. In the Montreux
    /// extract that is 22.7 % of every footbridge in the scene.
    ///
    /// Three conditions say *carried*, and all three are load-bearing:
    ///
    /// - it runs within [`CARRY_LATERAL_M`] of a solved deck's surface,
    /// - over at least [`CARRY_COVER`] of its own length,
    /// - and **meets** that deck within [`CARRY_JOIN_M`] at one of its ends.
    ///
    /// The last is what separates a sidewalk from a path passing underneath.
    /// Without it the worst offender in the extract is a 12 m footbridge
    /// crossing a stream under a motorway viaduct, reported as 68 m wrong for
    /// the crime of being beneath a road — which is what it is for.
    fn visit_carried(&mut self, tile: &TileScene) {
        let carriers: Vec<&RoadMesh> =
            tile.roads.iter().filter(|r| r.is_deck() && !r.is_fitted_deck()).collect();
        for deck in tile.roads.iter().filter(|r| r.is_fitted_deck()) {
            let Some(ends) = deck_ends(&deck.mesh) else { continue };
            // A deck clipped by the tile border has an "end" that is the
            // border, and this measurement is taken at the ends; the tile that
            // owns them answers for it.
            if ends.iter().any(|e| !tile.owns(e.x, e.y)) {
                continue;
            }
            // The population is the *carried* decks, not every fitted one. A
            // footbridge that carries itself is not a weak instance of this
            // defect, it is a different thing, and counting it would bury the
            // measurement under a hundred structural zeroes. It also makes the
            // sample count the headline: a rule that puts sidewalks on their
            // bridges empties this population rather than flattening it.
            let Some(carrier) = carried_below(tile, deck, &carriers, &ends) else { continue };
            let v = carrier.sink;
            self.carried.push(v);
            if v > CARRY_M {
                let (lon, lat) = tile.lonlat(ends[0].x, ends[0].y);
                self.carried_worst.offer(Offender {
                    lon,
                    lat,
                    zoom: tile.z,
                    value: v,
                    note: format!(
                        "{} deck runs {:.0} % along a {} deck it meets to {:.2} m at one end, \
                         and sinks {:.2} m below it",
                        deck.class,
                        carrier.covered * 100.0,
                        carrier.class,
                        carrier.join,
                        v
                    ),
                });
            }
        }
    }

    /// Invariant 4 at a footbridge's abutment: how far the deck's lower end
    /// stands below the wall that runs up beside it.
    ///
    /// A fitted deck is chorded between the ground at its annotated ends
    /// (`synth::draped`). Where that end landed part way down a gorge wall the
    /// bridge starts in the riverbed it is supposed to cross, and the path
    /// dives in after it. The wall is what separates that from a bridge that
    /// correctly ends at the foot of a slope the path climbs: ground going up
    /// at a walkable grade is ground the path can be on, and only ground
    /// steeper than [`SEAT_WALL_GRADE`] is followed.
    ///
    /// Asked at the *lower* abutment, and bounded by the height of the higher
    /// one. Both halves are needed, and each excludes a whole population the
    /// other admits: without the wall test a footbridge landing at the foot of
    /// a slope reads as buried, and without the bound a level footbridge on a
    /// steep hillside — a hairpin path crossing its own gully — reads as
    /// twenty metres wrong because the mountain is above it, which it is
    /// supposed to be. What is left is the thing that is actually broken: a
    /// deck tilted because one of its ends fell down a wall, measured against
    /// the end that did not.
    fn visit_seats(&mut self, tile: &TileScene, terrain: &SurfaceMesh) {
        for deck in tile.roads.iter().filter(|r| r.is_fitted_deck()) {
            let Some([a, b]) = deck_ends(&deck.mesh) else { continue };
            // A deck clipped by the tile border has an "end" that is the
            // border; the tile that owns it answers for it.
            if !tile.owns(a.x, a.y) || !tile.owns(b.x, b.y) {
                continue;
            }
            let (low, high) = if a.top <= b.top { (a, b) } else { (b, a) };
            let (dx, dy) = (
                (low.x - high.x) * tile.scale.mx,
                (low.y - high.y) * tile.scale.my,
            );
            let len = dx.hypot(dy);
            if len < 1.0 {
                continue;
            }
            let (ux, uy) = (dx / len / tile.scale.mx, dy / len / tile.scale.my);
            // The ground *at* the abutment counts before any marching: a deck
            // whose end is already under the surface it is supposed to start on
            // is buried, and that needs no wall to be a defect.
            let mut prev = drawn_ground(tile, terrain, low.x, low.y).unwrap_or(low.top);
            let mut climbed = (prev - low.top).max(0.0);
            let mut d = SEAT_STEP_M;
            while d <= SEAT_PROBE_M {
                let Some(h) = drawn_ground(tile, terrain, low.x + ux * d, low.y + uy * d) else {
                    break;
                };
                if (h - prev) / SEAT_STEP_M < SEAT_WALL_GRADE {
                    break; // the climb relaxed: the wall, if any, ends here
                }
                climbed = climbed.max(h - low.top);
                prev = h;
                d += SEAT_STEP_M;
            }
            let v = climbed.clamp(0.0, high.top - low.top);
            self.seat.push(v);
            if v > SEAT_M {
                let (lon, lat) = tile.lonlat(low.x, low.y);
                self.seat_worst.offer(Offender {
                    lon,
                    lat,
                    zoom: tile.z,
                    value: v,
                    note: format!(
                        "{} deck starts at {:.2} m against {:.2} m at its far end, with wall \
                         above it the whole way",
                        deck.class, low.top, high.top
                    ),
                });
            }
        }
    }
}

/// The drawn ground at a plan point, through the hole cut under the asphalt.
///
/// The terrain mesh stops at the kerb (docs/GROUND.md §3), so a probe that
/// walked onto a road would find nothing and stop — which is exactly where a
/// footbridge's bank tends to be, since the path arrives at a street. Where
/// there is no terrain the at-grade carriageway *is* the drawn ground.
fn drawn_ground(tile: &TileScene, terrain: &SurfaceMesh, px: f64, py: f64) -> Option<f64> {
    terrain.height_at(px, py).or_else(|| {
        tile.roads
            .iter()
            .filter(|r| r.is_pavement() || r.is_rim())
            .find_map(|r| r.mesh.height_at(px, py))
    })
}

/// What a metre outside a kerb is standing on: the walkway band where one has
/// been laid, and the drawn terrain everywhere else.
///
/// **The probe follows the drawn world, not the terrain mesh.** A walkway takes
/// its own hole out of the ground, so once sidewalks exist the terrain answers
/// `None` exactly where the strip beside the kerb is properly occupied — and
/// `contact.kerb_lip`, reading terrain alone, silently dropped those samples
/// and reported a *worse* rate on the bare verges left behind. The metric had
/// not moved; its best members had left the population. Reading the band gives
/// the honest answer there: a kerb rise, not a drop.
fn beside_ground(tile: &TileScene, terrain: &SurfaceMesh, px: f64, py: f64) -> Option<f64> {
    tile.roads
        .iter()
        .filter(|r| {
            r.level == 0 && (r.class.starts_with("walk_") || r.class.starts_with("path_"))
        })
        .find_map(|r| r.mesh.height_at(px, py))
        .or_else(|| terrain.height_at(px, py))
}

/// One end of a deck: a point on its centerline and the deck top there.
struct DeckEnd {
    x: f64,
    y: f64,
    top: f64,
}

/// A deck's two ends, in unit plan space.
///
/// The long axis is taken as the vertex furthest from the centroid and the
/// vertex furthest from *that*, which for a swept slab is a pair of opposite
/// end corners. Each end is then the centroid of the vertices in the outermost
/// tenth of the projection onto that axis, so the point sits on the deck's
/// centerline rather than on a corner — a corner query lands on the boundary,
/// where the terrain lookup is a coin toss.
fn deck_ends(m: &SurfaceMesh) -> Option<[DeckEnd; 2]> {
    let n = m.vertex_count();
    if n < 3 {
        return None;
    }
    let (mut cx, mut cy) = (0.0, 0.0);
    for i in 0..n {
        let (x, y, _) = m.vertex(i);
        cx += x / n as f64;
        cy += y / n as f64;
    }
    let far = |from: (f64, f64)| {
        (0..n).fold(((0.0, 0.0), -1.0), |best, i| {
            let (x, y, _) = m.vertex(i);
            let d = (x - from.0).hypot(y - from.1);
            if d > best.1 {
                ((x, y), d)
            } else {
                best
            }
        })
        .0
    };
    let a = far((cx, cy));
    let b = far(a);
    let (ax, ay) = (b.0 - a.0, b.1 - a.1);
    let len2 = ax * ax + ay * ay;
    if len2 <= 0.0 {
        return None;
    }
    let t_of = |i: usize| {
        let (x, y, _) = m.vertex(i);
        ((x - a.0) * ax + (y - a.1) * ay) / len2
    };
    let end = |near_zero: bool| {
        let (mut sx, mut sy, mut top, mut k) = (0.0, 0.0, f64::NEG_INFINITY, 0usize);
        for i in 0..n {
            let t = t_of(i);
            if (near_zero && t <= 0.1) || (!near_zero && t >= 0.9) {
                let (x, y, z) = m.vertex(i);
                sx += x;
                sy += y;
                top = top.max(z);
                k += 1;
            }
        }
        (k > 0).then(|| DeckEnd { x: sx / k as f64, y: sy / k as f64, top })
    };
    Some([end(true)?, end(false)?])
}

/// The solved deck carrying a fitted one, and how far the fitted deck sinks
/// below it.
struct Carrier<'a> {
    class: &'a str,
    /// Fraction of the fitted deck's samples that found this deck alongside.
    covered: f64,
    /// The closer of the two ends' height disagreements — how well the two
    /// decks meet where the sidewalk joins its bridge.
    join: f64,
    /// The deepest the fitted deck runs below it, never negative: a footway
    /// riding a little *above* its carriageway is a kerb, not a defect.
    sink: f64,
}

/// A deck's long axis as a unit vector in metres. Unit plan space is not
/// isotropic — a tile is about twice as wide as it is tall in metres per unit —
/// so an angle taken there would call two decks parallel that are not.
fn axis_dir(tile: &TileScene, ends: &[DeckEnd; 2]) -> Option<(f64, f64)> {
    let dx = (ends[1].x - ends[0].x) * tile.scale.mx;
    let dy = (ends[1].y - ends[0].y) * tile.scale.my;
    let len = dx.hypot(dy);
    (len > 0.0).then(|| (dx / len, dy / len))
}

/// Which solved deck, if any, is carrying this fitted one.
///
/// Walks the fitted deck's own axis and asks, at each step, what a solved deck
/// spans nearby. The carrier is the one alongside the most of it — the same
/// rule `synth::carried` uses, and for the same reason: a footway at a
/// road junction can run beside two decks, and the one it is *on* is the one
/// it never leaves.
fn carried_below<'a>(
    tile: &TileScene,
    deck: &RoadMesh,
    carriers: &[&'a RoadMesh],
    ends: &[DeckEnd; 2],
) -> Option<Carrier<'a>> {
    if carriers.is_empty() {
        return None;
    }
    let len = tile.scale.dist(ends[0].x, ends[0].y, ends[1].x, ends[1].y);
    if len < 1.0 {
        return None;
    }
    // The fitted deck's own top along its axis. The height is read off the
    // chord rather than out of the mesh: a fitted deck *is* a chord between its
    // two ends (`synth::draped`), so this is exact, where a `span_near` on its
    // own surface would return the highest corner of whatever triangle reaches
    // the query — the far abutment, on a deck swept as two long triangles.
    // The mesh is still asked whether it is *there*, so a span that curves away
    // from its chord drops the samples that left it instead of measuring the
    // air beside it.
    let steps = (len / CARRY_STEP_M).ceil() as usize;
    let mut axis: Vec<(f64, f64, f64)> = Vec::with_capacity(steps + 1);
    for i in 0..=steps {
        let t = i as f64 / steps as f64;
        let px = ends[0].x + (ends[1].x - ends[0].x) * t;
        let py = ends[0].y + (ends[1].y - ends[0].y) * t;
        if deck.mesh.span_near(px, py, &tile.scale, CARRY_NEAR_M).is_some() {
            axis.push((px, py, ends[0].top + (ends[1].top - ends[0].top) * t));
        }
    }
    if axis.len() < 2 {
        return None;
    }
    let along = axis_dir(tile, ends)?;
    let mut best: Option<Carrier> = None;
    for c in carriers {
        // Along it, not across it. Asked before any sampling: a deck the
        // footway merely crosses answers the lateral and coverage tests over a
        // short span and has no business being called its carrier.
        let parallel = deck_ends(&c.mesh)
            .and_then(|e| axis_dir(tile, &e))
            .is_some_and(|d| (d.0 * along.0 + d.1 * along.1).abs() >= CARRY_ALONG);
        if !parallel {
            continue;
        }
        let (mut hits, mut sink) = (0usize, f64::NEG_INFINITY);
        // The disagreement at the *nearest sample to each end* that found this
        // deck at all, rather than at the end sample itself. A road bridge's
        // deck rarely starts and stops exactly where the sidewalk's annotated
        // span does, and demanding it be there at the last centimetre rejects
        // the carrier over a metre of arc.
        let (mut first, mut last) = (None, None);
        for &(px, py, top) in axis.iter() {
            let Some((_, ctop)) = c.mesh.span_near(px, py, &tile.scale, CARRY_LATERAL_M) else {
                continue;
            };
            hits += 1;
            sink = sink.max(ctop - top);
            first = first.or(Some(ctop - top));
            last = Some(ctop - top);
        }
        let covered = hits as f64 / axis.len() as f64;
        if covered < CARRY_COVER {
            continue;
        }
        // How well they meet at an end. A deck reaching neither end of the
        // footway is not what the footway is standing on, whatever it covers
        // in between.
        let join = match (first, last) {
            (Some(f), Some(l)) => f.abs().min(l.abs()),
            (Some(x), None) | (None, Some(x)) => x.abs(),
            (None, None) => continue,
        };
        if join > CARRY_JOIN_M {
            continue;
        }
        if best.as_ref().is_none_or(|b| covered > b.covered) {
            best = Some(Carrier { class: c.class.as_str(), covered, join, sink: sink.max(0.0) });
        }
    }
    best
}

impl Check for Contact {
    fn visit(&mut self, tile: &TileScene, _opt: &Options) {
        // Two decks against each other: no terrain needed, and asked before the
        // terrain gate so a zoom that carries structures but no ground mesh
        // still answers for them.
        self.visit_carried(tile);
        let Some(terrain) = &tile.terrain else { return };
        self.visit_seats(tile, terrain);
        self.visit_rail(tile, terrain);
        // The kerb lip: at every silhouette edge of the carriageway, the road's
        // own height against the ground a metre outside it.
        //
        // A few centimetres is a kerb. Fifteen metres is a retaining wall the
        // model implies and does not draw — a hole you can see the hillside
        // through.
        for road in tile.roads.iter().filter(|r| r.is_pavement()) {
            for (a, b, opp) in road.mesh.boundary_edges() {
                let (ax, ay, az) = road.mesh.vertex(a);
                let (bx, by, bz) = road.mesh.vertex(b);
                let (ox, oy, _) = road.mesh.vertex(opp);
                let (mx, my) = ((ax + bx) * 0.5, (ay + by) * 0.5);
                if !tile.owns(mx, my) {
                    continue;
                }
                // Outward is away from the one triangle holding this edge.
                let (dx, dy) = (mx - ox, my - oy);
                let len = tile.scale.dist(0.0, 0.0, dx, dy);
                if len <= 0.0 {
                    continue;
                }
                let (px, py) = (
                    mx + dx / len * LIP_PROBE_M,
                    my + dy / len * LIP_PROBE_M,
                );
                let Some(gz) = beside_ground(tile, terrain, px, py) else { continue };
                let kerb_z = (az + bz) * 0.5;
                let v = kerb_z - gz;
                self.lip.push(v);
                if v > LIP_M {
                    let (lon, lat) = tile.lonlat(mx, my);
                    self.lip_worst.offer(Offender {
                        lon,
                        lat,
                        zoom: tile.z,
                        value: v,
                        note: format!("kerb {kerb_z:.2} m, ground {gz:.2} m a metre outside it"),
                    });
                }
            }
        }
        // Watertightness, asked of the hole's own rim.
        //
        // The asphalt's interior mesh stops an inset short of its silhouette
        // and the terrain's hole is cut *at* the silhouette, so the two
        // boundaries are 35 cm apart and no query anchored on one finds the
        // other. Anchoring on the terrain's rim instead makes it structural:
        // every terrain boundary edge that is not the tile's own edge is a hole
        // rim, the asphalt (interior or rim) answers for the road's height
        // over it, and anything between the two heights that no apron spans is
        // a gap you can see through.
        for (a, b, _) in terrain.boundary_edges() {
            let (ax, ay, az) = terrain.vertex(a);
            let (bx, by, bz) = terrain.vertex(b);
            let (mx, my) = ((ax + bx) * 0.5, (ay + by) * 0.5);
            if !tile.owns(mx, my) {
                continue;
            }
            // The tile's own edge is not a hole: the neighbour's terrain
            // continues across it.
            let on_edge = |v: f64| v.abs() < 1e-6 || (v - 1.0).abs() < 1e-6;
            if on_edge(mx) || on_edge(my) {
                continue;
            }
            let rim_z = (az + bz) * 0.5;
            let Some(road_z) = tile
                .roads
                .iter()
                .filter(|r| r.is_pavement() || r.is_rim())
                .filter_map(|r| r.mesh.height_at(mx, my))
                .next()
            else {
                continue; // no asphalt over it: not the hole's rim
            };
            let gap = (road_z - rim_z).abs();
            let (lo_z, hi_z) = (road_z.min(rim_z), road_z.max(rim_z));
            let walled = gap <= LIP_M || super::apron_spans(tile, mx, my, lo_z, hi_z);
            self.unwalled.push(if walled { 0.0 } else { gap });
            if !walled {
                let (lon, lat) = tile.lonlat(mx, my);
                self.unwalled_worst.offer(Offender {
                    lon,
                    lat,
                    zoom: tile.z,
                    value: gap,
                    note: format!(
                        "asphalt {road_z:.2} m, terrain rim {rim_z:.2} m, nothing between them"
                    ),
                });
            }
        }
    }

    fn finish(self: Box<Self>) -> Vec<Metric> {
        let skipped = || {
            "no at-grade road surface at this zoom (below ROAD_SURFACE_MIN_ZOOM, or the archive \
             carries no DEM)"
                .to_string()
        };
        vec![
            Metric {
                id: "contact.kerb_unwalled".into(),
                invariant: Invariant::I4,
                title: "Gap at the hole's rim with nothing spanning it".into(),
                population: format!(
                    "Every terrain-mesh boundary edge midpoint that is not on the tile's own \
                     edge and has at-grade asphalt or rim over it. A cut edge carries no \
                     apron by design and is excluded with the tile edge. The apron is vertical, \
                     so its span is asked within {:.1} m of the rim rather than at \
                     it, with {:.1} m of slack at each end.",
                    super::APRON_NEAR_M,
                    super::APRON_SLOP_M
                ),
                detail: "Watertightness, walked along the terrain's own hole rim: at every \
                         terrain boundary edge that is not the tile's edge, the asphalt's height \
                         over it against the terrain's, where no apron spans the difference. \
                         Anchored on the rim rather than on the asphalt because the two \
                         boundaries are an inset apart, so a query anchored on one never finds \
                         the other — and asked at the same point rather than a metre out, \
                         because a cutting's terrain rises steeply but perfectly continuously \
                         and there is nothing to see through."
                    .into(),
                sense: Sense::HigherIsWorse,
                threshold: LIP_M,
                skipped: self.unwalled.is_empty().then(skipped),
                dist: self.unwalled,
                worst: self.unwalled_worst.into_vec(),
            },
            Metric {
                id: "contact.kerb_lip".into(),
                invariant: Invariant::I4,
                title: "Drop from the kerb to the ground beside it".into(),
                population: format!(
                    "Every silhouette (boundary) edge midpoint of every level-0 road_surface \
                     mesh inside the tile proper, probed {LIP_PROBE_M:.0} m along the outward \
                     normal. Structures are excluded: a deck edge is not a kerb."
                ),
                detail: format!(
                    "Carriageway edge height minus the drawn ground {LIP_PROBE_M:.0} m outside \
                     it. With the ground cut back to the kerb this is where the road and the \
                     terrain part company, and it is the only place left that can see a road \
                     standing on an embankment nobody built. Not a defect to drive to zero: it \
                     is the honest height of the earthwork the profile asked for, and past \
                     {LIP_M:.2} m the model implies a retaining wall it must also draw \
                     (`contact.kerb_unwalled` is how much of it is missing)."
                ),
                sense: Sense::HigherIsWorse,
                threshold: LIP_M,
                skipped: self.lip.is_empty().then(skipped),
                dist: self.lip,
                worst: self.lip_worst.into_vec(),
            },
            Metric {
                id: "contact.deck_seat".into(),
                invariant: Invariant::I4,
                title: "Wall standing over a footbridge's lower abutment".into(),
                population: format!(
                    "The lower abutment of every *fitted* deck — a draped class carrying an \
                     elevated span, whose deck is chorded to the ground rather than solved \
                     (`synth::draped`) — lying wholly inside the tile proper. Followed \
                     {SEAT_PROBE_M:.0} m outward along the deck's own axis in \
                     {SEAT_STEP_M:.0} m steps, and only while the ground climbs at \
                     {:.0} % or more. Three coverage limits: a deck clipped by the tile \
                     border is left to the tile that owns it; a span whose abutment stands on \
                     a wall the drawn terrain mesh is too coarse to resolve reads lower than \
                     it is; and a span with *both* abutments down in the notch reads zero, \
                     because the bound is its own far end and there is nothing in the archive \
                     that says how high the path beyond it goes.",
                    SEAT_WALL_GRADE * 100.0
                ),
                detail: format!(
                    "How far a footbridge's lower abutment stands below the wall beside it, \
                     bounded by its own far end. A fitted deck's ends come from the ground at \
                     the annotated span's edges, and against a near-vertical wall two metres \
                     of plan disagreement between the annotation and the DEM is a dozen metres \
                     of height: the bridge begins part way down the gorge it crosses and the \
                     path dives in after it. Ground under {:.0} % is a slope the path walks \
                     rather than a wall it cannot, and the bound keeps a level footbridge on a \
                     hillside from scoring the hillside. Past {SEAT_M:.1} m the deck is tilted \
                     into the notch instead of sitting on its bank.",
                    SEAT_WALL_GRADE * 100.0
                ),
                sense: Sense::HigherIsWorse,
                threshold: SEAT_M,
                skipped: self.seat.is_empty().then(|| {
                    "no fitted decks at this zoom — the extract carries no draped feature with \
                     an elevated span, or the archive carries no DEM"
                        .to_string()
                }),
                dist: self.seat,
                worst: self.seat_worst.into_vec(),
            },
            Metric {
                id: "contact.deck_carried".into(),
                invariant: Invariant::I4,
                title: "Footway deck sunk below the bridge carrying it".into(),
                population: format!(
                    "Every *carried* fitted deck (`synth::draped`) whose two ends lie inside \
                     the tile proper — one where a single solved deck runs within \
                     {CARRY_LATERAL_M:.0} m of it in plan over {:.0} % of its length and meets \
                     it to within {CARRY_JOIN_M:.1} m at one end. The sample count is half the \
                     measurement: an ordinary footbridge is not a weak instance of this defect \
                     and is not counted, so a rule that puts sidewalks on their bridges empties \
                     this population rather than flattening it. Two coverage limits: a footway \
                     whose carrier is in the neighbouring tile is not measured, and the lateral \
                     test is to the carrier's drawn surface, so a deck narrower than the real \
                     carriageway shortens the reach.",
                    CARRY_COVER * 100.0
                ),
                detail: format!(
                    "How far a footway's own deck sinks below the solved deck alongside it. \
                     Overture tags a road bridge's separated sidewalk as a bridge in its own \
                     right, and a footway is a draped feature, so it is fitted a deck chorded \
                     to the ground at its span's ends — the ground *under* the bridge it is \
                     standing on. What is drawn is a second, smaller structure hanging \
                     beneath the real one, joined to it at whichever abutment the annotation \
                     and the DEM happen to agree on. Past {CARRY_M:.1} m — beyond any kerb or \
                     parapet — the two read as separate bridges."
                ),
                sense: Sense::HigherIsWorse,
                threshold: CARRY_M,
                skipped: self.carried.is_empty().then(|| {
                    "no fitted decks at this zoom — the extract carries no draped feature with \
                     an elevated span"
                        .to_string()
                }),
                dist: self.carried,
                worst: self.carried_worst.into_vec(),
            },
            Metric {
                id: "contact.rail_standoff".into(),
                invariant: Invariant::I4,
                title: "Drawn railway standing off the ground beneath it".into(),
                population: format!(
                    "Every vertex of every level-0 rail centerline inside the tile proper whose \
                     class names a gauge or a system, where the drawn terrain has a triangle \
                     under it. `unknown` rail is excluded — the archive carries no subtype, so \
                     that class is indistinguishable from an unrecognised road class. A vertex \
                     within {RAIL_ON_DECK_M:.1} m of a drawn structure surface is excluded too. \
                     Coverage limit: from z{} the union paves the formation and \
                     `pipeline::paves_via_union` deletes the rail stroke, so at the surface \
                     zooms this population is empty by construction — the band itself is \
                     measured where asphalt is, by the kerb and burial checks, and a \
                     formation-coverage successor (every ballast model arc covered by drawn \
                     band or deck) is on the roadmap. Measure a pre-surface rung to see this.",
                    crate::priors::ROAD_SURFACE_MIN_ZOOM
                ),
                detail: format!(
                    "Rail formation height minus the drawn ground directly under it. Distinct \
                     from `contact.kerb_lip`, which probes a metre *outside* a carriageway where \
                     an embankment legitimately drops away: this asks under the formation, which \
                     the bench is supposed to have raised to meet, so there is no legitimate \
                     positive answer. When the rail stroke existed at the detail zooms the \
                     population here was the *residue*: rail whose ballast band failed to mesh, \
                     so the drawn ground survived beneath the stroke. Past \
                     {RAIL_STANDOFF_M:.1} m the gap is wider than the detail lattice can \
                     explain and the track is in the air."
                ),
                sense: Sense::HigherIsWorse,
                threshold: RAIL_STANDOFF_M,
                skipped: self.rail.is_empty().then(|| {
                    format!(
                        "no rail stroke at this zoom — from z{} the union paves the formation \
                         and the stroke is deleted (pipeline::paves_via_union); measure a \
                         pre-surface rung to see this",
                        crate::priors::ROAD_SURFACE_MIN_ZOOM
                    )
                }),
                dist: self.rail,
                worst: self.rail_worst.into_vec(),
            },
        ]
    }
}

/// Signed clearance of one surface over another at a plan point, for callers
/// that already hold both. Kept here so the sign convention has one home.
pub fn clearance_over(upper: &SurfaceMesh, lower: &SurfaceMesh, px: f64, py: f64) -> Option<f64> {
    let u = upper.height_range_at(px, py)?.0;
    let l = lower.height_range_at(px, py)?.1;
    Some(u - l)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::project::Bounds;
    use crate::verify::mesh::Scale;
    use crate::verify::scene::RoadMesh;
    use std::collections::HashMap;

    const LIP: &str = "contact.kerb_lip";
    const UNWALLED: &str = "contact.kerb_unwalled";

    fn quad(x0: f32, x1: f32, z: f32) -> SurfaceMesh {
        SurfaceMesh::from_parts(
            vec![x0, x1, x1, x0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![z; 4],
            vec![0, 1, 2, 0, 2, 3],
        )
        .unwrap()
    }

    /// A vertical wall along `x`, spanning `lo..hi` — the shape `build_apron`
    /// emits: plan-degenerate, so only a `span_near` query can see it.
    fn wall(x: f32, lo: f32, hi: f32) -> SurfaceMesh {
        SurfaceMesh::from_parts(
            vec![x, x, x, x],
            vec![0.0, 1.0, 1.0, 0.0],
            vec![hi, hi, lo, lo],
            vec![0, 1, 2, 0, 2, 3],
        )
        .unwrap()
    }

    /// The shape the hole leaves: asphalt over `x < 0.5` at `road_m`, ground
    /// starting exactly at the kerb and lying at `ground_m` outside it, plus
    /// whatever `extra` features stand between them.
    fn kerbed(road_m: f32, ground_m: f32, extra: Vec<RoadMesh>) -> TileScene {
        let b = Bounds::of_tile(16, 34000, 23000);
        let mut roads = vec![RoadMesh {
            class: "road_surface".into(),
            level: 0,
            band: String::new(), fades: false, sheet: None,
            mesh: quad(0.0, 0.5, road_m),
        }];
        roads.extend(extra);
        TileScene {
            z: 16,
            x: 34000,
            y: 23000,
            scale: Scale::of(&b),
            bounds: b,
            terrain: Some(quad(0.5, 1.0, ground_m)),
            roads,
            lines: Vec::new(),
            waters: Vec::new(),
            buildings: Vec::new(),
        }
    }

    /// Metrics by id, so a test names what it asserts on and adding a metric
    /// does not silently repoint every index in the module.
    fn run(tile: &TileScene) -> HashMap<String, Metric> {
        let opt = Options { spacing_m: 1.0, ..Default::default() };
        let mut c = Box::new(Contact::new(&opt));
        c.visit(tile, &opt);
        c.finish().into_iter().map(|m| (m.id.clone(), m)).collect()
    }

    #[test]
    fn a_kerb_standing_on_nothing_is_caught_as_a_lip() {
        // Asphalt ten metres above the ground its kerb meets. The lip reports
        // the wall's height in metres, not a ratio — that is what makes it
        // readable as "the model implies a ten-metre retaining wall here".
        let m = run(&kerbed(110.0, 100.0, vec![]));
        let lip = &m[LIP];
        assert!(lip.violations() > 0, "a 10 m drop at the kerb must be caught");
        assert!(
            (lip.worst_value().unwrap() - 10.0).abs() < 0.5,
            "the wall's height, not a ratio: {:?}",
            lip.worst_value()
        );
    }

    #[test]
    fn a_road_level_with_the_ground_beside_it_has_no_lip() {
        // The common case, and the one that must not be reported: a bench holds
        // and the two surfaces meet at the kerb.
        let m = run(&kerbed(100.0, 100.0, vec![]));
        assert!(!m[LIP].dist.is_empty(), "the kerb must actually be walked");
        assert_eq!(m[LIP].violations(), 0, "a flush kerb is not a lip");
        assert_eq!(m[UNWALLED].violations(), 0, "and there is nothing to wall");
    }

    #[test]
    fn a_lip_with_no_apron_over_it_is_unwalled() {
        // The gate. The same ten-metre drop, with nothing drawn between the
        // asphalt and the terrain's rim: a hole you can see the hillside
        // through.
        let m = run(&kerbed(110.0, 100.0, vec![]));
        let unwalled = &m[UNWALLED];
        assert!(unwalled.violations() > 0, "an unspanned 10 m gap must be caught");
        assert!(
            (unwalled.worst_value().unwrap() - 10.0).abs() < 0.5,
            "the gap's own height: {:?}",
            unwalled.worst_value()
        );
        assert!(!unwalled.worst.is_empty(), "a violation must name a place");
    }

    #[test]
    fn an_apron_spanning_the_drop_closes_it() {
        // The same scene with the wall drawn. The lip is unchanged — the
        // earthwork is still ten metres tall, and that is honest — but nothing
        // is unwalled, which is the property the apron exists to give.
        let apron =
            RoadMesh { class: "road_apron".into(), level: 0, band: String::new(), fades: false, sheet: None, mesh: wall(0.5, 100.0, 110.0) };
        let m = run(&kerbed(110.0, 100.0, vec![apron]));
        assert!(m[LIP].violations() > 0, "the lip is a fact about the model, not a defect");
        assert_eq!(m[UNWALLED].violations(), 0, "the apron spans it, so nothing is unwalled");
    }

    #[test]
    fn an_apron_too_short_for_the_drop_does_not_close_it() {
        // A wall covering the top three metres of a ten-metre drop leaves seven
        // metres of sky. Spanning *part* of the gap must not count as spanning
        // it, or a truncated apron reads as a closed one.
        let apron =
            RoadMesh { class: "road_apron".into(), level: 0, band: String::new(), fades: false, sheet: None, mesh: wall(0.5, 107.0, 110.0) };
        let m = run(&kerbed(110.0, 100.0, vec![apron]));
        assert!(m[UNWALLED].violations() > 0, "a partial wall does not close the gap");
    }

    #[test]
    fn a_road_in_a_cutting_is_walled_the_other_way_round() {
        // Both directions. A road below the ground it is cut into opens the same
        // gap with the sign reversed, and the apron closes it the same way —
        // which is the case that only appeared once the carriageway stopped
        // being clamped up to the terrain (`road::on_ground`).
        let m = run(&kerbed(100.0, 108.0, vec![]));
        assert!(m[UNWALLED].violations() > 0, "a cutting leaves the same open gap");
        let apron =
            RoadMesh { class: "road_apron".into(), level: 0, band: String::new(), fades: false, sheet: None, mesh: wall(0.5, 100.0, 108.0) };
        let m = run(&kerbed(100.0, 108.0, vec![apron]));
        assert_eq!(m[UNWALLED].violations(), 0, "and the same apron closes it");
    }

    #[test]
    fn a_tile_without_terrain_is_skipped_rather_than_scored_clean() {
        let mut tile = kerbed(110.0, 100.0, vec![]);
        tile.terrain = None;
        let m = run(&tile);
        assert!(m[LIP].skipped.is_some(), "no ground must read as skipped, not as passing");
        assert_eq!(m[LIP].violations(), 0);
    }

    #[test]
    fn buffer_geometry_is_the_neighbours_business() {
        // A road running well past the tile edge must contribute only the part
        // inside the tile proper, or every border defect is counted twice.
        let b = Bounds::of_tile(16, 34000, 23000);
        let tile = TileScene {
            z: 16,
            x: 34000,
            y: 23000,
            scale: Scale::of(&b),
            bounds: b,
            terrain: Some(quad(0.5, 1.5, 100.0)),
            roads: vec![RoadMesh {
                class: "road_surface".into(),
                level: 0,
                band: String::new(), fades: false, sheet: None,
                mesh: quad(-0.4, 0.5, 110.0),
            }],
            lines: Vec::new(),
            waters: Vec::new(),
            buildings: Vec::new(),
        };
        let m = run(&tile);
        // The kerb at x = 0.5 is inside the tile and reported; the outer edge at
        // x = -0.4 is the neighbour's.
        assert!(m[LIP].violations() > 0, "the interior kerb must be walked");
        for o in &m[LIP].worst {
            let (lon, _) = (o.lon, o.lat);
            assert!(lon >= b.west && lon <= b.east, "offender at {lon} is outside the tile");
        }
    }

    /// A terrain profile as a strip of quads along `x`, from `(x, height)`
    /// breakpoints — the ground a deck's abutment is judged against.
    fn ground_strip(pts: &[(f32, f32)]) -> SurfaceMesh {
        let (mut x, mut y, mut z, mut idx) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
        for (i, &(px, pz)) in pts.iter().enumerate() {
            x.extend([px, px]);
            y.extend([-1.0, 2.0]);
            z.extend([pz, pz]);
            if i > 0 {
                let b = (i as u32 - 1) * 2;
                idx.extend([b, b + 1, b + 3, b, b + 3, b + 2]);
            }
        }
        SurfaceMesh::from_parts(x, y, z, idx).unwrap()
    }

    /// A deck: a slab from `x0` to `x1` with its top running `z0` to `z1`.
    fn deck(class: &str, x0: f32, x1: f32, z0: f32, z1: f32) -> RoadMesh {
        RoadMesh {
            class: class.into(),
            level: 1,
            band: String::new(), fades: false, sheet: None,
            mesh: SurfaceMesh::from_parts(
                vec![x0, x1, x1, x0],
                vec![0.4, 0.4, 0.6, 0.6],
                vec![z0, z1, z1, z0],
                vec![0, 1, 2, 0, 2, 3],
            )
            .unwrap(),
        }
    }

    fn with_ground(terrain: SurfaceMesh, roads: Vec<RoadMesh>) -> TileScene {
        let b = Bounds::of_tile(16, 34000, 23000);
        TileScene {
            z: 16,
            x: 34000,
            y: 23000,
            scale: Scale::of(&b),
            bounds: b,
            terrain: Some(terrain),
            roads,
            lines: Vec::new(),
            waters: Vec::new(),
            buildings: Vec::new(),
        }
    }

    const SEAT: &str = "contact.deck_seat";

    /// The defect: a footbridge whose lower end fell down a gorge wall, so it
    /// starts part way into what it is supposed to cross. The tile is ~600 m
    /// wide, so 0.02 of unit space is about 12 m.
    #[test]
    fn a_deck_that_starts_down_a_wall_is_caught() {
        // Bank at 435 to x = 0.30, a wall to the riverbed at 415, far bank at
        // 435. The deck runs from 421 — part way down the near wall — to 435.
        let terrain = ground_strip(&[
            (0.0, 435.0),
            (0.30, 435.0),
            (0.33, 415.0),
            (0.37, 415.0),
            (0.40, 435.0),
            (1.0, 435.0),
        ]);
        let m = run(&with_ground(terrain, vec![deck("footway", 0.32, 0.40, 421.0, 435.0)]));
        assert!(m[SEAT].violations() > 0, "an abutment 14 m down a wall must be caught");
        assert!(
            m[SEAT].worst_value().unwrap() > 5.0,
            "the wall above it is metres, not centimetres: {:?}",
            m[SEAT].worst_value()
        );
    }

    /// The same bridge, seated: both ends on their banks, the deck level over
    /// the gorge. Nothing to report, though the ground under mid-span is 20 m
    /// below the deck — which is what a bridge is.
    #[test]
    fn a_deck_seated_on_its_banks_is_clean() {
        let terrain = ground_strip(&[
            (0.0, 435.0),
            (0.30, 435.0),
            (0.33, 415.0),
            (0.37, 415.0),
            (0.40, 435.0),
            (1.0, 435.0),
        ]);
        let m = run(&with_ground(terrain, vec![deck("footway", 0.29, 0.41, 435.0, 435.0)]));
        assert_eq!(m[SEAT].violations(), 0, "a level deck bank to bank is not a defect");
    }

    /// A level footbridge on a mountainside keeps its clean bill: the ground
    /// climbing away above it is the mountain, and the deck's own far end —
    /// level with the near one — is the bound that says so.
    #[test]
    fn a_level_deck_on_a_hillside_does_not_score_the_hillside() {
        let terrain = ground_strip(&[(0.0, 560.0), (0.5, 500.0), (1.0, 440.0)]);
        let m = run(&with_ground(terrain, vec![deck("path", 0.48, 0.52, 502.4, 502.4)]));
        assert_eq!(
            m[SEAT].violations(),
            0,
            "a 100 % hillside above a level deck is terrain, not a defect: {:?}",
            m[SEAT].worst_value()
        );
    }

    const CARRIED: &str = "contact.deck_carried";

    /// A slab from `x0..x1` and `y0..y1`, its top running `z0` to `z1`.
    fn slab(class: &str, x0: f32, x1: f32, y0: f32, y1: f32, z0: f32, z1: f32) -> RoadMesh {
        RoadMesh {
            class: class.into(),
            level: 1,
            band: String::new(), fades: false, sheet: None,
            mesh: SurfaceMesh::from_parts(
                vec![x0, x1, x1, x0],
                vec![y0, y0, y1, y1],
                vec![z0, z1, z1, z0],
                vec![0, 1, 2, 0, 2, 3],
            )
            .unwrap(),
        }
    }

    /// A tile holding decks and no ground: the carried check compares two
    /// structures and reads no terrain, so it must answer without one.
    fn decks_only(roads: Vec<RoadMesh>) -> TileScene {
        let b = Bounds::of_tile(16, 34000, 23000);
        TileScene {
            z: 16,
            x: 34000,
            y: 23000,
            scale: Scale::of(&b),
            bounds: b,
            terrain: None,
            roads,
            lines: Vec::new(),
            waters: Vec::new(),
            buildings: Vec::new(),
        }
    }

    /// The shape Overture gives a road bridge with a separated sidewalk: a
    /// 55 m residential deck, and 3 m beside it a footway tagged as a bridge
    /// of its own. `sink` is how far the footway's far end falls below the
    /// deck it joins at the near one. The tile is ~546 m across and ~304 m
    /// up, so the y offsets here are metres, not a coincidence of scale.
    fn sidewalk(sink: f32) -> Vec<RoadMesh> {
        vec![
            slab("residential", 0.45, 0.55, 0.4934, 0.5066, 500.0, 500.0),
            slab("footway", 0.45, 0.55, 0.5124, 0.5206, 500.0, 500.0 - sink),
        ]
    }

    /// The defect: the sidewalk is fitted to the ground *under* the bridge it
    /// is standing on, so it joins the deck at one abutment and dives away
    /// from it at the other.
    #[test]
    fn a_sidewalk_sunk_below_its_road_bridge_is_caught() {
        let m = run(&decks_only(sidewalk(4.0)));
        assert!(m[CARRIED].violations() > 0, "a footway 4 m under its bridge must be caught");
        let worst = m[CARRIED].worst_value().unwrap();
        assert!((worst - 4.0).abs() < 0.5, "the sink in metres: {worst:?}");
    }

    /// The same sidewalk, riding the deck that carries it. Still in the
    /// population — it is a carried deck — but nothing to report.
    #[test]
    fn a_sidewalk_level_with_its_bridge_is_clean() {
        let m = run(&decks_only(sidewalk(0.0)));
        assert_eq!(m[CARRIED].dist.count(), 1, "a carried deck is measured either way");
        assert_eq!(m[CARRIED].violations(), 0, "level with its carrier is not a defect");
    }

    /// A footbridge passing *underneath* a viaduct is not its sidewalk, and
    /// this is the case that made the end-agreement test necessary: without it
    /// the worst offender in the extract is a 12 m footbridge under a motorway
    /// whose deck is 68 m overhead.
    #[test]
    fn a_footbridge_under_a_viaduct_is_not_carried() {
        let mut roads = sidewalk(0.0);
        roads[1] = slab("footway", 0.45, 0.55, 0.5124, 0.5206, 460.0, 460.0);
        let m = run(&decks_only(roads));
        assert!(m[CARRIED].skipped.is_some(), "40 m below is passing under, not riding on");
    }

    /// A footbridge with no structure beside it is an ordinary footbridge, and
    /// not a weak instance of this defect: it is not in the population at all.
    #[test]
    fn a_lone_footbridge_is_not_in_the_population() {
        let m = run(&decks_only(vec![slab(
            "footway", 0.45, 0.55, 0.5124, 0.5206, 500.0, 496.0,
        )]));
        assert!(m[CARRIED].skipped.is_some(), "nothing is carrying it: {:?}", m[CARRIED].dist.count());
    }

    /// A road bridge 25 m away carries nothing. The lateral test is what says
    /// so, and the probe's finding is that it has a wide empty band to sit in.
    #[test]
    fn a_deck_too_far_to_one_side_does_not_carry() {
        let mut roads = sidewalk(4.0);
        roads[0] = slab("residential", 0.45, 0.55, 0.5988, 0.6120, 500.0, 500.0);
        let m = run(&decks_only(roads));
        assert!(m[CARRIED].skipped.is_some(), "a deck 25 m off is a different bridge");
    }

    /// A footway crossing a bridge at right angles shares one deck width with
    /// it, not its length. Coverage is the test that says so — without it the
    /// join and lateral tests alone would call every path near an abutment a
    /// sidewalk.
    #[test]
    fn a_footway_crossing_a_bridge_is_not_carried_by_it() {
        let m = run(&decks_only(vec![
            slab("residential", 0.45, 0.55, 0.4934, 0.5066, 500.0, 500.0),
            // Across the deck rather than along it, and long enough that the
            // shared run is a small fraction of it.
            slab("footway", 0.4975, 0.5025, 0.45, 0.55, 500.0, 496.0),
        ]));
        assert!(m[CARRIED].skipped.is_some(), "crossing a bridge is not riding it");
    }

    /// A solved deck is not this check's business: its abutments come from a
    /// profile with anchors and a grade ceiling, not from the ground at an
    /// annotation's edge.
    #[test]
    fn a_solved_deck_is_not_in_the_population() {
        let terrain = ground_strip(&[(0.0, 435.0), (0.30, 435.0), (0.33, 415.0), (1.0, 415.0)]);
        let m = run(&with_ground(terrain, vec![deck("motorway", 0.32, 0.40, 421.0, 435.0)]));
        assert!(m[SEAT].skipped.is_some(), "no fitted decks here: {:?}", m[SEAT].dist.count());
    }

    const RAIL: &str = "contact.rail_standoff";

    /// A tile whose whole extent is flat ground at `ground_m`, carrying one
    /// level-0 centerline of `class` at `line_m`, plus any extra meshes.
    fn railed(class: &str, ground_m: f32, line_m: f64, extra: Vec<RoadMesh>) -> TileScene {
        let b = Bounds::of_tile(16, 34000, 23000);
        TileScene {
            z: 16,
            x: 34000,
            y: 23000,
            scale: Scale::of(&b),
            bounds: b,
            terrain: Some(quad(0.0, 1.0, ground_m)),
            roads: extra,
            lines: vec![crate::verify::scene::RoadLine {
                class: class.into(),
                level: 0,
                width_m: 0.0,
                parts: vec![vec![(0.3, 0.5, line_m), (0.5, 0.5, line_m), (0.7, 0.5, line_m)]],
            }],
            waters: Vec::new(),
            buildings: Vec::new(),
        }
    }

    #[test]
    fn a_railway_in_the_air_is_caught() {
        let m = run(&railed("narrow_gauge", 800.0, 812.0, vec![]));
        let rail = &m[RAIL];
        assert!(rail.violations() > 0, "a 12 m gap under a railway must be caught");
        assert!(
            (rail.worst_value().unwrap() - 12.0).abs() < 0.5,
            "the height of the air under it, in metres: {:?}",
            rail.worst_value()
        );
    }

    /// The formation is where the ground is. Nothing to report, and the
    /// population is still counted — a zero is evidence, a skip is not.
    #[test]
    fn a_railway_on_its_formation_scores_zero() {
        let m = run(&railed("standard_gauge", 800.0, 800.0, vec![]));
        assert!(m[RAIL].skipped.is_none() && m[RAIL].dist.count() > 0, "the samples are the proof");
        assert_eq!(m[RAIL].violations(), 0);
    }

    /// The population trap: a viaduct's paint stroke reaches the archive at
    /// level 0 because the level ordinal is attached after it is emitted
    /// (`pipeline.rs`). It is metres above the ground because that is what a
    /// viaduct is, and counting it would measure the emit order.
    #[test]
    fn paint_riding_a_viaduct_is_not_a_float() {
        let m = run(&railed(
            "narrow_gauge",
            800.0,
            812.0,
            vec![RoadMesh { class: "narrow_gauge".into(), level: 1, band: String::new(), fades: false, sheet: None, mesh: quad(0.0, 1.0, 812.0) }],
        ));
        assert!(m[RAIL].skipped.is_some(), "a deck under the stroke carries it: {:?}", m[RAIL].dist.count());
    }

    /// `unknown` rail is excluded, because the archive carries no subtype and
    /// the class is then indistinguishable from a road class the parser does
    /// not recognise — the same reason `priors` gives it the junior default.
    #[test]
    fn an_unclassified_railway_is_not_in_the_population() {
        let m = run(&railed("unknown", 800.0, 812.0, vec![]));
        assert!(m[RAIL].skipped.is_some(), "no gauge, no measurement");
    }

    /// A road is not measured here. It has a kerb, a hole and an apron, and
    /// `contact.kerb_lip` is where its drop is reported; folding it in would
    /// pool two populations with different legitimate bands.
    #[test]
    fn a_road_centerline_is_not_measured_as_rail() {
        let m = run(&railed("residential", 800.0, 812.0, vec![]));
        assert!(m[RAIL].skipped.is_some(), "a street is the kerb metric's business");
    }
}
