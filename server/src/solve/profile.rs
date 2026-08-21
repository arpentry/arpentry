//! The solved vertical model of one corridor — the road's own (gentle)
//! elevation profile through its bridges and tunnels (docs/GENERATION.md §5
//! stage 3).
//!
//! The terrain along a corridor is wild — the Viaduc de Chillon's centerline
//! DEM drops into a ravine (a bridge spans it ~70 m up) and rears up to a
//! 680 m hill (a tunnel pierces it) — while the road itself holds a gentle
//! highway grade (~3 %). The road surface is therefore anchored to a
//! reconstructed *road profile*:
//!
//! 1. Sample the reference terrain along the whole corridor.
//! 2. Where the road is at grade it sits on the ground, so those nodes are
//!    elevation *anchors*: `profile = terrain`.
//! 3. Across every bridge/tunnel span, the profile is a straight (gentle)
//!    interpolation between the bounding anchors — the grade the road actually
//!    holds — independent of the terrain excursion under it.
//!
//! A structure thus rides the gentle profile: where the terrain dips below it
//! (a ravine) a bridge deck stands high above the floor — the visible viaduct;
//! where the terrain rises above it (a flank, a tunnelled hill) the body
//! passes below ground and the terrain, drawn first and owning the depth
//! buffer, occludes it.
//!
//! The profile is solved once, over the whole corridor, before tiling. Every
//! tile fragment of the corridor reads the same [`Profile`] through
//! `Arc<SolvedModel>`, so heights are a function of the global model only —
//! never of the tile window — and seams line up by construction
//! (docs/GENERATION.md I5).

use geo_types::Coord;

use crate::priors::{
    BUMP_SHAVE_MAX_M, BUMP_SPAN_M, MAX_ROAD_DEVIATION_M, MIN_EARTHWORK_M, NOTCH_FILL_MAX_M,
    NOTCH_SPAN_M, PORTAL_MAX_M,
};
use crate::scene::{metric_len, run_cos_lat, Span, SpanKind, DEG_M};

pub use crate::priors::NODE_SPACING_M;

/// Cap on densified vertices per corridor — a runaway guard for pathological
/// inputs; real corridors are bounded by `priors::MAX_CORRIDOR_M`.
const MAX_NODES: usize = 65_536;

/// Half-window ceiling in metres of the local quadratic regression that smooths
/// the centerline before sweeping. A digitised road line carries lateral vertex
/// wiggle out to ~120 m wavelength — the scales that read as a snaking deck
/// edge — so the window wants to span a full period of the worst of it to
/// average it away. It is a *ceiling* because on anything but a straight road
/// the turn budget below binds first.
const SMOOTH_WINDOW_M: f64 = 100.0;

/// How much the road may turn, in radians, between the node being fitted and
/// either edge of its window.
///
/// **This is what keeps a curve a curve.** The fit is a quadratic in arc length
/// per coordinate, which is a parabola in the plane, and a parabola matches a
/// circular arc only while the arc is shallow: fitting a radius `R` over a half
/// window `L` displaces the fitted point at the centre by about `L⁴/(280·R³)`,
/// which is `R·Δθ⁴/280` at a turn of `Δθ = L/R`. The error is therefore
/// governed by the *angle* the window spans, not by its length — and a fixed
/// 100 m half-window spans 4 radians on a 50 m corner, where no polynomial in
/// any frame can follow the road at all.
///
/// That is exactly what the fixed window did: measured on the Montreux extract,
/// nodes on radii under 60 m were displaced a median 4.00 m — the deviation
/// clamp, saturated, meaning the fit wanted to cut the corner even further —
/// and the whole network sat a median 0.84 m off its own centerline. Held to
/// 0.6 rad the same formula gives under 0.1 m out to a 200 m radius and
/// millimetres on anything straighter, while a 50 m corner keeps a ±30 m
/// window, which is several vertices and still averages its own scale of
/// wiggle. A corner is a short feature; its window should be short too.
///
/// The turn is accumulated *signed*, so digitising zigzag — which is what the
/// smoothing exists to remove — cancels instead of closing the window at
/// precisely the places that need it open.
const SMOOTH_MAX_TURN_RAD: f64 = 0.6;

/// Shortest chord, in metres, that a heading is read over when measuring how far
/// the road has turned. Long enough to span a period of the lateral wiggle the
/// smoothing exists to remove — so that wiggle cancels out of the turn instead
/// of closing the window on it — and short enough to still resolve a real bend
/// at the scale [`SMOOTH_MAX_TURN_RAD`] budgets.
const HEADING_CHORD_M: f64 = 15.0;

/// Passes of the local quadratic regression. Each pass deepens the noise
/// suppression; curves survive every pass (the fit reproduces them), so a
/// few passes cost nothing but time.
const SMOOTH_PASSES: usize = 2;

/// Safety cap on how far smoothing may displace a centerline node from its
/// input position, in metres — a backstop on the shapes the turn budget cannot
/// bound, a reversal inside a single vertex above all. With the window held to
/// [`SMOOTH_MAX_TURN_RAD`] the fit's own error is decimetres, so this stops
/// being the thing that decides where a road goes and goes back to being what
/// its name says.
const SMOOTH_MAX_DEV_M: f64 = 1.0;

/// Relaxation passes for [`limit_road_grade`]. A handful alternating forward
/// and backward spreads a steep pitch's deviation evenly — a cut on the way
/// up, a fill on the way down — instead of letting one anchored direction
/// drift to one side.
const GRADE_PASSES: usize = 8;

/// Passes of [`smooth_vgrades`] vertical-curve smoothing. A handful of
/// arc-weighted Laplacian passes rounds the sharp grade breaks an engineered
/// profile carries — the abutment corner where an embankment approach meets a
/// deck ramp, the kinks the grade limiter leaves on a cut or fill — into gentle
/// sag/crest curves. A straight ramp is harmonic and passes through untouched;
/// only the corners move, and they round out within ~√passes nodes.
const VGRADE_PASSES: usize = 6;

/// Relaxation weight per [`smooth_vgrades`] pass: each engineered node moves
/// this fraction of the way to the arc-chord of its neighbours.
const VGRADE_LAMBDA: f64 = 0.5;

/// Passes of [`absorb_infeasible_anchors`] + re-solve. Each pass may extend a
/// structure run by up to [`PORTAL_MAX_M`] past its current edge, so a long
/// infeasible flank (a gorge wall the road pierces obliquely) is absorbed in
/// a few steps; the cap bounds the creep on pathological terrain.
const ABSORB_PASSES: usize = 8;

/// Percentile of the ground's own grade, along the stretches the data maps as
/// at grade, taken as the alignment's measured grade in [`measured_grade`].
/// High enough that the line's steepest sustained character counts, below 1 so
/// a single DEM step or a digitising spike cannot set the ceiling by itself.
const MEASURED_GRADE_PCTL: f64 = 0.90;

/// Least mapped-at-grade length a corridor must carry before its ground is
/// read as a measurement of its grade. Below this the sample is a handful of
/// edges beside a structure — exactly the cliff [`absorb_infeasible_anchors`]
/// exists to catch — and the class prior stands.
const MEASURED_GRADE_MIN_M: f64 = 100.0;

/// Floor of the plausibility cap on the grade the ground may claim: beyond
/// this the read is a DEM defect or a mis-chained corridor, not an alignment.
/// Sized for the classes whose convention is gentle — a motorway prior of 6 %
/// and a narrow-gauge one of 7 % both cover the steepest rack railway here at
/// 22 % with room to spare, and nothing a *road* is mapped along sustains a
/// third of its length past it.
const MEASURED_GRADE_MAX: f64 = 0.30;

/// How far past its own convention a class's ground may be believed, when the
/// class is steeper than [`MEASURED_GRADE_MAX`]. A funicular's prior is 45 %
/// and the Territet–Glion bed measures 56.9 %: a bound that cannot admit that
/// would clip the very alignment the prior was written for, and one with no
/// class in it would let a 7 % railway claim a cliff.
const MEASURED_GRADE_HEADROOM: f64 = 1.5;

/// A solved at-grade pitch steeper than this multiple of the class grade
/// ceiling, adjacent to a structure, marks the stretch as infeasible — the
/// deviation budget lost to the terrain there, so the road cannot in fact be
/// at grade. Above 1 so that the limiter's small relaxation residue (and a
/// genuinely firm pitch a real road would hold) is left alone; the disease
/// this hunts is a 3–10× violation at a cliff.
const ABSORB_GRADE_FACTOR: f64 = 1.5;

/// How far a solved at-grade road may stand clear of the natural terrain,
/// beside a mapped structure and in the same direction that structure leaves
/// the ground, before it is read as part of that structure rather than as an
/// embankment or a cutting. Well above the ordinary approach berm (the p99
/// standoff across a dense network is ~2.5 m) and below the tallest
/// embankments a road really is built on, so only annotation shortfall — a
/// viaduct mapped as a single short span over the road it crosses — is caught.
pub(crate) const ABSORB_STANDOFF_M: f64 = 5.0;

/// How far outward of a structure edge [`seek_rim_anchors`] may migrate the
/// bounding anchor to the local terrain extremum — the gorge rim a deck
/// launches from, the flank base a bore emerges at. The disease this cures is
/// the anchor point-sampling a DEM roll-off (the smoothed shoulder between a
/// plateau and the wall below), a one-to-few-lattice-cell artifact, so the
/// reach is a few profile nodes — far under the annotation-trust radius.
const ANCHOR_SEEK_M: f64 = 32.0;

/// Terrain excursion inside the span (relative to its bounding anchor) that
/// classifies the structure side as flying (a deck to launch high) or buried
/// (a bore to emerge low) for the anchor seek. Below it the ground is
/// effectively flat and the anchor is left where the annotation put it.
const SEEK_GAP_M: f64 = 2.0;

/// Least anchor improvement worth migrating for — under this the roll-off is
/// cosmetically flat and moving the abutment buys nothing.
const SEEK_MIN_GAIN_M: f64 = 0.5;

/// A corridor's solved surface profile: a densified centerline with a per-node
/// road-surface height. Evaluated at an arbitrary point by projecting it onto
/// the corridor, so independent tile fragments of one structure compute the
/// same heights and the seams line up.
///
/// This surface is the single road model: a bridge deck and a tunnel bore both
/// ride it (their road face *is* this surface), so where a bridge meets a
/// tunnel or an approach road the road is continuous (invariant 2).
pub struct Profile {
    /// Densified corridor nodes in (lon, lat).
    nodes: Vec<Coord>,
    /// `nodes` low-pass-smoothed (endpoint-preserving), the line a deck box is
    /// swept along so it follows the road without tracing every digitising
    /// wiggle.
    smooth: Vec<Coord>,
    /// Cumulative metric arc length at each node (`arc[0] == 0`).
    arc: Vec<f64>,
    /// Road-surface height in metres above the ellipsoid at each node.
    road_m: Vec<f64>,
    /// Deck-top height in metres at each node: [`road_m`](Self::road_m) with
    /// each structure span replaced by a single straight ramp fit over that
    /// span (the at-grade nodes keep the draped road height). This is the
    /// height swept deck/bore boxes ride: one ramp per *global* span, so no
    /// tile sliver fits its own steep line and no seam steps.
    deck_m: Vec<f64>,
    /// Reference terrain height in metres at each node — the same samples the
    /// at-grade anchors are built from. `road_m − terrain_m` is the signed gap
    /// that says where the road stands proud (a visible deck) or runs buried
    /// (a bore); a tunnel's portal is exactly that gap's zero crossing.
    terrain_m: Vec<f64>,
    /// Whether each node lies in an at-grade span (an anchor) or a structure.
    at_grade: Vec<bool>,
    /// The grade ceiling this profile was actually solved to — the class prior
    /// raised to the alignment's own measured grade where the ground says so
    /// ([`measured_grade`]). `None` for a draped class, which holds no grade.
    /// The relaxation must hold the corridor's edges to *this*, not to the
    /// prior, or it undoes the profile it was warm-started from.
    max_grade: Option<f64>,
    /// `cos(mean latitude)`, scaling longitude into the local metric space.
    cos_lat: f64,
}

/// One swept deck cross-section: the (smoothed) centerline position, the
/// deck-top height, the unit left-perpendicular (ENU metres) the section
/// spans, and the *global* corridor arc it sits at — the reference pier
/// placement snaps to, so tile fragments of one viaduct plant identical
/// piers.
pub struct DeckNode {
    pub lon: f64,
    pub lat: f64,
    pub height_m: f64,
    pub left_e: f64,
    pub left_n: f64,
    pub arc_m: f64,
}

/// How a corridor's class shapes its solve (docs/GROUND.md §1). One vertical
/// model, three parameterisations: an engineered class gets the full
/// treatment; a drivable street holds a plausible bed grade within a tight
/// deviation budget; a non-drivable corridor that still carries structures
/// (rail, a footpath with a bridge) drapes its at-grade spans as they are.
#[derive(Clone, Copy, Debug)]
pub enum Mode {
    /// Rim anchoring, the class grade ceiling, infeasible-anchor absorption.
    Engineered { grade: f64 },
    /// A bed grade held within `deviation_m` of the conditioned reference,
    /// with symmetric vertical smoothing; anchors stay where mapped (S9).
    Street { grade: f64, deviation_m: f64, spacing_m: f64 },
    /// At-grade spans drape as they are; structures chord between anchors.
    Draped,
}

impl Mode {
    /// The mode a corridor's [`Kind`](crate::priors::Kind) implies, from its
    /// §9 `grade_shape` alone.
    ///
    /// The shape *is* the mode: a constant gradient is not a very steep
    /// ceiling, and a surveyed alignment is not a street with a tighter one.
    pub fn for_kind(kind: crate::priors::Kind) -> Mode {
        use crate::priors::GradeShape;
        let prior = kind.prior();
        match prior.grade_shape {
            // A surveyed alignment — a motorway, a trunk road, a railway — gets
            // rim anchoring, the class ceiling, and infeasible-anchor
            // absorption into structures.
            _ if prior.engineered => Mode::Engineered { grade: prior.grade().unwrap_or(0.06) },
            // A bed grade held inside a tight deviation budget: the street
            // trusts the hill it was laid on (S9).
            GradeShape::Bounded(grade) => Mode::Street {
                grade,
                deviation_m: prior.deviation_m,
                spacing_m: prior.node_spacing_m,
            },
            // No profile at all. Unreachable from the scene — the gate admits
            // only strata that solve — and kept as the degradation floor.
            _ => Mode::Draped,
        }
    }

    /// The grade this mode holds before the ground is consulted. `None` for a
    /// draped class, which holds none.
    pub fn grade(self) -> Option<f64> {
        match self {
            Mode::Engineered { grade } | Mode::Street { grade, .. } => Some(grade),
            Mode::Draped => None,
        }
    }

    /// Profile node spacing, metres — the street classes sample sparsely to
    /// bound the network-wide solve.
    fn spacing_m(self) -> f64 {
        match self {
            Mode::Street { spacing_m, .. } => spacing_m,
            _ => NODE_SPACING_M,
        }
    }
}

/// Solves the surface profile of one corridor: densify, sample the reference
/// terrain through `elev`, anchor the road at the at-grade spans, interpolate
/// the gentle profile across the structures, and hold the road to its mode's
/// grade and deviation budget. `None` for a degenerate corridor. The terrain
/// sampler is injected so tests can bypass the DEM.
pub fn solve(
    nodes: &[Coord],
    spans: &[Span],
    mode: Mode,
    elev: &mut dyn FnMut(Coord) -> f64,
) -> Option<Profile> {
    if nodes.len() < 2 {
        return None;
    }
    let raw = nodes;
    let cos_lat = run_cos_lat(raw);
    let (nodes, arc, params) = densify(raw, cos_lat, mode.spacing_m());
    let n = nodes.len();
    if n < 2 {
        return None;
    }
    let terrain: Vec<f64> = nodes.iter().map(|c| elev(*c)).collect();
    // The road's anchor surface: the terrain conditioned symmetrically
    // ([`condition_reference`]) — narrow notches filled (a mapped at-grade
    // road spans a gully on engineered fill instead of diving through the
    // DEM's image of it) and narrow bumps shaved (it was graded through
    // canopy shadows and upsampling ripple instead of climbing them). The
    // raw `terrain` keeps every structural read (rim seeking, daylighting,
    // pier footing, the earthworks' cut/fill depths).
    let road_ref = condition_reference(&arc, &terrain);
    let mut at_grade: Vec<bool> =
        arc.iter().map(|&a| kind_at(spans, a) == SpanKind::Grade).collect();
    // The alignment's own grade, read before `seek_rim_anchors` moves an
    // anchor: the question is what the *data* maps as lying on the ground.
    let measured = measured_grade(&arc, &road_ref, &at_grade, mode.grade().unwrap_or(0.0));
    let mut max_grade = mode.grade();
    let mut road_m;
    match mode {
        Mode::Engineered { grade: g } => {
            // A ceiling that the ground under the mapped alignment already
            // beats is not a ceiling, it is a contradiction; the ground wins.
            let g = g.max(measured.unwrap_or(0.0));
            max_grade = Some(g);
            // Robust structure anchors: snap each structure-bounding anchor to
            // the local terrain extremum (a bridge launches from the rim crest,
            // a bore emerges at the flank base) before any chord is fit — an
            // anchor point-sampled on a rim roll-off otherwise launches the
            // deck metres below the rim and digs the approach into a cut to
            // reach it. Engineered classes only: an unengineered road drapes
            // the terrain whatever it does, and moving its anchors just
            // reshapes its (already steep, genuine) pitches.
            seek_rim_anchors(&arc, &terrain, &mut at_grade);
            let solve_once = |road_m: &mut Vec<f64>, at_grade: &[bool]| {
                *road_m = road_profile(&arc, &road_ref, at_grade);
                limit_road_grade(&arc, road_m, &road_ref, at_grade, g, MAX_ROAD_DEVIATION_M);
                rechord_structures(&arc, road_m, at_grade);
            };
            road_m = Vec::new();
            solve_once(&mut road_m, &at_grade);
            // Solved structure ends (S5's trust model, applied to the profile):
            // where the solved road still pitches far beyond the grade ceiling
            // right beside a structure, the annotation ended before the road
            // actually reached the ground — a bridge landing into a gorge
            // wall, a tunnel emerging under a climbing flank. The deviation
            // budget lost to the terrain there, so no at-grade road can exist:
            // absorb the stretch into the structure run and re-solve, until
            // the profile is feasible. The spans are grown to match at
            // reconcile time (`portals::grow_spans`), so sweeps and paint
            // follow.
            for _ in 0..ABSORB_PASSES {
                if !absorb_infeasible_anchors(&arc, &mut at_grade, &road_m, &terrain, g) {
                    break;
                }
                solve_once(&mut road_m, &at_grade);
            }
        }
        // A street holds its bed grade within the class deviation budget —
        // the relaxation that irons DEM noise flat while a genuinely climbing
        // street still climbs (S9). Its mapped anchors stay put (no rim
        // seeking, no absorption: an unengineered annotation is trusted), and
        // any structure chord is re-pinned to where the relaxed road arrives.
        Mode::Street { grade, deviation_m, .. } => {
            // The measured bed raises the cap here for the same reason it does
            // for the engineered classes: a ceiling the ground under the mapped
            // alignment already beats is a contradiction, and the ground wins.
            // The street case that forces it is a junction chain down a flank
            // steeper than the class bed grade — the Chauderon gorge lanes
            // measure 23–41 % under a 15 % cap. The published `max_grade` is
            // also the fused graph's edge cap (`graph::build`), and held at
            // the class number there, the junction welds won over the
            // ground-hugging box and the chain hung 26 m over its own hillside
            // at grade.
            let g = grade.max(measured.unwrap_or(0.0));
            max_grade = Some(g);
            road_m = road_profile(&arc, &road_ref, &at_grade);
            limit_road_grade(&arc, &mut road_m, &road_ref, &at_grade, g, deviation_m);
            rechord_structures(&arc, &mut road_m, &at_grade);
        }
        Mode::Draped => {
            road_m = road_profile(&arc, &road_ref, &at_grade);
        }
    }
    // Round the profile's grade breaks (abutments, cut/fill kinks) into
    // gentle vertical curves. Engineered classes move only nodes already
    // lifted off the ground (draped nodes stay pinned); streets smooth every
    // at-grade node within their deviation budget — the symmetric low-pass
    // that removes residual wobble without floating the street. Runs before
    // the deck ramp so decks stay straight, and before portals/ground read
    // the profile so their gap zero-crossings stay exact.
    match mode {
        Mode::Engineered { .. } => smooth_vgrades(&arc, &mut road_m, &road_ref, &at_grade),
        Mode::Street { deviation_m, .. } => {
            smooth_vgrades_street(&arc, &mut road_m, &road_ref, &at_grade, deviation_m)
        }
        Mode::Draped => {}
    }
    let deck_m = deck_ramp(&arc, &road_m, &at_grade);
    let smooth = smooth_path(&spline_path(raw, &params, cos_lat));
    Some(Profile {
        nodes,
        smooth,
        arc,
        road_m,
        deck_m,
        terrain_m: terrain,
        at_grade,
        max_grade,
        cos_lat,
    })
}

/// The span kind at arc position `a` (grade when no span covers it).
fn kind_at(spans: &[Span], a: f64) -> SpanKind {
    spans
        .iter()
        .find(|s| a >= s.arc0 && a <= s.arc1)
        .map_or(SpanKind::Grade, |s| s.kind)
}

impl Profile {
    /// Road-surface height in metres above the ellipsoid at `(lon, lat)`,
    /// found by projecting the point onto the nearest corridor edge and
    /// interpolating. Clipped fragment vertices all lie on the corridor, so
    /// the nearest on-corridor height is exact.
    pub fn height_at(&self, lon: f64, lat: f64) -> f64 {
        project_onto(&self.nodes, &self.road_m, self.cos_lat, lon, lat)
    }

    /// Reference terrain height in metres at `(lon, lat)`, projected onto the
    /// corridor exactly as [`height_at`](Self::height_at) projects the road,
    /// so the two share a reference and `height_at − surface_at` is a clean
    /// signed gap. A tunnel bore is the span where this gap is negative; its
    /// portal is the zero crossing where the road emerges.
    pub fn surface_at(&self, lon: f64, lat: f64) -> f64 {
        project_onto(&self.nodes, &self.terrain_m, self.cos_lat, lon, lat)
    }

    /// Deck cross-sections for a structure's (clipped, densified, in-order)
    /// centerline `pts`: each vertex placed on the *smoothed* global road line
    /// at the [`deck_m`](Self::deck_m) deck height, with a smoothed
    /// cross-section direction, so the swept box is a regular prism that
    /// follows the road instead of tracing every wiggle and dive.
    pub fn deck_nodes(&self, pts: &[Coord]) -> Vec<DeckNode> {
        self.walk(pts)
            .iter()
            .map(|&(i, t)| {
                let height_m = lerp(self.deck_m[i], self.deck_m[i + 1], t);
                let c = self.smooth_point(i, t);
                let (lon, lat) = (c.x, c.y);
                // Interpolate the bounding nodes' perpendiculars so the
                // cross-section direction turns continuously along a curve
                // instead of stepping once per edge (which twists the ribbon).
                let (le0, ln0) = self.node_left(i);
                let (le1, ln1) = self.node_left(i + 1);
                let (le, ln) = (lerp(le0, le1, t), lerp(ln0, ln1, t));
                let len = (le * le + ln * ln).sqrt();
                let (left_e, left_n) = if len > 1e-9 { (le / len, ln / len) } else { (le0, ln0) };
                let arc_m = lerp(self.arc[i], self.arc[i + 1], t);
                DeckNode { lon, lat, height_m, left_e, left_n, arc_m }
            })
            .collect()
    }

    /// `(lon, lat)` carried from the raw corridor line onto the smoothed sweep
    /// line **at its own lateral offset**, or `None` when the projection falls
    /// on the corridor's very ends (where pulling a vertex would fold a line
    /// that continues past the corridor) or when the carry would move it
    /// farther than `max_m` (a vertex that isn't really this corridor's).
    ///
    /// Road line work snaps through this so a corridor's paint follows the same
    /// smooth curve as its swept structures instead of tracing raw digitising
    /// wiggle beside them. **The offset is part of the answer, not noise to be
    /// projected away.** A road's centerline sits at offset zero and lands on
    /// the sweep line exactly as before; a painted marking sits at the offset
    /// the cross-section put it at (`synth::markings` — an edge line 4.2 m out
    /// on a 9 m carriageway) and must keep it. Projecting every vertex onto the
    /// curve collapsed both edge lines and every lane divider onto the axis,
    /// which is the road-relative parameterization of docs/ROADS.md H4 thrown
    /// away one stage after it was computed.
    ///
    /// The offset is measured against the raw edge the vertex projects to and
    /// re-applied along the *smoothed* line's own normal there, so paint stays
    /// square to the curve it now rides.
    pub fn smooth_at(&self, lon: f64, lat: f64, max_m: f64) -> Option<Coord> {
        let edges = self.nodes.len().saturating_sub(1);
        if edges == 0 {
            return None;
        }
        let p = Coord { x: lon, y: lat };
        let (i, t) = nearest_edge(&self.nodes, self.cos_lat, 0, edges, p);
        if (i == 0 && t <= 0.0) || (i + 1 >= edges && t >= 1.0) {
            return None;
        }
        let c = self.offset_from(self.smooth_point(i, t), i, t, self.lateral_offset_m(i, p));
        let de = (c.x - lon) * self.cos_lat * DEG_M;
        let dn = (c.y - lat) * DEG_M;
        (de * de + dn * dn <= max_m * max_m).then_some(c)
    }

    /// Signed distance in metres from raw edge `i` to `p`, positive to the left
    /// of the edge's direction of travel.
    fn lateral_offset_m(&self, i: usize, p: Coord) -> f64 {
        let (a, b) = (self.nodes[i], self.nodes[i + 1]);
        let (ex, en) = ((b.x - a.x) * self.cos_lat, b.y - a.y);
        let len = (ex * ex + en * en).sqrt();
        if len < 1e-12 {
            return 0.0;
        }
        let (px, pn) = ((p.x - a.x) * self.cos_lat, p.y - a.y);
        (ex * pn - en * px) / len * DEG_M
    }

    /// `c` moved `offset_m` metres to the left of the smoothed sweep line's
    /// direction at edge `i`, parameter `t`. A zero offset returns `c`
    /// unchanged, which is the road centerline's own case.
    fn offset_from(&self, c: Coord, i: usize, t: f64, offset_m: f64) -> Coord {
        if offset_m.abs() < 1e-9 {
            return c;
        }
        // The tangent by central difference on the same curve, in the local
        // metric frame. The Catmull-Rom is a polynomial, so evaluating it a
        // little outside `[0, 1]` at the ends is a valid extrapolation.
        const H: f64 = 1e-3;
        let (before, after) = (self.smooth_point(i, t - H), self.smooth_point(i, t + H));
        let (tx, ty) = ((after.x - before.x) * self.cos_lat, after.y - before.y);
        let len = (tx * tx + ty * ty).sqrt();
        if len < 1e-15 {
            return c;
        }
        let (lx, ly) = (-ty / len, tx / len);
        Coord {
            x: c.x + lx * offset_m / (DEG_M * self.cos_lat),
            y: c.y + ly * offset_m / DEG_M,
        }
    }

    /// The smoothed sweep line evaluated at edge `i`, parameter `t` — a
    /// Catmull-Rom *through* the smooth nodes rather than their chord: the
    /// noise is already regressed away, so interpolation is safe, and it
    /// removes the node-spacing facets a chord-sampled curve keeps.
    fn smooth_point(&self, i: usize, t: f64) -> Coord {
        let m = self.smooth.len();
        let p1 = self.smooth[i.min(m - 1)];
        let p2 = self.smooth[(i + 1).min(m - 1)];
        let p0 = if i == 0 { mirror(p1, p2) } else { self.smooth[i - 1] };
        let p3 = if i + 2 >= m { mirror(p2, p1) } else { self.smooth[i + 2] };
        catmull_rom(p0, p1, p2, p3, t, self.cos_lat)
    }

    /// Deck-top heights only — the height half of
    /// [`deck_nodes`](Self::deck_nodes), kept for direct testing.
    pub fn deck_line(&self, pts: &[Coord]) -> Vec<f64> {
        self.walk(pts).iter().map(|&(i, t)| lerp(self.deck_m[i], self.deck_m[i + 1], t)).collect()
    }

    /// Projects an in-order on-corridor polyline onto the profile, returning
    /// the `(edge, t)` of each vertex. The walk is monotonic from a robust
    /// interior seed, so a vertex is confined to the arc its neighbours sit on
    /// — a curving corridor that nears itself in plan can't snap a vertex onto
    /// a far arc. A clipped fragment may run either way; the direction is read
    /// from two interior points (the ends are where a self-approach lurks).
    fn walk(&self, pts: &[Coord]) -> Vec<(usize, f64)> {
        let edges = self.nodes.len().saturating_sub(1);
        if edges < 2 || pts.len() < 3 {
            return pts.iter().map(|p| self.project(0, edges.max(1), *p)).collect();
        }
        let ia = self.project(0, edges, pts[pts.len() / 3]).0;
        let ib = self.project(0, edges, pts[2 * pts.len() / 3]).0;
        let dir: isize = if ib >= ia { 1 } else { -1 };
        // The cursor may range ~6 edges per step: enough slack for one deck
        // step (about one profile edge) while still walling off a far arc.
        const WIN: isize = 6;
        let step = |cur: isize, towards: isize, p: Coord| -> (usize, f64) {
            let (lo, hi) = if towards >= 0 {
                (cur.max(0), (cur + WIN + 1).min(edges as isize))
            } else {
                ((cur - WIN).max(0), (cur + 1).min(edges as isize))
            };
            self.project(lo as usize, hi as usize, p)
        };
        let mid = pts.len() / 2;
        let mut out = vec![(0usize, 0.0); pts.len()];
        out[mid] = self.project(0, edges, pts[mid]);
        let mut cur = out[mid].0 as isize;
        for k in mid + 1..pts.len() {
            out[k] = step(cur, dir, pts[k]);
            cur = out[k].0 as isize;
        }
        cur = out[mid].0 as isize;
        for k in (0..mid).rev() {
            out[k] = step(cur, -dir, pts[k]);
            cur = out[k].0 as isize;
        }
        out
    }

    /// Unit left-perpendicular (ENU metres) of the smoothed line at *node*
    /// `i`, from the central-difference tangent. The spline-smoothed line is
    /// already gentle, so the tight window tracks a curve faithfully;
    /// [`deck_nodes`](Self::deck_nodes) interpolates between node
    /// perpendiculars for a continuously turning cross-section.
    fn node_left(&self, i: usize) -> (f64, f64) {
        let m = self.smooth.len();
        let lo = i.saturating_sub(1);
        let hi = (i + 1).min(m - 1);
        let de = (self.smooth[hi].x - self.smooth[lo].x) * self.cos_lat;
        let dn = self.smooth[hi].y - self.smooth[lo].y;
        let len = (de * de + dn * dn).sqrt().max(1e-12);
        (-dn / len, de / len)
    }

    /// Nearest edge to `p` over `[lo, hi)` and the clamped parameter along it.
    fn project(&self, lo: usize, hi: usize, p: Coord) -> (usize, f64) {
        nearest_edge(&self.nodes, self.cos_lat, lo, hi, p)
    }

    /// The densified nodes, for stage-artifact dumps and the ground stage.
    pub fn nodes(&self) -> &[Coord] {
        &self.nodes
    }

    /// The smoothed sweep line, for stage-artifact dumps.
    pub fn smooth(&self) -> &[Coord] {
        &self.smooth
    }

    /// Per-node solved road heights.
    pub fn road_m(&self) -> &[f64] {
        &self.road_m
    }

    /// Per-node deck-ramp heights, for stage-artifact dumps.
    pub fn deck_m(&self) -> &[f64] {
        &self.deck_m
    }

    /// Per-node reference terrain heights.
    pub fn terrain_m(&self) -> &[f64] {
        &self.terrain_m
    }

    /// Per-node at-grade flags (false inside a structure span).
    /// The grade ceiling this profile was solved to — the class prior raised
    /// to the alignment's measured grade where the ground earned it
    /// ([`measured_grade`]). `None` for a draped class.
    pub fn max_grade(&self) -> Option<f64> {
        self.max_grade
    }

    pub fn at_grade(&self) -> &[bool] {
        &self.at_grade
    }

    /// Per-node cumulative arc, metres.
    pub fn arc(&self) -> &[f64] {
        &self.arc
    }

    /// The centerline point at arc position `a`, interpolated in its edge.
    pub fn point_at_arc(&self, a: f64) -> Coord {
        let (i, t) = self.edge_at_arc(a);
        Coord {
            x: self.nodes[i].x + (self.nodes[i + 1].x - self.nodes[i].x) * t,
            y: self.nodes[i].y + (self.nodes[i + 1].y - self.nodes[i].y) * t,
        }
    }

    /// The same station on the **smoothed** sweep line — the curve a structure
    /// is swept along, a bore tubed along, and the ground benched beside.
    ///
    /// This is the accessor that lets the at-grade band read the same
    /// centerline as everything else (docs/ROADS.md H2 and invariant 5). It
    /// takes an arc rather than a point on purpose: a plan lookup projects,
    /// and projection is what puts a hairpin's vertex on the other arm and
    /// throws a lateral offset away.
    pub fn smooth_at_arc(&self, a: f64) -> Coord {
        let (i, t) = self.edge_at_arc(a);
        self.smooth_point(i, t)
    }

    /// The solved road height at arc position `a`.
    pub fn road_at_arc(&self, a: f64) -> f64 {
        let (i, t) = self.edge_at_arc(a);
        self.road_m[i] + (self.road_m[i + 1] - self.road_m[i]) * t
    }

    /// The raw terrain height at arc position `a`.
    pub fn surface_at_arc(&self, a: f64) -> f64 {
        let (i, t) = self.edge_at_arc(a);
        self.terrain_m[i] + (self.terrain_m[i + 1] - self.terrain_m[i]) * t
    }

    /// The deck-ramp height at arc position `a`.
    pub fn deck_at_arc(&self, a: f64) -> f64 {
        let (i, t) = self.edge_at_arc(a);
        self.deck_m[i] + (self.deck_m[i + 1] - self.deck_m[i]) * t
    }

    /// The edge index and parameter containing arc position `a`.
    fn edge_at_arc(&self, a: f64) -> (usize, f64) {
        let n = self.nodes.len();
        let a = a.clamp(0.0, *self.arc.last().unwrap_or(&0.0));
        let i = match self.arc.binary_search_by(|v| v.partial_cmp(&a).expect("finite arc")) {
            Ok(i) => i.min(n - 2),
            Err(i) => i.saturating_sub(1).min(n - 2),
        };
        let span = self.arc[i + 1] - self.arc[i];
        let t = if span > 0.0 { (a - self.arc[i]) / span } else { 0.0 };
        (i, t)
    }

    /// The corridor arc position (metres) nearest to `(lon, lat)`.
    pub fn arc_of(&self, lon: f64, lat: f64) -> f64 {
        let (i, t) = nearest_edge(
            &self.nodes,
            self.cos_lat,
            0,
            self.nodes.len().saturating_sub(1),
            Coord { x: lon, y: lat },
        );
        self.arc[i] + (self.arc[i + 1] - self.arc[i]) * t
    }

    /// Deck-top height at `(lon, lat)` — what a swept structure box renders.
    pub fn deck_height_at(&self, lon: f64, lat: f64) -> f64 {
        project_onto(&self.nodes, &self.deck_m, self.cos_lat, lon, lat)
    }

    /// Refits the per-span deck ramps after the road surface changed.
    fn rebuild_deck(&mut self) {
        self.deck_m = deck_ramp(&self.arc, &self.road_m, &self.at_grade);
    }

    /// Makes the deck ride the road profile exactly, node for node — for a
    /// monotone class, whose line hugs its bed through bridge and bore alike
    /// (§9: one cable, one hill). The straight-ramp fit is per *non-at-grade
    /// run*, and a funicular's run can span a whole tunnel–bridge–absorbed
    /// sequence: one chord over 209 m of curved bed put the drawn 13 m bridge
    /// deck metres above the band it must meet at the abutment — a step in a
    /// line that cannot step. With the deck equal to the road, the seam
    /// between a band piece and a deck piece is the same number on both sides
    /// by construction.
    pub fn set_deck_to_road(&mut self) {
        self.deck_m = self.road_m.clone();
    }

    /// Marks `[arc0, arc1]` as running in a structure — an annexed bore
    /// (`portals::annex_spans`) — and refits the deck over the merged run.
    /// One-way: nodes are only ever *removed* from the at-grade set, so an
    /// absorbed stretch the solve already flagged stays flagged. With
    /// `deck_follows_road` the deck stays the road line node for node (the
    /// monotone contract, [`Self::set_deck_to_road`]); otherwise the per-run
    /// straight ramps are refit.
    pub fn annex_structure(&mut self, arc0: f64, arc1: f64, deck_follows_road: bool) {
        for k in 0..self.at_grade.len() {
            if self.arc[k] >= arc0 && self.arc[k] <= arc1 {
                self.at_grade[k] = false;
            }
        }
        if deck_follows_road {
            self.set_deck_to_road();
        } else {
            self.rebuild_deck();
        }
    }

    /// Marks `[arc0, arc1]` as running at grade — the inverse of
    /// [`Self::annex_structure`], for a tunnel stretch the reconciliation
    /// degraded (`portals::reconcile_spans`): the solved line never went
    /// below the ground here, so the benches, the bands and the paint must
    /// all treat it as an open roadbed. Heights are left as solved — on a
    /// degraded stretch the line rides at or above its terrain by
    /// construction (the bore ceiling caps it at the surface and the zero
    /// crossings bound the buried run) — so only the flags move, and the deck
    /// refits around the runs that remain.
    pub fn degrade_structure(&mut self, arc0: f64, arc1: f64, deck_follows_road: bool) {
        for k in 0..self.at_grade.len() {
            if self.arc[k] >= arc0 && self.arc[k] <= arc1 {
                self.at_grade[k] = true;
            }
        }
        if deck_follows_road {
            self.set_deck_to_road();
        } else {
            self.rebuild_deck();
        }
    }

    /// Overwrites the solved road heights with the global relaxation's output
    /// ([`crate::solve::relax`]) and refits the deck ramps. `road` must have one
    /// entry per node; a length mismatch leaves the profile unchanged (the
    /// corridor keeps its warm-start solve).
    pub fn set_road_m(&mut self, road: &[f64]) {
        if road.len() != self.road_m.len() {
            return;
        }
        self.road_m.copy_from_slice(road);
        self.rebuild_deck();
    }

    /// A flat profile holding `height_m` over the given centerline — a DEM-free
    /// constructor for tests and degenerate inputs.
    pub fn flat(nodes: &[Coord], height_m: f64) -> Profile {
        let n = nodes.len();
        Profile {
            cos_lat: run_cos_lat(nodes),
            arc: cumulative(nodes),
            smooth: smooth_path(nodes),
            nodes: nodes.to_vec(),
            road_m: vec![height_m; n],
            // A flat profile is a single height, so the deck ramp is it too.
            deck_m: vec![height_m; n],
            // No terrain relief: the ground sits on the road, gap zero
            // everywhere (no buried span, no bore).
            terrain_m: vec![height_m; n],
            at_grade: vec![true; n],
            // Not solved to any ceiling: a single height holds no grade.
            max_grade: None,
        }
    }

    /// A profile over `nodes` with explicit per-node road and terrain heights.
    ///
    /// The constructor for a surface that was *fitted* rather than solved: a
    /// draped feature's deck chorded between the ground at its ends
    /// ([`crate::synth::draped`]), and the test fixtures that set up a bore's
    /// buried span and portal crossings deterministically without a DEM.
    pub fn from_heights(nodes: &[Coord], road_m: Vec<f64>, terrain_m: Vec<f64>) -> Profile {
        // The caller supplies the road heights it wants a deck to ride, so the
        // deck ramp is the road profile as given (no span-splitting here).
        // Every node is flagged at grade: burial is expressed through the
        // heights (the road/terrain gap), and an all-absorbed flag set would
        // make `portals::grow_spans` swallow whole corridors.
        let deck_m = road_m.clone();
        let n = nodes.len();
        Profile {
            cos_lat: run_cos_lat(nodes),
            arc: cumulative(nodes),
            smooth: smooth_path(nodes),
            nodes: nodes.to_vec(),
            road_m,
            deck_m,
            terrain_m,
            at_grade: vec![true; n],
            // Fitted, not solved: the caller's heights are the answer, and no
            // ceiling was applied to reach them.
            max_grade: None,
        }
    }
}

/// Cumulative metric arc length at each node.
fn cumulative(nodes: &[Coord]) -> Vec<f64> {
    crate::scene::cumulative_arc(nodes)
}

/// Linear interpolation between `a` and `b` at `t`.
fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

/// Endpoint-preserving centerline smoothing: [`SMOOTH_PASSES`] passes of a
/// local quadratic regression along arc length (Savitzky–Golay style), each
/// node refit from the neighbourhood that reaches ±[`SMOOTH_WINDOW_M`] or
/// ±[`SMOOTH_MAX_TURN_RAD`] of the road's own turning, whichever comes first.
/// Uncorrelated vertex wiggle averages away with no straight-chord kinks (a
/// plain low-pass clamped to a deviation tube goes piecewise-straight and kinks
/// where it touches the tube), while the turn budget is what keeps a genuine
/// curve: the fit is a parabola in the plane, which follows a road arc only
/// while the window spans a shallow angle of it.
///
/// A quadratic in arc length does **not** reproduce a circular arc — the claim
/// this once carried, and the reason the window was allowed to run to a fixed
/// ±100 m. It reproduces a parabola, which agrees with a circle to fourth order
/// and then leaves it: over a half window `L` on radius `R` the fitted centre
/// misses by about `L⁴/(280·R³)`, six millimetres at 400 m radius and 2.9
/// metres at 50 m. That is why the fixed window read as heavy corner-cutting on
/// every urban corner and hairpin in the extract, and why the budget is now on
/// the angle. [`SMOOTH_MAX_DEV_M`] remains as a backstop for the shapes an
/// angle cannot bound.
fn smooth_path(nodes: &[Coord]) -> Vec<Coord> {
    let n = nodes.len();
    if n < 5 {
        return nodes.to_vec();
    }
    let cos_lat = run_cos_lat(nodes);
    let arc = cumulative(nodes);
    let heading = cumulative_heading(nodes, &arc, cos_lat);
    let max_dev_x = SMOOTH_MAX_DEV_M / (DEG_M * cos_lat.max(1e-9));
    let max_dev_y = SMOOTH_MAX_DEV_M / DEG_M;
    let mut cur = nodes.to_vec();
    for _ in 0..SMOOTH_PASSES {
        let prev = cur.clone();
        for i in 1..n - 1 {
            // The window: as much arc as the ceiling allows, but never more of
            // the road than the turn budget — a quadratic can only follow a
            // shallow arc, so the angle is the binding constraint on a curve.
            let turned = |j: usize| (heading[j] - heading[i]).abs() > SMOOTH_MAX_TURN_RAD;
            let (mut lo, mut hi) = (i, i);
            while lo > 0 && arc[i] - arc[lo - 1] <= SMOOTH_WINDOW_M && !turned(lo - 1) {
                lo -= 1;
            }
            while hi + 1 < n && arc[hi + 1] - arc[i] <= SMOOTH_WINDOW_M && !turned(hi + 1) {
                hi += 1;
            }
            if hi - lo < 4 {
                continue;
            }
            let (sx, sy) = match quad_fit(&prev, &arc, lo, hi, i) {
                Some(p) => p,
                None => continue,
            };
            // Clamp the displacement from the *input* node, in metric space.
            let (dx, dy) = ((sx - nodes[i].x) / max_dev_x, (sy - nodes[i].y) / max_dev_y);
            let d = (dx * dx + dy * dy).sqrt();
            let k = if d > 1.0 { 1.0 / d } else { 1.0 };
            cur[i] = Coord {
                x: nodes[i].x + (sx - nodes[i].x) * k,
                y: nodes[i].y + (sy - nodes[i].y) * k,
            };
        }
    }
    cur
}

/// Least-squares quadratic in arc length over `pts[lo..=hi]`, evaluated at
/// node `i` (each coordinate fit independently, centred on `arc[i]` for
/// conditioning). `None` when the window's arc spread is degenerate.
fn quad_fit(pts: &[Coord], arc: &[f64], lo: usize, hi: usize, i: usize) -> Option<(f64, f64)> {
    let (mut s1, mut s2, mut s3, mut s4) = (0.0, 0.0, 0.0, 0.0);
    let (mut vx0, mut vx1, mut vx2) = (0.0, 0.0, 0.0);
    let (mut vy0, mut vy1, mut vy2) = (0.0, 0.0, 0.0);
    let m = (hi - lo + 1) as f64;
    for j in lo..=hi {
        let s = arc[j] - arc[i];
        let (x, y) = (pts[j].x - pts[i].x, pts[j].y - pts[i].y);
        s1 += s;
        s2 += s * s;
        s3 += s * s * s;
        s4 += s * s * s * s;
        vx0 += x;
        vx1 += x * s;
        vx2 += x * s * s;
        vy0 += y;
        vy1 += y * s;
        vy2 += y * s * s;
    }
    // Solve the 3×3 normal equations [m s1 s2; s1 s2 s3; s2 s3 s4] · c = v
    // for each coordinate; the fitted value at s = 0 is c₀.
    let det = m * (s2 * s4 - s3 * s3) - s1 * (s1 * s4 - s3 * s2) + s2 * (s1 * s3 - s2 * s2);
    if det.abs() < 1e-12 {
        return None;
    }
    let c0 = |v0: f64, v1: f64, v2: f64| -> f64 {
        (v0 * (s2 * s4 - s3 * s3) - s1 * (v1 * s4 - s3 * v2) + s2 * (v1 * s3 - s2 * v2)) / det
    };
    Some((pts[i].x + c0(vx0, vx1, vx2), pts[i].y + c0(vy0, vy1, vy2)))
}

/// Fits one straight ramp to arc-referenced heights and returns the fitted
/// value at each arc. The central span is least-squares-fit (the ends are
/// trimmed: a structure's busy landings — abutment touchdown, portal stub —
/// must not tilt the line). For a single chord the fit recovers it exactly, so
/// tile fragments of one span share the line.
fn fit_ramp(s: &[f64], h: &[f64]) -> Vec<f64> {
    let n = s.len();
    if n < 4 {
        return h.to_vec();
    }
    let cut = (n / 6).max(1);
    let (lo, hi) = (cut, n - cut);
    let m = (hi - lo) as f64;
    let (mut sx, mut sy, mut sxx, mut sxy) = (0.0, 0.0, 0.0, 0.0);
    for k in lo..hi {
        sx += s[k];
        sy += h[k];
        sxx += s[k] * s[k];
        sxy += s[k] * h[k];
    }
    let denom = m * sxx - sx * sx;
    if denom.abs() < f64::EPSILON {
        // Degenerate arc spread (a near-point piece): hold the mean height.
        return vec![sy / m; n];
    }
    let b = (m * sxy - sx * sy) / denom;
    let a = (sy - b * sx) / m;
    s.iter().map(|&si| a + b * si).collect()
}

/// Value at `(lon, lat)` from a per-node series, found by projecting the point
/// onto the nearest corridor edge of `nodes` and interpolating.
fn project_onto(nodes: &[Coord], vals: &[f64], cos_lat: f64, lon: f64, lat: f64) -> f64 {
    let (i, t) =
        nearest_edge(nodes, cos_lat, 0, nodes.len().saturating_sub(1), Coord { x: lon, y: lat });
    vals[i] + (vals[i + 1] - vals[i]) * t
}

/// Nearest edge to `p` over the edge index range `[lo, hi)` (edge `i` spans
/// `nodes[i]..nodes[i+1]`), returning the edge index and the clamped parameter
/// `t` of the foot of the perpendicular. Longitudes are scaled by `cos_lat`
/// into the local metric space. A bounded range lets the arc-order walk
/// confine the search to one arc; `lo = 0, hi = edges` makes it a full scan.
fn nearest_edge(nodes: &[Coord], cos_lat: f64, lo: usize, hi: usize, p: Coord) -> (usize, f64) {
    let px = p.x * cos_lat;
    let py = p.y;
    let mut best_d2 = f64::INFINITY;
    let mut best_i = lo.min(nodes.len().saturating_sub(2));
    let mut best_t = 0.0;
    for i in lo..hi {
        let (a, b) = (nodes[i], nodes[i + 1]);
        let ax = a.x * cos_lat;
        let dx = b.x * cos_lat - ax;
        let dy = b.y - a.y;
        let len2 = dx * dx + dy * dy;
        let t = if len2 > 0.0 {
            (((px - ax) * dx + (py - a.y) * dy) / len2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let cx = ax + dx * t;
        let cy = a.y + dy * t;
        let d2 = (px - cx) * (px - cx) + (py - cy) * (py - cy);
        if d2 < best_d2 {
            best_d2 = d2;
            best_i = i;
            best_t = t;
        }
    }
    (best_i, best_t)
}

/// The terrain profile with its narrow notches filled — the anchor surface a
/// mapped at-grade road actually rides. A surface DEM images the gully, the
/// stream cut, or the tree-shadow artifact *under* a road as a sharp V; the
/// road existed first, so ground continuity across it was engineered (fill
/// and a culvert, a retaining wall) and the road must not dive in and out.
/// Morphological closing along the arc: a running max then a running min
/// over ±[`NOTCH_SPAN_M`]/2, which fills every valley narrower than the span
/// to its rims and passes bumps, ramps, and wide valleys through untouched
/// (`closed ≥ h` everywhere, equality outside the notches). A notch whose
/// fill would exceed [`NOTCH_FILL_MAX_M`] is a genuine descent — or a gorge
/// owed a mapped bridge — so that run keeps the raw terrain; the reversion
/// is per contiguous run and the closing already meets the terrain at the
/// run's edges, so no step appears.
pub fn close_notches(arc: &[f64], h: &[f64]) -> Vec<f64> {
    close_bounded(arc, h, NOTCH_SPAN_M, NOTCH_FILL_MAX_M)
}

/// The terrain profile with its narrow convex bumps shaved — the opening
/// dual of [`close_notches`]. A surface DEM images canopy shadows, parked
/// vehicles, and upsampling ripple as sharp crests *on* the carriageway; the
/// road was graded through them, so the anchor surface must not climb them.
/// Opening is closing under negation (`open(h) = −close(−h)`), so it reuses
/// the same bounded machinery: every bump narrower than [`BUMP_SPAN_M`] is
/// shaved to its shoulders, everything else passes through untouched
/// (`opened ≤ h`, equality outside the bumps), and a run whose shave would
/// exceed [`BUMP_SHAVE_MAX_M`] is a genuine crest (S9) and keeps the raw
/// terrain.
pub fn open_bumps(arc: &[f64], h: &[f64]) -> Vec<f64> {
    let neg: Vec<f64> = h.iter().map(|&v| -v).collect();
    let mut opened = close_bounded(arc, &neg, BUMP_SPAN_M, BUMP_SHAVE_MAX_M);
    for v in &mut opened {
        *v = -*v;
    }
    opened
}

/// The conditioned anchor surface every road profile rides: the terrain with
/// its narrow notches filled ([`close_notches`]) and then its narrow bumps
/// shaved ([`open_bumps`]). Symmetric by construction — DEM noise enters the
/// profile in neither direction, genuine relief passes through in both.
/// Closing runs first so a notch-and-bump pair (one signal ringing both
/// ways) resolves toward the engineered fill rather than the artifact.
pub fn condition_reference(arc: &[f64], h: &[f64]) -> Vec<f64> {
    open_bumps(arc, &close_notches(arc, h))
}

/// Bounded morphological closing along the arc: running max then running min
/// over ±`span`/2, with per-run reversion wherever the fill exceeds `cap`
/// (the trust boundary — see the callers for what a too-deep run means).
fn close_bounded(arc: &[f64], h: &[f64], span: f64, cap: f64) -> Vec<f64> {
    close_bounded_runs(arc, h, span, cap).0
}

/// The arc intervals [`close_notches`] *refused* — narrow valleys whose fill
/// would exceed [`NOTCH_FILL_MAX_M`], reverted to raw terrain by the bounded
/// closing. Each interval covers exactly the nodes the closing wanted to lift
/// (the notch interior); the bracketing rim nodes, where the closing already
/// meets the terrain, are outside it. This is the detector behind the S20
/// notch-crossing span promotion (`solve::promote_notch_crossings`): a slot
/// this deep under a line mapped level across it is a crossing owed a
/// structure, not a bed. Computed from the closing pass on the raw heights —
/// never from [`open_bumps`], whose reverted runs are refused *crests* on
/// negated heights, nor from [`condition_reference`], whose opening pass
/// reshapes what the closing saw.
pub fn refused_notches(arc: &[f64], h: &[f64]) -> Vec<(f64, f64)> {
    close_bounded_runs(arc, h, NOTCH_SPAN_M, NOTCH_FILL_MAX_M).1
}

/// [`close_bounded`] plus the reverted runs: the closed heights, and one
/// `(arc_first, arc_last)` per contiguous run whose fill exceeded `cap`.
fn close_bounded_runs(
    arc: &[f64],
    h: &[f64],
    span: f64,
    cap: f64,
) -> (Vec<f64>, Vec<(f64, f64)>) {
    let n = h.len();
    if n == 0 {
        return (Vec::new(), Vec::new());
    }
    let r = span * 0.5;
    // Pad each end with one edge-replicated virtual node at ±r: the erosion
    // window at a profile end is otherwise half-open and cannot recover the
    // dilation there, so an ascending start would read as half a "notch" and
    // lift by slope·r. With the pad, closing is identity on monotone ground
    // right up to the ends; a genuine dip touching an end stays unfilled
    // (conservative — its rim is off the profile, so no fill is provable).
    let mut pa = Vec::with_capacity(n + 2);
    let mut ph = Vec::with_capacity(n + 2);
    pa.push(arc[0] - r);
    ph.push(h[0]);
    pa.extend_from_slice(arc);
    ph.extend_from_slice(h);
    pa.push(arc[n - 1] + r);
    ph.push(h[n - 1]);
    let dilated = window_fold(&pa, &ph, r, f64::max);
    let closed_padded = window_fold(&pa, &dilated, r, f64::min);
    let mut closed: Vec<f64> = closed_padded[1..=n].to_vec();
    let mut refused: Vec<(f64, f64)> = Vec::new();
    let mut i = 0;
    while i < n {
        if closed[i] - h[i] <= 1e-6 {
            closed[i] = h[i];
            i += 1;
            continue;
        }
        let start = i;
        let mut deepest = 0.0f64;
        while i < n && closed[i] - h[i] > 1e-6 {
            deepest = deepest.max(closed[i] - h[i]);
            i += 1;
        }
        if deepest > cap {
            for k in start..i {
                closed[k] = h[k];
            }
            refused.push((arc[start], arc[i - 1]));
        }
    }
    (closed, refused)
}

/// `fold` of `h` over the arc window ±`r` around each node. The window is
/// tiny (a handful of nodes), so the inner scan is cheap.
fn window_fold(arc: &[f64], h: &[f64], r: f64, fold: fn(f64, f64) -> f64) -> Vec<f64> {
    let n = h.len();
    let mut out = Vec::with_capacity(n);
    let (mut lo, mut hi) = (0usize, 0usize);
    for i in 0..n {
        while arc[lo] < arc[i] - r {
            lo += 1;
        }
        while hi < n && arc[hi] <= arc[i] + r {
            hi += 1;
        }
        let mut v = h[lo];
        for &x in &h[lo + 1..hi] {
            v = fold(v, x);
        }
        out.push(v);
    }
    out
}

/// The road elevation at each node: terrain at the anchors (at-grade nodes and
/// structure boundaries), and a straight interpolation between the bounding
/// anchors across each structure. The corridor endpoints are always anchors,
/// so every node is bracketed.
fn road_profile(arc: &[f64], terrain: &[f64], anchor: &[bool]) -> Vec<f64> {
    let n = terrain.len();

    // Nearest anchor (arc, elevation) at-or-before and at-or-after each node,
    // in single forward/backward passes.
    let mut prev = vec![None; n];
    let mut last: Option<(f64, f64)> = None;
    for i in 0..n {
        if anchor[i] {
            last = Some((arc[i], terrain[i]));
        }
        prev[i] = last;
    }
    let mut next = vec![None; n];
    let mut coming: Option<(f64, f64)> = None;
    for i in (0..n).rev() {
        if anchor[i] {
            coming = Some((arc[i], terrain[i]));
        }
        next[i] = coming;
    }

    (0..n)
        .map(|i| {
            if anchor[i] {
                return terrain[i];
            }
            // A structure run with no anchor on one side (a corridor that
            // starts or ends mid-structure) may chord down to the terrain at
            // that corridor end — but only where that end's ground lies
            // *below* the anchor: holding the anchor's height flat there
            // would leave the structure unsupported in the air (a descending
            // mountain tunnel once floated 177 m over its lower portal).
            // Where the end's ground rises above the anchor the flat grade is
            // kept: the structure passes *under* the hill and the terrain
            // occludes it (S5/S7), it does not climb the flank.
            let chord = |sa: f64, ta: f64, sb: f64, tb: f64| {
                if sb > sa {
                    ta + (tb - ta) * (arc[i] - sa) / (sb - sa)
                } else {
                    ta
                }
            };
            match (prev[i], next[i]) {
                (Some((sa, ta)), Some((sb, tb))) => chord(sa, ta, sb, tb),
                (Some((sa, ta)), None) => {
                    chord(sa, ta, arc[n - 1], terrain[n - 1].min(ta))
                }
                (None, Some((sb, tb))) => chord(arc[0], terrain[0].min(tb), sb, tb),
                (None, None) => {
                    // No at-grade anchor anywhere: chord the corridor endpoints.
                    chord(arc[0], terrain[0], arc[n - 1], terrain[n - 1])
                }
            }
        })
        .collect()
}

/// Holds the at-grade road to its grade cap (`max_grade`) while keeping it
/// within `deviation_m` of the terrain reference. It flattens the steep
/// flanks the draped ground throws up — the dive into a bridge abutment, a
/// rolling bump — onto gentle cuttings and embankments, but never drifts far
/// from the ground: where the terrain climbs faster than the grade allows over
/// a long stretch, the road follows the slope (steeper, but hugging the ground
/// and visible) instead of flying off it into a phantom viaduct or burying
/// itself under a hill (S9).
///
/// Structure nodes are *pinned* (a bridge deck / tunnel bore already rides the
/// gentle reconstructed ramp) and so anchor the limiter — the steep approach
/// beside a structure is pulled to the structure's grade, the embankment that
/// reaches the abutment. The deviation clamp is applied last, so the bound on
/// cut/fill depth always holds (the grade is then best-effort — the
/// ground-hugging budget wins a conflict).
fn limit_road_grade(
    arc: &[f64],
    road_m: &mut [f64],
    terrain: &[f64],
    at_grade: &[bool],
    max_grade: f64,
    deviation_m: f64,
) {
    let n = road_m.len();
    if n < 2 {
        return;
    }
    let to_terrain = |road_m: &mut [f64], i: usize| {
        road_m[i] = road_m[i].clamp(terrain[i] - deviation_m, terrain[i] + deviation_m);
    };
    let to_grade = |road_m: &mut [f64], i: usize, nb: usize| {
        let cap = max_grade * (arc[i] - arc[nb]).abs();
        road_m[i] = road_m[i].clamp(road_m[nb] - cap, road_m[nb] + cap);
    };
    for pass in 0..=GRADE_PASSES {
        // The last pass is always forward; each node is grade-clamped then
        // pulled back inside the deviation budget, so that bound holds on exit.
        if pass % 2 == 0 || pass == GRADE_PASSES {
            for i in 1..n {
                if at_grade[i] {
                    to_grade(road_m, i, i - 1);
                    to_terrain(road_m, i);
                }
            }
        } else {
            for i in (0..n - 1).rev() {
                if at_grade[i] {
                    to_grade(road_m, i, i + 1);
                    to_terrain(road_m, i);
                }
            }
        }
    }
}

/// The grade the ground itself holds along the stretches the data maps as at
/// grade — the alignment's grade as *built*, against the class prior's grade
/// as *designed*.
///
/// A class ceiling is a convention for a profile the solve has to invent. Where
/// a corridor is mapped at grade, the conditioned terrain under it is not a
/// guess: it is the formation, cuttings and embankments included, and the
/// railway is the reason that shape is there (§4.2). So the ceiling is the
/// prior *or* this, whichever is steeper.
///
/// The case that forces it: Overture has no class for a rack railway. The
/// Montreux–Glion line climbs 285 m in 2.6 km — 11 % sustained, and it is
/// tagged `narrow_gauge`, whose adhesion prior is 7 %. Held to 7 % the profile
/// dives tens of metres under its own track bed, which
/// [`absorb_infeasible_anchors`] then reads as an annotation that ended early
/// and flies the whole alignment into the air.
///
/// Taken as a [percentile](MEASURED_GRADE_PCTL) over at-grade edges, so a
/// *sustained* steep line raises its ceiling while a *local* plunge at a
/// structure end — the annotation shortfall absorption exists for — does not.
/// Bounded by [`MEASURED_GRADE_MAX`] or the class's own convention plus
/// [`MEASURED_GRADE_HEADROOM`], whichever is larger, so a gentle class cannot
/// claim a cliff and a steep one is not clipped below the bed it was written
/// for. `None` when too little of the corridor is mapped at grade
/// ([`MEASURED_GRADE_MIN_M`]) for the read to mean anything.
///
/// **Per-edge, and a windowed rule was measured and rejected.** The obvious
/// worry about the per-edge percentile is that a V-shaped plunge-and-recover
/// inside one notch span has steep edges while going nowhere, so it could buy
/// a licence to dive that a road crossing a gully should not have. Replacing
/// the edge grades with the reference's *net* rise over
/// [`NOTCH_SPAN_M`]-wide windows — which a V cancels out of — was censused
/// before it was built (`examples/grade_census`), and it is not a tightening
/// but a deletion: of the 1,998 corridors the escape raises over the Montreux
/// zone, 1,857 would lose it, **including 22 of the 26 narrow-gauge escapes**,
/// which is the rack railway this exists for (S18). The escape is not a rare
/// exception — 8.5 % of the network holds one and 94 % of those spend it —
/// because a mapped centerline traversing a steep flank disagrees with the DEM
/// at the 12–24 m node scale far more than it does over 60 m. Any future
/// attempt has to separate cross-slope sampling disagreement from a genuine
/// V, which the window does not do; and see the `Mode::Street` arm below for
/// why holding a street to its class number instead is what hung the Chauderon
/// chain 26 m over its own hillside.
fn measured_grade(
    arc: &[f64],
    reference: &[f64],
    at_grade: &[bool],
    class_grade: f64,
) -> Option<f64> {
    let mut grades: Vec<f64> = Vec::new();
    let mut spanned = 0.0;
    for i in 1..arc.len() {
        // Both ends at grade: an edge into or out of a structure is a chord,
        // and its pitch is the solve's, not the ground's.
        if !at_grade[i] || !at_grade[i - 1] {
            continue;
        }
        let run = arc[i] - arc[i - 1];
        if run <= 0.0 {
            continue;
        }
        spanned += run;
        grades.push((reference[i] - reference[i - 1]).abs() / run);
    }
    if spanned < MEASURED_GRADE_MIN_M || grades.is_empty() {
        return None;
    }
    grades.sort_by(|a, b| a.partial_cmp(b).expect("finite grade"));
    let k = ((grades.len() - 1) as f64 * MEASURED_GRADE_PCTL).round() as usize;
    let cap = MEASURED_GRADE_MAX.max(class_grade * MEASURED_GRADE_HEADROOM);
    Some(grades[k].min(cap))
}

/// Re-pins every two-sided structure chord to the *limited* road heights at
/// its bounding anchors. [`limit_road_grade`] may carry an approach onto an
/// embankment (or into a cut) right up to a structure edge — where terrain
/// falls faster than the grade ceiling, the road arrives metres away from
/// the raw terrain sample the chord was fit from, and the boundary step
/// would crack the road surface at the abutment. Refitting the chord through
/// where the road actually arrives restores continuity; the deck then
/// launches from the embankment top, which is where a real abutment sits. A
/// one-sided run (a corridor starting or ending mid-structure) keeps its
/// solved heights — there is no arrival to meet.
fn rechord_structures(arc: &[f64], road_m: &mut [f64], at_grade: &[bool]) {
    let n = road_m.len();
    let mut i = 0;
    while i < n {
        if at_grade[i] {
            i += 1;
            continue;
        }
        let start = i;
        while i < n && !at_grade[i] {
            i += 1;
        }
        let end = i - 1;
        if start == 0 || end + 1 >= n {
            continue; // one-sided: no bounding anchor to meet on that side
        }
        let (sa, ha) = (arc[start - 1], road_m[start - 1]);
        let (sb, hb) = (arc[end + 1], road_m[end + 1]);
        if sb <= sa {
            continue;
        }
        for k in start..=end {
            road_m[k] = ha + (hb - ha) * (arc[k] - sa) / (sb - sa);
        }
    }
}

/// Snaps each structure-bounding anchor to the local terrain extremum within
/// [`ANCHOR_SEEK_M`] of it, by flipping the intervening at-grade nodes into
/// the structure run. The annotation edge lands where the mapper split the
/// segment, and the DEM there is a roll-off blend of rim and wall (or flank
/// and floor) — the least trustworthy sample in sight. The physical anchor
/// is the rim crest a deck launches from (the span's terrain falls away
/// below: seek the *maximum*) or the flank base a bore emerges at (it rises
/// above: seek the *minimum*). Runs once, before any chord is fit; a side
/// whose span terrain stays within [`SEEK_GAP_M`] of the anchor is left
/// alone (flat ground — nothing to launch over), as is an anchor already
/// within [`SEEK_MIN_GAIN_M`] of the extremum. Returns whether any anchor
/// moved.
fn seek_rim_anchors(arc: &[f64], terrain: &[f64], at_grade: &mut [bool]) -> bool {
    let n = at_grade.len();
    if n < 3 {
        return false;
    }
    // Structure runs as inclusive node ranges, gathered up front (flipping
    // must not create new edges within this single pass).
    let mut runs: Vec<(usize, usize)> = Vec::new();
    let mut i = 0;
    while i < n {
        if at_grade[i] {
            i += 1;
            continue;
        }
        let start = i;
        while i < n && !at_grade[i] {
            i += 1;
        }
        runs.push((start, i - 1));
    }
    let mut changed = false;
    // One side of one run: `anchor` is the bounding at-grade node, `inside`
    // a node a couple of steps into the run (where the flying/buried gap has
    // developed), `dir` +1 walking outward toward higher indices.
    let mut seek = |anchor: usize, inside: usize, dir: isize, at_grade: &mut [bool]| {
        let sign = terrain[inside] - terrain[anchor];
        if sign.abs() < SEEK_GAP_M {
            return; // effectively flat: the annotation edge is fine
        }
        // Terrain falls into the span → a deck: the anchor belongs on the
        // highest ground in reach (the rim). Rises → a bore: on the lowest.
        let better = |a: f64, b: f64| if sign < 0.0 { a > b } else { a < b };
        let mut best = anchor;
        let mut k = anchor as isize + dir;
        while k >= 0
            && (k as usize) < n
            && at_grade[k as usize]
            && (arc[k as usize] - arc[anchor]).abs() <= ANCHOR_SEEK_M
        {
            if better(terrain[k as usize], terrain[best]) {
                best = k as usize;
            }
            k += dir;
        }
        if best != anchor && (terrain[best] - terrain[anchor]).abs() >= SEEK_MIN_GAIN_M {
            // Flip everything between the old anchor (inclusive) and the new
            // one (exclusive): the roll-off joins the structure.
            let (lo, hi) = if best < anchor { (best + 1, anchor) } else { (anchor, best - 1) };
            at_grade[lo..=hi].fill(false);
            changed = true;
        }
    };
    for &(start, end) in &runs {
        if start > 0 && at_grade[start - 1] {
            seek(start - 1, (start + 2).min(end), -1, at_grade);
        }
        if end + 1 < n && at_grade[end + 1] {
            seek(end + 1, end.saturating_sub(2).max(start), 1, at_grade);
        }
    }
    changed
}

/// Flips at-grade nodes into the neighbouring structure run where the road
/// cannot in fact be at grade there. Two symptoms mark the stretch, and both
/// say the same thing: the annotation ended before the physical structure did
/// (S10).
///
/// - *Infeasible pitch*: the solved road still pitches beyond
///   [`ABSORB_GRADE_FACTOR`] × the class grade ceiling — the leftover of
///   [`limit_road_grade`]'s deviation clamp beating its grade clamp, which
///   only happens where the terrain near a structure end is too steep for any
///   at-grade road (a bridge landing into a gorge wall, a tunnel emerging
///   under a climbing flank).
/// - *Standing off the ground*: the road runs more than
///   [`ABSORB_STANDOFF_M`] clear of the natural terrain, on the same side the
///   structure is on, in an unbroken run out from the structure edge. A
///   motorway 12 m above the valley floor beside a mapped bridge span is that
///   bridge, not a 12 m embankment — and modelled as an embankment it built a
///   wall of ground across whatever passes beneath it.
///
/// The search reaches [`PORTAL_MAX_M`] past each run edge — the same
/// annotation-trust radius the portal solver uses — and everything from the
/// edge through the farthest violation flips, so the structure continues
/// through the whole infeasible stretch. Returns whether anything flipped.
fn absorb_infeasible_anchors(
    arc: &[f64],
    at_grade: &mut [bool],
    road_m: &[f64],
    terrain: &[f64],
    max_grade: f64,
) -> bool {
    let n = at_grade.len();
    if n < 2 {
        return false;
    }
    let tol = max_grade * ABSORB_GRADE_FACTOR;
    // Pitch of the edge into node `i`, against the tolerance.
    let steep = |i: usize| -> bool {
        let run = arc[i] - arc[i - 1];
        run > 0.0 && (road_m[i] - road_m[i - 1]).abs() / run > tol
    };
    // Structure runs as inclusive node ranges, gathered up front: the flips
    // below must not be re-scanned as new run edges within one pass.
    let mut runs: Vec<(usize, usize)> = Vec::new();
    let mut i = 0;
    while i < n {
        if at_grade[i] {
            i += 1;
            continue;
        }
        let start = i;
        while i < n && !at_grade[i] {
            i += 1;
        }
        runs.push((start, i - 1));
    }
    let mut changed = false;
    for &(start, end) in &runs {
        // Which way this structure leaves the ground: a deck flies, a bore is
        // buried. Only a standoff in the same direction continues it.
        let flying = (start..=end).map(|k| road_m[k] - terrain[k]).sum::<f64>() >= 0.0;
        let stands_off = |i: usize| -> bool {
            let clear = if flying { road_m[i] - terrain[i] } else { terrain[i] - road_m[i] };
            clear > ABSORB_STANDOFF_M
        };
        // Outward past the run's high edge: the farthest steep pitch within
        // reach, or the end of an unbroken standoff run, marks the infeasible
        // stretch. The standoff must be contiguous from the edge — one distant
        // flyer must not swallow the at-grade road between.
        let mut worst = None;
        let mut contiguous = true;
        let mut k = end + 1;
        while k < n && at_grade[k] && arc[k] - arc[end] <= PORTAL_MAX_M {
            if steep(k) {
                worst = Some(k);
            }
            contiguous = contiguous && stands_off(k);
            if contiguous {
                worst = Some(worst.map_or(k, |w: usize| w.max(k)));
            }
            k += 1;
        }
        if let Some(hi) = worst {
            at_grade[end + 1..=hi].fill(false);
            changed = true;
        }
        // And past the low edge, mirrored.
        let mut worst = None;
        let mut contiguous = true;
        let mut k = start;
        while k > 0 && at_grade[k - 1] && arc[start] - arc[k - 1] <= PORTAL_MAX_M {
            if steep(k) {
                worst = Some(k - 1);
            }
            contiguous = contiguous && stands_off(k - 1);
            if contiguous {
                worst = Some(worst.map_or(k - 1, |w: usize| w.min(k - 1)));
            }
            k -= 1;
        }
        if let Some(lo) = worst {
            at_grade[lo..start].fill(false);
            changed = true;
        }
    }
    changed
}

/// Net bed rise below which a monotone class's direction is not trusted: a
/// station loop or a stub is effectively level, and forcing a direction onto
/// noise would tilt a flat track.
pub const MONOTONE_MIN_RISE_M: f64 = 5.0;

/// The direction a monotone corridor climbs, read from its *bed* (the terrain
/// reference at its two ends) rather than its solved heights — the solved
/// heights are exactly what the constraint exists to correct. `None` when the
/// net rise is under [`MONOTONE_MIN_RISE_M`].
pub fn monotone_direction(terrain: &[f64]) -> Option<f64> {
    let (first, last) = (*terrain.first()?, *terrain.last()?);
    let rise = last - first;
    (rise.abs() >= MONOTONE_MIN_RISE_M).then(|| rise.signum())
}

/// Projects `h` onto the monotone (non-decreasing along `dir`) sequence
/// nearest to it — isotonic regression by pool-adjacent-violators, uniform
/// weights. The L2 projection, not a directional clamp: a clamp propagates a
/// single bad height across everything downstream of it, where the projection
/// averages the violators into the level stretch a real line would hold.
pub fn monotone_project(h: &mut [f64], dir: f64) {
    let n = h.len();
    if n < 2 {
        return;
    }
    // Work in ascending orientation; flip for a descending line.
    let flip = dir < 0.0;
    let val = |h: &[f64], i: usize| if flip { -h[i] } else { h[i] };
    // Pools of (mean, count), merged while the tail violates.
    let mut mean: Vec<f64> = Vec::with_capacity(n);
    let mut count: Vec<usize> = Vec::with_capacity(n);
    for i in 0..n {
        mean.push(val(h, i));
        count.push(1);
        while mean.len() > 1 && mean[mean.len() - 2] > mean[mean.len() - 1] {
            let (m2, c2) = (mean.pop().expect("tail"), count.pop().expect("tail"));
            let l = mean.len() - 1;
            let c1 = count[l] as f64;
            mean[l] = (mean[l] * c1 + m2 * c2 as f64) / (c1 + c2 as f64);
            count[l] += c2;
        }
    }
    let mut i = 0;
    for (m, c) in mean.iter().zip(&count) {
        for _ in 0..*c {
            h[i] = if flip { -m } else { *m };
            i += 1;
        }
    }
}

/// Projects `h` onto the monotone sequences of direction `dir` whose rise per
/// metre never exceeds `max_grade` — §9's line, with the class's own bend
/// limit attached.
///
/// The plain projection alone turns a bed bump into a lurch and a plateau: the
/// pooled average clears the bump inside one node spacing and then holds level
/// while the bed catches up — measured as a 133 % pitch on the Collonge
/// funicular's north approach, where a gully crosses the alignment and the DEM
/// carries the scar into the conditioned bed. A rail cannot pitch like that; it
/// holds its grade and cuts through the bump, and the ground stage digs
/// whatever the line now passes through.
///
/// The set {monotone along `dir`, |rise| ≤ `max_grade`·Δarc} is the
/// intersection of two convex cone constraints, each of which is one PAVA in a
/// transformed coordinate (the slope cap is isotonic regression on
/// `h − g·arc`, run the other way). A few alternating projections land near
/// the intersection; a final forward/backward clamp guarantees feasibility
/// exactly, redistributing any residual into the constant-grade ramp a real
/// line would hold.
pub fn monotone_project_graded(h: &mut [f64], arcs: &[f64], dir: f64, max_grade: f64) {
    let n = h.len();
    if n < 2 || arcs.len() != n || max_grade <= 0.0 {
        monotone_project(h, dir);
        return;
    }
    // Ascending orientation: w must be non-decreasing with slope ≤ g.
    let s = if dir < 0.0 { -1.0 } else { 1.0 };
    let mut w: Vec<f64> = h.iter().map(|&v| s * v).collect();
    for _ in 0..3 {
        monotone_project(&mut w, 1.0);
        for (v, &a) in w.iter_mut().zip(arcs) {
            *v -= max_grade * a;
        }
        monotone_project(&mut w, -1.0);
        for (v, &a) in w.iter_mut().zip(arcs) {
            *v += max_grade * a;
        }
    }
    for k in 1..n {
        let step = max_grade * (arcs[k] - arcs[k - 1]).max(0.0);
        w[k] = w[k].clamp(w[k - 1], w[k - 1] + step);
    }
    for k in (0..n - 1).rev() {
        let step = max_grade * (arcs[k + 1] - arcs[k]).max(0.0);
        w[k] = w[k].clamp(w[k + 1] - step, w[k + 1]);
    }
    for (out, &v) in h.iter_mut().zip(&w) {
        *out = s * v;
    }
}

/// Rounds the sharp vertical grade breaks of an *engineered* road profile into
/// gentle parabolic curves — the abutment corner where an embankment approach
/// meets a deck ramp, and the kinks [`limit_road_grade`] leaves on a cut or
/// fill. Each pass pulls every engineered node toward the arc-chord of its
/// neighbours (a discrete second-derivative penalty — the vertical curvature a
/// real road holds for comfort and sight distance); a straight ramp is harmonic
/// and passes through unchanged, so only the corners round out.
///
/// Only nodes already lifted onto a fill or sunk into a cut are reshaped
/// (`|road − terrain| ≥ MIN_EARTHWORK_M`): a genuinely draped at-grade node
/// stays pinned to the ground, so the road never floats off the rendered
/// terrain (invariant 4). Those pinned nodes, the structure nodes (their deck
/// ramp is refit straight afterward), and the corridor ends are the smoothing's
/// fixed boundaries. Each moved node is held within [`MAX_ROAD_DEVIATION_M`] of
/// the terrain so the rounding can never drift the road far from the ground.
fn smooth_vgrades(arc: &[f64], road_m: &mut [f64], terrain: &[f64], at_grade: &[bool]) {
    let n = road_m.len();
    if n < 3 {
        return;
    }
    // Reshapeable only where the road is engineered off the ground (a fill or
    // cut) and at grade — decks stay straight, draped nodes stay on the terrain.
    let movable = |road_m: &[f64], i: usize| {
        at_grade[i] && (road_m[i] - terrain[i]).abs() >= MIN_EARTHWORK_M
    };
    for _ in 0..VGRADE_PASSES {
        // Jacobi: every node reads the previous pass, so the relaxation is
        // order-independent and a corridor's fragments cannot diverge.
        let prev = road_m.to_vec();
        for i in 1..n - 1 {
            if !movable(&prev, i) {
                continue;
            }
            let (a0, a1, a2) = (arc[i - 1], arc[i], arc[i + 1]);
            let span = a2 - a0;
            if span <= 0.0 {
                continue;
            }
            // The point on the chord prev[i-1]→prev[i+1] at this node's arc, so
            // uneven node spacing doesn't bias the curve.
            let t = (a1 - a0) / span;
            let chord = prev[i - 1] + (prev[i + 1] - prev[i - 1]) * t;
            let moved = prev[i] + VGRADE_LAMBDA * (chord - prev[i]);
            road_m[i] =
                moved.clamp(terrain[i] - MAX_ROAD_DEVIATION_M, terrain[i] + MAX_ROAD_DEVIATION_M);
        }
    }
}

/// The street variant of [`smooth_vgrades`]: every at-grade node moves (a
/// street has no engineered fills to distinguish — the whole bed is
/// reshaped), each pass clamped to `deviation_m` of the conditioned
/// reference so the smoothing irons node-scale wobble without floating the
/// street off a slope it genuinely climbs (S9). Structure nodes and the ends
/// stay pinned, like the engineered variant.
fn smooth_vgrades_street(
    arc: &[f64],
    road_m: &mut [f64],
    terrain: &[f64],
    at_grade: &[bool],
    deviation_m: f64,
) {
    let n = road_m.len();
    if n < 3 {
        return;
    }
    for _ in 0..VGRADE_PASSES {
        // Jacobi, like the engineered variant: order-independent, so a
        // corridor's fragments cannot diverge (invariant 5).
        let prev = road_m.to_vec();
        for i in 1..n - 1 {
            if !at_grade[i] {
                continue;
            }
            let (a0, a1, a2) = (arc[i - 1], arc[i], arc[i + 1]);
            let span = a2 - a0;
            if span <= 0.0 {
                continue;
            }
            let t = (a1 - a0) / span;
            let chord = prev[i - 1] + (prev[i + 1] - prev[i - 1]) * t;
            let moved = prev[i] + VGRADE_LAMBDA * (chord - prev[i]);
            road_m[i] = moved.clamp(terrain[i] - deviation_m, terrain[i] + deviation_m);
        }
    }
}

/// How far into a span a pinned boundary's correction is blended back into
/// the straight fit, in metres — the deck's vertical transition at an
/// abutment. About six profile nodes: the scale [`smooth_vgrades`] rounds an
/// abutment's grade break over, which is what the fit's residual at a
/// boundary mostly is (measured over the Montreux extract: 95 % of run ends
/// are already exact, the rest ≤ 3 m). A span shorter than two tapers uses
/// half its length, so each end's correction dies before the other end and
/// both still land exactly.
const DECK_PIN_TAPER_M: f64 = 24.0;

/// The deck-top height at each node: [`road_profile`]'s heights with every
/// structure span (a maximal run of non-at-grade nodes) replaced by a single
/// straight ramp fit over that span and its bounding anchors. The at-grade
/// nodes keep their draped road height, so a deck meets the ground exactly at
/// an abutment.
///
/// **The ramp's ends are pinned to the road wherever a band meets them.** The
/// at-grade band ends at the span arc reading `road_at_arc`, and that arc
/// lies inside the boundary node's own segment — so the two surfaces agree at
/// the handover exactly when the boundary *structure* node carries the road
/// height (`seam.band_deck_step`, invariant 2). The fit alone misses it by
/// its end residual, which is whatever the vertical-curve rounding and the
/// clearance solve did to the road inside the trimmed sixth. The correction
/// is blended back into the fit over [`DECK_PIN_TAPER_M`] (a smoothstep, so
/// the deck leaves the joint at the ramp's own grade and rejoins the fit
/// tangentially): mid-span stays the straight fit — a whole-span tilt to the
/// boundary heights is the rejected first cut of the datum work, which
/// daylit bore roofs mid-span. A run at the corridor's own end has no band
/// on the far side and keeps the bare fit there: pinning it would chase the
/// annotation touchdown the trim exists to ignore.
fn deck_ramp(arc: &[f64], road_m: &[f64], at_grade: &[bool]) -> Vec<f64> {
    let n = road_m.len();
    let mut deck = road_m.to_vec();
    let mut i = 0;
    while i < n {
        if at_grade[i] {
            i += 1;
            continue;
        }
        // Maximal structure run [start, end); include the bounding anchors so
        // the ramp is fit to (and lands on) the road's true elevation at each
        // end.
        let start = i;
        while i < n && !at_grade[i] {
            i += 1;
        }
        let end = i;
        let lo = start.saturating_sub(1);
        let hi = end.min(n - 1);
        let fitted = fit_ramp(&arc[lo..=hi], &road_m[lo..=hi]);
        for (k, &v) in (lo..=hi).zip(fitted.iter()) {
            // Overwrite only the structure nodes; at-grade anchors stay draped.
            if !at_grade[k] {
                deck[k] = v;
            }
        }
        let last = end - 1;
        let taper = (0.5 * (arc[last] - arc[start])).min(DECK_PIN_TAPER_M);
        // Smoothstep from 1 at the boundary to 0 a taper in, flat at both
        // ends: the correction is a vertical transition curve, not a kink.
        let blend = |u: f64| {
            let v = (1.0 - u.clamp(0.0, 1.0)).powi(2);
            v * (1.0 + 2.0 * u.clamp(0.0, 1.0))
        };
        if start > 0 {
            // An at-grade anchor precedes: a band ends on this boundary.
            let e = road_m[start] - deck[start];
            if taper > 0.0 {
                for k in start..end {
                    let u = (arc[k] - arc[start]) / taper;
                    if u >= 1.0 {
                        break;
                    }
                    deck[k] += e * blend(u);
                }
            } else {
                deck[start] = road_m[start];
            }
        }
        if end < n {
            // An at-grade anchor follows.
            let e = road_m[last] - deck[last];
            if taper > 0.0 {
                for k in (start..end).rev() {
                    let u = (arc[last] - arc[k]) / taper;
                    if u >= 1.0 {
                        break;
                    }
                    deck[k] += e * blend(u);
                }
            } else {
                deck[last] = road_m[last];
            }
        }
    }
    deck
}

/// Densifies a corridor to ~`spacing_m` spacing ([`Mode::spacing_m`]),
/// returning the nodes, their cumulative metric arc length, and each node's
/// `(raw segment, t)` position on the input polyline — the parameter
/// [`spline_path`] evaluates the smoothing spline at.
fn densify(run: &[Coord], cos_lat: f64, spacing_m: f64) -> (Vec<Coord>, Vec<f64>, Vec<(usize, f64)>) {
    let mut nodes = vec![run[0]];
    let mut arc = vec![0.0];
    let mut params = vec![(0usize, 0.0)];
    let mut total = 0.0;
    for (k, w) in run.windows(2).enumerate() {
        let (p0, p1) = (w[0], w[1]);
        let n = ((metric_len(p0, p1, cos_lat) / spacing_m).ceil() as usize).clamp(1, MAX_NODES);
        for i in 1..=n {
            let t = i as f64 / n as f64;
            let c = Coord { x: p0.x + (p1.x - p0.x) * t, y: p0.y + (p1.y - p0.y) * t };
            total += metric_len(*nodes.last().expect("seeded"), c, cos_lat);
            nodes.push(c);
            arc.push(total);
            params.push((k, t));
        }
        if nodes.len() >= MAX_NODES {
            break;
        }
    }
    (nodes, arc, params)
}

/// Subsamples per raw segment used to measure the spline's own arc length when
/// inverting its parameterisation. The spline is a cubic over one segment, so
/// a chord sum this fine resolves its length to well under a centimetre on the
/// longest edges mapped roads carry.
const SPLINE_ARC_SAMPLES: usize = 32;

/// The line a deck box is swept along: a centripetal Catmull-Rom spline
/// through the raw corridor vertices, sampled at each densified node's
/// **station**. The raw polyline is a chain of chords — every vertex a visible
/// corner when swept as a 8 m-wide box — while the spline is C¹ through the
/// same vertices, so the swept edge curves instead of kinking. The centripetal
/// parameterisation (α = ½) is the standard choice that never loops or
/// overshoots on the wildly uneven vertex spacing of mapped roads.
///
/// **The station is the whole contract.** Every consumer pairs this array with
/// the densified one by index: `deck_nodes` places a cross-section at
/// `smooth_point(i, t)` and gives it the height `deck_m[i]`, and
/// `ground::corridor_earthworks` benches along `smooth[k]` at the road height
/// `road[k]`. So `smooth[i]` must be the *same point of the road* as
/// `nodes[i]`, only cleaned up. A centripetal Catmull-Rom's parameter is not
/// arc length — that is the point of the parameterisation — so evaluating it at
/// the densifier's chord fraction lands somewhere else along the segment
/// entirely, by a distance that grows with the segment's length and with how
/// uneven its neighbours are. Measured on the Montreux extract before this was
/// inverted: a median 0.37 m of slide, 9 % of nodes past 3.9 m, and a corridor
/// whose vertex spacing runs from metres to kilometres slid 721 m. A deck swept
/// at the wrong station carries the height solved for another one (the slide
/// times the grade) and lands its abutment short of or past the span it
/// belongs to — `verify::checks::abutment` measures both.
///
/// So the chord fraction is inverted into the parameter that sits at the same
/// *arc-length* fraction of the segment's own spline. Both ends are fixed
/// points of that map (the spline interpolates its control points), so the raw
/// vertices stay exactly where they were and only the interior is corrected.
fn spline_path(raw: &[Coord], params: &[(usize, f64)], cos_lat: f64) -> Vec<Coord> {
    let n = raw.len();
    let eval = |k: usize, t: f64| -> Coord {
        let p1 = raw[k.min(n - 1)];
        let p2 = raw[(k + 1).min(n - 1)];
        if n < 3 {
            return Coord { x: lerp(p1.x, p2.x, t), y: lerp(p1.y, p2.y, t) };
        }
        // Mirrored ghost points continue the end tangents.
        let p0 = if k == 0 { mirror(p1, p2) } else { raw[k - 1] };
        let p3 = if k + 2 >= n { mirror(p2, p1) } else { raw[k + 2] };
        catmull_rom(p0, p1, p2, p3, t, cos_lat)
    };
    // Cumulative arc length of each segment's spline at `SPLINE_ARC_SAMPLES`
    // even parameter steps, built once per segment rather than per query: a
    // densified corridor asks for many stations on each.
    let mut table: Vec<Vec<f64>> = Vec::with_capacity(n.saturating_sub(1).max(1));
    for k in 0..n.saturating_sub(1).max(1) {
        let mut cum = Vec::with_capacity(SPLINE_ARC_SAMPLES + 1);
        cum.push(0.0);
        let mut prev = eval(k, 0.0);
        let mut total = 0.0;
        for j in 1..=SPLINE_ARC_SAMPLES {
            let c = eval(k, j as f64 / SPLINE_ARC_SAMPLES as f64);
            total += metric_len(prev, c, cos_lat);
            cum.push(total);
            prev = c;
        }
        table.push(cum);
    }
    params
        .iter()
        .map(|&(k, t)| {
            let cum = &table[k.min(table.len() - 1)];
            let total = cum[SPLINE_ARC_SAMPLES];
            if total <= 0.0 {
                return eval(k, t);
            }
            // The parameter whose arc length is `t` of the way along.
            let target = t.clamp(0.0, 1.0) * total;
            let j = cum.partition_point(|&c| c < target).clamp(1, SPLINE_ARC_SAMPLES);
            let (lo, hi) = (cum[j - 1], cum[j]);
            let f = if hi > lo { (target - lo) / (hi - lo) } else { 0.0 };
            eval(k, (j as f64 - 1.0 + f) / SPLINE_ARC_SAMPLES as f64)
        })
        .collect()
}

/// Cumulative *signed* heading in radians along a polyline, one entry per node,
/// read over chords of at least [`HEADING_CHORD_M`].
///
/// Signed and cumulative is half the point. A road that curves accumulates turn
/// monotonically, so the difference between two nodes' entries is the angle the
/// road really turned through between them — which is what
/// [`SMOOTH_MAX_TURN_RAD`] budgets. Digitising zigzag turns one way and back
/// again, so it cancels and leaves the window open over the noise the smoother
/// exists to remove. Summing |turn| instead would do the opposite of what is
/// wanted: close the window tightest exactly where the line is noisiest.
///
/// The chord is the other half, and without it the cancellation never gets a
/// chance. Read edge by edge, a zigzag's *first* step already turns further
/// than the whole budget, so the window closes on the immediate neighbour and
/// the fit is skipped for want of points — the smoother switching itself off
/// precisely on the lines it exists for. Over a chord long enough to span a
/// period of the wiggle the two halves cancel before the angle is ever taken,
/// and what is left is the road's own bend.
fn cumulative_heading(nodes: &[Coord], arc: &[f64], cos_lat: f64) -> Vec<f64> {
    let n = nodes.len();
    let mut out = vec![0.0; n];
    if n < 3 {
        return out;
    }
    let dir = |a: Coord, b: Coord| -> Option<f64> {
        let (dx, dy) = ((b.x - a.x) * cos_lat, b.y - a.y);
        (dx.abs() > 1e-15 || dy.abs() > 1e-15).then(|| dy.atan2(dx))
    };
    let mut prev: Option<f64> = None;
    let mut anchor = 0usize;
    let mut total = 0.0;
    for i in 1..n {
        // Only close a chord once it is long enough to have a direction worth
        // reading. Under this length the heading is the noise's, not the
        // road's.
        if arc[i] - arc[anchor] < HEADING_CHORD_M && i + 1 < n {
            out[i] = total;
            continue;
        }
        if let Some(d) = dir(nodes[anchor], nodes[i]) {
            if let Some(p) = prev {
                // Wrap the turn into (−π, π]: a heading crossing due west would
                // otherwise read as a 2π turn the road never made.
                let mut turn = d - p;
                while turn > std::f64::consts::PI {
                    turn -= std::f64::consts::TAU;
                }
                while turn <= -std::f64::consts::PI {
                    turn += std::f64::consts::TAU;
                }
                total += turn;
            }
            prev = Some(d);
            anchor = i;
        }
        out[i] = total;
    }
    out
}

/// The ghost point beyond `a`, mirroring `b` through it.
fn mirror(a: Coord, b: Coord) -> Coord {
    Coord { x: 2.0 * a.x - b.x, y: 2.0 * a.y - b.y }
}

/// One centripetal Catmull-Rom evaluation (Barry–Goldman) on the segment
/// `p1..p2` at parameter `t ∈ [0, 1]`, with knots spaced by the square root
/// of the metric chord lengths (α = ½). Degenerate chords (duplicate
/// vertices) fall back to the straight segment.
fn catmull_rom(p0: Coord, p1: Coord, p2: Coord, p3: Coord, t: f64, cos_lat: f64) -> Coord {
    let knot = |a: Coord, b: Coord| -> f64 {
        let dx = (b.x - a.x) * cos_lat;
        let dy = b.y - a.y;
        (dx * dx + dy * dy).sqrt().sqrt()
    };
    let (d01, d12, d23) = (knot(p0, p1), knot(p1, p2), knot(p2, p3));
    if d01 < 1e-12 || d12 < 1e-12 || d23 < 1e-12 {
        return Coord { x: lerp(p1.x, p2.x, t), y: lerp(p1.y, p2.y, t) };
    }
    let (t0, t1) = (0.0, d01);
    let (t2, t3) = (t1 + d12, t1 + d12 + d23);
    let u = t1 + t * (t2 - t1);
    let mix = |a: Coord, b: Coord, ta: f64, tb: f64| -> Coord {
        let w = (tb - u) / (tb - ta);
        Coord { x: a.x * w + b.x * (1.0 - w), y: a.y * w + b.y * (1.0 - w) }
    };
    let a1 = mix(p0, p1, t0, t1);
    let a2 = mix(p1, p2, t1, t2);
    let a3 = mix(p2, p3, t2, t3);
    let b1 = mix(a1, a2, t0, t2);
    let b2 = mix(a2, a3, t1, t3);
    mix(b1, b2, t1, t2)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scene::DEG_M;

    /// The graded projection keeps the line monotone AND under the class
    /// ceiling: a bed lurch that the plain projection would answer with one
    /// impossible pitch and a plateau comes out as a constant-grade ramp.
    #[test]
    fn a_graded_monotone_line_never_exceeds_its_ceiling() {
        // 8 m spacing; a steady 50 % climb, then an 8 m lurch in one step,
        // then flat — the Collonge north-approach shape.
        let arcs: Vec<f64> = (0..12).map(|i| 8.0 * i as f64).collect();
        let mut h: Vec<f64> =
            vec![500.0, 504.0, 508.0, 512.0, 516.0, 524.0, 524.2, 524.4, 528.0, 532.0, 536.0, 540.0];
        let before = h.clone();
        monotone_project_graded(&mut h, &arcs, 1.0, 0.70);
        for k in 1..h.len() {
            let step = h[k] - h[k - 1];
            assert!(step >= -1e-9, "monotone broken at {k}: {step}");
            assert!(step <= 0.70 * 8.0 + 1e-9, "ceiling broken at {k}: {step}");
        }
        // It is a projection, not a rescale: the line stays near the input.
        let drift: f64 = h.iter().zip(&before).map(|(a, b)| (a - b).abs()).fold(0.0, f64::max);
        assert!(drift < 4.0, "moved {drift} m from the input");
    }

    /// A descending line takes the same bound, mirrored.
    #[test]
    fn a_graded_projection_mirrors_for_a_descending_line() {
        let arcs: Vec<f64> = (0..6).map(|i| 8.0 * i as f64).collect();
        let mut h: Vec<f64> = vec![540.0, 536.0, 526.0, 525.8, 521.8, 517.8];
        monotone_project_graded(&mut h, &arcs, -1.0, 0.70);
        for k in 1..h.len() {
            let step = h[k - 1] - h[k];
            assert!(step >= -1e-9, "monotone broken at {k}: {step}");
            assert!(step <= 0.70 * 8.0 + 1e-9, "ceiling broken at {k}: {step}");
        }
    }

    /// A structure span in corridor arc metres.
    fn span(arc0: f64, arc1: f64, level: i64) -> Span {
        let kind = match level.signum() {
            1 => SpanKind::Bridge,
            -1 => SpanKind::Tunnel,
            _ => SpanKind::Grade,
        };
        Span { arc0, arc1, level, kind }
    }

    /// A run of `n` evenly spaced vertices over `span_deg` of longitude at
    /// lat 46, and its total metric length.
    fn line(n: usize, span_deg: f64) -> (Vec<Coord>, f64) {
        let nodes: Vec<Coord> = (0..n)
            .map(|i| Coord { x: 6.0 + span_deg * i as f64 / (n - 1) as f64, y: 46.0 })
            .collect();
        let len_m = span_deg * 46.0_f64.to_radians().cos() * DEG_M;
        (nodes, len_m)
    }

    /// Solves a profile with an injected terrain sampler, bypassing the DEM.
    fn profile_from(seg: &[Coord], spans: &[Span], terrain: impl Fn(Coord) -> f64) -> Profile {
        let mut elev = |c: Coord| terrain(c);
        solve(seg, spans, Mode::Draped, &mut elev).expect("non-degenerate test corridor")
    }

    fn profile_from_limited(
        seg: &[Coord],
        spans: &[Span],
        max_grade: f64,
        terrain: impl Fn(Coord) -> f64,
    ) -> Profile {
        let mut elev = |c: Coord| terrain(c);
        solve(seg, spans, Mode::Engineered { grade: max_grade }, &mut elev).expect("non-degenerate test corridor")
    }

    #[test]
    fn a_sustained_steep_alignment_rides_its_own_track_bed() {
        // The Montreux–Glion case: a rack railway tagged `narrow_gauge`, whose
        // ground falls at 11 % for its whole mapped-at-grade length while the
        // class prior says 7 %. Held to the prior the profile dives tens of
        // metres under its own track bed; the ground is a measurement of what
        // was built, so it must win and the rail must stay on it.
        let (nodes, len_m) = line(80, 0.008);
        let slope = 0.11;
        let base = nodes[0].x;
        let cos = 46.0_f64.to_radians().cos();
        let ground = |c: Coord| 400.0 + (c.x - base) * cos * DEG_M * slope;
        let p = profile_from_limited(&nodes, &[span(0.0, len_m, 0)], 0.07, ground);

        assert!(p.max_grade().expect("engineered") >= slope - 0.01,
            "ceiling {:?} must rise to the ground's own {slope}", p.max_grade());
        let n = p.arc().len();
        for i in 0..n {
            assert!(p.at_grade()[i], "node {i} must stay at grade, not be absorbed");
            let standoff = p.road_m()[i] - p.terrain_m()[i];
            // The vertical-curve smoothing has no outward neighbour to chord
            // against at the very ends, so allow it a metre or two there; along
            // the run the rail must lie on its bed. Held to the 7 % prior these
            // same nodes sit tens of metres under it.
            let budget = if i < 3 || i + 3 >= n { 2.0 } else { 0.5 };
            assert!(standoff.abs() < budget,
                "node {i} stands {standoff:.2} m off its track bed");
        }
    }

    #[test]
    fn a_local_plunge_beside_a_structure_does_not_raise_the_ceiling() {
        // The case absorption exists for, which the measured ceiling must not
        // disarm: a level alignment that drops off a cliff at one end. The
        // steep stretch is local, so the corridor's measured grade stays at
        // the flat majority and the class prior still stands.
        let (nodes, len_m) = line(80, 0.008);
        let base = nodes[0].x;
        let cos = 46.0_f64.to_radians().cos();
        let ground = |c: Coord| {
            let s = (c.x - base) * cos * DEG_M;
            // Flat for 90 % of the run, then a gorge wall.
            if s < len_m * 0.9 { 400.0 } else { 400.0 - (s - len_m * 0.9) * 0.6 }
        };
        let p = profile_from_limited(&nodes, &[span(0.0, len_m, 0)], 0.07, ground);
        assert!(p.max_grade().expect("engineered") < 0.08,
            "a local cliff must not raise the ceiling, got {:?}", p.max_grade());
    }

    #[test]
    fn a_steep_class_is_not_clipped_below_the_bed_its_prior_was_written_for() {
        // A funicular's prior is 45 % and the Territet–Glion bed measures
        // 56.9 %. A flat plausibility bound would clip it to 30 % and bury the
        // line in the hillside it is pinned to; the class earns headroom over
        // its own convention, while a gentle class keeps the flat bound.
        let arc: Vec<f64> = (0..40).map(|i| i as f64 * 8.0).collect();
        let bed: Vec<f64> = arc.iter().map(|a| 400.0 + a * 0.569).collect();
        let at_grade = vec![true; arc.len()];
        let funicular = measured_grade(&arc, &bed, &at_grade, 0.45).expect("enough at grade");
        assert!(funicular > 0.55, "funicular bed clipped to {funicular}");
        // The same ground under a narrow-gauge prior is a cliff, not a bed.
        let rail = measured_grade(&arc, &bed, &at_grade, 0.07).expect("enough at grade");
        assert!((rail - MEASURED_GRADE_MAX).abs() < 1e-9, "gentle class claimed {rail}");
    }

    #[test]
    fn measured_grade_ignores_a_corridor_with_too_little_at_grade() {
        // Below `MEASURED_GRADE_MIN_M` of mapped-at-grade length the sample is
        // a few edges beside a structure, not a reading of the alignment.
        let arc: Vec<f64> = (0..8).map(|i| i as f64 * 8.0).collect();
        let reference: Vec<f64> = arc.iter().map(|a| a * 0.2).collect();
        assert_eq!(measured_grade(&arc, &reference, &vec![true; 8], 0.07), None);
    }

    #[test]
    fn vgrades_round_an_engineered_corner_but_pin_draped_nodes() {
        // A road draped at 100 m on both ends, lifted into a sharp tent (an
        // embankment) over the middle with a corner apex. Smoothing must round
        // the apex (drop its vertical curvature) while leaving the draped flanks
        // exactly on the terrain — the road must not float off the ground.
        let n = 21;
        let arc: Vec<f64> = (0..n).map(|i| i as f64 * 8.0).collect();
        let terrain = vec![100.0; n];
        let mut road: Vec<f64> = (0..n)
            .map(|i| {
                if i <= 4 || i >= 16 {
                    100.0
                } else if i <= 10 {
                    100.0 + (i - 4) as f64 * 2.0
                } else {
                    100.0 + (16 - i) as f64 * 2.0
                }
            })
            .collect();
        let at_grade = vec![true; n];
        let curv = |r: &[f64], i: usize| (r[i - 1] - 2.0 * r[i] + r[i + 1]).abs();
        let before = curv(&road, 10);
        smooth_vgrades(&arc, &mut road, &terrain, &at_grade);
        assert!(curv(&road, 10) < before, "apex curvature {} must drop below {before}", curv(&road, 10));
        // Draped flank nodes stay pinned to the terrain (no float).
        assert!((road[0] - 100.0).abs() < 1e-9);
        assert!((road[2] - 100.0).abs() < 1e-9);
        assert!((road[18] - 100.0).abs() < 1e-9);
        // The rounded apex stays below the original corner (a crest curve pulls
        // in) but well within the deviation budget.
        assert!(road[10] <= 112.0 + 1e-9 && road[10] > 100.0);
    }

    #[test]
    fn close_notches_fills_narrow_and_keeps_deep() {
        // 11 nodes, 30 m apart; a single-node 25 m-deep dip at arc 150.
        let arc: Vec<f64> = (0..11).map(|i| i as f64 * 30.0).collect();
        let mut h = vec![500.0; 11];
        h[5] = 475.0;
        let closed = close_notches(&arc, &h);
        assert_eq!(closed[5], 475.0, "a 25 m notch is past the fill cap");
        // A shallow 8 m dip: filled to the rim.
        h[5] = 492.0;
        let closed = close_notches(&arc, &h);
        assert_eq!(closed[5], 500.0, "an 8 m notch must fill");
        // A ramp passes through untouched (closing is identity on monotone).
        let ramp: Vec<f64> = arc.iter().map(|a| 400.0 + a * 0.2).collect();
        let closed = close_notches(&arc, &ramp);
        for (c, r) in closed.iter().zip(&ramp) {
            assert!((c - r).abs() < 1e-9, "a ramp must close to itself");
        }
        // A bump is never cut (closing only fills).
        let mut bump = vec![500.0; 11];
        bump[5] = 512.0;
        let closed = close_notches(&arc, &bump);
        assert_eq!(closed[5], 512.0, "closing must not shave bumps");
    }

    /// The refused-notch report: exactly the reverted run, rims excluded, and
    /// nothing at all for a notch the closing filled. This interval is what
    /// `solve::promote_notch_crossings` turns into a bridge span (S20), so its
    /// bounds are load-bearing: a rim node inside the interval would move a
    /// structure anchor off the rim.
    #[test]
    fn refused_notches_reports_the_reverted_run_and_only_it() {
        // Same fixture as the closing test: a single-node 25 m dip at arc 150
        // between 30 m-spaced nodes.
        let arc: Vec<f64> = (0..11).map(|i| i as f64 * 30.0).collect();
        let mut h = vec![500.0; 11];
        h[5] = 475.0;
        let refused = refused_notches(&arc, &h);
        assert_eq!(refused, vec![(150.0, 150.0)], "the refused run is the dip node alone");
        // A filled notch reports nothing.
        h[5] = 492.0;
        assert!(refused_notches(&arc, &h).is_empty(), "a filled notch is not refused");
        // A two-node-wide refused slot spans both nodes, not the rims.
        let arc2: Vec<f64> = (0..12).map(|i| i as f64 * 15.0).collect();
        let mut slot = vec![500.0; 12];
        slot[5] = 480.0;
        slot[6] = 480.0;
        let refused = refused_notches(&arc2, &slot);
        assert_eq!(refused, vec![(75.0, 90.0)], "the interval covers the interior, rims outside");
        // A refused *crest* must not surface as a refused notch: closing only
        // fills, so a 25 m bump yields nothing (the open_bumps dual runs on
        // negated heights and must stay a separate channel).
        let mut crest = vec![500.0; 11];
        crest[5] = 525.0;
        assert!(refused_notches(&arc, &crest).is_empty(), "a crest is not a notch");
    }

    #[test]
    fn open_bumps_shaves_narrow_and_keeps_tall() {
        // 11 nodes, 25 m apart (BUMP_SPAN_M = 50, so a single-node crest is
        // narrower than the span); a 3 m noise crest at arc 125.
        let arc: Vec<f64> = (0..11).map(|i| i as f64 * 25.0).collect();
        let mut h = vec![500.0; 11];
        h[5] = 503.0;
        let opened = open_bumps(&arc, &h);
        assert_eq!(opened[5], 500.0, "a 3 m noise crest must shave");
        // A crest past the shave cap is a genuine hill (S9): kept.
        h[5] = 506.0;
        let opened = open_bumps(&arc, &h);
        assert_eq!(opened[5], 506.0, "a 6 m crest is past the shave cap");
        // A ramp passes through untouched (opening is identity on monotone).
        let ramp: Vec<f64> = arc.iter().map(|a| 400.0 + a * 0.2).collect();
        let opened = open_bumps(&arc, &ramp);
        for (o, r) in opened.iter().zip(&ramp) {
            assert!((o - r).abs() < 1e-9, "a ramp must open to itself");
        }
        // A notch is never filled (opening only shaves).
        let mut notch = vec![500.0; 11];
        notch[5] = 497.0;
        let opened = open_bumps(&arc, &notch);
        assert_eq!(opened[5], 497.0, "opening must not fill notches");
        // A flat-topped hill wider than BUMP_SPAN_M is genuine relief: kept
        // exactly (a sharp apex would be rounded by up to slope·span/2 — the
        // crest vertical curve — but a plateau fits the structuring element).
        let arc17: Vec<f64> = (0..17).map(|i| i as f64 * 25.0).collect();
        let hill: Vec<f64> = arc17
            .iter()
            .map(|a| 500.0 + 3.0 * (1.0 - (((a - 200.0).abs() - 50.0).max(0.0) / 100.0)).min(1.0))
            .collect();
        let opened = open_bumps(&arc17, &hill);
        for (o, r) in opened.iter().zip(&hill) {
            assert!((o - r).abs() < 1e-9, "a wide flat-topped hill must pass through");
        }
    }

    #[test]
    fn condition_reference_is_symmetric() {
        // A notch and a bump side by side: closing fills the notch, opening
        // shaves the bump, and neither operator disturbs the other's fix.
        let arc: Vec<f64> = (0..17).map(|i| i as f64 * 25.0).collect();
        let mut h = vec![500.0; 17];
        h[4] = 494.0; // 6 m notch — fillable
        h[12] = 503.0; // 3 m bump — shavable
        let cond = condition_reference(&arc, &h);
        assert_eq!(cond[4], 500.0, "the notch must fill");
        assert_eq!(cond[12], 500.0, "the bump must shave");
        for (i, (c, r)) in cond.iter().zip(&h).enumerate() {
            if i != 4 && i != 12 {
                assert!((c - r).abs() < 1e-9, "flat ground must pass through at {i}");
            }
        }
    }

    #[test]
    fn an_unengineered_road_spans_a_narrow_notch() {
        // A draped (no grade ceiling) road across a 40 m-wide, 10 m-deep DEM
        // notch — the image of a gully the real road crosses on fill and a
        // culvert. The solved road must carry across at rim height, not dive
        // through the V; the wide terrain elsewhere is followed as before.
        let (seg, len) = line(128, 0.01);
        let mid = seg[64].x;
        let cos_lat = 46.0_f64.to_radians().cos();
        let terrain = move |c: Coord| {
            let dm = (c.x - mid).abs() * cos_lat * DEG_M;
            if dm < 20.0 { 500.0 - 10.0 * (1.0 - dm / 20.0) } else { 500.0 }
        };
        let nodes: Vec<Coord> = seg.clone();
        let p = solve(&nodes, &[span(0.0, len, 0)], Mode::Draped, &mut |c| terrain(c))
            .expect("a profile");
        let road = p.height_at(mid, 46.0);
        assert!(
            (road - 500.0).abs() < 0.75,
            "the road must span the notch at rim height, got {road}"
        );
        // A gorge deeper than the fill cap keeps the terrain in the anchor
        // surface — the *profile* still dives here, because the crossing is a
        // structure decision made a level up: `solve::promote_notch_crossings`
        // splices a bridge span over exactly this V before any profile is
        // solved (S20), so a bare all-grade span list means the promotion
        // declined (a corridor end, water, an annotated claim).
        let gorge = move |c: Coord| {
            let dm = (c.x - mid).abs() * cos_lat * DEG_M;
            if dm < 25.0 { 500.0 - 25.0 * (1.0 - dm / 25.0) } else { 500.0 }
        };
        let p = solve(&nodes, &[span(0.0, len, 0)], Mode::Draped, &mut |c| gorge(c))
            .expect("a profile");
        let road = p.height_at(mid, 46.0);
        assert!(road < 490.0, "a gorge past the fill cap keeps the terrain, got {road}");
    }

    #[test]
    fn bridge_spans_a_ravine_on_the_road_grade_not_the_terrain() {
        // At-grade approaches at 100 m on both ends; a bridge span in the middle
        // over a 60 m-deep ravine (scenario S1). The deck must stay near 100 m
        // (the road grade the anchors imply), high above the ravine floor.
        let (seg, len) = line(256, 0.06);
        let mid = seg[128].x;
        let cos_lat = 46.0_f64.to_radians().cos();
        let terrain = move |c: Coord| {
            let dm = (c.x - mid).abs() * cos_lat * DEG_M;
            if dm < 300.0 { 100.0 - 60.0 * (1.0 - dm / 300.0) } else { 100.0 }
        };
        let p = profile_from(&seg, &[span(0.34 * len, 0.66 * len, 1)], terrain);
        let floor = terrain(Coord { x: mid, y: 46.0 });
        let deck = p.height_at(mid, 46.0);
        assert!(deck - floor > 45.0, "deck {deck} only {} m over the ravine {floor}", deck - floor);
    }

    #[test]
    fn deck_holds_the_road_grade_over_a_steep_flank() {
        // A bridge span whose terrain climbs steeply (a hillside), but whose
        // at-grade anchors are 100 m and 110 m: the deck must follow the gentle
        // ~anchor grade, never the steep terrain.
        let (seg, len) = line(256, 0.06);
        let cos_lat = 46.0_f64.to_radians().cos();
        let len_m = 0.06 * cos_lat * DEG_M;
        let terrain = move |c: Coord| 100.0 + 300.0 * (c.x - 6.0) * cos_lat * DEG_M / len_m;
        let p = profile_from(&seg, &[span(0.03 * len, 0.97 * len, 1)], move |c| {
            let x = (c.x - 6.0) * cos_lat * DEG_M / len_m; // 0..1
            if x < 0.03 {
                100.0
            } else if x > 0.97 {
                110.0
            } else {
                terrain(c)
            }
        });
        let a = p.height_at(6.0 + 0.06 * 0.4, 46.0);
        let b = p.height_at(6.0 + 0.06 * 0.6, 46.0);
        let dx = 0.06 * 0.2 * cos_lat * DEG_M;
        let grade = (b - a).abs() / dx;
        assert!(grade < 0.15, "deck grade {grade} too steep (a={a} b={b})");
    }

    #[test]
    fn the_smooth_snap_carries_a_painted_line_at_its_own_offset() {
        // A raw centerline with a metre of digitising wiggle on it, smoothed
        // the way a corridor's sweep line is. Paint generated 4.2 m to the left
        // of the raw line — an edge line on a 9 m carriageway — must arrive
        // 4.2 m to the left of the *smoothed* line, not on top of it.
        //
        // Projecting it onto the curve instead collapsed both edge lines and
        // every lane divider onto the axis, which is the whole cross-section
        // thrown away one stage after it was computed.
        let (raw, _) = line(64, 0.02);
        let cos_lat = run_cos_lat(&raw);
        let wiggly: Vec<Coord> = raw
            .iter()
            .enumerate()
            .map(|(i, c)| Coord {
                x: c.x,
                y: c.y + if i % 2 == 0 { 1.0 } else { -1.0 } / DEG_M,
            })
            .collect();
        let n = wiggly.len();
        let arc = cumulative(&wiggly);
        let p = Profile {
            cos_lat,
            smooth: smooth_path(&wiggly),
            nodes: wiggly.clone(),
            arc,
            road_m: vec![100.0; n],
            deck_m: vec![100.0; n],
            terrain_m: vec![0.0; n],
            at_grade: vec![true; n],
            max_grade: None,
        };

        // The distance from a snapped point to the smoothed line, measured by
        // brute force against its own Catmull-Rom samples.
        let to_smooth = |c: Coord| -> f64 {
            let edges = p.nodes.len() - 1;
            let mut best = f64::INFINITY;
            for i in 0..edges {
                for k in 0..=20 {
                    let s = p.smooth_point(i, k as f64 / 20.0);
                    let de = (s.x - c.x) * cos_lat * DEG_M;
                    let dn = (s.y - c.y) * DEG_M;
                    best = best.min((de * de + dn * dn).sqrt());
                }
            }
            best
        };

        let (mut centre, mut edge) = (0usize, 0usize);
        for i in 8..n - 8 {
            let raw_pt = wiggly[i];
            // The centerline itself still lands *on* the sweep line.
            let on = p.smooth_at(raw_pt.x, raw_pt.y, 6.0).expect("an interior vertex snaps");
            assert!(to_smooth(on) < 0.05, "centerline pulled {} m off the sweep line", to_smooth(on));
            centre += 1;

            // A painted line 4.2 m to the left keeps its 4.2 m.
            let left = Coord { x: raw_pt.x, y: raw_pt.y + 4.2 / DEG_M };
            let Some(painted) = p.smooth_at(left.x, left.y, 6.0) else { continue };
            let d = to_smooth(painted);
            assert!(
                (d - 4.2).abs() < 0.35,
                "edge line landed {d:.2} m from the sweep line, not 4.2 m"
            );
            edge += 1;
        }
        assert!(centre > 20 && edge > 20, "too few vertices exercised: {centre}/{edge}");
    }

    #[test]
    fn deck_line_is_one_straight_ramp_over_a_folded_profile() {
        // A road profile that climbs steadily but hooks down over its last few
        // nodes (an abutment touchdown — the shape that folds a swept box).
        // `deck_line` must return a single straight ramp.
        let (nodes, _) = line(24, 0.02);
        let arc = cumulative(&nodes);
        let n = nodes.len();
        let mut road_m: Vec<f64> =
            (0..n).map(|i| 100.0 + 12.0 * i as f64 / (n - 1) as f64).collect();
        road_m[n - 1] = 105.0;
        road_m[n - 2] = 108.0;
        road_m[n - 3] = 110.0;
        let deck_m = deck_ramp(&arc, &road_m, &vec![false; n]);
        let p = Profile {
            cos_lat: run_cos_lat(&nodes),
            smooth: nodes.clone(),
            nodes: nodes.clone(),
            arc,
            road_m,
            deck_m,
            terrain_m: vec![0.0; n],
            at_grade: vec![false; n],
            max_grade: None,
        };

        let deck = p.deck_line(&nodes);
        for w in deck.windows(3) {
            let second = (w[2] - w[1]) - (w[1] - w[0]);
            assert!(second.abs() < 1e-6, "deck not straight: {deck:?}");
        }
        assert!(deck[n - 1] > deck[0], "deck should ramp up, got {deck:?}");
    }

    #[test]
    fn deck_ramp_arrives_at_the_road_where_a_band_meets_it() {
        // An anchored span whose road bulges 3 m mid-span (a clearance lift)
        // and so is nowhere near a chord: the straight fit misses the road at
        // both boundary nodes by the bulge's share. The band ends at the span
        // arc reading `road_at_arc`, inside the boundary node's own segment —
        // so the deck agrees with it exactly iff the boundary structure node
        // carries the road height (seam.band_deck_step, invariant 2).
        let n = 61;
        let arc: Vec<f64> = (0..n).map(|i| i as f64 * 5.0).collect();
        let at_grade: Vec<bool> = (0..n).map(|i| !(10..=50).contains(&i)).collect();
        let road: Vec<f64> = (0..n)
            .map(|i| {
                let u = (i as f64 - 10.0) / 40.0; // 0..1 across the span
                if (0.0..=1.0).contains(&u) {
                    100.0 + 3.0 * (std::f64::consts::PI * u).sin()
                } else {
                    100.0
                }
            })
            .collect();
        let deck = deck_ramp(&arc, &road, &at_grade);
        assert!(
            (deck[10] - road[10]).abs() < 1e-9 && (deck[50] - road[50]).abs() < 1e-9,
            "deck must land on the road at the anchored boundaries: \
             {} vs {} and {} vs {}",
            deck[10],
            road[10],
            deck[50],
            road[50]
        );
        // Mid-span keeps the fitted line, not the chord between the boundary
        // heights: tilting the whole span to its ends is the rejected first
        // cut of the datum work (it daylit bore roofs mid-span).
        assert!(deck[30] > 101.0, "mid-span sank toward the chord: {}", deck[30]);
        // Beyond the taper the ramp is straight again.
        let clear: Vec<f64> = (0..n)
            .filter(|&k| {
                !at_grade[k]
                    && arc[k] > arc[10] + DECK_PIN_TAPER_M
                    && arc[k] < arc[50] - DECK_PIN_TAPER_M
            })
            .map(|k| deck[k])
            .collect();
        for w in clear.windows(3) {
            let second = (w[2] - w[1]) - (w[1] - w[0]);
            assert!(second.abs() < 1e-6, "deck bent beyond the taper");
        }
    }

    #[test]
    fn a_short_span_pins_both_ends_exactly() {
        // Two structure nodes 4 m apart between anchors: the taper is half
        // the span, so each end's correction dies before the other node and
        // both still land exactly on the road.
        let arc: Vec<f64> = vec![0.0, 4.0, 8.0, 12.0, 16.0];
        let at_grade = vec![true, false, false, true, true];
        let road = vec![100.0, 100.9, 101.4, 102.0, 102.0];
        let deck = deck_ramp(&arc, &road, &at_grade);
        assert!((deck[1] - road[1]).abs() < 1e-9, "low end missed: {} vs {}", deck[1], road[1]);
        assert!((deck[2] - road[2]).abs() < 1e-9, "high end missed: {} vs {}", deck[2], road[2]);
    }

    #[test]
    fn enters_a_hillside_by_occlusion_not_a_plunge() {
        // ground | bridge over a ravine | tunnel into a hill (S5/S7). The deck
        // holds the gentle road grade across the bridge — standing high over
        // the ravine — and where the hill rises above it (the tunnel side) it
        // passes *under* the terrain rather than plunging down to it.
        let (seg, len) = line(256, 0.06);
        let cos_lat = 46.0_f64.to_radians().cos();
        let (x0, x1) = (seg[0].x, seg[seg.len() - 1].x);
        let portal = x0 + 0.6 * (x1 - x0);
        let ravine = x0 + 0.45 * (x1 - x0);
        let terrain = move |c: Coord| {
            let dr = (c.x - ravine).abs() * cos_lat * DEG_M;
            let base = if dr < 120.0 { 100.0 - 50.0 * (1.0 - dr / 120.0) } else { 100.0 };
            if c.x > portal { 100.0 + 300.0 * (c.x - portal) / (x1 - portal) } else { base }
        };
        let p = profile_from(
            &seg,
            &[span(0.3 * len, 0.6 * len, 1), span(0.6 * len, len, -5)],
            terrain,
        );
        let over = p.height_at(ravine, 46.0) - terrain(Coord { x: ravine, y: 46.0 });
        assert!(over > 30.0, "deck only {over} m over the ravine");
        let into = p.height_at(portal + 0.5 * (x1 - portal), 46.0)
            - terrain(Coord { x: portal + 0.5 * (x1 - portal), y: 46.0 });
        assert!(into < 0.0, "deck rides {into} m above the hill instead of under it");
    }

    #[test]
    fn engineered_grade_flattens_a_transient_bump() {
        // A flat road crossed by a 6 m terrain bump over 60 m. The road cuts
        // straight through it (within the deviation budget), holding a gentle
        // grade instead of tracing the bump's steep flanks.
        let n = 31;
        let arc: Vec<f64> = (0..n).map(|i| i as f64 * 10.0).collect();
        let at_grade = vec![true; n];
        let terrain: Vec<f64> = (0..n)
            .map(|i| 100.0 + 6.0 * (1.0 - ((i as f64 - 15.0).abs() / 3.0)).max(0.0))
            .collect();
        let mut road = terrain.clone();
        limit_road_grade(&arc, &mut road, &terrain, &at_grade, 0.06, MAX_ROAD_DEVIATION_M);
        for i in 1..n {
            let g = ((road[i] - road[i - 1]) / (arc[i] - arc[i - 1])).abs();
            assert!(g <= 0.06 + 1e-9, "grade {g} too steep at node {i}");
        }
        for i in 0..n {
            assert!((road[i] - terrain[i]).abs() <= MAX_ROAD_DEVIATION_M + 1e-9);
        }
    }

    #[test]
    fn a_deck_anchored_on_a_rim_roll_off_launches_from_the_rim() {
        // A gorge bridge whose annotation starts a few metres down the rim
        // roll-off: the anchor point-samples the wall (90 m) instead of the
        // rim (100 m). Without the anchor seek the deck launches 10 m low and
        // the approach digs a −6 % cut below the plateau to reach it; with it
        // the anchor snaps to the rim crest and the approach stays on the
        // ground.
        let (seg, len) = line(512, 0.06);
        let cos_lat = 46.0_f64.to_radians().cos();
        let len_m = 0.06 * cos_lat * DEG_M;
        let x_at = move |c: Coord| (c.x - 6.0) * cos_lat * DEG_M / len_m; // 0..1
        let terrain = move |c: Coord| {
            let x = x_at(c);
            if x < 0.4 || x > 0.6 {
                100.0 // the plateaus
            } else if x < 0.42 {
                100.0 - 40.0 * (x - 0.4) / 0.02 // west wall
            } else if x > 0.58 {
                100.0 - 40.0 * (0.6 - x) / 0.02 // east wall
            } else {
                60.0 // gorge floor
            }
        };
        // Annotated span edges ~23 m down each roll-off (terrain ≈ 90 m).
        let p = profile_from_limited(&seg, &[span(0.405 * len, 0.595 * len, 1)], 0.06, terrain);
        let (arc, road, terr, at_grade) = (p.arc(), p.road_m(), p.terrain_m(), p.at_grade());
        // The approach stays on the plateau — no cut diving toward a low
        // launch point.
        let i = arc.iter().position(|&a| a >= 0.39 * len).unwrap();
        assert!(at_grade[i], "the plateau approach stays at grade");
        assert!(
            (road[i] - terr[i]).abs() < 0.5,
            "approach must sit on the plateau, road {} terr {}",
            road[i],
            terr[i]
        );
        // The deck launches from rim height, not from the roll-off sample.
        let launch = p.height_at(6.0 + 0.06 * 0.405, 46.0);
        assert!(launch > 98.0, "deck must launch from the ~100 m rim, got {launch}");
        // And the roll-off itself is structure now.
        let j = arc.iter().position(|&a| a >= 0.41 * len).unwrap();
        assert!(!at_grade[j], "the roll-off joins the structure");
    }

    #[test]
    fn a_structure_ending_at_a_cliff_is_extended_not_pitched() {
        // A bridge annotation ends where the terrain climbs a ~40 % gorge
        // wall (the mapped span stops before the road actually reaches the
        // ground). The deviation budget cannot carry an at-grade road up the
        // wall, so the old solve left an impossible pitch there; the fix
        // absorbs the stretch into the structure and re-chords. The road must
        // hold a plausible grade everywhere it is at grade, and the wall
        // nodes must no longer be anchors.
        let (seg, len) = line(512, 0.06);
        let cos_lat = 46.0_f64.to_radians().cos();
        let len_m = 0.06 * cos_lat * DEG_M;
        let x_at = move |c: Coord| (c.x - 6.0) * cos_lat * DEG_M / len_m; // 0..1
        let terrain = move |c: Coord| {
            let x = x_at(c);
            if x < 0.5 {
                100.0 // approach and gorge rim
            } else if x < 0.51 {
                100.0 + 3000.0 * (x - 0.5) // the wall: +30 m over ~46 m
            } else {
                130.0 // the plateau above
            }
        };
        // Bridge annotated up to the foot of the wall.
        let p = profile_from_limited(&seg, &[span(0.3 * len, 0.5 * len, 1)], 0.06, terrain);
        let (arc, road, at_grade) = (p.arc(), p.road_m(), p.at_grade());
        for i in 1..arc.len() {
            if !(at_grade[i] && at_grade[i - 1]) || arc[i] <= arc[i - 1] {
                continue;
            }
            let g = (road[i] - road[i - 1]).abs() / (arc[i] - arc[i - 1]);
            assert!(g <= 0.09 + 1e-6, "at-grade pitch {g} survives at arc {}", arc[i]);
        }
        // The wall itself is structure now, not anchored road.
        let mid_wall = 0.505 * len;
        let i = arc.iter().position(|&a| a >= mid_wall).unwrap();
        assert!(!at_grade[i], "the wall must be absorbed into the structure");
        // And the plateau beyond is still ordinary at-grade road on the ground.
        let plateau = 0.7 * len;
        let j = arc.iter().position(|&a| a >= plateau).unwrap();
        assert!(at_grade[j], "absorption must stop once the ground is feasible");
        assert!((road[j] - 130.0).abs() < 1.0, "plateau road sits on the ground");
    }

    #[test]
    fn road_hugs_the_ground_on_a_long_steep_climb() {
        // Terrain climbing a sustained 15 % — far above the grade ceiling (S9).
        // The deviation budget wins: the road follows the slope, never drifting
        // more than MAX_ROAD_DEVIATION_M from the terrain.
        let n = 60;
        let arc: Vec<f64> = (0..n).map(|i| i as f64 * 10.0).collect();
        let at_grade = vec![true; n];
        let terrain: Vec<f64> = (0..n).map(|i| 100.0 + 0.15 * arc[i]).collect();
        let mut road = terrain.clone();
        limit_road_grade(&arc, &mut road, &terrain, &at_grade, 0.06, MAX_ROAD_DEVIATION_M);
        for i in 0..n {
            assert!(
                (road[i] - terrain[i]).abs() <= MAX_ROAD_DEVIATION_M + 1e-9,
                "road drifted {} m from terrain at node {i}",
                (road[i] - terrain[i]).abs()
            );
        }
    }

    #[test]
    fn a_gentle_road_is_left_on_the_terrain() {
        // A motorway on terrain that never exceeds the ceiling stays draped:
        // the limiter only intervenes where the ground is too steep.
        let (seg, _) = line(120, 0.05);
        let cos_lat = 46.0_f64.to_radians().cos();
        let terrain = move |c: Coord| 100.0 + 0.03 * (c.x - 6.0) * cos_lat * DEG_M;
        let p = profile_from_limited(&seg, &[], 0.06, terrain);
        let mid = seg[60].x;
        assert!(
            (p.height_at(mid, 46.0) - terrain(Coord { x: mid, y: 46.0 })).abs() < 0.5,
            "a gentle road should stay on the ground"
        );
    }

    #[test]
    fn meets_the_ground_at_at_grade_anchors() {
        // Flat ground, a bridge in the middle: the road surface at the at-grade
        // ends sits on the ground, meeting the draped approach road there.
        let (seg, len) = line(128, 0.04);
        let p = profile_from(&seg, &[span(0.4 * len, 0.6 * len, 1)], |_| 100.0);
        assert!((p.height_at(seg[0].x, seg[0].y) - 100.0).abs() < 0.5);
        assert!((p.height_at(seg[seg.len() - 1].x, seg[seg.len() - 1].y) - 100.0).abs() < 0.5);
    }

    #[test]
    fn flat_overpass_sits_at_grade() {
        // A bridge over flat ground (no dip): the road surface is the single
        // model, so the deck lies flush at the grade — no clearance offset that
        // would float its ends above an adjoining tunnel. (Real overpass lift
        // comes with crossing detection, milestone M-b.)
        let (seg, len) = line(256, 0.06);
        let p = profile_from(&seg, &[span(0.3 * len, 0.7 * len, 1)], |_| 100.0);
        assert!((p.height_at(seg[0].x, seg[0].y) - 100.0).abs() < 0.5);
        assert!((p.height_at(seg[128].x, 46.0) - 100.0).abs() < 0.5);
    }
}

