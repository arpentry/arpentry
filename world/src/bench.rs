//! The bench: the room holds its height.
//!
//! The mesh step put every paved triangle on the raw ground, coplanar with
//! it. This step gives the room the height the profile solved.
//!
//! **The room takes the profile.** The height of a point of the room — the
//! carriageway, its kerb returns and the pavement together — is the profile
//! height at the perpendicular foot on the nearest carriageway axis, and
//! the pavement stands [`KERB_RISE_M`] above it. The road is therefore
//! level crosswise: a 5.5 m residential along the contour of a 30 % slope
//! is cut 0.825 m at its uphill kerb and filled 0.825 m at its downhill
//! one, exactly, because half its width times the slope is what the ground
//! does across it. A crossfall is a later refinement, and the earth those
//! two numbers name is the earth the ground answers with below.
//!
//! Only the *ground* pieces' axes are in the field. A deck's height is the
//! chord the profile solved and belongs to the structure step; were it in
//! the field, the road under a viaduct would take the viaduct's height
//! wherever the deck's axis happened to be the nearer one.
//!
//! **A cross-section reaches as far as the room does.** The pavement rides
//! the road while it is within [`ROOM_REACH_M`] of the asphalt's edge —
//! the reach the room step itself paves to — and a walk band farther out
//! than that samples the ground, as stratum D says a draped class must.
//! Neither half of that rule will do alone, and the specimen said so.
//! Asked per region, a 20 m footway that merely touches a kerb at one end
//! was dragged 5.88 m up the ramp with the road at its far end. Asked per
//! point with nothing in between, it would stand on a 3.8 m cliff at the
//! line where the answer changed. So past the reach the walk comes down a
//! face at [`EARTHWORK_BATTER`] and stops exactly where it meets the
//! ground — which is the batter of this step's second half, carried by
//! the walk surface because the terrain cannot carry it yet. A fixed band
//! was tried first and rejected: 6 m of it on the box's 30 % flank came
//! out at 103 %, steeper than the wall it was there to avoid, and the
//! `step` check said so before the eye did.
//!
//! **Where two carriageways' domains meet at different heights** — two
//! terraces on a flank, two one-way carriageways across a slope — the
//! field steps on the line between them, and the mesh draws a face there:
//! a retaining wall, which is what the hillside physically has. The
//! alternative, a blend between the two, would ramp a pavement at 60 %
//! between two terraces, which is spectacle (invariant 6). The `step`
//! check counts those edges and the plan view marks them.
//!
//! **Except where they meet.** Every leg of a junction is level
//! crosswise, so on a flank the legs' cross-sections disagree everywhere
//! off the connector they share — by `g·s` at `s` metres from it where a
//! leg climbing a flank of grade `g` meets one along its contour — and the
//! nearest axis alone stepped on the line where two legs are equidistant:
//! up to 0.875 m on a 15 % flank, with junction corners whose edges
//! climbed 100 to 220 %, and a bump in every junction on the box's
//! hillside. So near a connector two or more axes share, the legs meeting
//! there are blended ([`Field::at`]). The legs agree at the connector — the
//! profile pins it — so what is blended is a disagreement that starts at
//! nothing and grows: a warp in the junction, not a ramp between terraces.
//! And a road that meets another nowhere near is never blended with it,
//! so the terraces keep their wall.
//!
//! **And the ground answers.** [`Ground`] is the terrain with the room cut
//! out of it: the room's own height at its outline, a face at
//! [`EARTHWORK_BATTER`] out of it — cut uphill, fill downhill — stopping
//! exactly where it meets the natural ground, and the natural ground
//! everywhere beyond. The terrain mesh is re-triangulated over
//! `rect − room` on the same lattice by the same mesher the room used, so
//! the ground stops at the kerb: no triangle of it lies under the asphalt,
//! which is where every artefact of a ground drawn beneath an opaque
//! surface lives (`data/plans/terrain-hole-plan.md`).
//!
//! **The seam is read, not recomputed.** Every vertex of the outline is a
//! vertex of the room's own mesh, so the ground takes its height from
//! there rather than evaluating anything: `contact` reads 2.5e-7 m on the
//! loop box, and `seam` — outline vertices the room's mesh did not have —
//! reads 0.
//!
//! **What this step does not do yet.** Neither the batter's toe nor the
//! wall at a bench's edge is a breakline, so a lattice triangle may
//! straddle one and stand off the engineered ground between its
//! vertices. `off` measures that, and the walls are where it lives: on
//! the loop box it reads 17 m against a tallest `wall` of 12.2 m, a
//! triangle spanning a wall *and* the batter beside it. Nothing
//! re-drapes:
//! the free lines and bands still sample the raw terrain, so a footpath
//! leaving a street does not yet run up the batter. And `height_at` is
//! still the terrain's — nothing downstream reads a ground yet, and the
//! structure step is where that has to change.

use std::collections::BTreeMap;
use std::collections::HashMap;
use std::collections::HashSet;

use crate::poly::{self, Pt, Ring, Shapes};
use crate::step::{Residual, Summary};
use crate::terrain::height_at;
use crate::width::{self, Family};
use crate::world::{connector, Bench, Kind, Mesh, Polyline2, Profile, Profiles, Sheets, Terrain, Tri};

/// How far, in metres, the pavement stands above the carriageway beside
/// it: one kerb face (`data/plans/surface-leaves-the-plane-2026-09-08.md`
/// §5).
pub const KERB_RISE_M: f64 = 0.12;

/// How far past the asphalt's edge, in metres, a carriageway's
/// cross-section carries the pavement level with it. It is the room
/// step's own `WALL_REACH_M`: what that step paves from a kerb — the
/// band, the rungs to a facade, a mapped sidewalk beside them — is what
/// this one lifts with the road, and nothing built there is left behind.
/// It is a *plateau*, so it is not free: on a 30 % flank every metre of
/// it is another metre of bench to cut, and 10 m of it left a wall at
/// its edge where 6 m leaves a face.
pub const ROOM_REACH_M: f64 = 6.0;

/// The slope of an earthwork face, as run over rise: 1 in 2.5
/// (`data/plans/surface-leaves-the-plane-2026-09-08.md` §5). Past the
/// room's reach the walk comes down a face of exactly this slope, so it
/// is as wide as the drop it has to close and no wider. A band of fixed
/// width was tried first and rejected: 6 m of it on the box's 30 % flank
/// came out at 103 %, steeper than the wall it was there to avoid, and
/// the `step` check said so before the eye did.
pub const EARTHWORK_BATTER: f64 = 2.5;

/// How far, in metres, the over-a-span mask is grown past the span's own
/// rim before a vertex is asked whether it stands on one.
///
/// A vertex of the meshed sheet lands exactly on the region's ring, and a
/// crossings test on a ring is ambiguous there. A centimetre settles it and
/// is far under anything the answer could be confused with: the nearest
/// real ground to a deck's edge is the abutment, and there the two are at
/// one height anyway.
pub const OVER_RIM_M: f64 = 0.01;

/// The tallest earthwork face, in metres, before the bench is walled at
/// its edge instead (`data/plans/surface-leaves-the-plane-2026-09-08.md`
/// §5). It does two things here. A walk band past the room's reach that
/// stands more than one face from the road beside it is not that road's
/// pavement and drapes — without that test a face on the box's flank
/// never daylights at all, because the mountain rises faster than 1 in
/// 2.5, and the first run over the loop box read 225 m of cut where the
/// field had carried a road's height half a kilometre up the hillside.
/// And a vertex of the room itself standing further than this from the
/// ground is counted as `walled`: the ground's answer there is a wall,
/// not a batter.
pub const MAX_BENCH_FACE_M: f64 = 3.0;

/// Slack, in metres, on the kerb's own rise before an edge counts as a
/// step. The rise arrives as a sum of floats and lands a hair either
/// side of itself: without the slack, a flat ground reported two steps,
/// and both were an edge between a lifted pavement and the free band
/// beside it — the kerb, exactly, and no more.
pub const STEP_SLACK_M: f64 = 1e-6;

/// A mesh edge whose ends differ in height by more than [`KERB_RISE_M`]
/// *and* by more than this many metres per metre of its own length is a
/// step: the field is discontinuous across it. Both conditions are needed
/// and neither alone would do. A street on the box's flank climbs 30 %, so
/// a plain gradient test would call every one of its edges a step; a kerb
/// return a centimetre across can drop a metre, so a plain height test
/// would miss nothing but would also flag a long edge on a steep street.
/// One metre per metre is steeper than any surface this world drives on
/// and shallower than any wall it builds.
pub const STEP_GRADE: f64 = 1.0;

/// Cell size of the nearest-segment indices, in metres. A station is at
/// most `NODE_M` (4 m) from the next, so a cell holds a few segments of
/// each axis crossing it.
const CELL_M: f64 = 16.0;

/// How far, in metres, a point may be from a carriageway axis and still
/// be asked about: the widest half-width the priors carry, plus the
/// room's reach, plus the run of a face. Past it every rule in this step
/// answers "the ground", so the query stops rather than searching the
/// whole world to find out.
const FIELD_LIMIT_M: f64 = 4.5 + ROOM_REACH_M + EARTHWORK_BATTER * MAX_BENCH_FACE_M;

/// How much farther than the nearest axis, in metres, another leg of the
/// same junction may run from a point and still have a say in its height:
/// the width of the band over which two legs' cross-sections are blended
/// either side of the line where they are equidistant. A disagreement of
/// `Δ` across the line is spread over it, so it adds about `Δ / BLEND_M`
/// of grade there.
pub const BLEND_M: f64 = 4.0;

/// Within this many metres of a connector two or more axes share, the legs
/// meeting there are blended in full: a street's half-width and the room's
/// reach, out along the bisector of a square corner, with a margin.
pub const JOINT_M: f64 = 15.0;

/// Past this many metres from the connector the blend has faded into the
/// nearest axis's own cross-section, and a leg is its own road again.
pub const JOINT_FADE_M: f64 = 25.0;

/// The room's height field: the solved profile of every carriageway axis
/// on the ground, indexed for the nearest-axis query every vertex makes.
///
/// The nearest axis is the road whose cross-section the point rides. Inside
/// a piece's own ribbon that is the piece's own axis, since no other axis
/// comes within its half-width without their ribbons overlapping; in the
/// pavement and the kerb returns it is the nearest road, which is the one
/// the pavement belongs to — except near a junction, where the legs that
/// meet there are blended ([`Field::at`]).
#[derive(Debug, Default)]
pub struct Field {
    at: Nearest,
    /// Per segment, the solved heights of its two ends and the half-width
    /// of the road it belongs to.
    seg: Vec<(f64, f64, f64)>,
    /// Per segment, the axis it belongs to: the profile's place among the
    /// ones the field was built from.
    axis: Vec<u32>,
    /// Every connector two or more axes share, with the axes meeting
    /// there, in connector order so a blend sums in an order that is a
    /// function of the world.
    joints: Vec<(Pt, Vec<u32>)>,
    /// The joints on a grid of [`JOINT_FADE_M`] cells.
    joint_cells: HashMap<(i32, i32), Vec<u32>>,
}

/// Segments on a grid of [`CELL_M`] cells, for many nearest-segment
/// queries from a point. Both the room's height field and the engineered
/// ground are a nearest-something-and-interpolate, and this is the
/// something: the caller keeps whatever it hangs off each segment.
///
/// Cells are searched in rings about the query's own, and the search
/// stops when the nearest segment found is closer than the ring's own
/// distance: everything outside a ring of `k` cells is at least `k` cells
/// away, so nothing nearer can be left unlooked at.
#[derive(Debug, Default)]
pub struct Nearest {
    seg: Vec<(Pt, Pt)>,
    cells: HashMap<(i32, i32), Vec<u32>>,
    /// The occupied cells' bounds, so a query knows when it has searched
    /// everything there is.
    span: Option<(i32, i32, i32, i32)>,
}

impl Nearest {
    fn push(&mut self, a: Pt, b: Pt) {
        let i = self.seg.len() as u32;
        self.seg.push((a, b));
        let box_ = [a[0].min(b[0]), a[1].min(b[1]), a[0].max(b[0]), a[1].max(b[1])];
        for cell in poly::cells_over(box_, CELL_M) {
            self.cells.entry(cell).or_default().push(i);
            self.span = Some(match self.span {
                None => (cell.0, cell.1, cell.0, cell.1),
                Some((c0, r0, c1, r1)) => (c0.min(cell.0), r0.min(cell.1), c1.max(cell.0), r1.max(cell.1)),
            });
        }
    }

    /// The nearest segment to `p` within `limit` metres: its index, how
    /// far along it the foot lies, and how far `p` is from it.
    ///
    /// The limit is what makes the query cheap where it matters least. A
    /// lattice vertex half a kilometre from the nearest road would
    /// otherwise expand its search until it had scanned the whole index
    /// to learn that the road is far away, and there are two million of
    /// them; with a limit the search stops at the first ring that cannot
    /// hold an answer.
    fn of(&self, p: Pt, limit: f64) -> Option<(usize, f64, f64)> {
        self.of_where(p, limit, |_| true)
    }

    /// The same, among the segments `keep` accepts.
    fn of_where(&self, p: Pt, limit: f64, keep: impl Fn(usize) -> bool) -> Option<(usize, f64, f64)> {
        let (c0, r0) = poly::cell_of(p, CELL_M);
        let (bc0, br0, bc1, br1) = self.span?;
        let rings = (c0 - bc0).abs().max((bc1 - c0).abs()).max((r0 - br0).abs()).max((br1 - r0).abs());
        let mut best: Option<(usize, f64, f64)> = None;
        for k in 0..=rings {
            for (c, r) in ring(c0, r0, k) {
                let Some(ids) = self.cells.get(&(c, r)) else {
                    continue;
                };
                for &i in ids.iter().filter(|&&i| keep(i as usize)) {
                    let (a, b) = self.seg[i as usize];
                    let f = poly::nearest_on_segment(a, b, p);
                    let d = (p[0] - f[0]).hypot(p[1] - f[1]);
                    if best.is_some_and(|(_, _, bd)| d >= bd) {
                        continue;
                    }
                    let len2 = (b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2);
                    let t = if len2 > 0.0 {
                        ((f[0] - a[0]) * (b[0] - a[0]) + (f[1] - a[1]) * (b[1] - a[1])) / len2
                    } else {
                        0.0
                    };
                    best = Some((i as usize, t, d));
                }
            }
            let reached = k as f64 * CELL_M;
            if best.is_some_and(|(_, _, d)| d <= reached) || reached > limit {
                break;
            }
        }
        best.filter(|(_, _, d)| *d <= limit)
    }
}

/// What the nearest axis says about a point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Foot {
    /// The road surface's height at the perpendicular foot.
    pub h: f64,
    /// How far the axis runs from the point, in metres.
    pub d: f64,
    /// Half the width of that road: where its asphalt ends.
    pub half_w: f64,
}

impl Foot {
    /// The height of the walk at this foot: `room_h` — the road's height
    /// plus the kerb — while the point is within the room's reach, and
    /// past it a face at [`EARTHWORK_BATTER`] descending to `ground` and
    /// stopping exactly where it meets it.
    ///
    /// The face is a function of the point alone, which is what makes it
    /// cheap: at `d` metres out it may have closed `(d − reach) / 2.5` of
    /// the difference and no more, so it is the ground wherever it has
    /// daylighted and the room's height wherever it has not left the reach.
    ///
    /// **It is not continuous, and the drape branch is where it breaks.**
    /// At `out = 0⁻` this returns `room_h`; at `out = 0⁺` with a drop past
    /// [`MAX_BENCH_FACE_M`] it returns `ground`, so the surface jumps by the
    /// whole drop — 15.641 m on the loop box — along the locus
    /// `d = half_w + ROOM_REACH_M`. The stretched pavement triangles sit
    /// exactly there: p25/p50/p75 of their distance from the carriageway is
    /// 5.67/6.11/6.75 m with 76 % inside 5–8 m, against a p50 of 4.13 m for
    /// the pavement at large. (A second, smaller one of the same kind sits
    /// at [`FIELD_LIMIT_M`], where `Field::at` goes `None` and the vertex
    /// drapes: 8 % of them.)
    ///
    /// The step is not itself the defect — a walk at road height by the
    /// kerb, with the ground fifteen metres down six metres out, *is* a
    /// retaining wall, and `a_footway_leaving_the_room_leaves_it_over_one_face`
    /// says so. The defect is that nothing draws it: every face this step
    /// builds comes off a *rim* (`wall` off the room's outline, `kerb` and
    /// `rail_face` off a mesh's boundary edges), and **1 599 of the box's
    /// 1 801 stretched triangles are fully interior**. A narrow band puts
    /// the step on its own edge, which is a rim — which is why the specimen
    /// corpus is green and only a band wide enough to be probed out to a
    /// facade shows it.
    ///
    /// Two ways out are already ruled out by measurement. Deciding per run
    /// instead of per point is guarded against three lines into that same
    /// test — it dragged the far end of the stub footway 5.88 m into the
    /// air, because one band is legitimately pavement at the kerb and not
    /// pavement twenty metres up the hill. And cutting the band at the
    /// reach so the step falls on a rim does nothing on its own: this
    /// function is positional, so a vertex on the cut and its twin on the
    /// far sheet share `d` and come out at the *same height*, leaving
    /// nothing between them to close (`wall_m2` unmoved at 12 036, interior
    /// stretched triangles 1 599 → 1 856, `unmet` 1.66 % → 6.96 % for the
    /// extra boolean). The cut only pays if each sheet is lifted by its own
    /// rule, which means carrying the split through `Mesh` and `Bench`.
    pub fn batter(&self, room_h: f64, ground: f64) -> f64 {
        let out = self.d - self.half_w - ROOM_REACH_M;
        if out <= 0.0 {
            return room_h;
        }
        // Past the reach the walk joins the road's bench only where a
        // face could reach it. A band standing more than one face from
        // the road is not that road's pavement at all — on the box's
        // flank a footway 20 m out and 21 m below a switchback was
        // otherwise hauled 18 m into the air by it — and it samples the
        // ground, whatever the road above it is doing.
        let drop = ground - room_h;
        if drop.abs() > MAX_BENCH_FACE_M {
            return ground;
        }
        let slack = out / EARTHWORK_BATTER;
        room_h + drop.clamp(-slack, slack)
    }

    /// [`Foot::batter`] as the *limit from outside the reach*: the same
    /// face, with no plateau however small `out` is.
    ///
    /// This is the far sheet's rule, and the difference between the two is
    /// the whole point of meshing the walk in two parts. `batter` returns
    /// `room_h` at `out = 0` while its limit from the right is `ground`
    /// wherever the drop is past [`MAX_BENCH_FACE_M`] — that gap is the
    /// step, and asked positionally both sheets sit exactly at `out = 0`
    /// on the cut, read `room_h`, and agree. Then there is nothing for a
    /// face to close and the step reappears one lattice vertex further
    /// out, inside the far sheet, which is where it already was.
    ///
    /// Deciding by sheet instead puts the step on the cut by construction,
    /// wherever the bench's own `d = half_w + ROOM_REACH_M` locus falls
    /// against the room's polygon-based band. What is left inside the far
    /// sheet is the `|drop| = MAX_BENCH_FACE_M` crossing, which is bounded
    /// by one face rather than by the drop.
    pub fn face(&self, room_h: f64, ground: f64) -> f64 {
        let drop = ground - room_h;
        if drop.abs() > MAX_BENCH_FACE_M {
            return ground;
        }
        let slack = (self.d - self.half_w - ROOM_REACH_M).max(0.0) / EARTHWORK_BATTER;
        room_h + drop.clamp(-slack, slack)
    }
}

impl Field {
    /// The field of `profiles`: every station the source did **not** map as
    /// a bridge or a bore, in order. A way is one profile now, so the filter
    /// is per station rather than per piece — the at-grade stretches of a way
    /// that also carries a deck are ground, and its deck is the structure
    /// step's.
    pub fn new(profiles: &[Profile]) -> Field {
        Field::grounded(profiles.iter())
    }

    /// The same over whichever profiles are given: the roads' field and the
    /// railways' are two, so a pavement beside a railway never rides it.
    pub fn grounded<'a>(profiles: impl Iterator<Item = &'a Profile>) -> Field {
        Field::of_stations(profiles.map(|p| {
            let runs = p.runs();
            let keep: Vec<(usize, usize)> =
                runs.iter().filter(|r| !r.2.is_structure()).map(|r| (r.0, r.1)).collect();
            (p, keep)
        }))
    }

    /// The field of whichever profiles are given. The bench reads the
    /// ground pieces; the structure step reads the spans, to ask whether
    /// a footway is carried on a road's deck rather than on one of its
    /// own.
    pub fn of<'a>(profiles: impl Iterator<Item = &'a Profile>) -> Field {
        Field::of_stations(profiles.map(|p| {
            let last = p.stations.len().saturating_sub(1);
            (p, vec![(0usize, last)])
        }))
    }

    /// The same, over named station ranges of each profile rather than all
    /// of it: a way is one profile now, and a caller usually wants one kind
    /// of its runs.
    pub fn of_stations<'a>(
        profiles: impl Iterator<Item = (&'a Profile, Vec<(usize, usize)>)>,
    ) -> Field {
        let mut f = Field::default();
        // A station keeps its way's own vertices, so wherever ways meet
        // every one of them has a station there: a joint is a connector
        // the stations of two axes share.
        let mut meets: HashMap<(i64, i64), (Pt, Vec<u32>)> = HashMap::new();
        for (axis, (p, ranges)) in profiles.enumerate() {
            let axis = axis as u32;
            let half_w = p.width_m / 2.0;
            for (k0, k1) in ranges {
                if p.stations.is_empty() {
                    continue;
                }
                let (k0, k1) = (k0.min(p.stations.len() - 1), k1.min(p.stations.len() - 1));
                for st in &p.stations[k0..=k1] {
                    let m = meets.entry(connector(st.p)).or_insert((st.p, Vec::new()));
                    if !m.1.contains(&axis) {
                        m.1.push(axis);
                    }
                }
                if k1 == k0 {
                    let st = p.stations[k0];
                    f.push(st.p, st.p, st.h, st.h, half_w, axis);
                    continue;
                }
                for w in p.stations[k0..=k1].windows(2) {
                    f.push(w[0].p, w[1].p, w[0].h, w[1].h, half_w, axis);
                }
            }
        }
        let mut joints: Vec<_> = meets.into_iter().filter(|(_, (_, m))| m.len() > 1).collect();
        joints.sort_unstable_by_key(|(key, _)| *key);
        for (_, (c, members)) in joints {
            f.joint_cells.entry(poly::cell_of(c, JOINT_FADE_M)).or_default().push(f.joints.len() as u32);
            f.joints.push((c, members));
        }
        f
    }

    fn push(&mut self, a: Pt, b: Pt, ha: f64, hb: f64, half_w: f64, axis: u32) {
        self.at.push(a, b);
        self.seg.push((ha, hb, half_w));
        self.axis.push(axis);
    }

    /// How many axis pieces the field holds.
    pub fn len(&self) -> usize {
        self.seg.len()
    }

    pub fn is_empty(&self) -> bool {
        self.seg.is_empty()
    }

    /// What the nearest carriageway axis says about `p`, its height blended
    /// with the other legs of any junction that axis meets near `p`. `None`
    /// if the field is empty.
    pub fn at(&self, p: Pt) -> Option<Foot> {
        let (i, t, d) = self.at.of(p, FIELD_LIMIT_M)?;
        let near = self.axis[i];
        Some(Foot { h: self.joined(p, near, self.height(i, t), d), d, half_w: self.seg[i].2 })
    }

    /// The solved height `t` of the way along segment `i`.
    fn height(&self, i: usize, t: f64) -> f64 {
        let (ha, hb, _) = self.seg[i];
        ha + (hb - ha) * t
    }

    /// `h`, the height the nearest axis `near` gives `p` from `d` metres
    /// away, blended with the other legs of every joint of `near` within
    /// [`JOINT_FADE_M`] of `p`.
    ///
    /// Within one joint each leg weighs by how nearly it is the nearest —
    /// in full at the line where two legs are equidistant, nothing once it
    /// runs [`BLEND_M`] farther than the nearest — so the blend is
    /// symmetric in the legs and continuous across that line, where the
    /// nearest axis alone stepped. The joints then weigh by their distance
    /// from `p`, in full within [`JOINT_M`] and fading to nothing at
    /// [`JOINT_FADE_M`], and whatever weight they leave is the nearest
    /// axis's own. An axis no joint near `p` holds is never asked: two
    /// roads that meet nowhere near keep the step between them.
    fn joined(&self, p: Pt, near: u32, h: f64, d: f64) -> f64 {
        let (c0, r0) = poly::cell_of(p, JOINT_FADE_M);
        let (mut sum, mut weight, mut most) = (0.0, 0.0, 0.0f64);
        for (c, r) in (-1..=1).flat_map(|dc| (-1..=1).map(move |dr| (c0 + dc, r0 + dr))) {
            for &j in self.joint_cells.get(&(c, r)).into_iter().flatten() {
                let (at, members) = &self.joints[j as usize];
                let reach = fade(((p[0] - at[0]).hypot(p[1] - at[1]) - JOINT_M) / (JOINT_FADE_M - JOINT_M));
                if reach <= 0.0 || !members.contains(&near) {
                    continue;
                }
                let (mut hs, mut ws) = (0.0, 0.0);
                for &m in members {
                    let (hm, dm) = if m == near {
                        (h, d)
                    } else {
                        match self.at.of_where(p, d + BLEND_M, |k| self.axis[k] == m) {
                            Some((k, t, dm)) => (self.height(k, t), dm),
                            None => continue,
                        }
                    };
                    let w = fade((dm - d) / BLEND_M);
                    hs += w * hm;
                    ws += w;
                }
                sum += reach * hs / ws;
                weight += reach;
                most = most.max(reach);
            }
        }
        (sum + (1.0 - most) * h) / (weight + (1.0 - most))
    }
}

/// A weight falling smoothly from 1 at `t ≤ 0` to 0 at `t ≥ 1`, with no
/// slope at either end, so a blend faded by it has no crease where it
/// starts or stops.
fn fade(t: f64) -> f64 {
    let t = t.clamp(0.0, 1.0);
    1.0 - t * t * (3.0 - 2.0 * t)
}

/// The cells exactly `k` away from `(c0, r0)` in the Chebyshev metric, in
/// a fixed order: the query's answer must not depend on a hash order.
fn ring(c0: i32, r0: i32, k: i32) -> Vec<(i32, i32)> {
    if k == 0 {
        return vec![(c0, r0)];
    }
    let mut out = Vec::with_capacity(8 * k as usize);
    for c in c0 - k..=c0 + k {
        for r in r0 - k..=r0 + k {
            if c == c0 - k || c == c0 + k || r == r0 - k || r == r0 + k {
                out.push((c, r));
            }
        }
    }
    out
}

/// The engineered ground: the terrain the world stands on once the room
/// has been cut into it.
///
/// It is the natural ground everywhere except within reach of the room,
/// where it is the room's own height at the outline and a face at
/// [`EARTHWORK_BATTER`] out of it, cut on the uphill side and filled on
/// the downhill, each face stopping exactly where it meets the natural
/// ground. Two things bound it, and both are [`MAX_BENCH_FACE_M`]:
///
/// - **The wall at the edge.** Where the room stands more than one face
///   from the ground at its own outline, no batter is built: the bench is
///   walled there (a vertical face — closure, invariant 9) and the ground
///   beyond it is the natural ground. The profile's deviation box should
///   have kept it inside that, and `walled` says whether it did.
/// - **The run.** A face is at most one face tall, so it runs at most
///   `EARTHWORK_BATTER · MAX_BENCH_FACE_M` — 7.5 m — and past that the
///   ground is natural again. Without the cap a face into a hillside
///   steeper than 1 in 2.5 never daylights at all.
///
/// It is a function of the point, not a mesh, so anything may be *proven*
/// to lie on it, which is the property the terrain step set out with and
/// the one the crate is built on.
#[derive(Debug, Default)]
pub struct Ground {
    at: Nearest,
    seg: Vec<Seg>,
}

/// One outline segment: the room's height **sampled where the segment
/// crosses the lattice** — the vertices both meshes put there themselves —
/// and the natural ground at its two ends.
///
/// The samples are what makes the seam exact. A kerb may run fifty metres
/// between two vertices of its ring while the profile under it does not
/// run straight at all, so a height interpolated between the ring's own
/// two ends is not the edge either mesh drew: that was a crack along 74 %
/// of the loop box's outline, up to 9.6 m of it. Sampling at the crossings
/// and interpolating between *them* is the mesh's own edge, and it costs
/// one binary search rather than four times the index.
#[derive(Debug, Default)]
struct Seg {
    /// One sample per lattice crossing, `[t, room, natural]`, ascending in
    /// `t` from 0 to 1. Interleaved in one allocation because
    /// [`Ground::at`] is asked seven million times over the loop box and
    /// every one of them lands on a sample and its neighbour.
    s: Vec<[f64; 3]>,
    /// The segment lies across a tunnel's mouth: no batter runs off it.
    mouth: bool,
}

/// How far, in metres, a point of the room's outline may stand off a
/// portal's cap and still be on it: the polygon kernel's rounding and the
/// booleans the cap has been through, far short of anything built.
const MOUTH_EPS_M: f64 = 0.05;

/// Where a ground piece ends against a tunnel: its square cap across the
/// road, which the tube's mouth stands in. The cap is the room's outline
/// there, and it must not be closed as if it were a cutting's end.
#[derive(Debug, Clone, Copy)]
pub struct Mouth {
    at: Pt,
    /// The unit direction into the tunnel.
    into: Pt,
    half: f64,
    /// How high the tube is inside.
    tube: f64,
}

/// Every mouth of the world's tunnel spans: both ends of each.
pub fn mouths(spans: &[Polyline2]) -> Vec<Mouth> {
    spans
        .iter()
        .filter(|w| matches!(w.kind, Kind::Tunnel(_)) && w.pts.len() >= 2)
        .flat_map(|w| {
            let n = w.pts.len();
            let (half, tube) = (w.width_m / 2.0, crate::structure::tube_m(&w.class));
            [(w.pts[0], w.pts[1]), (w.pts[n - 1], w.pts[n - 2])].map(|(at, q)| Mouth {
                at,
                into: poly::unit([q[0] - at[0], q[1] - at[1]]),
                half,
                tube,
            })
        })
        .collect()
}

/// Everywhere the ground must leave a tube room: the mouths of the opened
/// portals, and the galleries ([`crate::structure::is_gallery`]), whose
/// footprints are cut out of the ground whole.
#[derive(Debug, Default)]
pub struct Portals {
    mouths: Vec<Mouth>,
    /// The galleries' axes, and per segment the roadway's height at its two
    /// ends, the half-width of the tube and its height.
    gallery: Nearest,
    crowns: Vec<(f64, f64, f64, f64)>,
}

impl Portals {
    /// The portals of `spans` and the galleries of `profiles`, and the
    /// galleries' footprints — what the ground is to be opened over.
    pub fn new(spans: &[Polyline2], profiles: &Profiles) -> (Portals, Shapes) {
        let mut out = Portals { mouths: mouths(spans), ..Portals::default() };
        let mut footprints: Shapes = Vec::new();
        for p in &profiles.profiles {
            let (half, tube) = (crate::structure::half_width_m(&p.class, p.width_m), crate::structure::tube_m(&p.class));
            for (a, b) in crate::structure::gallery_runs(p) {
                let st = &p.stations[a..=b];
                for w in st.windows(2) {
                    out.gallery.push(w[0].p, w[1].p);
                    out.crowns.push((w[0].h, w[1].h, half, tube));
                }
                let axis: Vec<Pt> = st.iter().map(|x| x.p).collect();
                footprints.extend(poly::buffer_line_capped(&axis, 2.0 * half, [false, false]));
            }
        }
        (out, poly::union_all(&footprints))
    }

    /// The tube's section at `q`, as `(floor, roof)`, if `q` stands where a
    /// tube opens: across a mouth, the room's own height there (`room_h`)
    /// and a tube over it; on a gallery's footprint, its roadway and a tube
    /// over that.
    fn section(&self, q: Pt, room_h: f64) -> Option<(f64, f64)> {
        if let Some(tube) = on_mouth(&self.mouths, q) {
            return Some((room_h, room_h + tube));
        }
        let (i, t, d) = self.gallery.of(q, GALLERY_LIMIT_M)?;
        let (ha, hb, half, tube) = self.crowns[i];
        let floor = ha + (hb - ha) * t;
        (d <= half + MOUTH_EPS_M).then_some((floor, floor + tube))
    }

    /// Whether `q` stands where a tube opens.
    fn open(&self, q: Pt) -> bool {
        self.section(q, 0.0).is_some()
    }
}

/// How far from a gallery's axis, in metres, a point is asked about at all:
/// past the widest structure's half-width nothing is a gallery's edge.
const GALLERY_LIMIT_M: f64 = 8.0;

/// The tube height of the mouth `q` lies across, if it lies across one.
fn on_mouth(mouths: &[Mouth], q: Pt) -> Option<f64> {
    mouths
        .iter()
        .find(|m| {
            let d = [q[0] - m.at[0], q[1] - m.at[1]];
            let along = d[0] * m.into[0] + d[1] * m.into[1];
            let across = d[0] * m.into[1] - d[1] * m.into[0];
            along.abs() <= MOUTH_EPS_M && across.abs() <= m.half + MOUTH_EPS_M
        })
        .map(|m| m.tube)
}

impl Seg {
    /// The room's height and the natural ground at parameter `t`.
    ///
    /// **Both** are sampled, and they have to be: the batter is refused
    /// where the two differ by more than one face, so a decision taken on
    /// an interpolated ground and reported against the true one disagrees
    /// by up to exactly [`MAX_BENCH_FACE_M`], which is what `contact` read
    /// when only the room was sampled.
    fn at(&self, t: f64) -> (f64, f64) {
        let i = self.s.partition_point(|x| x[0] < t).clamp(1, self.s.len() - 1);
        let (a, b) = (self.s[i - 1], self.s[i]);
        let u = if b[0] > a[0] { ((t - a[0]) / (b[0] - a[0])).clamp(0.0, 1.0) } else { 0.0 };
        (a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u)
    }
}

impl Ground {
    /// The ground benched to `outline`, whose vertices stand at `room`
    /// over a natural ground of `natural`, sampled along `grid`.
    pub fn new(
        outline: &Shapes,
        grid: &crate::grid::Grid,
        room: &dyn Fn(Pt) -> f64,
        natural: &dyn Fn(Pt) -> f64,
        portals: &Portals,
    ) -> Ground {
        let mut g = Ground::default();
        for ring in outline.iter().flatten() {
            for i in 0..ring.len() {
                let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
                g.at.push(a, b);
                let len = (b[0] - a[0]).hypot(b[1] - a[1]);
                let mouth = portals.open(a) && portals.open(b);
                let mut seg = Seg { s: vec![[0.0, room(a), natural(a)]], mouth };
                // `split` ends with `b` itself, so the last sample is at 1.
                for q in crate::drape::split(grid, a, b) {
                    let u = if len > 0.0 { ((q[0] - a[0]).hypot(q[1] - a[1]) / len).clamp(0.0, 1.0) } else { 1.0 };
                    if u > seg.s.last().expect("seeded with 0")[0] {
                        seg.s.push([u, room(q), natural(q)]);
                    }
                }
                if seg.s.len() == 1 {
                    seg.s.push([1.0, room(b), natural(b)]);
                }
                g.seg.push(seg);
            }
        }
        g
    }

    /// How many outline pieces the ground is benched to.
    pub fn len(&self) -> usize {
        self.seg.len()
    }

    pub fn is_empty(&self) -> bool {
        self.seg.is_empty()
    }

    /// The engineered height at `p`, whose natural ground is `natural`.
    ///
    /// Outside the room only: a point inside it is under the room's own
    /// surface, which is what the room's mesh draws and what the terrain
    /// has a hole for.
    pub fn at(&self, p: Pt, natural: f64) -> f64 {
        let Some((i, t, d)) = self.at.of(p, EARTHWORK_BATTER * MAX_BENCH_FACE_M) else {
            return natural;
        };
        // **No batter runs into a tunnel's mouth.** Off a cap across a
        // cutting's end a face would climb from the road into the hill — up
        // the inside of the tube, across the opening — and shut the portal
        // this cutting was opened for. The hill is the hill there, and the
        // wall over the mouth closes it down to the tube's roof.
        if self.seg[i].mouth {
            return natural;
        }
        let (room, edge) = self.seg[i].at(t);
        if (edge - room).abs() > MAX_BENCH_FACE_M {
            return natural;
        }
        let slack = d / EARTHWORK_BATTER;
        room + (natural - room).clamp(-slack, slack)
    }
}

/// What the lift did to one family.
#[derive(Debug, Default, Clone)]
pub struct Stats {
    pub vertices: usize,
    /// Walk vertices whose nearest road stands more than one bench face
    /// above the natural ground under the vertex itself: a pavement asked
    /// to follow a road it is not on.
    pub flown: usize,
    /// Vertices that took the road's height whole.
    pub lifted: usize,
    /// Vertices on a batter face between the room's reach and the ground.
    pub battered: usize,
    /// Vertices left where the mesh step put them: the free bands.
    pub draped: usize,
    /// Of those, the ones **no road answered for at all** — nothing within
    /// [`FIELD_LIMIT_M`] — as against the ones whose batter simply
    /// daylighted, which is the face doing its job. A free vertex keeps the
    /// raw DEM, and the ground around it has been benched.
    pub free: usize,
    /// Vertices standing more than one face ([`MAX_BENCH_FACE_M`]) off
    /// the ground: where the ground's answer must be a wall rather than
    /// a batter.
    pub walled: usize,
    /// The deepest a vertex was let into the ground, in metres.
    pub cut: f64,
    /// The highest a vertex was raised above it.
    pub fill: f64,
    pub edges: usize,
    /// Edges the field steps across.
    pub steps: usize,
    /// The largest of those steps, in metres.
    pub worst: f64,
    /// Where they are.
    pub at: Vec<[f64; 2]>,
}

impl Stats {
    fn merge(&mut self, o: &Stats) {
        self.vertices += o.vertices;
        self.flown += o.flown;
        self.free += o.free;
        self.lifted += o.lifted;
        self.battered += o.battered;
        self.draped += o.draped;
        self.walled += o.walled;
        self.cut = self.cut.max(o.cut);
        self.fill = self.fill.max(o.fill);
        self.edges += o.edges;
        self.steps += o.steps;
        self.worst = self.worst.max(o.worst);
        self.at.extend_from_slice(&o.at);
    }
}

/// Lifts the world's room onto the profile, and cuts it into the ground.
/// The **drawn ground** against the raw DEM: [`crate::step::Residual`] over
/// every vertex of the terrain this step re-meshes.
///
/// This is where the run's residual chain ends, and it is the only link in it
/// measured over an area rather than along an axis. The steps before it report
/// the same quantity over their at-grade stations
/// ([`crate::crossing::residual_of`]), so the two are not the same population
/// and are not meant to be: an axis says how far the *road* has left the DEM,
/// and this says how far the *ground* has, which is the earthwork. A run where
/// the second is much the larger is a run whose earthwork is not the road's —
/// which is the question `data/plans/one-ground-2026-09-16.md` §1.1 could not
/// answer by sweeping constants.
fn drawn_residual(terrain: &Terrain, ground: &Tri) -> Residual {
    let mut r = Residual::new();
    for v in &ground.positions {
        r.push(v[2], height_at(terrain, v[0], v[1]));
    }
    r
}

pub fn run(
    terrain: &Terrain,
    profiles: &Profiles,
    mesh: &Mesh,
    sheets: &Sheets,
    spans: &[Polyline2],
    arrangement: &crate::arrangement::Arrangement,
) -> (Bench, Summary) {
    // The tunnels' openings: the mouths of the portals the partition cut
    // open, and the galleries, whose whole footprint the ground leaves to
    // the tube standing in it.
    // The galleries are the arrangement's now — it cuts a face for each —
    // and what is still wanted here are the portals' mouths.
    let (portals, _) = Portals::new(spans, profiles);
    let (bench, stats, axes, earth, reached, reach_m, carried_m2) = {
        // Two fields: the roads' and the railways'. A pavement is a road's
        // cross-section and never a railway's, and the ballast rides its
        // own track and nothing else — at a level crossing the asphalt is
        // the road's and the bed either side of it the railway's, and the
        // profile has already pinned the two to one height where they meet.
        // Where the paving is over a span rather than on the ground,
        // grown by [`OVER_RIM_M`] so that a vertex *on* the span's own rim
        // reads as over it. The rim is where the test matters most — a
        // deck's free edge is the highest thing on it, and left out of the
        // mask it put the whole standoff back into `fill`.
        // **And the walk a deck carries is over a span too.** The sheets
        // know which *asphalt* stands on one; the walk has no sheets, so a
        // sidewalk along a bridge — one the mapper never tagged as a bridge
        // — was over-a-span to nobody. `structure::carried` is this rule one
        // level up and cannot reach it: it asks whether a walk **span** runs
        // along a road's deck, and this walk has no span to ask about. In
        // plan it is 1–2 m from the deck's edge, inside [`ROOM_REACH_M`],
        // which is the reach that step uses and the width of a road's own
        // cross-section.
        //
        // Folded in here rather than subtracted later so that every
        // consumer agrees: the hole (invariant I3), the earthwork that must
        // not read a deck's standoff as fill, and `unmet`, which must not
        // look for ground under something in the air. It is the
        // arrangement's own mask ([`crate::arrangement::over_spans`]), the
        // one its faces were tagged by, so the hole and the lift agree.
        let carried_m2 = arrangement.carried_m2;
        let over = poly::Indexed::new(&arrangement.over);
        let rail = |p: &&Profile| width::family(&p.class) == Family::Rail;
        // The pavement is not partitioned — a footbridge has no profile, so
        // the walk has no sheets — and takes the roads' field whole, as it
        // always has. The railways' whole field goes with it, because the
        // ground's own fallback below asks which of the two is nearer to a
        // point that is in no mesh at all, and a question about the room
        // near a place is not a question about a vertex's sheet.
        let field = Field::grounded(profiles.profiles.iter().filter(|p| !rail(p)));
        let rails = Field::grounded(profiles.profiles.iter().filter(rail));
        // The two families that do have sheets take one field each, built
        // from that sheet's own axes and nothing else.
        let (car, car_over) = fields(sheets, Family::Carriageway, profiles);
        let (beds, beds_over) = fields(sheets, Family::Rail, profiles);
        // The asphalt is the road: it takes the whole of its own height
        // wherever it reaches. Only the walk beside it is asked how far
        // out it lies.
        let (c, cs) = by_sheet(&mesh.carriageway, &car, &car_over, &mesh.carriageway_sheet, Some(&over), 0.0, false, 0);
        let (p, ps) = lift(&mesh.pavement, &field, KERB_RISE_M, true, mesh.walk_split);
        let (b, bs) = by_sheet(&mesh.ballast, &beds, &beds_over, &mesh.ballast_sheet, Some(&over), 0.0, false, 0);
        let mut stats = Stats::default();
        stats.merge(&cs);
        stats.merge(&ps);
        stats.merge(&bs);
        let steps = std::mem::take(&mut stats.at);

        // The ground answers. The outline is the room's own boundary and
        // its heights are read off the room's mesh, vertex for vertex, so
        // the two meet at the seam rather than near it.
        // **The hole follows the sheets, less what is over a span.**
        //
        // Two reasons, and they pull opposite ways. The sheets hold paving
        // `paving` does not — the kerb returns the sheet step adds once a
        // junction's decks have joined it — and cut from `paving` alone
        // that paving stood on ground nobody had cut, its rim met nothing,
        // and `unmet` went 1.4 % -> 4.0 %. And the sheets hold the decks,
        // which must *not* cut: a viaduct flies over ground that is still
        // there (invariant I3), and the soffit is what closes under it.
        // **Invariant I3 for the pavement.** Only ground-level paving cuts
        // the terrain's hole — enforced for the asphalt by the line above
        // and, until now, for the walk not at all: `paving.walk` went in
        // whole, so a sidewalk on a bridge opened a hole eight metres under
        // itself. Measured at the Montreux overbridge: `room.pavement`
        // true, everything else false, the pavement at **404.13** beside a
        // carriageway at 403.65 over a terrain at 395.52 — the sidewalk is
        // on the deck, exactly where it belongs, and the ground beneath it
        // had been cut away.
        // **The hole is the arrangement's, as faces.** It used to be this
        // expression, and the expression is kept in the doc above because it
        // says what the rule is; what it cannot do is share a boundary with
        // the meshes that stop at it, because it is a different region
        // computed from different operands. `arrangement.hole()` is the same
        // region — measured at 0.000 m² of symmetric difference on every
        // specimen before the switch — built from the same split points as
        // the paving beside it.
        // **Unioned, because this one is a boundary and not a mesh.** The
        // faces are what the meshes are built from and they must stay
        // separate; `outline` is only walked for its edges — `dense` samples
        // along it and `Ground::new` reads the room's height there — and the
        // edge *between* two adjacent paved faces is not boundary at all. Fed
        // in as faces it was walked anyway, and on `house:row` that put 10
        // interior edges into `seam` out of nothing. The union costs nothing
        // here because nothing downstream of it is triangulated.
        let outline = poly::union_all(&arrangement.hole());
        let natural = |p: Pt| height_at(terrain, p[0], p[1]);
        // Before `seam` the binding shadows `seam` the function.
        let seam = seam(&[&c, &p, &b]);
        // `seam` had been reported and never counted: the closure fell
        // back silently and the line read 0 of however many. It counts now.
        let (asked, missed) = (std::cell::Cell::new(0usize), std::cell::Cell::new(0usize));
        let room = |q: Pt| {
            asked.set(asked.get() + 1);
            match at(&seam, q) {
                Some(h) => h,
                None => {
                    missed.set(missed.get() + 1);
                    // The nearer of the two fields answers: a road's
                    // cross-section with its kerb, or a railway's bed.
                    match (field.at(q), rails.at(q)) {
                        (Some(r), Some(t)) if t.d < r.d => t.batter(t.h, natural(q)),
                        (Some(r), _) => r.batter(r.h + KERB_RISE_M, natural(q)),
                        (None, Some(t)) => t.batter(t.h, natural(q)),
                        (None, None) => natural(q),
                    }
                }
            }
        };
        // Both meshes cut their own boundary edges where the ring crosses
        // the lattice, so those crossings are vertices of both, and between
        // two of them each mesh's edge is a straight line in 3D. The ground
        // samples the room's height *there* and the seam is measured and
        // walled over the same points: read at the ring's own corners
        // instead, the ground interpolated over a kerb that may run fifty
        // metres while the profile under it did not, which was a crack
        // along 74 % of the loop box's outline, up to 9.6 m of it, that
        // `contact` could not see because it was measured at the corners
        // too.
        let edge = dense(&outline, &terrain.grid);
        let ground = Ground::new(&outline, &terrain.grid, &room, &natural, &portals);
        let mut earth = Earth::new(&edge, &ground, &room, &natural, terrain);
        // **One call to the mesher instead of two.** The ground used to be
        // triangulated on its own — `mesh::triangulate(&cut, ...)` over
        // `arrangement.ground()` — and the diagnostic below, further down,
        // built `arrangement.mesh()` again over *every* face just to compare
        // against it. Both go through the same `mesh::tagged`, welding by
        // position over the same lattice, so they were never two
        // *disagreeing* triangulations of the ground — `seam`/`unmet` read
        // exactly the same with this change as without it, measured over the
        // whole loop box. What was real is that the ground was meshed twice
        // a run, once for each purpose, at the cost of the larger of the
        // two: this keeps the one call and gets both from it.
        //
        // **What `seam`/`unmet` actually measure is the gap between this
        // mesh and the room's** (`c`/`p`/`b`, a few lines up), which is built
        // by an entirely different path — `mesh::by_sheet`, one call per
        // sheet, never through `arrangement` at all. Closing that gap for
        // real means the room's own tris coming from `one` too, filtered by
        // material the way `g` is here, with their height from the lift
        // instead of from [`Ground::at`]. Not done in this change: the
        // pavement's near/far split and per-sheet field selection make that
        // a larger, separate piece of work, and this one is worth having on
        // its own — every vertex here still answers [`Ground::at`], the same
        // batter-and-wall formula as before, so nothing about the geometry
        // changes, only where it is computed.
        let one = arrangement.mesh(&terrain.grid, &natural);
        let (g, off, lossy, remap) = {
            // Kept, and compacted: `one` holds every face's vertices in one
            // array, and most of them belong to a paved face that is not
            // wanted here. Filtering the indices alone would leave every
            // paved position sitting unused in `g.positions` — harmless to
            // read, but it is what a glTF's vertex buffer pays for whether a
            // triangle names it or not, and it showed: 1.5 M orphaned
            // positions were 18 MB of the archive. `remap` is old index ->
            // new, built the first time a kept triangle names one.
            let mut remap: Vec<u32> = vec![u32::MAX; one.tri.positions.len()];
            let mut g = Tri::default();
            let mut off = 0.0f64;
            for (tri3, face) in one.tri.indices.chunks_exact(3).zip(&one.of_face) {
                if arrangement.faces[*face as usize].cuts() {
                    continue;
                }
                let mut new_id = |v: u32, g: &mut Tri| -> u32 {
                    let r = &mut remap[v as usize];
                    if *r == u32::MAX {
                        let mut p = one.tri.positions[v as usize];
                        // `one`'s positions start as the natural ground
                        // (`arrangement.mesh` was built with `natural` as
                        // its height), which is exactly [`Ground::at`]'s
                        // second argument.
                        p[2] = ground.at([p[0], p[1]], p[2]);
                        g.positions.push(p);
                        *r = (g.positions.len() - 1) as u32;
                    }
                    *r
                };
                let ids = [new_id(tri3[0], &mut g), new_id(tri3[1], &mut g), new_id(tri3[2], &mut g)];
                g.indices.extend_from_slice(&ids);
                let c = [
                    ids.iter().map(|&v| g.positions[v as usize][0]).sum::<f64>() / 3.0,
                    ids.iter().map(|&v| g.positions[v as usize][1]).sum::<f64>() / 3.0,
                ];
                let plane = ids.iter().map(|&v| g.positions[v as usize][2]).sum::<f64>() / 3.0;
                off = off.max((plane - ground.at(c, natural(c))).abs());
            }
            // The arrangement's own meshing cost, over every face rather
            // than the ground's alone: nothing reported this before, since
            // `arrangement.mesh()` was only ever built for the diagnostic
            // below.
            (g, off, one.stats.failed + one.stats.lossy, remap)
        };

        earth.triangles = g.indices.len() / 3;
        earth.vertices = g.positions.len();
        earth.off = off;
        earth.lossy = lossy;
        // **How much of the free walk the engineered ground would move.**
        // A vertex no road answered for keeps the raw DEM while the ground
        // around it has been benched — the open item in the docs, "a
        // footpath leaving a street does not run up the batter". The
        // engineered ground reaches [`EARTHWORK_BATTER`] × one face (7.5 m)
        // from the room's outline and is the natural ground beyond, so the
        // count is exact: ask it, and see where it answers something else.
        let (mut reached, mut reach_m) = (0usize, 0.0f64);
        for q in p.positions.iter() {
            let (at, nat) = ([q[0], q[1]], natural([q[0], q[1]]));
            if (q[2] - nat).abs() > 1e-9 {
                continue;
            }
            let engineered = ground.at(at, nat);
            if (engineered - nat).abs() > 1e-6 {
                reached += 1;
                reach_m = reach_m.max((engineered - nat).abs());
            }
        }
        let walk_cut = cut_seam(&p, mesh.walk_split);
        let (rim_n, unmet, contact) = meet(&[&c, &p, &b], &g, &walk_cut, &natural, &portals, &over);
        earth.rim = rim_n;
        earth.unmet = unmet;
        earth.contact = contact;
        let (wall, wall_m2) = wall(&edge, &ground, &room, &natural, &portals);
        // **One edge rule for every boundary inside the paving** (§3.3),
        // replacing `kerb`, `rail_face` and `walk_face`. The five maps are
        // the answers a face of each kind gives; the pavement's near and far
        // sheets are two of them, because the lifted mesh holds both and a
        // vertex must never answer for itself.
        let split = mesh.walk_split.min(p.positions.len());
        let (near, far) = (lowest(&p.positions[..split]), lowest(&p.positions[split..]));
        let (car, bed) = (lowest(&c.positions), lowest(&b.positions));
        let at_face = |f: &crate::arrangement::Face, q: Pt| -> Option<f64> {
            use crate::arrangement::Material;
            match f.material {
                Material::Carriageway => at(&car, q),
                Material::Ballast => at(&bed, q),
                Material::Pavement if f.near => at(&near, q),
                Material::Pavement => at(&far, q),
                // **The ground stays `wall`'s**, and what answering here
                // would cost is measured rather than guessed — see the note
                // on `wall`.
                Material::Ground => None,
            }
        };
        let (kerb, kerb_m2, tapered) = edge_faces(arrangement, &at_face);
        earth.kerb_m2 = kerb_m2;
        earth.tapered = tapered;
        // **Where the steps are.** §3.3 says an edge is welded or spanned by
        // a quad, "so `step` has nothing left to count". That holds only if
        // every discontinuity falls on a *face boundary* — and `step` counts
        // triangle edges inside one lifted mesh. This is the share of them
        // that lie on an arrangement edge at all: what the edge rule could
        // ever reach.
        earth.on_edge = {
            const NEAR_M: f64 = 0.05;
            // Each edge filed under every cell its box (grown by the
            // tolerance) touches, so a step asks only the edges of its own.
            let mut index: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
            for (i, e) in arrangement.edges.iter().enumerate() {
                let b = [
                    e.a[0].min(e.b[0]) - NEAR_M,
                    e.a[1].min(e.b[1]) - NEAR_M,
                    e.a[0].max(e.b[0]) + NEAR_M,
                    e.a[1].max(e.b[1]) + NEAR_M,
                ];
                for cell in poly::cells_over(b, poly::CELL_M) {
                    index.entry(cell).or_default().push(i);
                }
            }
            steps
                .iter()
                .filter(|m| {
                    index.get(&poly::cell_of(**m, poly::CELL_M)).into_iter().flatten().any(|&i| {
                        let e = &arrangement.edges[i];
                        poly::segment_distance(e.a, e.b, **m) < NEAR_M
                    })
                })
                .count()
        };
        // **What the relax solve would put here** (plan §3.2), measured
        // against the case-function that is here now. Nothing is replaced:
        // this pins the one mesh's paved vertices at the heights the lift
        // just gave them, relaxes, and reports how far the two fields stand
        // apart over the *ground* vertices — which is the whole of what step
        // 3 changes. `one` is the same mesh `g` was just built from, not a
        // second one: it used to be built twice, once here and once for `g`,
        // which was the last place this step still paid for the two-mesh
        // world it otherwise no longer builds.
        {
            let n = one.tri.positions.len();
            let mut pin = vec![crate::relax::Pin::Free; n];
            for (tri3, face) in one.tri.indices.chunks_exact(3).zip(&one.of_face) {
                if arrangement.faces[*face as usize].material
                    == crate::arrangement::Material::Ground
                {
                    continue;
                }
                for &v in tri3 {
                    let q = one.tri.positions[v as usize];
                    // The paved height the lift settled on, as a residual off
                    // the DEM — which is what `relax` solves in.
                    if let Some(h) = at(&seam, [q[0], q[1]]) {
                        pin[v as usize] = crate::relax::Pin::At(h - q[2]);
                    }
                }
            }
            let mut es: Vec<(u32, u32)> = Vec::new();
            for tri3 in one.tri.indices.chunks_exact(3) {
                for k in 0..3 {
                    let (a, b) = (tri3[k], tri3[(k + 1) % 3]);
                    es.push((a.min(b), a.max(b)));
                }
            }
            es.sort_unstable();
            es.dedup();
            let e = crate::relax::relax(n, &es, &pin, crate::relax::weight());
            let mut against = Residual::new();
            for (v, p) in one.tri.positions.iter().enumerate() {
                if pin[v] != crate::relax::Pin::Free {
                    continue;
                }
                // The engineered ground as it stands, against DEM + residual:
                // `g`'s height where the vertex made it into `g`, which every
                // ground face's did.
                let at = match remap[v] {
                    u32::MAX => ground.at([p[0], p[1]], p[2]),
                    r => g.positions[r as usize][2],
                };
                against.push(at, p[2] + e[v]);
            }
            earth.pinned = pin.iter().filter(|x| **x != crate::relax::Pin::Free).count();
            earth.relaxed = n;
            earth.against = against;
        }
        earth.unseamed = missed.get();
        earth.asked = asked.get();
        earth.wall_m2 = wall_m2;
        let axes = field.len() + rails.len();
        (Bench { carriageway: c, pavement: p, ballast: b, ground: g, wall, kerb, steps }, stats, axes, earth, reached, reach_m, carried_m2)
    };
    let summary = Summary::new()
        .with("axes", axes)
        .with_part("lifted", stats.lifted, stats.vertices)
        .with("battered", stats.battered)
        .with("draped", stats.draped)
        .with("cut", format!("{:.3}", stats.cut))
        .with("fill", format!("{:.3}", stats.fill))
        .with_share("step", stats.steps, stats.edges)
        .with("worst", format!("{:.3}", stats.worst))
        .with("ground", format!("{}/{}", earth.triangles, earth.vertices))
        .with_share("seam", earth.unseamed, earth.asked)
        .with_share("unmet", earth.unmet, earth.rim)
        .with("contact", format!("{:.2}", earth.contact))
        .with_share("step_on_edge", earth.on_edge, stats.steps)
        .with_share("pinned", earth.pinned, earth.relaxed)
        // What step 3 would change: the relax solve's ground against the
        // case-function's, over every free vertex of the one mesh.
        .with_quantiles("relax_vs_cases", earth.against)
        .with_share("walled", earth.walled, earth.outline)
        .with("wall", format!("{:.1}", earth.wall))
        .with_m2("wall_m2", earth.wall_m2)
        .with_m2("kerb_m2", earth.kerb_m2)
        .with("tapered", earth.tapered)
        .with_share("touched", earth.touched, earth.lattice)
        .with("off", format!("{:.1e}", earth.off))
        .with("flown", stats.flown)
        .with_part("free", stats.free, stats.draped)
        .with("regrade", format!("{reached} to {reach_m:.2}"))
        .with_m2("carried", carried_m2)
        .with("lossy", earth.lossy)
        .with_residual(drawn_residual(terrain, &bench.ground));
    (bench, summary)
}

/// Below this height, in metres, a step between the room and the ground
/// beside it is the seam's own rounding and not a wall. The seam reads
/// 2.5e-7 m on the loop box, so a millimetre is four orders clear of it.
const WALL_MIN_M: f64 = 1e-3;


/// The face that closes the step between the room's edge and the ground
/// outside it, and the area of it.
///
/// The two meshes meet exactly wherever a batter could run — the ground
/// takes the room's own height at the outline, and `contact` measures that
/// at 2.5e-7 m. Where the step is more than one face tall the batter is
/// refused ([`Ground::at`] hands back the natural ground rather than
/// manufacture a slope no hillside has), the two meshes part company by up
/// to `wall` metres, and until now nothing spanned the gap: **a hole you
/// could see the world through**, which is what invariant 9 forbids and
/// what `walled` had been counting all along without drawing.
///
/// The face is subdivided at the same lattice crossings the two meshes cut
/// their own edges at ([`drape::split`]), and its two rails are read from
/// the same two functions those meshes were built from, so the closure is
/// exact rather than near: no T-junction, no hairline. A segment whose
/// ends both agree to [`WALL_MIN_M`] is not drawn at all, which is most of
/// them.
///
/// The room's outer rings run counter-clockwise and its holes the other
/// way, so `[top_a, bottom_a, bottom_b, top_b]` faces away from the room
/// in both cases — outward at a kerb, into the courtyard at a hole.
fn wall(
    edge: &Shapes,
    ground: &Ground,
    room: &dyn Fn(Pt) -> f64,
    natural: &dyn Fn(Pt) -> f64,
    portals: &Portals,
) -> (Tri, f64) {
    let mut tri = Tri::default();
    let mut m2 = 0.0;
    // Across a tunnel's mouth, and along a gallery, the face closes the
    // ground onto the **tube's section**, never onto the road across the
    // opening. Where the hill stands over the roof it is the headwall, from
    // the roof up; where the ground falls below the floor — the downhill
    // side of a gallery on a flank — it is the footing, from the ground up
    // to the floor, without which the terrain's edge and the tube's wall
    // stood apart by the fall and the world showed through between them.
    // Where the ground meets the tube's wall between the two, the wall is
    // the closure and nothing is drawn.
    let rail = |q: Pt| {
        let (r, g) = (room(q), ground.at(q, natural(q)));
        match portals.section(q, r) {
            Some((_, roof)) if g > roof => (roof, g),
            Some((floor, _)) if g < floor => (floor, g),
            Some(_) => (g, g),
            None => (r, g),
        }
    };
    for ring in edge.iter().flatten() {
        for i in 0..ring.len() {
            let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
            let run = (b[0] - a[0]).hypot(b[1] - a[1]);
            // **A face of no width is not a face.** The outline this is
            // swept along carries every vertex the polygon kernel and the
            // lattice crossings put on it, and some of them are the same
            // point twice: measured over 25 m of the Montreux cutting, 9 of
            // 314 wall triangles had no area at all and 92 of 628 plan
            // edges were under a centimetre. A zero-area triangle has no
            // normal, so a viewer that computes its own shades the wall
            // from a vector that does not exist.
            //
            // Only the exactly-degenerate go here. Thinning the rest is a
            // *simplification* of the outline, and it has to be measured
            // against `wall_m2` rather than done in passing.
            if run <= f64::EPSILON {
                continue;
            }
            let (p0, p1) = (rail(a), rail(b));
            let (d0, d1) = ((p0.0 - p0.1).abs(), (p1.0 - p1.1).abs());
            if d0 <= WALL_MIN_M && d1 <= WALL_MIN_M {
                continue;
            }
            let base = tri.positions.len() as u32;
            tri.positions.extend_from_slice(&[
                [a[0], a[1], p0.0],
                [a[0], a[1], p0.1],
                [b[0], b[1], p1.1],
                [b[0], b[1], p1.0],
            ]);
            // The quad is two triangles, and where the face tapers to
            // nothing at one end — the rails meeting — one of them is a
            // line. It is left out rather than drawn flat.
            if d0 > f64::EPSILON {
                tri.indices.extend_from_slice(&[base, base + 1, base + 2]);
            }
            if d1 > f64::EPSILON {
                tri.indices.extend_from_slice(&[base, base + 2, base + 3]);
            }
            m2 += (d0 + d1) / 2.0 * run;
        }
    }
    (tri, m2)
}


/// **The edge rule** (`data/plans/one-ground-2026-09-16.md` §3.3): every
/// edge of the arrangement is either *welded* — its two faces answer with
/// one height — or *split*, and then the quad between them is drawn, always.
///
/// One rule in one place, over the subdivision's own edges, replacing three
/// sweeps that each walked one mesh's rim and looked the other side up:
/// `kerb` (asphalt|pavement, which splits by the kerb's rise), `rail_face`
/// (ballast against either neighbour, either way up) and `walk_face` (the
/// pavement against itself where the mesh cut it at the room's reach). Each
/// had to know which mesh to walk and which to look up, and the pavement's
/// own split needed a fourth map to stop a vertex answering for itself.
/// Here neither side is privileged: an edge is a pair of faces, and the
/// faces say what they say.
///
/// Not the retaining wall yet: `wall` still sweeps the room's outline
/// against the *ground*, which is the one boundary whose far side is not a
/// face of the paving.
fn edge_faces(
    arrangement: &crate::arrangement::Arrangement,
    at_face: &dyn Fn(&crate::arrangement::Face, Pt) -> Option<f64>,
) -> (Tri, f64, usize) {
    let mut tri = Tri::default();
    let (mut m2, mut tapered) = (0.0, 0usize);
    for e in &arrangement.edges {
        let Some(right) = e.right else { continue };
        if right == e.left {
            continue;
        }
        let (fa, fb) = (&arrangement.faces[e.left], &arrangement.faces[right]);
        let (aa, ab) = (at_face(fa, e.a), at_face(fa, e.b));
        let (ba, bb) = (at_face(fb, e.a), at_face(fb, e.b));
        // A face with an answer at one end and none at the other tapers to
        // nothing rather than butting against the next: the end of a run,
        // and wherever a mesh did not put a vertex where the arrangement
        // did.
        tapered += ((aa.is_none() != ba.is_none()) || (ab.is_none() != bb.is_none())) as usize;
        let (Some(aa), Some(ab), Some(ba), Some(bb)) = (aa, ab, ba, bb) else { continue };
        if (aa - ba).abs() <= WALL_MIN_M && (ab - bb).abs() <= WALL_MIN_M {
            continue;
        }
        // **A deck's edge over the ground is not a kerb.** The partition is
        // flat, so the rim of a deck and the pavement of the street it flies
        // over can share an edge in plan, and a quad between them was a
        // curtain hung from the viaduct to the street — 45 to 59 m of it at
        // the Viaduc de Chillon. What closes a deck's side is its slab
        // ([`crate::structure::DECK_THICKNESS_M`]), and under the slab is
        // air. A step within the slab's depth across the same boundary is a
        // kerb at an abutment or along a deck, and is drawn.
        //
        // **Either face over a span, not exactly one.** The street's own
        // sidewalk beside the viaduct is within the room's reach of the
        // deck, so the walk the deck *carries* claims it and it reads as
        // over a span too — at the Viaduc de Chillon that is every one of the
        // curtains, and a rule on `spanned` differing missed them all.
        let slab = crate::structure::DECK_THICKNESS_M;
        if (fa.spanned || fb.spanned) && ((aa - ba).abs() > slab || (ab - bb).abs() > slab) {
            continue;
        }
        // The higher rail first, as the other sweeps put theirs, so the
        // quad faces out of the step rather than into it.
        let (a_hi, a_lo) = (aa.max(ba), aa.min(ba));
        let (b_hi, b_lo) = (ab.max(bb), ab.min(bb));
        tri.quad([
            [e.a[0], e.a[1], a_hi],
            [e.a[0], e.a[1], a_lo],
            [e.b[0], e.b[1], b_lo],
            [e.b[0], e.b[1], b_hi],
        ]);
        m2 += (a_hi - a_lo + b_hi - b_lo) / 2.0 * (e.b[0] - e.a[0]).hypot(e.b[1] - e.a[1]);
    }
    (tri, m2, tapered)
}

/// The plan positions the walk's two sheets share: the cut at the room's
/// reach and nothing else, since only there does one position carry a
/// vertex on both sides of it.
///
/// [`meet`] needs it because the cut is a seam *inside a single mesh*. The
/// check already skips a rim vertex another room surface shares — that is a
/// seam with a face of its own — but it finds those by looking in the
/// *other* meshes, and this one is in the pavement's own. Left in, every
/// cut vertex is measured against a ground that was never meant to reach
/// it: the ground stops at the room's outline and the cut is well inside
/// it, so `unmet` read 1.66 % → 6.96 % with the numerator rising by 11 460
/// against 11 392 new rim vertices — all of them, which is the signature of
/// a miscounted question rather than a geometry that moved. [`edge_faces`]
/// draws this seam, exactly as the other in-room seams have their faces.
fn cut_seam(pave: &Tri, split: usize) -> HashMap<[i64; 2], f64> {
    let split = split.min(pave.positions.len());
    let near: HashSet<[i64; 2]> = pave.positions[..split].iter().map(|p| key(*p)).collect();
    let mut out = HashMap::new();
    for p in &pave.positions[split..] {
        let k = key(*p);
        if near.contains(&k) {
            out.insert(k, p[2]);
        }
    }
    out
}

/// How the room's meshes and the ground's actually meet — mesh against
/// mesh, which is the only way the question can be asked.
///
/// `contact` used to compare the ground's height at a point with the
/// room's at the same point, both read from the same closure: it answered
/// 2.5e-7 m and it was circular, because a point the room's mesh had no
/// vertex at fell back to a batter the ground had sampled from the same
/// fallback. This asks the meshes instead. At every vertex of the room's
/// rim that is not a kerb — the kerb has its own face — does the ground's
/// mesh have a vertex there at all, and where it does, how far apart do
/// the two stand away from the walls?
///
/// `unmet` is what no closing face can mend: a T-junction, where one mesh
/// put a vertex on a shared edge and the other did not. Both are cut from
/// the same outline by the same mesher, but the room's regions and the
/// ground's `rect − room` are cleaned and ear-clipped apart, so they do
/// not agree on where to subdivide it.
fn meet(
    room: &[&Tri],
    g: &Tri,
    walk_cut: &HashMap<[i64; 2], f64>,
    natural: &dyn Fn(Pt) -> f64,
    portals: &Portals,
    over: &poly::Indexed,
) -> (usize, usize, f64) {
    let gh = seam(&[g]);
    let (mut n, mut unmet, mut worst) = (0usize, 0usize, 0.0f64);
    for (k, tri) in room.iter().enumerate() {
        // The room's other surfaces: a rim vertex one of them shares is a
        // seam inside the room, which has a face of its own.
        let others: Vec<&Tri> = room.iter().enumerate().filter(|(j, _)| *j != k).map(|(_, t)| *t).collect();
        let other = seam(&others);
        let other = &other;
        for (i, j) in rim(tri) {
            for v in [tri.positions[i as usize], tri.positions[j as usize]] {
                // A seam inside the room has its own face, and a tunnel's
                // mouth is an opening: neither is a contact with the ground.
                // The walk's cut is the third of those — a seam within one
                // mesh rather than between two ([`cut_seam`]).
                // And a deck's rim is over the void by construction: what
                // closes it is the soffit the structure step lays, not the
                // ground. Asked for the ground under a viaduct, `unmet`
                // answered 22 % on the overpass specimen and was right
                // about the wrong question.
                if at(other, v).is_some()
                    || at(walk_cut, v).is_some()
                    || portals.open([v[0], v[1]])
                    || over.contains([v[0], v[1]])
                {
                    continue;
                }
                n += 1;
                match at(&gh, v) {
                    None => unmet += 1,
                    Some(h) if (natural([v[0], v[1]]) - v[2]).abs() <= MAX_BENCH_FACE_M => {
                        worst = worst.max((h - v[2]).abs())
                    }
                    Some(_) => {}
                }
            }
        }
    }
    (n, unmet, worst)
}

/// A mesh's boundary edges: the directed edges no triangle uses the other
/// way round, in the winding the one triangle that owns them gave.
///
/// The triangles wind counter-clockwise seen from above, so a boundary
/// edge has the mesh's interior on its left and the outside on its right —
/// the same hand as a region's outer ring, and a hole's boundary comes out
/// of it the same way with no case of its own.
fn rim(tri: &Tri) -> Vec<(u32, u32)> {
    let mut e: Vec<(u32, u32)> = Vec::with_capacity(tri.indices.len());
    for t in tri.indices.chunks_exact(3) {
        for k in 0..3 {
            e.push((t[k], t[(k + 1) % 3]));
        }
    }
    e.sort_unstable();
    e.iter().copied().filter(|(a, b)| e.binary_search(&(*b, *a)).is_err()).collect()
}

/// `outline` with every ring subdivided where it crosses the terrain
/// lattice: the vertices the two meshes put there themselves. The seam is
/// measured and the wall is built over these rather than over the ring's
/// own corners, so that neither can miss what happens between two of them —
/// which is where the crack was, and why `contact` could not see it.
fn dense(outline: &Shapes, grid: &crate::grid::Grid) -> Shapes {
    outline
        .iter()
        .map(|shape| {
            shape
                .iter()
                .map(|ring| {
                    let mut out: Ring = Vec::with_capacity(ring.len());
                    for i in 0..ring.len() {
                        let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
                        out.push(a);
                        let mut cut = crate::drape::split(grid, a, b);
                        cut.pop();
                        out.append(&mut cut);
                    }
                    out
                })
                .collect()
        })
        .collect()
}

/// The height the room's mesh gave every one of its vertices, keyed the
/// way [`mesh::WELD_M`] welds them. The room's outline runs through those
/// vertices, so reading its heights here rather than recomputing them is
/// what makes the seam exact rather than close. Where the carriageway and
/// the pavement both reach a position — a kerb the pavement ends at — the
/// lower is the ground's, so the ground meets the asphalt rather than
/// standing a kerb over it.
pub(crate) fn seam(tris: &[&Tri]) -> HashMap<[i64; 2], f64> {
    lowest(tris.iter().flat_map(|t| &t.positions))
}

/// The lowest height at each keyed position of `ps`: [`seam`] over bare
/// positions.
fn lowest<'a>(ps: impl IntoIterator<Item = &'a [f64; 3]>) -> HashMap<[i64; 2], f64> {
    let mut out: HashMap<[i64; 2], f64> = HashMap::new();
    for p in ps {
        out.entry(key(*p)).and_modify(|h| *h = h.min(p[2])).or_insert(p[2]);
    }
    out
}

/// Tolerance, in metres, at which two meshes' vertices are taken to be the
/// same one.
///
/// **Not [`mesh::WELD_M`].** A mesh welds its own vertices at a micron, and
/// within one mesh that is right; but the room's regions, the walk's and
/// their union each come out of the polygon kernel separately, and the
/// kernel snaps to [`poly::GRID_M`] — a tenth of a millimetre, a hundred
/// times the weld. A point that has been through one more boolean than its
/// neighbour lands up to half a grid away and never welds to it, so at a
/// micron the two meshes look like strangers along an edge they share.
/// This is the kernel's own grid, which is the finest tolerance at which
/// the question can honestly be asked.
const SEAM_M: f64 = poly::GRID_M;

pub(crate) fn key(p: impl AsRef<[f64]>) -> [i64; 2] {
    let p = p.as_ref();
    [(p[0] / SEAM_M).round() as i64, (p[1] / SEAM_M).round() as i64]
}

/// The height `map` holds at `p`, if either mesh put a vertex there.
///
/// Rounding to the kernel's grid is most of the answer and not all of it:
/// two points half a grid apart may still fall either side of a cell's
/// edge and key differently. The eight cells around are asked as well, in
/// a fixed order, so the answer is a function of the meshes and not of a
/// rounding. Any hit is within a grid and a half — a seventh of a
/// millimetre — of the point, and what it carries is a height.
pub(crate) fn at(map: &HashMap<[i64; 2], f64>, p: impl AsRef<[f64]>) -> Option<f64> {
    let k = key(p);
    if let Some(h) = map.get(&k) {
        return Some(*h);
    }
    [[-1, -1], [-1, 0], [-1, 1], [0, -1], [0, 1], [1, -1], [1, 0], [1, 1]]
        .into_iter()
        .find_map(|[dx, dy]| map.get(&[k[0] + dx, k[1] + dy]).copied())
}

/// What the ground's answer came to.
#[derive(Debug, Default, Clone)]
pub struct Earth {
    pub triangles: usize,
    pub vertices: usize,
    /// Outline vertices, and those whose height the room's mesh did not
    /// have: the seam is exact only for the ones it did.
    pub outline: usize,
    pub unseamed: usize,
    /// Steps lying on an arrangement edge: what §3.3's rule can reach.
    pub on_edge: usize,
    /// The one mesh's vertices, and how many of them a paved face pins.
    pub relaxed: usize,
    pub pinned: usize,
    /// How far the relax solve's ground stands from the case-function's,
    /// over every free vertex, in metres.
    pub against: Residual,
    pub asked: usize,
    /// The room's rim, and how much of it the ground's mesh has no vertex
    /// under: a T-junction, and the one thing a closing face cannot mend.
    pub rim: usize,
    pub unmet: usize,
    /// The largest step, in metres, between the room's mesh and the
    /// ground's at a vertex they do share, away from the walls.
    pub contact: f64,
    /// Outline vertices standing more than one face from the ground,
    /// where the bench is walled rather than battered, and the tallest
    /// of those walls in metres.
    pub walled: usize,
    pub wall: f64,
    /// Lattice vertices, and those the bench moved. Invariant 8 — the
    /// ground outside every toe is the DEM's, bit for bit — holds by
    /// construction, since a point further from the outline than a face
    /// may run is never asked about; the count says how much ground the
    /// bench actually moved.
    pub lattice: usize,
    pub touched: usize,
    /// How far, in metres, the ground mesh's triangles stand off the
    /// engineered ground at their centroids — the batter's crease, which
    /// no breakline resolves yet.
    pub off: f64,
    /// Regions of the ground the ear clipper could not read.
    pub lossy: usize,
    /// The area of the closing face, in square metres.
    pub wall_m2: f64,
    /// The area of the kerb's own face, and the faces of it that taper to
    /// nothing at one end because the pavement stops there.
    pub kerb_m2: f64,
    pub tapered: usize,
}

impl Earth {
    /// Measures the ground against the room it was cut for.
    fn new(outline: &Shapes, ground: &Ground, room: &dyn Fn(Pt) -> f64, natural: &dyn Fn(Pt) -> f64, t: &Terrain) -> Earth {
        let mut e = Earth::default();
        for ring in outline.iter().flatten() {
            for &q in ring {
                e.outline += 1;
                let (r, n) = (room(q), natural(q));
                // A walled vertex has no contact to measure: the ground
                // there is the natural ground and the wall between them
                // is the answer, so it is counted rather than averaged in.
                if (n - r).abs() > MAX_BENCH_FACE_M {
                    e.walled += 1;
                    e.wall = e.wall.max((n - r).abs());
                }
            }
        }
        e.lattice = t.grid.vertex_count();
        for i in 0..e.lattice {
            let [x, y, z] = t.position(i);
            if ground.at([x, y], z) != z {
                e.touched += 1;
            }
        }
        e
    }
}

/// `tri` at the field's height plus `rise`, and what that did. The
/// asphalt (`walk` false) takes the road's height wherever it reaches,
/// being the road; the walk beside it is asked how far out it lies, and
/// past the room's reach stands on [`Foot::batter`]'s face instead.
///
/// The mesh step left every vertex at the ground, so the ground a vertex
/// was cut from or filled to is the height it arrives with: this reads it
/// there rather than sampling the terrain again, and the two cannot drift
/// apart.
fn lift(tri: &Tri, field: &Field, rise: f64, walk: bool, split: usize) -> (Tri, Stats) {
    by_sheet(tri, std::slice::from_ref(field), &[], &[], None, rise, walk, split)
}

/// One height field per sheet of `family`, in the sheets' own order — the
/// order [`crate::mesh`] numbered them in, so a vertex's sheet id indexes
/// this directly.
///
/// A sheet names its axes as profile indices with an arc range, and the
/// ranges of one profile are gathered into one entry: a profile is one
/// axis, and [`Field::of_stations`] numbers axes by entry, so splitting a
/// way across two entries would make the blend treat it as two ways
/// meeting itself.
///
/// **Both the ground and the structure stations**, unlike
/// [`Field::grounded`]. A sheet that holds a span holds it because the
/// span shares a connector with the sheet's ground pieces, and the
/// profile is continuous through that connector — so the chord is the
/// honest answer over the deck, and asking the ground runs alone would
/// leave the deck to be answered by an approach eighteen metres away.
fn fields(sheets: &Sheets, family: Family, profiles: &Profiles) -> (Vec<Field>, Vec<Field>) {
    (of_axes(sheets, family, profiles, false), of_axes(sheets, family, profiles, true))
}

/// One field per sheet of [`Sheets::sheets`], indexed as the mesh's sheet
/// tags are; a sheet of another family has an empty one.
fn of_axes(sheets: &Sheets, family: Family, profiles: &Profiles, chords: bool) -> Vec<Field> {
    sheets
        .sheets
        .iter()
        .map(|sheet| {
            if sheet.family != family {
                return Field::default();
            }
            let mut ranges: BTreeMap<usize, Vec<(usize, usize)>> = BTreeMap::new();
            for &(profile, a0, a1) in if chords { &sheet.chords } else { &sheet.axes } {
                let Some(p) = profiles.profiles.get(profile) else {
                    continue;
                };
                // Inclusive at both ends: the station on a piece boundary is
                // the abutment and belongs to both sides, which is what
                // carries the field across it without a gap.
                let first = p.stations.iter().position(|st| st.s >= a0 - 1e-9);
                let last = p.stations.iter().rposition(|st| st.s <= a1 + 1e-9);
                if let (Some(k0), Some(k1)) = (first, last) {
                    if k0 <= k1 {
                        // **A station past each end.** The two fields have
                        // to overlap where they hand over, or the ground's
                        // axes stop short of the connector, only the
                        // chords' field has a joint to blend at, and the
                        // surface creases along the mask's edge. One
                        // station is `structure::runs_of`'s own abutment
                        // margin, read here for the same reason.
                        let last = p.stations.len() - 1;
                        ranges
                            .entry(profile)
                            .or_default()
                            .push((k0.saturating_sub(1), (k1 + 1).min(last)));
                    }
                }
            }
            Field::of_stations(ranges.into_iter().filter_map(|(i, r)| Some((profiles.profiles.get(i)?, r))))
        })
        .collect()
}

/// The same, with a field per sheet and the sheet of every vertex.
///
/// **A vertex's height is a function of its own sheet's field alone**
/// (invariant I1 of `one-surface-at-a-junction-2026-09-14.md`). One field
/// over the whole family answers a vertex from whatever axis is nearest
/// within [`FIELD_LIMIT_M`], and across a grade separation that is the
/// wrong axis: a deck's vertex takes the height of the street eighteen
/// metres below it because the street's axis happened to be nearer in
/// plan. Asked of its own sheet the question cannot be got wrong, because
/// the other surface's axes are not in the field to be found.
///
/// `of` empty means one field for every vertex, which is what the walk and
/// anything else unpartitioned wants.
fn by_sheet(
    tri: &Tri,
    fields: &[Field],
    chords: &[Field],
    of: &[u32],
    over: Option<&poly::Indexed>,
    rise: f64,
    walk: bool,
    split: usize,
) -> (Tri, Stats) {
    let mut out = tri.clone();
    let mut stats = Stats { vertices: tri.positions.len(), ..Stats::default() };
    for (i, p) in tri.positions.iter().enumerate() {
        // **The surface the vertex is on answers for it** (invariant I1).
        // The two fields overlap by a station at every abutment, so they
        // agree where the mask's edge falls.
        //
        // **And the mask is asked, not believed.** It is a boolean kernel's
        // answer to "which paving is over a deck", and a kernel's answer has
        // threads in it: at the Montreux overbridge two of them, 0.3 and
        // 0.8 m², lay seven metres from any chord in the middle of a
        // junction, and every vertex they caught was answered by the chords'
        // field — which reaches [`FIELD_LIMIT_M`] and clamps to the nearest
        // station, so it handed back the chord's *end* height. The asphalt
        // stood up in a 2.4 m fin along each sliver's edge. So the chord
        // answers only where it is **no further away than the ground the
        // sheet also holds**, which is a fact about the geometry and one the
        // mask cannot get wrong: on a deck the chord is underfoot and the
        // approach is a span away, and in a sliver it is the other way
        // round. Where the mask is right the two are the same profile
        // through one connector and agree anyway.
        let masked = over.is_some_and(|o| o.contains([p[0], p[1]]));
        // Which field answers for this vertex: its own sheet's, or the one
        // whole field there is when the paving was never partitioned.
        let k = if of.is_empty() { 0 } else { of.get(i).copied().unwrap_or(0) as usize };
        let ask = |v: &[Field]| v.get(k).filter(|f| f.len() > 0).and_then(|f| f.at([p[0], p[1]]));
        let grounded = ask(fields);
        let chord = masked
            .then(|| ask(chords))
            .flatten()
            .filter(|c| grounded.is_none_or(|g| c.d <= g.d));
        // Whether this vertex stands on a span, which the earthwork numbers
        // below need and the height does not: a deck's chord and its
        // approach's grade are one field, continuous through the connector
        // they share.
        let on = chord.is_some();
        let Some(foot) = chord.or(grounded) else {
            stats.draped += 1;
            stats.free += 1;
            continue;
        };
        let room_h = foot.h + rise;
        // **A walk takes a road's height only where that road is on the
        // ground the walk is on.** `p[2]` is the natural terrain at this
        // vertex — the mesh was cut on the lattice with every vertex at
        // `height_at` — so the comparison is exact and costs nothing.
        //
        // The walk has no profile and no sheets: it takes whatever road
        // axis is nearest within [`FIELD_LIMIT_M`], with no test that the
        // road is anywhere near it in *height*. Traced along the Chemin du
        // National at Montreux, a hundred metres of footway sits within
        // 13 cm of the ground and then its last vertex lands on the road's
        // axis at a bridge abutment, where the clearance plinth puts the
        // road 8.54 m up — so the pavement goes with it, the terrain opens
        // a hole for it, and the bench walls what it cannot batter.
        //
        // One bench face is the threshold because it is already the rule
        // one level out: a band standing further than [`MAX_BENCH_FACE_M`]
        // from the road beside it is not that road's pavement and drapes.
        // This is the same sentence with the height in it rather than the
        // plan distance. A sidewalk on a genuine embankment keeps its road,
        // which is within a face of the ground beside it by construction.
        let flown = walk && foot.h - p[2] > MAX_BENCH_FACE_M;
        stats.flown += flown as usize;
        // The sheet decides, not the distance ([`Foot::face`]): the walk's
        // near part is the room's plateau outright and its far part the
        // face, so the two disagree on the cut itself and the step is a
        // rim rather than a stretch across a triangle.
        let near = !walk || i < split;
        let h = if near { room_h } else { foot.face(room_h, p[2]) };
        out.positions[i][2] = h;
        // Where the vertex stands, not whether the height moved: on flat
        // ground the room's height *is* the ground, and a road there is
        // still a road.
        if near {
            stats.lifted += 1;
        } else if h == p[2] {
            stats.draped += 1;
        } else {
            stats.battered += 1;
        }
        // **A deck is paved, not banked.** The earthwork numbers are what
        // the ground stage owes; the standoff under a span is the
        // structure's, and counted here it reads as an embankment nobody
        // built — 6.5 m of it under the overpass specimen alone.
        if on {
            continue;
        }
        if (h - p[2]).abs() > MAX_BENCH_FACE_M {
            stats.walled += 1;
        }
        stats.cut = stats.cut.max(p[2] - h);
        stats.fill = stats.fill.max(h - p[2]);
    }
    let mut seen: HashSet<(u32, u32)> = HashSet::new();
    for t in out.indices.chunks_exact(3) {
        for e in 0..3 {
            let (a, b) = (t[e], t[(e + 1) % 3]);
            if !seen.insert((a.min(b), a.max(b))) {
                continue;
            }
            stats.edges += 1;
            let (p, q) = (out.positions[a as usize], out.positions[b as usize]);
            let dh = (q[2] - p[2]).abs();
            let len = (q[0] - p[0]).hypot(q[1] - p[1]);
            if dh > KERB_RISE_M + STEP_SLACK_M && dh > STEP_GRADE * len {
                stats.steps += 1;
                stats.worst = stats.worst.max(dh);
                stats.at.push([(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0]);
            }
        }
    }
    (out, stats)
}

#[cfg(test)]
pub(crate) mod tests {
    use crate::world::World;
    use crate::pipeline::tests::{built, upto};
    use crate::step::Step;
    

    
    use crate::world::Solved;
    

    use super::*;

    /// A world on `ground` with the network of `net` and the buildings of
    /// `houses`, meshed and benched.
    pub(crate) fn world(ground: &str, net: &str, houses: Option<&str>) -> (World, Summary) {
        let (w, ran) = built(ground, net, houses, 5.0, &upto(Step::Bench));
        (w, ran.last())
    }

    fn bench(w: &World) -> &Bench {
        w.bench.as_ref().unwrap()
    }

    #[test]
    fn the_field_is_the_profile_at_the_foot() {
        // One axis along x, rising 1 m per 10 m, stationed every 10 m.
        let stations: Vec<crate::world::Station> = (0..=10)
            .map(|k| {
                let x = k as f64 * 10.0;
                crate::world::Station {
                    s: x,
                    p: [x, 0.0],
                    ground: 400.0,
                    reference: 400.0,
                    h: 400.0 + x / 10.0,
                    solved: Solved::Grade,
                }
            })
            .collect();
        let p = Profile {
            way: 0,
            id: "road".into(),
            class: "residential".into(),
            width_m: 5.5,
            spans: vec![crate::world::Span { a0: 0.0, a1: 100.0, kind: crate::world::Kind::Ground }],
            stations,
        };
        let f = Field::new(std::slice::from_ref(&p));
        assert_eq!(f.len(), 10);
        // On the axis, beside it, and past its end: the foot's height, and
        // the perpendicular distance to it.
        for (q, want_h, want_d) in
            [([25.0, 0.0], 402.5, 0.0), ([25.0, 7.0], 402.5, 7.0), ([25.0, -3.0], 402.5, 3.0), ([115.0, 0.0], 410.0, 15.0)]
        {
            let foot = f.at(q).unwrap();
            assert!((foot.h - want_h).abs() < 1e-9 && (foot.d - want_d).abs() < 1e-9, "{q:?}: {foot:?}");
            assert_eq!(foot.half_w, 2.75);
        }
        // Past the room's reach the walk comes down a face at 1 in 2.5
        // and stops where it meets the ground: a 4 m drop daylights 10 m
        // out, a 1 m drop 2.5 m out, and inside the reach there is no
        // face at all.
        let reach = 2.75 + ROOM_REACH_M;
        let batter = |d: f64, ground: f64| Foot { h: 0.0, d, half_w: 2.75 }.batter(0.0, ground);
        assert_eq!(batter(0.0, 2.0), 0.0);
        assert_eq!(batter(reach, 2.0), 0.0);
        assert_eq!(batter(reach + 2.5, 2.0), 1.0);
        assert_eq!(batter(reach + 5.0, 2.0), 2.0);
        assert_eq!(batter(reach + 30.0, 2.0), 2.0, "daylighted, and the ground beyond");
        assert_eq!(batter(reach + 2.5, -2.0), -1.0, "the fill side is the mirror");
        assert_eq!(batter(reach + 0.001, 0.0), 0.0, "nothing to close, no face");
        // A difference no face may close is not this road's to close: the
        // band is free and takes the ground.
        assert_eq!(batter(reach + 2.5, MAX_BENCH_FACE_M + 0.5), MAX_BENCH_FACE_M + 0.5);
        // Past the limit the field has nothing to say, which is the same
        // answer as a world with no road in it: the ground.
        assert!(f.at([130.0, 0.0]).is_none());
        // A span is not in the field: nothing but ground pieces sets a
        // height the surface reads.
        let mut deck = p.clone();
        deck.spans = vec![crate::world::Span { a0: 0.0, a1: 100.0, kind: crate::world::Kind::Bridge(1) }];
        assert!(Field::new(std::slice::from_ref(&deck)).at([0.0, 0.0]).is_none());
    }

    /// **A chord answers only where it is.** The span mask is a boolean
    /// kernel's answer to which paving stands on a deck, and a kernel's
    /// answer has threads in it: at the Montreux overbridge two of them,
    /// 0.3 and 0.8 m², lay seven metres from any chord in the middle of a
    /// junction. Every vertex they caught was handed to the chords' field,
    /// which reaches [`FIELD_LIMIT_M`] and clamps to its nearest station, so
    /// it gave back the chord's *end* height — and the asphalt stood up in a
    /// 2.4 m fin along each sliver's edge (61 near-vertical carriageway
    /// triangles near that junction, the worst rising 2.40 m; 3 and 0.59 m
    /// after). The chord is believed only where it is no further off than
    /// the ground the same sheet holds, which the mask cannot get wrong.
    #[test]
    fn a_sliver_in_the_span_mask_does_not_lift_the_asphalt() {
        // One way: 100 m of ground along x, level to x = 60 and then
        // climbing its abutment at 20 %, and a deck that turns away from it
        // at the abutment and runs 40 m level — the shape the road has where
        // it leaves a junction onto a bridge.
        let at = |x: f64, y: f64, s: f64, h: f64| crate::world::Station {
            s,
            p: [x, y],
            ground: 400.0,
            reference: 400.0,
            h,
            solved: Solved::Grade,
        };
        let mut stations: Vec<crate::world::Station> =
            (0..=20).map(|k| k as f64 * 5.0).map(|x| at(x, 0.0, x, 400.0 + 0.2 * (x - 60.0).max(0.0))).collect();
        stations.extend((1..=4).map(|k| k as f64 * 10.0).map(|y| at(100.0, y, 100.0 + y, 408.0)));
        let p = Profile {
            way: 0,
            id: "road".into(),
            class: "residential".into(),
            width_m: 5.5,
            spans: vec![
                crate::world::Span { a0: 0.0, a1: 100.0, kind: crate::world::Kind::Ground },
                crate::world::Span { a0: 100.0, a1: 140.0, kind: crate::world::Kind::Bridge(1) },
            ],
            stations,
        };
        // The two fields the sheet would build, overlapping by one station
        // at the abutment exactly as `of_axes` makes them.
        let ground = Field::of_stations(std::iter::once((&p, vec![(0usize, 20usize)])));
        let chord = Field::of_stations(std::iter::once((&p, vec![(19usize, 24usize)])));

        // A vertex six metres off the axis and twelve short of the abutment.
        // The ground axis is 6 m away and says 405.6; the chord is 9.2 m
        // away — inside the field's reach — and, clamped to its own first
        // station, says 407.0.
        let mut tri = Tri::default();
        tri.triangle([[88.0, 6.0, 0.0], [88.0, 6.5, 0.0], [88.5, 6.0, 0.0]]);
        assert!((chord.at([88.0, 6.0]).expect("the chord reaches it").h - 407.0).abs() < 1e-9);
        // The mask lies about it — a sliver of a deck that is not there.
        let over = poly::Indexed::new(&vec![poly::rect(87.0, 5.0, 90.0, 7.0)]);

        let (out, stats) = by_sheet(&tri, &[ground], &[chord], &[0, 0, 0], Some(&over), 0.0, false, 0);
        let h = out.positions[0][2];
        assert!((h - 405.6).abs() < 1e-9, "the sliver handed the vertex the chord's end height: {h}");
        // And the earthwork under it is the ground's to owe, not a deck's.
        assert!(stats.fill > 0.0, "the sliver excused the fill under it too");
    }

    #[test]
    fn flat_ground_moves_nothing() {
        let (w, s) = world("flat", "net:straight?len=200", None);
        let b = bench(&w);
        assert!(b.carriageway.positions.iter().all(|p| (p[2] - 400.0).abs() < 1e-9), "{s}");
        assert_eq!(s.num("cut"), 0.0);
        assert_eq!(s.num("fill"), 0.0);
        assert_eq!(s.num("step"), 0.0);
        assert_eq!(s.num("lifted"), b.carriageway.positions.len() as f64);
    }

    #[test]
    fn a_road_along_the_contour_is_level_crosswise() {
        // The specimen of the plan: a 5.5 m residential across a 30 %
        // slope. Its axis is level, so the road is level, and the ground
        // it stands in is half the width times the slope — 0.825 m of cut
        // at the uphill kerb and 0.825 m of fill at the downhill one,
        // exactly. That is the earth the ground has still to move.
        let (w, s) = world("ramp?grade=0.3&bearing=0&radius=100000", "net:straight?len=200", None);
        let b = bench(&w);
        assert!(b.carriageway.positions.iter().all(|p| (p[2] - 400.0).abs() < 1e-9), "the road is not level");
        assert!((s.num("cut") - 0.825).abs() < 1e-6, "{s}");
        assert!((s.num("fill") - 0.825).abs() < 1e-6, "{s}");
        assert_eq!(s.num("step"), 0.0, "{s}");
        // The road reaches both kerbs: the two numbers above are the
        // ground at the edges of the asphalt, not at some point inside it.
        assert!(b.carriageway.positions.iter().any(|p| (p[1] - 2.75).abs() < 1e-9));
        assert!(b.carriageway.positions.iter().any(|p| (p[1] + 2.75).abs() < 1e-9));
    }

    #[test]
    fn a_road_up_the_slope_is_the_slope() {
        // The same 30 % ramp with the road running straight up it: a
        // street is not grade-limited, so its profile is the ground and
        // the bench moves nothing at all (S9).
        let (w, s) = world("ramp?grade=0.3&bearing=90&radius=100000", "net:straight?len=200", None);
        assert!(s.num("cut") < 1e-9 && s.num("fill") < 1e-9, "{s}");
        let b = bench(&w);
        let t = w.terrain.as_ref().expect("the terrain step ran");
        for p in &b.carriageway.positions {
            assert!((p[2] - crate::terrain::height_at(t, p[0], p[1])).abs() < 1e-9, "{p:?}");
        }
    }

    #[test]
    fn the_pavement_stands_a_kerb_above_the_road() {
        let (w, s) = world("flat", "net:sidewalk?d=6", None);
        let b = bench(&w);
        assert!(!b.pavement.positions.is_empty());
        assert!(b.pavement.positions.iter().all(|p| (p[2] - 400.12).abs() < 1e-9), "{s}");
        assert!(b.carriageway.positions.iter().all(|p| (p[2] - 400.0).abs() < 1e-9));
        assert_eq!(s.num("draped"), 0.0, "a sidewalk is pavement, not a free band: {s}");
    }

    /// **Under an overpass there are two roads, and neither is hung on the
    /// other.** The street keeps its asphalt at the ground under the deck,
    /// the deck keeps its own at its height over it, and no triangle of the
    /// asphalt or of the kerb's faces reaches from one to the other. The
    /// Viaduc de Chillon read the opposite on all three: the streets under
    /// it were bare terrain, and the deck's asphalt hung 44 m down onto them
    /// in curtains (`sheet`'s apart axes, `arrangement::decks`,
    /// [`edge_faces`]).
    #[test]
    fn a_street_under_a_deck_is_paved_and_the_deck_is_not_hung_on_it() {
        let (w, s) = world("flat?h=400", "net:overpass?len=201", None);
        let b = bench(&w);
        // Heights of the asphalt over a point of the crossing square, off
        // every edge of it.
        let q = [0.3, 0.2];
        let mut over: Vec<f64> = Vec::new();
        let c = &b.carriageway;
        for t in c.indices.chunks_exact(3) {
            let v: Vec<[f64; 3]> = t.iter().map(|&i| c.positions[i as usize]).collect();
            let side = |a: [f64; 3], b: [f64; 3]| (b[0] - a[0]) * (q[1] - a[1]) - (b[1] - a[1]) * (q[0] - a[0]);
            let d = [side(v[0], v[1]), side(v[1], v[2]), side(v[2], v[0])];
            if d.iter().all(|x| *x > 0.0) || d.iter().all(|x| *x < 0.0) {
                over.extend(v.iter().map(|p| p[2]));
            }
        }
        assert!(over.iter().any(|z| (z - 400.0).abs() < 0.5), "no street under the deck: {over:?} {s}");
        assert!(over.iter().any(|z| *z > 405.0), "no deck over the street: {over:?} {s}");
        // The tallest triangle of each: a ramp's rise over a cell, not a
        // curtain. The deck climbs 6.5 m over some 45 m, so nothing honest
        // is near five.
        for (name, t) in [("carriageway", &b.carriageway), ("kerb", &b.kerb)] {
            let worst = t
                .indices
                .chunks_exact(3)
                .map(|tr| {
                    let z: Vec<f64> = tr.iter().map(|&i| t.positions[i as usize][2]).collect();
                    z.iter().cloned().fold(f64::MIN, f64::max) - z.iter().cloned().fold(f64::MAX, f64::min)
                })
                .fold(0.0, f64::max);
            assert!(worst < 5.0, "{name} has a triangle {worst:.2} m tall: {s}");
        }
    }

    #[test]
    fn the_pavement_rides_the_road_it_is_beside() {
        // On the contour, the sidewalk 6 m off the axis is level with the
        // road plus the kerb — not on the hillside it lies on, which at
        // 6 m out stands 1.8 m higher.
        let (w, _) = world("ramp?grade=0.3&bearing=0&radius=100000", "net:sidewalk?d=6", None);
        let b = bench(&w);
        assert!(b.pavement.positions.iter().all(|p| (p[2] - 400.12).abs() < 1e-9));
        let m = w.mesh.as_ref().unwrap();
        assert!(m.pavement.positions.iter().any(|p| p[2] > 401.5), "the ground under it climbs");
    }

    #[test]
    fn a_footway_leaving_the_room_leaves_it_over_one_face() {
        // The stub footway runs 20 m north from the kerb, up a 30 %
        // slope. Its first metres are pavement and ride the road; past
        // the room's reach it comes down a face at 1 in 2.5; and where
        // the hill has climbed further than one face can follow, the band
        // is no longer this road's pavement and takes the ground.
        //
        // That last transition is a wall, and the numbers of it are the
        // measurement. The room's plateau ends 8.75 m out, where the
        // ground stands 2.51 m over the road. The face closes 0.4 m of
        // the difference per metre and the hill opens 0.3 m of it, so
        // they never meet: at 10.4 m out the difference reaches the
        // 3 m a face may be and the band steps to the ground, 2.34 m of
        // wall in the continuum and 3.53 m across the mesh edge that
        // straddles it. It is counted, it is marked in the plan view, and
        // the ground's answer — a batter cut into the hill, in this
        // step's second half — is what removes it.
        let (w, s) = world("ramp?grade=0.3&bearing=0&radius=100000", "net:stub?d=0.5", None);
        let b = bench(&w);
        assert!(s.num("lifted") > 0.0 && s.num("draped") > 0.0, "{s}");
        // Bounded: the plateau is 2.51 m of cut and nothing else is — and
        // it is now *exactly* that. The mesh step cuts the walk at the
        // room's reach, so a vertex lands on the plateau's own edge 8.75 m
        // out instead of at whichever lattice crossing fell just inside it,
        // and the measurement reaches the figure this comment always
        // described: 8.75 m of 30 % slope less the kerb's rise, 2.505. It
        // read 2.481 while that edge was only ever approached.
        assert!((s.num("cut") - 2.505).abs() < 0.01, "{s}");
        // Where asking per region dragged the far end of this same
        // footway 5.88 m into the air.
        assert!(s.num("cut") < 3.0, "{s}");
        assert!(s.num("step") > 0.0 && s.num("worst") < 4.0, "{s}");
        // The wall stands where the hill outruns the face, not at the
        // kerb: every step is out beyond the plateau.
        assert!(b.steps.iter().all(|p| p[1] > 2.75 + ROOM_REACH_M), "{:?}", b.steps);
    }

    /// A band whose lift/drape boundary falls in its own **interior**,
    /// which is the case the rest of this corpus cannot express.
    ///
    /// A mapped sidewalk is centred on the cut: 2.75 m of half-width plus
    /// the room's 6 m reach puts it 8.75 m off the axis, and the band is
    /// [`crate::width::WALK_M`] wide, so half lies within the reach and half
    /// beyond. On a 50 % flank the ground there stands 4.255 m over the road
    /// — 8.75 × 0.5 less the kerb's rise, past one face — so the far half
    /// drapes to the ground while the near half rides the road.
    ///
    /// **Every other specimen puts that boundary on a band's own edge**,
    /// where it is already a rim, and every face this step draws is built
    /// off a rim. That is why 232 tests stayed green while 1 599 of the loop
    /// box's 1 801 stretched pavement triangles were interior: a narrow band
    /// cannot reach across the reach, and only a mapped walk can. Both ways
    /// this can regress are caught here — merge the sheets and an edge spans
    /// the step; give the far sheet `batter` instead of `face` and the two
    /// agree on the cut, putting the step back one vertex out, inside the
    /// far sheet.
    ///
    /// It was falsified rather than merely written: handing the far sheet
    /// `batter` instead of `face` takes this specimen to `step` 168/2643,
    /// `worst` 4.755 and `wall_m2` 890 → 22 — the face stops being drawn
    /// and the drop goes back to being spanned by mesh edges, which is the
    /// defect exactly.
    #[test]
    fn a_band_across_the_reach_steps_on_a_rim_not_inside_itself() {
        let (w, s) = world("ramp?grade=0.5&bearing=0&radius=100000", "net:sidewalk?d=8.75", None);
        let b = bench(&w);
        assert!(!b.pavement.positions.is_empty(), "{s}");
        // Not vacuous: the band straddles, so both rules fire on it, and
        // the drop it straddles is the one a face cannot follow.
        assert!(s.num("lifted") > 0.0 && s.num("draped") > 0.0, "the band must straddle: {s}");
        assert!((s.num("cut") - 4.255).abs() < 0.01, "{s}");
        assert!(s.num("cut") > MAX_BENCH_FACE_M, "the drop must be past one face: {s}");
        // The property: no mesh edge spans the step. The sheets share no
        // vertex along the cut, so the drop is a face and not a stretch.
        assert_eq!(s.num("step"), 0.0, "an edge spans the walk's own step: {s}");
        assert_eq!(s.num("worst"), 0.0, "{s}");
        // And nothing in the pavement stands up: the tallest triangle is
        // lattice relief, not the 4.255 m the field steps by.
        let tallest = b
            .pavement
            .indices
            .chunks_exact(3)
            .map(|t| {
                let z = [
                    b.pavement.positions[t[0] as usize][2],
                    b.pavement.positions[t[1] as usize][2],
                    b.pavement.positions[t[2] as usize][2],
                ];
                z.iter().copied().fold(f64::MIN, f64::max) - z.iter().copied().fold(f64::MAX, f64::min)
            })
            .fold(0.0f64, f64::max);
        assert!(tallest < 1.0, "a pavement triangle spans {tallest:.3} m of the step: {s}");
    }

    #[test]
    fn a_junction_on_a_hill_has_one_height() {
        // Four legs meeting at the origin: the profile pins the connector,
        // so the four surfaces meet there without a step, and the field is
        // continuous across the whole junction.
        let (w, s) = world("hill?amp=60&radius=400", "net:cross?len=400", None);
        assert_eq!(s.num("step"), 0.0, "{s}");
        let b = bench(&w);
        let t = w.terrain.as_ref().expect("the terrain step ran");
        let centre: Vec<f64> =
            b.carriageway.positions.iter().filter(|p| p[0].hypot(p[1]) < 1.0).map(|p| p[2]).collect();
        assert!(!centre.is_empty());
        let top = crate::terrain::height_at(t, 0.0, 0.0);
        assert!(centre.iter().all(|h| (h - top).abs() < 1e-9), "the junction is not at the ground's height");
    }

    /// The three steepest edges of `tri` — rise over run, and where the
    /// edge lies — over the edges long enough for a grade to mean
    /// something. Where the worst edge is says which rule made it: a
    /// junction's own corner, or the ring where the blend hands back.
    fn steepest(tri: &Tri) -> Vec<(f64, Pt)> {
        let mut edges: Vec<(f64, Pt)> = tri
            .indices
            .chunks_exact(3)
            .flat_map(|t| [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])])
            .map(|(a, b)| {
                let (p, q) = (tri.positions[a as usize], tri.positions[b as usize]);
                let run = (q[0] - p[0]).hypot(q[1] - p[1]);
                let grade = if run < 0.1 { 0.0 } else { (q[2] - p[2]).abs() / run };
                (grade, [(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0])
            })
            .collect();
        edges.sort_by(|a, b| b.0.total_cmp(&a.0));
        edges.truncate(3);
        edges
    }

    #[test]
    fn a_junction_on_a_slope_is_one_surface() {
        // Every leg of a junction is level crosswise, so on a flank the
        // legs' cross-sections disagree everywhere off the connector: by
        // g·s where a leg climbing the flank meets one along its contour,
        // by twice that where two legs climb it at 45°. Asked of the
        // nearest axis alone, the room stepped on the line where two legs
        // are equidistant — 0.875 m on a 15 % flank — and every junction
        // on the box's hillside read as a bump.
        let mut got = Vec::new();
        for ground in ["ramp?grade=0.15&bearing=0&radius=100000", "ramp?grade=0.15&bearing=45&radius=100000"] {
            for net in ["net:tee", "net:cross"] {
                let (w, s) = world(ground, net, None);
                let worst = steepest(&bench(&w).carriageway);
                let from_joint = worst[0].1[0].hypot(worst[0].1[1]);
                eprintln!("{ground} {net}: step={} worst={worst:.3?} at {from_joint:.1} m from the joint", s.num("step"));
                got.push((ground, net, s.num("step"), worst[0].0));
            }
        }
        // What is left in the corners is a twist, not a step: a level road
        // meeting a 15 % side street has to warp through its kerb returns,
        // and every worst edge is one of those, ~6 m from the connector.
        // Measured 0.33 to 0.42 over the four, against 1.01 to 2.19 from
        // the nearest axis alone. The bound is where those sit, not where
        // the band was tuned: at BLEND_M 3 m they read 0.42 and at 8 m
        // 0.31, so the width barely moves them.
        for (ground, net, step, steep) in got {
            assert_eq!(step, 0.0, "{ground} {net}");
            assert!(steep < 0.5, "{ground} {net}: an edge of the junction climbs {steep:.3}");
        }
    }

    /// The engineered ground of a world, and its natural one.
    fn grounds(w: &World) -> (Ground, impl Fn(Pt) -> f64 + '_) {
        let terrain = w.terrain.as_ref().unwrap();
        let natural = move |p: Pt| crate::terrain::height_at(terrain, p[0], p[1]);
        let b = w.bench.as_ref().unwrap();
        let seam = seam(&[&b.carriageway, &b.pavement]);
        let outline = poly::union_of(&[
            &w.fillet.as_ref().expect("the fillet step ran").surface.carriageway,
            &w.room.as_ref().expect("the room step ran").surface.walk,
        ]);
        let room = |q: Pt| at(&seam, q).unwrap_or_else(|| natural(q));
        (Ground::new(&outline, &terrain.grid, &room, &natural, &Portals::default()), natural)
    }

    #[test]
    fn the_batter_is_one_in_two_and_a_half_and_stops_at_the_ground() {
        // The plan's specimen, with a ruler on it. A 5.5 m road along the
        // contour of a 30 % slope is level at 400 m; the ground at its
        // uphill kerb stands 0.825 m over it. A face at 1 in 2.5 gains
        // 0.4 m per metre where the hill gains 0.3, so it closes 0.1 m
        // per metre and would daylight 8.25 m out, 3.3 m above the road.
        let (w, s) = world("ramp?grade=0.3&bearing=0&radius=100000", "net:straight?len=200", None);
        let (g, natural) = grounds(&w);
        let at = |y: f64| g.at([0.0, y], natural([0.0, y]));
        for d in [0.5, 1.0, 2.0, 4.0, 7.0] {
            let (up, down) = (at(2.75 + d), at(-2.75 - d));
            assert!((up - (400.0 + d / EARTHWORK_BATTER)).abs() < 1e-9, "cut at {d}: {up}");
            assert!((down - (400.0 - d / EARTHWORK_BATTER)).abs() < 1e-9, "fill at {d}: {down}");
        }
        // It does not daylight, though: a face is at most
        // MAX_BENCH_FACE_M tall, so it runs 7.5 m and not the 8.25 m this
        // hill needs, and what it has not closed at that point — 0.075 m,
        // less than the kerb it stands beside — is a lip in the ground.
        // Raising the cap is exactly what the tiler measured as making
        // the drawn result worse (data/plans/terrain-hole-plan.md), so
        // the lip stays and is named.
        let run = EARTHWORK_BATTER * MAX_BENCH_FACE_M;
        let lip = natural([0.0, 2.75 + run]) - at(2.75 + run - 1e-9);
        assert!((lip - 0.075).abs() < 1e-6, "{lip}");
        // Past the run the ground is the ground, bit for bit.
        for d in [run + 1e-6, run + 5.0, 100.0] {
            assert_eq!(at(2.75 + d), natural([0.0, 2.75 + d]));
        }
        // And the ground meets the room at the kerb, exactly.
        assert!(s.num("contact") < 1e-9, "{s}");
        assert_eq!(s.num("walled"), 0.0, "{s}");
    }

    #[test]
    fn a_gentler_hill_daylights_where_the_plan_says() {
        // On a 10 % ramp the same road is cut 0.275 m at its uphill kerb
        // and the face closes 0.3 m per metre of run: 0.92 m of batter,
        // well inside what a face may run, so it daylights exactly.
        let (w, _) = world("ramp?grade=0.1&bearing=0&radius=100000", "net:straight?len=200", None);
        let (g, natural) = grounds(&w);
        let at = |y: f64| g.at([0.0, y], natural([0.0, y]));
        let toe = 0.275 / (1.0 / EARTHWORK_BATTER - 0.1);
        assert!((toe - 0.9166666666).abs() < 1e-6, "{toe}");
        assert!((at(2.75 + toe / 2.0) - (400.0 + toe / 2.0 / EARTHWORK_BATTER)).abs() < 1e-9);
        for d in [toe + 1e-6, toe + 1.0, 20.0] {
            let (p, n) = ([0.0, 2.75 + d], natural([0.0, 2.75 + d]));
            assert_eq!(g.at(p, n), n, "daylighted at {d}");
        }
    }

    #[test]
    fn the_ground_stops_at_the_kerb() {
        // The room is cut out of the ground: no ground triangle has its
        // centroid inside the asphalt, and the ground's own vertices on
        // the outline are the room's, at the room's height.
        let (w, s) = world("ramp?grade=0.3&bearing=0&radius=100000", "net:sidewalk?d=6", None);
        let b = w.bench.as_ref().unwrap();
        assert!(!b.ground.indices.is_empty());
        let inside = poly::Indexed::new(&w.fillet.as_ref().expect("the fillet step ran").surface.carriageway);
        let n = b
            .ground
            .indices
            .chunks_exact(3)
            .filter(|t| {
                let p = |i: u32| b.ground.positions[i as usize];
                let (a, b2, c) = (p(t[0]), p(t[1]), p(t[2]));
                inside.contains([(a[0] + b2[0] + c[0]) / 3.0, (a[1] + b2[1] + c[1]) / 3.0])
            })
            .count();
        assert_eq!(n, 0, "{n} ground triangles under the asphalt");
        assert!(s.num("contact") < 1e-9, "{s}");
        assert_eq!(s.num("seam"), 0.0, "every outline vertex is a room vertex: {s}");
        // The ground reaches the kerb: some ground vertex stands at the
        // pavement's own height, which on this slope is not the ground's.
        assert!(b.ground.positions.iter().any(|p| (p[2] - 400.12).abs() < 1e-9));
    }

    #[test]
    fn a_flat_world_is_left_alone() {
        // Nothing to bench: every lattice vertex keeps the DEM's height
        // (invariant 8) and the only earth moved is the kerb's own rise.
        let (_, s) = world("flat", "net:sidewalk?d=6", None);
        assert_eq!(s.num("cut"), 0.0, "{s}");
        assert!(s.num("touched") <= 0.0, "{s}");
        assert_eq!(s.num("walled"), 0.0, "{s}");
    }

    #[test]
    fn an_overpass_stands_on_fill_only_as_far_as_a_fill_goes() {
        // The crossing step lifts the approach 6.5 m and the earthwork is
        // this step's to owe: all fill, no cut, on flat ground.
        //
        // **But only up to `DECK_STANDOFF_M`.** Past the tallest face the
        // ground stage will build, the partition calls the approach a deck
        // and the structure step carries it, so the fill this step owes stops
        // there instead of climbing to 6.5 m and being closed by a 6.5 m
        // wall. That wall was the abutment block showing up as a number, and
        // this is the model answering it with the thing it actually is.
        let (_, s) = world("flat", "net:overpass?len=300", None);
        assert_eq!(s.num("cut"), 0.0, "{s}");
        assert!(s.num("fill") <= crate::grade::DECK_STANDOFF_M + 1e-6, "{s}");
        assert!(s.num("fill") > 2.0, "the approach is on no fill at all: {s}");
        assert_eq!(s.num("walled"), 0.0, "the ground still walls it: {s}");
        // **The mirror is not symmetric, and that is the point.** The
        // approach dips toward the bore and the ground owes the cut — but
        // only as far as `BORE_COVER_M`, because a cutting stays a cutting
        // until a tube fits under it, where a fill becomes a deck as soon as
        // it passes the tallest face the ground will build. So the cut runs
        // deeper than the fill did, and between the two thresholds it *is*
        // walled: a cutting three metres deep is a real cutting with real
        // walls, and a fill three metres tall is a deck drawn wrong.
        let (_, s) = world("flat", "net:underpass?len=300", None);
        assert_eq!(s.num("fill"), 0.0, "{s}");
        assert!(s.num("cut") <= crate::partition::BORE_COVER_M + 1e-6, "{s}");
        assert!(s.num("cut") > crate::grade::DECK_STANDOFF_M, "{s}");
        assert!(s.num("walled") > 0.0, "a cutting past one face is walled: {s}");
    }

    #[test]
    fn the_seam_holds_between_the_ring_s_own_vertices() {
        // A straight road's kerb is one 400 m segment of its outline, and
        // the hill under it is a cosine, so the room's height along that
        // segment is not a straight line. Read at the segment's two ends
        // alone, the ground missed it by metres in between — and `contact`
        // could not see that, because it was read at those same two ends.
        // Sampled where the lattice crosses, both meshes draw one edge and
        // there is nothing left to close.
        let (_, s) = world("hill?amp=60&radius=200", "net:straight?len=400", None);
        assert!(s.num("contact") < 1e-6, "{s}");
        assert_eq!(s.num("wall_m2"), 0.0, "a gentle hill has no step to close: {s}");
    }

    #[test]
    fn a_step_no_batter_can_run_is_closed_by_a_wall() {
        // A road along the lip of a 10 m cliff: its room reaches six metres
        // to each side, so one edge stands five metres over the ground and
        // the other five under it — more than `MAX_BENCH_FACE_M` either way,
        // so no batter may run and the ground keeps its own height. That is a
        // step between the room's edge and the terrain, and until it was
        // walled it was a hole you could see the world through (I9).
        //
        // The specimen is a cliff rather than an overpass because an
        // overpass no longer walls: past `DECK_STANDOFF_M` its approach is a
        // deck, which is what R2 is for. A wall is what the ground owes where
        // the road is *not* a structure, and a cliff is that.
        let (w, s) = world("step?rise=10&width=0&bearing=0", "net:straight?len=200", None);
        assert!(s.num("walled") > 0.0, "{s}");
        assert!(s.num("wall_m2") > 500.0, "{s}");
        let b = bench(&w);
        assert!(!b.wall.indices.is_empty());
        // It stands between the two surfaces it closes, and no further: the
        // ground below at one end, the room's own edge at the other.
        let z: Vec<f64> = b.wall.positions.iter().map(|p| p[2]).collect();
        let lo = z.iter().cloned().fold(f64::INFINITY, f64::min);
        let hi = z.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(hi - lo > 4.0, "the wall closes nothing: {lo}..{hi}");
        assert!(lo >= 400.0 - 1e-6 && hi <= 410.0 + 1e-6, "outside the step it closes: {lo}..{hi}");
    }

    #[test]
    fn the_kerb_stands_its_own_face_between_the_road_and_the_pavement() {
        // The pavement stands KERB_RISE_M over the road beside it, and the
        // two meshes met in plan and nowhere at all in the vertical: 0.12 m
        // of gap along every kerb in the model. A 200 m road with one
        // sidewalk is 200 m of kerb at 0.12 m, which is 24 m².
        let (w, s) = world("flat", "net:sidewalk?d=6", None);
        assert!((s.num("kerb_m2") - 24.0).abs() < 3.0, "{s}");
        let b = bench(&w);
        let z: Vec<f64> = b.kerb.positions.iter().map(|p| p[2]).collect();
        let lo = z.iter().cloned().fold(f64::INFINITY, f64::min);
        let hi = z.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!((lo - 400.0).abs() < 1e-9, "the foot is off the road: {lo}");
        assert!((hi - (400.0 + KERB_RISE_M)).abs() < 1e-9, "the head is not the pavement: {hi}");
        // And only where a pavement stands: a road with nothing beside it
        // has no kerb to draw, only the room's own outline, which is the
        // wall's business and not this one's.
        let (_, s) = world("flat", "net:straight", None);
        assert_eq!(s.num("kerb_m2"), 0.0, "{s}");
    }

    /// **A gallery meets the ground on both sides.** A gallery on a flank:
    /// its footprint is cut out of the ground, the ground at its uphill edge
    /// stands between the floor and the roof, and at its downhill edge it
    /// falls 1.375 m below the floor. The uphill side is closed by the
    /// tube's own wall; the downhill one only by a footing from the ground
    /// up to the floor, without which the terrain's edge and the tube stood
    /// apart by the fall and the world showed through between them (seen on
    /// the loop box as white slivers under a gallery along the flank).
    ///
    /// Read off the wall rule itself rather than a specimen: no single
    /// synthetic ground puts a mound over a way *and* a cross-slope under
    /// it, and on a mound the ground never falls below the chord a gallery
    /// runs on.
    #[test]
    fn a_gallery_meets_the_ground_on_both_sides() {
        let axis = [[-50.0, 0.0], [50.0, 0.0]];
        let (half, tube, floor) = (2.75, 5.0, 400.0);
        let mut portals = Portals::default();
        portals.gallery.push(axis[0], axis[1]);
        portals.crowns.push((floor, floor, half, tube));
        let outline = poly::buffer_line_capped(&axis, 2.0 * half, [false, false]);
        let grid = crate::grid::Grid::fit(&crate::frame::Rect { x0: -60.0, y0: -10.0, x1: 60.0, y1: 10.0 }, 1.0, usize::MAX);
        // Falling 1 in 2 toward −y: 1.375 m under the floor at the downhill
        // edge, 1.375 m over it — and under the roof — at the uphill one.
        let natural = |q: Pt| floor + 0.5 * q[1];
        let room = |_: Pt| floor;
        let ground = Ground::new(&outline, &grid, &room, &natural, &portals);
        let (tri, m2) = wall(&dense(&outline, &grid), &ground, &room, &natural, &portals);
        // The downhill side's 100 m at 1.375 m, and the half of each end cap
        // that falls below the floor, a triangle.
        let expected = 100.0 * 1.375 + 2.0 * 0.5 * half * 1.375;
        assert!((m2 - expected).abs() < 0.5, "footing {m2} m2 against {expected}");
        // Nothing on the uphill side, where the tube's wall is the closure.
        assert!(tri.positions.iter().all(|p| p[1] <= 1.5), "a face on the uphill side");
    }

    /// The track bed takes its railway's height at its foot, and nothing
    /// else rides the railway: the asphalt crossing it is the road's.
    #[test]
    fn the_ballast_rides_its_railway_and_nothing_else_does() {
        let (w, s) = world("hill?amp=20&radius=150", "net:level?len=300", None);
        let b = bench(&w);
        assert!(!b.ballast.indices.is_empty(), "{s}");
        let profiles = &w.profile.as_ref().unwrap().profiles;
        let rail = profiles.iter().find(|p| p.id == "rail").unwrap();
        let road = profiles.iter().find(|p| p.id == "road").unwrap();
        let (rails, roads) = (Field::new(std::slice::from_ref(rail)), Field::new(std::slice::from_ref(road)));
        for v in &b.ballast.positions {
            let foot = rails.at([v[0], v[1]]).expect("every bed vertex is beside its railway");
            assert!((v[2] - foot.h).abs() < 1e-9, "{v:?} vs {}", foot.h);
        }
        for v in &b.carriageway.positions {
            let foot = roads.at([v[0], v[1]]).expect("every asphalt vertex is beside its road");
            assert!((v[2] - foot.h).abs() < 1e-9, "{v:?} vs {}", foot.h);
        }
    }

    #[test]
    fn the_bench_is_a_function_of_the_world() {
        let (a, _) = world("hill?amp=60&radius=400", "net:cross?len=400", None);
        let (b, _) = world("hill?amp=60&radius=400", "net:cross?len=400", None);
        let (x, y) = (bench(&a), bench(&b));
        assert_eq!(x.carriageway.positions, y.carriageway.positions);
        assert_eq!(x.pavement.positions, y.pavement.positions);
        assert_eq!(x.ground.positions, y.ground.positions);
        assert_eq!(x.ground.indices, y.ground.indices);
        assert_eq!(x.steps, y.steps);
    }
}
