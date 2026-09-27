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
//! heights step on the line between them, and a face is drawn there: a
//! retaining wall, which is what the hillside physically has. The
//! alternative, a blend between the two, would ramp a pavement at 60 %
//! between two terraces, which is spectacle (invariant 6).
//!
//! **That face is drawn because the step is declared** ([`Rule`]). A paved
//! triangle takes one rule at its centroid — which field, which axis, which
//! stretch of it, and for the far pavement the face or the drape — and all
//! three of its corners are answered by it; a vertex two triangles answer
//! differently is two copies, welded where they agree within a kerb's rise
//! and split otherwise, and the edge rule draws the face across the split
//! (`split_m2`). It used to be decided per vertex, so the switch fell inside
//! whichever triangle straddled it and was drawn as a stretched triangle
//! that nothing closed: 8 400 of them on the loop box. `step` now counts
//! only a jump *within* one rule, and reads 144 there.
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
//! out of it and an earthwork residual added: pinned at the outline to the
//! room's height less the natural ground (one face of it at most, a wall for
//! the rest), falling to nothing at 1 in [`EARTHWORK_BATTER`] of the natural
//! ground, and blended where the nearest outline segment changes, so it is
//! continuous and always meets the ground within 7.5 m. No triangle of the
//! ground lies under the asphalt, which is where every artefact of a ground
//! drawn beneath an opaque surface lives (`data/plans/terrain-hole-plan.md`).
//!
//! **One mesh, copied per surface.** The mesh step triangulates the whole
//! rect once ([`crate::world::Mesh`]), so the paving and the ground share
//! every boundary vertex by index. This step copies each vertex once per
//! surface that reaches it — the ground, each carriageway and ballast sheet,
//! the pavement's near and far halves ([`Copies`]) — and gives each copy its
//! own surface's height. The outline is the one mesh's own edges between a
//! face that cuts and one that does not, the ground's segments are those
//! edges, the wall is swept along them and the edge rule draws its quads
//! across them, every height read by index. It used to be two meshes built
//! apart and matched by position at the kernel's grid with an eight-cell
//! search: `seam` and `unmet` were that search's misses, 3.70 % and 1.28 %
//! on the loop box, and both read exactly zero now on every specimen.
//!
//! **What this step does not do yet.** Nothing re-drapes: a footpath that
//! drapes past the room's reach samples the raw terrain beside a ground that
//! may be benched (`regrade` counts those vertices). And `height_at` is
//! still the terrain's — the structure step reads the natural ground, not
//! this one.

use std::collections::BTreeMap;
use std::collections::HashMap;
use std::collections::HashSet;

use crate::arrangement::{Arrangement, Face, Material};
use crate::poly::{self, Pt, Shapes};
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

/// How far past [`FIELD_LIMIT_M`], in metres, [`Field::on_axis`] still looks:
/// more than any paved triangle is wide, so a corner is never left without
/// the axis its triangle's centroid found.
const ON_AXIS_SLACK_M: f64 = 10.0;

/// The length of a stretch of axis a [`Rule`] names, in metres. Two stretches
/// that meet give their shared boundary the same foot, so they weld; two legs
/// of a hairpin lie more than this apart along it, so they cannot be taken
/// for each other.
pub const PART_M: f64 = 20.0;

/// The band, in metres of distance, over which a foot on one segment of an
/// axis hands over to the foot on the next ([`Field::along`]).
pub const SEGMENT_BLEND_M: f64 = 1.0;

/// How far along its own axis, in metres of arc, [`Field::along`] looks for
/// a second foot: two stations' spacing, far short of a hairpin's other leg.
pub const ALONG_ARC_M: f64 = 8.0;

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
    /// Per segment, the arc along its profile of its two ends.
    arc: Vec<(f64, f64)>,
    /// Per segment, the other end of its axis where the axis closes on
    /// itself — a ring whose last segment ends where its first begins.
    wrap: HashMap<u32, u32>,
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

    /// Every segment within `r` metres of `p`, with the foot's parameter and
    /// the distance. A segment filed under several cells may be visited more
    /// than once, which a max or a min does not mind.
    fn within(&self, p: Pt, r: f64, mut f: impl FnMut(usize, f64, f64)) {
        for cell in poly::cells_over([p[0] - r, p[1] - r, p[0] + r, p[1] + r], CELL_M) {
            for &i in self.cells.get(&cell).into_iter().flatten() {
                let (a, b) = self.seg[i as usize];
                let f0 = poly::nearest_on_segment(a, b, p);
                let d = (p[0] - f0[0]).hypot(p[1] - f0[1]);
                if d > r {
                    continue;
                }
                let len2 = (b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2);
                let t = if len2 > 0.0 { ((f0[0] - a[0]) * (b[0] - a[0]) + (f0[1] - a[1]) * (b[1] - a[1])) / len2 } else { 0.0 };
                f(i as usize, t, d);
            }
        }
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
    /// Which axis of the field answered, in the field's own numbering.
    pub axis: u32,
    /// How far along that axis the foot lies, in metres of its profile.
    pub s: f64,
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
                    f.push(st.p, st.p, st.h, st.h, half_w, axis, (st.s, st.s));
                    continue;
                }
                for w in p.stations[k0..=k1].windows(2) {
                    f.push(w[0].p, w[1].p, w[0].h, w[1].h, half_w, axis, (w[0].s, w[1].s));
                }
            }
        }
        // Every axis's first and last segment, and whether it closes.
        let mut ends: std::collections::BTreeMap<u32, (u32, u32)> = Default::default();
        for (i, &a) in f.axis.iter().enumerate() {
            let e = ends.entry(a).or_insert((i as u32, i as u32));
            e.1 = i as u32;
        }
        for (first, last) in ends.into_values() {
            let (s0, s1) = (f.at.seg[first as usize], f.at.seg[last as usize]);
            if first != last && connector(s1.1) == connector(s0.0) {
                f.wrap.insert(first, last);
                f.wrap.insert(last, first);
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

    #[allow(clippy::too_many_arguments)]
    fn push(&mut self, a: Pt, b: Pt, ha: f64, hb: f64, half_w: f64, axis: u32, arc: (f64, f64)) {
        self.arc.push(arc);
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
        Some(Foot { h: self.joined(p, near, self.along(p, i, t, d), d), d, half_w: self.seg[i].2, axis: near, s: self.arc_at(i, t) })
    }

    /// The arc of segment `i` at `t`.
    fn arc_at(&self, i: usize, t: f64) -> f64 {
        let (a, b) = self.arc[i];
        a + (b - a) * t
    }

    /// What axis `axis` says about `p`, whether or not it is the nearest:
    /// the height a triangle's rule gives a corner another axis is nearer
    /// to ([`Rule`]). Asked a little past [`FIELD_LIMIT_M`], because a corner
    /// can be a triangle's width farther than the centroid that chose it.
    ///
    /// **And only the stretch of it within `part`** ([`PART_M`], with half a
    /// part either side). A way is one axis however it winds, and beside a
    /// hairpin — or inside a roundabout's ring — the nearest foot on it can
    /// jump from one leg to the other, metres apart in height: a switch
    /// within one rule, which no split could see.
    pub fn on_axis(&self, p: Pt, axis: u32, part: i32) -> Option<Foot> {
        let (lo, hi) = ((part as f64 - 0.5) * PART_M, (part as f64 + 1.5) * PART_M);
        let (i, t, d) = self.at.of_where(p, FIELD_LIMIT_M + ON_AXIS_SLACK_M, |k| {
            let (a, b) = self.arc[k];
            self.axis[k] == axis && a.max(b) >= lo && a.min(b) <= hi
        })?;
        Some(Foot { h: self.joined(p, axis, self.along(p, i, t, d), d), d, half_w: self.seg[i].2, axis, s: self.arc_at(i, t) })
    }

    /// The solved height `t` of the way along segment `i`.
    fn height(&self, i: usize, t: f64) -> f64 {
        let (ha, hb, _) = self.seg[i];
        ha + (hb - ha) * t
    }

    /// The axis's height for `p`, whose nearest foot is `t` along segment
    /// `i`, `d` away: that foot blended with the feet on the segments either
    /// side of it, by how nearly each is the nearest ([`SEGMENT_BLEND_M`]).
    ///
    /// **The nearest point of a polyline jumps on the inside of a bend.**
    /// Past the bisector of a vertex the foot moves from one segment to the
    /// next, and on the concave side the two feet stand apart along the axis
    /// — by up to a station's spacing — so the height jumps by the grade
    /// times that. On a 15 % street it is under a kerb's rise; on the loop
    /// box's steepest streets, and round a roundabout climbing a flank, it
    /// was a step of metres inside a single triangle's rule.
    fn along(&self, p: Pt, i: usize, t: f64, d: f64) -> f64 {
        let h = self.height(i, t);
        let (mut hs, mut ws) = (h, 1.0);
        // The segments of the same axis within [`ALONG_ARC_M`] of this one,
        // walked end to end both ways — not just the two neighbours: a
        // profile has a station at every span edge, so a segment may be
        // centimetres long and the foot jump clean over it to the next — and
        // across a ring's seam, where the arc restarts but the road does not.
        let mut others: Vec<usize> = Vec::new();
        let (s0, s1) = self.arc[i];
        let mut j = i;
        while j > 0 && self.axis[j - 1] == self.axis[i] && self.arc[j - 1].1 == self.arc[j].0 && s0 - self.arc[j - 1].1 < ALONG_ARC_M {
            j -= 1;
            others.push(j);
        }
        let mut j = i;
        while j + 1 < self.axis.len() && self.axis[j + 1] == self.axis[i] && self.arc[j].1 == self.arc[j + 1].0 && self.arc[j + 1].0 - s1 < ALONG_ARC_M {
            j += 1;
            others.push(j);
        }
        if let Some(&w) = self.wrap.get(&(i as u32)) {
            others.push(w as usize);
        }
        for j in others {
            let (a, b) = self.at.seg[j];
            let len2 = (b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2);
            if len2 <= 0.0 {
                continue;
            }
            // **Only a foot inside the neighbour is a second answer.** On the
            // outside of a bend, or along a straight run, the neighbour's
            // nearest point is the vertex the two share, which the nearest
            // foot already accounts for; blended in, it flattened the height
            // near every station for a point off the axis.
            let tj = ((p[0] - a[0]) * (b[0] - a[0]) + (p[1] - a[1]) * (b[1] - a[1])) / len2;
            if !(tj > 0.0 && tj < 1.0) {
                continue;
            }
            let f = [a[0] + (b[0] - a[0]) * tj, a[1] + (b[1] - a[1]) * tj];
            let dj = (p[0] - f[0]).hypot(p[1] - f[1]);
            let w = fade((dj - d) / SEGMENT_BLEND_M);
            if w <= 0.0 {
                continue;
            }
            hs += w * self.height(j, tj);
            ws += w;
        }
        hs / ws
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
                // Each leg weighs by how nearly it is the *nearest* leg — not
                // the answering one. The two are the same when the nearest
                // axis answers; when a triangle's rule makes another answer a
                // corner ([`Rule`]), weighing against the answering axis
                // gave it full weight and the nearer leg less, so two legs
                // of one junction disagreed at the corners they share.
                let legs: Vec<(f64, f64)> = members
                    .iter()
                    .filter_map(|&m| {
                        if m == near {
                            return Some((h, d));
                        }
                        let (k, t, dm) = self.at.of_where(p, d + BLEND_M, |k| self.axis[k] == m)?;
                        Some((self.along(p, k, t, dm), dm))
                    })
                    .collect();
                let nearest = legs.iter().map(|l| l.1).fold(d, f64::min);
                let (mut hs, mut ws) = (0.0, 0.0);
                for (hm, dm) in legs {
                    let w = fade((dm - nearest) / BLEND_M);
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

/// The engineered ground: the natural ground plus an earthwork **residual**
/// that is pinned at the room's outline and falls to nothing at 1 in
/// [`EARTHWORK_BATTER`].
///
/// At an outline vertex the residual is the room's height less the natural
/// ground, held to one face ([`MAX_BENCH_FACE_M`]) either way; a wall at the
/// edge closes whatever is left over. Out from each outline segment it falls
/// at the batter's slope, perpendicular to the segment, and a point takes the
/// nearest segment's batter blended over [`EARTH_BLEND_M`] with any segment
/// nearly as near. Three things follow:
///
/// - **It is continuous.** The rule it replaces took the nearest segment
///   alone, so the ground stepped wherever two segments were equidistant and
///   answered differently — the inside of every kerb return — and between a
///   walled segment and a battered one, whose pins differed by the whole of
///   a drop. The blend removes the first, and a pin that is clamped rather
///   than refused removes the second: a drop past one face is a face of
///   batter and a wall for the rest, not all wall or none.
/// - **It always daylights, within 7.5 m.** A pin is at most one face, so its
///   batter is spent at `B · MAX_BENCH_FACE_M`. The rule it replaces ran its
///   face at an *absolute* 1 in 2.5, which up a hill steeper than that never
///   met the ground and was cut off at 7.5 m with a lip standing in it —
///   `off` read 21 m on the loop box.
/// - **Pins do not reach along the kerb.** A cone from every pin (the
///   steepest-allowed extension, tried first) coupled neighbouring pins: a
///   kerb whose cut changed by more than a batter's slope along its own
///   length had one pin's cone override the next, and each override was a
///   wall at the kerb — 10 k m² more of them on the loop box.
///
/// **The batter is relative to the natural ground, not absolute.** On flat
/// ground the two are the same 1 in 2.5. Across a 30 % hill the cut face
/// stands at 70 % and meets the ground 2 m out; at an absolute 1 in 2.5 it
/// never did. It is plan §3.2's residual — "the batter is `e` falling to
/// nothing" — with a slope where the relax solve had a decay.
///
/// It is a function of the point, not a mesh, so anything may be *proven*
/// to lie on it, which is the property the terrain step set out with and
/// the one the crate is built on.
#[derive(Debug, Default)]
pub struct Ground {
    at: Nearest,
    /// Per outline segment, the residual at its two ends.
    e: Vec<(f64, f64)>,
}

/// How far, in metres, a batter reaches before it is spent: one face at the
/// batter's slope.
const EARTH_REACH_M: f64 = EARTHWORK_BATTER * MAX_BENCH_FACE_M;

/// The band, in metres of distance, over which the batter of one outline
/// segment hands over to the next where the nearest changes ([`Ground`]).
pub const EARTH_BLEND_M: f64 = 1.0;

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

impl Ground {
    /// The ground benched to the room's outline: one segment per outline
    /// edge of the one mesh, as `(a, b, [room, natural] at a, the same at
    /// b)`. A segment across a tunnel's mouth is pinned at no residual — no
    /// batter runs into the tube — and the wall over the mouth closes the
    /// hill down to its roof.
    pub fn of_edges(edges: &[(Pt, Pt, [f64; 2], [f64; 2])], portals: &Portals) -> Ground {
        let mut g = Ground::default();
        let pin = |[room, natural]: [f64; 2]| (room - natural).clamp(-MAX_BENCH_FACE_M, MAX_BENCH_FACE_M);
        for &(a, b, at_a, at_b) in edges {
            g.at.push(a, b);
            let mouth = portals.open(a) && portals.open(b);
            g.e.push(if mouth { (0.0, 0.0) } else { (pin(at_a), pin(at_b)) });
        }
        g
    }

    /// How many outline pieces the ground is benched to.
    pub fn len(&self) -> usize {
        self.e.len()
    }

    pub fn is_empty(&self) -> bool {
        self.e.is_empty()
    }

    /// The earthwork residual at `p`: what the ground there stands off the
    /// natural ground.
    pub fn residual(&self, p: Pt) -> f64 {
        let mut near: Vec<(f64, f64)> = Vec::new();
        self.at.within(p, EARTH_REACH_M + EARTH_BLEND_M, |i, t, d| {
            // **A batter does not cross the room.** The outline runs with the
            // room on its left, so a point whose foot is inside a segment and
            // which lies on the segment's left is across the paving from it.
            // Beyond a segment's end its batter is its end vertex's, which a
            // neighbouring segment shares, so a corner stays continuous.
            let (a, b) = self.at.seg[i];
            let left = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]);
            if t > 0.0 && t < 1.0 && left > 0.0 {
                return;
            }
            let (ea, eb) = self.e[i];
            let e = ea + (eb - ea) * t;
            near.push((e.signum() * (e.abs() - d / EARTHWORK_BATTER).max(0.0), d));
        });
        let Some(nearest) = near.iter().map(|x| x.1).min_by(f64::total_cmp) else {
            return 0.0;
        };
        // The segments nearly as near as the nearest, blended by how nearly:
        // continuous where the nearest changes, which is where the rule this
        // replaces stepped. Sorted first, so the sum is a function of the
        // segments and not of the index's visiting order.
        near.sort_by(|x, y| x.1.total_cmp(&y.1).then(x.0.total_cmp(&y.0)));
        let (mut sum, mut weight) = (0.0, 0.0);
        for (e, d) in near {
            let w = fade((d - nearest) / EARTH_BLEND_M);
            sum += w * e;
            weight += w;
        }
        sum / weight
    }

    /// The engineered height at `p`, whose natural ground is `natural`.
    pub fn at(&self, p: Pt, natural: f64) -> f64 {
        natural + self.residual(p)
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
    /// Edges as steep as a step with both ends on the raw DEM.
    pub dem_steep: usize,
    /// Edges as steep as a step that their rule is continuous along.
    pub steep: usize,
}


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

/// No copy: a vertex this surface does not reach, or a rule no axis answers.
const NONE: u32 = u32::MAX;

/// Which surface a triangle of the one mesh is drawn in.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
enum Surface {
    /// The engineered ground: every partition face that does not cut the
    /// terrain, the ground under a deck included.
    Ground,
    /// A carriageway sheet, by its index into [`Sheets::sheets`] —
    /// `u32::MAX` for paving no sheet claims — and the same for the ballast.
    Carriageway(u32),
    Ballast(u32),
    /// The pavement within the room's reach, and past it.
    Near,
    Far,
}

impl Surface {
    /// The paved surface `face` is drawn in, if it is paved.
    fn paved(face: &Face) -> Option<Surface> {
        let sheet = face.sheet.map_or(u32::MAX, |s| s as u32);
        match face.material {
            Material::Ground => None,
            Material::Carriageway => Some(Surface::Carriageway(sheet)),
            Material::Ballast => Some(Surface::Ballast(sheet)),
            Material::Pavement if face.near => Some(Surface::Near),
            Material::Pavement => Some(Surface::Far),
        }
    }
}

/// How one paved triangle's height is decided: which of its surface's two
/// fields answered, which axis of that field, and — for the pavement past
/// the room's reach — whether it stands on the face or drapes.
///
/// **A triangle takes one rule, at all three of its corners.** The height of
/// a paved point used to be decided per vertex: the nearest axis, the chord
/// or the ground, the face or the drape, whichever the vertex's own position
/// chose. Where two neighbouring vertices chose differently the triangle
/// between them was stretched across the switch — 8 400 of them on the loop
/// box, 80 % inside one material where no face was drawn to close them,
/// which is plan §3.3's "a discontinuity is an accident of where a positional
/// case-function changes branch". Now the triangle's centroid chooses, every
/// corner is answered by that choice, and a vertex two triangles answer
/// differently is two copies: welded where they agree, and a declared edge
/// with a face on it where they do not.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub(crate) struct Rule {
    /// The chords' field answered, rather than the ground's.
    pub chord: bool,
    /// The answering axis, or [`NONE`] where no axis reaches.
    pub axis: u32,
    /// Which stretch of that axis, in [`PART_M`] parts of its arc.
    pub part: i32,
    /// The far pavement drapes to the natural ground here rather than
    /// standing on the face.
    pub drape: bool,
}

impl Rule {
    const FREE: Rule = Rule { chord: false, axis: NONE, part: 0, drape: false };
}

/// The fields one paved surface is lifted by, and how.
pub(crate) struct Lift<'a> {
    pub grounded: &'a Field,
    pub chords: Option<&'a Field>,
    /// Where the paving is over a span, so the chords may answer.
    pub over: Option<&'a poly::Indexed>,
    /// What the surface stands above the road: a kerb for the pavement.
    pub rise: f64,
    /// The pavement: past the room's reach it stands on a face.
    pub walk: bool,
    /// Within the room's reach: the road's height outright.
    pub near: bool,
}

impl Lift<'_> {
    /// The rule at `p`, whose natural ground is `natural`.
    ///
    /// **And the mask is asked, not believed.** It is a boolean kernel's
    /// answer to "which paving is over a deck", and a kernel's answer has
    /// threads in it: at the Montreux overbridge two of them, 0.3 and 0.8 m²,
    /// lay seven metres from any chord in the middle of a junction, and every
    /// vertex they caught was answered by the chords' field — which reaches
    /// [`FIELD_LIMIT_M`] and clamps to the nearest station, so it handed back
    /// the chord's *end* height, a 2.4 m fin. So the chord answers only where
    /// it is **no further away than the ground the sheet also holds**: on a
    /// deck the chord is underfoot and the approach a span away, and in a
    /// sliver it is the other way round.
    pub fn rule(&self, p: Pt, natural: f64) -> Rule {
        let ask = |f: &Field| if f.is_empty() { None } else { f.at(p) };
        let grounded = ask(self.grounded);
        let chord = if self.over.is_some_and(|o| o.contains(p)) { self.chords.and_then(ask) } else { None }
            .filter(|c| grounded.is_none_or(|g| c.d <= g.d));
        let (foot, chord) = match (chord, grounded) {
            (Some(c), _) => (c, true),
            (None, Some(g)) => (g, false),
            (None, None) => return Rule::FREE,
        };
        // Past the reach, a band standing more than one face from the road
        // beside it is not that road's pavement at all and drapes.
        let drape = self.walk && !self.near && (natural - (foot.h + self.rise)).abs() > MAX_BENCH_FACE_M;
        Rule { chord, axis: foot.axis, part: (foot.s / PART_M).floor() as i32, drape }
    }

    /// The height `rule` gives `p`, and the foot it read there.
    ///
    /// The rule's own axis is asked even where another is nearer, and the
    /// face is not switched to a drape by the point's own drop: a corner is
    /// answered as its triangle was, which is what keeps the triangle whole.
    pub fn height(&self, p: Pt, natural: f64, rule: Rule) -> (f64, Option<Foot>) {
        if rule.axis == NONE {
            return (natural, None);
        }
        let field = if rule.chord { self.chords.unwrap_or(self.grounded) } else { self.grounded };
        let Some(foot) = field.on_axis(p, rule.axis, rule.part) else {
            return (natural, None);
        };
        let room_h = foot.h + self.rise;
        let h = if !self.walk || self.near {
            room_h
        } else if rule.drape {
            natural
        } else {
            let slack = (foot.d - foot.half_w - ROOM_REACH_M).max(0.0) / EARTHWORK_BATTER;
            room_h + (natural - room_h).clamp(-slack, slack)
        };
        (h, Some(foot))
    }

    /// What a vertex lifted by `rule` to `h` counts as.
    fn account(&self, stats: &mut Stats, rule: Rule, h: f64, natural: f64, foot: Option<Foot>) {
        stats.vertices += 1;
        let Some(foot) = foot else {
            stats.draped += 1;
            stats.free += 1;
            return;
        };
        // **A walk takes a road's height only where that road is on the
        // ground the walk is on**; one bench face over is the threshold.
        stats.flown += (self.walk && foot.h - natural > MAX_BENCH_FACE_M) as usize;
        if !self.walk || self.near {
            stats.lifted += 1;
        } else if h == natural {
            stats.draped += 1;
        } else {
            stats.battered += 1;
        }
        // **A deck is paved, not banked.** The standoff under a span is the
        // structure's, and counted here it reads as an embankment nobody
        // built.
        if rule.chord {
            return;
        }
        if (h - natural).abs() > MAX_BENCH_FACE_M {
            stats.walled += 1;
        }
        stats.cut = stats.cut.max(natural - h);
        stats.fill = stats.fill.max(h - natural);
    }
}

/// A paved surface and the rule its triangle took: what a vertex is copied
/// per.
type Key = (Surface, Rule);

/// One mesh of copies: the positions, and per copy the one-mesh vertex it is
/// a copy of, its key, and the natural ground under it.
#[derive(Default)]
struct Part {
    tri: Tri,
    of: Vec<u32>,
    key: Vec<Key>,
    natural: Vec<f64>,
    /// Per triangle, the key it was answered by.
    face_key: Vec<Key>,
}

impl Part {
    /// The copy of `v` under `key`, made the first time it is asked for.
    fn copy(&mut self, slot: &mut u32, pos: &[[f64; 3]], v: u32, key: Key) -> u32 {
        if *slot == NONE {
            let q = pos[v as usize];
            self.tri.positions.push(q);
            self.of.push(v);
            self.key.push(key);
            self.natural.push(q[2]);
            *slot = (self.tri.positions.len() - 1) as u32;
        }
        *slot
    }
}

/// The one mesh's vertices, copied once per surface and rule that reaches
/// them, and the four meshes the copies make.
///
/// **This is the whole of the seam.** A kerb vertex is on the carriageway
/// and on the pavement, and the two answer a kerb's rise apart; an outline
/// vertex is on the paving and on the ground; a vertex between two roads'
/// domains is answered by each. So a vertex cannot be one height — but it is
/// one *position*, one index of [`Mesh::tri`], and every copy of it is found
/// from that index rather than from where it lies.
struct Copies {
    carriageway: Part,
    ballast: Part,
    pavement: Part,
    ground: Part,
    /// Per one-mesh vertex, its copy in the ground.
    in_ground: Vec<u32>,
    /// The paved copies, by vertex and key: which part, and which copy.
    paved: HashMap<(u32, Key), u32>,
}

impl Copies {
    fn new(mesh: &Mesh, arrangement: &Arrangement, rules: &[Rule]) -> Copies {
        let n = mesh.tri.positions.len();
        let mut c = Copies {
            carriageway: Part::default(),
            ballast: Part::default(),
            pavement: Part::default(),
            ground: Part::default(),
            in_ground: vec![NONE; n],
            paved: HashMap::new(),
        };
        let pos = &mesh.tri.positions;
        for (i, (t, &f)) in mesh.tri.indices.chunks_exact(3).zip(&mesh.of_face).enumerate() {
            let face = arrangement.face(f);
            if arrangement.in_partition(f) && !face.cuts() {
                for &v in t {
                    let id = c.ground.copy(&mut c.in_ground[v as usize], pos, v, (Surface::Ground, Rule::FREE));
                    c.ground.tri.indices.push(id);
                }
            }
            let Some(surface) = Surface::paved(face) else { continue };
            let key = (surface, rules[i]);
            for &v in t {
                let slot = c.paved.entry((v, key)).or_insert(NONE);
                let part = match surface {
                    Surface::Carriageway(_) => &mut c.carriageway,
                    Surface::Ballast(_) => &mut c.ballast,
                    _ => &mut c.pavement,
                };
                let id = part.copy(slot, pos, v, key);
                part.tri.indices.push(id);
            }
            match surface {
                Surface::Carriageway(_) => c.carriageway.face_key.push(key),
                Surface::Ballast(_) => c.ballast.face_key.push(key),
                _ => c.pavement.face_key.push(key),
            }
        }
        c
    }

    fn part(&self, surface: Surface) -> &Part {
        match surface {
            Surface::Ground => &self.ground,
            Surface::Carriageway(_) => &self.carriageway,
            Surface::Ballast(_) => &self.ballast,
            Surface::Near | Surface::Far => &self.pavement,
        }
    }

    /// The height of `v`'s copy under `key`, as the parts now hold it.
    fn height(&self, v: u32, key: Key) -> Option<f64> {
        let id = if key.0 == Surface::Ground { self.in_ground[v as usize] } else { *self.paved.get(&(v, key))? };
        (id != NONE).then(|| self.part(key.0).tri.positions[id as usize][2])
    }

    /// **Welds the copies of one vertex in one surface that agree.**
    ///
    /// Two triangles either side of the line where two roads' domains meet
    /// answer their shared corner each by its own road. Where the two agree
    /// within a kerb's rise — two legs of one junction, whose blend is
    /// continuous across the line between them — they are one vertex again,
    /// at their mean: a slope that small across a triangle is not a step. Where they do not, they stay two, and the edge between
    /// them is a declared step with a face on it ([`edge_faces`]).
    ///
    /// Returns how many copies went.
    fn weld(&mut self) -> usize {
        let mut groups: std::collections::BTreeMap<(u32, Surface), Vec<(Rule, u32)>> = Default::default();
        for (&(v, (surface, rule)), &id) in &self.paved {
            groups.entry((v, surface)).or_default().push((rule, id));
        }
        let mut into: [HashMap<u32, u32>; 3] = Default::default();
        let which = |s: Surface| match s {
            Surface::Carriageway(_) => 0,
            Surface::Ballast(_) => 1,
            _ => 2,
        };
        let mut gone = 0usize;
        for ((_, surface), mut copies) in groups {
            if copies.len() < 2 {
                continue;
            }
            let k = which(surface);
            let part = match k {
                0 => &mut self.carriageway,
                1 => &mut self.ballast,
                _ => &mut self.pavement,
            };
            let z = |id: u32, part: &Part| part.tri.positions[id as usize][2];
            copies.sort_by(|a, b| z(a.1, part).total_cmp(&z(b.1, part)).then(a.0.cmp(&b.0)));
            // Runs whose neighbours agree within a kerb's rise, and whose
            // spread does too, are one.
            let mut start = 0;
            while start < copies.len() {
                let mut end = start + 1;
                while end < copies.len() && z(copies[end].1, part) - z(copies[start].1, part) <= KERB_RISE_M {
                    end += 1;
                }
                if end - start > 1 {
                    let run = &copies[start..end];
                    let mean = run.iter().map(|c| z(c.1, part)).sum::<f64>() / run.len() as f64;
                    let keep = run.iter().map(|c| c.1).min().expect("a run");
                    part.tri.positions[keep as usize][2] = mean;
                    for c in run.iter().filter(|c| c.1 != keep) {
                        into[k].insert(c.1, keep);
                        gone += 1;
                    }
                }
                start = end;
            }
        }
        for (k, part) in [&mut self.carriageway, &mut self.ballast, &mut self.pavement].into_iter().enumerate() {
            for i in part.tri.indices.iter_mut() {
                if let Some(&j) = into[k].get(i) {
                    *i = j;
                }
            }
        }
        for (&(_, (surface, _)), id) in self.paved.iter_mut() {
            if let Some(&j) = into[which(surface)].get(id) {
                *id = j;
            }
        }
        gone
    }
}

/// One edge of the one mesh where the partition's triangles change surface
/// or rule: the vertices, in the winding of the triangle on `a`'s side, the
/// two faces and the two keys.
struct Boundary<'a> {
    u: u32,
    v: u32,
    a: &'a Face,
    b: &'a Face,
    ka: Option<Key>,
    kb: Option<Key>,
    /// Whether the far side is not there at all: a crack, or the rect's own
    /// edge.
    open: bool,
}

/// Every edge of the one mesh across which the partition's triangles change
/// what they are, found without building the whole mesh's adjacency.
///
/// Over the loop box the one mesh is 14 M triangles, and a table of all of
/// their edges is most of a gigabyte to find the few hundred thousand that
/// matter. An edge can only be a boundary if both its ends are vertices where
/// two keys meet, so only those are indexed.
fn boundaries<'a>(mesh: &Mesh, arrangement: &'a Arrangement, rules: &[Rule]) -> Vec<Boundary<'a>> {
    let key = |i: usize| {
        let face = arrangement.face(mesh.of_face[i]);
        (face.cuts(), Surface::paved(face).map(|s| (s, rules[i])))
    };
    let n = mesh.tri.positions.len();
    let mut first: Vec<Option<(bool, Option<Key>)>> = vec![None; n];
    let mut mixed = vec![false; n];
    let partition = |i: usize| arrangement.in_partition(mesh.of_face[i]);
    for (i, t) in mesh.tri.indices.chunks_exact(3).enumerate() {
        if !partition(i) {
            continue;
        }
        let k = key(i);
        for &v in t {
            match first[v as usize] {
                None => first[v as usize] = Some(k),
                Some(seen) if seen != k => mixed[v as usize] = true,
                Some(_) => {}
            }
        }
    }
    let mut edges: Vec<(u32, u32, u32)> = Vec::new();
    for (i, t) in mesh.tri.indices.chunks_exact(3).enumerate() {
        if !partition(i) {
            continue;
        }
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            if mixed[a as usize] && mixed[b as usize] {
                edges.push((a.min(b), a.max(b), i as u32));
            }
        }
    }
    edges.sort_unstable();
    let winding = |i: u32, a: u32, b: u32| -> bool {
        let t = &mesh.tri.indices[3 * i as usize..3 * i as usize + 3];
        (0..3).any(|k| t[k] == a && t[(k + 1) % 3] == b)
    };
    let mut out = Vec::new();
    for group in edges.chunk_by(|x, y| (x.0, x.1) == (y.0, y.1)) {
        let (lo, hi, t0) = group[0];
        let (u, v) = if winding(t0, lo, hi) { (lo, hi) } else { (hi, lo) };
        let a = arrangement.face(mesh.of_face[t0 as usize]);
        let ka = key(t0 as usize).1;
        match group {
            [_] => out.push(Boundary { u, v, a, b: a, ka, kb: ka, open: true }),
            [_, (.., t1)] => {
                if key(t0 as usize) != key(*t1 as usize) {
                    let b = arrangement.face(mesh.of_face[*t1 as usize]);
                    out.push(Boundary { u, v, a, b, ka, kb: key(*t1 as usize).1, open: false });
                }
            }
            // Three triangles on one edge is not a partition; `crack` in the
            // mesh step's line is where that shows.
            _ => {}
        }
    }
    out
}

/// Lifts the world's room onto the profile, and cuts it into the ground.
pub fn run(
    terrain: &Terrain,
    profiles: &Profiles,
    mesh: &Mesh,
    sheets: &Sheets,
    spans: &[Polyline2],
    arrangement: &Arrangement,
) -> (Bench, Summary) {
    // The tunnels' openings: the mouths of the portals the partition cut
    // open. The galleries are the arrangement's — it cuts a face for each.
    let (portals, _) = Portals::new(spans, profiles);
    // Where the paving is over a span rather than on the ground, as the
    // arrangement tagged it ([`crate::arrangement::over_spans`]): the sheets'
    // span paving, the walk a deck carries, and a centimetre of rim, so the
    // hole and the lift agree on it.
    let carried_m2 = arrangement.carried_m2;
    let over = poly::Indexed::new(&arrangement.over);
    let rail = |p: &&Profile| width::family(&p.class) == Family::Rail;
    // Two fields for what has no sheet: the roads' and the railways'. A
    // pavement is a road's cross-section and never a railway's. The railways'
    // whole field goes with it because a gallery's rim, which no paving
    // reaches, asks which of the two is nearer.
    let field = Field::grounded(profiles.profiles.iter().filter(|p| !rail(p)));
    let rails = Field::grounded(profiles.profiles.iter().filter(rail));
    // The two families that do have sheets take one field each, built from
    // that sheet's own axes and nothing else.
    let (car, car_over) = fields(sheets, Family::Carriageway, profiles);
    let (beds, beds_over) = fields(sheets, Family::Rail, profiles);
    let empty = Field::default();
    let natural = |q: Pt| height_at(terrain, q[0], q[1]);
    // The asphalt is the road: it takes the whole of its own height wherever
    // it reaches. Only the walk beside it is asked how far out it lies.
    let lift_of = |s: Surface| -> Lift {
        fn sheeted<'a>(fs: &'a [Field], k: u32, empty: &'a Field) -> &'a Field {
            fs.get(k as usize).unwrap_or(empty)
        }
        match s {
            Surface::Carriageway(k) => Lift {
                grounded: sheeted(&car, k, &empty),
                chords: Some(sheeted(&car_over, k, &empty)),
                over: Some(&over),
                rise: 0.0,
                walk: false,
                near: true,
            },
            Surface::Ballast(k) => Lift {
                grounded: sheeted(&beds, k, &empty),
                chords: Some(sheeted(&beds_over, k, &empty)),
                over: Some(&over),
                rise: 0.0,
                walk: false,
                near: true,
            },
            Surface::Near | Surface::Far => Lift {
                grounded: &field,
                chords: None,
                over: None,
                rise: KERB_RISE_M,
                walk: true,
                near: s == Surface::Near,
            },
            Surface::Ground => unreachable!("the ground is benched, not lifted"),
        }
    };

    // **Every paved triangle's rule, at its centroid.**
    let rules: Vec<Rule> = mesh
        .tri
        .indices
        .chunks_exact(3)
        .zip(&mesh.of_face)
        .map(|(t, &f)| {
            let Some(surface) = Surface::paved(arrangement.face(f)) else { return Rule::FREE };
            let c = [0usize, 1].map(|k| t.iter().map(|&v| mesh.tri.positions[v as usize][k]).sum::<f64>() / 3.0);
            lift_of(surface).rule(c, natural(c))
        })
        .collect();
    let mut copies = Copies::new(mesh, arrangement, &rules);
    let mut feet: [Vec<Option<Foot>>; 3] = Default::default();
    for (part, feet) in [&mut copies.carriageway, &mut copies.ballast, &mut copies.pavement].into_iter().zip(&mut feet) {
        for i in 0..part.tri.positions.len() {
            let (surface, rule) = part.key[i];
            let q = part.tri.positions[i];
            let (h, foot) = lift_of(surface).height([q[0], q[1]], part.natural[i], rule);
            part.tri.positions[i][2] = h;
            feet.push(foot);
        }
    }
    let welded = copies.weld();
    // Counted once welded, over the copies a triangle still names: a copy
    // merged into its twin is not a vertex of the world.
    let mut stats = Stats::default();
    for (part, feet) in [&copies.carriageway, &copies.ballast, &copies.pavement].into_iter().zip(&feet) {
        let mut used = vec![false; part.tri.positions.len()];
        for &i in &part.tri.indices {
            used[i as usize] = true;
        }
        for i in (0..used.len()).filter(|&i| used[i]) {
            let (surface, rule) = part.key[i];
            lift_of(surface).account(&mut stats, rule, part.tri.positions[i][2], part.natural[i], feet[i]);
        }
    }
    let at = |q: Pt, (surface, rule): Key| lift_of(surface).height(q, natural(q), rule).0;
    for part in [&copies.carriageway, &copies.ballast, &copies.pavement] {
        steps_of(part, &at, &mut stats);
    }
    let steps = std::mem::take(&mut stats.at);

    // **The room's height at a vertex is its lowest paved copy's**, read by
    // index. Where the asphalt and the pavement both reach a vertex — the
    // end of a kerb — the lower is the ground's, so the ground meets the
    // asphalt rather than standing a kerb over it.
    let n = mesh.tri.positions.len();
    let mut room_at = vec![f64::INFINITY; n];
    for part in [&copies.carriageway, &copies.pavement, &copies.ballast] {
        for (i, &v) in part.of.iter().enumerate() {
            room_at[v as usize] = room_at[v as usize].min(part.tri.positions[i][2]);
        }
    }
    // How many paved copies each vertex has once welded: where there are two
    // — two surfaces, or two rules of one — the seam between them is the edge
    // rule's, not a contact with the ground.
    let mut paved_by = vec![0u8; n];
    {
        let mut seen: std::collections::HashSet<(u32, Surface, u32)> = Default::default();
        for (&(v, (surface, _)), &id) in &copies.paved {
            if seen.insert((v, surface, id)) {
                paved_by[v as usize] = paved_by[v as usize].saturating_add(1);
            }
        }
    }
    let plan = |v: u32| {
        let q = mesh.tri.positions[v as usize];
        [q[0], q[1]]
    };
    // A vertex no paving reaches has no room height of its own: a gallery's
    // rim, whose footprint the arrangement cuts but nothing paves. The nearer
    // field's cross-section answers there, as it always has.
    let room = |v: u32| {
        let h = room_at[v as usize];
        if h.is_finite() {
            return h;
        }
        let q = plan(v);
        match (field.at(q), rails.at(q)) {
            (Some(r), Some(t)) if t.d < r.d => t.batter(t.h, natural(q)),
            (Some(r), _) => r.batter(r.h + KERB_RISE_M, natural(q)),
            (None, Some(t)) => t.batter(t.h, natural(q)),
            (None, None) => natural(q),
        }
    };

    // **The outline is the one mesh's own edges** between a face that cuts
    // and one that does not, each in the winding of the cutting side so the
    // room lies on its left. Both meshes that meet there are the one mesh, so
    // these edges are already split at every lattice crossing.
    let bounds = boundaries(mesh, arrangement, &rules);
    let mut outline: Vec<(u32, u32)> = Vec::new();
    // The key on the cutting side of each outline edge.
    let mut cutting: Vec<Option<Key>> = Vec::new();
    let mut unmet = 0usize;
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for q in &mesh.tri.positions {
        for k in 0..2 {
            lo[k] = lo[k].min(q[k]);
            hi[k] = hi[k].max(q[k]);
        }
    }
    let on_border = |v: u32| {
        let q = mesh.tri.positions[v as usize];
        (0..2).any(|k| (q[k] - lo[k]).abs() <= poly::GRID_M || (q[k] - hi[k]).abs() <= poly::GRID_M)
    };
    for e in &bounds {
        if e.open {
            // A paved rim with nothing beyond it, away from the rect's edge:
            // a crack in the one mesh, and the one thing no face can close.
            if e.a.cuts() && !(on_border(e.u) && on_border(e.v)) {
                unmet += 1;
            }
            continue;
        }
        match (e.a.cuts(), e.b.cuts()) {
            (true, false) => {
                outline.push((e.u, e.v));
                cutting.push(e.ka);
            }
            (false, true) => {
                outline.push((e.v, e.u));
                cutting.push(e.kb);
            }
            _ => {}
        }
    }

    // **The room's height along an outline edge is its cutting side's own
    // copy**, not the lowest copy at the vertex: where two rules of one
    // surface split a vertex, the lower belongs to the other side's edge.
    let top: Vec<[f64; 2]> = outline
        .iter()
        .zip(&cutting)
        .map(|(&(u, v), key)| [u, v].map(|w| key.and_then(|k| copies.height(w, k)).unwrap_or_else(|| room(w))))
        .collect();
    let segments: Vec<(Pt, Pt, [f64; 2], [f64; 2])> = outline
        .iter()
        .zip(&top)
        .map(|(&(u, v), t)| {
            let (a, b) = (plan(u), plan(v));
            (a, b, [t[0], natural(a)], [t[1], natural(b)])
        })
        .collect();
    let ground = Ground::of_edges(&segments, &portals);
    // **An outline vertex takes its own pin**, exactly: the blend is for the
    // ground between the outline's segments, and at a vertex it would hear a
    // segment round the corner and stand a few centimetres off the paving it
    // meets. Where two outline edges pin one vertex differently — a split —
    // the lower is the ground's, as the room's own height is.
    let mut pin_at: HashMap<u32, f64> = HashMap::new();
    for ((&(u, v), t), seg) in outline.iter().zip(&top).zip(&segments) {
        let mouth = portals.open(seg.0) && portals.open(seg.1);
        for (w, room_h, nat) in [(u, t[0], seg.2[1]), (v, t[1], seg.3[1])] {
            let e = if mouth { 0.0 } else { (room_h - nat).clamp(-MAX_BENCH_FACE_M, MAX_BENCH_FACE_M) };
            pin_at.entry(w).and_modify(|x| *x = x.min(e)).or_insert(e);
        }
    }
    // The ground's copies at the engineered ground. The one mesh was built
    // at the natural ground, which is exactly [`Ground::at`]'s second
    // argument.
    for (q, &v) in copies.ground.tri.positions.iter_mut().zip(&copies.ground.of) {
        q[2] = match pin_at.get(&v) {
            Some(e) => q[2] + e,
            None => ground.at([q[0], q[1]], q[2]),
        };
    }

    // How the paving and the ground meet at the outline: whether every
    // outline vertex has a paved copy on its cutting side (`seam`), and how
    // far apart the two stand where a batter runs (`contact`). Both are
    // constructions now rather than searches, so both are checks.
    let (mut asked, mut missed, mut contact) = (0usize, 0usize, 0.0f64);
    for (&(u, v), key) in outline.iter().zip(&cutting) {
        for w in [u, v] {
            let q = plan(w);
            let Some(key) = *key else { continue };
            if portals.open(q) || over.contains(q) {
                continue;
            }
            asked += 1;
            let Some(h) = copies.height(w, key) else {
                missed += 1;
                continue;
            };
            if paved_by[w as usize] == 1 && (natural(q) - h).abs() <= MAX_BENCH_FACE_M {
                if let Some(gh) = copies.height(w, (Surface::Ground, Rule::FREE)) {
                    contact = contact.max((gh - h).abs());
                }
            }
        }
    }
    let mut earth = Earth::new(&outline, &room, &|v| natural(plan(v)), &ground, terrain);
    earth.asked = asked;
    earth.unseamed = missed;
    earth.unmet = unmet;
    earth.rim = asked;
    earth.contact = contact;
    earth.welded = welded;
    let g = &copies.ground;
    earth.triangles = g.tri.indices.len() / 3;
    earth.vertices = g.tri.positions.len();
    // Only a triangle the bench moved can stand off the ground it benched:
    // one whose three vertices it left at the natural ground is the
    // terrain's own, and asking it cost 7.5 s of the loop box's bench.
    let moved = |v: u32| g.tri.positions[v as usize][2] != g.natural[v as usize];
    earth.off = g
        .tri
        .indices
        .chunks_exact(3)
        .filter(|t| t.iter().any(|&v| moved(v)))
        .map(|t| {
            let c3 = [0usize, 1].map(|k| t.iter().map(|&v| g.tri.positions[v as usize][k]).sum::<f64>() / 3.0);
            let plane = t.iter().map(|&v| g.tri.positions[v as usize][2]).sum::<f64>() / 3.0;
            (plane - ground.at(c3, natural(c3))).abs()
        })
        .fold(0.0, f64::max);

    // **How much of the free walk the engineered ground would move.** A
    // vertex no road answered for keeps the raw DEM while the ground around
    // it has been benched — "a footpath leaving a street does not run up the
    // batter". The engineered ground reaches [`EARTHWORK_BATTER`] × one face
    // (7.5 m) from the outline and is the natural ground beyond, so the count
    // is exact: ask it, and see where it answers something else.
    let (mut reached, mut reach_m) = (0usize, 0.0f64);
    for (q, &nat) in copies.pavement.tri.positions.iter().zip(&copies.pavement.natural) {
        if (q[2] - nat).abs() > 1e-9 {
            continue;
        }
        let engineered = ground.at([q[0], q[1]], nat);
        if (engineered - nat).abs() > 1e-6 {
            reached += 1;
            reach_m = reach_m.max((engineered - nat).abs());
        }
    }

    let ground_h = |v: u32| {
        copies.height(v, (Surface::Ground, Rule::FREE)).unwrap_or_else(|| ground.at(plan(v), natural(plan(v))))
    };
    let (wall, wall_m2) = wall(&outline, &top, &mesh.tri.positions, &ground_h, &portals);
    // **One edge rule for every boundary inside the paving** (§3.3), over
    // the one mesh's own edges and read by index — between two surfaces, and
    // between two rules of one surface that did not weld.
    let (kerb, edges) = edge_faces(&bounds, &mesh.tri.positions, &|v, k| copies.height(v, k));
    earth.kerb_m2 = edges.m2;
    earth.wall_m2 = wall_m2;

    // **Where the steps are.** §3.3 says an edge is welded or spanned by a
    // quad, "so `step` has nothing left to count". This is the share of the
    // steps that lie on an arrangement edge at all.
    earth.on_edge = {
        const NEAR_M: f64 = 0.05;
        let mut index: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
        for (i, e) in arrangement.edges.iter().enumerate() {
            let bx = [
                e.a[0].min(e.b[0]) - NEAR_M,
                e.a[1].min(e.b[1]) - NEAR_M,
                e.a[0].max(e.b[0]) + NEAR_M,
                e.a[1].max(e.b[1]) + NEAR_M,
            ];
            for cell in poly::cells_over(bx, poly::CELL_M) {
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

    let axes = field.len() + rails.len();
    let Copies { carriageway, ballast, pavement, ground: earthwork, .. } = copies;
    let bench = Bench {
        carriageway: compact(carriageway.tri),
        pavement: compact(pavement.tri),
        ballast: compact(ballast.tri),
        ground: earthwork.tri,
        wall,
        kerb,
        steps,
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
        // Edges steeper than a step whose both ends are on the raw DEM: the
        // terrain's own steepness under a draped pavement — a retaining wall
        // the DEM images — and not a discontinuity the lift made. They were
        // 10 028 of the loop box's 18 408 `step` before they were told apart.
        .with("steep", stats.steep)
        .with("dem_steep", stats.dem_steep)
        .with("welded", earth.welded)
        .with("ground", format!("{}/{}", earth.triangles, earth.vertices))
        .with_share("seam", earth.unseamed, earth.asked)
        .with_share("unmet", earth.unmet, earth.rim)
        .with("contact", format!("{:.2}", earth.contact))
        .with_share("step_on_edge", earth.on_edge, stats.steps)
        .with_share("walled", earth.walled, earth.outline)
        .with("wall", format!("{:.1}", earth.wall))
        .with_m2("wall_m2", earth.wall_m2)
        .with_m2("kerb_m2", earth.kerb_m2)
        .with("kerb_max", format!("{:.2}", edges.max))
        .with_m2("sheet_m2", edges.sheet_m2)
        .with_m2("split_m2", edges.split_m2)
        .with_share("touched", earth.touched, earth.lattice)
        .with("off", format!("{:.1e}", earth.off))
        .with("flown", stats.flown)
        .with_part("free", stats.free, stats.draped)
        .with("regrade", format!("{reached} to {reach_m:.2}"))
        .with_m2("carried", carried_m2)
        .with_residual(drawn_residual(terrain, &bench.ground));
    (bench, summary)
}

/// Counts the steps of one part: edges its heights **jump** across.
///
/// An edge over [`KERB_RISE_M`] and steeper than [`STEP_GRADE`] is asked
/// what its own rule gives its midpoint. A rule that is continuous along the
/// edge puts the midpoint near the mean of its ends, however steep the edge;
/// a jump puts it on one side, half the rise off the mean. Only the second
/// is a step. The first is `steep`: a street following a 150 % flank, a
/// junction warping between two legs, or — `dem_steep`, both ends on the
/// raw DEM — a pavement draped over a retaining wall the DEM images.
///
/// It used to count every edge over the grade and call it a discontinuity,
/// which it was only while a vertex could be answered by a different rule
/// from its neighbour; 10 028 of the loop box's 18 408 were the DEM's own
/// steepness.
fn steps_of(part: &Part, at: &dyn Fn(Pt, Key) -> f64, stats: &mut Stats) {
    let tri = &part.tri;
    let mut seen: HashSet<(u32, u32)> = HashSet::new();
    for (t, &key) in tri.indices.chunks_exact(3).zip(&part.face_key) {
        for e in 0..3 {
            let (a, b) = (t[e], t[(e + 1) % 3]);
            if !seen.insert((a.min(b), a.max(b))) {
                continue;
            }
            stats.edges += 1;
            let (p, q) = (tri.positions[a as usize], tri.positions[b as usize]);
            let dh = (q[2] - p[2]).abs();
            let len = (q[0] - p[0]).hypot(q[1] - p[1]);
            if !(dh > KERB_RISE_M + STEP_SLACK_M && dh > STEP_GRADE * len) {
                continue;
            }
            if p[2] == part.natural[a as usize] && q[2] == part.natural[b as usize] {
                stats.dem_steep += 1;
                continue;
            }
            let mid = [(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0];
            if (at(mid, key) - (p[2] + q[2]) / 2.0).abs() <= dh / 4.0 {
                stats.steep += 1;
                continue;
            }
            stats.steps += 1;
            stats.worst = stats.worst.max(dh);
            stats.at.push(mid);
        }
    }
}

/// `tri` with the positions no triangle names dropped: a copy welded into
/// another leaves its slot behind, and a glTF's vertex buffer pays for every
/// slot whether a triangle names it or not.
fn compact(mut tri: Tri) -> Tri {
    let mut remap = vec![NONE; tri.positions.len()];
    let mut positions = Vec::with_capacity(tri.positions.len());
    for i in tri.indices.iter_mut() {
        if remap[*i as usize] == NONE {
            remap[*i as usize] = positions.len() as u32;
            positions.push(tri.positions[*i as usize]);
        }
        *i = remap[*i as usize];
    }
    tri.positions = positions;
    tri
}

/// Below this height, in metres, a step between two surfaces is rounding
/// and not a face.
const WALL_MIN_M: f64 = 1e-3;

/// The face that closes the step between the room's edge and the ground
/// outside it, and the area of it.
///
/// The two meet exactly wherever a batter could run — the ground takes the
/// room's own height at the outline. Where the step is more than one face
/// tall the batter is refused ([`Ground::at`] hands back the natural ground
/// rather than manufacture a slope no hillside has), and the face closes the
/// gap: without it, **a hole you could see the world through**, which is
/// what invariant 9 forbids.
///
/// It is swept along the one mesh's own outline edges, directed with the
/// room on their left, and its rails are the heights of the two copies of
/// each vertex — the room's and the ground's — so the closure is exact by
/// index: no T-junction, no hairline. A segment whose ends both agree to
/// [`WALL_MIN_M`] is not drawn, which is most of them. Outward at a kerb,
/// into the courtyard at a hole, because the winding says which is which.
fn wall(
    outline: &[(u32, u32)],
    top: &[[f64; 2]],
    positions: &[[f64; 3]],
    ground: &dyn Fn(u32) -> f64,
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
    let rail = |v: u32, r: f64| {
        let q = [positions[v as usize][0], positions[v as usize][1]];
        let g = ground(v);
        match portals.section(q, r) {
            Some((_, roof)) if g > roof => (roof, g),
            Some((floor, _)) if g < floor => (floor, g),
            Some(_) => (g, g),
            None => (r, g),
        }
    };
    for (&(u, v), t) in outline.iter().zip(top) {
        let (a, b) = (positions[u as usize], positions[v as usize]);
        let run = (b[0] - a[0]).hypot(b[1] - a[1]);
        if run <= f64::EPSILON {
            continue;
        }
        let (p0, p1) = (rail(u, t[0]), rail(v, t[1]));
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
        // The quad is two triangles, and where the face tapers to nothing at
        // one end — the rails meeting — one of them is a line. It is left
        // out rather than drawn flat.
        if d0 > f64::EPSILON {
            tri.indices.extend_from_slice(&[base, base + 1, base + 2]);
        }
        if d1 > f64::EPSILON {
            tri.indices.extend_from_slice(&[base, base + 2, base + 3]);
        }
        m2 += (d0 + d1) / 2.0 * run;
    }
    (tri, m2)
}

/// **The edge rule** (`data/plans/one-ground-2026-09-16.md` §3.3): every
/// boundary between two paved surfaces is either *welded* — its two sides
/// answer with one height — or *split*, and then the quad between them is
/// drawn, always.
///
/// One rule over the one mesh's own edges, read by index: `height(v, face)`
/// is the height of `v`'s copy in the surface `face` is drawn in. It used to
/// run over the arrangement's edges, one quad per edge with its heights
/// looked up by position at the two ends — and an arrangement edge is not a
/// mesh edge: both meshes subdivide it at every lattice crossing, so each
/// quad met both rims in T-junctions. Here a quad is one mesh edge.
///
/// The ground stays `wall`'s: it is the one boundary whose far side is not a
/// face of the paving.
fn edge_faces(
    bounds: &[Boundary],
    positions: &[[f64; 3]],
    height: &dyn Fn(u32, Key) -> Option<f64>,
) -> (Tri, EdgeFaces) {
    let mut tri = Tri::default();
    let mut out = EdgeFaces::default();
    for e in bounds.iter().filter(|e| !e.open) {
        let (Some(ka), Some(kb)) = (e.ka, e.kb) else { continue };
        if ka == kb {
            continue;
        }
        let (Some(aa), Some(ab), Some(ba), Some(bb)) = (height(e.u, ka), height(e.v, ka), height(e.u, kb), height(e.v, kb))
        else {
            continue;
        };
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
        // over a span too.
        let slab = crate::structure::DECK_THICKNESS_M;
        if (e.a.spanned || e.b.spanned) && ((aa - ba).abs() > slab || (ab - bb).abs() > slab) {
            continue;
        }
        // Faced toward the lower side: `u → v` runs with `a` on its left, so
        // the edge is walked the other way when `b` is the higher.
        let (p, q) = (positions[e.u as usize], positions[e.v as usize]);
        let (p, q, ph, pl, qh, ql) = if aa + ab >= ba + bb {
            (p, q, aa.max(ba), aa.min(ba), ab.max(bb), ab.min(bb))
        } else {
            (q, p, ab.max(bb), ab.min(bb), aa.max(ba), aa.min(ba))
        };
        // Where the two sides meet at one end the face tapers to it, and the
        // half that would be a line is left out rather than drawn flat.
        let base = tri.positions.len() as u32;
        tri.positions.extend_from_slice(&[[p[0], p[1], ph], [p[0], p[1], pl], [q[0], q[1], ql], [q[0], q[1], qh]]);
        if ph - pl > f64::EPSILON {
            tri.indices.extend_from_slice(&[base, base + 1, base + 2]);
        }
        if qh - ql > f64::EPSILON {
            tri.indices.extend_from_slice(&[base, base + 2, base + 3]);
        }
        let m2 = (ph - pl + qh - ql) / 2.0 * (q[0] - p[0]).hypot(q[1] - p[1]);
        out.m2 += m2;
        out.max = out.max.max(ph - pl).max(qh - ql);
        if ka.0 == kb.0 {
            out.split_m2 += m2;
        } else if e.a.material == e.b.material && e.a.sheet != e.b.sheet {
            out.sheet_m2 += m2;
        }
    }
    (tri, out)
}

/// What the edge rule drew.
#[derive(Debug, Default, Clone, Copy)]
struct EdgeFaces {
    /// The area of every face, in square metres.
    m2: f64,
    /// The tallest face, in metres: a kerb is 0.12, and a face much taller
    /// than a slab is a curtain the slab rule did not catch.
    max: f64,
    /// The area drawn between two sheets of one family — two carriageways
    /// that meet at different heights. The position lookup this replaced
    /// read both from one map and so never drew these at all.
    sheet_m2: f64,
    /// The area drawn inside one surface, between two rules that did not
    /// weld: the retaining face where two roads' domains meet at different
    /// heights, or where the far pavement stops standing on its face and
    /// drapes. A step there used to be a stretched triangle nothing closed.
    split_m2: f64,
}

/// The height the lifted meshes gave every one of their vertices, keyed by
/// position at the kernel's grid, the lowest where two reach one position.
///
/// **The bench does not use this any more** — it reads a vertex's copies by
/// index ([`Copies`]). What still does is the structure step, which asks for
/// the paving's height at points of a deck's own outline, a region built
/// apart from the mesh; that is a lookup by position between two constructions
/// and keeps the same weakness the bench's used to have.
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

/// Tolerance, in metres, at which a position lookup takes two points to be
/// one: the kernel's grid, since the regions a caller asks about came out of
/// it. Not [`crate::mesh::WELD_M`], which welds a hundred times finer within one
/// mesh. (It used to be said that a point "through one more boolean" lands
/// half a grid away; [`crate::poly`] pins its adapter, so it cannot. Two
/// *different regions* is what a lookup like this has to bridge.)
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
    /// Outline vertices, counted once per outline edge end.
    pub outline: usize,
    /// Outline vertices away from a mouth, a gallery or a span, and those of
    /// them with no copy on the paving's side: the seam is a construction
    /// now, so this is a check that it still is one.
    pub asked: usize,
    pub unseamed: usize,
    /// Steps lying on an arrangement edge: what §3.3's rule can reach.
    pub on_edge: usize,
    /// The paving's rim, and the edges of it with nothing on the far side
    /// away from the rect's border: a crack, and the one thing a closing
    /// face cannot mend.
    pub rim: usize,
    pub unmet: usize,
    /// The largest step, in metres, between a paved copy of an outline
    /// vertex and its ground copy, where a batter runs.
    pub contact: f64,
    /// Outline vertices standing more than one face from the ground, where
    /// the batter takes one face and a wall the rest, and the tallest of
    /// those walls in metres.
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
    /// The area of the closing face, in square metres.
    pub wall_m2: f64,
    /// The area of the faces the edge rule draws between paved surfaces.
    pub kerb_m2: f64,
    /// Copies of one vertex under two rules that agreed and were welded.
    pub welded: usize,
}

impl Earth {
    /// Measures the ground against the room it was cut for, over the
    /// outline's own edges.
    fn new(
        outline: &[(u32, u32)],
        room: &dyn Fn(u32) -> f64,
        natural: &dyn Fn(u32) -> f64,
        ground: &Ground,
        t: &Terrain,
    ) -> Earth {
        let mut e = Earth::default();
        for &(u, _) in outline {
            e.outline += 1;
            let (r, n) = (room(u), natural(u));
            // A walled vertex has no contact to measure: the ground there is
            // the natural ground and the wall between them is the answer, so
            // it is counted rather than averaged in.
            if (n - r).abs() > MAX_BENCH_FACE_M {
                e.walled += 1;
                // The batter takes one face of the drop; the wall the rest.
                e.wall = e.wall.max((n - r).abs() - MAX_BENCH_FACE_M);
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

/// One height field per sheet of `family`, in the sheets' own order — the
/// index a face's `sheet` carries, so a copy's sheet id indexes this
/// directly.
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
        let batter = |d: f64, ground: f64| Foot { h: 0.0, d, half_w: 2.75, axis: 0, s: 0.0 }.batter(0.0, ground);
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
        assert!((chord.at([88.0, 6.0]).expect("the chord reaches it").h - 407.0).abs() < 1e-9);
        // The mask lies about it — a sliver of a deck that is not there.
        let over = poly::Indexed::new(&vec![poly::rect(87.0, 5.0, 90.0, 7.0)]);

        let lift = Lift { grounded: &ground, chords: Some(&chord), over: Some(&over), rise: 0.0, walk: false, near: true };
        let rule = lift.rule([88.0, 6.0], 0.0);
        let (h, foot) = lift.height([88.0, 6.0], 0.0, rule);
        assert!((h - 405.6).abs() < 1e-9, "the sliver handed the vertex the chord's end height: {h}");
        // And the earthwork under it is the ground's to owe, not a deck's.
        let mut stats = Stats::default();
        lift.account(&mut stats, rule, h, 0.0, foot);
        assert!(!rule.chord && stats.fill > 0.0, "the sliver excused the fill under it too");
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
        let pavement = crate::mesh::view(m, w.arrangement.as_ref().unwrap(), |f| f.material == Material::Pavement);
        assert!(pavement.positions.iter().any(|p| p[2] > 401.5), "the ground under it climbs");
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
        // And it is **drawn**: the band's triangles on the face and those
        // that drape take two rules, and the edge between them is split with
        // a face on it rather than stretched across — so it is a wall and
        // not a step.
        assert_eq!(s.num("step"), 0.0, "{s}");
        assert!(s.num("split_m2") > 0.0 && s.num("kerb_max") < 4.0, "{s}");
        // The wall stands where the hill outruns the face, not at the kerb:
        // every face taller than a kerb is out beyond the plateau.
        for q in b.kerb.positions.chunks_exact(4) {
            if (q[0][2] - q[1][2]).abs().max((q[3][2] - q[2][2]).abs()) > 2.0 * KERB_RISE_M {
                assert!(q[0][1] > 2.75 + ROOM_REACH_M, "a wall at the kerb: {q:?}");
            }
        }
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

    /// A polygon outline as the one mesh hands it to the bench: every ring
    /// split where it crosses the lattice, as directed edges over one list
    /// of positions.
    fn outline_of(outline: &Shapes, grid: &crate::grid::Grid) -> (Vec<[f64; 3]>, Vec<(u32, u32)>) {
        let (mut pos, mut edges) = (Vec::new(), Vec::new());
        for ring in outline.iter().flatten() {
            let start = pos.len() as u32;
            for i in 0..ring.len() {
                let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
                pos.push([a[0], a[1], 0.0]);
                let mut cut = crate::drape::split(grid, a, b);
                cut.pop();
                pos.extend(cut.into_iter().map(|q| [q[0], q[1], 0.0]));
            }
            let end = pos.len() as u32;
            edges.extend((start..end).map(|k| (k, if k + 1 == end { start } else { k + 1 })));
        }
        (pos, edges)
    }

    /// The ground benched to `outline` at `room` over `natural`.
    fn ground_on(
        outline: &Shapes,
        grid: &crate::grid::Grid,
        room: &dyn Fn(Pt) -> f64,
        natural: &dyn Fn(Pt) -> f64,
        portals: &Portals,
    ) -> Ground {
        let (pos, edges) = outline_of(outline, grid);
        let xy = |v: u32| [pos[v as usize][0], pos[v as usize][1]];
        let segs: Vec<_> = edges
            .iter()
            .map(|&(u, v)| (xy(u), xy(v), [room(xy(u)), natural(xy(u))], [room(xy(v)), natural(xy(v))]))
            .collect();
        Ground::of_edges(&segs, portals)
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
        (ground_on(&outline, &terrain.grid, &room, &natural, &Portals::default()), natural)
    }

    #[test]
    fn the_batter_is_one_in_two_and_a_half_and_stops_at_the_ground() {
        // The plan's specimen, with a ruler on it. A 5.5 m road along the
        // contour of a 30 % slope is level at 400 m; the ground at its uphill
        // kerb stands 0.825 m over it and at its downhill kerb 0.825 m under.
        // The residual — cut there, fill here — falls to nothing at 1 in 2.5
        // of the natural ground: 0.4 m per metre out, so it meets the ground
        // 2.0625 m from either kerb.
        let (w, s) = world("ramp?grade=0.3&bearing=0&radius=100000", "net:straight?len=200", None);
        let (g, natural) = grounds(&w);
        let at = |y: f64| g.at([0.0, y], natural([0.0, y]));
        let toe = 0.825 * EARTHWORK_BATTER;
        for d in [0.25, 0.5, 1.0, 2.0] {
            let (up, down) = (at(2.75 + d), at(-2.75 - d));
            let e = 0.825 - d / EARTHWORK_BATTER;
            assert!((up - (natural([0.0, 2.75 + d]) - e)).abs() < 1e-9, "cut at {d}: {up}");
            assert!((down - (natural([0.0, -2.75 - d]) + e)).abs() < 1e-9, "fill at {d}: {down}");
        }
        // It daylights, and there is no lip: the rule this replaced ran its
        // face at an absolute 1 in 2.5, which up this hill closed only 0.1 m
        // per metre, never met the ground within the 7.5 m a face may run,
        // and left 0.075 m standing where it stopped.
        for d in [toe + 1e-6, toe + 1.0, EARTHWORK_BATTER * MAX_BENCH_FACE_M, 100.0] {
            assert_eq!(at(2.75 + d), natural([0.0, 2.75 + d]), "not daylighted at {d}");
            assert_eq!(at(-2.75 - d), natural([0.0, -2.75 - d]), "not daylighted at -{d}");
        }
        // And the ground meets the room at the kerb, exactly.
        assert!(s.num("contact") < 1e-9, "{s}");
        assert_eq!(s.num("walled"), 0.0, "{s}");
        assert_eq!(s.num("wall_m2"), 0.0, "{s}");
    }

    #[test]
    fn a_gentler_hill_daylights_where_the_plan_says() {
        // On a 10 % ramp the same road is cut 0.275 m at its uphill kerb, and
        // the residual closes 0.4 m of it per metre: 0.6875 m of batter.
        let (w, _) = world("ramp?grade=0.1&bearing=0&radius=100000", "net:straight?len=200", None);
        let (g, natural) = grounds(&w);
        let at = |y: f64| g.at([0.0, y], natural([0.0, y]));
        let toe = 0.275 * EARTHWORK_BATTER;
        assert!((toe - 0.6875).abs() < 1e-9, "{toe}");
        let half = [0.0, 2.75 + toe / 2.0];
        assert!((at(half[1]) - (natural(half) - 0.275 / 2.0)).abs() < 1e-9);
        for d in [toe + 1e-6, toe + 1.0, 20.0] {
            let (p, n) = ([0.0, 2.75 + d], natural([0.0, 2.75 + d]));
            assert_eq!(g.at(p, n), n, "daylighted at {d}");
        }
    }

    /// **A drop past one face is a face of batter and a wall for the rest.**
    /// The rule this replaced refused the batter outright once the room stood
    /// more than one face off the ground, so the ground beside a walled
    /// stretch was the natural ground and beside the next battered one the
    /// batter — a step in the ground on the line between them. Clamped, the
    /// pin moves with the drop and the ground with it.
    #[test]
    fn a_deep_cut_is_battered_one_face_and_walled_for_the_rest() {
        let edges = [([-50.0, 0.0], [50.0, 0.0], [400.0, 405.0], [400.0, 405.0])];
        // The room on the left of the edge, the ground to the south of it
        // standing 5 m over the road.
        let g = Ground::of_edges(&edges, &Portals::default());
        let natural = |_: Pt| 405.0;
        assert!((g.at([0.0, -1e-9], natural([0.0, 0.0])) - 402.0).abs() < 1e-6, "one face of cut at the edge");
        assert!((g.at([0.0, -3.75], 405.0) - 403.5).abs() < 1e-9, "half way down the batter");
        assert_eq!(g.at([0.0, -7.5 - 1e-6], 405.0), 405.0, "daylighted after one face's run");
        // Across the room, north of the edge, the batter does not reach.
        assert_eq!(g.at([0.0, 3.0], 405.0), 405.0);
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
        // so a batter takes one face of it and a wall the other two metres.
        // That is a step between the room's edge and the ground, and until it
        // was walled it was a hole you could see the world through (I9).
        //
        // The specimen is a cliff rather than an overpass because an
        // overpass no longer walls: past `DECK_STANDOFF_M` its approach is a
        // deck, which is what R2 is for. A wall is what the ground owes where
        // the road is *not* a structure, and a cliff is that.
        let (w, s) = world("step?rise=10&width=0&bearing=0", "net:straight?len=200", None);
        assert!(s.num("walled") > 0.0, "{s}");
        assert!(s.num("wall_m2") > 300.0, "{s}");
        assert!((s.num("wall") - 2.0).abs() < 0.1, "the wall is the drop less one face: {s}");
        let b = bench(&w);
        assert!(!b.wall.indices.is_empty());
        // It stands between the two surfaces it closes, and no further: the
        // ground below at one end, the room's own edge at the other.
        let z: Vec<f64> = b.wall.positions.iter().map(|p| p[2]).collect();
        let lo = z.iter().cloned().fold(f64::INFINITY, f64::min);
        let hi = z.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(hi - lo > 1.9, "the wall closes nothing: {lo}..{hi}");
        for q in b.wall.positions.chunks_exact(4) {
            let tall = (q[0][2] - q[1][2]).abs().max((q[3][2] - q[2][2]).abs());
            assert!(tall <= 2.0 + 1e-6, "a wall taller than the drop past one face: {q:?}");
        }
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
        let ground = ground_on(&outline, &grid, &room, &natural, &portals);
        let (pos, edges) = outline_of(&outline, &grid);
        let xy = |v: u32| [pos[v as usize][0], pos[v as usize][1]];
        let top: Vec<[f64; 2]> = edges.iter().map(|&(u, v)| [room(xy(u)), room(xy(v))]).collect();
        let (tri, m2) = wall(&edges, &top, &pos, &|v| ground.at(xy(v), natural(xy(v))), &portals);
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
