//! Step 11: the bench — the room holds its height.
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

use std::collections::HashMap;
use std::collections::HashSet;

use crate::poly::{self, Pt, Shapes};
use crate::step::Summary;
use crate::terrain::height_at;
use crate::world::{Bench, Kind, Profile, Terrain, Tri, World};
use crate::mesh;

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

/// The room's height field: the solved profile of every carriageway axis
/// on the ground, indexed for the nearest-axis query every vertex makes.
///
/// The nearest axis is the road whose cross-section the point rides. Inside
/// a piece's own ribbon that is the piece's own axis, since no other axis
/// comes within its half-width without their ribbons overlapping; in the
/// pavement and the kerb returns it is the nearest road, which is the one
/// the pavement belongs to.
#[derive(Debug, Default)]
pub struct Field {
    at: Nearest,
    /// Per segment, the solved heights of its two ends and the half-width
    /// of the road it belongs to.
    seg: Vec<(f64, f64, f64)>,
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
        let (c0, r0) = poly::cell_of(p, CELL_M);
        let (bc0, br0, bc1, br1) = self.span?;
        let rings = (c0 - bc0).abs().max((bc1 - c0).abs()).max((r0 - br0).abs()).max((br1 - r0).abs());
        let mut best: Option<(usize, f64, f64)> = None;
        for k in 0..=rings {
            for (c, r) in ring(c0, r0, k) {
                let Some(ids) = self.cells.get(&(c, r)) else {
                    continue;
                };
                for &i in ids {
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
    /// cheap and continuous: at `d` metres out it may have closed
    /// `(d − reach) / 2.5` of the difference and no more, so it is the
    /// ground wherever it has daylighted and the room's height wherever
    /// it has not left the reach.
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
}

impl Field {
    /// The field of `profiles`: the ground pieces' stations, in order.
    pub fn new(profiles: &[Profile]) -> Field {
        let mut f = Field::default();
        for p in profiles.iter().filter(|p| p.mapped == Kind::Ground) {
            let half_w = p.width_m / 2.0;
            for w in p.stations.windows(2) {
                f.push(w[0].p, w[1].p, w[0].h, w[1].h, half_w);
            }
            if p.stations.len() == 1 {
                let st = p.stations[0];
                f.push(st.p, st.p, st.h, st.h, half_w);
            }
        }
        f
    }

    fn push(&mut self, a: Pt, b: Pt, ha: f64, hb: f64, half_w: f64) {
        self.at.push(a, b);
        self.seg.push((ha, hb, half_w));
    }

    /// How many axis pieces the field holds.
    pub fn len(&self) -> usize {
        self.seg.len()
    }

    pub fn is_empty(&self) -> bool {
        self.seg.is_empty()
    }

    /// What the nearest carriageway axis says about `p`. `None` if the
    /// field is empty.
    pub fn at(&self, p: Pt) -> Option<Foot> {
        let (i, t, d) = self.at.of(p, FIELD_LIMIT_M)?;
        let (ha, hb, half_w) = self.seg[i];
        Some(Foot { h: ha + (hb - ha) * t, d, half_w })
    }
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
    /// Per outline segment, the room's height and the natural ground at
    /// each of its two ends.
    seg: Vec<(f64, f64, f64, f64)>,
}

impl Ground {
    /// The ground benched to `outline`, whose vertices stand at `room`
    /// over a natural ground of `natural`.
    pub fn new(outline: &Shapes, room: &dyn Fn(Pt) -> f64, natural: &dyn Fn(Pt) -> f64) -> Ground {
        let mut g = Ground::default();
        for ring in outline.iter().flatten() {
            for i in 0..ring.len() {
                let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
                g.at.push(a, b);
                g.seg.push((room(a), room(b), natural(a), natural(b)));
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
        let (h0, h1, g0, g1) = self.seg[i];
        let room = h0 + (h1 - h0) * t;
        let edge = g0 + (g1 - g0) * t;
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
    /// Vertices that took the road's height whole.
    pub lifted: usize,
    /// Vertices on a batter face between the room's reach and the ground.
    pub battered: usize,
    /// Vertices left where the mesh step put them: the free bands.
    pub draped: usize,
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
pub fn run(world: &mut World) -> Summary {
    let (bench, stats, axes, earth) = {
        let terrain = world.terrain.as_ref().expect("the terrain step runs first");
        let profiles = world.profile.as_ref().expect("the profile step runs first");
        let mesh = world.mesh.as_ref().expect("the mesh step runs first");
        let field = Field::new(&profiles.profiles);
        // The asphalt is the road: it takes the whole of its own height
        // wherever it reaches. Only the walk beside it is asked how far
        // out it lies.
        let (c, cs) = lift(&mesh.carriageway, &field, 0.0, false);
        let (p, ps) = lift(&mesh.pavement, &field, KERB_RISE_M, true);
        let mut stats = Stats::default();
        stats.merge(&cs);
        stats.merge(&ps);
        let steps = std::mem::take(&mut stats.at);

        // The ground answers. The outline is the room's own boundary and
        // its heights are read off the room's mesh, vertex for vertex, so
        // the two meet at the seam rather than near it.
        let none = Shapes::new();
        let outline = poly::union_of(&[world.carriageway().unwrap_or(&none), world.walk().unwrap_or(&none)]);
        let natural = |p: Pt| height_at(terrain, p[0], p[1]);
        let seam = seam(&[&c, &p]);
        let room = |q: Pt| match seam.get(&key(q)) {
            Some(h) => *h,
            None => match field.at(q) {
                Some(foot) => foot.batter(foot.h + KERB_RISE_M, natural(q)),
                None => natural(q),
            },
        };
        let ground = Ground::new(&outline, &room, &natural);
        let mut earth = Earth::new(&outline, &ground, &room, &natural, terrain);
        let cut = poly::difference(&vec![poly::rect(world.rect.x0, world.rect.y0, world.rect.x1, world.rect.y1)], &outline);
        let (g, gs) = mesh::triangulate(&cut, &terrain.grid, &|q| ground.at(q, natural(q)));
        earth.triangles = g.indices.len() / 3;
        earth.vertices = g.positions.len();
        earth.off = gs.off_ground;
        earth.lossy = gs.failed + gs.lossy;
        (Bench { carriageway: c, pavement: p, ground: g, steps }, stats, field.len(), earth)
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
        .with_share("seam", earth.unseamed, earth.outline)
        .with("contact", format!("{:.1e}", earth.contact))
        .with_share("walled", earth.walled, earth.outline)
        .with("wall", format!("{:.1}", earth.wall))
        .with_share("touched", earth.touched, earth.lattice)
        .with("off", format!("{:.1e}", earth.off))
        .with("lossy", earth.lossy);
    world.bench = Some(bench);
    summary
}

/// The height the room's mesh gave every one of its vertices, keyed the
/// way [`mesh::WELD_M`] welds them. The room's outline runs through those
/// vertices, so reading its heights here rather than recomputing them is
/// what makes the seam exact rather than close. Where the carriageway and
/// the pavement both reach a position — a kerb the pavement ends at — the
/// lower is the ground's, so the ground meets the asphalt rather than
/// standing a kerb over it.
fn seam(tris: &[&Tri]) -> HashMap<[i64; 2], f64> {
    let mut out: HashMap<[i64; 2], f64> = HashMap::new();
    for tri in tris {
        for p in &tri.positions {
            out.entry(key(*p)).and_modify(|h| *h = h.min(p[2])).or_insert(p[2]);
        }
    }
    out
}

fn key(p: impl AsRef<[f64]>) -> [i64; 2] {
    let p = p.as_ref();
    [(p[0] / mesh::WELD_M).round() as i64, (p[1] / mesh::WELD_M).round() as i64]
}

/// What the ground's answer came to.
#[derive(Debug, Default, Clone, Copy)]
pub struct Earth {
    pub triangles: usize,
    pub vertices: usize,
    /// Outline vertices, and those whose height the room's mesh did not
    /// have: the seam is exact only for the ones it did.
    pub outline: usize,
    pub unseamed: usize,
    /// The largest disagreement, in metres, between the ground and the
    /// room at a vertex of the outline they share.
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
                } else {
                    e.contact = e.contact.max((ground.at(q, n) - r).abs());
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
fn lift(tri: &Tri, field: &Field, rise: f64, walk: bool) -> (Tri, Stats) {
    let mut out = tri.clone();
    let mut stats = Stats { vertices: tri.positions.len(), ..Stats::default() };
    for (i, p) in tri.positions.iter().enumerate() {
        let Some(foot) = field.at([p[0], p[1]]) else {
            stats.draped += 1;
            continue;
        };
        let room_h = foot.h + rise;
        let h = if walk { foot.batter(room_h, p[2]) } else { room_h };
        out.positions[i][2] = h;
        // Where the vertex stands, not whether the height moved: on flat
        // ground the room's height *is* the ground, and a road there is
        // still a road.
        if !walk || foot.d <= foot.half_w + ROOM_REACH_M {
            stats.lifted += 1;
        } else if h == p[2] {
            stats.draped += 1;
        } else {
            stats.battered += 1;
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
    use std::path::Path;

    use crate::terrain::{self, tests::dem};
    use crate::world::Solved;
    use crate::{drape, facade, fillet, kerb, mesh, profile, ribbon, room, surface};

    use super::*;

    /// A world on `ground` with the network of `net` and the buildings of
    /// `houses`, meshed and benched.
    pub(crate) fn world(ground: &str, net: &str, houses: Option<&str>) -> (World, Summary) {
        let mut w = terrain::tests::world();
        terrain::run(&mut w, &mut dem(ground), 5.0, usize::MAX);
        drape::run(&mut w, Path::new(net)).unwrap();
        profile::run(&mut w);
        facade::run(&mut w, houses.map(Path::new)).unwrap();
        ribbon::run(&mut w);
        surface::run(&mut w);
        kerb::run(&mut w);
        fillet::run(&mut w);
        room::run(&mut w);
        mesh::run(&mut w);
        let s = run(&mut w);
        (w, s)
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
                crate::world::Station { s: x, p: [x, 0.0], ground: 400.0, h: 400.0 + x / 10.0, solved: Solved::Grade }
            })
            .collect();
        let p = Profile {
            id: "road".into(),
            class: "residential".into(),
            width_m: 5.5,
            mapped: Kind::Ground,
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
        deck.mapped = Kind::Bridge(1);
        assert!(Field::new(std::slice::from_ref(&deck)).at([0.0, 0.0]).is_none());
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
        let t = w.terrain.as_ref().unwrap();
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
        // Bounded: the plateau is 2.51 m of cut and nothing else is.
        assert!((s.num("cut") - 2.481).abs() < 0.01, "{s}");
        // Where asking per region dragged the far end of this same
        // footway 5.88 m into the air.
        assert!(s.num("cut") < 3.0, "{s}");
        assert!(s.num("step") > 0.0 && s.num("worst") < 4.0, "{s}");
        // The wall stands where the hill outruns the face, not at the
        // kerb: every step is out beyond the plateau.
        assert!(b.steps.iter().all(|p| p[1] > 2.75 + ROOM_REACH_M), "{:?}", b.steps);
    }

    #[test]
    fn a_junction_on_a_hill_has_one_height() {
        // Four legs meeting at the origin: the profile pins the connector,
        // so the four surfaces meet there without a step, and the field is
        // continuous across the whole junction.
        let (w, s) = world("hill?amp=60&radius=400", "net:cross?len=400", None);
        assert_eq!(s.num("step"), 0.0, "{s}");
        let b = bench(&w);
        let t = w.terrain.as_ref().unwrap();
        let centre: Vec<f64> =
            b.carriageway.positions.iter().filter(|p| p[0].hypot(p[1]) < 1.0).map(|p| p[2]).collect();
        assert!(!centre.is_empty());
        let top = crate::terrain::height_at(t, 0.0, 0.0);
        assert!(centre.iter().all(|h| (h - top).abs() < 1e-9), "the junction is not at the ground's height");
    }

    /// The engineered ground of a world, and its natural one.
    fn grounds(w: &World) -> (Ground, impl Fn(Pt) -> f64 + '_) {
        let terrain = w.terrain.as_ref().unwrap();
        let natural = move |p: Pt| crate::terrain::height_at(terrain, p[0], p[1]);
        let b = w.bench.as_ref().unwrap();
        let seam = seam(&[&b.carriageway, &b.pavement]);
        let none = Shapes::new();
        let outline = poly::union_of(&[w.carriageway().unwrap_or(&none), w.walk().unwrap_or(&none)]);
        let room = |q: Pt| seam.get(&key(q)).copied().unwrap_or_else(|| natural(q));
        (Ground::new(&outline, &room, &natural), natural)
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
        let inside = poly::Indexed::new(w.carriageway().unwrap());
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
