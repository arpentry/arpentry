//! The earthwork: the ground answers the lifted room.
//!
//! [`Ground`] is the terrain with the room cut out of it and an earthwork
//! residual added: pinned at the outline to the room's height less the
//! natural ground (one face of it at most, a wall for the rest), falling to
//! nothing at 1 in [`crate::standard::EARTHWORK_BATTER`] of the natural
//! ground, and blended where the nearest outline segment changes, so it is
//! continuous and always meets the ground within 7.5 m. No triangle of the
//! ground lies under the asphalt, which is where every artefact of a ground
//! drawn beneath an opaque surface lives (`data/plans/terrain-hole-plan.md`).
//!
//! The outline is the one mesh's own edges between a face that cuts and one
//! that does not, and the ground's segments are those edges, every height
//! read by index from the lift's copies. **A pavement no road answers for,
//! or that drapes past one, is passive**: it is not the road's
//! cross-section, so it is the ground's — it takes the engineered ground,
//! and a footpath leaving a street runs up the street's batter instead of
//! standing on the terrain beside it (`regraded`).
//!
//! The faces that close what the ground cannot — the walls and the kerbs —
//! are the bench step's.

use std::collections::{HashMap, HashSet};

use crate::copies::{Copies, Fields, Key, Part, Rule, Stats, Surface, NONE};
use crate::field::Ground;
use crate::lattice::height_at;
use crate::poly::{self, Pt};
use crate::standard::{KERB_RISE_M, MAX_BENCH_FACE_M};
use crate::step::{Residual, Summary};
use crate::world::{Arrangement, Earthwork, Lifted, Mesh, Terrain, Tri};

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

/// How many samples of its own rule an edge is read at before it is called a
/// step ([`steps_of`]).
const STEP_SAMPLES: usize = 16;

/// The **drawn ground** against the raw DEM: [`crate::step::Residual`] over
/// every vertex of the terrain this step re-meshes.
///
/// This is where the run's residual chain ends, and it is the only link in it
/// measured over an area rather than along an axis. The steps before it report
/// the same quantity over their at-grade stations
/// ([`crate::step::residual_of`]), so the two are not the same population
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

/// Counts the steps of one part: edges its heights **jump** across.
///
/// An edge over [`KERB_RISE_M`] and steeper than [`STEP_GRADE`] is asked
/// about the **residual** — what the lift added over the raw DEM — because
/// that is the model's and the rest is the terrain's. Where the residual
/// barely changes along it the steepness is the DEM's own (`dem_steep`): a
/// street that follows a flank, a footpath on its regraded ground, a
/// retaining wall the DEM images. Otherwise the edge's own rule is sampled
/// along it ([`STEP_SAMPLES`]): a rule continuous along the edge, however
/// steep or curved, moves a little between samples (`steep`); a jump puts at
/// least half the rise between two of them. Only the second is a step. (A
/// single midpoint could not tell them apart: the DEM's bicubic curvature
/// under a steep edge reads as a jump.)
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
            // What the lift added over the DEM at each end, and at the middle
            // by the triangle's own rule.
            let (ra, rb) = (p[2] - part.natural[a as usize], q[2] - part.natural[b as usize]);
            if (rb - ra).abs() <= KERB_RISE_M + STEP_SLACK_M {
                stats.dem_steep += 1;
                continue;
            }
            // The rule along the edge, from one stored end to the other: a
            // continuous rule, however curved, moves a sixteenth of the way
            // between samples; a jump puts at least half of it between two.
            let mut prev = p[2];
            let mut widest = 0.0f64;
            for k in 1..=STEP_SAMPLES {
                let f = k as f64 / STEP_SAMPLES as f64;
                let z = if k == STEP_SAMPLES {
                    q[2]
                } else {
                    at([p[0] + (q[0] - p[0]) * f, p[1] + (q[1] - p[1]) * f], key)
                };
                widest = widest.max((z - prev).abs());
                prev = z;
            }
            let mid = [(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0];
            if widest <= (dh / 2.0).max(KERB_RISE_M) {
                stats.steep += 1;
                continue;
            }
            stats.steps += 1;
            stats.worst = stats.worst.max(dh);
            stats.at.push(mid);
        }
    }
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

/// The room's height at every vertex of the one mesh: its lowest paved
/// copy's, read by index.
///
/// Where the asphalt and the pavement both reach a vertex — the end of a
/// kerb — the lower is the ground's, so the ground meets the asphalt rather
/// than standing a kerb over it. A vertex no paving reaches has no room
/// height of its own: a gallery's rim, whose footprint the arrangement cuts
/// but nothing paves. The nearer field's cross-section answers there, as it
/// always has.
fn room_of<'a>(
    mesh: &'a Mesh,
    copies: &Copies,
    fields: &'a Fields,
    natural: &'a dyn Fn(Pt) -> f64,
) -> impl Fn(u32) -> f64 + 'a {
    let mut room_at = vec![f64::INFINITY; mesh.tri.positions.len()];
    for part in [&copies.carriageway, &copies.pavement, &copies.ballast] {
        for (i, &v) in part.of.iter().enumerate() {
            room_at[v as usize] = room_at[v as usize].min(part.tri.positions[i][2]);
        }
    }
    move |v: u32| {
        let h = room_at[v as usize];
        if h.is_finite() {
            return h;
        }
        let q = [mesh.tri.positions[v as usize][0], mesh.tri.positions[v as usize][1]];
        match (fields.roads.at(q), fields.rails.at(q)) {
            (Some(r), Some(t)) if t.d < r.d => t.batter(t.h, natural(q)),
            (Some(r), _) => r.batter(r.h + KERB_RISE_M, natural(q)),
            (None, Some(t)) => t.batter(t.h, natural(q)),
            (None, None) => natural(q),
        }
    }
}

/// **A pavement no road answers for, or that drapes past one, is passive**:
/// it takes the engineered ground, and pins nothing.
fn passive(k: Option<Key>) -> bool {
    matches!(k, Some((Surface::Near | Surface::Far, rule)) if rule.drape || rule.axis == NONE)
}

/// Whether one-mesh vertex `v` is on the rect's own border, to the kernel's
/// grid.
fn on_border(mesh: &Mesh) -> impl Fn(u32) -> bool + '_ {
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for q in &mesh.tri.positions {
        for k in 0..2 {
            lo[k] = lo[k].min(q[k]);
            hi[k] = hi[k].max(q[k]);
        }
    }
    move |v: u32| {
        let q = mesh.tri.positions[v as usize];
        (0..2).any(|k| (q[k] - lo[k]).abs() <= poly::GRID_M || (q[k] - hi[k]).abs() <= poly::GRID_M)
    }
}

/// Benches the ground to the lifted room, and regrades the passive
/// pavement onto it.
pub fn run(terrain: &Terrain, mesh: &Mesh, arrangement: &Arrangement, lifted: &Lifted) -> (Earthwork, Summary) {
    // The tunnels' openings: the mouths of the portals the partition cut
    // open, and the galleries the arrangement cut a face for.
    let portals = &arrangement.portals;
    let over = poly::Indexed::new(&arrangement.over);
    let fields = &lifted.fields;
    let natural = |q: Pt| height_at(terrain, q[0], q[1]);
    // The lift's copies, and the ground's beside them.
    let mut copies = lifted.copies.clone();
    copies.add_ground(mesh, arrangement);
    let room = room_of(mesh, &copies, fields, &natural);
    let plan = |v: u32| {
        let q = mesh.tri.positions[v as usize];
        [q[0], q[1]]
    };

    // **The outline is the one mesh's own edges** between a face that cuts
    // and one that does not, each in the winding of the cutting side so the
    // room lies on its left. Both meshes that meet there are the one mesh, so
    // these edges are already split at every lattice crossing. A pavement
    // that drapes is not an edge of the room: it follows the ground (below),
    // so it pins nothing and meets it with no face.
    let mut outline: Vec<(u32, u32)> = Vec::new();
    // The key on the cutting side of each outline edge.
    let mut cutting: Vec<Option<Key>> = Vec::new();
    for e in lifted.bounds.iter().filter(|e| !e.open) {
        match (arrangement.face(e.a).cuts(), arrangement.face(e.b).cuts()) {
            (true, false) if !passive(e.ka) => {
                outline.push((e.u, e.v));
                cutting.push(e.ka);
            }
            (false, true) if !passive(e.kb) => {
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
    let ground = Ground::of_edges(&segments, portals);
    // The ground's copies at the engineered ground. The one mesh was built
    // at the natural ground, which is exactly [`Ground::at`]'s second
    // argument.
    //
    // **One function, pins included.** The batter's blend narrows to nothing
    // at the outline, so at an outline vertex it *is* the pin wherever the
    // segments meeting there agree, and differs only where two edges pin one
    // vertex differently — a split, where the wall stands anyway. Pinning
    // those vertices exactly instead put a jump in every ground and footpath
    // triangle beside a split: 300 of them on the loop box.
    for q in copies.ground.tri.positions.iter_mut() {
        q[2] = ground.at([q[0], q[1]], q[2]);
    }

    // The passive pavement on the same ground, by the same function the
    // ground copy beside it took, so the two meet with nothing between.
    let (mut regraded, mut regrade_m) = (0usize, 0.0f64);
    {
        let part = &mut copies.pavement;
        for i in 0..part.tri.positions.len() {
            if !passive(Some(part.key[i])) {
                continue;
            }
            let (q, nat) = (part.tri.positions[i], part.natural[i]);
            let h = nat + ground.residual([q[0], q[1]]);
            let e = h - nat;
            part.tri.positions[i][2] = h;
            if e.abs() > 1e-9 {
                regraded += 1;
                regrade_m = regrade_m.max(e.abs());
            }
        }
    }

    // **Where the steps are**: edges the height field jumps across within one
    // rule, kept as marks for the plan view.
    let mut stats = Stats::default();
    let at = |q: Pt, key: Key| {
        if passive(Some(key)) {
            natural(q) + ground.residual(q)
        } else {
            fields.lift(key.0, Some(&over)).height(q, natural(q), key.1).0
        }
    };
    for part in [&copies.carriageway, &copies.ballast, &copies.pavement] {
        steps_of(part, &at, &mut stats);
    }
    let steps = std::mem::take(&mut stats.at);
    let summary = Summary::new()
        .with_share("step", stats.steps, stats.edges)
        .with("worst", format!("{:.3}", stats.worst))
        // Edges steeper than a step whose both ends are on the raw DEM: the
        // terrain's own steepness under a draped pavement — a retaining wall
        // the DEM images — and not a discontinuity the lift made. They were
        // 10 028 of the loop box's 18 408 `step` before they were told apart.
        .with("steep", stats.steep)
        .with("dem_steep", stats.dem_steep)
        // Passive pavement vertices the earthwork moved off the raw DEM: a
        // footpath running up a street's batter rather than beside it.
        .with("regraded", format!("{regraded} to {regrade_m:.2}"));
    (Earthwork { copies, outline, cutting, top, ground, steps }, summary)
}

/// What the room and the ground came to: where the room's vertices stand,
/// how the two meet at the outline, how much ground was moved, and how far
/// the drawn ground stands off the raw DEM.
pub fn check(terrain: &Terrain, mesh: &Mesh, arrangement: &Arrangement, lifted: &Lifted, earthwork: &Earthwork) -> Summary {
    let (copies, ground) = (&earthwork.copies, &earthwork.ground);
    let portals = &arrangement.portals;
    let over = poly::Indexed::new(&arrangement.over);
    let fields = &lifted.fields;
    let natural = |q: Pt| height_at(terrain, q[0], q[1]);
    let room = room_of(mesh, copies, fields, &natural);
    let plan = |v: u32| {
        let q = mesh.tri.positions[v as usize];
        [q[0], q[1]]
    };

    // Where the room's vertices stand. Counted once welded, over the copies
    // a triangle still names: a copy merged into its twin is not a vertex of
    // the world.
    let mut stats = Stats::default();
    for (part, feet) in [&copies.carriageway, &copies.ballast, &copies.pavement].into_iter().zip(&lifted.feet) {
        let mut used = vec![false; part.tri.positions.len()];
        for &i in &part.tri.indices {
            used[i as usize] = true;
        }
        for i in (0..used.len()).filter(|&i| used[i]) {
            let (surface, rule) = part.key[i];
            fields.lift(surface, Some(&over)).account(&mut stats, rule, part.tri.positions[i][2], part.natural[i], feet[i]);
        }
    }

    // A paved rim with nothing beyond it, away from the rect's edge: a crack
    // in the one mesh, and the one thing no face can close.
    let border = on_border(mesh);
    let unmet = lifted
        .bounds
        .iter()
        .filter(|e| e.open && arrangement.face(e.a).cuts() && !(border(e.u) && border(e.v)))
        .count();

    // How many paved copies each vertex has once welded: where there are two
    // — two surfaces, or two rules of one — the seam between them is the edge
    // rule's, not a contact with the ground.
    let mut paved_by = vec![0u8; mesh.tri.positions.len()];
    {
        let mut seen: HashSet<(u32, Surface, u32)> = Default::default();
        for (&(v, (surface, _)), &id) in &copies.paved {
            if seen.insert((v, surface, id)) {
                paved_by[v as usize] = paved_by[v as usize].saturating_add(1);
            }
        }
    }
    // How the paving and the ground meet at the outline: whether every
    // outline vertex has a paved copy on its cutting side (`seam`), and how
    // far apart the two stand where a batter runs (`contact`). Both are
    // constructions now rather than searches, so both are checks.
    let (mut asked, mut missed, mut contact) = (0usize, 0usize, 0.0f64);
    for (&(u, v), key) in earthwork.outline.iter().zip(&earthwork.cutting) {
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
    let mut earth = Earth::new(&earthwork.outline, &room, &|v| natural(plan(v)), ground, terrain);
    earth.asked = asked;
    earth.unseamed = missed;
    earth.unmet = unmet;
    earth.rim = asked;
    earth.contact = contact;
    let g = &copies.ground;
    earth.triangles = g.tri.indices.len() / 3;
    earth.vertices = g.tri.positions.len();
    // Only a triangle the earthwork moved can stand off the ground it
    // benched: one whose three vertices it left at the natural ground is the
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

    // §3.3 says an edge is welded or spanned by a quad, "so `step` has
    // nothing left to count". This is the share of the steps that lie on an
    // arrangement edge at all.
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
        earthwork
            .steps
            .iter()
            .filter(|m| {
                index.get(&poly::cell_of(**m, poly::CELL_M)).into_iter().flatten().any(|&i| {
                    let e = &arrangement.edges[i];
                    poly::segment_distance(e.a, e.b, **m) < NEAR_M
                })
            })
            .count()
    };

    Summary::new()
        .with_part("lifted", stats.lifted, stats.vertices)
        .with("battered", stats.battered)
        .with("draped", stats.draped)
        .with("cut", format!("{:.3}", stats.cut))
        .with("fill", format!("{:.3}", stats.fill))
        .with("ground", format!("{}/{}", earth.triangles, earth.vertices))
        .with_share("seam", earth.unseamed, earth.asked)
        .with_share("unmet", earth.unmet, earth.rim)
        .with("contact", format!("{:.2}", earth.contact))
        .with_share("step_on_edge", earth.on_edge, earthwork.steps.len())
        .with_share("walled", earth.walled, earth.outline)
        .with("wall", format!("{:.1}", earth.wall))
        .with_share("touched", earth.touched, earth.lattice)
        .with("off", format!("{:.1e}", earth.off))
        .with("flown", stats.flown)
        .with_part("free", stats.free, stats.draped)
        .with_m2("carried", arrangement.carried_m2)
        .with_residual(drawn_residual(terrain, &copies.ground.tri))
}
