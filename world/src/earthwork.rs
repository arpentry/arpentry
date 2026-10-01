//! The earthwork: the ground answers the lifted room.
//!
//! [`Ground`] is the terrain with the room cut out of it and an earthwork
//! residual added: pinned at the outline to the room's height less the
//! natural ground (one face of it at most, a wall for the rest), falling to
//! nothing at 1 in [`crate::standard::EARTHWORK_BATTER`] of the natural
//! ground, and blended where the nearest outline segment changes, so it is
//! continuous and always meets the ground within 7.5 m. No triangle of the
//! ground lies under the asphalt, which is where every artefact of a ground
//! drawn beneath an opaque surface lives.
//!
//! The outline is the one mesh's own edges between a face that cuts and one
//! that does not, and the ground's segments are those edges, every height
//! read by index from the lift's copies. **A pavement no road answers for,
//! or that drapes past one, is passive**: it is not the road's
//! cross-section, so it is the ground's — it takes the engineered ground,
//! and a footpath leaving a street runs up the street's batter instead of
//! standing on the terrain beside it (`regraded`). Where two pins that
//! disagree face each other across such a footpath, the batters fold into
//! each other under it, and there the ground is relaxed into a ramp from one
//! pin to the other instead ([`Ground::ramp`]; `ramps`, `ramp_grade`,
//! `path_grade`).
//!
//! The faces that close what the ground cannot — the walls and the kerbs —
//! are the bench step's.

use std::collections::{HashMap, HashSet};

use crate::line;
use crate::copies::{self, Copies, Fields, Key, Part, Rule, Surface};
use crate::ground::Ground;
use crate::lattice::height_at;
use crate::poly::{self, Pt};
use crate::standard::{KERB_RISE_M, MAX_BATTER_FACE_M};
use crate::step::{Residual, Summary};
use crate::world::{Arrangement, Earthwork, Lifted, Mesh, Terrain, Tri};

/// Slack, in metres, on the kerb's own rise before an edge counts as a
/// step. The rise arrives as a sum of floats and lands a hair either
/// side of itself: without the slack, the edge between a lifted pavement
/// and the free band beside it — the kerb, exactly, and no more — would
/// count as a step on flat ground.
const STEP_SLACK_M: f64 = 1e-6;

/// A mesh edge whose ends differ in height by more than [`KERB_RISE_M`]
/// *and* by more than this many metres per metre of its own length is a
/// step: the field is discontinuous across it. Both conditions are needed
/// and neither alone would do. A street on a mountain flank climbs 30 %, so
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
/// ([`crate::world::Profiles::residual`]), so the two are not the same
/// population and are not meant to be: an axis says how far the *road* has
/// left the DEM, and this says how far the *ground* has, which is the
/// earthwork. A run where the second is much the larger is a run whose
/// earthwork is not the road's.
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
fn steps_of(part: &Part, at: &dyn Fn(Pt, Key) -> f64, stats: &mut Steps) {
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

/// The edges the lifted surfaces' heights jump across ([`steps_of`]).
#[derive(Debug, Default, Clone)]
struct Steps {
    edges: usize,
    /// Edges the height field jumps across within one rule.
    steps: usize,
    /// The largest of those jumps, in metres.
    worst: f64,
    /// Where they are.
    at: Vec<[f64; 2]>,
    /// Edges as steep as a step whose residual over the DEM barely changes:
    /// the terrain's own steepness.
    dem_steep: usize,
    /// Edges as steep as a step that their rule is continuous along.
    steep: usize,
}

/// The room's height at every vertex of the one mesh: its lowest paved
/// copy's, read by index.
///
/// Where the asphalt and the pavement both reach a vertex — the end of a
/// kerb — the lower is the ground's, so the ground meets the asphalt rather
/// than standing a kerb over it. A vertex no paving reaches has no room
/// height of its own: a gallery's rim, whose footprint the arrangement cuts
/// but nothing paves. The nearer field's cross-section answers there.
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

/// A side of an edge that is paved, and passive ([`crate::copies::passive`]).
fn passive(k: Option<Key>) -> bool {
    k.is_some_and(copies::passive)
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
    let mut ground = Ground::of_edges(&segments, portals);
    let ramps = ramp(mesh, arrangement, &copies, &mut ground);
    // The ground's copies at the engineered ground. The one mesh was built
    // at the natural ground, which is exactly [`Ground::at`]'s second
    // argument.
    //
    // **One function, pins included.** The batter's blend narrows to nothing
    // at the outline, so at an outline vertex it *is* the pin wherever the
    // segments meeting there agree, and differs only where two edges pin one
    // vertex differently — a split, where the wall stands anyway. Pinning
    // those vertices exactly instead would put a jump in every ground and
    // footpath triangle beside a split.
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
    let mut stats = Steps::default();
    let at = |q: Pt, key: Key| {
        if passive(Some(key)) {
            natural(q) + ground.residual(q)
        } else {
            fields.lift(key.0, &over).height(q, natural(q), key.1).0
        }
    };
    for part in [&copies.carriageway, &copies.ballast, &copies.pavement] {
        steps_of(part, &at, &mut stats);
    }
    let steps = std::mem::take(&mut stats.at);
    let summary = Summary::new()
        .with_share("step", stats.steps, stats.edges)
        .with("worst", format!("{:.3}", stats.worst))
        // Edges as steep as a step that their own rule is continuous along,
        // and edges as steep as a step whose residual over the DEM barely
        // changes — the terrain's own steepness under a draped or regraded
        // surface, a retaining wall the DEM images. Neither is a
        // discontinuity the lift made.
        .with("steep", stats.steep)
        .with("dem_steep", stats.dem_steep)
        // Passive pavement vertices the earthwork moved off the raw DEM: a
        // footpath running up a street's batter rather than beside it.
        .with("regraded", format!("{regraded} to {regrade_m:.2}"));
    let summary = ramps.report(summary, mesh, &ground);
    (Earthwork { copies, outline, cutting, top, ground, steps }, summary)
}

/// The ramps the ground was relaxed over, as the one mesh's triangles, and
/// which of those are passive pavement.
struct Ramps {
    tris: Vec<[u32; 3]>,
    path: Vec<bool>,
    ramps: Vec<Vec<usize>>,
    /// The vertices the ramps solved for, in order.
    free: Vec<u32>,
}

/// Relaxes the ground where two of its batters fold into each other under a
/// footpath ([`Ground::ramp`]): over every partition triangle that takes the
/// engineered ground — the ground's own and the passive pavement's — holding
/// the batter wherever the ground itself meets anything else, which is where
/// its copy must stay the pin, and on the rect's border.
fn ramp(mesh: &Mesh, arrangement: &Arrangement, copies: &Copies, ground: &mut Ground) -> Ramps {
    let n = mesh.tri.positions.len();
    let plan: Vec<Pt> = mesh.tri.positions.iter().map(|q| [q[0], q[1]]).collect();
    let (mut tris, mut path) = (Vec::new(), Vec::new());
    // Whether a vertex touches a triangle outside the field, and whether it
    // has a ground copy at all ([`Copies::add_ground`]).
    let mut held = vec![false; n];
    let mut grounded = vec![false; n];
    // The pavement's triangles are its part's in the one mesh's order, so the
    // key a pavement triangle took is the next of its part's.
    let mut paved = 0usize;
    for (t, &f) in mesh.tri.indices.chunks_exact(3).zip(&mesh.of_face) {
        let face = arrangement.face(f);
        let surface = Surface::paved(face);
        let passive = match surface {
            Some(Surface::Near | Surface::Far) => {
                paved += 1;
                copies::passive(copies.pavement.face_key[paved - 1])
            }
            _ => false,
        };
        let t = [t[0], t[1], t[2]];
        if arrangement.in_partition(f) && !face.cuts() {
            for v in t {
                grounded[v as usize] = true;
            }
        }
        // A paved face that does not cut — a deck's, over the ground it
        // spans — is paving all the same, and the ground under it is held;
        // so is a gallery's, which cuts and is not paved.
        if arrangement.in_partition(f) && ((surface.is_none() && !face.cuts()) || passive) {
            tris.push(t);
            path.push(passive);
        } else {
            for v in t {
                held[v as usize] = true;
            }
        }
    }
    // A footpath's copy beside other paving is a split, with a face between
    // the two, and is the ramp's to move; the ground's copy beside it is a
    // pin.
    let border = mesh.on_border();
    for (v, h) in held.iter_mut().enumerate() {
        *h = (*h && grounded[v]) || border(v as u32);
    }
    let (ramps, free) = ground.ramp(&crate::ground::Domain { plan: &plan, tris: &tris, path: &path, held: &held });
    Ramps { tris, path, ramps, free }
}

impl Ramps {
    /// How many ramps there are, how many of them carry a footpath, how many
    /// vertices were relaxed, and how steep each footpath came out: the
    /// residual's own grade across it (`ramp_grade`, the ramp) and the drawn
    /// path's (`path_grade`, the natural ground and the ramp). A ramp's grade
    /// is the one nine tenths of its footpath's area is no steeper than, over
    /// its triangles larger than the census's speck with two corners the ramp
    /// solved for; and the summary gives the median, the ninetieth percentile
    /// and the steepest of those over the ramps.
    ///
    /// **Not the steepest triangle**: a triangle with a corner on a pin is
    /// that pin's, and where two pins split one vertex — a paved corner the
    /// ground meets at two heights — no ramp makes the triangles fanned round
    /// it less steep than the split. Its steepest sliver read 64 m/m on
    /// `net:driveway?d=9`, 1.4 m² of fin, and 75 m/m over the Montreux sites;
    /// counted that way, the ramps' grade would be the splits'.
    ///
    /// **A steep ramp is the honest cost of this rule**: two pins three
    /// metres apart in height and two apart in plan leave no surface between
    /// them gentler than 1.5, and the ramp is the least steep of them.
    fn report(&self, summary: Summary, mesh: &Mesh, ground: &Ground) -> Summary {
        const SPECK_M2: f64 = 0.01;
        let pos = &mesh.tri.positions;
        let (mut grades, mut paths): (Vec<f64>, Vec<f64>) = (Vec::new(), Vec::new());
        let verbose = std::env::var_os("ARPENTRY_RAMPS").is_some();
        // The grade nine tenths of `(grade, area)` is no steeper than.
        let p90 = |mut v: Vec<(f64, f64)>| {
            v.sort_by(|a, b| a.0.total_cmp(&b.0));
            let total: f64 = v.iter().map(|x| x.1).sum();
            let mut seen = 0.0;
            v.iter().find(|x| {
                seen += x.1;
                seen >= 0.9 * total
            })
            .map_or(0.0, |x| x.0)
        };
        for ramp in &self.ramps {
            let (mut residual, mut drawn, mut at, mut steepest) = (Vec::new(), Vec::new(), [0.0; 2], 0.0f64);
            for &t in ramp.iter().filter(|&&t| self.path[t]) {
                let solved = self.tris[t].iter().filter(|v| self.free.binary_search(v).is_ok()).count();
                let p = self.tris[t].map(|v| [pos[v as usize][0], pos[v as usize][1]]);
                let area = ((p[1][0] - p[0][0]) * (p[2][1] - p[0][1]) - (p[1][1] - p[0][1]) * (p[2][0] - p[0][0])).abs() / 2.0;
                if area < SPECK_M2 || solved < 2 {
                    continue;
                }
                let e = p.map(|q| ground.residual(q));
                let z = [0, 1, 2].map(|k| pos[self.tris[t][k] as usize][2] + e[k]);
                let s = crate::ground::grade_of(p, z).unwrap_or(0.0);
                residual.push((crate::ground::grade_of(p, e).unwrap_or(0.0), area));
                drawn.push((s, area));
                if s > steepest {
                    steepest = s;
                    at = [(p[0][0] + p[1][0] + p[2][0]) / 3.0, (p[0][1] + p[1][1] + p[2][1]) / 3.0];
                }
            }
            if drawn.is_empty() {
                continue;
            }
            let (grade, path) = (p90(residual), p90(drawn));
            grades.push(grade);
            paths.push(path);
            if verbose {
                println!(
                    "ramp tris={} grade={grade:.3} path={path:.3} steepest={steepest:.3} at {:.1},{:.1}",
                    ramp.len(),
                    at[0],
                    at[1]
                );
            }
        }
        let quantiles = |mut r: Vec<f64>| {
            if r.is_empty() {
                return "-".to_string();
            }
            r.sort_by(f64::total_cmp);
            let q = |f: f64| r[((r.len() as f64 - 1.0) * f).round() as usize];
            format!("{:.2}/{:.2}/{:.2}", q(0.5), q(0.9), q(1.0))
        };
        summary
            .with("ramps", format!("{} ({} paths)", self.ramps.len(), paths.len()))
            .with("ramped", self.free.len())
            .with("ramp_grade", quantiles(grades))
            .with("path_grade", quantiles(paths))
    }
}

/// How the ground meets the room: whether every outline vertex has a paved
/// copy on its cutting side (`seam`), whether any paved rim has nothing
/// beyond it (`unmet`), how far the two stand apart where a batter runs
/// (`contact`), where a wall stands instead, how much ground was moved, and
/// how far the drawn ground stands off the raw DEM.
pub fn check(terrain: &Terrain, mesh: &Mesh, arrangement: &Arrangement, lifted: &Lifted, earthwork: &Earthwork) -> Summary {
    let (copies, ground) = (&earthwork.copies, &earthwork.ground);
    let portals = &arrangement.portals;
    let over = poly::Indexed::new(&arrangement.over);
    let natural = |q: Pt| height_at(terrain, q[0], q[1]);
    let room = room_of(mesh, copies, &lifted.fields, &natural);
    let plan = |v: u32| {
        let q = mesh.tri.positions[v as usize];
        [q[0], q[1]]
    };

    // The paving's rim, and the edges of it with nothing beyond them away
    // from the rect's edge: a crack in the one mesh, and the one thing no
    // face can close.
    let border = mesh.on_border();
    let rim: Vec<_> = lifted.bounds.iter().filter(|e| arrangement.face(e.a).cuts()).collect();
    let unmet = rim.iter().filter(|e| e.open && !(border(e.u) && border(e.v))).count();

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
    let (mut asked, mut unseamed, mut contact) = (0usize, 0usize, 0.0f64);
    for (&(u, v), key) in earthwork.outline.iter().zip(&earthwork.cutting) {
        for w in [u, v] {
            let q = plan(w);
            let Some(key) = *key else { continue };
            if portals.open(q) || over.contains(q) {
                continue;
            }
            asked += 1;
            let Some(h) = copies.height(w, key) else {
                unseamed += 1;
                continue;
            };
            if paved_by[w as usize] == 1 && (natural(q) - h).abs() <= MAX_BATTER_FACE_M {
                if let Some(gh) = copies.height(w, (Surface::Ground, Rule::FREE)) {
                    contact = contact.max((gh - h).abs());
                }
            }
        }
    }

    // Outline vertices standing more than one face from the ground, where
    // the batter takes one face and a wall the rest, and the tallest wall.
    let (mut walled, mut wall) = (0usize, 0.0f64);
    for &(u, _) in &earthwork.outline {
        let drop = (natural(plan(u)) - room(u)).abs();
        if drop > MAX_BATTER_FACE_M {
            walled += 1;
            wall = wall.max(drop - MAX_BATTER_FACE_M);
        }
    }

    // Lattice vertices the earthwork moved. Invariant I8 of
    // docs/GENERATION.md §7 — the ground outside
    // every toe is the DEM's, bit for bit — holds by construction, since a
    // point further from the outline than a face may run is never asked
    // about; the count says how much ground was actually moved.
    let lattice = terrain.grid.vertex_count();
    let touched = (0..lattice)
        .filter(|&i| {
            let [x, y, z] = terrain.position(i);
            ground.at([x, y], z) != z
        })
        .count();

    // How far the ground mesh's triangles stand off the engineered ground at
    // their centroids — the batter's crease, which no breakline resolves.
    // Only a triangle the earthwork moved can: one whose three vertices it
    // left at the natural ground is the terrain's own.
    let g = &copies.ground;
    let moved = |v: u32| g.tri.positions[v as usize][2] != g.natural[v as usize];
    let off = g
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

    // The share of the steps that lie on an arrangement edge at all: the
    // ones a face across the edge could close.
    let on_edge = {
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
                    line::segment_distance(e.a, e.b, **m) < NEAR_M
                })
            })
            .count()
    };

    Summary::new()
        .with("ground", format!("{}/{}", g.tri.indices.len() / 3, g.tri.positions.len()))
        .with_share("seam", unseamed, asked)
        .with_share("unmet", unmet, rim.len())
        .with("contact", format!("{contact:.2}"))
        .with_share("step_on_edge", on_edge, earthwork.steps.len())
        .with_share("walled", walled, earthwork.outline.len())
        .with("wall", format!("{wall:.1}"))
        .with_share("touched", touched, lattice)
        .with("off", format!("{off:.1e}"))
        .with_m2("carried", arrangement.carried_m2)
        .with_residual(drawn_residual(terrain, &copies.ground.tri))
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::pipeline::tests::stood as world;
    use crate::poly::Shapes;
    use crate::portal::Portals;
    use crate::standard::EARTHWORK_BATTER;
    use crate::world::World;

    /// A polygon outline as the one mesh hands it to the earthwork: every ring
    /// split where it crosses the lattice, as directed edges over one list
    /// of positions.
    pub(crate) fn outline_of(outline: &Shapes, grid: &crate::grid::Grid) -> (Vec<[f64; 3]>, Vec<(u32, u32)>) {
        let (mut pos, mut edges) = (Vec::new(), Vec::new());
        for ring in outline.iter().flatten() {
            let start = pos.len() as u32;
            for i in 0..ring.len() {
                let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
                pos.push([a[0], a[1], 0.0]);
                let mut cut = crate::lattice::split(grid, a, b);
                cut.pop();
                pos.extend(cut.into_iter().map(|q| [q[0], q[1], 0.0]));
            }
            let end = pos.len() as u32;
            edges.extend((start..end).map(|k| (k, if k + 1 == end { start } else { k + 1 })));
        }
        (pos, edges)
    }

    /// The ground benched to `outline` at `room` over `natural`.
    pub(crate) fn ground_on(
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
    fn grounds(w: &World) -> (&Ground, impl Fn(Pt) -> f64 + '_) {
        let terrain = w.terrain.as_ref().unwrap();
        let natural = move |p: Pt| crate::lattice::height_at(terrain, p[0], p[1]);
        (&w.earthwork.as_ref().unwrap().ground, natural)
    }

    #[test]
    fn the_batter_is_one_in_two_and_a_half_and_stops_at_the_ground() {
        // A 5.5 m road along the
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
        // It daylights, and there is no lip: a face at an absolute 1 in 2.5
        // would close only 0.1 m per metre up this hill, never meet the
        // ground within the 7.5 m a face may run, and leave 0.075 m standing
        // where it stopped.
        for d in [toe + 1e-6, toe + 1.0, EARTHWORK_BATTER * MAX_BATTER_FACE_M, 100.0] {
            assert_eq!(at(2.75 + d), natural([0.0, 2.75 + d]), "not daylighted at {d}");
            assert_eq!(at(-2.75 - d), natural([0.0, -2.75 - d]), "not daylighted at -{d}");
        }
        // And the ground meets the room at the kerb, exactly.
        assert!(s.num("contact") < 1e-9, "{s}");
        assert_eq!(s.num("walled"), 0.0, "{s}");
        assert_eq!(s.num("wall_m2"), 0.0, "{s}");
    }

    #[test]
    fn a_gentler_hill_daylights_where_the_batter_meets_it() {
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
    /// Refusing the batter outright once the room stands more than one face
    /// off the ground would leave the natural ground beside a walled stretch
    /// and the batter beside the next battered one — a step in the ground on
    /// the line between them. Clamped, the pin moves with the drop and the
    /// ground with it.
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
        let inside = poly::Indexed::new(&w.legs.as_ref().expect("the legs step ran").surface.carriageway);
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
        // (invariant I8) and the only earth moved is the kerb's own rise.
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
        // ground will build, the partition calls the approach a deck and the
        // structure step carries it, so the fill this step owes stops there
        // instead of climbing to 6.5 m and being closed by a 6.5 m wall.
        let (_, s) = world("flat", "net:overpass?len=300", None);
        assert_eq!(s.num("cut"), 0.0, "{s}");
        assert!(s.num("fill") <= crate::standard::DECK_STANDOFF_M + 1e-6, "{s}");
        assert!(s.num("fill") > 2.0, "the approach is on no fill at all: {s}");
        assert_eq!(s.num("walled"), 0.0, "the ground still walls it: {s}");
        // **The mirror is not symmetric, and that is the point.** The
        // approach dips toward the bore and the ground owes the cut — but
        // only as far as `bore_cover_m`, because a cutting stays a cutting
        // until a tube fits under it, where a fill becomes a deck as soon as
        // it passes the tallest face the ground will build. So the cut runs
        // deeper than the fill did, and between the two thresholds it *is*
        // walled: a cutting three metres deep is a real cutting with real
        // walls, and a fill three metres tall is a deck drawn wrong.
        let (_, s) = world("flat", "net:underpass?len=300", None);
        assert_eq!(s.num("fill"), 0.0, "{s}");
        assert!(s.num("cut") <= crate::standard::bore_cover_m("residential") + 1e-6, "{s}");
        assert!(s.num("cut") > crate::standard::DECK_STANDOFF_M, "{s}");
        assert!(s.num("walled") > 0.0, "a cutting past one face is walled: {s}");
    }

    #[test]
    fn the_seam_holds_between_the_ring_s_own_vertices() {
        // A straight road's kerb is one 400 m segment of its outline, and
        // the hill under it is a cosine, so the room's height along that
        // segment is not a straight line. Read at the segment's two ends
        // alone, the ground would miss it by metres in between. The outline
        // is split where the lattice crosses it, so both meshes draw one edge
        // and there is nothing left to close.
        let (_, s) = world("hill?amp=60&radius=200", "net:straight?len=400", None);
        assert!(s.num("contact") < 1e-6, "{s}");
        assert_eq!(s.num("wall_m2"), 0.0, "a gentle hill has no step to close: {s}");
    }
}
