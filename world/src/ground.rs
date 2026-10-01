//! The engineered ground: the natural ground with the earthwork the lifted
//! room owes it, as a function of the point.
//!
//! The earthwork step builds it from the room's outline and the bench reads
//! it where it closes the room onto the ground, so it is a module of its own
//! rather than either step's.

use std::cell::Cell;
use std::collections::{BTreeMap, HashMap, VecDeque};

use serde::{Deserialize, Serialize};

use crate::line::{fade, Nearest};
use crate::poly::{self, Pt};
use crate::relax::{relax, Pin};
use crate::standard::{EARTHWORK_BATTER, MAX_BATTER_FACE_M};

/// The engineered ground: the natural ground plus an earthwork **residual**
/// that is pinned at the room's outline and falls to nothing at 1 in
/// [`EARTHWORK_BATTER`].
///
/// At an outline vertex the residual is the room's height less the natural
/// ground, held to one face ([`MAX_BATTER_FACE_M`]) either way; a wall at the
/// edge closes whatever is left over. Out from each outline segment it falls
/// at the batter's slope, perpendicular to the segment, and a point takes the
/// nearest segment's batter blended with any segment nearly as near — over
/// `EARTH_BLEND_M`, narrowing to nothing at the outline, so on the outline
/// it is the pins' own interpolation and the whole ground is one function.
/// Three things follow:
///
/// - **It is continuous.** The nearest segment alone would step wherever two
///   segments are equidistant and answer differently — the inside of every
///   kerb return — and between a walled segment and a battered one, whose
///   pins differ by the whole of a drop. The blend removes the first, and a
///   pin that is clamped rather than refused removes the second: a drop past
///   one face is a face of batter and a wall for the rest, not all wall or
///   none.
/// - **It always daylights, within 7.5 m.** A pin is at most one face, so its
///   batter is spent at `B · MAX_BATTER_FACE_M`. A face at an *absolute*
///   1 in 2.5 would never meet a hill steeper than that, and would have to be
///   cut off with a lip standing in it.
/// - **Pins do not reach along the kerb.** A cone from every pin (the
///   steepest-allowed extension) would couple neighbouring pins: where a
///   kerb's cut changes by more than a batter's slope along its own length,
///   one pin's cone overrides the next, and each override is a wall at the
///   kerb.
///
/// **The batter is relative to the natural ground, not absolute.** On flat
/// ground the two are the same 1 in 2.5. Across a 30 % hill the cut face
/// stands at 70 % and meets the ground 2 m out; at an absolute 1 in 2.5 it
/// never would.
///
/// **Where two pins that disagree face each other, it is a ramp**
/// ([`Ground::ramp`]): the blend can only hand one batter over to the other
/// in a fold a metre wide, so around every fold the field is the relax's
/// harmonic surface between the pins instead, and the batter everywhere
/// else.
///
/// It is a function of the point, not a mesh, so anything may be *proven*
/// to lie on it, which is the property the terrain step set out with and
/// the one the crate is built on. The ramp is too: linear across each
/// triangle it was solved on, and the batter's own value on its rim.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Ground {
    at: Nearest,
    /// Per outline segment, the residual at its two ends.
    e: Vec<(f64, f64)>,
    /// Where the batter folded, the relaxed residual that replaces it.
    ramp: Ramp,
}

/// How far, in metres, a batter reaches before it is spent: one face at the
/// batter's slope.
const EARTH_REACH_M: f64 = EARTHWORK_BATTER * MAX_BATTER_FACE_M;

/// The band, in metres of distance, over which the batter of one outline
/// segment hands over to the next where the nearest changes ([`Ground`]).
const EARTH_BLEND_M: f64 = 1.0;

/// What a metre behind an outline segment, across the room from it, adds to
/// the distance its batter is read at ([`Ground::residual`]): enough that the
/// batter falls there at 1 in 1 — the steepest face the lift lets a surface
/// stand ([`crate::earthwork::STEP_GRADE`]) — instead of 1 in
/// [`EARTHWORK_BATTER`], so a pin of one face is spent one face behind.
const BEHIND_M: f64 = EARTHWORK_BATTER - 1.0;

/// The steepest the batter draws the residual, in metres per metre: behind
/// an outline segment, at 1 in 1 ([`BEHIND_M`]). A footpath triangle steeper
/// than this by more than [`FOLD_SLACK`] is not on a batter but on the fold
/// where two batters hand over ([`Ground::ramp`]).
const FOLD_GRADE: f64 = 1.0;

/// What a fold must exceed [`FOLD_GRADE`] by: where two segments that agree
/// hand over, the blend's weight turns a hair steeper than either batter
/// (0.993 against 1 in 1 on `a_band_across_the_reach_steps_on_a_rim_not_inside_itself`),
/// and that is not a fold.
const FOLD_SLACK: f64 = 0.05;

/// A triangle smaller than this, in square metres, seeds no ramp: the
/// census's speck, under which a gradient is the mesh's and not the field's.
const SEED_M2: f64 = 0.01;

/// How far, in metres, a ramp reaches from the fold that seeded it, through
/// the field's own triangles and never across a pin: as far as a batter
/// does, so that where the ramp's rim cuts across a footpath the batter
/// there is the one the ramp itself would have drawn.
///
/// Measured over the 48 Montreux sites, pavement fins by reach: 1 m
/// 212.1 m², 2 m 129.1, 3 m 119.8, 4.5 m 125.4, 6 m 125.8, **7.5 m 125.3**,
/// 10 m 128.8 — under 2 m the region no longer spans the strip it folds
/// across, and past it the reach barely matters. What decides it is
/// `a_footpath_between_two_pins_is_a_ramp_not_a_fold`: at 3 m the rim
/// crossed that footway while its batter still climbed 1 in 1 across it,
/// and the triangle between the held rim and the relaxed ramp stood at
/// 1.56 m/m where its pins needed 1.0. Growing the region along the whole
/// footpath instead, whatever the distance, read 126.0 to 130.6.
const RAMP_REACH_M: f64 = EARTH_REACH_M;

/// The most a cotangent weighs: a needle's long edges face angles near
/// nothing, and uncapped they would tie its corners together a thousandfold
/// harder than any real edge does.
const COT_MAX: f64 = 100.0;

/// What every edge of the ramp weighs at least, so the clamp of an obtuse
/// angle's negative cotangent leaves no vertex out of the solve.
const EDGE_FLOOR: f64 = 1e-6;

/// The side of the ramp's lookup cells, in metres: a bucket, nothing about
/// the answer.
const RAMP_CELL_M: f64 = 4.0;

/// The ground's field as the one mesh draws it: every triangle whose
/// vertices take the engineered ground ([`Ground::ramp`]).
pub struct Domain<'a> {
    /// Per one-mesh vertex, its plan position.
    pub plan: &'a [Pt],
    /// The triangles the field answers for, as one-mesh vertices: the
    /// ground's and the passive pavement's.
    pub tris: &'a [[u32; 3]],
    /// Per triangle, whether it is passive pavement.
    pub path: &'a [bool],
    /// Per one-mesh vertex, whether the ramp must keep the batter's own value
    /// at it: where the ground meets something that is not the field — the
    /// outline's pins, a deck's paving over it — and on the rect's border.
    pub held: &'a [bool],
}

/// The relaxed residual where the batter folded: the triangles it was
/// solved on, with the residual at their corners.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
struct Ramp {
    tris: Vec<[Pt; 3]>,
    e: Vec<[f64; 3]>,
    cells: HashMap<(i32, i32), Vec<u32>>,
}

impl Ramp {
    fn push(&mut self, p: [Pt; 3], e: [f64; 3]) {
        let i = self.tris.len() as u32;
        let b = [
            p[0][0].min(p[1][0]).min(p[2][0]),
            p[0][1].min(p[1][1]).min(p[2][1]),
            p[0][0].max(p[1][0]).max(p[2][0]),
            p[0][1].max(p[1][1]).max(p[2][1]),
        ];
        for cell in poly::cells_over(b, RAMP_CELL_M) {
            self.cells.entry(cell).or_default().push(i);
        }
        self.tris.push(p);
        self.e.push(e);
    }

    /// The residual at `p`, if a triangle of the ramp holds it: linear
    /// across the triangle, which is what the mesh draws there.
    fn at(&self, p: Pt) -> Option<f64> {
        for &i in self.cells.get(&poly::cell_of(p, RAMP_CELL_M))? {
            let ([a, b, c], e) = (self.tris[i as usize], self.e[i as usize]);
            let det = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
            if det.abs() < 1e-12 {
                continue;
            }
            let l = [
                ((b[0] - p[0]) * (c[1] - p[1]) - (b[1] - p[1]) * (c[0] - p[0])) / det,
                ((c[0] - p[0]) * (a[1] - p[1]) - (c[1] - p[1]) * (a[0] - p[0])) / det,
                ((a[0] - p[0]) * (b[1] - p[1]) - (a[1] - p[1]) * (b[0] - p[0])) / det,
            ];
            if l.iter().all(|&x| x >= -1e-9) {
                return Some(l[0] * e[0] + l[1] * e[1] + l[2] * e[2]);
            }
        }
        None
    }
}

/// How steep `z` stands across the plan triangle `p`, in metres per metre;
/// nothing for a triangle with no area to stand on.
pub fn grade_of(p: [Pt; 3], z: [f64; 3]) -> Option<f64> {
    let (ux, uy) = (p[1][0] - p[0][0], p[1][1] - p[0][1]);
    let (vx, vy) = (p[2][0] - p[0][0], p[2][1] - p[0][1]);
    let det = ux * vy - uy * vx;
    if det.abs() < 1e-12 {
        return None;
    }
    let (d1, d2) = (z[1] - z[0], z[2] - z[0]);
    Some(((d1 * vy - d2 * uy) / det).hypot((ux * d2 - vx * d1) / det))
}

/// A triangle's plan area, in square metres.
fn area_of(p: [Pt; 3]) -> f64 {
    ((p[1][0] - p[0][0]) * (p[2][1] - p[0][1]) - (p[1][1] - p[0][1]) * (p[2][0] - p[0][0])).abs() / 2.0
}

impl Ground {
    /// The ground benched to the room's outline: one segment per outline
    /// edge of the one mesh, as `(a, b, [room, natural] at a, the same at
    /// b)`. A segment across a tunnel's mouth is pinned at no residual — no
    /// batter runs into the tube — and the wall over the mouth closes the
    /// hill down to its roof.
    pub fn of_edges(edges: &[(Pt, Pt, [f64; 2], [f64; 2])], portals: &crate::portal::Portals) -> Ground {
        let mut g = Ground::default();
        let pin = |[room, natural]: [f64; 2]| (room - natural).clamp(-MAX_BATTER_FACE_M, MAX_BATTER_FACE_M);
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
        self.ramp.at(p).unwrap_or_else(|| self.batter(p).0)
    }

    /// The batters' residual at `p`, and whether any pin reaches it at all:
    /// a point every batter has spent itself before is the natural ground,
    /// and the ramp may not lift it.
    fn batter(&self, p: Pt) -> (f64, bool) {
        let mut near: Vec<(f64, f64)> = Vec::new();
        self.at.within(p, EARTH_REACH_M + EARTH_BLEND_M, |i, t, d| {
            // **A batter does not cross the room.** The outline runs with the
            // room on its left, so a point whose foot is inside a segment and
            // which lies on the segment's left is across the paving from it.
            // How deep it is, `behind`, is the least of its distance off the
            // segment's line and its distance in from either end, so it is
            // nothing on the segment, on its right and beyond its ends — where
            // the batter is its end vertex's, which a neighbouring segment
            // shares — and grows continuously from there. The segment's batter
            // is spent across the room at 1 in 1 rather than 1 in
            // `EARTHWORK_BATTER` ([`BEHIND_M`]), which reaches nothing past one
            // face.
            //
            // **It was a hard cut**, and a cut is a jump for any point on the
            // room's side of a segment that asks the ground at all: passive
            // pavement, which is paving but takes this field, lies there
            // wherever it meets the room beyond a kerb's end. On `net:stub`
            // over a 30 % ramp two corners of one footway triangle, 0.46 m
            // apart either side of the line through a kerb's end, read -1.63
            // and -2.28, and the census charged the triangle as a fin
            // (0.70 m²).
            let (a, b) = self.at.seg[i];
            let len = (b[0] - a[0]).hypot(b[1] - a[1]);
            let left = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]);
            let behind = if len > 0.0 { (left / len).min(t.min(1.0 - t) * len).max(0.0) } else { 0.0 };
            let d = d + BEHIND_M * behind;
            let (ea, eb) = self.e[i];
            let e = ea + (eb - ea) * t;
            near.push((e.signum() * (e.abs() - d / EARTHWORK_BATTER).max(0.0), d));
        });
        let Some(nearest) = near.iter().map(|x| x.1).min_by(f64::total_cmp) else {
            return (0.0, false);
        };
        let reached = near.iter().any(|x| x.0 != 0.0);
        // The segments nearly as near as the nearest, blended by how nearly:
        // continuous where the nearest changes, which is where the nearest
        // alone would step. Sorted first, so the sum is a function of the
        // segments and not of the index's visiting order.
        near.sort_by(|x, y| x.1.total_cmp(&y.1).then(x.0.total_cmp(&y.0)));
        // **The band narrows to nothing at the outline**, so there the
        // nearest segment alone answers — the linear interpolation of its own
        // pins, which is the edge the mesh drew — and a vertex pinned on it is
        // the limit of the field around it. At a full metre everywhere, a
        // segment a few decimetres off with a different pin would be blended
        // into the outline itself, and the ground would stand a jump off
        // every pin.
        let band = EARTH_BLEND_M.min(nearest);
        let (mut sum, mut weight) = (0.0, 0.0);
        for (e, d) in near {
            let w = if band > 0.0 { fade((d - nearest) / band) } else { (d <= nearest) as u8 as f64 };
            sum += w * e;
            weight += w;
        }
        (sum / weight, reached)
    }

    /// The engineered height at `p`, whose natural ground is `natural`.
    pub fn at(&self, p: Pt, natural: f64) -> f64 {
        natural + self.residual(p)
    }

    /// **Where two pins that disagree face each other across a strip, the
    /// residual between them is a ramp, not a fold.**
    ///
    /// The blend hands the nearest segment's batter over to the next over
    /// [`EARTH_BLEND_M`], which is right where the two nearly agree — the
    /// inside of a kerb return — and wrong where they face each other across
    /// a strip narrower than their batters: a kerb pinned three metres of
    /// fill four metres from ballast pinned at none. There each batter runs
    /// down its own side at 1 in 2.5 until the two are equidistant, and the
    /// whole of what is left between them is spent inside the blend's metre:
    /// a fold at two to four metres per metre, and on a footpath a fin. A
    /// wider blend was measured and does not remove it — it only moves the
    /// fold.
    ///
    /// So around every fold the residual is the relax's harmonic surface
    /// (`w = 0`: the least `∫ ‖∇e‖²`), held at the batter's own value
    /// wherever the ground meets something that is not the field — the
    /// outline's pins, a deck's paving over it — on the rect's border and at
    /// the rim of the region, so the field stays one continuous function; and
    /// at zero wherever no pin reaches, so ground no pin reaches is still the
    /// DEM to the bit.
    /// Between two parallel pins that is the straight ramp from one to the
    /// other, the least steep any surface meeting both can be.
    ///
    /// **The footpath takes its share of the slope.** Weighting its
    /// triangles above the ground's in the energy flattens the path and
    /// steepens the bank beside it: over the 48 Montreux sites a weight of
    /// ten leaves 93.8 m² of pavement fins against 125.3 at one, and thirty
    /// 89.2. That is a level path above a bank as steep as the pins make it —
    /// the ground is never charged a fin, so the census would read it as
    /// better, but it is the wall's answer drawn in earth, not a ramp.
    ///
    /// **A footpath's edge against other paving is not held.** There the two
    /// are already two copies with the edge rule's face between them (a
    /// split), and the batter's value along that edge is the fold itself
    /// seen from the side: held, it kept half the fins (pavement fins over
    /// the 48 Montreux sites 188.4 m² held, 125.3 free, at a reach of 7.5 m),
    /// for 88 m² more split face.
    ///
    /// **The region is bounded to where it is needed.** A fold is a
    /// footpath triangle steeper than any batter is drawn ([`FOLD_GRADE`]);
    /// the region is every field triangle within [`RAMP_REACH_M`] of one,
    /// reached through vertices the ramp may move. Everywhere else the batter
    /// stands, and inside the region the harmonic surface of a batter is
    /// nearly the batter itself — a residual falling linearly off a straight
    /// kerb is harmonic — so what moves is the fold. Folds in the ground
    /// alone seed nothing: relaxing them too (1 895 ramps over the sites
    /// against 148) left more pavement fins, not fewer (139.8 m² against
    /// 125.3), and the gate read 39 lines worse where it reads 15 without.
    ///
    /// Returns the ramps, each as the indices of the domain's triangles it
    /// was solved on, and the vertices they solved for, in order.
    pub fn ramp(&mut self, d: &Domain) -> (Vec<Vec<usize>>, Vec<u32>) {
        let n = d.plan.len();
        let corners = |t: usize| d.tris[t].map(|v| d.plan[v as usize]);
        // The batter at a vertex, and whether a pin reaches it: asked only
        // of the vertices a fold could reach, once each — the field is most
        // of the mesh and the folds are a few hundred of its triangles.
        let batters = vec![Cell::new(None::<(f64, bool)>); n];
        let this = &*self;
        let batter = |v: u32| {
            let slot = &batters[v as usize];
            slot.get().unwrap_or_else(|| {
                let b = this.batter(d.plan[v as usize]);
                slot.set(Some(b));
                b
            })
        };
        let movable = |v: u32| !d.held[v as usize] && batter(v).1;

        // The folds.
        let seeds: Vec<usize> = (0..d.tris.len())
            .filter(|&t| {
                d.path[t]
                    && d.tris[t].iter().any(|&v| movable(v))
                    && area_of(corners(t)) >= SEED_M2
                    && grade_of(corners(t), d.tris[t].map(|v| batter(v).0)).is_some_and(|g| g > FOLD_GRADE + FOLD_SLACK)
            })
            .collect();
        if seeds.is_empty() {
            return (Vec::new(), Vec::new());
        }

        // The region: grown from the folds through the vertices the ramp may
        // move, to within the reach of the fold each triangle was reached
        // from.
        let mut first = vec![0u32; n + 1];
        for t in d.tris {
            for &v in t {
                first[v as usize + 1] += 1;
            }
        }
        for v in 0..n {
            first[v + 1] += first[v];
        }
        let mut fill = first.clone();
        let mut around = vec![0u32; first[n] as usize];
        for (t, tri) in d.tris.iter().enumerate() {
            for &v in tri {
                around[fill[v as usize] as usize] = t as u32;
                fill[v as usize] += 1;
            }
        }
        let tris_at = |v: u32| &around[first[v as usize] as usize..first[v as usize + 1] as usize];
        let centroid = |t: usize| {
            let c = corners(t);
            [(c[0][0] + c[1][0] + c[2][0]) / 3.0, (c[0][1] + c[1][1] + c[2][1]) / 3.0]
        };
        let mut from = vec![u32::MAX; d.tris.len()];
        let mut queue: VecDeque<usize> = VecDeque::new();
        for &s in &seeds {
            from[s] = s as u32;
            queue.push_back(s);
        }
        while let Some(t) = queue.pop_front() {
            let o = centroid(from[t] as usize);
            for &v in d.tris[t].iter().filter(|&&v| movable(v)) {
                for &u in tris_at(v) {
                    let u = u as usize;
                    if from[u] == u32::MAX {
                        let c = centroid(u);
                        if (c[0] - o[0]).hypot(c[1] - o[1]) <= RAMP_REACH_M {
                            from[u] = from[t];
                            queue.push_back(u);
                        }
                    }
                }
            }
        }
        let inside = |t: u32| from[t as usize] != u32::MAX;
        // A vertex the ramp solves for: one it may move, every triangle
        // round which is in the region. The rest keep the batter.
        let free = |v: u32| movable(v) && tris_at(v).iter().all(|&t| inside(t));

        // The ramps, as the region's triangles joined through free vertices,
        // each numbered by its first triangle.
        let region: Vec<usize> = (0..d.tris.len()).filter(|&t| inside(t as u32)).collect();
        let mut root: HashMap<usize, usize> = HashMap::new();
        let mut ramps: Vec<Vec<usize>> = Vec::new();
        for &t in &region {
            if root.contains_key(&t) || !d.tris[t].iter().any(|&v| free(v)) {
                continue;
            }
            let k = ramps.len();
            let mut members = vec![t];
            root.insert(t, k);
            let mut i = 0;
            while i < members.len() {
                let m = members[i];
                i += 1;
                for &v in d.tris[m].iter().filter(|&&v| free(v)) {
                    for &u in tris_at(v) {
                        if let std::collections::hash_map::Entry::Vacant(slot) = root.entry(u as usize) {
                            slot.insert(k);
                            members.push(u as usize);
                        }
                    }
                }
            }
            members.sort_unstable();
            ramps.push(members);
        }

        // One solve over every ramp: the free vertices numbered in the order
        // they are met, the held ones as pins at the batter's value.
        let mut local: HashMap<u32, u32> = HashMap::new();
        let mut global: Vec<u32> = Vec::new();
        for ramp in &ramps {
            for &t in ramp {
                for &v in &d.tris[t] {
                    local.entry(v).or_insert_with(|| {
                        global.push(v);
                        global.len() as u32 - 1
                    });
                }
            }
        }
        let mut weight: BTreeMap<(u32, u32), f64> = BTreeMap::new();
        for ramp in &ramps {
            for &t in ramp {
                let p = corners(t);
                let twice = (p[1][0] - p[0][0]) * (p[2][1] - p[0][1]) - (p[1][1] - p[0][1]) * (p[2][0] - p[0][0]);
                if twice.abs() < 1e-12 {
                    continue;
                }
                for k in 0..3 {
                    let (i, j) = ((k + 1) % 3, (k + 2) % 3);
                    let (u, w) = ([p[i][0] - p[k][0], p[i][1] - p[k][1]], [p[j][0] - p[k][0], p[j][1] - p[k][1]]);
                    let cot = (u[0] * w[0] + u[1] * w[1]) / twice.abs();
                    let (a, b) = (local[&d.tris[t][i]], local[&d.tris[t][j]]);
                    *weight.entry((a.min(b), a.max(b))).or_insert(EDGE_FLOOR) += 0.5 * cot.clamp(0.0, COT_MAX);
                }
            }
        }
        let edges: Vec<(u32, u32, f64)> = weight.into_iter().map(|((a, b), w)| (a, b, w)).collect();
        let pins: Vec<Pin> = global.iter().map(|&v| if free(v) { Pin::Free } else { Pin::At(batter(v).0) }).collect();
        let e = relax(global.len(), &edges, &pins, 0.0);
        let mut free: Vec<u32> = global.iter().copied().filter(|&v| free(v)).collect();
        free.sort_unstable();
        for ramp in &ramps {
            for &t in ramp {
                self.ramp.push(corners(t), d.tris[t].map(|v| e[local[&v] as usize]));
            }
        }
        (ramps, free)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::portal::Portals;

    /// A ground of one segment per edge, `(a, b, pin at a, pin at b)`, over
    /// a natural ground at 0.
    fn ground(edges: &[(Pt, Pt, f64, f64)]) -> Ground {
        let edges: Vec<_> = edges.iter().map(|&(a, b, ea, eb)| (a, b, [ea, 0.0], [eb, 0.0])).collect();
        Ground::of_edges(&edges, &Portals::default())
    }

    /// **The residual is continuous on the room's side of a segment too.**
    /// Passive pavement is paving that takes the ground's field, so it asks
    /// the field on the room's side of the outline: the footway beyond the
    /// end of a stub's kerb, where the kerb runs on and the footway turns in
    /// behind it. A batter simply refused there steps on the line through
    /// the kerb's end — 2 m across a fifth of a millimetre here, and a fin
    /// wherever a footway's triangle straddled it.
    #[test]
    fn the_residual_does_not_step_behind_a_kerb_s_end() {
        // A kerb along y = 0 ending at x = 10, the room north of it, pinned
        // two metres of fill.
        let g = ground(&[([0.0, 0.0], [10.0, 0.0], 2.0, 2.0)]);
        for y in [0.0, 1e-3, 0.3, 0.5, 1.0, 2.0] {
            let (before, after) = (g.residual([10.0 - 1e-4, y]), g.residual([10.0 + 1e-4, y]));
            assert!((before - after).abs() < 1e-3, "at y = {y}: {before:.4} before the kerb's end, {after:.4} past it");
        }
        // And across the segment's own line, inside its ends.
        for x in [0.01, 0.3, 5.0, 9.7, 9.99] {
            let (room, ground) = (g.residual([x, 1e-4]), g.residual([x, -1e-4]));
            assert!((room - ground).abs() < 1e-3, "at x = {x}: {room:.4} on the room's side, {ground:.4} on the ground's");
        }
    }

    /// A footpath strip `len` m long and `wide` m across, from `y = 0` to
    /// `y = wide`, as a lattice of half-metre cells: its plan, its
    /// triangles, and which vertices are held — its two long edges, where
    /// it meets the paving each pin belongs to, and its two ends.
    fn strip(len: f64, wide: f64) -> (Vec<Pt>, Vec<[u32; 3]>, Vec<bool>) {
        let h = 0.5;
        let (nx, ny) = ((len / h).round() as u32 + 1, (wide / h).round() as u32 + 1);
        let id = |i: u32, j: u32| j * nx + i;
        let plan: Vec<Pt> = (0..ny).flat_map(|j| (0..nx).map(move |i| [i as f64 * h, j as f64 * h])).collect();
        let mut tris = Vec::new();
        for j in 0..ny - 1 {
            for i in 0..nx - 1 {
                tris.push([id(i, j), id(i + 1, j), id(i + 1, j + 1)]);
                tris.push([id(i, j), id(i + 1, j + 1), id(i, j + 1)]);
            }
        }
        let held = (0..ny).flat_map(|j| (0..nx).map(move |i| i == 0 || j == 0 || i == nx - 1 || j == ny - 1)).collect();
        (plan, tris, held)
    }

    /// **Between two pins that disagree, the footpath is a ramp, not a
    /// fold.** A kerb pinned three metres of fill faces ballast pinned at
    /// none across a 4.5 m footpath. Each batter runs down its own side at
    /// 1 in 2.5 until the two are equidistant, 2.1 m still standing there,
    /// and the blend spends it in its metre: the fold. Relaxed, the path is
    /// the straight ramp from one pin to the other — 3 m over 4.5, 0.67 m/m
    /// — which no surface meeting both can be gentler than; and it is one
    /// continuous function, the batter's own at its rim.
    #[test]
    fn a_strip_between_facing_pins_is_a_ramp_not_a_fold() {
        let (len, wide) = (20.0, 4.5);
        // The kerb along y = 0 with its room to the south, the ballast along
        // y = 4.5 with its room to the north: each on its segment's left.
        let mut g = ground(&[([len, 0.0], [0.0, 0.0], 3.0, 3.0), ([0.0, wide], [len, wide], 0.0, 0.0)]);
        let across = |g: &Ground, x: f64| {
            let h = 0.01;
            (0..(wide / h) as usize)
                .map(|k| (g.residual([x, (k + 1) as f64 * h]) - g.residual([x, k as f64 * h])).abs() / h)
                .fold(0.0, f64::max)
        };
        // The batter alone folds.
        assert!(across(&g, len / 2.0) > 2.0, "no fold to relax: {:.2} m/m", across(&g, len / 2.0));

        let (plan, tris, held) = strip(len, wide);
        let path = vec![true; tris.len()];
        let (ramps, free) = g.ramp(&Domain { plan: &plan, tris: &tris, path: &path, held: &held });
        assert_eq!(ramps.len(), 1, "one strip, one ramp");
        assert!(!free.is_empty());
        let need = 3.0 / wide;
        // Away from the strip's ends — held at the batter, which folds there
        // too — every triangle is the ramp, to the solve's tolerance.
        for t in &tris {
            let p = t.map(|v| plan[v as usize]);
            if p.iter().any(|q| q[0] < 6.0 || q[0] > len - 6.0) {
                continue;
            }
            let grade = grade_of(p, p.map(|q| g.residual(q))).unwrap();
            assert!((grade - need).abs() < 0.02, "a triangle at {:?} stands at {grade:.3} m/m, not {need:.3}", p[0]);
        }
        // Continuous, sampled a centimetre apart across the strip and a
        // metre past each pin, where the batter takes over from the ramp.
        for x in [6.0, 10.0, 13.3] {
            let h = 0.01;
            for k in -100..550 {
                let (a, b) = (g.residual([x, k as f64 * h]), g.residual([x, (k + 1) as f64 * h]));
                assert!((a - b).abs() < 0.02, "a jump of {:.3} at ({x}, {:.2})", (a - b).abs(), k as f64 * h);
            }
        }
        // The pins are the pins, and nothing past the ballast is lifted.
        assert!((g.residual([10.0, 0.0]) - 3.0).abs() < 1e-9);
        assert!(g.residual([10.0, wide]).abs() < 1e-9);
        assert_eq!(g.residual([10.0, wide + 8.0]), 0.0);
    }

    /// **Ground no pin reaches is the DEM to the bit, ramp or not.** A strip
    /// that runs on past every batter's reach relaxes only where a pin
    /// reaches: the vertices every batter has spent itself before are held at
    /// nothing, and read exactly that, inside the ramp as outside it.
    #[test]
    fn a_ramp_lifts_no_ground_no_pin_reaches() {
        let (len, wide) = (40.0, 4.5);
        // The same two pins, over the strip's first ten metres only.
        let mut g = ground(&[([10.0, 0.0], [0.0, 0.0], 3.0, 3.0), ([0.0, wide], [10.0, wide], 0.0, 0.0)]);
        let (plan, tris, held) = strip(len, wide);
        let path = vec![true; tris.len()];
        let (ramps, _) = g.ramp(&Domain { plan: &plan, tris: &tris, path: &path, held: &held });
        assert!(!ramps.is_empty());
        let mut asked = 0;
        for q in plan.iter().filter(|&&q| !g.batter(q).1) {
            asked += g.ramp.at(*q).is_some() as usize;
            assert_eq!(g.residual(*q), 0.0, "the ramp lifted ground at {q:?}");
        }
        assert!(asked > 0, "the ramp reached no vertex the batters had spent themselves before");
    }

    /// Nowhere about a kerb's end does the residual climb faster than one
    /// in one: in front of it the batter falls at 1 in `EARTHWORK_BATTER`,
    /// behind it at 1 in 1, and between the two it is continuous. Sampled
    /// on a grid 5 cm apart.
    #[test]
    fn the_residual_about_a_kerb_s_end_is_no_steeper_than_one_in_one() {
        let g = ground(&[([0.0, 0.0], [10.0, 0.0], 3.0, 3.0)]);
        let h = 0.05;
        let mut worst = (0.0f64, [0.0, 0.0]);
        for i in 0..=200 {
            for j in 0..=200 {
                let p = [5.0 + i as f64 * h, -5.0 + j as f64 * h];
                let r = g.residual(p);
                for q in [[p[0] + h, p[1]], [p[0], p[1] + h]] {
                    let grade = (g.residual(q) - r).abs() / h;
                    if grade > worst.0 {
                        worst = (grade, p);
                    }
                }
            }
        }
        assert!(worst.0 <= 1.0 + 1e-6, "the residual climbs {:.3} m/m at {:?}", worst.0, worst.1);
        // Behind the kerb the pin is spent one face in, not a batter's run.
        assert_eq!(g.residual([5.0, 3.0]), 0.0);
        assert!((g.residual([5.0, 1.0]) - 2.0).abs() < 1e-9);
    }
}

