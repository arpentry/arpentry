//! The bench: the room and the ground closed into one surface.
//!
//! The lift put the room at its height and the earthwork benched the ground
//! to it. Where the two cannot meet — a drop past one face of batter, the
//! mouth of a tube, a kerb, two roads' domains at different heights — this
//! step draws the face that closes the gap, because a step nothing spans is
//! a hole you can see the world through (invariant I9 of docs/GENERATION.md
//! §7). Then it hands on the
//! room and the ground as the meshes the structure step and the renderers
//! read.
//!
//! **Two rules, both over the one mesh's own edges.** The ground's is
//! `wall`: swept along the outline, between the room's copy of each vertex
//! and the ground's. Every other boundary inside the paving is the edge rule
//! (`edge_faces`): welded where its two sides answer with one height, and
//! a quad between them where they do not.

use crate::copies::{Boundary, Key, Rule, Surface, NONE};
use crate::lattice::height_at;
use crate::portal::Portals;
use crate::step::Summary;
use crate::world::{Arrangement, Bench, Earthwork, Lifted, Mesh, Terrain, Tri};

/// Closes the room onto the ground and onto itself, and hands on the meshes.
pub fn run(terrain: &Terrain, mesh: &Mesh, arrangement: &Arrangement, lifted: &Lifted, earthwork: &Earthwork) -> (Bench, Summary) {
    let copies = &earthwork.copies;
    let positions = &mesh.tri.positions;
    let ground_h = |v: u32| {
        copies.height(v, (Surface::Ground, Rule::FREE)).unwrap_or_else(|| {
            let q = positions[v as usize];
            earthwork.ground.at([q[0], q[1]], height_at(terrain, q[0], q[1]))
        })
    };
    let (wall, wall_m2) = wall(&earthwork.outline, &earthwork.top, positions, &ground_h, &arrangement.portals);
    // **One edge rule for every boundary inside the paving**, over
    // the one mesh's own edges and read by index — between two surfaces, and
    // between two rules of one surface that did not weld.
    let (kerb, edges) = edge_faces(&lifted.bounds, arrangement, positions, &|v, k| copies.height(v, k));
    let bench = Bench {
        carriageway: compact(&copies.carriageway.tri),
        pavement: compact(&copies.pavement.tri),
        ballast: compact(&copies.ballast.tri),
        ground: copies.ground.tri.clone(),
        wall,
        kerb,
    };
    let summary = Summary::new()
        .with_m2("wall_m2", wall_m2)
        .with_m2("kerb_m2", edges.m2)
        .with("kerb_max", format!("{:.2}", edges.max))
        .with_m2("sheet_m2", edges.sheet_m2)
        .with_m2("split_m2", edges.split_m2);
    (bench, summary)
}

/// `tri` with the positions no triangle names dropped: a copy welded into
/// another leaves its slot behind, and a glTF's vertex buffer pays for every
/// slot whether a triangle names it or not.
fn compact(tri: &Tri) -> Tri {
    let mut tri = Tri { positions: tri.positions.clone(), indices: tri.indices.clone() };
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
/// tall the batter takes one face of it (the earthwork's
/// [`crate::ground::Ground`] clamps its pin to
/// [`crate::standard::MAX_BATTER_FACE_M`]), and this face closes the rest:
/// without it, **a hole you could see the world through**, which is what
/// invariant I9 forbids.
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
    // would stand apart by the fall and the world show through between them.
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
        m2 += tri.face(a, b, [p0.0, p0.1], [p1.0, p1.1]);
    }
    (tri, m2)
}

/// **The edge rule**: every boundary between two paved surfaces is either
/// *welded* — its two sides answer with one height — or *split*, and then
/// the quad between them is drawn, always.
///
/// One rule over the one mesh's own edges, read by index: `height(v, face)`
/// is the height of `v`'s copy in the surface `face` is drawn in. A quad is
/// one mesh edge, not an arrangement edge: the mesh subdivides an
/// arrangement edge at every lattice crossing, so a quad per arrangement
/// edge would meet both rims in T-junctions.
///
/// The ground stays `wall`'s: it is the one boundary whose far side is not a
/// face of the paving.
fn edge_faces(
    bounds: &[Boundary],
    arrangement: &Arrangement,
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
        // over can share an edge in plan, and a quad between them would be a
        // curtain hung from the viaduct to the street. What closes a deck's
        // side is its slab
        // ([`crate::standard::DECK_THICKNESS_M`]), and under the slab is
        // air. A step within the slab's depth across the same boundary is a
        // kerb at an abutment or along a deck, and is drawn.
        //
        // **Either face over a span, not exactly one.** The street's own
        // sidewalk beside the viaduct is within the room's reach of the
        // deck, so the walk the deck *carries* claims it and it reads as
        // over a span too.
        let slab = crate::standard::DECK_THICKNESS_M;
        let (fa, fb) = (arrangement.face(e.a), arrangement.face(e.b));
        if (fa.spanned || fb.spanned) && ((aa - ba).abs() > slab || (ab - bb).abs() > slab) {
            continue;
        }
        // Faced toward the lower side: `u → v` runs with `a` on its left, so
        // the edge is walked the other way when `b` is the higher.
        let (p, q) = (positions[e.u as usize], positions[e.v as usize]);
        // **Where the two sides cross, the face is two triangles meeting at
        // the crossing.** One quad from the higher to the lower height at
        // each end has rails that are neither surface's rim when `a` is the
        // higher at one end and `b` at the other: its top rail runs from
        // one surface to the other, and both rims were left open — a bowtie
        // of sky along the edge, up to 0.85 m wide, and 128 of the loop
        // box's 145 `gap`s in the kerb layer. At the crossing the two rims
        // are at one height, so each half closes its own step and faces its
        // own lower side.
        let (da, db) = (aa - ba, ab - bb);
        let m2 = if da * db < 0.0 {
            let t = da / (da - db);
            let m = [p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t, 0.0];
            let hm = aa + (ab - aa) * t;
            let half = |tri: &mut Tri, s: [f64; 3], hi: f64, lo: f64, high_on_left: bool| {
                if high_on_left {
                    tri.face(s, m, [hi, lo], [hm, hm])
                } else {
                    tri.face(m, s, [hm, hm], [hi, lo])
                }
            };
            half(&mut tri, p, aa.max(ba), aa.min(ba), da > 0.0) + half(&mut tri, q, ab.max(bb), ab.min(bb), db < 0.0)
        } else {
            let (p, q, ph, pl, qh, ql) = if aa + ab >= ba + bb {
                (p, q, aa.max(ba), aa.min(ba), ab.max(bb), ab.min(bb))
            } else {
                (q, p, ab.max(bb), ab.min(bb), aa.max(ba), aa.min(ba))
            };
            tri.face(p, q, [ph, pl], [qh, ql])
        };
        let (ph, pl, qh, ql) = (aa.max(ba), aa.min(ba), ab.max(bb), ab.min(bb));
        out.m2 += m2;
        out.max = out.max.max(ph - pl).max(qh - ql);
        if ka.0 == kb.0 {
            out.split_m2 += m2;
        } else if fa.material == fb.material && fa.sheet != fb.sheet {
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
    /// that meet at different heights.
    sheet_m2: f64,
    /// The area drawn inside one surface, between two rules that did not
    /// weld: the retaining face where two roads' domains meet at different
    /// heights, or where the far pavement stops standing on its face and
    /// drapes.
    split_m2: f64,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::earthwork::tests::{ground_on, outline_of};
    use crate::pipeline::tests::stood as world;
    use crate::poly::{self, Pt};
    use crate::portal::Portals;
    use crate::standard::KERB_RISE_M;
    use crate::world::World;

    fn bench(w: &World) -> &Bench {
        w.bench.as_ref().unwrap()
    }

    #[test]
    fn a_step_no_batter_can_run_is_closed_by_a_wall() {
        // A road along the lip of a 10 m cliff: its room reaches six metres
        // to each side, so one edge stands five metres over the ground and
        // the other five under it — more than `MAX_BATTER_FACE_M` either way,
        // so a batter takes one face of it and a wall the other two metres.
        // Unwalled, that step would be a hole you could see the world
        // through (I9).
        //
        // The specimen is a cliff rather than an overpass because an
        // overpass does not wall: past `DECK_STANDOFF_M` its approach is a
        // deck. A wall is what the ground owes where the road is *not* a
        // structure, and a cliff is that.
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
        // The pavement stands KERB_RISE_M over the road beside it, so the
        // two meshes meet in plan and not in the vertical: the kerb's face
        // closes the gap. A 200 m road with one sidewalk is 200 m of kerb at
        // 0.12 m, which is 24 m².
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
    /// up to the floor, without which the terrain's edge and the tube would
    /// stand apart by the fall and the world show through between them.
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

    #[test]
    fn the_bench_is_a_function_of_the_world() {
        let (a, _) = world("hill?amp=60&radius=400", "net:cross?len=400", None);
        let (b, _) = world("hill?amp=60&radius=400", "net:cross?len=400", None);
        let (x, y) = (bench(&a), bench(&b));
        assert_eq!(x.carriageway.positions, y.carriageway.positions);
        assert_eq!(x.pavement.positions, y.pavement.positions);
        assert_eq!(x.ground.positions, y.ground.positions);
        assert_eq!(x.ground.indices, y.ground.indices);
    }
}
