//! The bench: the room and the ground closed into one surface.
//!
//! The lift put the room at its height and the earthwork benched the ground
//! to it. Where the two cannot meet — a drop past one face of batter, the
//! mouth of a tube, a kerb, two roads' domains at different heights — this
//! step draws the face that closes the gap, because a step nothing spans is
//! a hole you can see the world through (invariant 9). Then it hands on the
//! room and the ground as the meshes the structure step and the renderers
//! read.
//!
//! **Two rules, both over the one mesh's own edges.** The ground's is
//! [`wall`]: swept along the outline, between the room's copy of each vertex
//! and the ground's. Every other boundary inside the paving is the edge rule
//! ([`edge_faces`]): welded where its two sides answer with one height, and
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
    // **One edge rule for every boundary inside the paving** (§3.3), over
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
        steps: earthwork.steps.clone(),
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
        // over can share an edge in plan, and a quad between them was a
        // curtain hung from the viaduct to the street — 45 to 59 m of it at
        // the Viaduc de Chillon. What closes a deck's side is its slab
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
    /// that meet at different heights. The position lookup this replaced
    /// read both from one map and so never drew these at all.
    sheet_m2: f64,
    /// The area drawn inside one surface, between two rules that did not
    /// weld: the retaining face where two roads' domains meet at different
    /// heights, or where the far pavement stops standing on its face and
    /// drapes. A step there used to be a stretched triangle nothing closed.
    split_m2: f64,
}

#[cfg(test)]
pub(crate) mod tests {
    use crate::world::World;
    use crate::pipeline::tests::{built, upto};
    use crate::step::Step;
    

    
    use crate::world::Solved;
    

    use super::*;
    use crate::copies::*;
    use crate::field::{at, seam, Field, Foot, Ground};
    use crate::poly::{self, Pt, Shapes};
    use crate::standard::{EARTHWORK_BATTER, KERB_RISE_M, MAX_BENCH_FACE_M, ROOM_REACH_M};
    use crate::world::{Material, Profile};

    /// A world on `ground` with the network of `net` and the buildings of
    /// `houses`, meshed and benched.
    pub(crate) fn world(ground: &str, net: &str, houses: Option<&str>) -> (World, Summary) {
        let (w, ran) = built(ground, net, houses, 5.0, &upto(Step::Bench));
        (w, ran.merged(&crate::pipeline::tests::BENCH))
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
            assert!((p[2] - crate::lattice::height_at(t, p[0], p[1])).abs() < 1e-9, "{p:?}");
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
        // One edge is left, and it is a centimetre wide: where the band's
        // face meets its drape *on the outline*, two edges pin one vertex
        // differently and the ground has one copy there, so the earthwork is
        // two-valued at that point. Continuing the split into the ground is
        // what removes it; until then it is counted, and bounded.
        assert!(s.num("step") <= 1.0 && s.num("worst") < 0.6, "{s}");
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
        // And nothing in the pavement stands up: the steepest triangle is the
        // far half following the ground's batter — the 50 % flank, 1 in 2.5
        // of residual on it, and a little more where two outline segments
        // hand over and the blend's weight turns (0.993 here) — not the
        // 4.255 m the field steps by, which is a face.
        let steepest = b
            .pavement
            .indices
            .chunks_exact(3)
            .map(|t| {
                let [p, q, r] = [t[0], t[1], t[2]].map(|i| b.pavement.positions[i as usize]);
                let (u, v) = ([q[0] - p[0], q[1] - p[1], q[2] - p[2]], [r[0] - p[0], r[1] - p[1], r[2] - p[2]]);
                let n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
                if n[2].abs() < 1e-12 { 0.0 } else { n[0].hypot(n[1]) / n[2].abs() }
            })
            .fold(0.0f64, f64::max);
        assert!(steepest < 1.0, "a pavement triangle climbs {steepest:.3}: {s}");
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
        let top = crate::lattice::height_at(t, 0.0, 0.0);
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
        let natural = move |p: Pt| crate::lattice::height_at(terrain, p[0], p[1]);
        let b = w.bench.as_ref().unwrap();
        let seam = seam(&[&b.carriageway, &b.pavement]);
        let outline = poly::union_of(&[
            &w.legs.as_ref().expect("the legs step ran").surface.carriageway,
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
        let profiles = &w.solved().unwrap().profiles;
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
