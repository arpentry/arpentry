//! Step 6: the kerb returns.
//!
//! At a junction the concave corner between two legs is rounded: the curb
//! return, the fillet that a real kerb wears because a vehicle cannot turn a
//! right angle. The surface step's union has a sharp notch there, and it is
//! the one place a road outline made of buffered lines still looks like
//! buffered lines.
//!
//! A morphological **closing** — dilate by `r`, erode by `r` — rounds
//! exactly those concave corners and nothing else that is already convex.
//! Applied everywhere it would also bridge any two regions closer than
//! `2r`: a dual carriageway's median, two streets across a narrow block, the
//! inside of a road that hooks back on itself. So it is done **per
//! corner**: a corner is a vertex where the carriageway's boundary turns
//! inward by [`BEND_MIN_DEG`] or more — which a junction's notch has and a
//! bend, drawn as a chain of small turns, never has — and the return of
//! radius `r` at a corner of turn `τ` lies in the triangle between the
//! corner and its two tangent points, `r·tan(τ/2)` along either kerb (at
//! most [`RETURN_MAX_RADII`] radii: sharper is two kerbs grazing). The
//! closing is computed over the carriageway near those triangles, and what
//! it gained is clipped to them, so nothing else survives.
//!
//! The radius is a class prior ([`crate::width::fillet_m`]) of the
//! *narrower* of the two ways whose kerbs meet at the corner: the return
//! is the turn a vehicle makes between them, and a driveway on a primary
//! gets the driveway's return, not the primary's.
//!
//! The asphalt still wins: the pavement is re-cut by the fillets, so the
//! kerb a sidewalk stands on is the filleted one. `kerb_gap` is measured
//! again on the result, because the invariant is the final surface's.

use std::collections::HashMap;

use crate::kerb::{self, RoadIndex};
use crate::poly::{self, Pt, Shape, Shapes};
use crate::step::Summary;
use crate::width::{self, Family};
use crate::world::{Fillet, Polyline2, World};

/// A vertex where the kerb turns inward by less than this, in degrees, is
/// a bend drawn as a chain of turns, not a corner worth a return.
pub const BEND_MIN_DEG: f64 = 30.0;

/// How far from a corner a way's kerb may pass and still be one of the
/// two kerbs meeting there, in metres: the lattice and the union's
/// intersection vertices are exact to a millimetre.
const KERB_TOL_M: f64 = 0.02;

/// The opening that separates a fillet from the closing's noise, in metres.
/// A closing does not return a boundary of polygonal arcs exactly: it
/// leaves hairline slivers up to a decimetre wide along it (the chord
/// sagitta of an arc at the dilated radius). What the closing gained is cut
/// back by this from the kerb and grown again, which erases anything
/// thinner than twice it and blunts a real fillet's cusps by as much.
const OPEN_M: f64 = 0.1;

/// How far past the opening a fillet is grown back, in metres, so that it
/// overlaps the kerb it belongs to rather than touching it: a boolean union
/// keeps touching shapes apart, and a touch is a hairline hole.
const OVERLAP_M: f64 = 0.01;

/// A hole in the asphalt smaller than this, in square metres, is not a
/// feature the data could describe; it is a sliver the opening left where
/// a fillet meets the triangle's edge, and it is filled.
pub const HOLE_MIN_M2: f64 = 0.5;

/// Slack on a corner's triangle, in metres: the closing's polygonal arc
/// and the opening's regrowth reach a little past the exact tangent
/// points, and a little outside the kerbs.
const CORNER_SLACK_M: f64 = 0.5;

/// The longest a return runs along a kerb from its corner, in radii. The
/// tangent points recede as `tan(τ/2)`, without bound as the kerbs come
/// to meet head-on: a corner sharper than about 143° is two kerbs grazing
/// each other, and its return stops here.
const RETURN_MAX_RADII: f64 = 3.0;

/// A corner of the kerb: a vertex where the carriageway's boundary turns
/// inward, by how much, the directions the two kerbs leave it in, and the
/// radius of the return it gets.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Corner {
    pub at: Pt,
    pub turn_deg: f64,
    /// Unit directions from the corner along the kerb before it and after it.
    pub along: [Pt; 2],
    pub radius_m: f64,
}

impl Corner {
    /// The triangle a return of radius `r` lies in: the corner and its two
    /// tangent points, `r·tan(τ/2)` along either kerb, grown by the slack.
    /// Counter-clockwise; `None` if it has no area.
    fn wedge(&self) -> Option<Shape> {
        let along = (self.turn_deg.to_radians() / 2.0).tan().min(RETURN_MAX_RADII);
        let len = self.radius_m * along + CORNER_SLACK_M;
        let [u, v] = self.along;
        let tri = vec![self.at, [self.at[0] + u[0] * len, self.at[1] + u[1] * len], [self.at[0] + v[0] * len, self.at[1] + v[1] * len]];
        poly::dilate_sharp(&vec![vec![poly::oriented(tri, true)]], CORNER_SLACK_M).into_iter().next()
    }
}

/// Rounds the world's corners.
pub fn run(world: &mut World) -> Summary {
    let roads = world.roads.as_ref().expect("the drape step runs first");
    let surface = world.surface.as_ref().expect("the surface step runs first");
    let k = world.kerb.as_ref().expect("the kerb step runs first");
    let ways: Vec<&Polyline2> = roads.plan.iter().filter(|w| width::family(&w.class) == Family::Carriageway).collect();
    let index = RoadIndex::build(ways.iter().copied());
    let corners = corners(&surface.carriageway, BEND_MIN_DEG, &ways, &index);

    // One closing per radius, over the carriageway near that radius's
    // triangles: the closing is a local operation, and `2r` of context
    // past a triangle is all it can see from inside it.
    let mut by_radius: HashMap<u64, Vec<&Corner>> = HashMap::new();
    for c in &corners {
        by_radius.entry(c.radius_m.to_bits()).or_default().push(c);
    }
    let mut radii: Vec<u64> = by_radius.keys().copied().collect();
    radii.sort();
    let mut fillets: Shapes = Vec::new();
    for bits in radii {
        let r = f64::from_bits(bits);
        let wedges: Shapes = by_radius[&bits].iter().filter_map(|c| c.wedge()).collect();
        let wedges = poly::union_all(&wedges);
        let context = poly::intersect(&surface.carriageway, &poly::dilate(&wedges, 2.0 * r));
        let closed = poly::erode(&poly::dilate(&context, r), r);
        // The opening: cut back from the kerb, then grown again — sharp, so
        // the notch's own corner is reached, and a little past, so the
        // fillet overlaps the kerb rather than touching it; then held to
        // the closing, so its arc is the closing's arc and not grown.
        let core = poly::difference(&closed, &poly::dilate(&context, OPEN_M));
        let grown = poly::dilate_sharp(&core, OPEN_M + OVERLAP_M);
        let gained = poly::intersect(&grown, &closed);
        fillets.extend(poly::intersect(&gained, &wedges));
    }
    let fillets = poly::union_all(&fillets);
    // The buildings win over the return as over the kerb it rounds.
    let carriageway =
        world.asphalt(&poly::fill_holes_under(poly::union_of(&[&surface.carriageway, &fillets]), HOLE_MIN_M2));
    // The pavement follows the kerb return: where a fillet ate into a
    // pavement, at least the narrowest pavement is laid back outside the
    // new kerb, so a sidewalk wraps the corner rather than ending at it.
    let laid_back = poly::dilate(&poly::intersect(&fillets, &k.pavement), kerb::WALK_MIN_M);
    let pavement = world.pavement(&poly::union_of(&[&k.pavement, &laid_back]), &carriageway);
    let bare = kerb::Bare::new(&carriageway, &pavement, world.walls());
    let (gap_n, gap_of) = kerb::kerb_gap(&carriageway, &bare, &k.attached);
    let summary = Summary::new()
        .with("corners", corners.len())
        .with("fillets", fillets.len())
        .with_m2("fillet_m2", poly::area(&fillets))
        .with_regions("carriageway", &carriageway)
        .with_m2("pavement_m2", poly::area(&pavement))
        .with_share("kerb_gap", gap_n, gap_of);
    world.fillet = Some(Fillet { corners, fillets, carriageway, pavement });
    summary
}

/// The corners of `carriageway`: every vertex of its rings where the
/// boundary turns toward the region — right, the region being on the left
/// of every ring — by at least `min_deg`. Each gets the return radius of
/// the narrowest of `ways` whose kerb passes through it (`index` is built
/// over `ways`, in order); a corner no kerb passes through, which the
/// lattice can make of a near-touch, gets the smallest radius there is.
pub fn corners(carriageway: &Shapes, min_deg: f64, ways: &[&Polyline2], index: &RoadIndex) -> Vec<Corner> {
    let mut out = Vec::new();
    for ring in carriageway.iter().flatten() {
        let n = ring.len();
        if n < 3 {
            continue;
        }
        for i in 0..n {
            let (a, b, c) = (ring[(i + n - 1) % n], ring[i], ring[(i + 1) % n]);
            let turn = poly::turn_deg(a, b, c);
            if turn > -min_deg {
                continue;
            }
            let radius_m = index
                .kerbs_at(b, KERB_TOL_M)
                .into_iter()
                .map(|w| width::fillet_m(&ways[w].class))
                .fold(f64::INFINITY, f64::min);
            let radius_m = if radius_m.is_finite() { radius_m } else { width::fillet_m("") };
            let (u, v) = (poly::unit([b[0] - a[0], b[1] - a[1]]), poly::unit([c[0] - b[0], c[1] - b[1]]));
            out.push(Corner { at: b, turn_deg: -turn, along: [[-u[0], -u[1]], v], radius_m });
        }
    }
    out
}

#[cfg(test)]
pub(crate) mod tests {
    use std::f64::consts::PI;

    use crate::kerb;

    use super::*;

    /// A flat world with the network of `spec`, kerbed.
    pub(crate) fn world(spec: &str) -> World {
        let mut w = kerb::tests::world(spec);
        kerb::run(&mut w);
        w
    }

    /// Runs `w` from the ribbon step through the fillet: the fillet's line.
    pub(crate) fn pave(w: &mut World) -> Summary {
        crate::ribbon::run(w);
        crate::surface::run(w);
        kerb::run(w);
        run(w)
    }

    /// The area a closing of radius `r` adds to `n` right-angle notches.
    fn notch_gain(n: usize, r: f64) -> f64 {
        n as f64 * r * r * (1.0 - PI / 4.0)
    }

    /// What the fillet step added to the carriageway.
    fn gain(w: &World) -> f64 {
        poly::area(&w.fillet.as_ref().unwrap().carriageway) - poly::area(&w.surface.as_ref().unwrap().carriageway)
    }

    #[test]
    fn a_cross_gains_four_kerb_returns() {
        let mut w = world("net:cross?len=200");
        let s = run(&mut w);
        let f = w.fillet.as_ref().unwrap();
        assert_eq!(f.corners.len(), 4, "{s}");
        assert!(f.corners.iter().all(|c| c.radius_m == 4.0 && (c.turn_deg - 90.0).abs() < 1e-6), "{:?}", f.corners);
        let gain = gain(&w);
        let exact = notch_gain(4, 4.0);
        assert!((gain - exact).abs() < 0.05 * exact, "{gain} vs {exact}: {s}");
        assert_eq!(f.carriageway.len(), 1);
        assert_eq!(f.carriageway[0].len(), 1, "no holes: {s}");
        // The corner point of a notch is now asphalt; a point well outside is not.
        let c = 2.75 + 4.0 * (1.0 - 1.0 / 2.0f64.sqrt()) - 0.05;
        assert!(poly::contains(&f.carriageway, [c, c]), "{c}");
        assert!(!poly::contains(&f.carriageway, [2.75 + 4.0, 2.75 + 4.0]));
    }

    #[test]
    fn a_tee_gains_two_and_a_dual_none() {
        let mut w = world("net:tee?len=200");
        run(&mut w);
        let gain = gain(&w);
        let exact = notch_gain(2, 4.0);
        assert!((gain - exact).abs() < 0.05 * exact, "{gain} vs {exact}");
        let mut w = world("net:dual?gap=4&len=200");
        let s = run(&mut w);
        let f = w.fillet.as_ref().unwrap();
        assert_eq!(f.corners.len(), 0, "{s}");
        assert_eq!(f.carriageway.len(), 2);
        assert_eq!(poly::area(&f.fillets), 0.0);
    }

    #[test]
    fn a_bend_within_one_way_rounds_its_inner_side_only() {
        // The road turns a right angle at one vertex: the round join
        // rounds the outside, and the inside is a corner like any other —
        // no real kerb turns sharp — that gets the road's own return.
        let mut w = world("net:corner?d=5&len=200");
        let s = run(&mut w);
        let f = w.fillet.as_ref().unwrap();
        assert_eq!(f.corners.len(), 1, "{s}");
        assert_eq!(f.corners[0].radius_m, 4.0);
        let gain = gain(&w);
        let exact = notch_gain(1, 4.0);
        assert!((gain - exact).abs() < 0.05 * exact, "{gain} vs {exact}: {s}");
        assert!(s.to_string().contains("kerb_gap=0/"), "{s}");
    }

    #[test]
    fn a_roundabout_has_two_corners_per_leg_and_keeps_its_pavement() {
        let mut w = world("net:roundabout?r=15&d=5&len=200");
        let s = run(&mut w);
        let f = w.fillet.as_ref().unwrap();
        assert_eq!(f.corners.len(), 8, "{s}");
        // Eight notches, each between a straight leg and the ring's convex
        // outer kerb, which opens the corner past a right angle: less than
        // eight right-angle returns, and well over half of them.
        let g = gain(&w);
        assert!(g > 0.5 * notch_gain(8, 4.0) && g < notch_gain(8, 4.0), "{g}: {s}");
        assert_eq!(f.carriageway.len(), 1, "{s}");
        assert_eq!(f.carriageway[0].len(), 2, "the island is the only hole: {s}");
        assert!(s.to_string().contains("kerb_gap=0/"), "{s}");
        // The pavement lost what the fillets took; with a 5 m sidewalk ring
        // there was pavement past every return already, so nothing is added.
        let before = poly::area(&w.kerb.as_ref().unwrap().pavement);
        let taken = poly::area(&poly::intersect(&w.kerb.as_ref().unwrap().pavement, &f.fillets));
        assert!((poly::area(&f.pavement) - (before - taken)).abs() < 1e-3);
    }

    #[test]
    fn a_pavement_eaten_by_a_return_wraps_the_corner() {
        // A sidewalk 3.75 m off the axis: a 1 m band from the kerb, thinner
        // than the 1.66 m a 4 m return reaches at a right angle. The
        // sidewalk crosses the leg, so the return north of the road eats it.
        let mut w = world("net:tee?len=200");
        let walk = crate::world::Polyline2 {
            id: "walk".into(),
            class: "footway".into(),
            subclass: "sidewalk".into(),
            width_m: width::WALK_M,
            kind: crate::world::Kind::Ground,
            pts: vec![[-100.0, 3.75], [100.0, 3.75]],
        };
        w.roads.as_mut().unwrap().plan.push(walk);
        let s = pave(&mut w);
        let f = w.fillet.as_ref().unwrap();
        // Just outside the return's arc there is pavement.
        let c = 2.75 + 4.0 * (1.0 - 1.0 / 2.0f64.sqrt());
        let probe = [c + 0.2, c + 0.2];
        assert!(!poly::contains(&f.carriageway, probe));
        assert!(poly::contains(&f.pavement, probe), "{s}");
        assert!(s.to_string().contains("kerb_gap=0/"), "{s}");
    }

    #[test]
    fn a_hook_is_not_filled_to_a_disc() {
        // Road-e leaves the tee and hooks back on a 5 m radius: the two
        // straights' kerbs are 4.5 m apart, under `2r`, and a closing of
        // the junction's surroundings filled the whole inside.
        let mut w = world("net:tee?hook=5&len=200");
        let s = run(&mut w);
        let f = w.fillet.as_ref().unwrap();
        assert_eq!(f.corners.len(), 2, "{s}");
        assert!(!poly::contains(&f.carriageway, [10.0, 5.0]), "the inside of the hook: {s}");
        assert!(!poly::contains(&f.carriageway, [8.0, 5.0]), "{s}");
        // Nor the half-metre between the hook's end and the leg's kerb.
        assert!(!poly::contains(&f.carriageway, [3.0, 10.0]), "{s}");
        // The tee's own returns are still there.
        let c = 2.75 + 4.0 * (1.0 - 1.0 / 2.0f64.sqrt()) - 0.05;
        assert!(poly::contains(&f.carriageway, [-c, c]), "{s}");
        assert!(poly::contains(&f.carriageway, [c, c]), "{s}");
    }

    #[test]
    fn the_narrower_way_sets_the_radius() {
        // A service driveway on a primary: the corners between them get
        // the driveway's 3 m, not the primary's 8 m.
        let mut w = world("net:tee?class=primary&len=200");
        let leg = w.roads.as_mut().unwrap().plan.iter_mut().find(|l| l.id == "leg").unwrap();
        leg.class = "service".into();
        leg.width_m = width::of("service", "");
        let s = pave(&mut w);
        let f = w.fillet.as_ref().unwrap();
        assert_eq!(f.corners.len(), 2, "{s}");
        assert!(f.corners.iter().all(|c| c.radius_m == 3.0), "{:?}", f.corners);
    }

    #[test]
    fn small_holes_are_filled() {
        let shapes: Shapes = vec![vec![
            vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]],
            vec![[4.0, 4.0], [4.0, 4.5], [4.5, 4.5], [4.5, 4.0]],
            vec![[6.0, 6.0], [6.0, 8.0], [8.0, 8.0], [8.0, 6.0]],
        ]];
        let out = poly::fill_holes_under(shapes, 0.5);
        assert_eq!(out[0].len(), 2);
        assert!((poly::area(&out) - 96.0).abs() < 1e-9);
    }
}
