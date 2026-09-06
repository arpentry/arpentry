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
//! `2r`: a dual carriageway's median, two streets across a narrow block. So
//! it is **masked**: closed only within a disc about each junction, and a
//! junction is a place where at least three legs meet, or two meet at a
//! bend of more than [`BEND_MIN_DEG`] — two arcs of one road that continue
//! straight are a connector in the data and nothing on the ground. The
//! radius is a prior of the widest leg's class ([`crate::width::fillet_m`]).
//!
//! The asphalt still wins: the pavement is re-cut by the fillets, so the
//! kerb a sidewalk stands on is the filleted one. `kerb_gap` is measured
//! again on the result, because the invariant is the final surface's.

use std::collections::HashMap;

use crate::kerb;
use crate::poly::{self, Pt, Shape, Shapes};
use crate::step::Summary;
use crate::width::{self, Family};
use crate::world::{Fillet, Polyline2, World};

/// Two legs meeting at a bend sharper than this, in degrees, are a corner
/// worth a kerb return; straighter is a continuation.
pub const BEND_MIN_DEG: f64 = 30.0;

/// Endpoints closer than this, in metres, are one connector. The frame
/// maps a shared source coordinate to one local point exactly; the slack
/// is for the clip and for hand-made specimens.
const SNAP_M: f64 = 0.01;

/// Sides of the polygon that stands in for a junction disc.
const DISC_SIDES: usize = 32;

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
/// a fillet meets the mask's edge, and it is filled.
const HOLE_MIN_M2: f64 = 0.5;

/// One junction: where the legs meet, the fillet radius, and the disc the
/// closing is confined to.
#[derive(Debug, Clone, PartialEq)]
pub struct Junction {
    pub at: Pt,
    pub legs: usize,
    pub radius_m: f64,
    pub mask_m: f64,
}

/// Rounds the world's junctions.
pub fn run(world: &mut World) -> Summary {
    let roads = world.roads.as_ref().expect("the drape step runs first");
    let surface = world.surface.as_ref().expect("the surface step runs first");
    let k = world.kerb.as_ref().expect("the kerb step runs first");
    let junctions = junctions(roads.plan.iter().filter(|w| width::family(&w.class) == Family::Carriageway));

    // One closing per radius, over the carriageway clipped to the masks'
    // surroundings: the closing is a local operation, and `2r` of context
    // past a mask is all it can see from inside it.
    let mut by_radius: HashMap<u64, Vec<&Junction>> = HashMap::new();
    for j in &junctions {
        by_radius.entry(j.radius_m.to_bits()).or_default().push(j);
    }
    let mut radii: Vec<u64> = by_radius.keys().copied().collect();
    radii.sort();
    let mut fillets: Shapes = Vec::new();
    for bits in radii {
        let r = f64::from_bits(bits);
        let masks: Shapes = by_radius[&bits].iter().map(|j| disc(j.at, j.mask_m)).collect();
        let masks = poly::union_all(&masks);
        let context = poly::intersect(&surface.carriageway, &poly::dilate(&masks, 2.0 * r));
        let closed = poly::erode(&poly::dilate(&context, r), r);
        // The opening: cut back from the kerb, then grown again — sharp, so
        // the notch's own corner is reached, and a little past, so the
        // fillet overlaps the kerb rather than touching it; then held to
        // the closing, so its arc is the closing's arc and not grown.
        let core = poly::difference(&closed, &poly::dilate(&context, OPEN_M));
        let grown = poly::dilate_sharp(&core, OPEN_M + OVERLAP_M);
        let gained = poly::intersect(&grown, &closed);
        fillets.extend(poly::intersect(&gained, &masks));
    }
    let fillets = poly::union_all(&fillets);
    let mut all = surface.carriageway.clone();
    all.extend(fillets.iter().cloned());
    let carriageway = drop_small_holes(poly::union_all(&all), HOLE_MIN_M2);
    // The pavement follows the kerb return: where a fillet ate into a
    // pavement, at least the narrowest pavement is laid back outside the
    // new kerb, so a sidewalk wraps the corner rather than ending at it.
    let eaten = poly::intersect(&fillets, &k.pavement);
    let mut pavement = k.pavement.clone();
    pavement.extend(poly::dilate(&eaten, kerb::WALK_MIN_M));
    let pavement = poly::difference(&poly::union_all(&pavement), &carriageway);
    let (gap_n, gap_of) = kerb::kerb_gap(&carriageway, &pavement, &k.attached);
    let summary = Summary::new()
        .with("junctions", junctions.len())
        .with("fillets", fillets.len())
        .with("fillet_m2", format!("{:.0}", poly::area(&fillets)))
        .with("carriageway", format!("{}/{}", carriageway.len(), holes(&carriageway)))
        .with("pavement_m2", format!("{:.0}", poly::area(&pavement)))
        .with("kerb_gap", format!("{gap_n}/{gap_of} ({:.2}%)", pct(gap_n, gap_of)));
    world.fillet = Some(Fillet { junctions, fillets, carriageway, pavement });
    summary
}

/// `shapes` with every hole under `min_m2` filled.
fn drop_small_holes(shapes: Shapes, min_m2: f64) -> Shapes {
    shapes
        .into_iter()
        .map(|shape| {
            let mut it = shape.into_iter();
            let outer = it.next();
            outer.into_iter().chain(it.filter(|ring| -poly::ring_area(ring) >= min_m2)).collect()
        })
        .collect()
}

fn pct(n: usize, of: usize) -> f64 {
    if of == 0 {
        0.0
    } else {
        100.0 * n as f64 / of as f64
    }
}

fn holes(shapes: &Shapes) -> usize {
    shapes.iter().map(|s| s.len() - 1).sum()
}

/// One leg's end: where it meets, the direction it leaves in, its class.
struct End {
    at: Pt,
    dir: Pt,
    class: String,
    half_m: f64,
}

/// The junctions of a set of carriageway ways: every place where the ends
/// of three or more legs coincide, or two at a bend.
pub fn junctions<'a>(ways: impl Iterator<Item = &'a Polyline2>) -> Vec<Junction> {
    let mut ends: HashMap<(i64, i64), Vec<End>> = HashMap::new();
    for way in ways {
        let n = way.pts.len();
        if n < 2 {
            continue;
        }
        let half_m = way.width_m / 2.0;
        for (at, next) in [(way.pts[0], way.pts[1]), (way.pts[n - 1], way.pts[n - 2])] {
            let dir = unit([next[0] - at[0], next[1] - at[1]]);
            let key = ((at[0] / SNAP_M).round() as i64, (at[1] / SNAP_M).round() as i64);
            ends.entry(key).or_default().push(End { at, dir, class: way.class.clone(), half_m });
        }
    }
    let mut out: Vec<Junction> = Vec::new();
    for legs in ends.values() {
        let is_junction = match legs.len() {
            0 | 1 => false,
            2 => {
                let (a, b) = (legs[0].dir, legs[1].dir);
                // Continuing straight: the two legs leave in opposite directions.
                let cos = -(a[0] * b[0] + a[1] * b[1]);
                cos < BEND_MIN_DEG.to_radians().cos()
            }
            _ => true,
        };
        if !is_junction {
            continue;
        }
        let radius_m = legs.iter().map(|e| width::fillet_m(&e.class)).fold(0.0, f64::max);
        let half_m = legs.iter().map(|e| e.half_m).fold(0.0, f64::max);
        // The concave corner lies within `half / sin(θ/2)` of the meeting
        // point — three half-widths covers a 40° fork — and the fillet arc
        // within `r` of that.
        let mask_m = 3.0 * half_m + 2.0 * radius_m;
        out.push(Junction { at: legs[0].at, legs: legs.len(), radius_m, mask_m });
    }
    // A function of the set, not of the hash order.
    out.sort_by(|a, b| a.at.partial_cmp(&b.at).expect("finite"));
    out
}

/// A regular polygon standing in for the disc of radius `r` about `c`.
fn disc(c: Pt, r: f64) -> Shape {
    let ring = (0..DISC_SIDES)
        .map(|k| {
            let a = k as f64 * std::f64::consts::TAU / DISC_SIDES as f64;
            [c[0] + r * a.cos(), c[1] + r * a.sin()]
        })
        .collect();
    vec![ring]
}

fn unit(v: Pt) -> Pt {
    let len = v[0].hypot(v[1]);
    if len < 1e-12 {
        [0.0, 0.0]
    } else {
        [v[0] / len, v[1] / len]
    }
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
        assert_eq!(f.junctions.len(), 1, "{s}");
        assert_eq!(f.junctions[0].legs, 4);
        assert_eq!(f.junctions[0].radius_m, 4.0);
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
        assert_eq!(f.junctions.len(), 0, "{s}");
        assert_eq!(f.carriageway.len(), 2);
        assert_eq!(poly::area(&f.fillets), 0.0);
    }

    #[test]
    fn a_bend_within_one_way_is_not_a_junction() {
        let mut w = world("net:corner?d=5&len=200");
        let s = run(&mut w);
        assert!(s.to_string().contains("junctions=0"), "{s}");
        assert!(s.to_string().contains("kerb_gap=0/"), "{s}");
    }

    #[test]
    fn a_roundabout_has_a_junction_per_leg_and_keeps_its_pavement() {
        let mut w = world("net:roundabout?r=15&d=5&len=200");
        let s = run(&mut w);
        let f = w.fillet.as_ref().unwrap();
        assert_eq!(f.junctions.len(), 4, "{s}");
        assert!(f.junctions.iter().all(|j| j.legs == 3));
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
            pts: vec![[-100.0, 3.75], [100.0, 3.75]],
        };
        w.roads.as_mut().unwrap().plan.push(walk);
        crate::ribbon::run(&mut w);
        crate::surface::run(&mut w);
        kerb::run(&mut w);
        let s = run(&mut w);
        let f = w.fillet.as_ref().unwrap();
        // Just outside the return's arc there is pavement.
        let c = 2.75 + 4.0 * (1.0 - 1.0 / 2.0f64.sqrt());
        let probe = [c + 0.2, c + 0.2];
        assert!(!poly::contains(&f.carriageway, probe));
        assert!(poly::contains(&f.pavement, probe), "{s}");
        assert!(s.to_string().contains("kerb_gap=0/"), "{s}");
    }

    #[test]
    fn small_holes_are_filled() {
        let shapes: Shapes = vec![vec![
            vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]],
            vec![[4.0, 4.0], [4.0, 4.5], [4.5, 4.5], [4.5, 4.0]],
            vec![[6.0, 6.0], [6.0, 8.0], [8.0, 8.0], [8.0, 6.0]],
        ]];
        let out = drop_small_holes(shapes, 0.5);
        assert_eq!(out[0].len(), 2);
        assert!((poly::area(&out) - 96.0).abs() < 1e-9);
    }

    #[test]
    fn a_fork_of_two_ways_at_a_bend_is_a_junction() {
        let ways = [
            Polyline2 { id: "a".into(), class: "residential".into(), subclass: String::new(), width_m: 5.5, pts: vec![[-50.0, 0.0], [0.0, 0.0]] },
            Polyline2 { id: "b".into(), class: "residential".into(), subclass: String::new(), width_m: 5.5, pts: vec![[0.0, 0.0], [50.0, 50.0]] },
            Polyline2 { id: "c".into(), class: "primary".into(), subclass: String::new(), width_m: 7.0, pts: vec![[50.0, 50.0], [100.0, 100.0]] },
        ];
        let js = junctions(ways.iter());
        assert_eq!(js.len(), 1, "{js:?}");
        assert_eq!(js[0].at, [0.0, 0.0]);
        assert_eq!(js[0].legs, 2);
    }
}

