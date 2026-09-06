//! Step 4: one region per family.
//!
//! The ribbons unioned per [`Family`]: every carriageway into one set of
//! disjoint regions with their holes, every pedestrian way into another. A
//! junction is not an object; it is where legs overlap, and the union is
//! where they stop being several ribbons and become one surface — the
//! tiler's "one unioned region per level" (docs/ROADS.md invariant 2), held
//! by construction because there are no two objects left to disagree about
//! a boundary.
//!
//! Where the two families overlap, **the asphalt wins**: the walk region has
//! the carriageway subtracted from it. A crossing stub mapped across a road,
//! a footway that runs onto a street, a sidewalk the data placed under the
//! prior width of its road — each is bitten back to the kerb and never draws
//! a slab across the asphalt. What is bitten off is reported, because it is
//! the first measure of how much the pedestrian network and the roads
//! disagree about where the kerb is.

use crate::poly::{self, Shapes};
use crate::step::Summary;
use crate::width::Family;
use crate::world::{Surface, World};

/// Unions the world's ribbons per family.
pub fn run(world: &mut World) -> Summary {
    let ribbons = world.ribbons.as_ref().expect("the ribbon step runs first");
    let mut per_family: [Shapes; 2] = [Vec::new(), Vec::new()];
    for r in &ribbons.ribbons {
        per_family[r.family as usize].extend(r.shape.iter().cloned());
    }
    let carriageway = poly::union_all(&per_family[Family::Carriageway as usize]);
    let walk_alone = poly::union_all(&per_family[Family::Walk as usize]);
    let walk = poly::difference(&walk_alone, &carriageway);
    let bitten = poly::area(&walk_alone) - poly::area(&walk);
    let summary = Summary::new()
        .with("carriageway", regions(&carriageway))
        .with("carriageway_m2", format!("{:.0}", poly::area(&carriageway)))
        .with("walk", regions(&walk))
        .with("walk_m2", format!("{:.0}", poly::area(&walk)))
        .with("walk_under_asphalt_m2", format!("{:.0}", bitten));
    world.surface = Some(Surface { carriageway, walk });
    summary
}

/// `regions/holes` of a set of shapes, for the summary line.
fn regions(shapes: &Shapes) -> String {
    let holes: usize = shapes.iter().map(|s| s.len() - 1).sum();
    format!("{}/{}", shapes.len(), holes)
}

#[cfg(test)]
pub(crate) mod tests {
    use std::f64::consts::PI;

    use crate::ribbon;

    use super::*;

    /// A flat world with the network of `spec`, ribboned.
    pub(crate) fn world(spec: &str) -> World {
        let mut w = ribbon::tests::world(spec);
        ribbon::run(&mut w);
        w
    }

    /// The area of one round-capped straight.
    fn straight(len: f64, w: f64) -> f64 {
        len * w + PI * (w / 2.0) * (w / 2.0)
    }

    #[test]
    fn a_cross_is_one_region_less_the_overlap() {
        let mut w = world("net:cross?len=200");
        run(&mut w);
        let s = w.surface.as_ref().unwrap();
        assert_eq!(s.carriageway.len(), 1);
        assert_eq!(s.carriageway[0].len(), 1, "no holes");
        let exact = 2.0 * straight(200.0, 5.5) - 5.5 * 5.5;
        let a = poly::area(&s.carriageway);
        let cap = PI * 2.75 * 2.75;
        assert!(a <= exact + 1e-3 && a > exact - 0.02 * cap, "{a} vs {exact}");
        assert!(s.walk.is_empty());
    }

    #[test]
    fn a_union_never_exceeds_its_ribbons_and_equals_them_when_disjoint() {
        let mut w = world("net:dual?gap=4&len=200");
        run(&mut w);
        let ribbons = poly::area(
            &w.ribbons.as_ref().unwrap().ribbons.iter().flat_map(|r| r.shape.iter().cloned()).collect(),
        );
        let s = w.surface.as_ref().unwrap();
        assert_eq!(s.carriageway.len(), 2, "a dual carriageway is two regions");
        assert!((poly::area(&s.carriageway) - ribbons).abs() < 1e-3);
        let mut w = world("net:tee?len=200");
        run(&mut w);
        let ribbons = poly::area(
            &w.ribbons.as_ref().unwrap().ribbons.iter().flat_map(|r| r.shape.iter().cloned()).collect(),
        );
        assert!(poly::area(&w.surface.as_ref().unwrap().carriageway) < ribbons - 1.0);
    }

    #[test]
    fn the_asphalt_wins_where_a_crossing_meets_it() {
        let mut w = world("net:crossing?d=6&len=100");
        let s = run(&mut w);
        let surf = w.surface.as_ref().unwrap();
        // No walk point on the asphalt, and none of the asphalt is missing.
        assert!(!poly::contains(&surf.walk, [0.0, 0.0]));
        assert!(!poly::contains(&surf.walk, [0.0, 2.7]));
        assert!(poly::contains(&surf.carriageway, [0.0, 2.7]));
        // The stub survives between the kerb and each sidewalk.
        assert!(poly::contains(&surf.walk, [0.0, 4.0]));
        assert!(poly::contains(&surf.walk, [0.0, -4.0]));
        // Each sidewalk and its half of the stub is one region: two in all.
        assert_eq!(surf.walk.len(), 2, "{:?}", surf.walk.len());
        // What was bitten off is the stub's 3 m × 5.5 m across the asphalt.
        let bitten: f64 = s
            .to_string()
            .split("walk_under_asphalt_m2=")
            .nth(1)
            .and_then(|t| t.split(' ').next())
            .and_then(|t| t.parse().ok())
            .unwrap();
        assert!((bitten - 16.5).abs() < 0.6, "{s}");
        // Families never overlap.
        assert!(poly::intersect(&surf.walk, &surf.carriageway).is_empty());
    }

    #[test]
    fn a_sidewalk_under_the_prior_width_is_cut_to_the_kerb() {
        let mut w = world("net:sidewalk?d=2&len=100");
        run(&mut w);
        let surf = w.surface.as_ref().unwrap();
        // Mapped at 2 m with a 1 m half-width: [1, 3]; the kerb is at 2.75.
        assert!(poly::contains(&surf.walk, [0.0, 2.9]));
        assert!(!poly::contains(&surf.walk, [0.0, 2.6]));
        assert!(!poly::contains(&surf.walk, [0.0, 3.1]));
    }
}
