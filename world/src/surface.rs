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
//!
//! And **the buildings win over both**: what the facade step refuses is
//! subtracted from each family, so a prior width that runs into a wall
//! stops at the wall — the asphalt at the closed facade
//! ([`World::built`]), so its edge does not follow every notch, the walk
//! at the walls themselves ([`World::solid`]). What each family lost to
//! the buildings is reported beside what the walk lost to the asphalt.

use crate::poly::{self, Shapes};
use crate::step::Summary;
use crate::width::Family;
use crate::world::{Facade, Ribbons, Surface};

/// Unions the world's ribbons per family.
pub fn run(ribbons: &Ribbons, facade: &Facade) -> (Surface, Summary) {
    let mut per_family: [Shapes; 2] = [Vec::new(), Vec::new()];
    for r in &ribbons.ribbons {
        per_family[r.family as usize].extend(r.shape.iter().cloned());
    }
    let carriageway_open = poly::union_all(&per_family[Family::Carriageway as usize]);
    let carriageway = facade.asphalt(&carriageway_open);
    let walled_carriageway = poly::area(&carriageway_open) - poly::area(&carriageway);
    // [`World::pavement`]'s two cuts, made here one at a time so each is
    // reported: what the walk lost to the asphalt, then to the walls.
    let walk_alone = poly::union_all(&per_family[Family::Walk as usize]);
    let walk_open = poly::difference(&walk_alone, &carriageway);
    let bitten = poly::area(&walk_alone) - poly::area(&walk_open);
    let walk = poly::difference(&walk_open, &facade.solid);
    let walled_walk = poly::area(&walk_open) - poly::area(&walk);
    let summary = Summary::new()
        .with_regions("carriageway", &carriageway)
        .with_m2("carriageway_m2", poly::area(&carriageway))
        .with_regions("walk", &walk)
        .with_m2("walk_m2", poly::area(&walk))
        .with_m2("walk_under_asphalt_m2", bitten)
        .with_m2("carriageway_in_building_m2", walled_carriageway)
        .with_m2("walk_in_building_m2", walled_walk);
    (Surface { carriageway, walk }, summary)
}

#[cfg(test)]
pub(crate) mod tests {
    use crate::world::World;
    use crate::pipeline::tests::{built, plan};
    use crate::step::Step;

    

    use super::*;

    /// A flat world with the network of `spec`, ribboned.
    pub(crate) fn world(spec: &str) -> (World, Summary) {
        let (w, ran) = built("flat", spec, None, 100.0, &plan(Step::Surface));
        (w, ran.last())
    }

    #[test]
    fn a_cross_is_one_region_less_the_overlap() {
        // Four legs square at their dead ends, joined by discs at the
        // origin that lie inside the union: two straights less the overlap.
        let (w, _) = world("net:cross?len=200");
        let s = w.surface.as_ref().unwrap();
        assert_eq!(s.carriageway.len(), 1);
        assert_eq!(s.carriageway[0].len(), 1, "no holes");
        let exact = 2.0 * 200.0 * 5.5 - 5.5 * 5.5;
        let a = poly::area(&s.carriageway);
        assert!((a - exact).abs() < 1e-3, "{a} vs {exact}");
        assert!(s.walk.is_empty());
    }

    #[test]
    fn a_union_never_exceeds_its_ribbons_and_equals_them_when_disjoint() {
        let (w, _) = world("net:dual?gap=4&len=200");
        let ribbons = poly::area(
            &w.ribbons.as_ref().unwrap().ribbons.iter().flat_map(|r| r.shape.iter().cloned()).collect(),
        );
        let s = w.surface.as_ref().unwrap();
        assert_eq!(s.carriageway.len(), 2, "a dual carriageway is two regions");
        assert!((poly::area(&s.carriageway) - ribbons).abs() < 1e-3);
        let (w, _) = world("net:tee?len=200");
        let ribbons = poly::area(
            &w.ribbons.as_ref().unwrap().ribbons.iter().flat_map(|r| r.shape.iter().cloned()).collect(),
        );
        assert!(poly::area(&w.surface.as_ref().unwrap().carriageway) < ribbons - 1.0);
    }

    #[test]
    fn the_asphalt_wins_where_a_crossing_meets_it() {
        let (w, s) = world("net:crossing?d=6&len=100");
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
        // What was bitten off is the stub's 2 m × 5.5 m across the asphalt.
        assert!((s.num("walk_under_asphalt_m2") - 11.0).abs() < 0.6, "{s}");
        // Families never overlap.
        assert!(poly::intersect(&surf.walk, &surf.carriageway).is_empty());
    }

    #[test]
    fn a_sidewalk_under_the_prior_width_is_cut_to_the_kerb() {
        let (w, _) = world("net:sidewalk?d=2&len=100");
        let surf = w.surface.as_ref().unwrap();
        // Mapped at 2 m with a 1 m half-width: [1, 3]; the kerb is at 2.75.
        assert!(poly::contains(&surf.walk, [0.0, 2.9]));
        assert!(!poly::contains(&surf.walk, [0.0, 2.6]));
        assert!(!poly::contains(&surf.walk, [0.0, 3.1]));
    }
}
