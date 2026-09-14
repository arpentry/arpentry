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
//!
//! **The railways' ballast is the third region**, and it stands between the
//! other two: the asphalt wins over it — a level crossing is the road's
//! surface with the rails running through it — and it wins over the walk.
//! It does not stop at a building, because a station roof over its
//! platforms is a level relation the model cannot state, and narrowing the
//! formation there shaves the platform (the server's
//! `a_rail_formation_is_not_narrowed_by_the_roof_over_it`). A double track
//! is two ways a metre or less apart, and their beds are one bed: the
//! ballast is closed across a gap under [`TWIN_GAP_M`].

use crate::poly::{self, Shapes};
use crate::step::Summary;
use crate::width::Family;
use crate::world::{Facade, Ribbons, Surface};

/// The widest gap, in metres, between two tracks' beds that is closed into
/// one: a double track's centres stand 3.8–4.5 m apart on the Swiss
/// network, so two 3.5 m track zones leave a strip of 0.3–1 m between them
/// that no real formation has. A siding further off keeps its own bed.
pub const TWIN_GAP_M: f64 = 1.5;

/// Unions the world's ribbons per family.
/// One region per `(family, group)`: **step 3 of
/// `data/plans/one-surface-at-a-junction-2026-09-14.md`**, as a function.
///
/// [`run`] unions the *ground* pieces alone, because that is all the surface
/// steps have ever read — a span's paving is swept by `structure` afterwards,
/// and the two meet neither in plan nor in height. This unions **every**
/// piece, ground and span alike, within the group
/// [`crate::partition::groups`] puts it in. So a junction standing on a deck
/// comes out as one region together with its approaches, and a viaduct stays
/// off the street it flies over because the grouping has already split them.
///
/// The caps are `ribbon`'s own rule, which needs no change here: a round cap
/// is a disc that closes a joint whatever the angle, and a free end is
/// square.
///
/// It is a pure function of the roads and stores nothing. Steps 4 and 5 are
/// its consumers; until they land, [`run`] and `Surface` are untouched and
/// nothing downstream moves.
pub fn grouped(roads: &crate::world::Roads) -> Vec<(Family, usize, Shapes)> {
    let (group, ..) = crate::partition::groups(&roads.plan, &roads.spans);
    let pieces: Vec<&crate::world::Polyline2> = roads.pieces().collect();
    let fam_of = |p: &crate::world::Polyline2| crate::width::family(&p.class) as usize;

    let mut ends: std::collections::HashMap<(usize, (i64, i64)), usize> = std::collections::HashMap::new();
    for p in &pieces {
        if p.pts.len() < 2 {
            continue;
        }
        for e in [p.pts[0], p.pts[p.pts.len() - 1]] {
            *ends.entry((fam_of(p), crate::world::connector(e))).or_default() += 1;
        }
    }

    // A `BTreeMap`, so the regions come out in the world's order and not a
    // hasher's: these are an output.
    let mut by: std::collections::BTreeMap<(usize, usize), Shapes> = std::collections::BTreeMap::new();
    for (i, p) in pieces.iter().enumerate() {
        if p.pts.len() < 2 {
            continue;
        }
        let n = p.pts.len();
        let round = |e: [f64; 2]| ends.get(&(fam_of(p), crate::world::connector(e))).copied().unwrap_or(0) > 1;
        let caps = [round(p.pts[0]), round(p.pts[n - 1])];
        by.entry((fam_of(p), group[i]))
            .or_default()
            .extend(poly::buffer_line_capped(&p.pts, p.width_m, caps));
    }
    by.into_iter()
        .map(|((fam, g), parts)| {
            let family = match fam {
                0 => Family::Carriageway,
                1 => Family::Walk,
                _ => Family::Rail,
            };
            (family, g, poly::union_all(&parts))
        })
        .collect()
}

pub fn run(ribbons: &Ribbons, facade: &Facade) -> (Surface, Summary) {
    let mut per_family: [Shapes; 3] = Default::default();
    for r in &ribbons.ribbons {
        per_family[r.family as usize].extend(r.shape.iter().cloned());
    }
    let carriageway_open = poly::union_all(&per_family[Family::Carriageway as usize]);
    let carriageway = facade.asphalt(&carriageway_open);
    let walled_carriageway = poly::area(&carriageway_open) - poly::area(&carriageway);
    // The track bed: every rail ribbon, twin tracks closed into one bed,
    // the asphalt taken out of it where a road crosses.
    let rails = poly::union_all(&per_family[Family::Rail as usize]);
    let ballast_open = if rails.is_empty() {
        rails
    } else {
        poly::erode(&poly::dilate(&rails, TWIN_GAP_M / 2.0), TWIN_GAP_M / 2.0)
    };
    let ballast = poly::difference(&ballast_open, &carriageway);
    let crossed = poly::area(&ballast_open) - poly::area(&ballast);
    // [`World::pavement`]'s two cuts, made here one at a time so each is
    // reported: what the walk lost to the asphalt and the ballast, then to
    // the walls.
    let walk_alone = poly::union_all(&per_family[Family::Walk as usize]);
    let walk_open = poly::difference(&walk_alone, &carriageway);
    let bitten = poly::area(&walk_alone) - poly::area(&walk_open);
    let walk_off = poly::difference(&walk_open, &ballast);
    let on_rail = poly::area(&walk_open) - poly::area(&walk_off);
    let walk = poly::difference(&walk_off, &facade.solid);
    let walled_walk = poly::area(&walk_off) - poly::area(&walk);
    let summary = Summary::new()
        .with_regions("carriageway", &carriageway)
        .with_m2("carriageway_m2", poly::area(&carriageway))
        .with_regions("walk", &walk)
        .with_m2("walk_m2", poly::area(&walk))
        .with_regions("ballast", &ballast)
        .with_m2("ballast_m2", poly::area(&ballast))
        .with_m2("walk_under_asphalt_m2", bitten)
        .with_m2("walk_on_ballast_m2", on_rail)
        .with_m2("ballast_under_asphalt_m2", crossed)
        .with_m2("carriageway_in_building_m2", walled_carriageway)
        .with_m2("walk_in_building_m2", walled_walk);
    (Surface { carriageway, walk, ballast }, summary)
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

    /// A level crossing is the road's surface with the rails through it:
    /// the bed stops at the asphalt, either side of it.
    #[test]
    fn the_asphalt_wins_over_the_ballast_at_a_level_crossing() {
        let (w, s) = world("net:level?len=100");
        let surf = w.surface.as_ref().unwrap();
        assert_eq!(surf.ballast.len(), 2, "the bed either side of the road: {s}");
        assert!(poly::intersect(&surf.ballast, &surf.carriageway).is_empty());
        assert!(poly::contains(&surf.carriageway, [0.0, 0.0]));
        assert!(poly::contains(&surf.ballast, [0.0, 10.0]));
        assert!(!poly::contains(&surf.ballast, [0.0, 2.0]));
        // The road's 5.5 m across the track zone's 3.5.
        assert!((s.num("ballast_under_asphalt_m2") - 5.5 * crate::width::RAIL_M).abs() < 0.5, "{s}");
    }

    /// A double track is one bed; a siding further off keeps its own.
    #[test]
    fn a_double_track_is_one_bed() {
        let (w, _) = world("net:dual?gap=0.8&len=100&class=standard_gauge");
        let surf = w.surface.as_ref().unwrap();
        assert!(surf.carriageway.is_empty());
        assert_eq!(surf.ballast.len(), 1);
        assert!(poly::contains(&surf.ballast, [0.0, 0.0]), "the strip between the tracks is bed");
        let (w, _) = world("net:dual?gap=3&len=100&class=standard_gauge");
        assert_eq!(w.surface.as_ref().unwrap().ballast.len(), 2);
    }
}
