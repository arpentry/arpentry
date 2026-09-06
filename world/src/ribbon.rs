//! Step 3: every way is a polygon.
//!
//! The mapped centreline, buffered to the width the reader gave it
//! ([`crate::width::of_way`]), with round caps and round joins ([`crate::poly`] says why
//! round). Nothing is merged yet: a junction is still several ribbons lying
//! over one another, and a sidewalk still floats wherever it was mapped.
//! Those are the next steps, each a boolean on what this one returns, which
//! is why this one comes first — and why it has an oracle nobody had to
//! write: the SVG's `band` group is the same construction drawn by a browser.
//!
//! The ribbon reads the *plan* line, not the draped one: the drape splits a
//! way at every grid line and diagonal, and though collinear vertices do not
//! change a buffer's shape they would triple its vertex count.

use crate::poly::{self, Shapes};
use crate::step::Summary;
use crate::width::{self, Family};
use crate::world::{Ribbon, Ribbons, World};

/// Buffers every way of the world's network.
pub fn run(world: &mut World) -> Summary {
    let roads = world.roads.as_ref().expect("the drape step runs first");
    let mut out = Ribbons::default();
    let (mut contours, mut vertices) = (0usize, 0usize);
    let mut area = [0.0f64; 2];
    for line in &roads.plan {
        let shape = ribbon(&line.pts, line.width_m);
        if shape.is_empty() {
            continue;
        }
        let family = width::family(&line.class);
        contours += shape.iter().map(Vec::len).sum::<usize>();
        vertices += shape.iter().flatten().map(Vec::len).sum::<usize>();
        area[family as usize] += poly::area(&shape);
        out.ribbons.push(Ribbon {
            id: line.id.clone(),
            class: line.class.clone(),
            subclass: line.subclass.clone(),
            family,
            shape,
        });
    }
    let summary = Summary::new()
        .with("ribbons", out.ribbons.len())
        .with("contours", contours)
        .with("vertices", vertices)
        .with("carriageway_m2", format!("{:.0}", area[Family::Carriageway as usize]))
        .with("walk_m2", format!("{:.0}", area[Family::Walk as usize]));
    world.ribbons = Some(out);
    summary
}

/// The polygon of one way.
pub fn ribbon(pts: &[[f64; 2]], width_m: f64) -> Shapes {
    poly::buffer_line(pts, width_m)
}

#[cfg(test)]
pub(crate) mod tests {
    use std::f64::consts::PI;

    use crate::drape;
    use crate::terrain::{self, tests::dem};

    use super::*;

    /// A flat world with the network of `spec`.
    pub(crate) fn world(spec: &str) -> World {
        let mut w = terrain::tests::world();
        terrain::run(&mut w, &mut dem("flat"), 100.0, usize::MAX);
        drape::run(&mut w, std::path::Path::new(spec)).unwrap();
        w
    }

    #[test]
    fn a_straight_has_its_area() {
        let mut w = world("net:straight?len=200");
        let s = run(&mut w);
        let r = w.ribbons.as_ref().unwrap();
        assert_eq!(r.ribbons.len(), 1);
        assert_eq!(r.ribbons[0].family, Family::Carriageway);
        let exact = 200.0 * 5.5 + PI * 2.75 * 2.75;
        let a = poly::area(&r.ribbons[0].shape);
        assert!(a <= exact && a > exact - 0.01 * PI * 2.75 * 2.75, "{a} vs {exact}");
        assert!(s.to_string().contains("ribbons=1"), "{s}");
    }

    #[test]
    fn a_sidewalk_is_a_walk_ribbon_at_its_width() {
        let mut w = world("net:sidewalk?d=6&len=100");
        run(&mut w);
        let r = w.ribbons.as_ref().unwrap();
        let walk = r.ribbons.iter().find(|r| r.id == "walk-n").unwrap();
        assert_eq!(walk.family, Family::Walk);
        assert!(poly::contains(&walk.shape, [0.0, 6.9]));
        assert!(!poly::contains(&walk.shape, [0.0, 7.1]));
        assert!(!poly::contains(&walk.shape, [0.0, 4.9]));
        // The gap between kerb (2.75) and the sidewalk's inner edge (5) is
        // nobody's yet: that is the kerb step's job.
        let road = r.ribbons.iter().find(|r| r.id == "road").unwrap();
        assert!(!poly::contains(&road.shape, [0.0, 4.0]));
        assert!(!poly::contains(&walk.shape, [0.0, 4.0]));
    }

    #[test]
    fn a_cross_is_four_ribbons_that_overlap() {
        let mut w = world("net:cross?len=200");
        run(&mut w);
        let r = w.ribbons.as_ref().unwrap();
        assert_eq!(r.ribbons.len(), 4);
        assert!(r.ribbons.iter().all(|x| poly::contains(&x.shape, [0.0, 0.0])));
    }
}
