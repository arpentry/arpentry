//! Layers made by hand, for a step tested on its own.
//!
//! A specimen built through [`crate::pipeline::tests::built`] runs every step
//! before the one it is about, so a change to the ribbon moves a room test
//! and a change to the legs moves a mesh test. These build the layer a step
//! reads directly — a carriageway and a house as rectangles, a sheet with no
//! axes — so a test can say exactly what the step was given, and a failure
//! is the step's own.

use crate::poly::{self, Shapes};
use crate::width::Family;
use crate::world::{Facade, Profiles, Sheet, Sheets, Surface};

/// Rectangles `[x0, y0, x1, y1]`, as regions.
fn rects(rs: &[[f64; 4]]) -> Shapes {
    poly::union_all(&rs.iter().map(|r| poly::rect(r[0], r[1], r[2], r[3])).collect())
}

/// A paved surface of rectangles: the asphalt and the walk, the walk cut to
/// the asphalt as the surface step cuts it.
pub fn surface(carriageway: &[[f64; 4]], walk: &[[f64; 4]]) -> Surface {
    let carriageway = rects(carriageway);
    let walk = poly::difference(&rects(walk), &carriageway);
    Surface { carriageway, walk, spanned: Vec::new(), ballast: Vec::new() }
}

/// Houses as rectangles, with no passages and no pockets: every mask the
/// facade step makes is the footprints themselves.
pub fn facade(houses: &[[f64; 4]]) -> Facade {
    let footprints = rects(houses);
    Facade {
        buildings: Vec::new(),
        solid: footprints.clone(),
        built: footprints.clone(),
        passages: Vec::new(),
        footprints,
    }
}

/// One carriageway sheet per region of `surface`'s asphalt, with no axes: a
/// flat paving the arrangement can cut and the mesh can triangulate, with
/// nothing to lift it by.
pub fn sheets(surface: &Surface) -> Sheets {
    Sheets {
        sheets: surface
            .carriageway
            .iter()
            .enumerate()
            .map(|(group, shape)| Sheet {
                family: Family::Carriageway,
                group,
                shapes: vec![shape.clone()],
                spans: Vec::new(),
                axes: Vec::new(),
                chords: Vec::new(),
                spanning: false,
            })
            .collect(),
    }
}

/// No profiles: a world with nothing solved.
pub fn profiles() -> Profiles {
    Profiles::default()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::Extent;
    use crate::pipeline::tests::bbox;
    use crate::poly::contains;
    use crate::step::Summary;
    use crate::world::{Arrangement, Material};

    /// **The room step, given a street and a house and nothing else.** A 6 m
    /// carriageway along x and a house front 2 m off its kerb: the strip
    /// between is paved to the wall, and nothing is paved on the open side.
    /// No ribbon, surface, kerb or legs step ran to make the input.
    #[test]
    fn the_room_paves_to_a_wall_it_is_given() {
        let paving = surface(&[[-50.0, -3.0, 50.0, 3.0]], &[]);
        let houses = facade(&[[-10.0, 5.0, 10.0, 15.0]]);
        let (room, s) = crate::room::run(&paving, &[], &houses);
        assert!(contains(&room.surface.walk, [0.0, 4.0]), "the strip to the wall: {s}");
        assert!(!contains(&room.surface.walk, [0.0, -4.0]), "nothing on the open side: {s}");
        assert!(!contains(&room.surface.walk, [0.0, 6.0]), "nothing inside the house: {s}");
        assert_eq!(room.surface.carriageway, paving.carriageway, "the asphalt is passed through: {s}");
    }

    /// **The arrangement and the mesh, given a paving and nothing else.** A
    /// street and a sidewalk as rectangles cut the rect into faces that
    /// partition it, and the one mesh over those faces has no crack.
    #[test]
    fn a_hand_made_paving_arranges_and_meshes_without_a_crack() {
        let extent = Extent::of(bbox());
        let paving = surface(&[[-50.0, -3.0, 50.0, 3.0]], &[[-50.0, 3.0, 50.0, 5.0]]);
        let (a, built): (Arrangement, Summary) =
            crate::arrangement::run(&extent, &[], &profiles(), &paving, &sheets(&paving));
        let s = built.and(crate::arrangement::check(&a, &extent));
        assert_eq!(s.num("unshared"), 0.0, "{s}");
        assert!(s.num("closure") <= 1.0, "{s}");
        assert!(a.of(Material::Carriageway).count() >= 1 && a.of(Material::Pavement).count() >= 1, "{s}");

        let (terrain, _) = crate::terrain::run(&extent, &mut crate::terrain::tests::dem("flat"), 20.0, usize::MAX);
        let (mesh, built) = crate::mesh::run(&terrain, &a);
        let s = built.and(crate::mesh::check(&mesh, &a));
        assert_eq!(s.get("crack"), Some("0.0e0"), "{s}");
        assert!(s.num("carriageway") > 0.0 && s.num("pavement") > 0.0, "{s}");
    }
}
