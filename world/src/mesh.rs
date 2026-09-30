//! The mesh step: the whole rect as one triangulation, on the ground.
//!
//! Every face of the arrangement, both layers, triangulated in one pass by
//! [`crate::triangulate::tagged`], conforming to the terrain lattice and at
//! the natural ground. The construction and the three repairs that make the
//! one mesh crack-free are the triangulation module's; this step is the call
//! and the line that says what it came to.

use crate::lattice::height_at;
use crate::poly::{Pt, Shapes};
use crate::step::Summary;
use crate::triangulate::{one_sided, tagged};
use crate::world::{Arrangement, Material, Mesh, Terrain};

/// Triangulates every face of the arrangement, both layers, in one pass.
///
/// **One call, so one vertex per position.** [`tagged`] welds by position
/// across everything it is given and tags each triangle with the shape it
/// came from, so the paving and the ground beside it share their boundary
/// vertices by index, and no seam between them has to be found again by
/// position afterwards. Which surface a vertex is on is a question for the
/// `lift` and `earthwork` steps, which copy it once per surface that reaches
/// it ([`crate::copies::Copies`]).
///
/// A deck face is the same shape as the partition face under it, so it is
/// triangulated the same way on the same vertices, and told apart by its tag.
pub fn run(terrain: &Terrain, arrangement: &Arrangement) -> (Mesh, Summary) {
    let ground = |p: Pt| height_at(terrain, p[0], p[1]);
    let shapes: Shapes = arrangement.all().map(|f| f.shape.clone()).collect();
    let (tri, of_face, stats) = tagged(&shapes, &terrain.grid, &ground);
    let mesh = Mesh { tri, of_face };
    let summary = Summary::new()
        .with("failed", stats.failed)
        .with("washed", stats.washed)
        .with("lossy", stats.lossy)
        .with("slivers", stats.slivers)
        .with("degenerate", stats.degenerate)
        .with("centred", stats.centred)
        .with("unmerged", stats.unmerged)
        .with("welded", stats.welded)
        .with("joined", stats.joined)
        .with("junctions", stats.junctions)
        .with("lost_m2", format!("{:.1e}", stats.lost_m2))
        .with("off_ground", format!("{:.1e}", stats.off_ground));
    (mesh, summary)
}

/// The triangles per material, and the cracks.
pub fn check(mesh: &Mesh, arrangement: &Arrangement) -> Summary {
    let count = |m: Material| mesh.of_face.iter().filter(|&&f| arrangement.face(f).material == m).count();
    Summary::new()
        .with("triangles", mesh.of_face.len())
        .with("vertices", mesh.tri.positions.len())
        .with("ground", count(Material::Ground))
        .with("carriageway", count(Material::Carriageway))
        .with("pavement", count(Material::Pavement))
        .with("ballast", count(Material::Ballast))
        // The partition is a partition of the rect, so meshed as one its only
        // one-sided edges are the rect's own border. Anything else is a crack:
        // an edge one face subdivided and its neighbour did not.
        .with("crack", format!("{:.1e}", crack_m(mesh, arrangement)))
}

/// The length, in metres, of the partition's one-sided edges that are not
/// on the rect's border: every crack in the one mesh, summed in a defined
/// order. The decks are left out — they lie over partition faces and are
/// closed by their slabs, not by a neighbour.
fn crack_m(mesh: &Mesh, arrangement: &Arrangement) -> f64 {
    let border = mesh.on_border();
    let once = one_sided(&mesh.tri.indices, |i| arrangement.in_partition(mesh.of_face[i]));
    once.iter()
        .filter(|&&(a, b)| !(border(a) && border(b)))
        .map(|&(a, b)| (mesh.tri.positions[a as usize], mesh.tri.positions[b as usize]))
        .map(|(p, q)| (q[0] - p[0]).hypot(q[1] - p[1]))
        .sum::<f64>()
        + 0.0
}

