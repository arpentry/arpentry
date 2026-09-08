//! The mesh: the paved surface as triangles, on the ground.
//!
//! The flat plan's last step, done in three dimensions from the start.
//! Every paved region — the carriageway and the pavement, free walk bands
//! included — is triangulated **conforming to the terrain lattice**: each
//! region is ear-clipped, and each ear is then cut by the lattice's three
//! line families (the grid's columns, its rows, and the SW→NE diagonals of
//! its cells) into convex pieces, so that every triangle written lies
//! inside one triangle of the terrain. A vertex at [`height_at`] then puts
//! the whole triangle on the ground to the ulp — the `drape` step's
//! guarantee for lines, for areas. No profile is applied yet: the surfaces
//! lie *on* the raw ground, coplanar with it, which is correct and is what
//! the bench step moves.
//!
//! Two properties are built in rather than checked after:
//!
//! - **No cracks.** A cut point on an edge two ears share is computed from
//!   the edge's endpoints in one canonical order, so both ears get the same
//!   point bit for bit, and vertices are shared by exact position. The
//!   `seam` check reads the length of the mesh's one-sided edges against
//!   the regions' perimeter; a crack would add to it.
//! - **Nothing is dropped but the degenerate.** A cut along an edge that
//!   runs down a grid line leaves pieces of no area; those go. A sliver
//!   under [`SLIVER_M2`] is kept and counted — dropping it would open the
//!   crack its long edge spans — and is the bench step's to widen or the
//!   surface step's to avoid.
//!
//! The kernel is `earcutr` (the server's dependency) for the ears and a
//! Sutherland–Hodgman halving for the cuts; both are named in this file
//! and nowhere else.

use std::collections::HashMap;

use crate::grid::Grid;
use crate::poly::{self, Pt, Shapes};
use crate::step::Summary;
use crate::terrain::height_at;
use crate::world::{Mesh, Tri, World};

/// A triangle under this many square metres is a sliver: kept, counted.
pub const SLIVER_M2: f64 = 1e-6;

/// A hole under this many square metres — a square centimetre — is not a
/// hole but a kernel artefact (three lattice points a hair apart, sharing
/// their vertices with the hole beside them), and the ear clipper
/// double-covers the region around it. Dropped before clipping; the
/// region's area is read after.
pub const HOLE_MIN_M2: f64 = 1e-4;

/// How far, in square metres, the ears may disagree with their region
/// before the region counts as misread: a floor, plus this much per ear —
/// the rounding of one cross product at coordinates of a few kilometres.
/// A misread region loses or doubles whole triangles, square decimetres
/// at least; a region of a hundred thousand ears read right differs from
/// its own area by a tenth of a square millimetre.
const EAR_TOLERANCE_M2: f64 = 1e-6;
const EAR_TOLERANCE_PER_EAR_M2: f64 = 1e-8;

/// A piece under this many square metres has no area: a cut that ran along
/// an edge. Dropped.
const DEGENERATE_M2: f64 = 1e-12;

/// Two vertices within this many metres of each other are one vertex: a
/// hundredth of the polygon kernel's lattice, so nothing built stands that
/// far from anything else on purpose. It is not ulp noise that needs it.
/// Three lattice points in a row are all but never exactly collinear — the
/// middle one stands off the line by a nanometre to a micron — so the ear
/// clipper hands the same ring edge to two ears as two segments a micron
/// apart, and a cut line crosses them at two points a micron apart. Welded,
/// they are one vertex and the two ears meet; kept apart, every such
/// crossing is a T-junction. A triangle two of whose vertices weld together
/// is dropped.
pub const WELD_M: f64 = 1e-6;


/// A vertex within this many lattice units of a cut line is on it: it goes
/// to both sides and spawns no crossing. A cell corner reached by two
/// crossings in a row is on the diagonal by construction and off it by an
/// ulp in arithmetic; without the tolerance that ulp is a triangle of no
/// area whose dropping leaves a T-junction.
const ON_LINE: f64 = 1e-9;

/// What one family's triangulation found.
#[derive(Debug, Default, Clone, Copy)]
pub struct Stats {
    pub regions: usize,
    /// Regions the ear clipper refused.
    pub failed: usize,
    /// Regions the clipper misread at first and read right once the
    /// polygon kernel's union had separated what touched in them.
    pub washed: usize,
    /// Regions whose ears did not add up to their area even after that;
    /// meshed as best it could.
    pub lossy: usize,
    pub slivers: usize,
    /// Pieces of no area dropped.
    pub degenerate: usize,
    /// Pieces fanned from their centroid because three of their vertices
    /// in a row were collinear.
    pub centred: usize,
    /// Triangles dropped because two of their vertices welded into one.
    pub welded: usize,
    /// The area, in square metres, by which the triangles disagree with
    /// their regions, summed over regions.
    pub lost_m2: f64,
    /// The largest height, in metres, by which a triangle's plane stands
    /// off the terrain at its centroid.
    pub off_ground: f64,
    /// The mesh's one-sided edge length less the regions' perimeter, in
    /// metres: a crack shows here.
    pub seam: f64,
}

/// Triangulates the world's carriageway and pavement on its terrain.
pub fn run(world: &mut World) -> Summary {
    let terrain = world.terrain.as_ref().expect("the terrain step runs first");
    let none = Shapes::new();
    let carriageway = world.carriageway().unwrap_or(&none);
    let pavement = world.walk().unwrap_or(&none);
    let ground = |p: Pt| height_at(terrain, p[0], p[1]);
    let (c, cs) = triangulate(carriageway, &terrain.grid, &ground);
    let (p, ps) = triangulate(pavement, &terrain.grid, &ground);
    let summary = Summary::new()
        .with("carriageway", format!("{}/{}", c.indices.len() / 3, c.positions.len()))
        .with("pavement", format!("{}/{}", p.indices.len() / 3, p.positions.len()))
        .with("failed", cs.failed + ps.failed)
        .with("washed", cs.washed + ps.washed)
        .with("lossy", cs.lossy + ps.lossy)
        .with("slivers", cs.slivers + ps.slivers)
        .with("degenerate", cs.degenerate + ps.degenerate)
        .with("centred", cs.centred + ps.centred)
        .with("welded", cs.welded + ps.welded)
        .with("lost_m2", format!("{:.1e}", cs.lost_m2 + ps.lost_m2))
        .with("off_ground", format!("{:.1e}", cs.off_ground.max(ps.off_ground)))
        .with("seam", format!("{:.1e}", cs.seam.max(ps.seam)));
    world.mesh = Some(Mesh { carriageway: c, pavement: p });
    summary
}

/// `shapes` as triangles conforming to `grid`, every one inside one of
/// its cell triangles, each vertex at `height`, vertices shared by
/// position.
///
/// The height comes in as a function rather than being read off the
/// terrain, because the ground the surface stands on is not always the
/// terrain: the bench step triangulates the engineered ground on the same
/// lattice with the same guarantee.
pub fn triangulate(shapes: &Shapes, grid: &Grid, height: &dyn Fn(Pt) -> f64) -> (Tri, Stats) {
    let mut tri = Tri::default();
    let mut stats = Stats { regions: shapes.len(), ..Stats::default() };
    let mut index: HashMap<[i64; 2], u32> = HashMap::new();
    let mut vertex = |p: Pt, tri: &mut Tri| -> u32 {
        *index.entry([(p[0] / WELD_M).round() as i64, (p[1] / WELD_M).round() as i64]).or_insert_with(|| {
            tri.positions.push([p[0], p[1], height(p)]);
            (tri.positions.len() - 1) as u32
        })
    };
    let mut edges: HashMap<(u32, u32), u32> = HashMap::new();
    let mut perimeter = 0.0;
    for shape in shapes {
        for (shape, ears) in read(shape, &mut stats) {
            let want = poly::area(&vec![shape.clone()]);
            for ring in &shape {
                for i in 0..ring.len() {
                    let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
                    perimeter += (b[0] - a[0]).hypot(b[1] - a[1]);
                }
            }
            let mut got = 0.0;
            for ear in ears {
                for piece in split(ear.to_vec(), grid) {
                    for t in fan(&piece, &mut stats) {
                        let a = tri_area(t);
                        if a < SLIVER_M2 {
                            stats.slivers += 1;
                        }
                        got += a;
                        let ids = [vertex(t[0], &mut tri), vertex(t[1], &mut tri), vertex(t[2], &mut tri)];
                        if ids[0] == ids[1] || ids[1] == ids[2] || ids[0] == ids[2] {
                            stats.welded += 1;
                            continue;
                        }
                        tri.indices.extend_from_slice(&ids);
                        for e in 0..3 {
                            let (x, y) = (ids[e], ids[(e + 1) % 3]);
                            *edges.entry((x.min(y), x.max(y))).or_insert(0) += 1;
                        }
                        // The centroid's height on the triangle's plane is
                        // the vertex mean, exactly; a plane solve there is
                        // ill-conditioned on a sliver.
                        let c = [(t[0][0] + t[1][0] + t[2][0]) / 3.0, (t[0][1] + t[1][1] + t[2][1]) / 3.0];
                        let z = ids.iter().map(|&i| tri.positions[i as usize][2]).sum::<f64>() / 3.0;
                        stats.off_ground = stats.off_ground.max((z - height(c)).abs());
                    }
                }
            }
            stats.lost_m2 += (got - want).abs();
        }
    }
    let boundary: f64 = edges
        .iter()
        .filter(|(_, n)| **n == 1)
        .map(|((a, b), _)| {
            let (p, q) = (tri.positions[*a as usize], tri.positions[*b as usize]);
            (q[0] - p[0]).hypot(q[1] - p[1])
        })
        .sum();
    stats.seam = (boundary - perimeter).abs();
    (tri, stats)
}

/// The regions the clipper reads for `shape`, each with its ears: the
/// shape itself, cleaned; or — when the clipper misreads it (a hole
/// touching another, a spike a lattice cell wide) — the shapes the
/// kernel's union of it separates into, each cleaned and gated again.
/// What cannot be read either way is meshed as the clipper had it, and
/// counted. Every check downstream reads the rings returned here, never
/// the ones passed in, so the seam is measured against the boundary that
/// was actually triangulated.
fn read(shape: &poly::Shape, stats: &mut Stats) -> Vec<(poly::Shape, Vec<[Pt; 3]>)> {
    let Some(shape) = readable(shape) else {
        return Vec::new();
    };
    let want = poly::area(&vec![shape.clone()]);
    match ears(&shape, want) {
        Ok(e) => vec![(shape, e)],
        Err(None) => {
            stats.failed += 1;
            Vec::new()
        }
        Err(Some(e)) => {
            let washed: Vec<(poly::Shape, Option<Vec<[Pt; 3]>>)> = poly::union_all(&vec![shape.clone()])
                .iter()
                .filter_map(readable)
                .map(|s| {
                    let e = ears(&s, poly::area(&vec![s.clone()])).ok();
                    (s, e)
                })
                .collect();
            if !washed.is_empty() && washed.iter().all(|(_, e)| e.is_some()) {
                stats.washed += 1;
                washed.into_iter().map(|(s, e)| (s, e.expect("gated"))).collect()
            } else {
                stats.lossy += 1;
                vec![(shape, e)]
            }
        }
    }
}

/// `shape` as the clipper may read it: every ring cleaned, holes under
/// [`HOLE_MIN_M2`] gone; `None` if no area is left.
fn readable(shape: &poly::Shape) -> Option<poly::Shape> {
    let out: poly::Shape = shape
        .iter()
        .enumerate()
        .map(|(i, r)| (i, cleaned(r)))
        .filter(|(i, r)| r.len() >= 3 && (*i == 0 || poly::ring_area(r).abs() >= HOLE_MIN_M2))
        .map(|(_, r)| r)
        .collect();
    (!out.is_empty() && poly::area(&vec![out.clone()]) > 0.0).then_some(out)
}

/// The triangles of one convex piece. A fan from its first vertex, unless
/// that fan holds a triangle of no area — three vertices in a row on one
/// cut line, which happens where a cut runs through a vertex — in which
/// case the fan is from the centroid instead. Dropping the flat triangle
/// would leave its middle vertex on the inside of the next triangle's edge:
/// a T-junction, which a step that moves the vertex opens into a slit. A
/// piece of no area at all is dropped whole and counted.
fn fan(piece: &[Pt], stats: &mut Stats) -> Vec<[Pt; 3]> {
    if piece.len() < 3 {
        return Vec::new();
    }
    let plain: Vec<[Pt; 3]> = (1..piece.len() - 1).map(|k| [piece[0], piece[k], piece[k + 1]]).collect();
    if plain.iter().all(|t| tri_area(*t) >= DEGENERATE_M2) {
        return plain;
    }
    if poly::ring_area(&piece.to_vec()) < DEGENERATE_M2 {
        stats.degenerate += 1;
        return Vec::new();
    }
    stats.centred += 1;
    let n = piece.len() as f64;
    let c = [piece.iter().map(|p| p[0]).sum::<f64>() / n, piece.iter().map(|p| p[1]).sum::<f64>() / n];
    (0..piece.len())
        .map(|k| [c, piece[k], piece[(k + 1) % piece.len()]])
        .filter(|t| tri_area(*t) >= DEGENERATE_M2)
        .collect()
}

fn tri_area(t: [Pt; 3]) -> f64 {
    0.5 * ((t[1][0] - t[0][0]) * (t[2][1] - t[0][1]) - (t[2][0] - t[0][0]) * (t[1][1] - t[0][1]))
}

/// A vertex within this many metres of the line between its neighbours is
/// on it: the polygon kernel's lattice, below which a ring carries no
/// shape. A road's straight edge comes out of the kernel as a chain of
/// vertices a metre apart that zigzag by a few hundredths of a millimetre;
/// the ear clipper reads each triple as a sliver ear and, stuck on the
/// next, cures itself by dropping a vertex some other ear still uses — a
/// T-junction on the boundary at every second vertex.
pub const COLLINEAR_M: f64 = 1e-4;

/// `ring` without the vertices that add no shape: a repeated point, a point
/// within [`COLLINEAR_M`] of the segment between its neighbours, and the
/// tip of a spike — a vertex the ring doubles back at, one neighbour lying
/// on the segment to the other within the same tolerance — which the
/// kernel leaves a lattice cell wide along a boundary and which, meshed,
/// is two boundary edges a micron apart that weld into one. A spike of
/// several vertices collapses from its tip. The ring's area and perimeter
/// move by less than the lattice per vertex removed.
fn cleaned(ring: &[Pt]) -> Vec<Pt> {
    let mut out: Vec<Pt> = ring.to_vec();
    loop {
        let n = out.len();
        if n < 3 {
            return out;
        }
        let Some(i) = (0..n).find(|&i| {
            let (a, b, c) = (out[(i + n - 1) % n], out[i], out[(i + 1) % n]);
            a == b
                || poly::segment_distance(a, c, b) < COLLINEAR_M
                || poly::segment_distance(a, b, c) < COLLINEAR_M
                || poly::segment_distance(b, c, a) < COLLINEAR_M
        }) else {
            return out;
        };
        out.remove(i);
    }
}

/// The ears of one region of area `want`, counter-clockwise. `Err(None)`
/// if the clipper refused it; `Err(Some(ears))` if its ears do not add up
/// to the region's area, which is how the clipper reports a region it
/// could not read.
fn ears(shape: &poly::Shape, want: f64) -> Result<Vec<[Pt; 3]>, Option<Vec<[Pt; 3]>>> {
    let ears = ear_clip(shape).ok_or(None)?;
    let got: f64 = ears.iter().map(|e| tri_area(*e)).sum();
    if (got - want).abs() <= EAR_TOLERANCE_M2 + EAR_TOLERANCE_PER_EAR_M2 * ears.len() as f64 {
        Ok(ears)
    } else {
        Err(Some(ears))
    }
}

/// The clipper's ears, counter-clockwise, or `None` if it refused.
fn ear_clip(shape: &poly::Shape) -> Option<Vec<[Pt; 3]>> {
    let mut coords: Vec<f64> = Vec::new();
    let mut holes: Vec<usize> = Vec::new();
    for (i, ring) in shape.iter().enumerate() {
        if i > 0 {
            holes.push(coords.len() / 2);
        }
        for p in ring {
            coords.push(p[0]);
            coords.push(p[1]);
        }
    }
    let idx = earcutr::earcut(&coords, &holes, 2).ok()?;
    let at = |i: usize| [coords[2 * i], coords[2 * i + 1]];
    Some(
        idx.chunks_exact(3)
            .map(|t| {
                let tri = [at(t[0]), at(t[1]), at(t[2])];
                if tri_area(tri) < 0.0 {
                    [tri[0], tri[2], tri[1]]
                } else {
                    tri
                }
            })
            .collect(),
    )
}

/// `poly`, convex and counter-clockwise, cut by the lattice's columns, rows
/// and cell diagonals into convex pieces each inside one terrain triangle.
pub fn split(poly: Vec<Pt>, grid: &Grid) -> Vec<Vec<Pt>> {
    let (x0, y0, dx, dy) = (grid.x0, grid.y0, grid.dx, grid.dy);
    let families: [Box<dyn Fn(Pt) -> f64>; 3] = [
        Box::new(move |p: Pt| (p[0] - x0) / dx),
        Box::new(move |p: Pt| (p[1] - y0) / dy),
        Box::new(move |p: Pt| (p[0] - x0) / dx - (p[1] - y0) / dy),
    ];
    let mut pieces = vec![poly];
    for f in &families {
        let mut next = Vec::with_capacity(pieces.len());
        for piece in pieces {
            let vals: Vec<f64> = piece.iter().map(|&p| f(p)).collect();
            let lo = vals.iter().copied().fold(f64::INFINITY, f64::min).ceil() as i64;
            let hi = vals.iter().copied().fold(f64::NEG_INFINITY, f64::max).floor() as i64;
            let mut rest = piece;
            for k in lo..=hi {
                let (below, above) = halve(&rest, f, k as f64);
                if below.len() >= 3 {
                    next.push(below);
                }
                rest = above;
                if rest.len() < 3 {
                    break;
                }
            }
            if rest.len() >= 3 {
                next.push(rest);
            }
        }
        pieces = next;
    }
    pieces
}

/// `piece` cut by the line `f = k`: the part with `f ≤ k` and the part
/// with `f ≥ k`, in the piece's winding. A vertex on the line (within
/// [`ON_LINE`]) belongs to both; a crossing point is computed from the
/// edge's endpoints in lexicographic order, so the two pieces of an edge
/// two polygons share agree bit for bit.
pub fn halve(piece: &[Pt], f: &dyn Fn(Pt) -> f64, k: f64) -> (Vec<Pt>, Vec<Pt>) {
    let (mut below, mut above) = (Vec::new(), Vec::new());
    let n = piece.len();
    for i in 0..n {
        let (a, b) = (piece[i], piece[(i + 1) % n]);
        let on = |s: f64| if s.abs() < ON_LINE { 0.0 } else { s };
        let (sa, sb) = (on(f(a) - k), on(f(b) - k));
        if sa <= 0.0 {
            below.push(a);
        }
        if sa >= 0.0 {
            above.push(a);
        }
        if (sa < 0.0 && sb > 0.0) || (sa > 0.0 && sb < 0.0) {
            let p = crossing(a, b, f, k);
            below.push(p);
            above.push(p);
        }
    }
    (below, above)
}

/// The point of `a→b` where `f = k`, from the endpoints in canonical order.
fn crossing(a: Pt, b: Pt, f: &dyn Fn(Pt) -> f64, k: f64) -> Pt {
    let (lo, hi) = if a <= b { (a, b) } else { (b, a) };
    let (flo, fhi) = (f(lo), f(hi));
    let t = ((k - flo) / (fhi - flo)).clamp(0.0, 1.0);
    [lo[0] + (hi[0] - lo[0]) * t, lo[1] + (hi[1] - lo[1]) * t]
}

#[cfg(test)]
pub(crate) mod tests {
    use std::path::Path;

    use crate::frame::Rect;
    use crate::terrain::{self, tests::dem};
    use crate::{drape, facade, fillet, kerb, profile, ribbon, room, surface};

    use super::*;

    /// A world on `terrain_spec` with the network of `net`, built through
    /// the room step and meshed.
    pub(crate) fn world(terrain_spec: &str, net: &str) -> (World, Summary) {
        let mut w = terrain::tests::world();
        terrain::run(&mut w, &mut dem(terrain_spec), 5.0, usize::MAX);
        drape::run(&mut w, Path::new(net)).unwrap();
        profile::run(&mut w);
        facade::run(&mut w, None).unwrap();
        ribbon::run(&mut w);
        surface::run(&mut w);
        kerb::run(&mut w);
        fillet::run(&mut w);
        room::run(&mut w);
        let s = run(&mut w);
        (w, s)
    }

    fn area_of(tri: &Tri) -> f64 {
        tri.indices
            .chunks_exact(3)
            .map(|t| {
                let p = |i: u32| {
                    let v = tri.positions[i as usize];
                    [v[0], v[1]]
                };
                tri_area([p(t[0]), p(t[1]), p(t[2])])
            })
            .sum()
    }

    #[test]
    fn cleaning_drops_collinear_vertices_and_spikes() {
        // A square with a collinear point on one side, a repeated corner,
        // and a spike a hair wide off another side.
        let ring = vec![
            [0.0, 0.0],
            [5.0, 0.00001],
            [10.0, 0.0],
            [10.0, 0.0],
            [10.0, 10.0],
            [4.0, 10.0],
            [4.00001, 13.0],
            [3.99999, 13.0],
            [3.99998, 10.0],
            [0.0, 10.0],
        ];
        let out = cleaned(&ring);
        assert_eq!(out.len(), 4, "{out:?}");
        assert!((poly::ring_area(&out) - 100.0).abs() < 1e-3);
        // A genuine notch a decimetre wide stays.
        let notched = vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [4.0, 10.0], [4.0, 8.0], [3.9, 8.0], [3.9, 10.0], [0.0, 10.0]];
        assert_eq!(cleaned(&notched).len(), 8);
    }

    #[test]
    fn a_piece_with_three_collinear_vertices_fans_from_its_centroid() {
        let mut stats = Stats::default();
        let piece = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [2.0, 1.0]];
        let tris = fan(&piece, &mut stats);
        assert_eq!(stats.centred, 1);
        assert_eq!(tris.len(), 4);
        let area: f64 = tris.iter().map(|t| tri_area(*t)).sum();
        assert!((area - 1.0).abs() < 1e-12, "{area}");
        assert!(tris.iter().all(|t| tri_area(*t) > 0.0));
        // Every edge of the piece is an edge of exactly one triangle.
        for k in 0..4 {
            let (a, b) = (piece[k], piece[(k + 1) % 4]);
            let n = tris.iter().filter(|t| (0..3).any(|i| t[i] == a && t[(i + 1) % 3] == b)).count();
            assert_eq!(n, 1, "{a:?}->{b:?}");
        }
        // A convex piece with no such triple fans plainly, and one with no
        // area is dropped.
        assert_eq!(fan(&[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]], &mut stats).len(), 2);
        assert!(fan(&[[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]], &mut stats).is_empty());
        assert_eq!((stats.centred, stats.degenerate), (1, 1));
    }

    #[test]
    fn halving_a_square_is_exact() {
        let sq = vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
        let f = |p: Pt| p[0];
        let (below, above) = halve(&sq, &f, 0.25);
        assert_eq!(below, vec![[0.0, 0.0], [0.25, 0.0], [0.25, 1.0], [0.0, 1.0]]);
        assert_eq!(above, vec![[0.25, 0.0], [1.0, 0.0], [1.0, 1.0], [0.25, 1.0]]);
        // On the line: the vertex belongs to both, and nothing is cut.
        let (below, above) = halve(&sq, &f, 1.0);
        assert_eq!(below.len(), 4);
        assert_eq!(above, vec![[1.0, 0.0], [1.0, 1.0]]);
    }

    #[test]
    fn split_puts_every_piece_in_one_cell_triangle() {
        let grid = Grid::fit(&Rect { x0: 0.0, y0: 0.0, x1: 10.0, y1: 10.0 }, 1.0, usize::MAX);
        let tri = vec![[0.3, 0.2], [7.9, 1.4], [3.1, 8.6]];
        let pieces = split(tri.clone(), &grid);
        let total: f64 = pieces.iter().map(|p| poly::ring_area(p)).sum();
        assert!((total - tri_area([tri[0], tri[1], tri[2]])).abs() < 1e-9, "{total}");
        assert!(pieces.len() > 40, "{}", pieces.len());
        for piece in &pieces {
            assert!(poly::ring_area(piece) > 0.0, "{piece:?}");
            let (u, v): (Vec<f64>, Vec<f64>) = piece.iter().map(|p| grid.to_uv(p[0], p[1])).unzip();
            let (cu, cv) = grid.cell(u.iter().sum::<f64>() / u.len() as f64, v.iter().sum::<f64>() / v.len() as f64);
            let upper = u.iter().zip(&v).map(|(u, v)| (u - cu as f64) - (v - cv as f64)).sum::<f64>() >= 0.0;
            for (u, v) in u.iter().zip(&v) {
                let (fu, fv) = (u - cu as f64, v - cv as f64);
                assert!((-1e-9..=1.0 + 1e-9).contains(&fu) && (-1e-9..=1.0 + 1e-9).contains(&fv), "{piece:?}");
                if upper {
                    assert!(fu >= fv - 1e-9, "{piece:?} straddles the diagonal");
                } else {
                    assert!(fu <= fv + 1e-9, "{piece:?} straddles the diagonal");
                }
            }
        }
    }

    #[test]
    fn a_straight_on_flat_ground_meshes_to_its_area() {
        let (w, s) = world("flat", "net:straight?len=200");
        let m = w.mesh.as_ref().unwrap();
        let want = poly::area(w.carriageway().unwrap());
        assert!((area_of(&m.carriageway) - want).abs() / want < 1e-9, "{} vs {want}", area_of(&m.carriageway));
        assert!(m.carriageway.positions.iter().all(|p| p[2] == 400.0));
        assert!(m.pavement.indices.is_empty());
        assert_eq!(s.get("failed"), Some("0"), "{s}");
        assert!(s.num("lost_m2") < 1e-9 && s.num("off_ground") < 1e-9 && s.num("seam") < 1e-9, "{s}");
        assert_eq!(s.get("degenerate"), Some("0"), "{s}");
        // Every triangle counter-clockwise, seen from above.
        for t in m.carriageway.indices.chunks_exact(3) {
            let p = |i: u32| [m.carriageway.positions[i as usize][0], m.carriageway.positions[i as usize][1]];
            assert!(tri_area([p(t[0]), p(t[1]), p(t[2])]) > 0.0);
        }
    }

    #[test]
    fn a_mesh_on_a_hill_lies_on_the_ground() {
        let (w, s) = world("hill?amp=60&radius=400", "net:cross?len=400");
        let t = w.terrain.as_ref().unwrap();
        let m = w.mesh.as_ref().unwrap();
        assert!(s.num("off_ground") < 1e-9 && s.num("seam") < 1e-9 && s.num("lost_m2") < 1e-9, "{s}");
        // Not only the centroid: points across every triangle.
        for tri in m.carriageway.indices.chunks_exact(3) {
            let [a, b, c] = [tri[0], tri[1], tri[2]].map(|i| m.carriageway.positions[i as usize]);
            for (wa, wb) in [(0.2, 0.3), (0.6, 0.1), (0.1, 0.8), (0.05, 0.05)] {
                let wc = 1.0 - wa - wb;
                let x = wa * a[0] + wb * b[0] + wc * c[0];
                let y = wa * a[1] + wb * b[1] + wc * c[1];
                let z = wa * a[2] + wb * b[2] + wc * c[2];
                assert!((z - height_at(t, x, y)).abs() < 1e-9, "({x}, {y}): {z} vs {}", height_at(t, x, y));
            }
        }
        assert!(m.carriageway.positions.iter().any(|p| p[2] > 401.0), "the cross climbs the hill");
    }

    #[test]
    fn the_pavement_is_meshed_beside_the_road() {
        let (w, s) = world("flat", "net:sidewalk?d=6");
        let m = w.mesh.as_ref().unwrap();
        let want = poly::area(w.walk().unwrap());
        assert!(want > 0.0);
        assert!((area_of(&m.pavement) - want).abs() / want < 1e-9, "{} vs {want}", area_of(&m.pavement));
        assert!(s.num("seam") < 1e-9, "{s}");
        // The two families share no vertex array: the kerb is drawn twice,
        // once by each surface, at one height.
        let kerb: Vec<&[f64; 3]> = m.pavement.positions.iter().filter(|p| (p[1] - 2.75).abs() < 1e-9).collect();
        assert!(!kerb.is_empty());
        assert!(kerb.iter().all(|p| p[2] == 400.0));
    }
}
