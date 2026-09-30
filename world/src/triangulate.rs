//! The triangulation: regions as triangles conforming to the terrain lattice.
//!
//! Every face of the arrangement — the paving, the ground, and the decks
//! over them — is triangulated **conforming to the terrain lattice** in one
//! pass, and **cut first, triangulated after** ([`cut_first`]): the face's
//! part in each terrain triangle — bounded by the lattice's three line
//! families (the grid's columns, its rows, and the SW→NE diagonals of its
//! cells) and by the face's own rings — is found, and only then
//! triangulated, inside that one terrain triangle. So every triangle written
//! lies inside one triangle of the terrain, and none has an edge drawn across
//! a lattice line near a lattice vertex, which is where the needles came
//! from. A vertex at [`crate::lattice::height_at`] then puts the whole
//! triangle on the ground to the ulp — the `drape` step's guarantee for
//! lines, for areas. No profile is applied here: the `lift` step moves the
//! paving off the ground and the `earthwork` step benches the ground.
//!
//! **One vertex per position, and no crack.** Vertices are welded by exact
//! position across every face, so two faces that share an edge share its
//! vertices by index, and `crack` — the partition's one-sided edges away
//! from the rect's border — must read 0. Three things make it so:
//!
//! - **Rings are cleaned together** (`cleaned_together`): a vertex that is
//!   a corner of one face and collinear in its neighbour stays in both.
//! - **A degenerate ear leaves no T-junction** (`close_t_junctions`).
//! - **Twin corners a grid step apart are one** (`weld_open`), along open
//!   edges only.
//!
//! A cut point on an edge two ears share is computed from the edge's
//! endpoints in one canonical order, so both ears get the same point bit for
//! bit. A sliver under `SLIVER_M2` is kept and counted — dropping it would
//! open the crack its long edge spans. What slivers are left are the rings'
//! own: a ring vertex or a ring edge within microns of a lattice line or a
//! lattice vertex, which no triangulation inside the terrain triangle can
//! avoid.
//!
//! The kernel is `earcutr` (the server's dependency) for the ears — of the
//! face, which the cut is found through, and of each terrain triangle's
//! part — a Sutherland–Hodgman halving for the cuts, and Lawson's flips for
//! the shape; all are named in this file and nowhere else.

use std::collections::HashMap;

use crate::line;
use crate::grid::Grid;
use crate::poly::{self, Pt, Shapes};
use crate::world::Tri;

/// A triangle under this many square metres is a sliver: kept, counted.
const SLIVER_M2: f64 = 1e-6;

/// A hole under this many square metres — a square centimetre — is not a
/// hole but a kernel artefact (three lattice points a hair apart, sharing
/// their vertices with the hole beside them), and the ear clipper
/// double-covers the region around it. Dropped before clipping; the
/// region's area is read after.
const HOLE_MIN_M2: f64 = 1e-4;

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
///
/// **A needle is not this constant's to fix.** A needle — a triangle of a
/// few square microns whose long edge is a metre — is harmless in f64, where
/// every check here runs, but the glTF writer rounds positions to `f32`,
/// whose ulp at a few hundred metres is wider than the needle, and the
/// triangle collapses or *inverts*. Welding at [`poly::GRID_M`] would drop
/// each needle only after it exists, leaving its long interior edge open as
/// a slit, and the needles are not made by the lattice cut either. They come
/// from the rings: [`ear_clip`] adds no vertices, so a needle's three
/// corners are all ring corners, two of them a neck the kernel's own grid
/// cannot resolve — [`cleaned`] only ever compares a vertex with its
/// *neighbours*, so a neck between two parts of a ring walks straight
/// through it. Closing the neck in the ring, before the clipper, is where
/// the fix belongs. (Those are the ring's needles. The far more numerous
/// ones the lattice cut made across the clipper's own diagonals are gone
/// since the face is cut first: [`cut_first`].)
pub const WELD_M: f64 = 1e-6;

/// A vertex within this many lattice units of a cut line is on it: it goes
/// to both sides and spawns no crossing. A cell corner reached by two
/// crossings in a row is on the diagonal by construction and off it by an
/// ulp in arithmetic; without the tolerance that ulp is a triangle of no
/// area whose dropping leaves a T-junction. It is not what makes the needles
/// [`WELD_M`] describes, so it is not widened for them.
const ON_LINE: f64 = 1e-9;

/// What a triangulation found.
#[derive(Debug, Default, Clone, Copy)]
pub struct Stats {
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
    /// Terrain triangles whose pieces did not merge into an outline, and
    /// were fanned piece by piece ([`cut_first`]).
    pub unmerged: usize,
    /// Triangles dropped because two of their vertices welded into one.
    pub welded: usize,
    /// T-junctions closed: a triangle edge split at a vertex that lay on it
    /// (`close_t_junctions`).
    pub junctions: usize,
    /// Vertices of open edges merged into a twin within the kernel's grid
    /// (`weld_open`).
    pub joined: usize,
    /// The area, in square metres, by which the triangles disagree with
    /// their regions, summed over regions.
    pub lost_m2: f64,
    /// The largest height, in metres, by which a triangle's plane stands
    /// off the terrain at its centroid.
    pub off_ground: f64,
}

/// `shapes` as triangles conforming to `grid`, every one inside one of
/// its cell triangles, each vertex at `height`, vertices shared by
/// position.
///
/// The height comes in as a function rather than being read off the
/// terrain, because what a region stands on is not always the terrain: the
/// structure step meshes a bore's floor at the road's own height.
pub fn triangulate(shapes: &Shapes, grid: &Grid, height: &dyn Fn(Pt) -> f64) -> (Tri, Stats) {
    let (tri, _, stats) = tagged(shapes, grid, height);
    (tri, stats)
}

/// The same, with the region every triangle came from: one entry per
/// triangle, indexing `shapes`.
///
/// **This is what lets the whole rect be meshed at once.** Vertices are
/// welded by position across everything given, so passing every face of the
/// arrangement in one call gives one vertex array with the materials sharing
/// their boundary vertices; the tag says which face, and so which material,
/// each triangle is.
pub fn tagged(
    shapes: &Shapes,
    grid: &Grid,
    height: &dyn Fn(Pt) -> f64,
) -> (Tri, Vec<u32>, Stats) {
    let mut of_region: Vec<u32> = Vec::new();
    let mut tri = Tri::default();
    let mut stats = Stats::default();
    let mut index: HashMap<[i64; 2], u32> = HashMap::new();
    let mut vertex = |p: Pt, tri: &mut Tri| -> u32 {
        *index.entry([(p[0] / WELD_M).round() as i64, (p[1] / WELD_M).round() as i64]).or_insert_with(|| {
            tri.positions.push([p[0], p[1], height(p)]);
            (tri.positions.len() - 1) as u32
        })
    };
    let shapes = cleaned_together(shapes);
    for (region, shape) in shapes.iter().enumerate() {
        for (shape, ears) in read(shape, &mut stats) {
            let want = poly::area(std::slice::from_ref(&shape));
            let mut got = 0.0;
            for t in cut_first(&shape, &ears, grid, &mut stats) {
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
                // The centroid's height on the triangle's plane is the vertex
                // mean, exactly; a plane solve there is ill-conditioned on a
                // sliver.
                let c = [(t[0][0] + t[1][0] + t[2][0]) / 3.0, (t[0][1] + t[1][1] + t[2][1]) / 3.0];
                let z = ids.iter().map(|&i| tri.positions[i as usize][2]).sum::<f64>() / 3.0;
                stats.off_ground = stats.off_ground.max((z - height(c)).abs());
            }
            stats.lost_m2 += (got - want).abs();
            of_region.resize(tri.indices.len() / 3, region as u32);
        }
    }
    stats.joined = weld_open(&mut tri, &mut of_region);
    stats.junctions = close_t_junctions(&mut tri, &mut of_region);
    (tri, of_region, stats)
}

/// Every edge that exactly one of the triangles `keep` accepts uses, as
/// `(low, high)` vertex pairs in ascending order.
///
/// Counted by sorting packed keys rather than in a map: the one mesh has
/// tens of millions of edge uses, and a hash map of them would be most of
/// the mesh step's time.
pub fn one_sided(indices: &[u32], keep: impl Fn(usize) -> bool) -> Vec<(u32, u32)> {
    let mut keys: Vec<u64> = Vec::with_capacity(indices.len());
    for (i, t) in indices.chunks_exact(3).enumerate() {
        if !keep(i) {
            continue;
        }
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            keys.push(((a.min(b) as u64) << 32) | a.max(b) as u64);
        }
    }
    keys.sort_unstable();
    keys.chunk_by(|x, y| x == y)
        .filter(|run| run.len() == 1)
        .map(|run| ((run[0] >> 32) as u32, run[0] as u32))
        .collect()
}

/// Merges every vertex of an open edge into a twin within [`poly::GRID_M`]
/// that is also on one, and returns how many it merged.
///
/// The mesh welds at [`WELD_M`], a hundredth of the kernel's grid, which is
/// right inside a face. It is not right where the slice has left two corners
/// a grid step apart that are one point — a needle face of no area between
/// them, the edge one neighbour runs from the first and the other from the
/// second. Each neighbour then cuts that edge at the lattice from its own
/// start, and the two sets of crossings land ~1e-5 m apart along the whole
/// of it: a carriageway and the ground then share a kerb line and not one
/// vertex along it. Only open edges are asked, so a
/// mesh with none is untouched; a triangle two of whose corners merge is
/// dropped, as a weld drops it.
fn weld_open(tri: &mut Tri, of_region: &mut Vec<u32>) -> usize {
    let mut ends: Vec<u32> = one_sided(&tri.indices, |_| true).into_iter().flat_map(|e| [e.0, e.1]).collect();
    ends.sort_unstable();
    ends.dedup();
    let cell = |p: [f64; 3]| ((p[0] / poly::GRID_M).floor() as i64, (p[1] / poly::GRID_M).floor() as i64);
    let mut grid: HashMap<(i64, i64), Vec<u32>> = HashMap::new();
    for &v in &ends {
        grid.entry(cell(tri.positions[v as usize])).or_default().push(v);
    }
    // Each vertex into the lowest-numbered twin it has, so the answer is a
    // function of the mesh and not of an order.
    let mut into: HashMap<u32, u32> = HashMap::new();
    for &v in &ends {
        let p = tri.positions[v as usize];
        let (cx, cy) = cell(p);
        let mut best = v;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for &w in grid.get(&(cx + dx, cy + dy)).into_iter().flatten() {
                    let q = tri.positions[w as usize];
                    if w < best && (q[0] - p[0]).abs() <= poly::GRID_M && (q[1] - p[1]).abs() <= poly::GRID_M {
                        best = w;
                    }
                }
            }
        }
        if best != v {
            into.insert(v, best);
        }
    }
    if into.is_empty() {
        return 0;
    }
    let root = |mut v: u32| {
        while let Some(&w) = into.get(&v) {
            v = w;
        }
        v
    };
    let (mut indices, mut regions) = (Vec::with_capacity(tri.indices.len()), Vec::with_capacity(of_region.len()));
    for (t, &r) in tri.indices.chunks_exact(3).zip(of_region.iter()) {
        let m = [root(t[0]), root(t[1]), root(t[2])];
        if m[0] == m[1] || m[1] == m[2] || m[0] == m[2] {
            continue;
        }
        indices.extend_from_slice(&m);
        regions.push(r);
    }
    tri.indices = indices;
    *of_region = regions;
    into.len()
}

/// Splits every triangle edge that has another boundary edge's vertex lying
/// on it, and returns how many such vertices it found.
///
/// **A degenerate ear leaves one.** The ear clipper, bridging a hole to
/// the next along the line a ring's own edge runs on, cuts an ear whose
/// three corners are collinear — at a portal, for one, where the
/// carriageway's end and the ground's hole corner meet a bridge running on
/// along the same kerb line. The ear has no area and is dropped, and the
/// triangle on the far side of the ring's edge is left spanning two of its
/// corners with the third lying on it: a crack along that edge.
/// Every vertex involved is already a vertex of the mesh, so the repair adds
/// none; it only fans the one triangle out from the vertex that was skipped.
///
/// Only one-sided edges are asked, so a mesh that has none — every edge
/// welded to a neighbour, the rect's own border aside — passes through
/// untouched, and so does a single region meshed on its own, whose boundary
/// has no stranger's vertex on it.
fn close_t_junctions(tri: &mut Tri, of_region: &mut Vec<u32>) -> usize {
    // The kernel's grid, not the weld: a crossing the far side computed off a
    // twin of this edge lies a few 1e-5 m off it, and is on it.
    const ON_M: f64 = poly::GRID_M;
    const CELL: f64 = 1.0;
    let mut closed = 0usize;
    // A few passes: a triangle split along one edge may carry a second.
    for _ in 0..4 {
        let once: std::collections::HashSet<(u32, u32)> = one_sided(&tri.indices, |_| true).into_iter().collect();
        if once.is_empty() {
            break;
        }
        // The one-sided edges, as (triangle, which edge of it).
        let mut open: Vec<(usize, usize)> = Vec::new();
        let mut ends: std::collections::BTreeSet<u32> = Default::default();
        for (i, t) in tri.indices.chunks_exact(3).enumerate() {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                if once.contains(&(a.min(b), a.max(b))) {
                    open.push((i, k));
                    ends.insert(a);
                    ends.insert(b);
                }
            }
        }
        let cell = |p: [f64; 3]| ((p[0] / CELL).floor() as i64, (p[1] / CELL).floor() as i64);
        let mut grid: HashMap<(i64, i64), Vec<u32>> = HashMap::new();
        for &v in &ends {
            grid.entry(cell(tri.positions[v as usize])).or_default().push(v);
        }
        // Per triangle, its one edge to split and the vertices on it, in order.
        let mut splits: std::collections::BTreeMap<usize, (usize, Vec<u32>)> = Default::default();
        for &(i, k) in &open {
            if splits.contains_key(&i) {
                continue;
            }
            let t = &tri.indices[3 * i..3 * i + 3];
            let (a, b) = (t[k], t[(k + 1) % 3]);
            let (p, q) = (tri.positions[a as usize], tri.positions[b as usize]);
            let (c0, c1) = (cell(p), cell(q));
            let mut on: Vec<(f64, u32)> = Vec::new();
            for cx in c0.0.min(c1.0)..=c0.0.max(c1.0) {
                for cy in c0.1.min(c1.1)..=c0.1.max(c1.1) {
                    for &w in grid.get(&(cx, cy)).into_iter().flatten() {
                        if w == a || w == b {
                            continue;
                        }
                        let r = tri.positions[w as usize];
                        let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
                        let len2 = dx * dx + dy * dy;
                        if len2 <= 0.0 {
                            continue;
                        }
                        let s = ((r[0] - p[0]) * dx + (r[1] - p[1]) * dy) / len2;
                        let off = ((r[0] - p[0]) * dy - (r[1] - p[1]) * dx).abs() / len2.sqrt();
                        let along = s * len2.sqrt();
                        if off < ON_M && along > ON_M && (1.0 - s) * len2.sqrt() > ON_M {
                            on.push((s, w));
                        }
                    }
                }
            }
            if !on.is_empty() {
                on.sort_by(|x, y| x.partial_cmp(y).expect("finite"));
                on.dedup_by_key(|x| x.1);
                splits.insert(i, (k, on.into_iter().map(|x| x.1).collect()));
            }
        }
        if splits.is_empty() {
            break;
        }
        for (i, (k, on)) in splits {
            let t = [tri.indices[3 * i], tri.indices[3 * i + 1], tri.indices[3 * i + 2]];
            let (a, b, c) = (t[k], t[(k + 1) % 3], t[(k + 2) % 3]);
            closed += on.len();
            // a → w1 → … → b, each piece closed at the opposite corner, in
            // the triangle's own winding.
            let chain: Vec<u32> = std::iter::once(a).chain(on).chain(std::iter::once(b)).collect();
            let region = of_region[i];
            for (n, w) in chain.windows(2).enumerate() {
                let piece = [w[0], w[1], c];
                if n == 0 {
                    tri.indices[3 * i..3 * i + 3].copy_from_slice(&piece);
                } else {
                    tri.indices.extend_from_slice(&piece);
                    of_region.push(region);
                }
            }
        }
    }
    closed
}

/// The regions the clipper reads for `shape`, each with its ears: the
/// shape itself, cleaned; or — when the clipper misreads it (a hole
/// touching another, a spike a lattice cell wide) — the shapes the
/// kernel's union of it separates into, each cleaned and gated again.
/// What cannot be read either way is meshed as the clipper had it, and
/// counted. `lost_m2` reads the rings returned here, never the ones passed
/// in, so it is measured against the boundary that was actually
/// triangulated.
fn read(shape: &poly::Shape, stats: &mut Stats) -> Vec<(poly::Shape, Vec<[Pt; 3]>)> {
    let Some(shape) = readable(shape, false) else {
        return Vec::new();
    };
    let want = poly::area(std::slice::from_ref(&shape));
    match ears(&shape, want) {
        Ok(e) => vec![(shape, e)],
        Err(None) => {
            stats.failed += 1;
            Vec::new()
        }
        Err(Some(e)) => {
            let washed: Vec<(poly::Shape, Option<Vec<[Pt; 3]>>)> = poly::union_all(&vec![shape.clone()])
                .iter()
                .filter_map(|s| readable(s, true))
                .map(|s| {
                    let e = ears(&s, poly::area(std::slice::from_ref(&s))).ok();
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

/// `shape` as the clipper may read it: holes under [`HOLE_MIN_M2`] gone,
/// every ring cleaned on its own if `clean` says so; `None` if no area is
/// left.
///
/// [`tagged`] has already cleaned every ring *together* ([`cleaned_together`]),
/// and cleaning one again on its own would undo exactly what that is for.
/// Only the fallback — a shape the kernel's union has just rebuilt — asks.
fn readable(shape: &poly::Shape, clean: bool) -> Option<poly::Shape> {
    let out: poly::Shape = shape
        .iter()
        .enumerate()
        .map(|(i, r)| (i, if clean { cleaned(r) } else { r.clone() }))
        .filter(|(i, r)| r.len() >= 3 && (*i == 0 || poly::ring_area(r).abs() >= HOLE_MIN_M2))
        .map(|(_, r)| r)
        .collect();
    (!out.is_empty() && poly::area(std::slice::from_ref(&out)) > 0.0).then_some(out)
}

/// One terrain triangle, named by the integer band of each of the lattice's
/// three line families it lies in: its column, its row, and its diagonal
/// band (which of the cell's two halves).
type Cell = [i64; 3];

/// A vertex's identity: its position rounded to [`WELD_M`], as the mesh
/// welds it.
type Key = [i64; 2];

fn weld_key(p: Pt) -> Key {
    [(p[0] / WELD_M).round() as i64, (p[1] / WELD_M).round() as i64]
}

/// The lattice's three line families at `p`, in lattice units, exactly as
/// [`split`] cuts by them: the column coordinate, the row coordinate, and
/// their difference, whose integer levels are the cell diagonals.
fn families(p: Pt, grid: &Grid) -> [f64; 3] {
    let u = (p[0] - grid.x0) / grid.dx;
    let v = (p[1] - grid.y0) / grid.dy;
    [u, v, u - v]
}

/// The terrain triangle a piece of [`split`] lies in. Read from the piece's
/// highest value of each family rather than from its centroid, so a piece a
/// hair wide along a lattice line is not handed to the triangle across it.
fn cell_of(piece: &[Pt], grid: &Grid) -> Cell {
    let mut hi = [f64::NEG_INFINITY; 3];
    for &p in piece {
        let f = families(p, grid);
        for i in 0..3 {
            hi[i] = hi[i].max(f[i]);
        }
    }
    hi.map(|h| (h - ON_LINE).ceil() as i64 - 1)
}

/// For each family, the lattice line `p` lies on, if any.
fn on_lines(p: Pt, grid: &Grid) -> [Option<i64>; 3] {
    families(p, grid).map(|f| {
        let k = f.round();
        ((f - k).abs() < ON_LINE).then_some(k as i64)
    })
}

/// A region's triangles, **cut first and triangulated after**: every
/// triangle lies inside one terrain triangle, and none has an edge the ear
/// clipper drew across a lattice line.
///
/// **Why not the ears themselves.** The clipper's ears are cut to the
/// lattice ([`split`]) and each convex piece fanned; but an ear's diagonal
/// crosses the lattice wherever it happens to, and where it passes within
/// microns of a lattice vertex the column, the row and the diagonal cut it
/// within microns of each other — a needle a millimetre wide and a metre
/// long, whose corners are none of them the region's. On the loop box 52 %
/// of the mesh's slivers were those, and a needle is what the lift's curved
/// field tips on end: 20 % of the carriageway's triangles had an altitude
/// under a centimetre, and they were most of the census's fins. Cut first,
/// 2.9 % do, nearly all with a corner of the region's own, and the loop box
/// reads 5 227 slivers against 71 157 and 430 fins against 4 463.
///
/// So the pieces are only the way to the region's part in each terrain
/// triangle: the pieces one terrain triangle holds are merged back into
/// their outline ([`outline`]), every point where an ear's diagonal crossed
/// the lattice is taken off it ([`mesh_cell`]) — it lies inside the region,
/// on a straight run of lattice line, and nothing needs it — and what is
/// left, the region's own corners, its edges' crossings with the lattice and
/// the lattice vertices inside it, is triangulated and flipped towards
/// Delaunay inside that one triangle. A terrain triangle the region covers
/// whole comes back as itself.
///
/// **A cell that cannot be read keeps its pieces**: pieces that do not
/// close into rings (a pinch, an overlap the clipper left), or rings the
/// clipper misreads, are fanned as before (`unmerged`), and every vertex
/// they carry is kept by the cells around them too, so the two sides of a
/// lattice line keep agreeing on its vertices.
fn cut_first(shape: &poly::Shape, ears: &[[Pt; 3]], grid: &Grid, stats: &mut Stats) -> Vec<[Pt; 3]> {
    let mut cells: std::collections::BTreeMap<Cell, Vec<Vec<Pt>>> = Default::default();
    for ear in ears {
        for piece in split(ear.to_vec(), grid) {
            if local_area(&piece) < DEGENERATE_M2 {
                stats.degenerate += 1;
                continue;
            }
            cells.entry(cell_of(&piece, grid)).or_default().push(piece);
        }
    }
    let cells: Vec<Vec<Vec<Pt>>> = cells.into_values().collect();
    let corners: std::collections::HashSet<Key> = shape.iter().flatten().map(|&p| weld_key(p)).collect();
    let mut outlines: Vec<Option<Outline>> = cells.iter().map(|pieces| outline(pieces)).collect();
    let mut pinned: std::collections::HashSet<Key> = Default::default();
    for (pieces, o) in cells.iter().zip(&outlines) {
        if o.is_none() {
            pinned.extend(pieces.iter().flatten().map(|&p| weld_key(p)));
        }
    }
    // A cell that fails once its points are dropped keeps its pieces, so its
    // points are pinned and every cell that carries one is asked again.
    let mut meshed: Vec<Option<Vec<[Pt; 3]>>> = vec![None; cells.len()];
    let mut ask: Vec<usize> = (0..cells.len()).filter(|&i| outlines[i].is_some()).collect();
    while !ask.is_empty() {
        let mut fresh: std::collections::HashSet<Key> = Default::default();
        for &i in &ask {
            let Some(o) = outlines[i].as_ref() else {
                continue;
            };
            meshed[i] = mesh_cell(o, &corners, &pinned, grid);
            if meshed[i].is_none() {
                outlines[i] = None;
                for p in cells[i].iter().flatten() {
                    if pinned.insert(weld_key(*p)) {
                        fresh.insert(weld_key(*p));
                    }
                }
            }
        }
        if fresh.is_empty() {
            break;
        }
        ask = (0..cells.len())
            .filter(|&i| outlines[i].as_ref().is_some_and(|o| o.pts.iter().any(|&p| fresh.contains(&weld_key(p)))))
            .collect();
    }
    let mut out = Vec::new();
    for (pieces, m) in cells.iter().zip(meshed) {
        match m {
            Some(ts) => out.extend(ts),
            None => {
                stats.unmerged += 1;
                for piece in pieces {
                    out.extend(fan(piece, stats));
                }
            }
        }
    }
    out
}

/// The region's part in one terrain triangle, as closed rings over its own
/// vertex table, with the area of the pieces it was merged from.
struct Outline {
    pts: Vec<Pt>,
    rings: Vec<Vec<usize>>,
    area: f64,
}

/// A point within this many metres of an edge, between its ends, is on it:
/// the pieces of one cell meet along an edge one of them may carry a vertex
/// of the other on — a degenerate piece dropped, or a T-junction the
/// clipper's rounding left. Far below the weld, so it joins only what the
/// arithmetic split.
const ON_EDGE_M: f64 = 1e-9;

/// The pieces' outline: their edges, each met by its reverse cancelled, and
/// what is left chained into rings. `None` if what is left does not chain —
/// a vertex two rings pass through, or an edge two pieces both claim.
fn outline(pieces: &[Vec<Pt>]) -> Option<Outline> {
    let mut pts: Vec<Pt> = Vec::new();
    let mut keys: Vec<Key> = Vec::new();
    let mut edges: Vec<(usize, usize)> = Vec::new();
    for piece in pieces {
        let ids: Vec<usize> = piece
            .iter()
            .map(|&p| {
                let k = weld_key(p);
                keys.iter().position(|&q| q == k).unwrap_or_else(|| {
                    pts.push(p);
                    keys.push(k);
                    pts.len() - 1
                })
            })
            .collect();
        for k in 0..ids.len() {
            let (a, b) = (ids[k], ids[(k + 1) % ids.len()]);
            if a != b {
                edges.push((a, b));
            }
        }
    }
    let area = pieces.iter().map(|p| local_area(p)).sum();
    let rings = chain(&edges, pts.len()).or_else(|| chain(&split_at_vertices(&edges, &pts), pts.len()))?;
    Some(Outline { pts, rings, area })
}

/// Every edge split at every vertex lying on it.
fn split_at_vertices(edges: &[(usize, usize)], pts: &[Pt]) -> Vec<(usize, usize)> {
    let mut out = Vec::with_capacity(edges.len());
    for &(a, b) in edges {
        let (p, q) = (pts[a], pts[b]);
        let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
        let len2 = dx * dx + dy * dy;
        let mut on: Vec<(f64, usize)> = (0..pts.len())
            .filter(|&v| v != a && v != b)
            .filter_map(|v| {
                let r = pts[v];
                let s = ((r[0] - p[0]) * dx + (r[1] - p[1]) * dy) / len2;
                (s > 0.0 && s < 1.0 && line::segment_distance(p, q, r) < ON_EDGE_M).then_some((s, v))
            })
            .collect();
        on.sort_by(|x, y| x.0.total_cmp(&y.0));
        let mut from = a;
        for (_, v) in on {
            out.push((from, v));
            from = v;
        }
        out.push((from, b));
    }
    out
}

/// Directed edges, each cancelled against its reverse, chained into rings;
/// `None` unless every vertex left has one edge in and one out.
fn chain(edges: &[(usize, usize)], n: usize) -> Option<Vec<Vec<usize>>> {
    let mut sorted: Vec<(usize, usize, i32)> =
        edges.iter().map(|&(a, b)| if a < b { (a, b, 1) } else { (b, a, -1) }).collect();
    sorted.sort_unstable();
    let mut next: Vec<usize> = vec![usize::MAX; n];
    let mut into: Vec<bool> = vec![false; n];
    for run in sorted.chunk_by(|x, y| x.0 == y.0 && x.1 == y.1) {
        let net: i32 = run.iter().map(|e| e.2).sum();
        let (a, b) = match net {
            0 => continue,
            1 => (run[0].0, run[0].1),
            -1 => (run[0].1, run[0].0),
            _ => return None,
        };
        if next[a] != usize::MAX || into[b] {
            return None;
        }
        next[a] = b;
        into[b] = true;
    }
    let mut seen = vec![false; n];
    let mut rings = Vec::new();
    for start in 0..n {
        if next[start] == usize::MAX || seen[start] {
            continue;
        }
        let mut ring = Vec::new();
        let mut v = start;
        while !seen[v] {
            seen[v] = true;
            ring.push(v);
            v = next[v];
            if v == usize::MAX {
                return None;
            }
        }
        if v != start || ring.len() < 3 {
            return None;
        }
        rings.push(ring);
    }
    (!rings.is_empty()).then_some(rings)
}

/// One cell's outline as triangles, or `None` if it cannot be read.
///
/// A vertex goes if it is not a corner of the region, not pinned, lies on
/// exactly one lattice line — so not a lattice vertex — and both its
/// neighbours lie on that line too: a point where an ear's diagonal crossed
/// the lattice. The same point is taken off the cell across the line, whose
/// outline runs straight through it as well. What is left is ear-clipped,
/// checked against the pieces' area, and flipped towards Delaunay.
fn mesh_cell(
    o: &Outline,
    corners: &std::collections::HashSet<Key>,
    pinned: &std::collections::HashSet<Key>,
    grid: &Grid,
) -> Option<Vec<[Pt; 3]>> {
    let lines: Vec<[Option<i64>; 3]> = o.pts.iter().map(|&p| on_lines(p, grid)).collect();
    let goes = |prev: usize, v: usize, next: usize| {
        let key = weld_key(o.pts[v]);
        if corners.contains(&key) || pinned.contains(&key) {
            return false;
        }
        let on: Vec<usize> = (0..3).filter(|&i| lines[v][i].is_some()).collect();
        on.len() == 1 && {
            let i = on[0];
            lines[prev][i] == lines[v][i] && lines[next][i] == lines[v][i]
        }
    };
    let mut rings: Vec<Vec<Pt>> = Vec::with_capacity(o.rings.len());
    for ring in &o.rings {
        let n = ring.len();
        let kept: Vec<Pt> = (0..n)
            .filter(|&i| !goes(ring[(i + n - 1) % n], ring[i], ring[(i + 1) % n]))
            .map(|i| o.pts[ring[i]])
            .collect();
        if kept.len() < 3 || local_area(&kept).abs() < DEGENERATE_M2 {
            return None;
        }
        rings.push(kept);
    }
    // The outer rings wind counter-clockwise, the holes clockwise; each hole
    // goes with the outer ring around it.
    let (outers, holes): (Vec<Vec<Pt>>, Vec<Vec<Pt>>) = rings.into_iter().partition(|r| local_area(r) > 0.0);
    let mut groups: Vec<poly::Shape> = outers.into_iter().map(|r| vec![r]).collect();
    for hole in holes {
        let around: Vec<usize> = (0..groups.len()).filter(|&g| inside_ring(&groups[g][0], hole[0])).collect();
        match around.as_slice() {
            [g] => groups[*g].push(hole),
            _ if groups.len() == 1 => groups[0].push(hole),
            _ => return None,
        }
    }
    let mut out: Vec<[Pt; 3]> = Vec::new();
    for group in &groups {
        if group.len() == 1 && group[0].len() == 3 {
            out.push([group[0][0], group[0][1], group[0][2]]);
            continue;
        }
        let mut ts = ear_clip(group)?;
        delaunay(&mut ts);
        out.extend(ts);
    }
    let got: f64 = out.iter().map(|t| tri_area(*t)).sum();
    ((got - o.area).abs() <= CELL_TOLERANCE_M2 && out.iter().all(|t| tri_area(*t) >= 0.0)).then_some(out)
}

/// How far, in square metres, one cell's triangles may disagree with the
/// pieces they replace: a point dropped within [`ON_LINE`] of its line moves
/// the outline by a few nanometres over a few metres, and the areas are
/// read relative to a corner ([`local_area`]), so a cell read right agrees
/// to 1e-10 m² and one misread loses a triangle.
const CELL_TOLERANCE_M2: f64 = 1e-8;

/// A ring's signed area, summed as triangles from its first vertex rather
/// than by the shoelace over absolute coordinates, which at a few kilometres
/// from the origin rounds by 1e-9 m² a vertex — far above the cell's own
/// disagreements, and above [`DEGENERATE_M2`].
fn local_area(ring: &[Pt]) -> f64 {
    (1..ring.len().saturating_sub(1)).map(|k| tri_area([ring[0], ring[k], ring[k + 1]])).sum()
}

/// Whether `p` lies inside `ring`, by the crossing count.
fn inside_ring(ring: &[Pt], p: Pt) -> bool {
    let mut inside = false;
    let n = ring.len();
    for i in 0..n {
        let (a, b) = (ring[i], ring[(i + 1) % n]);
        if (a[1] > p[1]) != (b[1] > p[1]) && p[0] < a[0] + (p[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]) {
            inside = !inside;
        }
    }
    inside
}

/// How much, in radians, a flip must raise the smaller of two triangles'
/// least angles before it is made: the rounding of an angle, so a flip and
/// its reverse can never both be taken.
const FLIP_GAIN: f64 = 1e-9;

/// `ts`, counter-clockwise triangles of one polygon, with every interior
/// edge flipped while that raises the least angle of the two triangles
/// beside it (Lawson's flips, which end at the constrained Delaunay
/// triangulation). A flip replaces one diagonal of a convex quad with the
/// other, so it stays inside the quad and the polygon's boundary is never
/// touched. The clipper leaves the thinnest triangle it can; this leaves
/// the fattest.
fn delaunay(ts: &mut [[Pt; 3]]) {
    if ts.len() < 2 {
        return;
    }
    let least = |t: [Pt; 3]| {
        (0..3)
            .map(|k| {
                let (p, q, r) = (t[k], t[(k + 1) % 3], t[(k + 2) % 3]);
                let (u, v) = ([q[0] - p[0], q[1] - p[1]], [r[0] - p[0], r[1] - p[1]]);
                (u[0] * v[1] - u[1] * v[0]).abs().atan2(u[0] * v[0] + u[1] * v[1])
            })
            .fold(f64::INFINITY, f64::min)
    };
    // Enough for any polygon a terrain triangle holds; each flip raises the
    // sorted angle vector, so the loop ends long before.
    for _ in 0..4 * ts.len() * ts.len() {
        let mut flipped = false;
        for i in 0..ts.len() {
            for k in 0..3 {
                let t = ts[i];
                let (a, b, c) = (t[k], t[(k + 1) % 3], t[(k + 2) % 3]);
                let Some((j, m)) =
                    (0..ts.len()).filter(|&j| j != i).find_map(|j| (0..3).find(|&m| ts[j][m] == b && ts[j][(m + 1) % 3] == a).map(|m| (j, m)))
                else {
                    continue;
                };
                let d = ts[j][(m + 2) % 3];
                let (u, w) = ([a, d, c], [d, b, c]);
                if tri_area(u) <= 0.0 || tri_area(w) <= 0.0 {
                    continue;
                }
                if least(u).min(least(w)) > least(t).min(least(ts[j])) + FLIP_GAIN {
                    ts[i] = u;
                    ts[j] = w;
                    flipped = true;
                }
            }
        }
        if !flipped {
            return;
        }
    }
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
const COLLINEAR_M: f64 = 1e-4;

/// `ring` without the vertices that add no shape: a repeated point, a point
/// within `COLLINEAR_M` of the segment between its neighbours, and the
/// tip of a spike — a vertex the ring doubles back at, one neighbour lying
/// on the segment to the other within the same tolerance — which the
/// kernel leaves a lattice cell wide along a boundary and which, meshed,
/// is two boundary edges a micron apart that weld into one. A spike of
/// several vertices collapses from its tip. The ring's area and perimeter
/// move by less than the lattice per vertex removed.
pub fn cleaned(ring: &[Pt]) -> Vec<Pt> {
    let mut out: Vec<Pt> = ring.to_vec();
    loop {
        let n = out.len();
        if n < 3 {
            return out;
        }
        let Some(i) = (0..n).find(|&i| {
            let (a, b, c) = (out[(i + n - 1) % n], out[i], out[(i + 1) % n]);
            a == b
                || line::segment_distance(a, c, b) < COLLINEAR_M
                || line::segment_distance(a, b, c) < COLLINEAR_M
                || line::segment_distance(b, c, a) < COLLINEAR_M
        }) else {
            return out;
        };
        out.remove(i);
    }
}

/// Every ring of `shapes` cleaned as [`cleaned`] cleans one, **with one
/// decision per vertex**: a vertex goes only where it adds no shape to any
/// ring that carries it.
///
/// Rings that share a boundary share its vertices, and a vertex that lies on
/// a straight edge of one ring can be a corner of the next. Where a road's
/// butt end meets the ground and a pavement, the point they meet at is a
/// corner of both and a point on the carriageway's straight end; cleaned
/// ring by ring it would go from the carriageway alone, and the one mesh
/// would have a T-junction there.
///
/// Two neighbours never go in one pass, so a chain of near-collinear
/// vertices drifts off its line by at most one [`COLLINEAR_M`] per vertex
/// removed, as it does one ring at a time.
fn cleaned_together(shapes: &Shapes) -> Shapes {
    type Key = (u64, u64);
    let key = |p: &Pt| (p[0].to_bits(), p[1].to_bits());
    let removable = |a: Pt, b: Pt, c: Pt| {
        a == b
            || line::segment_distance(a, c, b) < COLLINEAR_M
            || line::segment_distance(a, b, c) < COLLINEAR_M
            || line::segment_distance(b, c, a) < COLLINEAR_M
    };
    // **A face of no area has no say.** The slice leaves needles where cut
    // lines nearly coincide — a ring `A, B, C, B`, which bounds nothing — and
    // one meshes to nothing, but its corners would still veto the cleaning of
    // every ring around it: a spike in the carriageway's ring kept for a
    // needle's sake, beside a ground ring that lost it, is a crack. Such a
    // shape is emptied here, so it neither votes nor meshes.
    let mut out: Shapes = shapes
        .iter()
        .map(|s| if poly::area(std::slice::from_ref(s)).abs() < DEGENERATE_M2 { Vec::new() } else { s.clone() })
        .collect();
    // **And a ring that doubles back on itself loses the return trip.** Where
    // a ring reads `A, B, A` it has walked out along a segment and back, which
    // bounds nothing; `B, A` goes, whatever any other ring thinks of `A` and
    // `B`, because both are still in the ring once. The collinear test below
    // cannot do it when both are corners of a neighbour, and a ring tracing a
    // segment three times beside a neighbour that traces it once is a crack.
    for ring in out.iter_mut().flatten() {
        loop {
            let n = ring.len();
            if n < 4 {
                break;
            }
            let Some(i) = (0..n).find(|&i| ring[(i + n - 1) % n] == ring[(i + 1) % n]) else {
                break;
            };
            // Remove the tip and the repeat after it, the later index first.
            let (tip, back) = (i, (i + 1) % n);
            ring.remove(tip.max(back));
            ring.remove(tip.min(back));
        }
    }
    loop {
        let (mut want, mut veto): (std::collections::HashSet<Key>, std::collections::HashSet<Key>) =
            Default::default();
        for ring in out.iter().flatten() {
            let n = ring.len();
            for i in 0..n {
                let (a, b, c) = (ring[(i + n - 1) % n], ring[i], ring[(i + 1) % n]);
                if n >= 3 && removable(a, b, c) {
                    want.insert(key(&b));
                } else {
                    veto.insert(key(&b));
                }
            }
        }
        let drop: std::collections::HashSet<Key> = want.difference(&veto).copied().collect();
        // Not two in a row: a vertex whose predecessor in any ring also goes
        // waits for the next pass.
        let mut wait: std::collections::HashSet<Key> = Default::default();
        for ring in out.iter().flatten() {
            let n = ring.len();
            for i in 0..n {
                let (a, b) = (key(&ring[(i + n - 1) % n]), key(&ring[i]));
                if drop.contains(&a) && drop.contains(&b) && a != b {
                    wait.insert(b);
                }
            }
        }
        let now: std::collections::HashSet<Key> = drop.difference(&wait).copied().collect();
        if now.is_empty() {
            return out;
        }
        for ring in out.iter_mut().flatten() {
            // A ring is only ever cut down to three; past that it has no shape
            // left to keep, and `readable` drops it.
            ring.retain(|p| !now.contains(&key(p)));
        }
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
pub fn ear_clip(shape: &poly::Shape) -> Option<Vec<[Pt; 3]>> {
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
/// `ON_LINE`) belongs to both; a crossing point is computed from the
/// edge's endpoints in lexicographic order, so the two pieces of an edge
/// two polygons share agree bit for bit.
fn halve(piece: &[Pt], f: &dyn Fn(Pt) -> f64, k: f64) -> (Vec<Pt>, Vec<Pt>) {
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
    use crate::world::Material;
    use crate::world::World;
    use crate::pipeline::tests::{built, upto};
    use crate::step::Step;
    use crate::frame::Rect;
    use super::*;
    use crate::lattice::height_at;
    use crate::step::Summary;

    /// A world on `terrain_spec` with the network of `net`, built through
    /// the mesh step.
    pub(crate) fn world(terrain_spec: &str, net: &str) -> (World, Summary) {
        let (w, ran) = built(terrain_spec, net, None, 5.0, &upto(Step::Mesh));
        (w, ran.last())
    }

    /// The unlifted triangles of one material.
    fn of(w: &World, material: Material) -> Tri {
        w.mesh.as_ref().unwrap().view(w.arrangement.as_ref().unwrap(), |f| f.material == material)
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
        let tol = 1e-9;
        let total: f64 = pieces.iter().map(|p| poly::ring_area(p)).sum();
        assert!((total - tri_area([tri[0], tri[1], tri[2]])).abs() < tol, "{total}");
        assert!(pieces.len() > 40, "{}", pieces.len());
        for piece in &pieces {
            assert!(poly::ring_area(piece) > 0.0, "{piece:?}");
            let (u, v): (Vec<f64>, Vec<f64>) = piece.iter().map(|p| grid.to_uv(p[0], p[1])).unzip();
            let (cu, cv) = grid.cell(u.iter().sum::<f64>() / u.len() as f64, v.iter().sum::<f64>() / v.len() as f64);
            let upper = u.iter().zip(&v).map(|(u, v)| (u - cu as f64) - (v - cv as f64)).sum::<f64>() >= 0.0;
            for (u, v) in u.iter().zip(&v) {
                let (fu, fv) = (u - cu as f64, v - cv as f64);
                assert!((-tol..=1.0 + tol).contains(&fu) && (-tol..=1.0 + tol).contains(&fv), "{piece:?}");
                if upper {
                    assert!(fu >= fv - tol, "{piece:?} straddles the diagonal");
                } else {
                    assert!(fu <= fv + tol, "{piece:?} straddles the diagonal");
                }
            }
        }
    }

    /// **No needle the region does not force.** A quad on a 1 m lattice
    /// whose corners and edges keep a few centimetres clear of every
    /// lattice vertex, but either of whose diagonals passes within two
    /// microns of one: at (2, 2) for A–C, at (3, 1) for B–D. Cut as the ear
    /// clipper's ears, the column, the row and the diagonal through that
    /// vertex cut the diagonal within microns of each other, and the pieces
    /// between are needles a micron wide (9.6e-7 m here). Cut first, the
    /// diagonal is never drawn, and nothing is thinner than the quad's own
    /// geometry makes it.
    #[test]
    fn a_diagonal_near_a_lattice_vertex_leaves_no_needle() {
        let grid = Grid::fit(&Rect { x0: 0.0, y0: 0.0, x1: 5.0, y1: 5.0 }, 1.0, usize::MAX);
        let quad = vec![[0.4, 0.7], [3.440003, 0.54], [3.92, 3.560003], [0.8, 3.3]];
        let (tri, stats) = triangulate(&vec![vec![quad.clone()]], &grid, &|_| 0.0);
        let corners = |t: &[u32]| [t[0], t[1], t[2]].map(|i| [tri.positions[i as usize][0], tri.positions[i as usize][1]]);
        let altitude = |c: [Pt; 3]| {
            let long = (0..3).map(|k| (c[(k + 1) % 3][0] - c[k][0]).hypot(c[(k + 1) % 3][1] - c[k][1])).fold(0.0, f64::max);
            2.0 * tri_area(c).abs() / long
        };
        let thinnest = tri.indices.chunks_exact(3).map(|t| altitude(corners(t))).fold(f64::INFINITY, f64::min);
        assert!(thinnest > 1e-3, "a needle {thinnest:.1e} m wide: {stats:?}");
        // Still the quad, and still on the lattice: every triangle in one
        // terrain triangle, and together they are the quad's area.
        let area: f64 = tri.indices.chunks_exact(3).map(|t| tri_area(corners(t))).sum();
        assert!((area - poly::ring_area(&quad)).abs() < 1e-9, "{area}");
        for t in tri.indices.chunks_exact(3) {
            let c = corners(t);
            let cell = cell_of(&c, &grid);
            for p in c {
                let f = families(p, &grid);
                for i in 0..3 {
                    assert!(f[i] >= cell[i] as f64 - 1e-9 && f[i] <= cell[i] as f64 + 1.0 + 1e-9, "{c:?} leaves its terrain triangle");
                }
            }
        }
        assert_eq!(stats.unmerged, 0, "{stats:?}");
    }

    #[test]
    fn a_straight_on_flat_ground_meshes_to_its_area() {
        let (w, s) = world("flat", "net:straight?len=200");
        let c = of(&w, Material::Carriageway);
        let want = poly::area(&w.legs.as_ref().expect("the legs step ran").surface.carriageway);
        assert!((area_of(&c) - want).abs() / want < 1e-9, "{} vs {want}", area_of(&c));
        assert!(c.positions.iter().all(|p| p[2] == 400.0));
        assert!(of(&w, Material::Pavement).indices.is_empty());
        assert_eq!(s.get("failed"), Some("0"), "{s}");
        // Over the whole rect, the ground with its holes included: what is
        // lost is the ear sums' rounding.
        assert!(s.num("lost_m2") < 1e-5 && s.num("off_ground") < 1e-9, "{s}");
        assert_eq!(s.num("crack"), 0.0, "{s}");
        // Every triangle counter-clockwise, seen from above.
        for t in c.indices.chunks_exact(3) {
            let p = |i: u32| [c.positions[i as usize][0], c.positions[i as usize][1]];
            assert!(tri_area([p(t[0]), p(t[1]), p(t[2])]) > 0.0);
        }
    }

    #[test]
    fn a_mesh_on_a_hill_lies_on_the_ground() {
        let (w, s) = world("hill?amp=60&radius=400", "net:cross?len=400");
        let t = w.terrain.as_ref().expect("the terrain step ran");
        let m = w.mesh.as_ref().unwrap();
        assert!(s.num("off_ground") < 1e-9 && s.num("lost_m2") < 1e-5, "{s}");
        assert_eq!(s.num("crack"), 0.0, "{s}");
        // Not only the centroid: points across every triangle, of every
        // material, the ground's included.
        for tri in m.tri.indices.chunks_exact(3) {
            let [a, b, c] = [tri[0], tri[1], tri[2]].map(|i| m.tri.positions[i as usize]);
            for (wa, wb) in [(0.2, 0.3), (0.6, 0.1), (0.1, 0.8), (0.05, 0.05)] {
                let wc = 1.0 - wa - wb;
                let x = wa * a[0] + wb * b[0] + wc * c[0];
                let y = wa * a[1] + wb * b[1] + wc * c[1];
                let z = wa * a[2] + wb * b[2] + wc * c[2];
                assert!((z - height_at(t, x, y)).abs() < 1e-9, "({x}, {y}): {z} vs {}", height_at(t, x, y));
            }
        }
        assert!(of(&w, Material::Carriageway).positions.iter().any(|p| p[2] > 401.0), "the cross climbs the hill");
    }

    #[test]
    fn the_pavement_is_meshed_beside_the_road() {
        let (w, s) = world("flat", "net:sidewalk?d=6");
        let p = of(&w, Material::Pavement);
        let want = poly::area(&w.room.as_ref().expect("the room step ran").surface.walk);
        assert!(want > 0.0);
        assert!((area_of(&p) - want).abs() / want < 1e-9, "{} vs {want}", area_of(&p));
        assert_eq!(s.num("crack"), 0.0, "{s}");
    }

    /// **One mesh: the materials share their vertices by index.** A kerb
    /// vertex is one entry of one array, reached from a carriageway triangle
    /// and from a pavement triangle alike, and an outline vertex likewise
    /// from the paving and the ground — so the lift and the earthwork find
    /// every copy of it from that index and never by where it lies.
    #[test]
    fn one_mesh_shares_its_vertices_between_materials() {
        let (w, _) = world("flat", "net:sidewalk?d=6");
        let (m, a) = (w.mesh.as_ref().unwrap(), w.arrangement.as_ref().unwrap());
        assert_eq!(m.of_face.len(), m.tri.indices.len() / 3, "one tag per triangle");
        let mut at: HashMap<u32, std::collections::BTreeSet<&str>> = HashMap::new();
        for (t, &f) in m.tri.indices.chunks_exact(3).zip(&m.of_face) {
            for &v in t {
                at.entry(v).or_default().insert(a.face(f).material.name());
            }
        }
        let both = |x: &str, y: &str| at.values().filter(|s| s.contains(x) && s.contains(y)).count();
        assert!(both("carriageway", "pavement") > 0, "the kerb is not shared");
        assert!(both("pavement", "ground") > 0, "the outline is not shared");
    }
}
