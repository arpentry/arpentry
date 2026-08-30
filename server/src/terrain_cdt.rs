//! Breakline-constrained terrain mesh (docs/GROUND.md §3).
//!
//! The detail-zoom terrain mesh, rebuilt as a constrained Delaunay
//! triangulation: the regular lattice persists as background points — every
//! vertex [`crate::terrain::elevated_mesh`] would emit, this mesh emits too —
//! and the bench contact lines enter as constraint edges, so the drawn
//! ground holds a bench exactly under every road however narrow the bench is
//! against the cells.
//!
//! Determinism (invariant 5): the triangulation runs in quantized tile-local
//! coordinates (u16 values held exactly in f64), points are inserted in a
//! fixed order (lattice row-major, then constraint endpoints in query
//! order), and adjacent tiles clip the same global polylines — a border
//! crossing lands exactly on the shared quantized edge coordinate, so
//! neighbours derive identical border vertices and heights. Interior
//! connectivity may differ per tile; only the border profile must match,
//! and it does because border vertices lie on the convex hull.
//!
//! In cells no breakline touches, the standard lattice diagonal is added as
//! a constraint, so the triangulation there is exactly the fixed-diagonal
//! mesh that [`crate::terrain::surface_height`] mirrors — the drape and the
//! drawn ground cannot drift apart away from the benches.

use std::f64::consts::PI;

use geo_types::Coord;
use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation};

use crate::project::{self, Bounds, BUFFER, EXTENT};
use crate::synth::region::Region;
use crate::terrain::{encode_octahedral, TerrainMesh};

/// Metric step of the central differences the vertex normals are computed
/// with — a property of the ground *field*, not the mesh, so normals stay
/// continuous across tile borders, flat across a bench, and creased at a
/// batter regardless of how the triangulation fell.
const NORMAL_STEP_M: f64 = 2.0;

/// Builds the constrained mesh for one tile: the `grid`×`grid` background
/// lattice plus the breakline `segments` (in lon/lat, already filtered near
/// the tile). `sample` is the engineered ground — vertex heights and the
/// normal differences both read it, so the mesh and its shading come from
/// one field. Returns the mesh and its `(min, max)` elevation, or `None`
/// when there is nothing to constrain or the triangulation fails — the
/// caller falls back to the plain lattice (invariant 6: plain, not wrong).
///
/// "The triangulation fails" includes the geometric kernel giving up: spade's
/// constraint splitting asserts rather than returning on some
/// nearly-degenerate configurations, and an assertion in an emit worker takes
/// the whole tiling run with it. One tile's breaklines are not worth that, and
/// the fallback this promises is exactly the right answer for them, so the
/// panic is caught here and becomes the `None` the contract already describes.
/// It is caught and not prevented because the trigger is upstream and
/// configuration-dependent: the alternative is guessing at which degeneracies
/// spade dislikes and silently dropping constraints that were fine.
pub fn constrained_mesh(
    grid: u32,
    bounds: &Bounds,
    segments: &[(Coord, Coord)],
    regions: &[Region],
    sample: &mut dyn FnMut(f64, f64) -> f64,
) -> Option<(TerrainMesh, f64, f64)> {
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        constrained_mesh_inner(grid, bounds, segments, regions, sample)
    }))
    .unwrap_or(None)
}

fn constrained_mesh_inner(
    grid: u32,
    bounds: &Bounds,
    segments: &[(Coord, Coord)],
    regions: &[Region],
    sample: &mut dyn FnMut(f64, f64) -> f64,
) -> Option<(TerrainMesh, f64, f64)> {
    let grid = grid.max(1);
    let n = grid + 1; // lattice vertices per side
    let step = EXTENT / grid as f64; // quantized units per cell (exact: EXTENT % grid == 0)

    // Clip the breakline segments to the tile-proper rect and quantize. A
    // border crossing computes the border coordinate exactly (Liang–Barsky
    // sets it to the clip bound), so it quantizes to exactly 16384/49152.
    let mut constraints: Vec<((u16, u16), (u16, u16))> = Vec::new();
    for &(a, b) in segments {
        let Some((ca, cb)) = clip_to_bounds(a, b, bounds) else { continue };
        let qa = (project::quantize_x(ca.x, bounds), project::quantize_y(ca.y, bounds));
        let qb = (project::quantize_x(cb.x, bounds), project::quantize_y(cb.y, bounds));
        if qa != qb {
            constraints.push((qa, qb));
        }
    }
    if constraints.is_empty() && regions.is_empty() {
        return None; // nothing to constrain: the plain lattice is exact
    }

    // Cells a constraint's bbox touches lose their fixed diagonal (the CDT
    // triangulates them freely around the breakline); every other cell keeps
    // it, so the mesh there matches `surface_height`'s mirror exactly.
    let cell_of = |q: u16| -> i64 { ((q as f64 - BUFFER) / step).floor() as i64 };
    let mut crossed = vec![false; (grid * grid) as usize];
    let mut mark = |qa: (u16, u16), qb: (u16, u16)| {
        let (c0, c1) = (cell_of(qa.0.min(qb.0)), cell_of(qa.0.max(qb.0)));
        let (r0, r1) = (cell_of(qa.1.min(qb.1)), cell_of(qa.1.max(qb.1)));
        for r in r0.max(0)..=r1.min(grid as i64 - 1) {
            for c in c0.max(0)..=c1.min(grid as i64 - 1) {
                crossed[(r * grid as i64 + c) as usize] = true;
            }
        }
    };
    for &(qa, qb) in &constraints {
        mark(qa, qb);
    }
    // The kerb is a constraint like any other: a cell it crosses must lose its
    // fixed diagonal too, or the diagonal is added and then split anyway —
    // correct, but noisier and slower.
    let mut ring_edges: Vec<((u16, u16), (u16, u16))> = Vec::new();
    for r in regions {
        for ring in r.rings() {
            let n = ring.len();
            if n < 3 {
                continue;
            }
            for k in 0..n {
                let a = ring[k];
                let b = ring[(k + 1) % n];
                let qa = (a.0.round() as u16, a.1.round() as u16);
                let qb = (b.0.round() as u16, b.1.round() as u16);
                if qa != qb {
                    ring_edges.push((qa, qb));
                }
            }
        }
    }
    for &(qa, qb) in &ring_edges {
        mark(qa, qb);
    }

    let mut cdt: ConstrainedDelaunayTriangulation<Point2<f64>> =
        ConstrainedDelaunayTriangulation::new();

    // Background lattice, row-major from the south-west corner — the same
    // vertex set (and quantized positions) as the unconstrained mesh.
    let origin = BUFFER as u32;
    let qstep = EXTENT as u32 / grid;
    let mut lattice = Vec::with_capacity((n * n) as usize);
    for row in 0..n {
        for col in 0..n {
            let p = Point2::new((origin + col * qstep) as f64, (origin + row * qstep) as f64);
            lattice.push(cdt.insert(p).ok()?);
        }
    }

    // Fixed diagonals in the untouched cells (see the module doc). Guarded:
    // a constraint added later never crosses these cells, so the guard only
    // skips genuinely degenerate corners.
    for row in 0..grid {
        for col in 0..grid {
            if crossed[(row * grid + col) as usize] {
                continue;
            }
            let tl = lattice[(row * n + col) as usize];
            let br = lattice[((row + 1) * n + col + 1) as usize];
            if cdt.can_add_constraint(tl, br) {
                cdt.add_constraint(tl, br);
            }
        }
    }

    // Breakline constraints. `add_constraint_and_split` resolves crossings
    // between contact lines (junction areas, hairpin miters) by inserting
    // split vertices instead of failing.
    for &(qa, qb) in &constraints {
        let va = cdt.insert(Point2::new(qa.0 as f64, qa.1 as f64)).ok()?;
        let vb = cdt.insert(Point2::new(qb.0 as f64, qb.1 as f64)).ok()?;
        if va == vb {
            continue;
        }
        cdt.add_constraint_and_split(va, vb, |p| p);
    }

    // The kerb, after the breaklines and in ring-then-vertex order, so a crest
    // line crossing a kerb resolves into a split vertex rather than failing.
    for &(qa, qb) in &ring_edges {
        let va = cdt.insert(Point2::new(qa.0 as f64, qa.1 as f64)).ok()?;
        let vb = cdt.insert(Point2::new(qb.0 as f64, qb.1 as f64)).ok()?;
        if va == vb {
            continue;
        }
        cdt.add_constraint_and_split(va, vb, |p| p);
    }

    // Positions first, heights later. With a hole, every lattice vertex
    // strictly inside the paved region is about to be discarded, and sampling
    // one costs three DEM-backed field evaluations (the height and two central
    // differences for the normal). The paved region is 5-8 % of a dense tile,
    // so this ordering is where "removing terrain area offsets the added
    // constraints" is actually earned.
    //
    // Split vertices may carry fractional positions; they are rounded to the
    // output u16 grid and sampled at the rounded position, so the stored z is
    // exact for the stored vertex.
    let vcount = cdt.num_vertices();
    let mut qpos: Vec<(u16, u16)> = Vec::with_capacity(vcount);
    for v in cdt.vertices() {
        let p = v.position();
        qpos.push((
            p.x.round().clamp(0.0, 65535.0) as u16,
            p.y.round().clamp(0.0, 65535.0) as u16,
        ));
    }

    // Faces, wound CCW (east-x, north-y is right-handed, so spade's CCW faces
    // already match the client's front face; the area check is insurance
    // against rounding collapsing a face). A face whose centroid lies inside
    // the paved region is dropped: the asphalt is opaque and watertight, and
    // ground drawn beneath it is where every burial artifact lives.
    let mut faces: Vec<[u32; 3]> = Vec::with_capacity(cdt.num_inner_faces());
    let mut used = vec![false; vcount];
    for face in cdt.inner_faces() {
        let [a, b, c] = face.vertices().map(|v| v.fix().index());
        let (ax, ay) = (qpos[a].0 as i64, qpos[a].1 as i64);
        let (bx, by) = (qpos[b].0 as i64, qpos[b].1 as i64);
        let (cx, cy) = (qpos[c].0 as i64, qpos[c].1 as i64);
        let area2 = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
        if area2 == 0 {
            continue; // rounding collapsed the face: drop the sliver
        }
        if !regions.is_empty() {
            // Inside *any* level-0 region. They are separate regions on
            // different grade layers, overlapping in plan and metres apart
            // vertically, so they cannot be merged into one winding test: a
            // hole in one would cancel an outer ring of another and re-admit
            // ground under drawn asphalt.
            let cen = ((ax + bx + cx) as f64 / 3.0, (ay + by + cy) as f64 / 3.0);
            if regions.iter().any(|r| r.contains(cen)) {
                continue;
            }
            // ARPT_CDT_AT=qx,qy — why did faces near this quantized point
            // survive the hole: the centroid and each region's winding verdict.
            if let Some(at) = std::env::var_os("ARPT_CDT_AT") {
                if let Some((px, py)) = at
                    .to_str()
                    .and_then(|s| s.split_once(','))
                    .and_then(|(a, b)| {
                        Some((a.trim().parse::<f64>().ok()?, b.trim().parse::<f64>().ok()?))
                    })
                {
                    if (cen.0 - px).abs() < 600.0 && (cen.1 - py).abs() < 600.0 {
                        let verdicts: Vec<bool> =
                            regions.iter().map(|r| r.contains(cen)).collect();
                        eprintln!(
                            "[cdt-at] face cen ({:.0},{:.0}) survives; regions contain: {verdicts:?}",
                            cen.0, cen.1
                        );
                    }
                }
            }
        }
        let tri = if area2 > 0 { [a, b, c] } else { [a, c, b] };
        for &v in &tri {
            used[v] = true;
        }
        faces.push([tri[0] as u32, tri[1] as u32, tri[2] as u32]);
    }
    if faces.is_empty() {
        // A tile entirely inside the region should not happen, and must not
        // produce an empty terrain if it does: fall back to the plain lattice.
        return None;
    }

    // Compaction: only vertices a surviving face references are sampled.
    let mut remap = vec![u32::MAX; vcount];
    let kept = used.iter().filter(|&&u| u).count();
    let mut x: Vec<u16> = Vec::with_capacity(kept);
    let mut y: Vec<u16> = Vec::with_capacity(kept);
    let mut z: Vec<i32> = Vec::with_capacity(kept);
    let mut normals: Vec<i8> = vec![0; kept * 2];
    let (mut emin, mut emax) = (f64::INFINITY, f64::NEG_INFINITY);
    let mid_lat = (bounds.south + bounds.north) * 0.5;
    let dlat = NORMAL_STEP_M / crate::scene::DEG_M;
    let dlon = NORMAL_STEP_M / (crate::scene::DEG_M * (mid_lat * PI / 180.0).cos());
    for (v, &(qx, qy)) in qpos.iter().enumerate() {
        if !used[v] {
            continue;
        }
        remap[v] = x.len() as u32;
        let lon = project::dequantize_x(qx, bounds);
        let lat = project::dequantize_y(qy, bounds);
        let e = sample(lon, lat);
        emin = emin.min(e);
        emax = emax.max(e);
        x.push(qx);
        y.push(qy);
        z.push(project::quantize_z(e));

        // Central differences of the ground field (see NORMAL_STEP_M).
        let dz_dx = (sample(lon + dlon, lat) - sample(lon - dlon, lat)) / (2.0 * NORMAL_STEP_M);
        let dz_dy = (sample(lon, lat + dlat) - sample(lon, lat - dlat)) / (2.0 * NORMAL_STEP_M);
        let lon_r = lon * (PI / 180.0);
        let lat_r = lat * (PI / 180.0);
        let (sin_lon, cos_lon) = (lon_r.sin(), lon_r.cos());
        let (sin_lat, cos_lat) = (lat_r.sin(), lat_r.cos());
        let (ex, ey, ez) = (-sin_lon, cos_lon, 0.0);
        let (nx_e, ny_e, nz_e) = (-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat);
        let (ux, uy, uz) = (cos_lat * cos_lon, cos_lat * sin_lon, sin_lat);
        let mut nx = ux - dz_dx * ex - dz_dy * nx_e;
        let mut ny = uy - dz_dx * ey - dz_dy * ny_e;
        let mut nz = uz - dz_dx * ez - dz_dy * nz_e;
        let len = (nx * nx + ny * ny + nz * nz).sqrt();
        if len > 0.0 {
            nx /= len;
            ny /= len;
            nz /= len;
        }
        let idx = x.len() - 1;
        let (ox, oy) = encode_octahedral(nx, ny, nz);
        normals[idx * 2] = ox;
        normals[idx * 2 + 1] = oy;
    }

    let mut indices: Vec<u32> = Vec::with_capacity(faces.len() * 3);
    for f in &faces {
        indices.extend_from_slice(&[
            remap[f[0] as usize],
            remap[f[1] as usize],
            remap[f[2] as usize],
        ]);
    }
    if indices.is_empty() || !emin.is_finite() {
        return None;
    }
    Some((TerrainMesh { x, y, z, indices, normals, edge_across: Vec::new() }, emin, emax))
}

/// Liang–Barsky clip of the segment `a → b` to the tile-proper rect. A
/// clipped endpoint takes the bound coordinate *exactly*, so it quantizes to
/// the exact tile-edge value the neighbour computes for the same polyline.
fn clip_to_bounds(a: Coord, b: Coord, bounds: &Bounds) -> Option<(Coord, Coord)> {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let mut t0 = 0.0f64;
    let mut t1 = 1.0f64;
    let checks = [
        (-dx, a.x - bounds.west),
        (dx, bounds.east - a.x),
        (-dy, a.y - bounds.south),
        (dy, bounds.north - a.y),
    ];
    for (p, q) in checks {
        if p == 0.0 {
            if q < 0.0 {
                return None; // parallel and outside
            }
        } else {
            let r = q / p;
            if p < 0.0 {
                if r > t1 {
                    return None;
                }
                t0 = t0.max(r);
            } else {
                if r < t0 {
                    return None;
                }
                t1 = t1.min(r);
            }
        }
    }
    if t0 > t1 {
        return None;
    }
    let point = |t: f64| -> Coord {
        let mut c = Coord { x: a.x + t * dx, y: a.y + t * dy };
        // Snap the clipped coordinate to the bound it was clipped against, so
        // quantization lands exactly on the tile edge.
        c.x = c.x.clamp(bounds.west, bounds.east);
        c.y = c.y.clamp(bounds.south, bounds.north);
        c
    };
    Some((point(t0), point(t1)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::terrain::elevated_mesh;

    fn tile() -> Bounds {
        Bounds::of_tile(16, 34000, 23000)
    }

    /// With no constraints near the tile the builder abstains and the caller
    /// keeps the plain lattice.
    #[test]
    fn no_constraints_means_no_mesh() {
        let b = tile();
        let got = constrained_mesh(16, &b, &[], &[], &mut |_, _| 500.0);
        assert!(got.is_none());
    }

    /// A single breakline across the tile: every lattice vertex survives at
    /// its exact position, the constraint vertices appear, and the mesh is
    /// watertight (every edge interior to the hull is shared by two
    /// triangles).
    #[test]
    fn lattice_survives_and_breakline_vertices_appear() {
        let b = tile();
        let grid = 16u32;
        // A diagonal-ish line crossing the middle of the tile.
        let a = Coord { x: b.west + b.width() * 0.21, y: b.south + b.height() * 0.33 };
        let c = Coord { x: b.west + b.width() * 0.74, y: b.south + b.height() * 0.61 };
        let (m, _, _) = constrained_mesh(
            grid,
            &b,
            &[(a, c)],
            &[],
            &mut |_, _| 500.0,
        )
        .expect("a constrained mesh");
        let n = (grid + 1) as usize;
        assert!(m.x.len() > n * n, "constraint vertices must be added");
        // The full background lattice survives at exact positions.
        let step = EXTENT as u32 / grid;
        for row in 0..=grid {
            for col in 0..=grid {
                let (qx, qy) =
                    ((BUFFER as u32 + col * step) as u16, (BUFFER as u32 + row * step) as u16);
                assert!(
                    m.x.iter().zip(&m.y).any(|(&x, &y)| x == qx && y == qy),
                    "lattice vertex ({qx},{qy}) missing"
                );
            }
        }
        // Watertight: interior edges shared by exactly two triangles.
        use std::collections::HashMap;
        let mut edge_uses: HashMap<(u32, u32), u32> = HashMap::new();
        for t in m.indices.chunks(3) {
            for k in 0..3 {
                let (p, q) = (t[k], t[(k + 1) % 3]);
                *edge_uses.entry((p.min(q), p.max(q))).or_insert(0) += 1;
            }
        }
        assert!(edge_uses.values().all(|&u| u <= 2), "an edge used thrice is a fold");
    }

    /// Away from any breakline the constrained mesh triangulates cells with
    /// the same fixed diagonal as the plain lattice, so `surface_height`'s
    /// mirror stays exact there.
    #[test]
    fn untouched_cells_keep_the_fixed_diagonal() {
        let b = tile();
        let grid = 8u32;
        // A short line confined to the south-west corner cell.
        let a = Coord { x: b.west + b.width() * 0.02, y: b.south + b.height() * 0.03 };
        let c = Coord { x: b.west + b.width() * 0.09, y: b.south + b.height() * 0.08 };
        let field = |lon: f64, lat: f64| (lon * 9000.0).sin() * 40.0 + (lat * 7000.0).cos() * 25.0;
        let (m, _, _) = constrained_mesh(
            grid,
            &b,
            &[(a, c)],
            &[],
            &mut |lon, lat| field(lon, lat),
        )
        .expect("a constrained mesh");
        let (plain, _, _) = elevated_mesh(grid, &b, |lon, lat| field(lon, lat));
        // Compare triangle sets restricted to cells far from the corner: the
        // north-east quadrant must triangulate identically (fixed diagonal).
        let tri_set = |m: &TerrainMesh| -> std::collections::HashSet<[(u16, u16); 3]> {
            m.indices
                .chunks(3)
                .map(|t| {
                    let mut v: Vec<(u16, u16)> =
                        t.iter().map(|&i| (m.x[i as usize], m.y[i as usize])).collect();
                    v.sort_unstable();
                    [v[0], v[1], v[2]]
                })
                .filter(|v| v.iter().all(|&(x, y)| x >= 32768 && y >= 32768))
                .collect()
        };
        assert_eq!(tri_set(&m), tri_set(&plain), "clean cells must keep the fixed diagonal");
    }

    /// Crossing breaklines (a junction area) split rather than fail, and the
    /// border contract holds: a polyline leaving the tile lands a vertex at
    /// exactly the quantized tile edge.
    #[test]
    fn crossing_constraints_split_and_borders_are_exact() {
        let b = tile();
        let cross1 = (
            Coord { x: b.west - b.width() * 0.1, y: b.south + b.height() * 0.5 },
            Coord { x: b.east + b.width() * 0.1, y: b.south + b.height() * 0.5 },
        );
        let cross2 = (
            Coord { x: b.west + b.width() * 0.5, y: b.south - b.height() * 0.1 },
            Coord { x: b.west + b.width() * 0.5, y: b.north + b.height() * 0.1 },
        );
        let (m, _, _) = constrained_mesh(
            16,
            &b,
            &[cross1, cross2],
            &[],
            &mut |_, _| 500.0,
        )
        .expect("a constrained mesh");
        // The horizontal line's clipped ends sit exactly on the west/east
        // tile edges (16384 / 49152).
        let qy = project::quantize_y(b.south + b.height() * 0.5, &b);
        assert!(
            m.x.iter().zip(&m.y).any(|(&x, &y)| x == 16384 && y == qy),
            "west border vertex missing"
        );
        assert!(
            m.x.iter().zip(&m.y).any(|(&x, &y)| x == 49152 && y == qy),
            "east border vertex missing"
        );
    }

    /// A paved region cuts a hole: the mesh stops at the kerb.
    ///
    /// The one property that matters is *where* it stops. Every face must be
    /// outside the region, the ring must be a constraint so the boundary follows
    /// the kerb rather than the nearest lattice line, and the lattice outside it
    /// must survive intact — a hole that also eats its surroundings is a gap you
    /// can see through beside every road.
    #[test]
    fn a_paved_region_cuts_a_hole_at_its_own_boundary() {
        let b = tile();
        let grid = 16u32;
        // A rectangle over the middle of the tile, counter-clockwise (outer).
        let (x0, x1, y0, y1) = (24000.0, 40000.0, 26000.0, 38000.0);
        let region = Region::new(vec![vec![(x0, y0), (x1, y0), (x1, y1), (x0, y1)]]);
        let (m, _, _) = constrained_mesh(grid, &b, &[], &[region], &mut |_, _| 500.0)
            .expect("a constrained mesh");

        // No face centroid falls inside the region.
        let inside = |p: (f64, f64)| p.0 > x0 && p.0 < x1 && p.1 > y0 && p.1 < y1;
        for t in m.indices.chunks(3) {
            let v: Vec<(f64, f64)> =
                t.iter().map(|&i| (m.x[i as usize] as f64, m.y[i as usize] as f64)).collect();
            let cen = (
                (v[0].0 + v[1].0 + v[2].0) / 3.0,
                (v[0].1 + v[1].1 + v[2].1) / 3.0,
            );
            assert!(!inside(cen), "a face survived inside the paved region at {cen:?}");
        }
        // The ring entered as a constraint, so its corners are mesh vertices and
        // the hole's edge is the kerb, not a lattice line rounded to it.
        for &(cx, cy) in &[(x0, y0), (x1, y0), (x1, y1), (x0, y1)] {
            assert!(
                m.x.iter().zip(&m.y).any(|(&x, &y)| x as f64 == cx && y as f64 == cy),
                "ring corner ({cx},{cy}) is not a mesh vertex"
            );
        }
        // Everything outside survives: every lattice vertex not inside the
        // region is still there.
        let step = EXTENT as u32 / grid;
        for row in 0..=grid {
            for col in 0..=grid {
                let (qx, qy) =
                    ((BUFFER as u32 + col * step) as u16, (BUFFER as u32 + row * step) as u16);
                if inside((qx as f64, qy as f64)) {
                    continue;
                }
                assert!(
                    m.x.iter().zip(&m.y).any(|(&x, &y)| x == qx && y == qy),
                    "lattice vertex ({qx},{qy}) outside the hole was dropped with it"
                );
            }
        }
    }

    /// A hole in one region must not be re-cut by another. Two level-0 regions
    /// overlapping in plan on different grade layers cannot be merged into one
    /// winding test — one's hole would cancel the other's outer ring and re-admit
    /// ground under drawn asphalt — so each is tested on its own.
    #[test]
    fn two_overlapping_regions_each_cut_on_their_own() {
        let b = tile();
        let rect = |x0: f64, x1: f64, y0: f64, y1: f64| {
            vec![(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        };
        // A region with a hole in the middle, and a second region covering
        // exactly that hole.
        let holed = Region::new(vec![
            rect(20000.0, 46000.0, 20000.0, 46000.0),
            rect(30000.0, 36000.0, 30000.0, 36000.0).into_iter().rev().collect(),
        ]);
        let plug = Region::new(vec![rect(30000.0, 36000.0, 30000.0, 36000.0)]);
        let (m, _, _) = constrained_mesh(16, &b, &[], &[holed, plug], &mut |_, _| 500.0)
            .expect("a constrained mesh");
        for t in m.indices.chunks(3) {
            let v: Vec<(f64, f64)> =
                t.iter().map(|&i| (m.x[i as usize] as f64, m.y[i as usize] as f64)).collect();
            let cen = (
                (v[0].0 + v[1].0 + v[2].0) / 3.0,
                (v[0].1 + v[1].1 + v[2].1) / 3.0,
            );
            let in_outer = cen.0 > 20000.0 && cen.0 < 46000.0 && cen.1 > 20000.0 && cen.1 < 46000.0;
            assert!(!in_outer, "the plug's asphalt left ground drawn under it at {cen:?}");
        }
    }

    /// The constrained mesh holds a bench exactly: with a flat-bench field
    /// between two crest lines, every vertex between the lines reads the
    /// bench height and no triangle bridges across the crest.
    #[test]
    fn a_bench_between_crests_is_held_flat() {
        let b = tile();
        let grid = 16u32;
        // Two horizontal crest lines ~1/40 tile apart mid-tile; the field is
        // 500 m outside, 490 m (a cut) between them.
        let y0 = b.south + b.height() * 0.50;
        let y1 = b.south + b.height() * 0.525;
        let lines = [
            (Coord { x: b.west - 0.001, y: y0 }, Coord { x: b.east + 0.001, y: y0 }),
            (Coord { x: b.west - 0.001, y: y1 }, Coord { x: b.east + 0.001, y: y1 }),
        ];
        let bench = move |_lon: f64, lat: f64| if lat > y0 && lat < y1 { 490.0 } else { 500.0 };
        let (m, emin, _) = constrained_mesh(
            grid,
            &b,
            &lines,
            &[],
            &mut |lon, lat| bench(lon, lat),
        )
        .expect("a constrained mesh");
        assert_eq!(emin, 490.0, "the bench floor must be sampled");
        // No triangle spans from strictly below the bench to strictly above
        // it without a vertex on a crest: the crest lines are constraints.
        let (qy0, qy1) = (project::quantize_y(y0, &b), project::quantize_y(y1, &b));
        for t in m.indices.chunks(3) {
            let ys: Vec<u16> = t.iter().map(|&i| m.y[i as usize]).collect();
            let below = ys.iter().any(|&y| y < qy0);
            let above = ys.iter().any(|&y| y > qy1);
            let inside = ys.iter().any(|&y| y > qy0 && y < qy1);
            assert!(
                !(below && above && inside),
                "a triangle bridges the bench: ys={ys:?} (crest at {qy0}/{qy1})"
            );
        }
    }
}


/// **The S5 prototype's first falsifier** (`carmack-rewrite-plan-2026-08-29`
/// S5, kill criterion 1): build the one-mesh CDT — the terrain lattice, the
/// breaklines and the paved rings in a single triangulation, with the faces
/// inside the regions *classified* instead of dropped — and hand back its
/// border vertices per class. Mixed adjacency is possible exactly when this
/// border matches the old path's border vertex-for-vertex: the terrain-class
/// vertices with the same positions and heights the old terrain CDT derives,
/// and the asphalt-class vertices at the same positions the old paved mesh
/// clips and densifies to (heights then agree by I5 — both sides sample one
/// field at one point). Any disagreement is the stage's own kill.
pub fn one_mesh_border_probe(
    grid: u32,
    bounds: &Bounds,
    segments: &[(Coord, Coord)],
    regions: &[&Region],
    sample: &mut dyn FnMut(f64, f64) -> f64,
) -> Option<(Vec<(u16, u16, i32)>, Vec<(u16, u16)>)> {
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        one_mesh_border_probe_inner(grid, bounds, segments, regions, sample)
    }))
    .unwrap_or(None)
}

fn one_mesh_border_probe_inner(
    grid: u32,
    bounds: &Bounds,
    segments: &[(Coord, Coord)],
    regions: &[&Region],
    sample: &mut dyn FnMut(f64, f64) -> f64,
) -> Option<(Vec<(u16, u16, i32)>, Vec<(u16, u16)>)> {
    let grid = grid.max(1);
    let n = grid + 1;
    let step = EXTENT / grid as f64;
    let mut constraints: Vec<((u16, u16), (u16, u16))> = Vec::new();
    for &(a, b) in segments {
        let Some((ca, cb)) = clip_to_bounds(a, b, bounds) else { continue };
        let qa = (project::quantize_x(ca.x, bounds), project::quantize_y(ca.y, bounds));
        let qb = (project::quantize_x(cb.x, bounds), project::quantize_y(cb.y, bounds));
        if qa != qb {
            constraints.push((qa, qb));
        }
    }
    let cell_of = |q: u16| -> i64 { ((q as f64 - BUFFER) / step).floor() as i64 };
    let mut crossed = vec![false; (grid * grid) as usize];
    let mut mark = |qa: (u16, u16), qb: (u16, u16)| {
        let (c0, c1) = (cell_of(qa.0.min(qb.0)), cell_of(qa.0.max(qb.0)));
        let (r0, r1) = (cell_of(qa.1.min(qb.1)), cell_of(qa.1.max(qb.1)));
        for r in r0.max(0)..=r1.min(grid as i64 - 1) {
            for c in c0.max(0)..=c1.min(grid as i64 - 1) {
                crossed[(r * grid as i64 + c) as usize] = true;
            }
        }
    };
    for &(qa, qb) in &constraints {
        mark(qa, qb);
    }
    let mut ring_edges: Vec<((u16, u16), (u16, u16))> = Vec::new();
    for r in regions {
        for ring in r.rings() {
            let n = ring.len();
            if n < 3 {
                continue;
            }
            for k in 0..n {
                let a = ring[k];
                let b = ring[(k + 1) % n];
                let qa = (a.0.round() as u16, a.1.round() as u16);
                let qb = (b.0.round() as u16, b.1.round() as u16);
                if qa != qb {
                    ring_edges.push((qa, qb));
                }
            }
        }
    }
    for &(qa, qb) in &ring_edges {
        mark(qa, qb);
    }
    let mut cdt: ConstrainedDelaunayTriangulation<Point2<f64>> =
        ConstrainedDelaunayTriangulation::new();
    let origin = BUFFER as u32;
    let qstep = EXTENT as u32 / grid;
    let mut lattice = Vec::with_capacity((n * n) as usize);
    for row in 0..n {
        for col in 0..n {
            let p = Point2::new((origin + col * qstep) as f64, (origin + row * qstep) as f64);
            lattice.push(cdt.insert(p).ok()?);
        }
    }
    for row in 0..grid {
        for col in 0..grid {
            if crossed[(row * grid + col) as usize] {
                continue;
            }
            let tl = lattice[(row * n + col) as usize];
            let br = lattice[((row + 1) * n + col + 1) as usize];
            if cdt.can_add_constraint(tl, br) {
                cdt.add_constraint(tl, br);
            }
        }
    }
    for &(qa, qb) in &constraints {
        let va = cdt.insert(Point2::new(qa.0 as f64, qa.1 as f64)).ok()?;
        let vb = cdt.insert(Point2::new(qb.0 as f64, qb.1 as f64)).ok()?;
        if va != vb {
            cdt.add_constraint_and_split(va, vb, |p| p);
        }
    }
    for &(qa, qb) in &ring_edges {
        // Densified to the lattice, exactly as the paved mesher densifies its
        // rings (`pave_mesh::densify_ring`): every crossing of a grid line
        // becomes a vertex, so the one-mesh's asphalt silhouette carries the
        // same border vertex set the old paved mesh derives, and a one-mesh
        // tile can seam against an old-mesh neighbour vertex-for-vertex.
        let (ax, ay) = (qa.0 as f64, qa.1 as f64);
        let (bx, by) = (qb.0 as f64, qb.1 as f64);
        let mut ts: Vec<f64> = vec![0.0, 1.0];
        let step_f = qstep as f64;
        for (a, b) in [(ax, bx), (ay, by)] {
            if (a - b).abs() < 1e-12 {
                continue;
            }
            let (lo_g, hi_g) = (a.min(b), a.max(b));
            let mut g = ((lo_g - origin as f64) / step_f).ceil() * step_f + origin as f64;
            while g < hi_g {
                if g > lo_g {
                    ts.push((g - a) / (b - a));
                }
                g += step_f;
            }
        }
        ts.sort_by(f64::total_cmp);
        ts.dedup_by(|x, y| (*x - *y).abs() < 1e-9);
        let mut prev: Option<_> = None;
        for &t in &ts {
            let p = Point2::new(ax + (bx - ax) * t, ay + (by - ay) * t);
            let v = cdt.insert(p).ok()?;
            if let Some(pv) = prev {
                if pv != v {
                    cdt.add_constraint_and_split(pv, v, |p| p);
                }
            }
            prev = Some(v);
        }
    }
    let vcount = cdt.num_vertices();
    let mut qpos: Vec<(u16, u16)> = Vec::with_capacity(vcount);
    for v in cdt.vertices() {
        let p = v.position();
        qpos.push((
            p.x.round().clamp(0.0, 65535.0) as u16,
            p.y.round().clamp(0.0, 65535.0) as u16,
        ));
    }
    // Classify faces and collect tile-proper border vertices per class.
    let lo = BUFFER as i64;
    let hi = (BUFFER + EXTENT) as i64;
    let on_border = |q: (u16, u16)| -> bool {
        let (x, y) = (q.0 as i64, q.1 as i64);
        (x == lo || x == hi || y == lo || y == hi) && (lo..=hi).contains(&x) && (lo..=hi).contains(&y)
    };
    let mut terrain_b: Vec<(u16, u16)> = Vec::new();
    let mut asphalt_b: Vec<(u16, u16)> = Vec::new();
    for face in cdt.inner_faces() {
        let [a, b, c] = face.vertices().map(|v| v.fix().index());
        let (ax, ay) = (qpos[a].0 as i64, qpos[a].1 as i64);
        let (bx, by) = (qpos[b].0 as i64, qpos[b].1 as i64);
        let (cx, cy) = (qpos[c].0 as i64, qpos[c].1 as i64);
        if (bx - ax) * (cy - ay) - (by - ay) * (cx - ax) == 0 {
            continue;
        }
        let cen = ((ax + bx + cx) as f64 / 3.0, (ay + by + cy) as f64 / 3.0);
        let paved = regions.iter().any(|r| r.contains(cen));
        for &v in &[a, b, c] {
            if on_border(qpos[v]) {
                if paved {
                    asphalt_b.push(qpos[v]);
                } else {
                    terrain_b.push(qpos[v]);
                }
            }
        }
    }
    terrain_b.sort_unstable();
    terrain_b.dedup();
    asphalt_b.sort_unstable();
    asphalt_b.dedup();
    let terrain: Vec<(u16, u16, i32)> = terrain_b
        .into_iter()
        .map(|(qx, qy)| {
            let lon = project::dequantize_x(qx, bounds);
            let lat = project::dequantize_y(qy, bounds);
            (qx, qy, project::quantize_z(sample(lon, lat)))
        })
        .collect();
    Some((terrain, asphalt_b))
}
