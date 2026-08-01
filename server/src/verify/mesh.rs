//! Meshes as *surfaces*, not as vertex lists.
//!
//! The distinction is the whole reason this module exists. The probe that
//! preceded it asked every road vertex whether it stood above the terrain, and
//! answered "mostly" while the asphalt was visibly chording across the ground
//! between those vertices — the defect lived strictly *between* the samples, so
//! a vertex-level instrument was blind to it by construction.
//!
//! So a check here samples the interior of every triangle at a metric spacing,
//! and interrogates the other mesh as a continuous field. Both halves matter:
//! sampling the road's interior finds where the road's own triangulation is too
//! coarse to hold what it read, and interpolating the terrain's triangles finds
//! where the ground's triangulation crosses it.
//!
//! Plan coordinates are unit-tile space throughout — 0 at the west/south edge
//! of the tile proper, 1 at the east/north, negative and >1 in the buffer
//! (FORMAT.md §5) — and heights are metres above the ellipsoid.

use std::collections::HashMap;

use crate::fb::tile::arpentry::tiles as fbt;
use crate::project::Bounds;

/// Metres per unit of tile-local plan space, per axis. Tiles are 2:1 in
/// degrees and longitude degrees shorten with latitude, so the two differ by
/// roughly a factor of two at the equator and more towards the poles; a check
/// that measured triangle size in unit space would sample a polar tile
/// hundreds of times more finely than an equatorial one.
#[derive(Clone, Copy, Debug)]
pub struct Scale {
    pub mx: f64,
    pub my: f64,
}

impl Scale {
    pub fn of(b: &Bounds) -> Scale {
        let mid = (b.south + b.north) * 0.5;
        Scale {
            mx: b.width() * 111_320.0 * mid.to_radians().cos().abs().max(1e-6),
            my: b.height() * 110_540.0,
        }
    }

    /// Plan distance in metres between two unit-space points.
    pub fn dist(&self, ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
        let dx = (bx - ax) * self.mx;
        let dy = (by - ay) * self.my;
        (dx * dx + dy * dy).sqrt()
    }
}

const EXTENT: f64 = 32768.0;
const BUFFER: f64 = 16384.0;

/// An undirected mesh edge, keyed by its two endpoints on the integer plan
/// lattice so coincident-but-distinct vertices weld.
type EdgeKey = ((i64, i64), (i64, i64));

/// One triangle, as the steepness checks see it.
#[derive(Clone, Copy, Debug)]
pub struct Face {
    pub index: usize,
    /// Rise over plan run of the triangle's steepest edge.
    pub slope: f64,
    /// The height that edge spans, in metres. A steep face spanning a
    /// millimetre is quantization on a sliver; one spanning eight metres is a
    /// wall. Without this the two are indistinguishable by ratio alone.
    pub rise: f64,
    pub x: f64,
    pub y: f64,
}

/// Dequantizes a tile-local `uint16` to unit-tile space (FORMAT.md §10).
pub fn dequantize(q: u16) -> f64 {
    (q as f64 - BUFFER) / EXTENT
}

/// A triangulated surface with a plan index, queried as a height field.
pub struct SurfaceMesh {
    x: Vec<f32>,
    y: Vec<f32>,
    z: Vec<f32>,
    idx: Vec<u32>,
    grid: Grid,
}

/// Uniform plan bucketing of triangles, in CSR form. Terrain meshes run to tens
/// of thousands of triangles and every road sample queries them, so the linear
/// scan the old probe used costs an hour where this costs a second.
struct Grid {
    lo_x: f64,
    lo_y: f64,
    inv_w: f64,
    inv_h: f64,
    nx: usize,
    ny: usize,
    starts: Vec<u32>,
    items: Vec<u32>,
}

impl SurfaceMesh {
    /// Reads a `MeshGeometry` into unit plan space and metric heights.
    /// Returns `None` for a mesh with no triangles — an empty surface answers
    /// no query, and every caller would otherwise have to check.
    pub fn from_geometry(g: &fbt::MeshGeometry<'_>) -> Option<SurfaceMesh> {
        let (gx, gy, gz) = (g.x(), g.y(), g.z());
        let gi = g.indices();
        let n = gx.len().min(gy.len()).min(gz.len());
        if n == 0 || gi.len() < 3 {
            return None;
        }
        let x: Vec<f32> = (0..n).map(|i| dequantize(gx.get(i)) as f32).collect();
        let y: Vec<f32> = (0..n).map(|i| dequantize(gy.get(i)) as f32).collect();
        let z: Vec<f32> = (0..n).map(|i| gz.get(i) as f32 * 0.001).collect();
        // Drop any triangle indexing past the vertex arrays rather than
        // panicking on it: a malformed mesh is a finding for the check that
        // notices it, not a crash in the reader.
        let mut idx: Vec<u32> = Vec::with_capacity(gi.len());
        for t in 0..gi.len() / 3 {
            let (a, b, c) = (gi.get(t * 3), gi.get(t * 3 + 1), gi.get(t * 3 + 2));
            if (a as usize) < n && (b as usize) < n && (c as usize) < n {
                idx.extend_from_slice(&[a, b, c]);
            }
        }
        SurfaceMesh::from_parts(x, y, z, idx)
    }

    /// Builds a surface from raw arrays already in unit plan space and metres.
    /// The synthetic-scenario checks construct meshes this way, and so do the
    /// tests; `None` for anything with no usable triangle.
    pub fn from_parts(x: Vec<f32>, y: Vec<f32>, z: Vec<f32>, idx: Vec<u32>) -> Option<SurfaceMesh> {
        if idx.len() < 3 || x.len() != y.len() || x.len() != z.len() {
            return None;
        }
        let grid = Grid::build(&x, &y, &idx);
        Some(SurfaceMesh { x, y, z, idx, grid })
    }

    pub fn vertex_count(&self) -> usize {
        self.x.len()
    }

    pub fn triangle_count(&self) -> usize {
        self.idx.len() / 3
    }

    /// Triangle `t` as three `(x, y, z)` corners.
    pub fn triangle(&self, t: usize) -> [(f64, f64, f64); 3] {
        let c = |k: usize| {
            let v = self.idx[t * 3 + k] as usize;
            (self.x[v] as f64, self.y[v] as f64, self.z[v] as f64)
        };
        [c(0), c(1), c(2)]
    }

    /// Vertex `i` as `(x, y, z)`.
    pub fn vertex(&self, i: usize) -> (f64, f64, f64) {
        (self.x[i] as f64, self.y[i] as f64, self.z[i] as f64)
    }

    /// Surface height at a plan position, by barycentric interpolation of the
    /// covering triangle. `None` outside the mesh.
    ///
    /// Where triangles overlap in plan — which for a road surface means a ramp
    /// folded over itself, a defect in its own right — the *highest* answer
    /// wins, so a check never reports a clearance against a surface the eye
    /// cannot see.
    pub fn height_at(&self, px: f64, py: f64) -> Option<f64> {
        let mut best: Option<f64> = None;
        for &t in self.grid.candidates(px, py) {
            if let Some(h) = self.interpolate(t as usize, px, py) {
                best = Some(match best {
                    Some(b) if b >= h => b,
                    _ => h,
                });
            }
        }
        best
    }

    /// Lowest and highest surface at a plan position, as `(min, max)`.
    ///
    /// A baked structure is a closed solid, so both faces answer: the maximum
    /// is the deck top or the bore roof, the minimum is the deck soffit or the
    /// bore invert. Which one a check wants is the check's business — assuming
    /// a thickness instead would bake a prior into the instrument.
    pub fn height_range_at(&self, px: f64, py: f64) -> Option<(f64, f64)> {
        let mut range: Option<(f64, f64)> = None;
        for &t in self.grid.candidates(px, py) {
            if let Some(h) = self.interpolate(t as usize, px, py) {
                range = Some(match range {
                    Some((lo, hi)) => (lo.min(h), hi.max(h)),
                    None => (h, h),
                });
            }
        }
        range
    }

    /// Which triangles have at least one edge no other triangle shares — the
    /// mesh silhouette.
    ///
    /// The carriageway's silhouette is its kerb, which is vertical on purpose,
    /// so a steepness check that counted it would read a designed rim as a
    /// permanent defect and never reach zero.
    pub fn boundary_faces(&self) -> Vec<bool> {
        // Key an edge by its two plan positions on the integer lattice the
        // format stores, so two coincident-but-distinct vertices still weld —
        // otherwise every seam in the triangulation would read as silhouette.
        let key = |v: (f64, f64, f64)| {
            ((v.0 * EXTENT).round() as i64, (v.1 * EXTENT).round() as i64)
        };
        let n = self.triangle_count();
        let mut count: HashMap<EdgeKey, u32> = HashMap::with_capacity(n * 2);
        let mut edges: Vec<[EdgeKey; 3]> = Vec::with_capacity(n);
        for t in 0..n {
            let tri = self.triangle(t);
            let mut e = [((0, 0), (0, 0)); 3];
            for i in 0..3 {
                let (a, b) = (key(tri[i]), key(tri[(i + 1) % 3]));
                let k = if a <= b { (a, b) } else { (b, a) };
                *count.entry(k).or_insert(0) += 1;
                e[i] = k;
            }
            edges.push(e);
        }
        edges.iter().map(|e| e.iter().any(|k| count[k] == 1)).collect()
    }

    /// Visits every triangle's steepest edge as rise over plan run, with the
    /// height that edge spans and the triangle's plan centroid.
    ///
    /// Local, needing no other surface, and the sharpest single signal of the
    /// pathologies that hide from every cross-mesh check: a retaining wall
    /// manufactured on a flank holds almost no plan area while spanning its
    /// whole height, and a road surface cannot legitimately exceed its class
    /// grade ceiling. Degenerate triangles (no plan area at all) report
    /// infinite slope, which is the honest answer.
    pub fn face_slopes<F: FnMut(Face)>(&self, scale: &Scale, mut f: F) {
        for t in 0..self.triangle_count() {
            let tri = self.triangle(t);
            let (mut slope, mut rise) = (0.0f64, 0.0f64);
            for i in 0..3 {
                let (a, b) = (tri[i], tri[(i + 1) % 3]);
                let run = scale.dist(a.0, a.1, b.0, b.1);
                let dz = (b.2 - a.2).abs();
                if dz <= 0.0 {
                    continue;
                }
                let s = if run > 1e-6 { dz / run } else { f64::INFINITY };
                if s > slope {
                    slope = s;
                    rise = dz;
                }
            }
            f(Face {
                index: t,
                slope,
                rise,
                x: (tri[0].0 + tri[1].0 + tri[2].0) / 3.0,
                y: (tri[0].1 + tri[1].1 + tri[2].1) / 3.0,
            });
        }
    }

    /// The steepest triangle in the mesh, with its plan centroid.
    pub fn max_slope(&self, scale: &Scale) -> Option<(f64, (f64, f64))> {
        let mut worst: Option<(f64, (f64, f64))> = None;
        self.face_slopes(scale, |f| {
            if worst.is_none_or(|(w, _)| f.slope > w) {
                worst = Some((f.slope, (f.x, f.y)));
            }
        });
        worst
    }

    /// Height of triangle `t` at `(px, py)`, or `None` if the point is outside
    /// it. The epsilon admits points on a shared edge to both neighbours, which
    /// is what keeps a seam from reading as a hole.
    fn interpolate(&self, t: usize, px: f64, py: f64) -> Option<f64> {
        let [(ax, ay, az), (bx, by, bz), (cx, cy, cz)] = self.triangle(t);
        let d = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy);
        if d.abs() < 1e-18 {
            return None;
        }
        let l1 = ((by - cy) * (px - cx) + (cx - bx) * (py - cy)) / d;
        let l2 = ((cy - ay) * (px - cx) + (ax - cx) * (py - cy)) / d;
        let l3 = 1.0 - l1 - l2;
        const EPS: f64 = -1e-9;
        (l1 >= EPS && l2 >= EPS && l3 >= EPS).then_some(l1 * az + l2 * bz + l3 * cz)
    }

    /// Visits every triangle of this mesh at a plan spacing of about
    /// `spacing_m` metres, passing `(x, y, z)` in unit plan space and metres.
    ///
    /// The pattern is the barycentric lattice at subdivision `k`, which always
    /// includes the three corners, the edge midpoints and the centroid — so it
    /// subsumes the vertex-level probe rather than replacing it, and the
    /// interior points are exactly where a chord stands furthest from what it
    /// chords over.
    pub fn sample<F: FnMut(f64, f64, f64)>(&self, scale: &Scale, spacing_m: f64, mut f: F) {
        for t in 0..self.triangle_count() {
            let tri = self.triangle(t);
            let k = subdivision(&tri, scale, spacing_m);
            let kf = k as f64;
            for i in 0..=k {
                for j in 0..=(k - i) {
                    let (l1, l2) = (i as f64 / kf, j as f64 / kf);
                    let l3 = 1.0 - l1 - l2;
                    f(
                        l1 * tri[0].0 + l2 * tri[1].0 + l3 * tri[2].0,
                        l1 * tri[0].1 + l2 * tri[1].1 + l3 * tri[2].1,
                        l1 * tri[0].2 + l2 * tri[1].2 + l3 * tri[2].2,
                    );
                }
            }
        }
    }
}

/// How finely to subdivide a triangle so its samples sit about `spacing_m`
/// apart. Capped because one enormous triangle — the very defect being hunted —
/// must not be allowed to dominate the sample budget.
fn subdivision(tri: &[(f64, f64, f64); 3], scale: &Scale, spacing_m: f64) -> usize {
    const MAX: usize = 24;
    let longest = (0..3)
        .map(|i| {
            let (a, b) = (tri[i], tri[(i + 1) % 3]);
            scale.dist(a.0, a.1, b.0, b.1)
        })
        .fold(0.0, f64::max);
    ((longest / spacing_m.max(1e-3)).ceil() as usize).clamp(1, MAX)
}

impl Grid {
    fn build(x: &[f32], y: &[f32], idx: &[u32]) -> Grid {
        let tris = idx.len() / 3;
        let side = (tris as f64).sqrt().ceil().clamp(1.0, 256.0) as usize;
        let (mut lo_x, mut lo_y) = (f64::INFINITY, f64::INFINITY);
        let (mut hi_x, mut hi_y) = (f64::NEG_INFINITY, f64::NEG_INFINITY);
        for i in 0..x.len() {
            lo_x = lo_x.min(x[i] as f64);
            hi_x = hi_x.max(x[i] as f64);
            lo_y = lo_y.min(y[i] as f64);
            hi_y = hi_y.max(y[i] as f64);
        }
        // A degenerate extent (a mesh one line wide) still needs a positive
        // cell size, or every query would divide by zero.
        let w = (hi_x - lo_x).max(1e-9);
        let h = (hi_y - lo_y).max(1e-9);
        let (nx, ny) = (side, side);
        let g = Grid {
            lo_x,
            lo_y,
            inv_w: nx as f64 / w,
            inv_h: ny as f64 / h,
            nx,
            ny,
            starts: Vec::new(),
            items: Vec::new(),
        };

        // Two passes into CSR: count per cell, prefix-sum, then place.
        let mut counts = vec![0u32; nx * ny + 1];
        let mut spans = Vec::with_capacity(tris);
        for t in 0..tris {
            let s = g.span(x, y, idx, t);
            for cy in s.1 .0..=s.1 .1 {
                for cx in s.0 .0..=s.0 .1 {
                    counts[cy * nx + cx + 1] += 1;
                }
            }
            spans.push(s);
        }
        for i in 1..counts.len() {
            counts[i] += counts[i - 1];
        }
        let mut items = vec![0u32; counts[counts.len() - 1] as usize];
        let mut cursor = counts.clone();
        for (t, s) in spans.iter().enumerate() {
            for cy in s.1 .0..=s.1 .1 {
                for cx in s.0 .0..=s.0 .1 {
                    let c = cy * nx + cx;
                    items[cursor[c] as usize] = t as u32;
                    cursor[c] += 1;
                }
            }
        }
        Grid { starts: counts, items, ..g }
    }

    /// The inclusive cell range triangle `t`'s bounding box covers.
    #[allow(clippy::type_complexity)]
    fn span(
        &self,
        x: &[f32],
        y: &[f32],
        idx: &[u32],
        t: usize,
    ) -> ((usize, usize), (usize, usize)) {
        let v: [usize; 3] =
            [idx[t * 3] as usize, idx[t * 3 + 1] as usize, idx[t * 3 + 2] as usize];
        let (mut x0, mut x1) = (f64::INFINITY, f64::NEG_INFINITY);
        let (mut y0, mut y1) = (f64::INFINITY, f64::NEG_INFINITY);
        for &i in &v {
            x0 = x0.min(x[i] as f64);
            x1 = x1.max(x[i] as f64);
            y0 = y0.min(y[i] as f64);
            y1 = y1.max(y[i] as f64);
        }
        ((self.cell_x(x0), self.cell_x(x1)), (self.cell_y(y0), self.cell_y(y1)))
    }

    fn cell_x(&self, x: f64) -> usize {
        (((x - self.lo_x) * self.inv_w) as isize).clamp(0, self.nx as isize - 1) as usize
    }

    fn cell_y(&self, y: f64) -> usize {
        (((y - self.lo_y) * self.inv_h) as isize).clamp(0, self.ny as isize - 1) as usize
    }

    /// Triangles whose bounding box covers the cell containing `(px, py)`.
    fn candidates(&self, px: f64, py: f64) -> &[u32] {
        let c = self.cell_y(py) * self.nx + self.cell_x(px);
        let (a, b) = (self.starts[c] as usize, self.starts[c + 1] as usize);
        &self.items[a..b]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A unit square split into two triangles, tilted so height varies with x.
    fn ramp() -> SurfaceMesh {
        SurfaceMesh::from_parts(
            vec![0.0, 1.0, 1.0, 0.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 10.0, 10.0, 0.0],
            vec![0, 1, 2, 0, 2, 3],
        )
        .unwrap()
    }

    #[test]
    fn interpolates_inside_and_declines_outside() {
        let m = ramp();
        assert_eq!(m.height_at(0.0, 0.0), Some(0.0));
        assert!((m.height_at(0.5, 0.5).unwrap() - 5.0).abs() < 1e-9);
        assert!((m.height_at(0.25, 0.75).unwrap() - 2.5).abs() < 1e-9);
        assert_eq!(m.height_at(-0.5, 0.5), None);
        assert_eq!(m.height_at(0.5, 2.0), None);
    }

    #[test]
    fn the_shared_edge_answers_rather_than_reading_as_a_hole() {
        // The diagonal belongs to both triangles; a strict test would make the
        // seam a line of missing samples and hide defects along it.
        let m = ramp();
        for i in 1..10 {
            let t = i as f64 / 10.0;
            assert!(m.height_at(t, t).is_some(), "diagonal at {t}");
        }
    }

    #[test]
    fn sampling_reaches_triangle_interiors_not_just_corners() {
        // The point of the module: a 100 m triangle sampled at 1 m must produce
        // interior samples, including the centroid where a chord deviates most.
        let m = ramp();
        let scale = Scale { mx: 100.0, my: 100.0 };
        let mut n = 0;
        let mut saw_centroid = false;
        m.sample(&scale, 1.0, |x, y, _| {
            n += 1;
            // Centroid of the lower triangle (0,0) (1,0) (1,1).
            if (x - 2.0 / 3.0).abs() < 1e-6 && (y - 1.0 / 3.0).abs() < 1e-6 {
                saw_centroid = true;
            }
        });
        assert!(n > 100, "only {n} samples");
        assert!(saw_centroid, "centroid never sampled");
    }

    #[test]
    fn a_coarse_mesh_is_still_sampled_at_its_corners() {
        // spacing far larger than the triangle: k clamps to 1, giving exactly
        // the three corners — the vertex-level probe as the degenerate case.
        let m = ramp();
        let scale = Scale { mx: 1.0, my: 1.0 };
        let mut n = 0;
        m.sample(&scale, 1000.0, |_, _, _| n += 1);
        assert_eq!(n, 6, "two triangles, three corners each");
    }

    #[test]
    fn sampling_a_chord_finds_the_deviation_a_vertex_probe_misses() {
        // The exact failure the module exists for. A flat road slab spans a
        // valley whose ground mesh dips in the middle. At every shared corner
        // the two agree; only interior samples see the road under the ground.
        let road = SurfaceMesh::from_parts(
            vec![0.0, 1.0, 1.0, 0.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![10.0, 10.0, 10.0, 10.0],
            vec![0, 1, 2, 0, 2, 3],
        )
        .unwrap();
        // Ground: same corners at 10 m, but a ridge vertex at the centre at 14 m.
        let ground = SurfaceMesh::from_parts(
            vec![0.0, 1.0, 1.0, 0.0, 0.5],
            vec![0.0, 0.0, 1.0, 1.0, 0.5],
            vec![10.0, 10.0, 10.0, 10.0, 14.0],
            vec![0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4],
        )
        .unwrap();
        let scale = Scale { mx: 100.0, my: 100.0 };

        let mut worst_vertices = 0.0f64;
        for i in 0..road.vertex_count() {
            let (x, y, z) = road.vertex(i);
            if let Some(g) = ground.height_at(x, y) {
                worst_vertices = worst_vertices.max(g - z);
            }
        }
        assert_eq!(worst_vertices, 0.0, "vertices agree — this is the blind spot");

        let mut worst_surface = 0.0f64;
        road.sample(&scale, 2.0, |x, y, z| {
            if let Some(g) = ground.height_at(x, y) {
                worst_surface = worst_surface.max(g - z);
            }
        });
        assert!(worst_surface > 3.5, "surface sampling must see the ridge, saw {worst_surface}");
    }

    #[test]
    fn the_grid_agrees_with_a_linear_scan() {
        let m = ramp();
        for i in 0..20 {
            for j in 0..20 {
                let (px, py) = (i as f64 / 19.0, j as f64 / 19.0);
                let indexed = m.height_at(px, py);
                let scanned = (0..m.triangle_count())
                    .filter_map(|t| m.interpolate(t, px, py))
                    .fold(None::<f64>, |a, h| Some(a.map_or(h, |b: f64| b.max(h))));
                assert_eq!(indexed, scanned, "at {px},{py}");
            }
        }
    }

    #[test]
    fn scale_shrinks_longitude_with_latitude() {
        let equator = Scale::of(&Bounds { west: 0.0, south: 0.0, east: 1.0, north: 1.0 });
        let north = Scale::of(&Bounds { west: 0.0, south: 60.0, east: 1.0, north: 61.0 });
        assert!(north.mx < equator.mx * 0.55, "60N must be about half as wide");
        assert!((north.my - equator.my).abs() < 1.0, "latitude spacing is constant");
    }
}
