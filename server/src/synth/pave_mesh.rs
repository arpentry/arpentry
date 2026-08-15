//! Meshing the unioned road surface — one tile's asphalt as an opaque interior
//! plus an antialiased casing rim (docs/ROADS.md §6.1, P2).
//!
//! [`crate::synth::pavement`] produces the paved region as rings; this turns one
//! tile's share of it into [`TerrainMesh`]es. Three things have to be true at
//! once, and each shapes the design:
//!
//! 1. **Neighbouring tiles must not double-draw.** The region is clipped to the
//!    tile *proper*, never into the format's buffer — the discipline
//!    `synth::structure` already applies to opaque solids. Two tiles that each
//!    meshed into the buffer would blend their rims twice over the overlap.
//!
//! 2. **The seam must be invisible.** Every vertex on a tile cut comes from a
//!    *global* object clipped to that same line — the chunk's own ring, clipped
//!    against a rect both neighbours compute identically — and its height comes
//!    from the global height field. So both sides derive the same seam vertices,
//!    the same quantized coordinates, and the same heights. Interior
//!    connectivity may differ; only the seam profile has to match, exactly as
//!    `terrain_cdt`'s module doc promises for the terrain.
//!
//! 3. **The antialiasing rim must not run along a tile cut.** `edge_across`
//!    fades the outer pixel of a drivable surface into the ground
//!    (`fs_deck` in `client/shaders/terrain.wgsl`). Along a real silhouette that
//!    is what makes the edge crisp; along a tile cut it would draw a faded line
//!    down every tile border. Cut edges are therefore detected — both endpoints
//!    exactly on one side of the rect, which holds because the clip snaps them
//!    there — and their rim quads are skipped, so the asphalt runs to the border
//!    at full opacity and meets its neighbour invisibly.
//!
//! The interior is triangulated with the same constrained-Delaunay machinery and
//! the same determinism contract as `terrain_cdt` (read its module doc): the
//! triangulation runs in quantized tile-local `u16` coordinates held exactly in
//! `f64`, points go in in a fixed order, crossings are resolved by
//! `add_constraint_and_split`, split vertices are rounded and then *re-sampled at
//! the rounded position* so a stored height is exact for its stored vertex, and
//! faces are wound by signed area with the degenerate ones dropped.

use geo_types::Coord;
use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation};

use crate::building_mesh::Frame;
use crate::scene::DEG_M;
use crate::ground::sampler::GroundSampler;
use crate::priors;
use crate::project::{self, Bounds};
use crate::synth::height::HeightField;
use crate::synth::carriageway::Handover;
use crate::synth::pavement::LevelShapes;
use crate::synth::region::Region;
use crate::terrain::TerrainMesh;

/// A boundary ring clipped to the tile, with each edge flagged as a tile cut.
/// `cut[i]` describes the edge from `pts[i]` to `pts[i + 1]`.
struct TaggedRing {
    pts: Vec<Coord>,
    cut: Vec<bool>,
    /// Whether this ring bounds paved area (outer) rather than a hole.
    outer: bool,
}

/// The paved surface of one tile at one level: the opaque interior and the rim
/// that antialiases and casings its silhouette.
pub struct PavedMesh {
    pub level: i64,
    /// The region's material ([`crate::priors::Surface`]) — asphalt or
    /// ballast — which picks the class the feature is emitted under, and with
    /// it the style entry that colours it.
    pub material: crate::priors::Surface,
    pub surface: TerrainMesh,
    pub casing: Option<TerrainMesh>,
    /// A point inside the region, for the feature's anchor geometry.
    pub anchor: Coord,
    /// The wall between the kerb and the ground beside it, where the two are
    /// not the same height. `None` where every edge sits on its own bench and
    /// there is nothing to close.
    pub apron: Option<TerrainMesh>,
    /// The region this mesh actually covers — the true silhouette rings in plan.
    /// The terrain mesher cuts its hole from *this*, so a level whose asphalt
    /// failed to mesh cuts nothing (docs/GENERATION.md I6: plain, not
    /// wrong).
    pub region: Region,
}

/// Meshes every level of a chunk's region that reaches this tile.
///
/// `None` for a tile the region misses entirely. Degrades rather than fails: a
/// ring whose inset would fold emits no rim and antialiases through MSAA instead
/// (docs/GENERATION.md I6).
pub fn tile_meshes(
    levels: &[LevelShapes],
    field: &HeightField,
    sampler: &mut GroundSampler,
    z: u8,
    z_ref: u8,
    bounds: &Bounds,
    hole: bool,
    handovers: &[Handover],
) -> Vec<PavedMesh> {
    let mut out = Vec::new();
    for ls in levels {
        let rings = clip_to_tile(&ls.shapes, bounds);
        if rings.is_empty() {
            continue;
        }
        let anchor = rings
            .iter()
            .find(|r| r.outer)
            .and_then(|r| r.pts.first().copied())
            .unwrap_or(Coord { x: bounds.west, y: bounds.south });
        // Simplify the boundary to the zoom's own detail budget before meshing.
        // The union is built at sub-millimetre precision, which is right for the
        // model and absurd for a tile: at z13 a tile spans kilometres, so tens of
        // thousands of boundary vertices are invisible, cost a height-field
        // evaluation each, and bloat the archive. This is the same Douglas-Peucker
        // budget the tiler already applies to every line
        // (`pipeline::tolerance`).
        //
        // Cut runs are left verbatim. Their vertices are the shared seam: the
        // neighbouring tile derives the same ones by clipping the same ring
        // against the same border, and heights vary non-linearly along it, so
        // thinning one side and not the other would open a visible crack.
        // Capped at `PAVE_SIMPLIFY_M`: the generic per-zoom budget is sized for
        // cartographic lines, where only the path matters, and at z13 it would
        // move a carriageway edge by a fifth of the road's own width.
        let tol = crate::pipeline::tolerance(z).min(priors::PAVE_SIMPLIFY_M / crate::scene::DEG_M);
        let rings: Vec<TaggedRing> = rings.iter().map(|r| simplify_ring(r, tol)).collect();
        // …then densified back to the terrain's own resolution, so the
        // silhouette samples the ground as often as the ground is drawn.
        let grid = crate::terrain::grid_for(z, z_ref);
        let rings: Vec<TaggedRing> =
            rings.iter().map(|r| densify_ring(r, bounds, grid)).collect();

        let mut scratch = Vec::new();
        let mut height = |lon: f64, lat: f64| {
            let h = field.at(sampler, ls.level, ls.layer, z, z_ref, bounds, lon, lat, &mut scratch);
            project::quantize_z(h)
        };
        let probe = std::env::var_os("ARPT_PAVE_PROBE").is_some();
        let t = std::time::Instant::now();
        let verts: usize = rings.iter().map(|r| r.pts.len()).sum();
        let meshed =
            mesh_rings(&rings, bounds, crate::terrain::grid_for(z, z_ref), hole, handovers, &mut height);
        // The apron is the wall the hole exposes, so it is built only where the
        // hole is cut and only for the at-grade surface: a deck's silhouette is
        // its own edge over open air, not a kerb against the ground.
        let apron = if hole && ls.level == 0 {
            // `height` holds the sampler; release it before the apron's own
            // closure takes it, which needs both surfaces at once.
            drop(height);
            let mut both = |lon: f64, lat: f64| -> (i32, i32) {
                let road =
                    field.at(sampler, ls.level, ls.layer, z, z_ref, bounds, lon, lat, &mut scratch);
                // The same query the constrained terrain mesh makes for its own
                // vertices at this rung, so the apron's foot lands exactly on
                // the ground the neighbouring triangle draws.
                let ground = sampler.ground(lon, lat, z);
                (project::quantize_z(road), project::quantize_z(ground))
            };
            build_apron(&rings, bounds, &mut both)
        } else {
            None
        };
        if probe && t.elapsed().as_millis() > 100 {
            eprintln!(
                "[pave-mesh] z{} level {}: {} rings / {} ring-verts -> {} tris in {:?}",
                z,
                ls.level,
                rings.len(),
                verts,
                meshed.as_ref().map_or(0, |(m, _, _)| m.indices.len() / 3),
                t.elapsed()
            );
        }
        if let Some((surface, casing, region)) = meshed {
            out.push(PavedMesh {
                level: ls.level,
                material: ls.surface,
                surface,
                casing,
                anchor,
                region,
                apron,
            });
        }
    }
    out
}

/// Clips a level's rings to the tile proper and flags the cut edges.
///
/// A Sutherland–Hodgman clip per ring ([`crate::clip::clip_ring`], the one the
/// tiler already uses for polygons), not a boolean. The region of a built-up
/// chunk is a *single connected shape* whose bounding box is the whole chunk, so
/// no bounding-box reject can spare it: intersecting that shape with a detail
/// tile's rect took minutes per tile. The ring clip is linear in the vertices it
/// actually touches, and because [`crate::clip::intersect_x`] assigns the bound
/// coordinate verbatim, a cut vertex lands on the tile edge *exactly* — so the
/// seam needs no snapping pass and [`is_cut`] can compare with `==`.
///
/// Rings are clipped independently, as `clip.rs` does for a polygon's holes. A
/// hole wholly outside the tile disappears; one straddling the border comes back
/// hugging it, which the even-odd face test reads correctly either way.
fn clip_to_tile(shapes: &[Vec<Vec<Coord>>], bounds: &Bounds) -> Vec<TaggedRing> {
    let mut out = Vec::new();
    for shape in shapes {
        if !shape.first().is_some_and(|outer| ring_overlaps_tile(outer, bounds)) {
            continue;
        }
        for (i, ring) in shape.iter().enumerate() {
            let Some(clipped) = crate::clip::clip_ring(ring, bounds) else {
                continue;
            };
            let mut pts = clipped.0;
            // `clip_ring` re-closes; the rest of this module works on open rings.
            if pts.len() > 1 && pts[0] == *pts.last().expect("non-empty") {
                pts.pop();
            }
            if pts.len() < 3 {
                continue;
            }
            let n = pts.len();
            let cut = (0..n).map(|k| is_cut(pts[k], pts[(k + 1) % n], bounds)).collect();
            out.push(TaggedRing { pts, cut, outer: i == 0 });
        }
    }
    out
}

/// Thins a ring to `tol`, leaving every tile-cut run untouched.
///
/// Runs of equal cut flag are simplified independently with their endpoints
/// pinned, so the `cut` flags stay aligned with the edges they describe and a
/// seam run keeps all of its vertices.
fn simplify_ring(r: &TaggedRing, tol: f64) -> TaggedRing {
    let n = r.pts.len();
    if n < 4 || tol <= 0.0 {
        return TaggedRing { pts: r.pts.clone(), cut: r.cut.clone(), outer: r.outer };
    }
    // Start where the flag changes, so a run is never split across the seam of
    // the vertex list; with a uniform flag any start will do.
    let start = (0..n).find(|&i| r.cut[(i + n - 1) % n] != r.cut[i]).unwrap_or(0);
    let mut pts: Vec<Coord> = Vec::with_capacity(n);
    let mut cut: Vec<bool> = Vec::with_capacity(n);
    let mut k = 0usize;
    while k < n {
        let flag = r.cut[(start + k) % n];
        let mut len = 1usize;
        while k + len < n && r.cut[(start + k + len) % n] == flag {
            len += 1;
        }
        let run: Vec<Coord> = (0..=len).map(|t| r.pts[(start + k + t) % n]).collect();
        let kept = if flag { run } else { crate::simplify::douglas_peucker(&run, tol) };
        // The run's last vertex is the next run's first, so it is pushed there.
        for c in &kept[..kept.len() - 1] {
            pts.push(*c);
            cut.push(flag);
        }
        k += len;
    }
    if pts.len() < 3 {
        return TaggedRing { pts: r.pts.clone(), cut: r.cut.clone(), outer: r.outer };
    }
    TaggedRing { pts, cut, outer: r.outer }
}

/// Whether a ring's bounding box meets the tile at all.
fn ring_overlaps_tile(ring: &[Coord], b: &Bounds) -> bool {
    let mut bb = (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for c in ring {
        bb.0 = bb.0.min(c.x);
        bb.1 = bb.1.min(c.y);
        bb.2 = bb.2.max(c.x);
        bb.3 = bb.3.max(c.y);
    }
    bb.2 >= b.west && bb.0 <= b.east && bb.3 >= b.south && bb.1 <= b.north
}

/// Whether an edge lies along the tile border: both endpoints exactly on the
/// same side. Exact comparison is sound because [`snap`] put them there.
fn is_cut(a: Coord, b: Coord, bounds: &Bounds) -> bool {
    (a.x == bounds.west && b.x == bounds.west)
        || (a.x == bounds.east && b.x == bounds.east)
        || (a.y == bounds.south && b.y == bounds.south)
        || (a.y == bounds.north && b.y == bounds.north)
}

/// Builds the interior and rim meshes for one level's rings.
fn mesh_rings(
    rings: &[TaggedRing],
    bounds: &Bounds,
    grid: u32,
    hole: bool,
    handovers: &[Handover],
    height: &mut dyn FnMut(f64, f64) -> i32,
) -> Option<(TerrainMesh, Option<TerrainMesh>, Region)> {
    let up = Frame::at_center(bounds).encode_enu(0.0, 0.0, 1.0);
    let m_lon = crate::scene::DEG_M
        * ((bounds.south + bounds.north) * 0.5).to_radians().cos();

    // The inset boundary: what the interior is triangulated to, leaving the rim
    // between it and the true silhouette.
    let insets: Vec<Option<Vec<Coord>>> =
        rings.iter().map(|r| inset_ring(r, m_lon)).collect();

    let mut cdt: ConstrainedDelaunayTriangulation<Point2<f64>> =
        ConstrainedDelaunayTriangulation::new();
    // Constraint rings first, in ring then vertex order — a fixed insertion
    // order, so the triangulation is a function of the input alone.
    let mut constraint_rings: Vec<Vec<Coord>> = Vec::new();
    for (r, inset) in rings.iter().zip(&insets) {
        constraint_rings.push(inset.clone().unwrap_or_else(|| r.pts.clone()));
    }
    for ring in &constraint_rings {
        let q: Vec<(u16, u16)> = ring
            .iter()
            .map(|c| (project::quantize_x(c.x, bounds), project::quantize_y(c.y, bounds)))
            .collect();
        let n = q.len();
        for k in 0..n {
            let (a, b) = (q[k], q[(k + 1) % n]);
            if a == b {
                continue;
            }
            let va = cdt.insert(Point2::new(a.0 as f64, a.1 as f64)).ok()?;
            let vb = cdt.insert(Point2::new(b.0 as f64, b.1 as f64)).ok()?;
            if va != vb {
                cdt.add_constraint_and_split(va, vb, |p| p);
            }
        }
    }
    if cdt.num_vertices() < 3 {
        return None;
    }

    // The rings are quantized once, for the interior tests below and for the
    // face-centre test further down. Re-quantizing every ring vertex per query
    // made this O(faces x vertices) with a projection per step, which on a
    // detail tile is tens of millions of operations — the second half of why
    // the first version of this took minutes per tile.
    let qrings: Vec<Vec<(f64, f64)>> = constraint_rings
        .iter()
        .map(|r| {
            r.iter()
                .map(|c| {
                    (
                        project::quantize_x(c.x, bounds) as f64,
                        project::quantize_y(c.y, bounds) as f64,
                    )
                })
                .collect()
        })
        .collect();
    // One indexed region for both the interior-point pass and the face pass
    // below: each is O(faces x ring vertices) unindexed, which on a detail tile
    // is tens of millions of operations (see `synth::region`).
    let qregion = Region::new(qrings);

    // Interior sample points on the terrain's own lattice.
    //
    // Without them the region is triangulated from its *outline alone*, so a
    // carriageway is spanned by triangles as long as the road is wide and the
    // asphalt is a chord across whatever the ground does between its edges.
    // The terrain beside it is sampled every cell, so on a cross-slope the two
    // surfaces cross: the hillside surfaced through the asphalt in ragged
    // bites, worst exactly where the ground stage declined to bench and the
    // road is laid on the natural slope.
    //
    // The points are the *same* lattice the terrain mesh uses ([`grid`]), so
    // where the road rides the ground the two meshes sample the one field at
    // the one set of positions and agree there by construction, leaving only
    // the boundary strip to interpolation. They are global per zoom, so
    // neighbouring tiles derive identical ones, and they go in row-major, so
    // the triangulation stays a function of the input alone.
    let grid = grid.max(1);
    let qstep = (project::EXTENT / grid as f64).max(1.0);
    let n = grid + 1;
    for row in 0..n {
        let qy = project::BUFFER + row as f64 * qstep;
        for col in 0..n {
            let qx = project::BUFFER + col as f64 * qstep;
            // Inside the tile proper (the region is clipped to it, so a point
            // on the border sits on a cut edge) and inside the paved region.
            if qx <= project::BUFFER
                || qy <= project::BUFFER
                || qx >= project::BUFFER + project::EXTENT
                || qy >= project::BUFFER + project::EXTENT
            {
                continue;
            }
            if !qregion.contains((qx, qy)) {
                continue;
            }
            let _ = cdt.insert(Point2::new(qx, qy));
        }
    }

    // Vertices in handle order, heights re-sampled at the rounded position.
    let vcount = cdt.num_vertices();
    let mut x: Vec<u16> = Vec::with_capacity(vcount);
    let mut y: Vec<u16> = Vec::with_capacity(vcount);
    let mut zs: Vec<i32> = Vec::with_capacity(vcount);
    let mut normals: Vec<i8> = Vec::with_capacity(vcount * 2);
    for v in cdt.vertices() {
        let p = v.position();
        let qx = p.x.round().clamp(0.0, 65535.0) as u16;
        let qy = p.y.round().clamp(0.0, 65535.0) as u16;
        let lon = project::dequantize_x(qx, bounds);
        let lat = project::dequantize_y(qy, bounds);
        x.push(qx);
        y.push(qy);
        zs.push(height(lon, lat));
        normals.push(up.0);
        normals.push(up.1);
    }

    // Faces inside the region only: spade fills the convex hull, so the
    // concavities between roads and the islands inside a ring must go.
    let mut indices: Vec<u32> = Vec::new();
    for face in cdt.inner_faces() {
        let [a, b, c] = face.vertices().map(|v| v.fix().index());
        let (ax, ay) = (x[a] as i64, y[a] as i64);
        let (bx, by) = (x[b] as i64, y[b] as i64);
        let (cx, cy) = (x[c] as i64, y[c] as i64);
        let area2 = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
        if area2 == 0 {
            continue;
        }
        let cen = (
            (ax + bx + cx) as f64 / 3.0,
            (ay + by + cy) as f64 / 3.0,
        );
        if !qregion.contains(cen) {
            continue;
        }
        if area2 > 0 {
            indices.extend_from_slice(&[a as u32, b as u32, c as u32]);
        } else {
            indices.extend_from_slice(&[a as u32, c as u32, b as u32]);
        }
    }
    if indices.is_empty() {
        return None;
    }
    // The interior carries no across-coordinate: it is opaque everywhere, and
    // the rim is what fades. An empty `edge_across` means "no analytic AA" to
    // the decoder, which is exactly right for the interior.
    let mut surface = TerrainMesh { x, y, z: zs, indices, normals, edge_across: Vec::new() };
    // The rim splits by what its edge actually bounds. A kerb edge gets the
    // casing: the surface ends there, and the darker tone plus the analytic
    // fade are what edge it against the ground. A **handover** edge — the cut
    // where a deck takes over (`synth::carriageway::Handover`) — bounds nothing:
    // the road carries straight on across it, and edging it draws a kerb line
    // over the carriageway a third of a metre before the bridge. Those quads
    // keep their geometry (the interior is inset and something must cover the
    // strip) and join the *surface*, which is opaque, untoned and carries no
    // across-coordinate, so the asphalt runs into the deck unbroken.
    let (casing, handover) = build_rim(rings, &insets, bounds, up, hole, height, handovers);
    if let Some(h) = handover {
        append_mesh(&mut surface, h);
    }
    // The region the *terrain* is cut against is the true silhouette, not the
    // inset the interior was triangulated to: the asphalt reaches the silhouette
    // either way, via the rim where there is one and via the interior where the
    // inset folded. Plan positions only — the terrain's rim vertices take the
    // *ground's* height there, not the road's (docs/GROUND.md §3), and the
    // difference between the two is what `build_apron` draws.
    let sil: Vec<Vec<(f64, f64)>> = rings
        .iter()
        .map(|r| {
            r.pts
                .iter()
                .map(|c| {
                    (
                        project::quantize_x(c.x, bounds) as f64,
                        project::quantize_y(c.y, bounds) as f64,
                    )
                })
                .collect()
        })
        .collect();
    Some((surface, casing, Region::new(sil)))
}

/// Splits every ring edge where it crosses a line of the terrain lattice, so no
/// stretch of the silhouette spans more than one cell.
///
/// Interior lattice samples fix the middle of a carriageway but not its edge:
/// the boundary is a polyline of its own, and between two of its vertices the
/// asphalt edge is a straight chord in three dimensions while the ground under
/// it follows the lattice. On a steep flank the hillside rises through that
/// chord and eats scallops out of the road's edge — the residue left after the
/// interior was fixed, and the reason the silhouette is treated separately
/// rather than trusted to the boundary the union happened to produce.
///
/// The split points are the *global* lattice lines (tiles are aligned, so a
/// tile-local computation gives globally consistent lines), which keeps the
/// tile-border contract: neighbours clip the same ring against the same border
/// and split it at the same lines, so the seam vertices still match exactly.
/// Every piece of a split edge inherits its cut flag, so a densified border run
/// is still a border run and still grows no rim. Splitting *after*
/// simplification, not instead of it, keeps the boundary free of the union's
/// sub-millimetre noise while still sampling the ground where the ground moves.
fn densify_ring(r: &TaggedRing, bounds: &Bounds, grid: u32) -> TaggedRing {
    let n = r.pts.len();
    if grid <= 1 || n < 2 {
        return TaggedRing { pts: r.pts.clone(), cut: r.cut.clone(), outer: r.outer };
    }
    let (step_lon, step_lat) = (bounds.width() / grid as f64, bounds.height() / grid as f64);
    let mut pts = Vec::with_capacity(n * 2);
    let mut cut = Vec::with_capacity(n * 2);
    let mut ts: Vec<f64> = Vec::new();
    for k in 0..n {
        let (a, b) = (r.pts[k], r.pts[(k + 1) % n]);
        pts.push(a);
        cut.push(r.cut[k]);
        // Where the edge crosses each family of lattice lines, as parameters
        // along it. A line the edge only touches at an endpoint contributes
        // nothing: the endpoint is already a vertex.
        ts.clear();
        for (a0, b0, origin, step) in
            [(a.x, b.x, bounds.west, step_lon), (a.y, b.y, bounds.south, step_lat)]
        {
            let d = b0 - a0;
            if d.abs() < 1e-12 {
                continue;
            }
            let (lo, hi) = if d > 0.0 { (a0, b0) } else { (b0, a0) };
            let first = ((lo - origin) / step).floor() as i64 + 1;
            let last = ((hi - origin) / step).ceil() as i64 - 1;
            for i in first..=last {
                let t = (origin + i as f64 * step - a0) / d;
                if t > 1e-9 && t < 1.0 - 1e-9 {
                    ts.push(t);
                }
            }
        }
        if ts.is_empty() {
            continue;
        }
        ts.sort_by(|p, q| p.partial_cmp(q).expect("finite parameters"));
        let mut prev = 0.0;
        for &t in ts.iter() {
            if t - prev < 1e-9 {
                continue; // a corner crossing both families at once
            }
            prev = t;
            pts.push(Coord { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t });
            cut.push(r.cut[k]);
        }
    }
    TaggedRing { pts, cut, outer: r.outer }
}

/// Offsets a ring toward the paved material by [`priors::PAVE_RIM_M`], leaving
/// the rim between it and the true silhouette. `None` when the offset would fold
/// — a ring narrower than twice the rim, or a spur whose offset crosses itself;
/// the caller then meshes the ring itself and skips the rim.
///
/// Both ring kinds move the *same* way. Under the winding this module inherits
/// from the boolean (outer counter-clockwise, holes clockwise) the paved material
/// lies to the **left** of travel for both: walking an outer ring
/// counter-clockwise, left points into the region; walking a hole clockwise, left
/// points away from the hole and so also into the region. Shrinking the region
/// therefore moves an outer boundary inward and a hole boundary *outward*,
/// enlarging the hole — both by stepping left.
///
/// Giving holes the opposite sign, as this first did, shrinks them instead, so the
/// interior spills a rim's width into every island and median and each hole's rim
/// quads come out inverted.
fn inset_ring(ring: &TaggedRing, m_lon: f64) -> Option<Vec<Coord>> {
    let n = ring.pts.len();
    if n < 3 {
        return None;
    }
    let m_lat = crate::scene::DEG_M;
    // The material is on the left for both ring kinds (see above), so there is no
    // per-kind sign: the offset direction is the left normal either way.
    let sign = 1.0f64;
    let mut out = Vec::with_capacity(n);
    for k in 0..n {
        let prev = ring.pts[(k + n - 1) % n];
        let cur = ring.pts[k];
        let next = ring.pts[(k + 1) % n];
        let e0 = unit(cur, prev, m_lon, m_lat).map(|(e, nn)| (-e, -nn));
        let e1 = unit(cur, next, m_lon, m_lat);
        let (Some((e0e, e0n)), Some((e1e, e1n))) = (e0, e1) else {
            out.push(cur);
            continue;
        };
        // A vertex between two tile cuts must not move at all, and one between a
        // cut and a silhouette may only slide *along* the cut — otherwise the
        // interior would pull away from the border and open a gap the neighbour
        // does not have.
        let in_cut = ring.cut[(k + n - 1) % n];
        let out_cut = ring.cut[k];
        if in_cut && out_cut {
            out.push(cur);
            continue;
        }
        // Inward normal of each edge, averaged, with the miter scale.
        let (n0e, n0n) = (-e0n * sign, e0e * sign);
        let (n1e, n1n) = (-e1n * sign, e1e * sign);
        let (se, sn) = (n0e + n1e, n0n + n1n);
        let len = (se * se + sn * sn).sqrt();
        if len < 1e-9 {
            out.push(cur);
            continue;
        }
        let scale = (1.0 / (len * 0.5).min(1.0)).min(MITER_MAX);
        let mut de = se / len * priors::PAVE_RIM_M * scale;
        let mut dn = sn / len * priors::PAVE_RIM_M * scale;
        if in_cut || out_cut {
            // Project onto the cut edge's direction so the vertex stays on the
            // border line.
            let (ce, cn) = if in_cut { (e0e, e0n) } else { (e1e, e1n) };
            let t = de * ce + dn * cn;
            de = ce * t;
            dn = cn * t;
        }
        out.push(Coord { x: cur.x + de / m_lon, y: cur.y + dn / m_lat });
    }
    // Reject a fold: the offset must keep the ring's orientation, and it must
    // shrink the *region* — which for an outer ring means a smaller area and for a
    // hole a larger one. Testing "area decreased" for both, as this first did,
    // rejected every correctly-offset hole and so silently dropped their rims.
    let a0 = signed_area(&ring.pts, m_lon);
    let a1 = signed_area(&out, m_lon);
    if a1 == 0.0 || a0.signum() != a1.signum() {
        return None;
    }
    let shrank_region = if ring.outer { a1.abs() < a0.abs() } else { a1.abs() > a0.abs() };
    if !shrank_region {
        return None;
    }
    Some(out)
}

/// Below this the kerb and the ground beside it are the same surface and the
/// wall between them is not worth two triangles. A real kerb is about a
/// quarter-metre; the boundary also carries quantization and the odd centimetre
/// of interpolation, so this is set where "a kerb" stops and "a wall" starts.
const APRON_MIN_M: f64 = 0.25;

/// The wall between the drawn asphalt and the drawn ground, as one vertical
/// quad per silhouette edge.
///
/// The terrain stops at the kerb and its rim vertex takes the *ground's* height
/// there, not the road's. Where a bench holds, those are the same number and
/// this emits nothing. Where none does — a hairpin stacked over itself, a road
/// the earthwork declined to bench on a steep flank — they differ by whatever
/// the model failed to build, and that difference used to be either a smear of
/// terrain pulled up to road height across the first lattice cell or, once the
/// ground was cut away, a gap you could see the hillside through.
///
/// Drawn instead as what it is: a vertical face between the kerb and the
/// ground, with its own normals and its own class. Fifteen metres of it is a
/// retaining wall, which is what is physically there; a few centimetres is a
/// kerb and is skipped.
///
/// **Both ways.** The wall is drawn whichever surface is higher. A road on fill
/// stands above the ground and the face runs down to it; a road in a cutting
/// sits below and the face runs up to the ground it is cut into. Only the first
/// case was handled at first, and the second is not hypothetical — once the
/// carriageway rides its own profile rather than being clamped up to the
/// terrain (`road::on_ground`), every cutting became an open gap between the
/// asphalt and the terrain's rim, and the sky showed through it.
///
/// Cut edges carry no apron. A cut is a tile border, where the asphalt
/// continues into the neighbour and there is no kerb at all — walling it would
/// build a fence down every tile edge.
fn build_apron(
    rings: &[TaggedRing],
    bounds: &Bounds,
    at: &mut dyn FnMut(f64, f64) -> (i32, i32),
) -> Option<TerrainMesh> {
    let mut mesh = TerrainMesh {
        x: Vec::new(),
        y: Vec::new(),
        z: Vec::new(),
        indices: Vec::new(),
        normals: Vec::new(),
        edge_across: Vec::new(),
    };
    let min_mm = (APRON_MIN_M * 1000.0) as i32;
    for r in rings {
        let n = r.pts.len();
        if n < 3 {
            continue;
        }
        // Heights at the *rounded* position, which is where both the asphalt
        // and the terrain put their vertices.
        let mut sample = |c: &Coord| -> ((u16, u16), i32, i32) {
            let qx = project::quantize_x(c.x, bounds);
            let qy = project::quantize_y(c.y, bounds);
            let (road, ground) =
                at(project::dequantize_x(qx, bounds), project::dequantize_y(qy, bounds));
            ((qx, qy), road, ground)
        };
        for k in 0..n {
            if r.cut[k] {
                continue;
            }
            let k1 = (k + 1) % n;
            let (qa, road_a, gnd_a) = sample(&r.pts[k]);
            let (qb, road_b, gnd_b) = sample(&r.pts[k1]);
            if qa == qb {
                continue;
            }
            // Only where the two surfaces actually part company. Either end
            // parting is enough: a wall that tapers to nothing at one end is a
            // wall, and skipping it would leave that end open.
            if (road_a - gnd_a).abs().max((road_b - gnd_b).abs()) < min_mm {
                continue;
            }
            let base = mesh.x.len() as u32;
            for (q, z) in [(qa, road_a), (qb, road_b), (qb, gnd_b), (qa, gnd_a)] {
                mesh.x.push(q.0);
                mesh.y.push(q.1);
                mesh.z.push(z);
                // A vertical face has no meaningful octahedral "up"; the client
                // draws this cull-none and lights it flat, and a wall reading as
                // unlit is right for a wall.
                mesh.normals.push(0);
                mesh.normals.push(0);
            }
            mesh.indices.extend_from_slice(&[base, base + 1, base + 2]);
            mesh.indices.extend_from_slice(&[base, base + 2, base + 3]);
        }
    }
    (!mesh.indices.is_empty()).then_some(mesh)
}

/// Cap on the miter scale, matching the band this replaces.
const MITER_MAX: f64 = 1.5;

/// How far a boundary edge's midpoint may lie from a handover cut, in metres,
/// and still be that cut.
///
/// The band's run ends at the span's exact arc and the cut is drawn on the same
/// smoothed line from the same station, so a boundary vertex sits *on* it and
/// the tolerance only has to cover what happens afterwards: the union's own
/// coordinate grid, the boundary simplification (which removes vertices but
/// never moves them), and the quantized frame the rings are read back in.
const CUT_NEAR_M: f64 = 0.5;

/// How far a boundary edge's direction may turn from the cut's and still be
/// part of it, in radians. Loose, because the cut is one straight line and the
/// boundary crossing it may be simplified into a slightly different chord;
/// tight enough to reject the *kerb* edges that meet the cut at its corners,
/// which run across it at a right angle.
const CUT_PARALLEL_RAD: f64 = 0.5;

/// Whether a boundary edge is the cut where a structure takes over, rather than
/// a kerb. Both are silhouette; only one of them is an edge of anything.
fn is_handover(a: Coord, b: Coord, handovers: &[Handover], m_lon: f64) -> bool {
    let mid = Coord { x: 0.5 * (a.x + b.x), y: 0.5 * (a.y + b.y) };
    let (ex, ey) = ((b.x - a.x) * m_lon, (b.y - a.y) * DEG_M);
    let elen = (ex * ex + ey * ey).sqrt();
    if elen <= 0.0 {
        return false;
    }
    handovers.iter().any(|h| {
        let (dx, dy) = ((h.b.x - h.a.x) * m_lon, (h.b.y - h.a.y) * DEG_M);
        let len2 = dx * dx + dy * dy;
        if len2 <= 0.0 {
            return false;
        }
        // Parallel first: it is the cheap half and it is what separates the cut
        // from the kerb running into its corner.
        let cos = ((ex * dx + ey * dy) / (elen * len2.sqrt())).abs();
        if cos < CUT_PARALLEL_RAD.cos() {
            return false;
        }
        let (qx, qy) = ((mid.x - h.a.x) * m_lon, (mid.y - h.a.y) * DEG_M);
        let t = ((qx * dx + qy * dy) / len2).clamp(0.0, 1.0);
        let (px, py) = (qx - dx * t, qy - dy * t);
        (px * px + py * py).sqrt() <= CUT_NEAR_M
    })
}

/// The rim: one quad per non-cut boundary edge, `edge_across` 127 on the
/// silhouette pair and 0 on the inset pair, so the client fades the outer pixel.
///
/// Returns `(casing, handover)` — the kerb rim, and the quads on handover cuts,
/// which the caller folds into the opaque surface instead. Either is `None`
/// when nothing of that kind was built.
#[allow(clippy::too_many_arguments)]
fn build_rim(
    rings: &[TaggedRing],
    insets: &[Option<Vec<Coord>>],
    bounds: &Bounds,
    up: (i8, i8),
    hole: bool,
    height: &mut dyn FnMut(f64, f64) -> i32,
    handovers: &[Handover],
) -> (Option<TerrainMesh>, Option<TerrainMesh>) {
    let empty = || TerrainMesh {
        x: Vec::new(),
        y: Vec::new(),
        z: Vec::new(),
        indices: Vec::new(),
        normals: Vec::new(),
        edge_across: Vec::new(),
    };
    let m_lon = crate::scene::DEG_M
        * ((bounds.south + bounds.north) * 0.5).to_radians().cos();
    let mut hand = empty();
    let mut mesh = empty();
    for (r, inset) in rings.iter().zip(insets) {
        let Some(inset) = inset else { continue };
        let n = r.pts.len();
        for k in 0..n {
            if r.cut[k] {
                continue; // a tile border carries no fade
            }
            let k1 = (k + 1) % n;
            let quad = [r.pts[k], r.pts[k1], inset[k1], inset[k]];

            // Validate the quad *as it will be emitted*, in quantized space.
            //
            // At a sharp spike the mitered inset can overshoot past the edge it
            // belongs to, folding the quad into a bowtie — and drawn in the
            // casing's darker tone a bowtie reads as a small dark spur poking out
            // of the asphalt. Checking the offset geometry in degrees is not
            // enough: quantization to u16 can collapse a thin quad or flip its
            // orientation on its own, so a quad that is well-formed in metres can
            // still reach the renderer inverted. Testing the rounded coordinates
            // catches both causes at once.
            //
            // Dropping a quad costs that one edge its antialiasing, which is
            // invisible; drawing it costs a visible artifact.
            let q: Vec<(i64, i64)> = quad
                .iter()
                .map(|c| {
                    (
                        project::quantize_x(c.x, bounds) as i64,
                        project::quantize_y(c.y, bounds) as i64,
                    )
                })
                .collect();
            let tri = |a: usize, b: usize, c: usize| {
                (q[b].0 - q[a].0) * (q[c].1 - q[a].1) - (q[c].0 - q[a].0) * (q[b].1 - q[a].1)
            };
            let (t0, t1) = (tri(0, 1, 2), tri(0, 2, 3));
            if t0 == 0 || t1 == 0 || (t0 > 0) != (t1 > 0) {
                continue;
            }
            // The analytic fade exists because the asphalt overlapped coplanar
            // ground with a depth bias: a floating edge with no geometric
            // silhouette for MSAA to resolve. Where the ground is cut away
            // there is nothing underneath to blend *into* but sky, and a
            // half-alpha kerb against sky is a background halo along every road
            // in the map. With the hole the asphalt boundary and the terrain
            // boundary are the same edge at the same heights, drawn by two
            // adjacent opaque meshes, which MSAA resolves per-sample for free.
            // So the rim keeps its geometry and its darker tone and loses only
            // the sub-pixel fade (docs/ROADS.md §6.1).
            let across = if hole { [0i8; 4] } else { [127i8, 127, 0, 0] };
            // A handover quad goes to the surface instead, and there it is
            // interior: opaque, no across-coordinate, no tone of its own.
            let handover = is_handover(r.pts[k], r.pts[k1], handovers, m_lon);
            let out = if handover { &mut hand } else { &mut mesh };
            let base = out.x.len() as u32;
            for ((_, a), qc) in quad.iter().zip(across).zip(&q) {
                out.x.push(qc.0 as u16);
                out.y.push(qc.1 as u16);
                // Sampled at the *rounded* position, which is where the vertex
                // is emitted and where the interior mesh samples its own. The
                // two used to disagree by a sub-quantum amount, an invisible
                // hairline between casing and surface — and once the terrain
                // seams to these heights it would stop being invisible.
                out.z.push(height(
                    project::dequantize_x(qc.0 as u16, bounds),
                    project::dequantize_y(qc.1 as u16, bounds),
                ));
                out.normals.push(up.0);
                out.normals.push(up.1);
                if !handover {
                    out.edge_across.push(a);
                }
            }
            // Two triangles; winding is fixed up by the client's cull-none
            // pipeline, but keep them consistent with the ring's own order.
            out.indices.extend_from_slice(&[base, base + 1, base + 2]);
            out.indices.extend_from_slice(&[base, base + 2, base + 3]);
        }
    }
    (
        (!mesh.indices.is_empty()).then_some(mesh),
        (!hand.indices.is_empty()).then_some(hand),
    )
}

/// Appends `add` to `into`, offsetting its indices. Both must agree on whether
/// they carry an across-coordinate — an empty `edge_across` means "no analytic
/// AA" for the whole mesh, so a half-filled one would silently misalign with
/// its vertices at the decoder.
fn append_mesh(into: &mut TerrainMesh, add: TerrainMesh) {
    debug_assert!(
        into.edge_across.is_empty() && add.edge_across.is_empty(),
        "merging meshes that disagree about analytic AA"
    );
    let base = into.x.len() as u32;
    into.x.extend_from_slice(&add.x);
    into.y.extend_from_slice(&add.y);
    into.z.extend_from_slice(&add.z);
    into.normals.extend_from_slice(&add.normals);
    into.indices.extend(add.indices.iter().map(|i| i + base));
}



/// Unit ENU direction from `a` to `b`, or `None` for a degenerate step.
fn unit(a: Coord, b: Coord, m_lon: f64, m_lat: f64) -> Option<(f64, f64)> {
    let (de, dn) = ((b.x - a.x) * m_lon, (b.y - a.y) * m_lat);
    let len = (de * de + dn * dn).sqrt();
    (len > 1e-9).then(|| (de / len, dn / len))
}

/// Twice the signed area of a ring in metre-ish units — only its sign and
/// relative magnitude are used.
fn signed_area(ring: &[Coord], m_lon: f64) -> f64 {
    let m_lat = crate::scene::DEG_M;
    let mut acc = 0.0;
    for k in 0..ring.len() {
        let a = ring[k];
        let b = ring[(k + 1) % ring.len()];
        acc += (a.x * m_lon) * (b.y * m_lat) - (b.x * m_lon) * (a.y * m_lat);
    }
    acc * 0.5
}

#[cfg(test)]
mod tests {
    use super::*;

    const Z: u8 = 15;

    fn bounds() -> Bounds {
        crate::solve::tile_containing(Z, 6.0, 46.0)
    }

    /// A rectangle ring, counter-clockwise, inset `frac` of the tile from each side.
    fn box_ring(b: &Bounds, frac: f64) -> Vec<Coord> {
        let (w, h) = (b.width(), b.height());
        vec![
            Coord { x: b.west + frac * w, y: b.south + frac * h },
            Coord { x: b.east - frac * w, y: b.south + frac * h },
            Coord { x: b.east - frac * w, y: b.north - frac * h },
            Coord { x: b.west + frac * w, y: b.north - frac * h },
        ]
    }

    fn tagged(pts: Vec<Coord>, b: &Bounds, outer: bool) -> TaggedRing {
        let n = pts.len();
        let cut = (0..n).map(|k| is_cut(pts[k], pts[(k + 1) % n], b)).collect();
        TaggedRing { pts, cut, outer }
    }

    #[test]
    fn an_interior_ring_meshes_watertight_and_inside_itself() {
        let b = bounds();
        let ring = tagged(box_ring(&b, 0.25), &b, true);
        let (surface, casing, _) =
            mesh_rings(&[ring], &b, 1, false, &[], &mut |_, _| 1000).expect("a mesh");
        assert!(!surface.indices.is_empty());
        assert_eq!(surface.indices.len() % 3, 0);
        assert!(casing.is_some(), "an interior ring gets a full rim");

        // Watertight: no edge shared by more than two faces (the census
        // `terrain_cdt` uses).
        let mut uses: std::collections::HashMap<(u32, u32), u32> = Default::default();
        for t in surface.indices.chunks_exact(3) {
            for (a, c) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                let key = if a < c { (a, c) } else { (c, a) };
                *uses.entry(key).or_default() += 1;
            }
        }
        assert!(uses.values().all(|&u| u <= 2), "an edge is used more than twice");
    }

    /// The interior must sample the height field across the region, not chord
    /// from edge to edge. A ring meshed from its outline alone spans a
    /// carriageway with triangles as wide as the road, and on a cross-slope the
    /// terrain — sampled every lattice cell — surfaces straight through the
    /// asphalt. Meshed against the lattice, no drawn point of the surface may
    /// fall far below the field it is meshing.
    #[test]
    fn the_interior_tracks_the_field_instead_of_chording_across_it() {
        let b = bounds();
        // A field with a ridge across the middle of the tile, 10 m high: a
        // chord from one edge of the region to the other misses it entirely.
        let ridge = |lon: f64, _lat: f64| {
            let t = (lon - b.west) / b.width(); // 0..1 across the tile
            (10_000.0 * (1.0 - (2.0 * t - 1.0).abs())).round() as i32
        };
        let ring = || tagged(box_ring(&b, 0.25), &b, true);
        let (chorded, _, _) = mesh_rings(&[ring()], &b, 1, false, &[], &mut |lon, lat| ridge(lon, lat))
            .expect("a mesh");
        let (sampled, _, _) =
            mesh_rings(&[ring()], &b, crate::terrain::TERRAIN_GRID_DETAIL, false, &[], &mut |lon, lat| {
                ridge(lon, lat)
            })
            .expect("a mesh");
        assert!(
            sampled.x.len() > chorded.x.len() * 10,
            "the lattice must add interior samples ({} vs {})",
            sampled.x.len(),
            chorded.x.len()
        );

        // The worst gap between the drawn surface and the field, measured at
        // every vertex of the *other* mesh — where a chord is furthest from the
        // ridge it spans.
        let worst = |m: &TerrainMesh| {
            let mut worst = 0i32;
            for i in 0..m.x.len() {
                let lon = project::dequantize_x(m.x[i], &b);
                let lat = project::dequantize_y(m.y[i], &b);
                worst = worst.max((ridge(lon, lat) - m.z[i]).abs());
            }
            worst
        };
        assert_eq!(worst(&sampled), 0, "every vertex must carry its own field value");
        // The ridge crest is 10 m above the region's edges, so the chorded mesh
        // is metres wrong in between while the sampled one holds it.
        let crest = ridge((b.west + b.east) * 0.5, (b.south + b.north) * 0.5);
        let sampled_crest = sampled
            .z
            .iter()
            .copied()
            .max()
            .expect("a vertex");
        let chorded_crest = chorded.z.iter().copied().max().expect("a vertex");
        assert!(
            (crest - sampled_crest).abs() < 100,
            "the lattice-sampled surface must reach the crest ({sampled_crest} vs {crest})"
        );
        assert!(
            crest - chorded_crest > 4_000,
            "the outline-only surface must miss the crest by metres ({chorded_crest} vs {crest})"
        );
    }

    /// A boundary edge on a handover cut carries no kerb line: the road runs
    /// straight on across it onto the deck. The quad still has to be *drawn* —
    /// the interior is inset and something must cover the strip — so it moves
    /// into the opaque surface instead of into the casing, and it brings no
    /// across-coordinate with it.
    #[test]
    fn a_handover_edge_joins_the_surface_instead_of_the_casing() {
        let b = bounds();
        let ring = box_ring(&b, 0.25);
        // The cut runs along the ring's southern edge, a little past each end,
        // exactly as `junction::handover_cut` builds it.
        let (a, c) = (ring[0], ring[1]);
        let over = 0.1 * (c.x - a.x);
        let cut = [
            Coord { x: a.x - over, y: a.y },
            Coord { x: c.x + over, y: c.y },
        ];
        let tagged_ring = tagged(ring, &b, true);

        let plain = mesh_rings(&[tagged(box_ring(&b, 0.25), &b, true)], &b, 1, true, &[], &mut |_, _| 0);
        let (plain_surface, plain_casing, _) = plain.expect("a mesh");
        let handovers = [Handover { a: cut[0], b: cut[1] }];
        let (surface, casing, _) =
            mesh_rings(&[tagged_ring], &b, 1, true, &handovers, &mut |_, _| 0).expect("a mesh");
        let casing = casing.expect("three kerb edges still get a rim");
        let plain_casing = plain_casing.expect("a rim on all four edges");

        // One of the four rim quads left the casing…
        assert_eq!(
            plain_casing.indices.len() - casing.indices.len(),
            6,
            "exactly one quad should have left the casing"
        );
        // …and arrived in the surface, which has grown by that quad and still
        // carries no analytic AA.
        assert_eq!(
            surface.indices.len() - plain_surface.indices.len(),
            6,
            "the handover quad must be drawn by the surface"
        );
        assert!(surface.edge_across.is_empty(), "the surface fades nothing");
        assert_eq!(surface.z.len(), surface.x.len(), "the merged mesh stays consistent");
    }

    /// A cut is only the cut where the boundary actually runs *along* it. The
    /// kerb edges meeting it at its two corners cross it at a right angle, and
    /// stripping their casing would take the kerb line off the road for half a
    /// carriageway either side of every bridge.
    #[test]
    fn a_kerb_crossing_the_cut_keeps_its_casing() {
        let b = bounds();
        let ring = box_ring(&b, 0.25);
        // A cut through the ring's south-west corner, running north-south:
        // parallel to the western kerb, square across the southern one.
        let (a, c) = (ring[0], ring[3]);
        let handovers = [Handover { a, b: c }];
        let m_lon = crate::scene::DEG_M
            * ((b.south + b.north) * 0.5).to_radians().cos();
        assert!(
            is_handover(ring[3], ring[0], &handovers, m_lon),
            "the western kerb lies along the cut"
        );
        assert!(
            !is_handover(ring[0], ring[1], &handovers, m_lon),
            "the southern kerb only meets the cut at its corner"
        );
    }

    /// The silhouette is split at every lattice line it crosses, so no stretch
    /// of it spans more than one cell of the ground drawn under it — the
    /// scallops a chorded edge lets the hillside eat out of a road. Border runs
    /// keep their cut flag through the split, and the split points are the
    /// global lattice, so a neighbour clipping the same ring derives the same
    /// seam vertices.
    #[test]
    fn the_silhouette_is_split_at_every_cell_it_crosses() {
        let b = bounds();
        let grid = crate::terrain::TERRAIN_GRID_DETAIL;
        let ring = tagged(box_ring(&b, 0.25), &b, true);
        let dense = densify_ring(&ring, &b, grid);
        assert_eq!(dense.pts.len(), dense.cut.len(), "a flag per edge");

        // No edge of the densified ring spans more than one cell in either
        // axis: every crossing became a vertex.
        let (cw, ch) = (b.width() / grid as f64, b.height() / grid as f64);
        let cell = |c: Coord| {
            (((c.x - b.west) / cw).floor() as i64, ((c.y - b.south) / ch).floor() as i64)
        };
        for k in 0..dense.pts.len() {
            let (p, q) = (dense.pts[k], dense.pts[(k + 1) % dense.pts.len()]);
            let (pc, qc) = (cell(p), cell(q));
            assert!(
                (pc.0 - qc.0).abs() <= 1 && (pc.1 - qc.1).abs() <= 1,
                "edge {k} jumps from cell {pc:?} to {qc:?}"
            );
        }
        // The ring still bounds the same area — densifying adds vertices, never
        // moves the boundary.
        let m_lon = crate::scene::DEG_M
            * ((b.south + b.north) * 0.5).to_radians().cos();
        let (a0, a1) = (signed_area(&ring.pts, m_lon), signed_area(&dense.pts, m_lon));
        assert!((a0 - a1).abs() / a0.abs() < 1e-9, "the boundary moved: {a0} vs {a1}");

        // A ring clipped to the tile border: every piece of a cut run stays a
        // cut, so the seam still grows no rim.
        let border = tagged(
            vec![
                Coord { x: b.west, y: b.south + 0.25 * b.height() },
                Coord { x: b.west, y: b.south + 0.75 * b.height() },
                Coord { x: b.west + 0.5 * b.width(), y: b.south + 0.5 * b.height() },
            ],
            &b,
            true,
        );
        let dense = densify_ring(&border, &b, grid);
        for k in 0..dense.pts.len() {
            let (p, q) = (dense.pts[k], dense.pts[(k + 1) % dense.pts.len()]);
            assert_eq!(
                dense.cut[k],
                is_cut(p, q, &b),
                "edge {k} lost its cut flag through the split"
            );
        }
    }

    #[test]
    fn the_rim_carries_across_only_on_the_silhouette() {
        let b = bounds();
        let ring = tagged(box_ring(&b, 0.25), &b, true);
        let (surface, casing, _) = mesh_rings(&[ring], &b, 1, false, &[], &mut |_, _| 0).expect("a mesh");
        // The interior declares no across-coordinate at all: it is opaque.
        assert!(surface.edge_across.is_empty());
        let rim = casing.expect("a rim");
        assert_eq!(rim.edge_across.len(), rim.x.len());
        let outer = rim.edge_across.iter().filter(|&&a| a == 127).count();
        let inner = rim.edge_across.iter().filter(|&&a| a == 0).count();
        assert_eq!(outer, inner, "every rim quad pairs a silhouette and an inset vertex");
        assert!(rim.edge_across.iter().all(|&a| a == 127 || a == 0), "stray across value");
    }

    #[test]
    fn a_tile_border_edge_carries_no_rim() {
        // A ring flush against the tile's western edge: that edge is a cut, so it
        // must produce no rim quad, and no rim vertex may sit on it.
        let b = bounds();
        let (w, h) = (b.width(), b.height());
        let pts = vec![
            Coord { x: b.west, y: b.south + 0.25 * h },
            Coord { x: b.west + 0.5 * w, y: b.south + 0.25 * h },
            Coord { x: b.west + 0.5 * w, y: b.north - 0.25 * h },
            Coord { x: b.west, y: b.north - 0.25 * h },
        ];
        let ring = tagged(pts, &b, true);
        assert_eq!(ring.cut, vec![false, false, false, true], "the west edge is the cut");
        let (_, casing, _) = mesh_rings(&[ring], &b, 1, false, &[], &mut |_, _| 0).expect("a mesh");
        let rim = casing.expect("a rim on the three real edges");
        // Three real edges, one quad each — the cut edge contributed none.
        assert_eq!(rim.indices.len(), 3 * 6, "expected three rim quads");
        assert_eq!(rim.x.len(), 3 * 4);

        // The property that matters is that no rim runs *along* the border. A
        // corner where a genuine silhouette meets the border does fade, and must:
        // the asphalt really does end there. What would draw a line down the tile
        // seam is a quad whose whole silhouette edge lies on it.
        let qwest = project::quantize_x(b.west, &b);
        for quad in rim.x.chunks_exact(4) {
            let (o0, o1) = (quad[0], quad[1]); // the silhouette pair
            assert!(
                !(o0 == qwest && o1 == qwest),
                "a rim quad runs along the tile border"
            );
        }
        // And the inset never leaves the border: a vertex pinned to the cut line
        // stays on it, so the interior still reaches the seam.
        for quad in rim.x.chunks_exact(4) {
            if quad[0] == qwest {
                assert_eq!(quad[3], qwest, "the inset pulled away from the border");
            }
            if quad[1] == qwest {
                assert_eq!(quad[2], qwest, "the inset pulled away from the border");
            }
        }
    }

    #[test]
    fn a_hole_is_not_meshed_over() {
        // A box with a concentric hole: no triangle centroid may fall in the hole.
        let b = bounds();
        let outer = tagged(box_ring(&b, 0.1), &b, true);
        let mut inner_pts = box_ring(&b, 0.35);
        inner_pts.reverse(); // holes wind the other way
        let inner = tagged(inner_pts.clone(), &b, false);
        let (surface, _, _) = mesh_rings(&[outer, inner], &b, 1, false, &[], &mut |_, _| 0).expect("a mesh");

        let hole: Vec<Coord> = inner_pts;
        let hole_q: Vec<(f64, f64)> = hole
            .iter()
            .map(|c| {
                (project::quantize_x(c.x, &b) as f64, project::quantize_y(c.y, &b) as f64)
            })
            .collect();
        let mut in_hole = 0;
        for t in surface.indices.chunks_exact(3) {
            let cen = (
                (surface.x[t[0] as usize] as f64
                    + surface.x[t[1] as usize] as f64
                    + surface.x[t[2] as usize] as f64)
                    / 3.0,
                (surface.y[t[0] as usize] as f64
                    + surface.y[t[1] as usize] as f64
                    + surface.y[t[2] as usize] as f64)
                    / 3.0,
            );
            if point_in(&hole_q, cen) {
                in_hole += 1;
            }
        }
        assert_eq!(in_hole, 0, "{in_hole} triangles cover the island");
    }

    /// Standalone even-odd test for the assertions above.
    fn point_in(ring: &[(f64, f64)], p: (f64, f64)) -> bool {
        let mut inside = false;
        for k in 0..ring.len() {
            let (x0, y0) = ring[k];
            let (x1, y1) = ring[(k + 1) % ring.len()];
            if (y0 > p.1) != (y1 > p.1) {
                let t = (p.1 - y0) / (y1 - y0);
                if p.0 < x0 + t * (x1 - x0) {
                    inside = !inside;
                }
            }
        }
        inside
    }

    #[test]
    fn a_folded_rim_quad_is_dropped_rather_than_drawn() {
        // A ring with a needle-thin spike. The mitered inset at the spike's tip
        // overshoots past the edges it belongs to; those quads must be dropped,
        // not emitted as bowties — drawn in the casing's darker tone a bowtie
        // reads as a dark spur poking out of the asphalt.
        let b = bounds();
        let (w, h) = (b.width(), b.height());
        let (cx, cy) = (b.west + 0.5 * w, b.south + 0.5 * h);
        let pts = vec![
            Coord { x: cx - 0.2 * w, y: cy - 0.05 * h },
            Coord { x: cx + 0.2 * w, y: cy - 0.05 * h },
            // The spike: out and back through a very sharp turn.
            Coord { x: cx + 0.201 * w, y: cy + 0.30 * h },
            Coord { x: cx + 0.199 * w, y: cy - 0.049 * h },
            Coord { x: cx - 0.2 * w, y: cy + 0.05 * h },
        ];
        let ring = tagged(pts, &b, true);
        let Some((_, casing, _)) = mesh_rings(&[ring], &b, 1, false, &[], &mut |_, _| 0) else {
            return; // degenerating entirely is also acceptable
        };
        let Some(rim) = casing else { return };
        // Every surviving quad is a proper one: its inset pair is on the material
        // side of its silhouette pair, so its two triangles wind the same way.
        for q in rim.x.chunks_exact(4).zip(rim.y.chunks_exact(4)) {
            let (xs, ys) = q;
            let p = |i: usize| (xs[i] as f64, ys[i] as f64);
            let cross = |a: (f64, f64), c: (f64, f64), d: (f64, f64)| {
                (c.0 - a.0) * (d.1 - a.1) - (d.0 - a.0) * (c.1 - a.1)
            };
            let t0 = cross(p(0), p(1), p(2));
            let t1 = cross(p(0), p(2), p(3));
            assert!(
                t0 == 0.0 || t1 == 0.0 || t0.signum() == t1.signum(),
                "a rim quad is a bowtie: {t0} vs {t1}"
            );
        }
    }

    #[test]
    fn a_sliver_ring_degrades_to_no_rim() {
        // A ring narrower than twice the rim cannot be inset: the mesher must
        // still produce a surface, just without the fade.
        let b = bounds();
        let m_lat = crate::scene::DEG_M;
        let thin = 0.3 * priors::PAVE_RIM_M; // well under 2 x the rim
        let pts = vec![
            Coord { x: b.west + 0.2 * b.width(), y: b.south + 0.5 * b.height() },
            Coord { x: b.east - 0.2 * b.width(), y: b.south + 0.5 * b.height() },
            Coord { x: b.east - 0.2 * b.width(), y: b.south + 0.5 * b.height() + thin / m_lat },
            Coord { x: b.west + 0.2 * b.width(), y: b.south + 0.5 * b.height() + thin / m_lat },
        ];
        let ring = tagged(pts, &b, true);
        assert!(inset_ring(&ring, 77000.0).is_none(), "a sliver must not inset");
        let meshed = mesh_rings(&[ring], &b, 1, false, &[], &mut |_, _| 0);
        if let Some((surface, casing, _)) = meshed {
            assert!(casing.is_none(), "a sliver must not carry a rim");
            assert!(surface.edge_across.is_empty());
        }
    }

    #[test]
    fn the_inset_shrinks_the_region_for_both_ring_kinds() {
        // The bug this pins down: a hole must be offset *outward*, growing it, so
        // the interior stops short of the island rather than spilling into it —
        // and it must still get a rim, which an "area always decreases" guard
        // silently denied it.
        let b = bounds();
        let m_lon = crate::scene::DEG_M
            * ((b.south + b.north) * 0.5).to_radians().cos();

        let outer = tagged(box_ring(&b, 0.1), &b, true);
        let out_inset = inset_ring(&outer, m_lon).expect("an outer ring insets");
        let (a0, a1) = (signed_area(&outer.pts, m_lon), signed_area(&out_inset, m_lon));
        assert_eq!(a0.signum(), a1.signum(), "the inset flipped the outer winding");
        assert!(a1.abs() < a0.abs(), "the outer ring did not shrink");

        let mut hole_pts = box_ring(&b, 0.35);
        hole_pts.reverse(); // a hole winds the other way
        let hole = tagged(hole_pts, &b, false);
        let hole_inset = inset_ring(&hole, m_lon).expect("a hole ring insets");
        let (h0, h1) = (signed_area(&hole.pts, m_lon), signed_area(&hole_inset, m_lon));
        assert_eq!(h0.signum(), h1.signum(), "the inset flipped the hole winding");
        assert!(
            h1.abs() > h0.abs(),
            "the hole shrank ({:.3} -> {:.3}); it must grow so the interior keeps clear of it",
            h0.abs(),
            h1.abs()
        );

        // And a region with a hole produces rim quads for both boundaries.
        let (_, casing, _) = mesh_rings(&[outer, hole], &b, 1, false, &[], &mut |_, _| 0).expect("a mesh");
        let rim = casing.expect("a rim");
        let quads = rim.x.len() / 4;
        assert!(quads >= 8, "expected rim on both rings' four sides, got {quads} quads");
    }

    #[test]
    fn simplification_thins_the_interior_and_never_the_seam() {
        let b = bounds();
        let (w, h) = (b.width(), b.height());
        // A ring flush against the west border, with a finely sampled wobble
        // along its southern edge and extra vertices along the border itself.
        let mut pts = vec![Coord { x: b.west, y: b.south + 0.2 * h }];
        for i in 0..=200 {
            let t = i as f64 / 200.0;
            pts.push(Coord {
                x: b.west + t * 0.6 * w,
                // A wobble far below the zoom's tolerance: pure noise.
                y: b.south + 0.2 * h + (t * 60.0).sin() * 1e-9,
            });
        }
        pts.push(Coord { x: b.west + 0.6 * w, y: b.north - 0.2 * h });
        // Intermediate vertices *on* the border: the seam the neighbour shares.
        for i in (1..5).rev() {
            pts.push(Coord { x: b.west, y: b.south + 0.2 * h + i as f64 * 0.1 * h });
        }
        let ring = tagged(pts, &b, true);
        let seam_before = ring.cut.iter().filter(|&&c| c).count();
        assert!(seam_before >= 4, "the fixture needs a multi-edge seam run");

        let tol = crate::pipeline::tolerance(Z);
        let simple = simplify_ring(&ring, tol);

        // The noisy interior collapses...
        assert!(
            simple.pts.len() < ring.pts.len() / 4,
            "interior barely thinned: {} -> {}",
            ring.pts.len(),
            simple.pts.len()
        );
        // ...and every seam edge survives, because the neighbour has them too.
        assert_eq!(
            simple.cut.iter().filter(|&&c| c).count(),
            seam_before,
            "a tile-border edge was simplified away"
        );
        // Every vertex that was on the border still is.
        let on_border = |r: &TaggedRing| r.pts.iter().filter(|c| c.x == b.west).count();
        assert_eq!(on_border(&simple), on_border(&ring), "a seam vertex moved off the border");
        // The flags still describe the edges they are paired with.
        assert_eq!(simple.pts.len(), simple.cut.len());
        for k in 0..simple.pts.len() {
            let (a, c) = (simple.pts[k], simple.pts[(k + 1) % simple.pts.len()]);
            assert_eq!(simple.cut[k], is_cut(a, c, &b), "flag {k} no longer matches its edge");
        }
    }

    #[test]
    fn degenerate_rings_yield_no_mesh() {
        let b = bounds();
        assert!(mesh_rings(&[], &b, 1, false, &[], &mut |_, _| 0).is_none(), "no rings");
        let p = Coord { x: b.west + 0.5 * b.width(), y: b.south + 0.5 * b.height() };
        let dot = tagged(vec![p, p, p], &b, true);
        assert!(mesh_rings(&[dot], &b, 1, false, &[], &mut |_, _| 0).is_none(), "a degenerate ring");
    }
}
