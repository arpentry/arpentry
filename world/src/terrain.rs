//! Step 1: the terrain mesh.
//!
//! A regular lattice over the bbox, one height per vertex from the DEM, two
//! triangles per cell. Nothing engineered yet — no benches, no breaklines —
//! just the ground the next steps stand on, in a form whose surface can be
//! evaluated analytically ([`height_at`]) so a later layer can be *proven* to
//! lie on it rather than looked at.

use arpentry_server::dem::Dem;

use crate::grid::Grid;
use crate::step::Summary;
use crate::world::{Terrain, World};

/// The DEM zoom every height is sampled at. Mapterhorn's z16 is ~0.8 m per
/// pixel near 46°, finer than the default 2 m lattice; one fixed zoom means
/// one ground, whatever the bbox size.
pub const ZOOM: u8 = 16;

/// Builds the terrain layer at about `spacing` metres, at most `cap`
/// vertices (see [`Grid::fit`]).
pub fn run(world: &mut World, dem: &mut Dem, spacing: f64, cap: usize) -> Summary {
    let grid = Grid::fit(&world.rect, spacing, cap);
    let terrain = build(&grid, &mut |x, y| {
        let (lon, lat) = world.frame.to_geo(x, y);
        dem.elevation(lon, lat, ZOOM)
    });
    let summary = Summary::new()
        .with("vertices", terrain.grid.vertex_count())
        .with("cells", format!("{}x{}", grid.cols, grid.rows))
        .with("spacing", format!("{:.2}x{:.2}", grid.dx, grid.dy))
        .with("zmin", format!("{:.1}", terrain.zmin))
        .with("zmax", format!("{:.1}", terrain.zmax));
    world.terrain = Some(terrain);
    summary
}

/// The mesh over `grid` with heights from `sample(x, y)` in local metres.
///
/// Samples one vertex past the grid on every side so the normals at the
/// boundary are the same centred differences as everywhere else.
pub fn build(grid: &Grid, sample: &mut dyn FnMut(f64, f64) -> f64) -> Terrain {
    let (cols, rows) = (grid.cols as usize, grid.rows as usize);
    let (pw, ph) = (cols + 3, rows + 3);
    let mut elev = vec![0.0; pw * ph];
    for pr in 0..ph {
        for pc in 0..pw {
            let [x, y] = grid.vertex_at(pc as f64 - 1.0, pr as f64 - 1.0);
            elev[pr * pw + pc] = sample(x, y);
        }
    }

    let n = grid.vertex_count();
    let mut z = Vec::with_capacity(n);
    let mut normals = Vec::with_capacity(n);
    let (mut zmin, mut zmax) = (f64::INFINITY, f64::NEG_INFINITY);
    for r in 0..=rows {
        for c in 0..=cols {
            let pi = (r + 1) * pw + (c + 1);
            let e = elev[pi];
            zmin = zmin.min(e);
            zmax = zmax.max(e);
            z.push(e);
            let dz_dx = (elev[pi + 1] - elev[pi - 1]) / (2.0 * grid.dx);
            let dz_dy = (elev[pi + pw] - elev[pi - pw]) / (2.0 * grid.dy);
            let len = (dz_dx * dz_dx + dz_dy * dz_dy + 1.0).sqrt();
            normals.push([(-dz_dx / len) as f32, (-dz_dy / len) as f32, (1.0 / len) as f32]);
        }
    }

    let mut indices = Vec::with_capacity(grid.cell_count() * 6);
    for r in 0..grid.rows {
        for c in 0..grid.cols {
            let i00 = grid.index(c, r) as u32;
            let i10 = grid.index(c + 1, r) as u32;
            let i11 = grid.index(c + 1, r + 1) as u32;
            let i01 = grid.index(c, r + 1) as u32;
            indices.extend_from_slice(&[i00, i10, i11, i00, i11, i01]);
        }
    }
    Terrain { grid: *grid, z, normals, indices, zmin, zmax }
}

/// The height of the mesh surface at local `(x, y)`: the plane of the triangle
/// the point falls in, with the cell split on its SW→NE diagonal. Beyond the
/// grid the ground holds its edge: the height of the nearest point of the
/// grid, which keeps the surface continuous there (an edge cell's plane
/// extended outward would jump at every column and row line), and keeps a
/// piece of paving that pokes past the bbox by its half-width planar.
///
/// This is the one definition of "on the ground" every later step is held to.
pub fn height_at(t: &Terrain, x: f64, y: f64) -> f64 {
    let (u, v) = t.grid.to_uv(x, y);
    let (u, v) = (u.clamp(0.0, t.grid.cols as f64), v.clamp(0.0, t.grid.rows as f64));
    let (c, r) = t.grid.cell(u, v);
    let (fu, fv) = (u - c as f64, v - r as f64);
    let z00 = t.z[t.grid.index(c, r)];
    let z10 = t.z[t.grid.index(c + 1, r)];
    let z11 = t.z[t.grid.index(c + 1, r + 1)];
    let z01 = t.z[t.grid.index(c, r + 1)];
    if fu >= fv {
        z00 + fu * (z10 - z00) + fv * (z11 - z10)
    } else {
        z00 + fv * (z01 - z00) + fu * (z11 - z01)
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use std::path::Path;

    use arpentry_server::project::Bounds;

    use super::*;

    /// The roundabout box: ~1.5 km × 1.1 km, centred on (6.92, 46.435).
    pub(crate) fn world() -> World {
        World::new(Bounds { west: 6.91, south: 46.43, east: 6.93, north: 46.44 })
    }

    /// A synthetic ground with its origin at the test bbox's centre.
    pub(crate) fn dem(spec: &str) -> Dem {
        let sep = if spec.contains('?') { '&' } else { '?' };
        Dem::open(Path::new(&format!("{spec}{sep}at=6.92,46.435"))).expect("spec parses")
    }

    #[test]
    fn flat_is_flat() {
        let mut w = world();
        let s = run(&mut w, &mut dem("flat?h=400"), 10.0, usize::MAX);
        let t = w.terrain.as_ref().unwrap();
        assert!(t.z.iter().all(|&z| z == 400.0));
        assert!(t.normals.iter().all(|&n| n == [0.0, 0.0, 1.0]));
        assert_eq!((t.zmin, t.zmax), (400.0, 400.0));
        assert_eq!(t.indices.len(), t.grid.cell_count() * 6);
        assert!(s.to_string().contains("zmin=400.0"), "{s}");
    }

    #[test]
    fn ramp_matches_the_frame() {
        let mut w = world();
        run(&mut w, &mut dem("ramp?grade=0.03&bearing=90&radius=100000"), 5.0, usize::MAX);
        let t = w.terrain.as_ref().unwrap();
        let want = [-0.03f64, 0.0, 1.0];
        let len = (1.0f64 + 0.03 * 0.03).sqrt();
        for i in 0..t.grid.vertex_count() {
            let [x, _, z] = t.position(i);
            assert!((z - (400.0 + 0.03 * x)).abs() < 1e-6, "vertex {i}: z {z} at x {x}");
            for k in 0..3 {
                assert!((t.normals[i][k] as f64 - want[k] / len).abs() < 1e-6, "normal {i}");
            }
        }
    }

    #[test]
    fn cap_clamps_and_spans() {
        let mut w = world();
        run(&mut w, &mut dem("flat"), 2.0, 500);
        let t = w.terrain.as_ref().unwrap();
        assert!(t.grid.vertex_count() <= 500);
        let [x, y] = t.grid.vertex(t.grid.cols, t.grid.rows);
        assert!((x - w.rect.x1).abs() < 1e-9 && (y - w.rect.y1).abs() < 1e-9);
    }

    #[test]
    fn height_at_matches_the_vertices() {
        let mut w = world();
        run(&mut w, &mut dem("hill?amp=60&radius=400"), 10.0, usize::MAX);
        let t = w.terrain.as_ref().unwrap();
        for i in 0..t.grid.vertex_count() {
            let [x, y, z] = t.position(i);
            let (c, r) = t.grid.vertex_of(i);
            let h = height_at(t, x, y);
            if c < t.grid.cols && r < t.grid.rows {
                assert_eq!(h, z, "interior vertex {i}");
            } else {
                assert!((h - z).abs() < 1e-9, "edge vertex {i}: {h} vs {z}");
            }
        }
        assert!(t.zmax > 450.0 && t.zmin == 400.0, "{} {}", t.zmin, t.zmax);
    }

    #[test]
    fn beyond_the_grid_the_ground_holds_its_edge() {
        let mut w = world();
        run(&mut w, &mut dem("ramp?grade=0.03&bearing=90&radius=100000"), 10.0, usize::MAX);
        let t = w.terrain.as_ref().unwrap();
        let [x1, y1] = t.grid.vertex(t.grid.cols, t.grid.rows);
        // East of the grid the ramp stops rising; north of it nothing changes.
        assert_eq!(height_at(t, x1 + 50.0, 0.0), height_at(t, x1, 0.0));
        assert_eq!(height_at(t, 0.0, y1 + 50.0), height_at(t, 0.0, 0.0));
        assert_eq!(height_at(t, x1 + 5.0, y1 + 5.0), height_at(t, x1, y1));
        // And continuously: a step past the edge is no step.
        assert!((height_at(t, x1 + 1e-9, 12.3) - height_at(t, x1, 12.3)).abs() < 1e-9);
    }
}
