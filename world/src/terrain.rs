//! The terrain: a mesh over the bbox, one height per vertex from the DEM.
//!
//! A regular lattice, two triangles per cell. Nothing engineered — no
//! benches, no breaklines — just the ground the next steps stand on, in a
//! form whose surface can be evaluated analytically
//! ([`crate::lattice::height_at`]) so a later layer can be *proven* to lie
//! on it rather than looked at.

use arpentry_server::dem::Dem;

use crate::frame::Extent;
use crate::grid::Grid;
use crate::step::Summary;
use crate::world::Terrain;

/// The DEM zoom every height is *asked* for. One fixed zoom means one ground,
/// whatever the bbox size.
///
/// It is not necessarily the zoom that answers: [`Dem`] clamps the request to
/// the range its archive carries, and the clamp is silent. z16 is ~0.8 m per
/// pixel near 46°, but an extract is only as fine as the source it was cut
/// from, and may top out at z14, ~3.3 m per pixel. So the run reports
/// `dem_z`: what the ground can actually resolve, beside what the lattice was
/// built at. Where the two disagree by much, the extra vertices are
/// interpolation.
const ZOOM: u8 = 16;

/// Builds the terrain layer at about `spacing` metres, at most `cap`
/// vertices (see [`Grid::fit`]).
pub fn run(extent: &Extent, dem: &mut Dem, spacing: f64, cap: usize) -> (Terrain, Summary) {
    let grid = Grid::fit(&extent.rect, spacing, cap);
    let served = dem.served_zoom(ZOOM);
    let terrain = build(&grid, &mut |x, y| {
        let (lon, lat) = extent.frame.to_geo(x, y);
        dem.elevation(lon, lat, ZOOM)
    });
    let summary = Summary::new()
        .with("vertices", terrain.grid.vertex_count())
        .with("cells", format!("{}x{}", grid.cols, grid.rows))
        .with("spacing", format!("{:.2}x{:.2}", grid.dx, grid.dy))
        .with("dem_z", dem_zoom(served, extent, &grid))
        .with("zmin", format!("{:.1}", terrain.zmin))
        .with("zmax", format!("{:.1}", terrain.zmax));
    (terrain, summary)
}

/// How the run reports the ground's own resolution: the zoom served, its
/// pixel pitch in metres at this latitude, and a `!` when the request was
/// clamped — the lattice is then finer than anything the DEM can say.
/// A field answers everywhere at every zoom, and reports as `field`.
fn dem_zoom(served: Option<u8>, extent: &Extent, grid: &Grid) -> String {
    let Some(z) = served else {
        return "field".to_string();
    };
    // The pitch of one 512 px Terrarium tile's pixel at the world's latitude.
    let (_, lat) = extent.frame.to_geo(0.0, 0.0);
    let pitch = 40_075_016.7 * lat.to_radians().cos() / ((1u64 << z as u32) as f64 * 512.0);
    let clamped = if z < ZOOM { "!" } else { "" };
    let coarse = if pitch > 1.5 * grid.dx.max(grid.dy) { " coarser-than-lattice" } else { "" };
    format!("z{z}{clamped}/{pitch:.2}m{coarse}")
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

#[cfg(test)]
pub(crate) mod tests {
    use std::path::Path;

    use super::*;
    use crate::lattice::height_at;
    use crate::pipeline::tests::bbox;

    /// The extent of the box the specimens are built in.
    pub(crate) fn extent() -> Extent {
        Extent::of(bbox())
    }

    /// A synthetic ground with its origin at the test bbox's centre.
    pub(crate) fn dem(spec: &str) -> Dem {
        let sep = if spec.contains('?') { '&' } else { '?' };
        Dem::open(Path::new(&format!("{spec}{sep}at=6.92,46.435"))).expect("spec parses")
    }

    #[test]
    fn flat_is_flat() {
        let (t, s) = run(&extent(), &mut dem("flat?h=400"), 10.0, usize::MAX);
        let t = &t;
        assert!(t.z.iter().all(|&z| z == 400.0));
        assert!(t.normals.iter().all(|&n| n == [0.0, 0.0, 1.0]));
        assert_eq!((t.zmin, t.zmax), (400.0, 400.0));
        assert_eq!(t.indices.len(), t.grid.cell_count() * 6);
        assert!(s.to_string().contains("zmin=400.0"), "{s}");
    }

    #[test]
    fn ramp_matches_the_frame() {
        let (t, _) = run(&extent(), &mut dem("ramp?grade=0.03&bearing=90&radius=100000"), 5.0, usize::MAX);
        let t = &t;
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
        let (t, _) = run(&extent(), &mut dem("flat"), 2.0, 500);
        let t = &t;
        assert!(t.grid.vertex_count() <= 500);
        let [x, y] = t.grid.vertex(t.grid.cols, t.grid.rows);
        let rect = extent().rect;
        assert!((x - rect.x1).abs() < 1e-9 && (y - rect.y1).abs() < 1e-9);
    }

    #[test]
    fn height_at_matches_the_vertices() {
        let (t, _) = run(&extent(), &mut dem("hill?amp=60&radius=400"), 10.0, usize::MAX);
        let t = &t;
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
        let (t, _) = run(&extent(), &mut dem("ramp?grade=0.03&bearing=90&radius=100000"), 10.0, usize::MAX);
        let t = &t;
        let [x1, y1] = t.grid.vertex(t.grid.cols, t.grid.rows);
        // East of the grid the ramp stops rising; north of it nothing changes.
        assert_eq!(height_at(t, x1 + 50.0, 0.0), height_at(t, x1, 0.0));
        assert_eq!(height_at(t, 0.0, y1 + 50.0), height_at(t, 0.0, 0.0));
        assert_eq!(height_at(t, x1 + 5.0, y1 + 5.0), height_at(t, x1, y1));
        // And continuously: a step past the edge is no step.
        assert!((height_at(t, x1 + 1e-9, 12.3) - height_at(t, x1, 12.3)).abs() < 1e-9);
    }
}
