//! The terrain lattice as a surface: the height at any point, and a line
//! laid onto it exactly.
//!
//! Every step reads the ground through these, so they live apart from the
//! terrain and drape *steps* that first use them: a step module is a step
//! and nothing else, and what several steps share is a module of its own.

use crate::grid::Grid;
use crate::world::{Polyline2, Polyline3, Terrain};

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

/// `line` on the surface of `terrain`: split so every piece lies inside one
/// triangle, every vertex at the mesh height.
pub fn drape(terrain: &Terrain, line: &[[f64; 2]]) -> Vec<[f64; 3]> {
    let lift = |p: [f64; 2]| [p[0], p[1], height_at(terrain, p[0], p[1])];
    let mut out: Vec<[f64; 3]> = Vec::new();
    let Some(&first) = line.first() else {
        return out;
    };
    out.push(lift(first));
    for pair in line.windows(2) {
        for p in split(&terrain.grid, pair[0], pair[1]) {
            let v = lift(p);
            let last = out[out.len() - 1];
            if (v[0] - last[0]).hypot(v[1] - last[1]) >= 1e-9 {
                out.push(v);
            }
        }
    }
    out
}

/// The points after `p` where `p→q` crosses a grid line or a cell diagonal,
/// in order, ending with `q` itself.
pub fn split(grid: &Grid, p: [f64; 2], q: [f64; 2]) -> Vec<[f64; 2]> {
    let (u0, v0) = grid.to_uv(p[0], p[1]);
    let (u1, v1) = grid.to_uv(q[0], q[1]);
    let mut ts: Vec<f64> = Vec::new();
    crossings(u0, u1, &mut ts);
    crossings(v0, v1, &mut ts);
    crossings(u0 - v0, u1 - v1, &mut ts);
    ts.sort_by(|a, b| a.partial_cmp(b).expect("finite"));
    let mut out = Vec::with_capacity(ts.len() + 1);
    let mut last = 0.0f64;
    for t in ts {
        if t <= 0.0 || t >= 1.0 || t - last < 1e-9 {
            continue;
        }
        last = t;
        out.push([p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t]);
    }
    out.push(q);
    out
}

/// The parameters `t` at which `a + (b − a)·t` crosses an integer.
fn crossings(a: f64, b: f64, ts: &mut Vec<f64>) {
    if a == b {
        return;
    }
    let (lo, hi) = (a.min(b), a.max(b));
    let (k0, k1) = (lo.ceil() as i64, hi.floor() as i64);
    for k in k0..=k1 {
        ts.push((k as f64 - a) / (b - a));
    }
}

/// Reads a `Polyline2` as the plan line it is; for tests and callers that
/// already hold lines in the frame.
pub fn drape_line(terrain: &Terrain, line: &Polyline2) -> Polyline3 {
    Polyline3 {
        id: line.id.clone(),
        class: line.class.clone(),
        subclass: line.subclass.clone(),
        width_m: line.width_m,
        pts: drape(terrain, &line.pts),
    }
}

#[cfg(test)]
mod tests {
    use crate::frame::Rect;
    use crate::grid::Grid;
    use crate::terrain::{self, tests::{dem, extent}};

    use super::*;

    /// A 5×5 lattice of 1 m cells with a hill on it.
    fn hill() -> Terrain {
        let grid = Grid::fit(&Rect { x0: 0.0, y0: 0.0, x1: 5.0, y1: 5.0 }, 1.0, usize::MAX);
        terrain::build(&grid, &mut |x, y| {
            let d = (x - 2.5).hypot(y - 2.5);
            3.0 * (1.0 + (d * 1.3).cos()) + 0.1 * x * y
        })
    }

    /// Every point along every piece is on the surface.
    fn assert_on_surface(t: &Terrain, pts: &[[f64; 3]]) {
        for pair in pts.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            for k in 0..=10 {
                let s = k as f64 / 10.0;
                let x = a[0] + (b[0] - a[0]) * s;
                let y = a[1] + (b[1] - a[1]) * s;
                let z = a[2] + (b[2] - a[2]) * s;
                let h = height_at(t, x, y);
                assert!((h - z).abs() < 1e-9, "({x}, {y}): line {z} vs ground {h}");
            }
        }
    }

    /// Both endpoints of a piece lie in one closed triangle: same cell, same
    /// side of its diagonal.
    fn assert_one_triangle(grid: &Grid, a: [f64; 3], b: [f64; 3]) {
        let m = [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0];
        let (um, vm) = grid.to_uv(m[0], m[1]);
        let (c, r) = grid.cell(um, vm);
        let upper = (um - c as f64) >= (vm - r as f64);
        for p in [a, b] {
            let (u, v) = grid.to_uv(p[0], p[1]);
            let (fu, fv) = (u - c as f64, v - r as f64);
            assert!((-1e-9..=1.0 + 1e-9).contains(&fu) && (-1e-9..=1.0 + 1e-9).contains(&fv));
            if upper {
                assert!(fu >= fv - 1e-9, "{p:?} below the diagonal of cell ({c}, {r})");
            } else {
                assert!(fu <= fv + 1e-9, "{p:?} above the diagonal of cell ({c}, {r})");
            }
        }
    }

    #[test]
    fn a_diagonal_line_stays_in_one_triangle_per_piece() {
        let t = hill();
        let pts = drape(&t, &[[0.3, 0.1], [4.9, 4.2]]);
        // Four u lines, four v lines, no diagonal (u−v stays in (0, 1)).
        assert_eq!(pts.len(), 10, "{pts:?}");
        assert_on_surface(&t, &pts);
        for pair in pts.windows(2) {
            assert_one_triangle(&t.grid, pair[0], pair[1]);
        }
    }

    #[test]
    fn a_polyline_is_draped_piece_by_piece() {
        let t = hill();
        let pts = drape(&t, &[[0.5, 0.5], [4.5, 0.5], [4.5, 4.5], [0.2, 4.8], [2.5, 2.5]]);
        assert_on_surface(&t, &pts);
        for pair in pts.windows(2) {
            assert_one_triangle(&t.grid, pair[0], pair[1]);
        }
        assert_eq!(pts[0][..2], [0.5, 0.5]);
        assert_eq!(pts[pts.len() - 1][..2], [2.5, 2.5]);
    }

    #[test]
    fn a_line_along_a_grid_line_has_no_duplicates() {
        let t = hill();
        let pts = drape(&t, &[[2.0, 0.0], [2.0, 5.0]]);
        assert_eq!(pts.len(), 6, "{pts:?}");
        for pair in pts.windows(2) {
            assert!((pair[1][1] - pair[0][1] - 1.0).abs() < 1e-12);
        }
        assert_on_surface(&t, &pts);
        // Along a diagonal: every vertex is a lattice vertex, no extras.
        let pts = drape(&t, &[[0.0, 0.0], [5.0, 5.0]]);
        assert_eq!(pts.len(), 6, "{pts:?}");
    }

    #[test]
    fn split_ends_with_q_and_never_repeats_p() {
        let g = Grid::fit(&Rect { x0: 0.0, y0: 0.0, x1: 4.0, y1: 4.0 }, 1.0, usize::MAX);
        let out = split(&g, [1.0, 1.0], [1.0, 1.0]);
        assert_eq!(out, vec![[1.0, 1.0]]);
        let out = split(&g, [1.0, 1.0], [3.0, 1.0]);
        assert_eq!(out, vec![[2.0, 1.0], [3.0, 1.0]]);
    }

    #[test]
    fn a_ramp_drapes_to_a_straight_line() {
        let (t, _) = terrain::run(&extent(), &mut dem("ramp?grade=0.05&bearing=90&radius=100000"), 7.0, usize::MAX);
        let pts = drape(&t, &[[-500.0, -300.0], [400.0, 350.0]]);
        for p in &pts {
            assert!((p[2] - (400.0 + 0.05 * p[0])).abs() < 1e-6, "{p:?}");
        }
    }
}
