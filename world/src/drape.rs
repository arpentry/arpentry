//! The drape: road centrelines laid exactly onto the terrain mesh.
//!
//! "Exactly" means the drawn line lies *on* the drawn surface, not near it: a
//! chord between two points of one plane lies in that plane, so a polyline
//! whose every piece stays inside one triangle of the mesh, and whose
//! vertices take their height from that triangle, is on the mesh to the ulp.
//! Splitting at grid lines alone is not enough — a piece that crosses a
//! cell's diagonal spans two planes and cuts through the ridge or valley the
//! diagonal makes — so the split runs at the diagonals too.
//!
//! What this step does *not* do is engineer anything: the line follows every
//! wrinkle of the raw ground. That is the point of this step. The steps after it
//! will have to earn every metre they move the ground by.

use std::path::Path;

use arpentry_server::geoparquet::ReadError;

use crate::frame::Extent;
use crate::grid::Grid;
use crate::net;
use crate::roads;
use crate::step::Summary;
use crate::terrain::height_at;
use crate::world::{Polyline2, Polyline3, Roads, Terrain};

/// Reads the ways of `segments` — a parquet, or a [`net`] spec — and drapes
/// them onto the world's terrain.
pub fn run(extent: &Extent, terrain: &Terrain, segments: &Path) -> Result<(Roads, Summary), String> {
    let read = match segments.to_str().filter(|s| net::is_spec(s)) {
        Some(spec) => synthetic(spec, &extent.rect)?,
        None => roads::read(segments, &extent.bbox, &extent.frame, &extent.rect)
            .map_err(|e: ReadError| e.to_string())?,
    };
    let mut roads = Roads::default();
    let (mut pieces, mut vertices) = (0usize, 0usize);
    // The *whole* way is draped, spans and all: the drawn centreline is the
    // way the source drew, and the cut into ground and structure pieces is
    // the partition step's (R1).
    for way in &read.ways {
        let pts = drape(terrain, &way.pts);
        pieces += pts.len().saturating_sub(1);
        vertices += pts.len();
        roads.lines.push(Polyline3 {
            id: way.id.clone(),
            class: way.class.clone(),
            subclass: way.subclass.clone(),
            width_m: way.width_m,
            pts,
        });
    }
    let summary = Summary::new()
        .with("features", read.features)
        .with("ways", read.kept)
        .with("structures", format!("{} ({} off the ground entirely)", read.structures, read.dropped))
        .with("measured", read.measured)
        .with("oneway", read.oneway)
        .with("rail", format!("{} ({} street rail not kept)", read.rail, read.street_rail))
        .with("layered", read.layered)
        .with("clipped", read.ways.len())
        .with("draped", roads.lines.len())
        .with("pieces", pieces)
        .with("vertices", vertices);
    roads.ways = read.ways;
    Ok((roads, summary))
}

/// The ways of a synthetic network, clipped to the rect like a read one —
/// whole, with their span tables re-based on each clipped run.
fn synthetic(spec: &str, rect: &crate::frame::Rect) -> Result<roads::Read, String> {
    let mut out = roads::Read::default();
    for way in net::parse(spec)? {
        out.features += 1;
        out.kept += 1;
        out.rail += (crate::width::family(&way.class) == crate::width::Family::Rail) as usize;
        if way.has_structure() {
            out.structures += 1;
        }
        out.ways.extend(roads::clip_way(&way, rect));
    }
    Ok(out)
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
