//! The terrain lattice: a regular grid of cells spanning a rectangle exactly.

use crate::frame::Rect;

/// A regular lattice of `cols × rows` cells. Vertices are `(cols+1) × (rows+1)`,
/// indexed row-major by [`Grid::index`]; vertex `(c, r)` sits at
/// `(x0 + c·dx, y0 + r·dy)`.
///
/// Each cell is two triangles split on its SW→NE diagonal — the same
/// convention as the server's lattice (`terrain.rs`), which is what makes a
/// drape and a mesh agree: [`crate::terrain::height_at`] and the triangle
/// list read the same rule.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Grid {
    pub x0: f64,
    pub y0: f64,
    pub dx: f64,
    pub dy: f64,
    pub cols: u32,
    pub rows: u32,
}

impl Grid {
    /// A grid over `rect` at about `spacing` metres, holding at most `cap`
    /// vertices.
    ///
    /// The spacing is clamped *up* until the vertex count fits, never refused:
    /// a bbox too large for the cap gets a coarser mesh and says so in the
    /// summary. The grid always spans `rect` exactly (`dx = width / cols`), so
    /// the effective spacing is at most the requested one within a cell.
    pub fn fit(rect: &Rect, spacing: f64, cap: usize) -> Grid {
        let w = rect.width().max(0.0);
        let h = rect.height().max(0.0);
        let cap = cap.max(4) as f64;
        let mut s = spacing.max(1e-3);
        // `cols ≤ w/s + 1`, so `(cols+1)(rows+1) ≤ (w/s + 2)(h/s + 2)`: solve
        // that bound for the largest `1/s` with the quadratic, then let the
        // loop below absorb rounding.
        if w > 0.0 && h > 0.0 {
            let b = w + h;
            let q = (-b + (b * b + w * h * (cap - 4.0)).sqrt()) / (w * h);
            if q > 0.0 {
                s = s.max(1.0 / q);
            }
        }
        let (mut cols, mut rows) = Grid::counts(w, h, s);
        while (cols as f64 + 1.0) * (rows as f64 + 1.0) > cap {
            s *= 1.001;
            (cols, rows) = Grid::counts(w, h, s);
        }
        let dx = if w > 0.0 { w / cols as f64 } else { s };
        let dy = if h > 0.0 { h / rows as f64 } else { s };
        Grid { x0: rect.x0, y0: rect.y0, dx, dy, cols, rows }
    }

    fn counts(w: f64, h: f64, s: f64) -> (u32, u32) {
        let cols = ((w / s).ceil() as u32).max(1);
        let rows = ((h / s).ceil() as u32).max(1);
        (cols, rows)
    }

    pub fn vertex_count(&self) -> usize {
        (self.cols as usize + 1) * (self.rows as usize + 1)
    }

    pub fn cell_count(&self) -> usize {
        self.cols as usize * self.rows as usize
    }

    /// Vertex index of `(c, r)`.
    pub fn index(&self, c: u32, r: u32) -> usize {
        r as usize * (self.cols as usize + 1) + c as usize
    }

    /// `(c, r)` of vertex index `i`.
    pub fn vertex_of(&self, i: usize) -> (u32, u32) {
        let stride = self.cols as usize + 1;
        ((i % stride) as u32, (i / stride) as u32)
    }

    /// Position of vertex `(c, r)`.
    pub fn vertex(&self, c: u32, r: u32) -> [f64; 2] {
        self.vertex_at(c as f64, r as f64)
    }

    /// Position at fractional lattice coordinates; `(−1, −1)` is the halo
    /// vertex outside the south-west corner.
    pub fn vertex_at(&self, c: f64, r: f64) -> [f64; 2] {
        [self.x0 + c * self.dx, self.y0 + r * self.dy]
    }

    /// Lattice coordinates of a local point: `u` in cells east of `x0`, `v`
    /// in cells north of `y0`.
    pub fn to_uv(&self, x: f64, y: f64) -> (f64, f64) {
        ((x - self.x0) / self.dx, (y - self.y0) / self.dy)
    }

    /// The cell containing lattice point `(u, v)`, clamped into the grid so a
    /// point on the east or north edge belongs to the last cell.
    pub fn cell(&self, u: f64, v: f64) -> (u32, u32) {
        let c = u.floor().clamp(0.0, (self.cols - 1) as f64) as u32;
        let r = v.floor().clamp(0.0, (self.rows - 1) as f64) as u32;
        (c, r)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rect(w: f64, h: f64) -> Rect {
        Rect { x0: -w / 2.0, y0: -h / 2.0, x1: w / 2.0, y1: h / 2.0 }
    }

    #[test]
    fn spans_the_rect_exactly() {
        let r = rect(1001.0, 503.0);
        let g = Grid::fit(&r, 2.0, usize::MAX);
        assert_eq!((g.cols, g.rows), (501, 252));
        let [x, y] = g.vertex(g.cols, g.rows);
        assert!((x - r.x1).abs() < 1e-9 && (y - r.y1).abs() < 1e-9);
        assert!(g.dx <= 2.0 && g.dy <= 2.0);
    }

    #[test]
    fn cap_clamps_the_spacing() {
        let r = rect(1500.0, 1100.0);
        for cap in [4usize, 10, 100, 1000, 12345] {
            let g = Grid::fit(&r, 2.0, cap);
            assert!(g.vertex_count() <= cap, "cap {cap}: {} vertices", g.vertex_count());
            let [x, y] = g.vertex(g.cols, g.rows);
            assert!((x - r.x1).abs() < 1e-9 && (y - r.y1).abs() < 1e-9);
        }
        // A generous cap leaves the requested spacing alone.
        let g = Grid::fit(&r, 2.0, 2_000_000);
        assert!((g.dx - 2.0).abs() < 0.01 && (g.dy - 2.0).abs() < 0.01);
    }

    #[test]
    fn edge_points_fall_in_the_last_cell() {
        let g = Grid::fit(&rect(10.0, 10.0), 1.0, usize::MAX);
        assert_eq!(g.cell(10.0, 10.0), (9, 9));
        assert_eq!(g.cell(0.0, 0.0), (0, 0));
        assert_eq!(g.cell(-0.5, 12.0), (0, 9));
        assert_eq!(g.cell(3.7, 4.2), (3, 4));
    }

    #[test]
    fn index_round_trips() {
        let g = Grid::fit(&rect(7.0, 3.0), 1.0, usize::MAX);
        for i in 0..g.vertex_count() {
            let (c, r) = g.vertex_of(i);
            assert_eq!(g.index(c, r), i);
        }
    }
}
