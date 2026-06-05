//! Douglas–Peucker line simplification (TILER.md §simplify).
//!
//! Removes vertices that deviate from the line between retained endpoints by
//! less than `tolerance` (in the input coordinate units — the pipeline picks a
//! per-zoom tolerance). The core runs iteratively (explicit stack) so very long
//! lines can't overflow the call stack.

use geo_types::{Coord, Geometry, LineString, MultiLineString, MultiPolygon, Polygon};

/// Simplifies a coordinate sequence with Douglas–Peucker, returning the kept
/// vertices. Endpoints are always retained; a sequence of ≤2 points (or a
/// non-positive tolerance) passes through unchanged.
pub fn douglas_peucker(points: &[Coord], tolerance: f64) -> Vec<Coord> {
    let n = points.len();
    if n <= 2 || tolerance <= 0.0 {
        return points.to_vec();
    }
    let tol_sq = tolerance * tolerance;
    let mut keep = vec![false; n];
    keep[0] = true;
    keep[n - 1] = true;

    let mut stack = vec![(0usize, n - 1)];
    while let Some((first, last)) = stack.pop() {
        if last <= first + 1 {
            continue;
        }
        let mut max_d = 0.0;
        let mut split = first;
        for i in (first + 1)..last {
            let d = perp_dist_sq(points[i], points[first], points[last]);
            if d > max_d {
                max_d = d;
                split = i;
            }
        }
        if max_d > tol_sq {
            keep[split] = true;
            stack.push((first, split));
            stack.push((split, last));
        }
    }

    points
        .iter()
        .zip(keep)
        .filter_map(|(p, k)| k.then_some(*p))
        .collect()
}

/// Squared perpendicular distance from `p` to the line through `a` and `b`
/// (degenerating to the distance to `a` when `a == b`).
fn perp_dist_sq(p: Coord, a: Coord, b: Coord) -> f64 {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let len_sq = dx * dx + dy * dy;
    if len_sq == 0.0 {
        let ex = p.x - a.x;
        let ey = p.y - a.y;
        return ex * ex + ey * ey;
    }
    let cross = dx * (p.y - a.y) - dy * (p.x - a.x);
    (cross * cross) / len_sq
}

/// Simplifies a line, returning `None` if it collapses below 2 points.
pub fn simplify_linestring(ls: &LineString, tolerance: f64) -> Option<LineString> {
    let pts = douglas_peucker(&ls.0, tolerance);
    (pts.len() >= 2).then(|| LineString(pts))
}

/// Simplifies a ring, returning `None` if it collapses below a valid ring.
///
/// A closed ring needs at least 4 points (the closing point repeats the first).
fn simplify_ring(ring: &LineString, tolerance: f64) -> Option<LineString> {
    let pts = douglas_peucker(&ring.0, tolerance);
    (pts.len() >= 4).then(|| LineString(pts))
}

fn simplify_polygon(poly: &Polygon, tolerance: f64) -> Option<Polygon> {
    let exterior = simplify_ring(poly.exterior(), tolerance)?;
    let interiors: Vec<LineString> = poly
        .interiors()
        .iter()
        .filter_map(|r| simplify_ring(r, tolerance))
        .collect();
    Some(Polygon::new(exterior, interiors))
}

/// Simplifies any supported geometry, returning `None` if it collapses to
/// nothing. Points pass through unchanged.
pub fn simplify_geometry(geom: &Geometry, tolerance: f64) -> Option<Geometry> {
    match geom {
        Geometry::Point(_) | Geometry::MultiPoint(_) => Some(geom.clone()),
        Geometry::LineString(ls) => simplify_linestring(ls, tolerance).map(Geometry::LineString),
        Geometry::MultiLineString(mls) => {
            let lines: Vec<LineString> = mls
                .0
                .iter()
                .filter_map(|ls| simplify_linestring(ls, tolerance))
                .collect();
            (!lines.is_empty()).then(|| Geometry::MultiLineString(MultiLineString(lines)))
        }
        Geometry::Polygon(p) => simplify_polygon(p, tolerance).map(Geometry::Polygon),
        Geometry::MultiPolygon(mp) => {
            let polys: Vec<Polygon> = mp
                .0
                .iter()
                .filter_map(|p| simplify_polygon(p, tolerance))
                .collect();
            (!polys.is_empty()).then(|| Geometry::MultiPolygon(MultiPolygon(polys)))
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn c(x: f64, y: f64) -> Coord {
        Coord { x, y }
    }

    #[test]
    fn collinear_points_collapse_to_endpoints() {
        let line = [c(0.0, 0.0), c(1.0, 0.0), c(2.0, 0.0), c(3.0, 0.0)];
        let out = douglas_peucker(&line, 0.01);
        assert_eq!(out, vec![c(0.0, 0.0), c(3.0, 0.0)]);
    }

    #[test]
    fn spike_above_tolerance_is_kept() {
        // A clear deviation in the middle must survive.
        let line = [c(0.0, 0.0), c(1.0, 5.0), c(2.0, 0.0)];
        let out = douglas_peucker(&line, 1.0);
        assert_eq!(out, vec![c(0.0, 0.0), c(1.0, 5.0), c(2.0, 0.0)]);
    }

    #[test]
    fn small_deviation_below_tolerance_is_dropped() {
        let line = [c(0.0, 0.0), c(1.0, 0.001), c(2.0, 0.0)];
        let out = douglas_peucker(&line, 0.1);
        assert_eq!(out, vec![c(0.0, 0.0), c(2.0, 0.0)]);
    }

    #[test]
    fn endpoints_and_closure_preserved_for_ring() {
        // A square ring (closed). With a tiny tolerance the 4 corners + closing
        // point survive; closure (first == last) is preserved.
        let ring = LineString(vec![
            c(0.0, 0.0),
            c(0.0, 10.0),
            c(10.0, 10.0),
            c(10.0, 0.0),
            c(0.0, 0.0),
        ]);
        let out = simplify_ring(&ring, 0.001).expect("ring kept");
        assert_eq!(out.0.first(), out.0.last());
        assert!(out.0.len() >= 4);
    }

    #[test]
    fn degenerate_ring_dropped() {
        // A near-zero-area sliver simplifies away.
        let ring = LineString(vec![c(0.0, 0.0), c(1.0, 0.0), c(2.0, 0.0), c(0.0, 0.0)]);
        assert!(simplify_ring(&ring, 0.5).is_none());
    }
}
