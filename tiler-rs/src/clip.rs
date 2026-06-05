//! Geometry clipping and tile assignment (TILER.md §clip).
//!
//! - **Points**: bounding-box containment.
//! - **Lines**: Liang–Barsky segment clipping.
//! - **Polygons**: Sutherland–Hodgman four-edge clipping.
//!
//! [`assign_tiles`] is the high-level entry: it fans a geometry out to every
//! tile (at one zoom) whose buffered bounds it touches, clipping it directly to
//! each tile.
//!
//! Everything works in WGS84 degrees against a [`Bounds`] rectangle; the clip
//! rect is the tile bounds expanded by [`BUFFER_FRAC`] (the format buffer).

use geo_types::{Coord, Geometry, LineString, MultiLineString, MultiPoint, MultiPolygon, Point, Polygon};

use crate::project::{Bounds, BUFFER, EXTENT};

/// Buffer as a fraction of tile size per side (16384 / 32768 = 0.5).
pub const BUFFER_FRAC: f64 = BUFFER / EXTENT;

/// Clips a geometry to a rectangle, or `None` if nothing remains.
pub fn clip_geometry(geom: &Geometry, rect: &Bounds) -> Option<Geometry> {
    match geom {
        Geometry::Point(p) => rect.contains(p.0.x, p.0.y).then(|| geom.clone()),
        Geometry::MultiPoint(mp) => {
            let pts: Vec<Point> = mp.0.iter().copied().filter(|p| rect.contains(p.0.x, p.0.y)).collect();
            (!pts.is_empty()).then(|| Geometry::MultiPoint(MultiPoint(pts)))
        }
        Geometry::Line(l) => {
            let ls = LineString(vec![l.start, l.end]);
            lines_to_geometry(clip_linestring(&ls, rect))
        }
        Geometry::LineString(ls) => lines_to_geometry(clip_linestring(ls, rect)),
        Geometry::MultiLineString(mls) => {
            let lines: Vec<LineString> =
                mls.0.iter().flat_map(|ls| clip_linestring(ls, rect)).collect();
            lines_to_geometry(lines)
        }
        Geometry::Polygon(p) => clip_polygon(p, rect).map(Geometry::Polygon),
        Geometry::MultiPolygon(mp) => {
            let polys: Vec<Polygon> = mp.0.iter().filter_map(|p| clip_polygon(p, rect)).collect();
            (!polys.is_empty()).then(|| Geometry::MultiPolygon(MultiPolygon(polys)))
        }
        _ => None,
    }
}

/// Assigns a geometry to tiles at `zoom`, clipping it to each tile's buffered
/// bounds and invoking `emit(x, y, clipped)` for every non-empty result.
pub fn assign_tiles(geom: &Geometry, zoom: u8, mut emit: impl FnMut(u32, u32, Geometry)) {
    let Some((min_x, min_y, max_x, max_y)) = bbox(geom) else {
        return;
    };
    // Grid is 2^z × 2^z (one root tile at z0), matching the C client/server.
    let cols = 1u64 << zoom as u32;
    let rows = 1u64 << zoom as u32;
    let tile_w = 360.0 / cols as f64;
    let tile_h = 180.0 / rows as f64;
    let margin_x = tile_w * BUFFER_FRAC;
    let margin_y = tile_h * BUFFER_FRAC;

    // Candidate tile range covering the buffer-expanded bounding box.
    let x0 = tile_index((min_x - margin_x) + 180.0, tile_w, cols);
    let x1 = tile_index((max_x + margin_x) + 180.0, tile_w, cols);
    let y0 = tile_index((min_y - margin_y) + 90.0, tile_h, rows);
    let y1 = tile_index((max_y + margin_y) + 90.0, tile_h, rows);

    // Clip the geometry directly against each candidate tile. (A two-pass
    // row-band/column "stripe" clip is tempting for speed, but Sutherland–
    // Hodgman introduces connecting edges along the band boundary that then
    // leak into and fill columns the geometry never actually reaches.)
    for ty in y0..=y1 {
        for tx in x0..=x1 {
            let tile_rect = Bounds::of_tile(zoom, tx as u32, ty as u32).expanded(BUFFER_FRAC);
            if let Some(clipped) = clip_geometry(geom, &tile_rect) {
                emit(tx as u32, ty as u32, clipped);
            }
        }
    }
}

/// Maps a shifted coordinate (origin-relative degrees) to a clamped tile index.
fn tile_index(shifted: f64, tile_size: f64, count: u64) -> u64 {
    let i = (shifted / tile_size).floor();
    if i < 0.0 {
        0
    } else {
        (i as u64).min(count - 1)
    }
}

/// Axis-aligned bounding box of a geometry's coordinates, if any.
pub fn bbox(geom: &Geometry) -> Option<(f64, f64, f64, f64)> {
    let mut min_x = f64::INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    let mut any = false;
    let mut visit = |c: Coord| {
        any = true;
        min_x = min_x.min(c.x);
        min_y = min_y.min(c.y);
        max_x = max_x.max(c.x);
        max_y = max_y.max(c.y);
    };
    for_each_coord(geom, &mut visit);
    any.then_some((min_x, min_y, max_x, max_y))
}

fn for_each_coord(geom: &Geometry, f: &mut impl FnMut(Coord)) {
    match geom {
        Geometry::Point(p) => f(p.0),
        Geometry::MultiPoint(mp) => mp.0.iter().for_each(|p| f(p.0)),
        Geometry::Line(l) => {
            f(l.start);
            f(l.end);
        }
        Geometry::LineString(ls) => ls.0.iter().for_each(|c| f(*c)),
        Geometry::MultiLineString(mls) => {
            mls.0.iter().for_each(|ls| ls.0.iter().for_each(|c| f(*c)))
        }
        Geometry::Polygon(p) => ring_coords(p, f),
        Geometry::MultiPolygon(mp) => mp.0.iter().for_each(|p| ring_coords(p, f)),
        _ => {}
    }
}

fn ring_coords(p: &Polygon, f: &mut impl FnMut(Coord)) {
    p.exterior().0.iter().for_each(|c| f(*c));
    p.interiors().iter().for_each(|r| r.0.iter().for_each(|c| f(*c)));
}

fn lines_to_geometry(mut lines: Vec<LineString>) -> Option<Geometry> {
    match lines.len() {
        0 => None,
        1 => Some(Geometry::LineString(lines.pop().unwrap())),
        _ => Some(Geometry::MultiLineString(MultiLineString(lines))),
    }
}

// --- Liang–Barsky line clipping ---

/// Clips a segment to the rectangle, returning the trimmed endpoints if any
/// part lies inside.
fn liang_barsky(p0: Coord, p1: Coord, r: &Bounds) -> Option<(Coord, Coord)> {
    let dx = p1.x - p0.x;
    let dy = p1.y - p0.y;
    let p = [-dx, dx, -dy, dy];
    let q = [p0.x - r.west, r.east - p0.x, p0.y - r.south, r.north - p0.y];
    let mut t0 = 0.0;
    let mut t1 = 1.0;
    for i in 0..4 {
        if p[i] == 0.0 {
            // Parallel to this edge and outside it → wholly rejected.
            if q[i] < 0.0 {
                return None;
            }
        } else {
            let t = q[i] / p[i];
            if p[i] < 0.0 {
                if t > t1 {
                    return None;
                }
                if t > t0 {
                    t0 = t;
                }
            } else {
                if t < t0 {
                    return None;
                }
                if t < t1 {
                    t1 = t;
                }
            }
        }
    }
    let a = Coord { x: p0.x + t0 * dx, y: p0.y + t0 * dy };
    let b = Coord { x: p0.x + t1 * dx, y: p0.y + t1 * dy };
    Some((a, b))
}

/// Clips a linestring to a rectangle, splitting it into the pieces that remain.
fn clip_linestring(ls: &LineString, r: &Bounds) -> Vec<LineString> {
    let mut out = Vec::new();
    let mut cur: Vec<Coord> = Vec::new();
    for w in ls.0.windows(2) {
        match liang_barsky(w[0], w[1], r) {
            Some((a, b)) => {
                if cur.is_empty() {
                    cur.push(a);
                    cur.push(b);
                } else if coords_eq(*cur.last().unwrap(), a) {
                    // Segment continues from the previous interior vertex.
                    cur.push(b);
                } else {
                    // Re-entered the rect → start a new disconnected piece.
                    finish_piece(&mut out, std::mem::take(&mut cur));
                    cur.push(a);
                    cur.push(b);
                }
            }
            None => finish_piece(&mut out, std::mem::take(&mut cur)),
        }
    }
    finish_piece(&mut out, cur);
    out
}

fn finish_piece(out: &mut Vec<LineString>, piece: Vec<Coord>) {
    if piece.len() >= 2 {
        out.push(LineString(piece));
    }
}

// --- Sutherland–Hodgman polygon clipping ---

fn clip_polygon(poly: &Polygon, r: &Bounds) -> Option<Polygon> {
    let exterior = clip_ring(&poly.exterior().0, r)?;
    let interiors: Vec<LineString> = poly
        .interiors()
        .iter()
        .filter_map(|ring| clip_ring(&ring.0, r))
        .collect();
    Some(Polygon::new(exterior, interiors))
}

/// Clips one ring against all four rectangle edges, returning a closed ring
/// (first == last) or `None` if fewer than 3 distinct vertices remain.
fn clip_ring(ring: &[Coord], r: &Bounds) -> Option<LineString> {
    // Work on the open ring (drop a repeated closing point).
    let mut poly: Vec<Coord> = ring.to_vec();
    if poly.len() > 1 && coords_eq(poly[0], *poly.last().unwrap()) {
        poly.pop();
    }
    poly = clip_edge(&poly, |c| c.x >= r.west, |a, b| intersect_x(a, b, r.west));
    poly = clip_edge(&poly, |c| c.x <= r.east, |a, b| intersect_x(a, b, r.east));
    poly = clip_edge(&poly, |c| c.y >= r.south, |a, b| intersect_y(a, b, r.south));
    poly = clip_edge(&poly, |c| c.y <= r.north, |a, b| intersect_y(a, b, r.north));

    if poly.len() < 3 {
        return None;
    }
    poly.push(poly[0]); // re-close
    Some(LineString(poly))
}

/// One Sutherland–Hodgman pass against a single half-plane.
fn clip_edge(
    input: &[Coord],
    inside: impl Fn(Coord) -> bool,
    intersect: impl Fn(Coord, Coord) -> Coord,
) -> Vec<Coord> {
    let n = input.len();
    let mut out = Vec::new();
    if n == 0 {
        return out;
    }
    for i in 0..n {
        let cur = input[i];
        let prev = input[(i + n - 1) % n];
        let cur_in = inside(cur);
        let prev_in = inside(prev);
        if cur_in {
            if !prev_in {
                out.push(intersect(prev, cur));
            }
            out.push(cur);
        } else if prev_in {
            out.push(intersect(prev, cur));
        }
    }
    out
}

fn intersect_x(a: Coord, b: Coord, x: f64) -> Coord {
    let t = (x - a.x) / (b.x - a.x);
    Coord { x, y: a.y + t * (b.y - a.y) }
}

fn intersect_y(a: Coord, b: Coord, y: f64) -> Coord {
    let t = (y - a.y) / (b.y - a.y);
    Coord { x: a.x + t * (b.x - a.x), y }
}

fn coords_eq(a: Coord, b: Coord) -> bool {
    a.x == b.x && a.y == b.y
}

#[cfg(test)]
mod tests {
    use super::*;

    fn c(x: f64, y: f64) -> Coord {
        Coord { x, y }
    }

    fn rect(west: f64, south: f64, east: f64, north: f64) -> Bounds {
        Bounds { west, south, east, north }
    }

    #[test]
    fn point_containment() {
        let r = rect(0.0, 0.0, 10.0, 10.0);
        assert!(clip_geometry(&Geometry::Point(Point::new(5.0, 5.0)), &r).is_some());
        assert!(clip_geometry(&Geometry::Point(Point::new(-1.0, 5.0)), &r).is_none());
    }

    #[test]
    fn line_clipped_to_boundary() {
        let r = rect(0.0, 0.0, 10.0, 10.0);
        // Horizontal line from -5 to 15 at y=5 → clipped to x in [0, 10].
        let ls = LineString(vec![c(-5.0, 5.0), c(15.0, 5.0)]);
        let out = clip_linestring(&ls, &r);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].0, vec![c(0.0, 5.0), c(10.0, 5.0)]);
    }

    #[test]
    fn line_fully_inside_unchanged() {
        let r = rect(0.0, 0.0, 10.0, 10.0);
        let ls = LineString(vec![c(2.0, 2.0), c(4.0, 4.0), c(6.0, 3.0)]);
        let out = clip_linestring(&ls, &r);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].0, ls.0);
    }

    #[test]
    fn line_fully_outside_dropped() {
        let r = rect(0.0, 0.0, 10.0, 10.0);
        let ls = LineString(vec![c(20.0, 20.0), c(30.0, 25.0)]);
        assert!(clip_linestring(&ls, &r).is_empty());
    }

    #[test]
    fn line_splits_into_two_pieces() {
        let r = rect(0.0, 0.0, 10.0, 10.0);
        // Dips out of the rect in the middle, re-enters → two pieces.
        let ls = LineString(vec![
            c(1.0, 5.0),
            c(5.0, -5.0), // excursion below the rect
            c(9.0, 5.0),
        ]);
        let out = clip_linestring(&ls, &r);
        assert_eq!(out.len(), 2, "should split into two disconnected pieces");
    }

    #[test]
    fn polygon_clipped_to_rect() {
        let r = rect(0.0, 0.0, 10.0, 10.0);
        // A big square covering the rect → clipped result is the rect itself.
        let poly = Polygon::new(
            LineString(vec![
                c(-5.0, -5.0),
                c(15.0, -5.0),
                c(15.0, 15.0),
                c(-5.0, 15.0),
                c(-5.0, -5.0),
            ]),
            vec![],
        );
        let clipped = clip_polygon(&poly, &r).expect("non-empty");
        // Closed ring of the clip rect's 4 corners (+ closing point).
        assert_eq!(clipped.exterior().0.first(), clipped.exterior().0.last());
        let xs: Vec<f64> = clipped.exterior().0.iter().map(|p| p.x).collect();
        let ys: Vec<f64> = clipped.exterior().0.iter().map(|p| p.y).collect();
        assert!(xs.iter().all(|&x| (0.0..=10.0).contains(&x)));
        assert!(ys.iter().all(|&y| (0.0..=10.0).contains(&y)));
    }

    #[test]
    fn polygon_fully_outside_dropped() {
        let r = rect(0.0, 0.0, 10.0, 10.0);
        let poly = Polygon::new(
            LineString(vec![c(20.0, 20.0), c(30.0, 20.0), c(30.0, 30.0), c(20.0, 20.0)]),
            vec![],
        );
        assert!(clip_polygon(&poly, &r).is_none());
    }

    #[test]
    fn bbox_of_polygon() {
        let poly = Geometry::Polygon(Polygon::new(
            LineString(vec![c(1.0, 2.0), c(5.0, 2.0), c(5.0, 8.0), c(1.0, 8.0), c(1.0, 2.0)]),
            vec![],
        ));
        assert_eq!(bbox(&poly), Some((1.0, 2.0, 5.0, 8.0)));
    }

    #[test]
    fn assign_point_to_containing_tile() {
        // A point well inside one tile (away from edges) lands in exactly one
        // tile at zoom 2. Grid: 4 cols × 4 rows.
        let geom = Geometry::Point(Point::new(-90.0 + 0.001, 45.0 + 0.001));
        let mut hits = Vec::new();
        assign_tiles(&geom, 2, |x, y, _| hits.push((x, y)));
        // -90 lon at z2: col width = 360/4 = 90; (-90+180)/90 = 1 → col 1.
        // 45 lat: row height = 180/4 = 45; (45+90)/45 = 3 → row 3.
        assert!(hits.contains(&(1, 3)), "got {hits:?}");
    }

    #[test]
    fn assign_point_near_edge_hits_neighbors_via_buffer() {
        // A point right on a tile boundary falls within the buffer of both
        // adjacent tiles, so it is assigned to more than one.
        let geom = Geometry::Point(Point::new(0.0, 0.0)); // tile corner at z1
        let mut hits = Vec::new();
        assign_tiles(&geom, 1, |x, y, _| hits.push((x, y)));
        assert!(hits.len() > 1, "edge point should hit multiple tiles: {hits:?}");
    }

    #[test]
    fn assign_polygon_clips_per_tile() {
        // A polygon spanning two columns at zoom 1 is emitted, clipped, for the
        // tiles it covers.
        let geom = Geometry::Polygon(Polygon::new(
            LineString(vec![
                c(-10.0, -10.0),
                c(10.0, -10.0),
                c(10.0, 10.0),
                c(-10.0, 10.0),
                c(-10.0, -10.0),
            ]),
            vec![],
        ));
        let mut hits = Vec::new();
        assign_tiles(&geom, 1, |x, y, g| {
            assert!(matches!(g, Geometry::Polygon(_) | Geometry::MultiPolygon(_)));
            hits.push((x, y));
        });
        assert!(!hits.is_empty());
        // Every emitted tile is a valid z1 index (2 cols × 2 rows).
        assert!(hits.iter().all(|&(x, y)| x < 2 && y < 2));
    }
}
