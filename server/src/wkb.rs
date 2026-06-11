//! WKB (Well-Known Binary) geometry parser (TILER.md §wkb).
//!
//! Decodes geometry types 1–7 (Point, LineString, Polygon, Multi*, and
//! GeometryCollection) into `geo-types`. Handles both byte orders and the
//! 2D / ISO-Z(M) / EWKB coordinate variants; Z and M ordinates are read and
//! discarded (tile elevation comes from terrain, not source geometry — see
//! FORMAT.md §3.4). Hand-rolled to stay independent of the churning geoarrow /
//! wkb crate APIs and to decode exactly the variants the inputs use.

use geo_types::{
    Coord, Geometry, GeometryCollection, LineString, MultiLineString, MultiPoint, MultiPolygon,
    Point, Polygon,
};

/// Cap on pre-allocation from counts read out of the blob: a corrupt count
/// can't trigger a huge up-front allocation (vectors still grow as needed,
/// and reading stops at end-of-buffer anyway).
const MAX_PREALLOC: usize = 65_536;

/// Why a WKB blob could not be parsed.
#[derive(Debug, PartialEq, Eq)]
pub enum WkbError {
    /// Ran out of bytes mid-record.
    UnexpectedEof,
    /// Byte-order flag was neither 0 (big) nor 1 (little).
    BadByteOrder(u8),
    /// Geometry type code is not one this parser handles.
    UnknownType(u32),
    /// A multi-geometry contained a member of the wrong type.
    MemberMismatch,
}

/// Parses a single WKB geometry from `data`.
pub fn parse(data: &[u8]) -> Result<Geometry, WkbError> {
    let mut cur = Cursor { data, pos: 0 };
    parse_geometry(&mut cur)
}

/// Serializes a geometry to little-endian 2D WKB (the inverse of [`parse`]).
///
/// Used to store clipped geometries compactly in the external-sort store.
/// Unsupported variants (e.g. `Rect`/`Triangle`) produce empty output.
pub fn to_wkb(geom: &Geometry) -> Vec<u8> {
    let mut buf = Vec::new();
    write_geometry(&mut buf, geom);
    buf
}

fn write_header(buf: &mut Vec<u8>, type_code: u32) {
    buf.push(1); // little-endian
    buf.extend_from_slice(&type_code.to_le_bytes());
}

fn write_coord(buf: &mut Vec<u8>, c: Coord) {
    buf.extend_from_slice(&c.x.to_le_bytes());
    buf.extend_from_slice(&c.y.to_le_bytes());
}

fn write_ring(buf: &mut Vec<u8>, ring: &LineString) {
    buf.extend_from_slice(&(ring.0.len() as u32).to_le_bytes());
    for c in &ring.0 {
        write_coord(buf, *c);
    }
}

fn write_geometry(buf: &mut Vec<u8>, geom: &Geometry) {
    match geom {
        Geometry::Point(p) => {
            write_header(buf, 1);
            write_coord(buf, p.0);
        }
        Geometry::LineString(ls) => {
            write_header(buf, 2);
            write_ring(buf, ls);
        }
        Geometry::Polygon(p) => {
            write_header(buf, 3);
            buf.extend_from_slice(&((1 + p.interiors().len()) as u32).to_le_bytes());
            write_ring(buf, p.exterior());
            for hole in p.interiors() {
                write_ring(buf, hole);
            }
        }
        Geometry::MultiPoint(mp) => {
            write_header(buf, 4);
            buf.extend_from_slice(&(mp.0.len() as u32).to_le_bytes());
            for p in &mp.0 {
                write_geometry(buf, &Geometry::Point(*p));
            }
        }
        Geometry::MultiLineString(mls) => {
            write_header(buf, 5);
            buf.extend_from_slice(&(mls.0.len() as u32).to_le_bytes());
            for ls in &mls.0 {
                write_geometry(buf, &Geometry::LineString(ls.clone()));
            }
        }
        Geometry::MultiPolygon(mp) => {
            write_header(buf, 6);
            buf.extend_from_slice(&(mp.0.len() as u32).to_le_bytes());
            for p in &mp.0 {
                write_geometry(buf, &Geometry::Polygon(p.clone()));
            }
        }
        _ => {}
    }
}

struct Cursor<'a> {
    data: &'a [u8],
    pos: usize,
}

impl Cursor<'_> {
    fn u8(&mut self) -> Result<u8, WkbError> {
        let b = *self.data.get(self.pos).ok_or(WkbError::UnexpectedEof)?;
        self.pos += 1;
        Ok(b)
    }

    fn u32(&mut self, le: bool) -> Result<u32, WkbError> {
        let arr: [u8; 4] = self.take(4)?.try_into().unwrap();
        Ok(if le { u32::from_le_bytes(arr) } else { u32::from_be_bytes(arr) })
    }

    fn f64(&mut self, le: bool) -> Result<f64, WkbError> {
        let arr: [u8; 8] = self.take(8)?.try_into().unwrap();
        Ok(if le { f64::from_le_bytes(arr) } else { f64::from_be_bytes(arr) })
    }

    fn take(&mut self, n: usize) -> Result<&[u8], WkbError> {
        let end = self.pos.checked_add(n).ok_or(WkbError::UnexpectedEof)?;
        let slice = self.data.get(self.pos..end).ok_or(WkbError::UnexpectedEof)?;
        self.pos = end;
        Ok(slice)
    }
}

/// Reads the byte-order + type header, returning (base type, little-endian,
/// has_z, has_m). Consumes the EWKB SRID field when present.
fn read_header(cur: &mut Cursor) -> Result<(u32, bool, bool, bool), WkbError> {
    let le = match cur.u8()? {
        0 => false,
        1 => true,
        other => return Err(WkbError::BadByteOrder(other)),
    };
    let t = cur.u32(le)?;
    // EWKB sets high flag bits; ISO encodes dimensionality in the thousands.
    if t & 0xE000_0000 != 0 {
        if t & 0x2000_0000 != 0 {
            cur.u32(le)?; // SRID — ignored
        }
        Ok((t & 0xFF, le, t & 0x8000_0000 != 0, t & 0x4000_0000 != 0))
    } else {
        let dim = t / 1000;
        Ok((t % 1000, le, dim == 1 || dim == 3, dim == 2 || dim == 3))
    }
}

fn parse_geometry(cur: &mut Cursor) -> Result<Geometry, WkbError> {
    let (base, le, hz, hm) = read_header(cur)?;
    match base {
        1 => Ok(Geometry::Point(Point(read_coord(cur, le, hz, hm)?))),
        2 => Ok(Geometry::LineString(read_linestring(cur, le, hz, hm)?)),
        3 => Ok(Geometry::Polygon(read_polygon(cur, le, hz, hm)?)),
        4 => Ok(Geometry::MultiPoint(MultiPoint(read_members(cur, le, |g| match g {
            Geometry::Point(p) => Some(p),
            _ => None,
        })?))),
        5 => Ok(Geometry::MultiLineString(MultiLineString(read_members(cur, le, |g| match g {
            Geometry::LineString(l) => Some(l),
            _ => None,
        })?))),
        6 => Ok(Geometry::MultiPolygon(MultiPolygon(read_members(cur, le, |g| match g {
            Geometry::Polygon(p) => Some(p),
            _ => None,
        })?))),
        7 => {
            let n = cur.u32(le)?;
            let mut geoms = Vec::with_capacity((n as usize).min(MAX_PREALLOC));
            for _ in 0..n {
                geoms.push(parse_geometry(cur)?);
            }
            Ok(Geometry::GeometryCollection(GeometryCollection(geoms)))
        }
        other => Err(WkbError::UnknownType(other)),
    }
}

fn read_coord(cur: &mut Cursor, le: bool, hz: bool, hm: bool) -> Result<Coord, WkbError> {
    let x = cur.f64(le)?;
    let y = cur.f64(le)?;
    if hz {
        cur.f64(le)?;
    }
    if hm {
        cur.f64(le)?;
    }
    Ok(Coord { x, y })
}

fn read_linestring(cur: &mut Cursor, le: bool, hz: bool, hm: bool) -> Result<LineString, WkbError> {
    let n = cur.u32(le)?;
    let mut coords = Vec::with_capacity((n as usize).min(MAX_PREALLOC));
    for _ in 0..n {
        coords.push(read_coord(cur, le, hz, hm)?);
    }
    Ok(LineString(coords))
}

fn read_polygon(cur: &mut Cursor, le: bool, hz: bool, hm: bool) -> Result<Polygon, WkbError> {
    let nrings = cur.u32(le)?;
    let mut rings = Vec::with_capacity((nrings as usize).min(MAX_PREALLOC));
    for _ in 0..nrings {
        rings.push(read_linestring(cur, le, hz, hm)?);
    }
    if rings.is_empty() {
        return Ok(Polygon::new(LineString(vec![]), vec![]));
    }
    let exterior = rings.remove(0);
    Ok(Polygon::new(exterior, rings))
}

/// Reads a multi-geometry: a count (in the container's byte order `le`)
/// followed by that many fully-headered members, each projected to the
/// expected variant.
fn read_members<T>(
    cur: &mut Cursor,
    le: bool,
    project: impl Fn(Geometry) -> Option<T>,
) -> Result<Vec<T>, WkbError> {
    let n = cur.u32(le)?;
    let mut out = Vec::with_capacity((n as usize).min(MAX_PREALLOC));
    for _ in 0..n {
        let g = parse_geometry(cur)?;
        out.push(project(g).ok_or(WkbError::MemberMismatch)?);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- WKB builders for tests (little-endian unless noted) ---

    fn hdr(buf: &mut Vec<u8>, type_code: u32) {
        buf.push(1);
        buf.extend_from_slice(&type_code.to_le_bytes());
    }

    fn xy(buf: &mut Vec<u8>, x: f64, y: f64) {
        buf.extend_from_slice(&x.to_le_bytes());
        buf.extend_from_slice(&y.to_le_bytes());
    }

    fn point(x: f64, y: f64) -> Vec<u8> {
        let mut b = Vec::new();
        hdr(&mut b, 1);
        xy(&mut b, x, y);
        b
    }

    fn ring(buf: &mut Vec<u8>, pts: &[(f64, f64)]) {
        buf.extend_from_slice(&(pts.len() as u32).to_le_bytes());
        for &(x, y) in pts {
            xy(buf, x, y);
        }
    }

    fn linestring(pts: &[(f64, f64)]) -> Vec<u8> {
        let mut b = Vec::new();
        hdr(&mut b, 2);
        ring(&mut b, pts);
        b
    }

    fn polygon(rings: &[&[(f64, f64)]]) -> Vec<u8> {
        let mut b = Vec::new();
        hdr(&mut b, 3);
        b.extend_from_slice(&(rings.len() as u32).to_le_bytes());
        for r in rings {
            ring(&mut b, r);
        }
        b
    }

    #[test]
    fn parses_point() {
        let g = parse(&point(30.0, 10.0)).unwrap();
        assert_eq!(g, Geometry::Point(Point::new(30.0, 10.0)));
    }

    #[test]
    fn parses_big_endian_point() {
        let mut b = Vec::new();
        b.push(0); // big-endian
        b.extend_from_slice(&1u32.to_be_bytes());
        b.extend_from_slice(&30.0f64.to_be_bytes());
        b.extend_from_slice(&10.0f64.to_be_bytes());
        assert_eq!(parse(&b).unwrap(), Geometry::Point(Point::new(30.0, 10.0)));
    }

    #[test]
    fn parses_iso_z_point_discarding_z() {
        // Type 1001 = PointZ. The z ordinate is read and dropped.
        let mut b = Vec::new();
        hdr(&mut b, 1001);
        b.extend_from_slice(&1.0f64.to_le_bytes());
        b.extend_from_slice(&2.0f64.to_le_bytes());
        b.extend_from_slice(&99.0f64.to_le_bytes()); // z
        assert_eq!(parse(&b).unwrap(), Geometry::Point(Point::new(1.0, 2.0)));
    }

    #[test]
    fn parses_ewkb_point_with_srid() {
        // 0x20000000 (SRID) | 1, followed by a 4-byte SRID.
        let mut b = Vec::new();
        b.push(1);
        b.extend_from_slice(&(0x2000_0001u32).to_le_bytes());
        b.extend_from_slice(&4326u32.to_le_bytes());
        xy(&mut b, 7.0, 8.0);
        assert_eq!(parse(&b).unwrap(), Geometry::Point(Point::new(7.0, 8.0)));
    }

    #[test]
    fn parses_linestring() {
        let g = parse(&linestring(&[(0.0, 0.0), (1.0, 1.0), (2.0, 0.0)])).unwrap();
        match g {
            Geometry::LineString(ls) => assert_eq!(ls.0.len(), 3),
            other => panic!("expected LineString, got {other:?}"),
        }
    }

    #[test]
    fn parses_polygon_with_hole() {
        let exterior: &[(f64, f64)] = &[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0), (0.0, 0.0)];
        let hole: &[(f64, f64)] = &[(2.0, 2.0), (4.0, 2.0), (4.0, 4.0), (2.0, 2.0)];
        let g = parse(&polygon(&[exterior, hole])).unwrap();
        match g {
            Geometry::Polygon(p) => {
                assert_eq!(p.exterior().0.len(), 5);
                assert_eq!(p.interiors().len(), 1);
                assert_eq!(p.interiors()[0].0.len(), 4);
            }
            other => panic!("expected Polygon, got {other:?}"),
        }
    }

    #[test]
    fn parses_multipoint_of_headered_members() {
        let mut b = Vec::new();
        hdr(&mut b, 4);
        b.extend_from_slice(&2u32.to_le_bytes());
        b.extend_from_slice(&point(1.0, 1.0)); // each member carries its own header
        b.extend_from_slice(&point(2.0, 2.0));
        match parse(&b).unwrap() {
            Geometry::MultiPoint(mp) => assert_eq!(mp.0.len(), 2),
            other => panic!("expected MultiPoint, got {other:?}"),
        }
    }

    #[test]
    fn parses_multipolygon() {
        let sq1: &[(f64, f64)] = &[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0)];
        let sq2: &[(f64, f64)] = &[(5.0, 5.0), (6.0, 5.0), (6.0, 6.0), (5.0, 5.0)];
        let mut b = Vec::new();
        hdr(&mut b, 6);
        b.extend_from_slice(&2u32.to_le_bytes());
        b.extend_from_slice(&polygon(&[sq1]));
        b.extend_from_slice(&polygon(&[sq2]));
        match parse(&b).unwrap() {
            Geometry::MultiPolygon(mp) => assert_eq!(mp.0.len(), 2),
            other => panic!("expected MultiPolygon, got {other:?}"),
        }
    }

    #[test]
    fn rejects_truncated_and_bad_byte_order() {
        assert_eq!(parse(&[]), Err(WkbError::UnexpectedEof));
        assert_eq!(parse(&point(1.0, 2.0)[..5]), Err(WkbError::UnexpectedEof));
        let mut bad = point(1.0, 2.0);
        bad[0] = 7; // invalid byte-order flag
        assert_eq!(parse(&bad), Err(WkbError::BadByteOrder(7)));
    }

    #[test]
    fn rejects_unknown_type() {
        let mut b = Vec::new();
        hdr(&mut b, 99);
        assert_eq!(parse(&b), Err(WkbError::UnknownType(99)));
    }

    #[test]
    fn writer_roundtrips_through_parser() {
        use geo_types::{MultiPolygon, Polygon};
        let cases = vec![
            Geometry::Point(Point::new(3.0, 4.0)),
            Geometry::LineString(LineString(vec![
                Coord { x: 0.0, y: 0.0 },
                Coord { x: 1.0, y: 2.0 },
            ])),
            Geometry::Polygon(Polygon::new(
                LineString(vec![
                    Coord { x: 0.0, y: 0.0 },
                    Coord { x: 4.0, y: 0.0 },
                    Coord { x: 4.0, y: 4.0 },
                    Coord { x: 0.0, y: 0.0 },
                ]),
                vec![LineString(vec![
                    Coord { x: 1.0, y: 1.0 },
                    Coord { x: 2.0, y: 1.0 },
                    Coord { x: 2.0, y: 2.0 },
                    Coord { x: 1.0, y: 1.0 },
                ])],
            )),
            Geometry::MultiPolygon(MultiPolygon(vec![Polygon::new(
                LineString(vec![
                    Coord { x: 5.0, y: 5.0 },
                    Coord { x: 6.0, y: 5.0 },
                    Coord { x: 6.0, y: 6.0 },
                    Coord { x: 5.0, y: 5.0 },
                ]),
                vec![],
            )])),
        ];
        for g in cases {
            assert_eq!(parse(&to_wkb(&g)).unwrap(), g, "roundtrip failed for {g:?}");
        }
    }
}
