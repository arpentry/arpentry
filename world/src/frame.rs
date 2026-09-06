//! The local metric frame: equirectangular metres around the bbox centre.
//!
//! `east = (lon − lon0)·cos(lat0)·DEG_M`, `north = (lat − lat0)·DEG_M`, up is
//! height — the same formula, and the same constant, as the synthetic ground
//! in `arpentry_server::dem::Field`, so a ramp of grade `g` is exactly
//! `g·east` in this frame and a test can assert it to the ulp. One shared
//! `DEG_M` is the point: the server once carried three competing constants and
//! measured a 0.70 % north–south bias between them (`scene.rs`).
//!
//! For a box under ~15 km the projection error is well below anything a step
//! will ever assert on, and the frame is trivially invertible, which the DEM
//! sampling needs.

use arpentry_server::project::Bounds;
use arpentry_server::scene::DEG_M;

/// A local frame centred on `(lon0, lat0)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Frame {
    pub lon0: f64,
    pub lat0: f64,
    cos_lat0: f64,
}

/// An axis-aligned rectangle in local metres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rect {
    pub x0: f64,
    pub y0: f64,
    pub x1: f64,
    pub y1: f64,
}

impl Rect {
    pub fn width(&self) -> f64 {
        self.x1 - self.x0
    }

    pub fn height(&self) -> f64 {
        self.y1 - self.y0
    }

    /// Whether `p` lies inside or on the boundary.
    pub fn contains(&self, p: [f64; 2]) -> bool {
        p[0] >= self.x0 && p[0] <= self.x1 && p[1] >= self.y0 && p[1] <= self.y1
    }
}

impl Frame {
    /// The frame centred on `bbox`.
    pub fn centred(bbox: &Bounds) -> Frame {
        Frame::at((bbox.west + bbox.east) / 2.0, (bbox.south + bbox.north) / 2.0)
    }

    /// The frame centred on `(lon0, lat0)`.
    pub fn at(lon0: f64, lat0: f64) -> Frame {
        Frame { lon0, lat0, cos_lat0: lat0.to_radians().cos() }
    }

    /// Local metres of a geographic point.
    pub fn to_local(&self, lon: f64, lat: f64) -> [f64; 2] {
        [(lon - self.lon0) * self.cos_lat0 * DEG_M, (lat - self.lat0) * DEG_M]
    }

    /// Geographic point of local metres.
    pub fn to_geo(&self, x: f64, y: f64) -> (f64, f64) {
        (self.lon0 + x / (self.cos_lat0 * DEG_M), self.lat0 + y / DEG_M)
    }

    /// `bbox` in this frame.
    pub fn rect(&self, bbox: &Bounds) -> Rect {
        let [x0, y0] = self.to_local(bbox.west, bbox.south);
        let [x1, y1] = self.to_local(bbox.east, bbox.north);
        Rect { x0, y0, x1, y1 }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips() {
        let f = Frame::at(6.92, 46.435);
        for &(lon, lat) in &[(6.91, 46.43), (6.93, 46.44), (6.92, 46.435)] {
            let [x, y] = f.to_local(lon, lat);
            let (lon2, lat2) = f.to_geo(x, y);
            assert!((lon - lon2).abs() < 1e-12 && (lat - lat2).abs() < 1e-12);
        }
    }

    #[test]
    fn centre_is_the_origin() {
        let bbox = Bounds { west: 6.91, south: 46.43, east: 6.93, north: 46.44 };
        let f = Frame::centred(&bbox);
        let r = f.rect(&bbox);
        assert!((r.x0 + r.x1).abs() < 1e-9 && (r.y0 + r.y1).abs() < 1e-9);
        assert!(r.width() > 1000.0 && r.width() < 2000.0, "{}", r.width());
        assert!((r.height() - 0.01 * DEG_M).abs() < 1e-9);
    }
}
