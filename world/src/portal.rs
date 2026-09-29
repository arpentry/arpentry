//! The portals: where a tube opens, and the galleries whose footprint the
//! ground is opened over.
//!
//! The arrangement builds one [`Portals`] from the partition's span pieces
//! and profiles, cuts a face for every gallery's footprint and hands it on:
//! the earthwork runs no batter across a mouth, and the bench closes the
//! ground onto the tube's section there rather than onto the road.

use serde::{Deserialize, Serialize};
use crate::line::{self, Nearest};
use crate::poly::{self, Pt, Shapes};
use crate::standard::{gallery_runs, half_width_m, tube_m};
use crate::world::{Kind, Polyline2, Profiles};

/// How far, in metres, a point of the room's outline may stand off a
/// portal's cap and still be on it: the polygon kernel's rounding and the
/// booleans the cap has been through, far short of anything built.
const MOUTH_EPS_M: f64 = 0.05;

/// Where a ground piece ends against a tunnel: its square cap across the
/// road, which the tube's mouth stands in. The cap is the room's outline
/// there, and it must not be closed as if it were a cutting's end.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Mouth {
    at: Pt,
    /// The unit direction into the tunnel.
    into: Pt,
    half: f64,
    /// How high the tube is inside.
    tube: f64,
}

/// Every mouth of the world's tunnel spans: both ends of each.
pub fn mouths(spans: &[Polyline2]) -> Vec<Mouth> {
    spans
        .iter()
        .filter(|w| matches!(w.kind, Kind::Tunnel(_)) && w.pts.len() >= 2)
        .flat_map(|w| {
            let n = w.pts.len();
            let (half, tube) = (w.width_m / 2.0, tube_m(&w.class));
            [(w.pts[0], w.pts[1]), (w.pts[n - 1], w.pts[n - 2])].map(|(at, q)| Mouth {
                at,
                into: line::unit([q[0] - at[0], q[1] - at[1]]),
                half,
                tube,
            })
        })
        .collect()
}

/// Everywhere the ground must leave a tube room: the mouths of the opened
/// portals, and the galleries ([`crate::standard::is_gallery`]), whose
/// footprints are cut out of the ground whole.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Portals {
    mouths: Vec<Mouth>,
    /// The galleries' axes, and per segment the roadway's height at its two
    /// ends, the half-width of the tube and its height.
    pub(crate) gallery: Nearest,
    pub(crate) crowns: Vec<(f64, f64, f64, f64)>,
}

impl Portals {
    /// The portals of `spans` and the galleries of `profiles`, and the
    /// galleries' footprints — what the ground is to be opened over.
    pub fn new(spans: &[Polyline2], profiles: &Profiles) -> (Portals, Shapes) {
        let mut out = Portals { mouths: mouths(spans), ..Portals::default() };
        let mut footprints: Shapes = Vec::new();
        for p in &profiles.profiles {
            let (half, tube) = (half_width_m(&p.class, p.width_m), tube_m(&p.class));
            for (a, b) in gallery_runs(p) {
                let st = &p.stations[a..=b];
                for w in st.windows(2) {
                    out.gallery.push(w[0].p, w[1].p);
                    out.crowns.push((w[0].h, w[1].h, half, tube));
                }
                let axis: Vec<Pt> = st.iter().map(|x| x.p).collect();
                footprints.extend(poly::buffer_line_capped(&axis, 2.0 * half, [false, false]));
            }
        }
        (out, poly::union_all(&footprints))
    }

    /// The tube's section at `q`, as `(floor, roof)`, if `q` stands where a
    /// tube opens: across a mouth, the room's own height there (`room_h`)
    /// and a tube over it; on a gallery's footprint, its roadway and a tube
    /// over that.
    pub(crate) fn section(&self, q: Pt, room_h: f64) -> Option<(f64, f64)> {
        if let Some(tube) = on_mouth(&self.mouths, q) {
            return Some((room_h, room_h + tube));
        }
        let (i, t, d) = self.gallery.of(q, GALLERY_LIMIT_M)?;
        let (ha, hb, half, tube) = self.crowns[i];
        let floor = ha + (hb - ha) * t;
        (d <= half + MOUTH_EPS_M).then_some((floor, floor + tube))
    }

    /// Whether `q` stands where a tube opens.
    pub(crate) fn open(&self, q: Pt) -> bool {
        self.section(q, 0.0).is_some()
    }
}

/// How far from a gallery's axis, in metres, a point is asked about at all:
/// past the widest structure's half-width nothing is a gallery's edge.
const GALLERY_LIMIT_M: f64 = 8.0;

/// The tube height of the mouth `q` lies across, if it lies across one.
fn on_mouth(mouths: &[Mouth], q: Pt) -> Option<f64> {
    mouths
        .iter()
        .find(|m| {
            let d = [q[0] - m.at[0], q[1] - m.at[1]];
            let along = d[0] * m.into[0] + d[1] * m.into[1];
            let across = d[0] * m.into[1] - d[1] * m.into[0];
            along.abs() <= MOUTH_EPS_M && across.abs() <= m.half + MOUTH_EPS_M
        })
        .map(|m| m.tube)
}
