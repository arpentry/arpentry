//! The engineered ground: the natural ground with the earthwork the lifted
//! room owes it, as a function of the point.
//!
//! The earthwork step builds it from the room's outline and the bench reads
//! it where it closes the room onto the ground, so it is a module of its own
//! rather than either step's.

use serde::{Deserialize, Serialize};

use crate::line::{fade, Nearest};
use crate::poly::Pt;
use crate::standard::{EARTHWORK_BATTER, MAX_BATTER_FACE_M};

/// The engineered ground: the natural ground plus an earthwork **residual**
/// that is pinned at the room's outline and falls to nothing at 1 in
/// [`EARTHWORK_BATTER`].
///
/// At an outline vertex the residual is the room's height less the natural
/// ground, held to one face ([`MAX_BATTER_FACE_M`]) either way; a wall at the
/// edge closes whatever is left over. Out from each outline segment it falls
/// at the batter's slope, perpendicular to the segment, and a point takes the
/// nearest segment's batter blended with any segment nearly as near — over
/// `EARTH_BLEND_M`, narrowing to nothing at the outline, so on the outline
/// it is the pins' own interpolation and the whole ground is one function.
/// Three things follow:
///
/// - **It is continuous.** The nearest segment alone would step wherever two
///   segments are equidistant and answer differently — the inside of every
///   kerb return — and between a walled segment and a battered one, whose
///   pins differ by the whole of a drop. The blend removes the first, and a
///   pin that is clamped rather than refused removes the second: a drop past
///   one face is a face of batter and a wall for the rest, not all wall or
///   none.
/// - **It always daylights, within 7.5 m.** A pin is at most one face, so its
///   batter is spent at `B · MAX_BATTER_FACE_M`. A face at an *absolute*
///   1 in 2.5 would never meet a hill steeper than that, and would have to be
///   cut off with a lip standing in it.
/// - **Pins do not reach along the kerb.** A cone from every pin (the
///   steepest-allowed extension) would couple neighbouring pins: where a
///   kerb's cut changes by more than a batter's slope along its own length,
///   one pin's cone overrides the next, and each override is a wall at the
///   kerb.
///
/// **The batter is relative to the natural ground, not absolute.** On flat
/// ground the two are the same 1 in 2.5. Across a 30 % hill the cut face
/// stands at 70 % and meets the ground 2 m out; at an absolute 1 in 2.5 it
/// never would.
///
/// It is a function of the point, not a mesh, so anything may be *proven*
/// to lie on it, which is the property the terrain step set out with and
/// the one the crate is built on.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Ground {
    at: Nearest,
    /// Per outline segment, the residual at its two ends.
    e: Vec<(f64, f64)>,
}

/// How far, in metres, a batter reaches before it is spent: one face at the
/// batter's slope.
const EARTH_REACH_M: f64 = EARTHWORK_BATTER * MAX_BATTER_FACE_M;

/// The band, in metres of distance, over which the batter of one outline
/// segment hands over to the next where the nearest changes ([`Ground`]).
const EARTH_BLEND_M: f64 = 1.0;

impl Ground {
    /// The ground benched to the room's outline: one segment per outline
    /// edge of the one mesh, as `(a, b, [room, natural] at a, the same at
    /// b)`. A segment across a tunnel's mouth is pinned at no residual — no
    /// batter runs into the tube — and the wall over the mouth closes the
    /// hill down to its roof.
    pub fn of_edges(edges: &[(Pt, Pt, [f64; 2], [f64; 2])], portals: &crate::portal::Portals) -> Ground {
        let mut g = Ground::default();
        let pin = |[room, natural]: [f64; 2]| (room - natural).clamp(-MAX_BATTER_FACE_M, MAX_BATTER_FACE_M);
        for &(a, b, at_a, at_b) in edges {
            g.at.push(a, b);
            let mouth = portals.open(a) && portals.open(b);
            g.e.push(if mouth { (0.0, 0.0) } else { (pin(at_a), pin(at_b)) });
        }
        g
    }

    /// How many outline pieces the ground is benched to.
    pub fn len(&self) -> usize {
        self.e.len()
    }

    pub fn is_empty(&self) -> bool {
        self.e.is_empty()
    }

    /// The earthwork residual at `p`: what the ground there stands off the
    /// natural ground.
    pub fn residual(&self, p: Pt) -> f64 {
        let mut near: Vec<(f64, f64)> = Vec::new();
        self.at.within(p, EARTH_REACH_M + EARTH_BLEND_M, |i, t, d| {
            // **A batter does not cross the room.** The outline runs with the
            // room on its left, so a point whose foot is inside a segment and
            // which lies on the segment's left is across the paving from it.
            // Beyond a segment's end its batter is its end vertex's, which a
            // neighbouring segment shares, so a corner stays continuous.
            let (a, b) = self.at.seg[i];
            let left = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]);
            if t > 0.0 && t < 1.0 && left > 0.0 {
                return;
            }
            let (ea, eb) = self.e[i];
            let e = ea + (eb - ea) * t;
            near.push((e.signum() * (e.abs() - d / EARTHWORK_BATTER).max(0.0), d));
        });
        let Some(nearest) = near.iter().map(|x| x.1).min_by(f64::total_cmp) else {
            return 0.0;
        };
        // The segments nearly as near as the nearest, blended by how nearly:
        // continuous where the nearest changes, which is where the nearest
        // alone would step. Sorted first, so the sum is a function of the
        // segments and not of the index's visiting order.
        near.sort_by(|x, y| x.1.total_cmp(&y.1).then(x.0.total_cmp(&y.0)));
        // **The band narrows to nothing at the outline**, so there the
        // nearest segment alone answers — the linear interpolation of its own
        // pins, which is the edge the mesh drew — and a vertex pinned on it is
        // the limit of the field around it. At a full metre everywhere, a
        // segment a few decimetres off with a different pin would be blended
        // into the outline itself, and the ground would stand a jump off
        // every pin.
        let band = EARTH_BLEND_M.min(nearest);
        let (mut sum, mut weight) = (0.0, 0.0);
        for (e, d) in near {
            let w = if band > 0.0 { fade((d - nearest) / band) } else { (d <= nearest) as u8 as f64 };
            sum += w * e;
            weight += w;
        }
        sum / weight
    }

    /// The engineered height at `p`, whose natural ground is `natural`.
    pub fn at(&self, p: Pt, natural: f64) -> f64 {
        natural + self.residual(p)
    }
}

