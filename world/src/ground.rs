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

/// What a metre behind an outline segment, across the room from it, adds to
/// the distance its batter is read at ([`Ground::residual`]): enough that the
/// batter falls there at 1 in 1 — the steepest face the lift lets a surface
/// stand ([`crate::earthwork::STEP_GRADE`]) — instead of 1 in
/// [`EARTHWORK_BATTER`], so a pin of one face is spent one face behind.
const BEHIND_M: f64 = EARTHWORK_BATTER - 1.0;

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
            // How deep it is, `behind`, is the least of its distance off the
            // segment's line and its distance in from either end, so it is
            // nothing on the segment, on its right and beyond its ends — where
            // the batter is its end vertex's, which a neighbouring segment
            // shares — and grows continuously from there. The segment's batter
            // is spent across the room at 1 in 1 rather than 1 in
            // `EARTHWORK_BATTER` ([`BEHIND_M`]), which reaches nothing past one
            // face.
            //
            // **It was a hard cut**, and a cut is a jump for any point on the
            // room's side of a segment that asks the ground at all: passive
            // pavement, which is paving but takes this field, lies there
            // wherever it meets the room beyond a kerb's end. On `net:stub`
            // over a 30 % ramp two corners of one footway triangle, 0.46 m
            // apart either side of the line through a kerb's end, read -1.63
            // and -2.28, and the census charged the triangle as a fin
            // (0.70 m²).
            let (a, b) = self.at.seg[i];
            let len = (b[0] - a[0]).hypot(b[1] - a[1]);
            let left = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]);
            let behind = if len > 0.0 { (left / len).min(t.min(1.0 - t) * len).max(0.0) } else { 0.0 };
            let d = d + BEHIND_M * behind;
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::portal::Portals;

    /// A ground of one segment per edge, `(a, b, pin at a, pin at b)`, over
    /// a natural ground at 0.
    fn ground(edges: &[(Pt, Pt, f64, f64)]) -> Ground {
        let edges: Vec<_> = edges.iter().map(|&(a, b, ea, eb)| (a, b, [ea, 0.0], [eb, 0.0])).collect();
        Ground::of_edges(&edges, &Portals::default())
    }

    /// **The residual is continuous on the room's side of a segment too.**
    /// Passive pavement is paving that takes the ground's field, so it asks
    /// the field on the room's side of the outline: the footway beyond the
    /// end of a stub's kerb, where the kerb runs on and the footway turns in
    /// behind it. A batter simply refused there steps on the line through
    /// the kerb's end — 2 m across a fifth of a millimetre here, and a fin
    /// wherever a footway's triangle straddled it.
    #[test]
    fn the_residual_does_not_step_behind_a_kerb_s_end() {
        // A kerb along y = 0 ending at x = 10, the room north of it, pinned
        // two metres of fill.
        let g = ground(&[([0.0, 0.0], [10.0, 0.0], 2.0, 2.0)]);
        for y in [0.0, 1e-3, 0.3, 0.5, 1.0, 2.0] {
            let (before, after) = (g.residual([10.0 - 1e-4, y]), g.residual([10.0 + 1e-4, y]));
            assert!((before - after).abs() < 1e-3, "at y = {y}: {before:.4} before the kerb's end, {after:.4} past it");
        }
        // And across the segment's own line, inside its ends.
        for x in [0.01, 0.3, 5.0, 9.7, 9.99] {
            let (room, ground) = (g.residual([x, 1e-4]), g.residual([x, -1e-4]));
            assert!((room - ground).abs() < 1e-3, "at x = {x}: {room:.4} on the room's side, {ground:.4} on the ground's");
        }
    }

    /// Nowhere about a kerb's end does the residual climb faster than one
    /// in one: in front of it the batter falls at 1 in `EARTHWORK_BATTER`,
    /// behind it at 1 in 1, and between the two it is continuous. Sampled
    /// on a grid 5 cm apart.
    #[test]
    fn the_residual_about_a_kerb_s_end_is_no_steeper_than_one_in_one() {
        let g = ground(&[([0.0, 0.0], [10.0, 0.0], 3.0, 3.0)]);
        let h = 0.05;
        let mut worst = (0.0f64, [0.0, 0.0]);
        for i in 0..=200 {
            for j in 0..=200 {
                let p = [5.0 + i as f64 * h, -5.0 + j as f64 * h];
                let r = g.residual(p);
                for q in [[p[0] + h, p[1]], [p[0], p[1] + h]] {
                    let grade = (g.residual(q) - r).abs() / h;
                    if grade > worst.0 {
                        worst = (grade, p);
                    }
                }
            }
        }
        assert!(worst.0 <= 1.0 + 1e-6, "the residual climbs {:.3} m/m at {:?}", worst.0, worst.1);
        // Behind the kerb the pin is spent one face in, not a batter's run.
        assert_eq!(g.residual([5.0, 3.0]), 0.0);
        assert!((g.residual([5.0, 1.0]) - 2.0).abs() < 1e-9);
    }
}

