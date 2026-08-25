//! The street's cross-section: one allotment of the room, spent in order.
//!
//! docs/ROADS.md invariant 1 asks for **one cross-section function** — one
//! derivation from data and priors to the band stack and total width, read by
//! everything that needs a width — and says in the same breath that a sidewalk
//! is part of that cross-section and not a feature of its own. Until this
//! module the code said otherwise: `carriageway::sections_along` spent the
//! facade room on asphalt, and `walkway::seat` re-measured the same facades
//! with a different reach for what it thought was left. Two measurements of one
//! street, sequenced so that neither could see what the other had spent.
//!
//! `sections_along`'s own doc comment already named the fix and left it unbuilt
//! — *"Phase 2 has only asphalt to spend it on, so the whole cap lands here;
//! when the walk band and the per-side bench exist they take their share first
//! and this becomes the remainder."* This is that.
//!
//! **The order is the model, and asphalt is first in it.** A footprint carries
//! its own plan error while a carriageway width is a survey prior, so the road
//! takes its prior and the pavement takes what is left; only when the wall
//! stands closer than the prior itself does the asphalt narrow. Where the
//! remainder is under [`priors::WALK_MIN_WIDTH_M`] there is simply no pavement
//! on that side, which is what a street too narrow for one looks like.
//!
//! **Why widening the query cannot move the carriageway.** The pedestrian
//! allotment needs to see further out than the asphalt one, so the room is now
//! read at a reach of `half + want + FACADE_CLEAR_M` rather than
//! `half + FACADE_CLEAR_M`. That cannot change what the asphalt gets, and the
//! proof is one line of [`allot_side`]: it clamps `room − FACADE_CLEAR_M` to at
//! most `prior`. If a wall stands inside the *narrow* reach both queries return
//! the same true distance; if it does not, the narrow query returns
//! `half + FACADE_CLEAR_M` and the wide one returns something larger, and both
//! clamp to `prior`. So P1 is byte-identical for every carriageway **by
//! construction**, not by measurement — which is the only kind of A/B control
//! worth having for a refactor this wide.

use geo_types::Coord;

use crate::assemble::facades::{Facades, Section};
use crate::priors;
use crate::scene::{Corridor, DEG_M};

/// One station's whole street, from the centerline outward on each side.
///
/// Three bands, in the order the room is spent. They are stored separately
/// rather than as running offsets because each has a different consumer — the
/// asphalt is buffered, the pavement is buffered beside it, the verge is what
/// the bench's batter is allowed to eat — and a consumer that had to subtract
/// its way to its own band would be re-deriving the allotment.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StreetSection {
    /// Asphalt half-width per side — exactly what [`Section`] has always
    /// meant, and what `carriageway_sources` reads.
    pub carriage: Section,
    /// Pavement width per side, metres, measured *outward from the carriageway
    /// edge*. Zero where the street has no pavement on that side: no evidence,
    /// or no room left for one.
    pub walk: [f64; 2],
    /// What remains between the pavement's outer edge and the facade
    /// clearance, per side.
    pub verge: [f64; 2],
}

impl StreetSection {
    /// The cross-section of a street with room to spare and no pavement — what
    /// every corridor got before there was anything but asphalt to spend on.
    pub fn uniform(half_m: f64) -> StreetSection {
        StreetSection { carriage: Section::uniform(half_m), walk: [0.0; 2], verge: [0.0; 2] }
    }

    /// Where the pavement's centre sits on `side`, as an offset from the
    /// centerline — the kerb plus half the strip.
    ///
    /// **This is what makes `street.kerb_join` zero by construction.** The
    /// pavement's inner edge is `carriage.on(side)` and the carriageway's outer
    /// edge is the same number; there is no second derivation that could put
    /// bare ground between a road and its own pavement.
    pub fn walk_centre(&self, side: usize) -> f64 {
        self.carriage.on(side) + self.walk[side] * 0.5
    }
}

/// The cross-section at every station of one run.
///
/// `want` is the pavement width the pedestrian evidence asks for at each
/// station, per side — `priors::WALK_WIDTH_M` where a way was attached, zero
/// elsewhere. An empty slice means "no pavement anywhere on this run", which is
/// the carriageway-only call and the one that has to stay byte-identical.
///
/// Asphalt only, as before: a railway through a building is a station under its
/// roof, not a formation drawn through a wall, and narrowing the ballast there
/// would shave the platform it stands on.
#[allow(clippy::too_many_arguments)]
pub fn sections_along(
    c: &Corridor,
    stops: &[f64],
    pts: &[Coord],
    half_m: f64,
    want: &[[f64; 2]],
    facades: &Facades,
    no_room: bool,
    scratch: &mut Vec<u32>,
) -> Vec<StreetSection> {
    let asked = |i: usize, side: usize| want.get(i).map_or(0.0, |w| w[side]);
    let open = |i: usize| StreetSection {
        carriage: Section::uniform(half_m),
        // With no walls anywhere the pavement gets exactly what it asked for.
        walk: [strip(asked(i, 0), f64::MAX), strip(asked(i, 1), f64::MAX)],
        verge: [0.0; 2],
    };
    if no_room || facades.is_empty() || c.kind.prior().surface != priors::Surface::Asphalt {
        return (0..stops.len()).map(open).collect();
    }
    let m_lon = DEG_M * c.cos_lat;
    let mut out: Vec<StreetSection> = Vec::with_capacity(stops.len());
    for i in 0..stops.len() {
        // The tangent is a central difference where there is one, so a station
        // reads the direction the road runs *through* it rather than the
        // direction of whichever chord happens to be indexed with it.
        let (j, k) = (i.saturating_sub(1), (i + 1).min(stops.len() - 1));
        if j == k {
            out.push(open(i));
            continue;
        }
        let (dx, dy) = ((pts[k].x - pts[j].x) * m_lon, (pts[k].y - pts[j].y) * DEG_M);
        let len = dx.hypot(dy);
        if !(len > 0.0) {
            out.push(open(i));
            continue;
        }
        // **A station is responsible for its own stretch of centerline**, so it
        // looks at least as far along the road as the gap to its neighbours.
        // That is what makes consecutive stations see the same wall: a facade
        // is caught by every station whose window it falls in, so the two that
        // bracket its ends are both narrowed and the width interpolated between
        // them never crosses it. A shorter window would let a wall between two
        // stations go unseen; a longer one only tapers the street sooner.
        let window = (stops[k] - stops[j])
            .max(super::carriageway::ROOM_WINDOW_MIN_M)
            .min(super::carriageway::ROOM_WINDOW_MAX_M);
        // One query, reaching far enough for the whole cross-section. See the
        // module header for why the wider reach cannot move the asphalt.
        let reach = half_m
            + asked(i, 0).max(asked(i, 1))
            + priors::FACADE_CLEAR_M;
        let room = facades.room(pts[i], c.cos_lat, (dx / len, dy / len), reach, window, scratch);
        let carriage = room.allot(half_m, priors::MIN_CARRIAGEWAY_HALF_M);
        let mut walk = [0.0f64; 2];
        let mut verge = [0.0f64; 2];
        for side in 0..2 {
            // What is left of this side once the wall's clearance and the
            // asphalt have been paid for.
            let spare = (room.on(side) - priors::FACADE_CLEAR_M - carriage.on(side)).max(0.0);
            walk[side] = strip(asked(i, side), spare);
            verge[side] = spare - walk[side];
        }
        out.push(StreetSection { carriage, walk, verge });
    }
    out
}

/// The pavement a side gets: what it asked for, capped by the room left, and
/// nothing at all below the narrowest strip worth drawing.
///
/// Snapped down to the width ladder so a strip that merely varies along a
/// street draws one width instead of a new one at every station
/// ([`priors::quantize_walk_width`]).
fn strip(want_m: f64, spare_m: f64) -> f64 {
    if !(want_m > 0.0) {
        return 0.0;
    }
    let got = want_m.min(spare_m);
    if got < priors::WALK_MIN_WIDTH_M {
        return 0.0;
    }
    priors::quantize_walk_width(got)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn asphalt_takes_its_prior_before_the_pavement_takes_anything() {
        // 10 m of room, a 3 m half-carriageway, 2 m asked for the pavement.
        let s = allot(10.0, 3.0, 2.0);
        assert_eq!(s.carriage.left_m, 3.0);
        assert_eq!(s.walk[0], 2.0);
    }

    #[test]
    fn the_pavement_narrows_before_the_asphalt_does() {
        // 4.9 m of room: the asphalt keeps its 3 m prior and the pavement
        // takes the 1.4 m remainder, snapped down the ladder.
        let s = allot(4.9, 3.0, 2.0);
        assert_eq!(s.carriage.left_m, 3.0);
        assert!(s.walk[0] > 0.0 && s.walk[0] < 2.0);
        assert!(s.walk[0] <= 4.9 - priors::FACADE_CLEAR_M - 3.0 + 1e-9);
    }

    #[test]
    fn a_street_with_no_room_for_a_pavement_simply_has_none() {
        let s = allot(4.0, 3.0, 2.0);
        assert_eq!(s.carriage.left_m, 3.0);
        assert_eq!(s.walk[0], 0.0, "under WALK_MIN_WIDTH_M is no band, not a sliver");
    }

    #[test]
    fn asking_for_a_pavement_never_moves_the_asphalt() {
        // The whole P1 control, as a property: for every room a wall could
        // leave, the carriageway is what it was before the pavement existed.
        for r in 0..200 {
            let room = r as f64 * 0.1;
            let with = allot(room, 3.0, 2.0).carriage.left_m;
            let without = allot(room, 3.0, 0.0).carriage.left_m;
            assert_eq!(with, without, "room {room}");
        }
    }

    #[test]
    fn the_inner_edge_of_the_pavement_is_the_edge_of_the_road() {
        let s = allot(10.0, 3.0, 2.0);
        assert_eq!(s.walk_centre(0) - s.walk[0] * 0.5, s.carriage.on(0));
    }

    /// One station's allotment against a wall `room_m` away on both sides.
    fn allot(room_m: f64, half_m: f64, want_m: f64) -> StreetSection {
        let room = crate::assemble::facades::Room {
            left: room_m,
            right: room_m,
        };
        let carriage = room.allot(half_m, priors::MIN_CARRIAGEWAY_HALF_M);
        let mut walk = [0.0f64; 2];
        let mut verge = [0.0f64; 2];
        for side in 0..2 {
            let spare = (room.on(side) - priors::FACADE_CLEAR_M - carriage.on(side)).max(0.0);
            walk[side] = strip(want_m, spare);
            verge[side] = spare - walk[side];
        }
        StreetSection { carriage, walk, verge }
    }
}
