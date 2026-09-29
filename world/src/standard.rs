//! The standards: the dimensions and rules more than one step reads.
//!
//! How thick a deck is, how high a tube, how far a kerb stands proud, how
//! narrow a pavement may be. Each was a constant of the step that first
//! needed it, and every other step that needed it imported that step —
//! so the steps depended on each other through their constants, where
//! [`crate::pipeline`] could not see it. They live here now, and a step
//! module imports no other step module.

use crate::grade::STRUCTURE_MIN_M;
use crate::poly;
use crate::width::{self, Family};
use crate::world::{Kind, Station};

/// How thick a road deck is, in metres
/// (`data/plans/surface-leaves-the-plane-2026-09-08.md` §5): the slab, its
/// beams and its bearings, as one number.
pub const DECK_THICKNESS_M: f64 = 1.5;

/// How thick a footbridge is. The plan carries one deck thickness, and it
/// is a road bridge's: 1.5 m of beam under a 3 m footway is not a
/// footbridge but a wall with a path on it, and most of the box's spans
/// are pedestrian. A prior of this step's own, named here.
pub const WALK_DECK_M: f64 = 0.4;

/// How high a bore is inside, in metres, from the roadway to the crown.
pub const TUNNEL_HEIGHT_M: f64 = 5.0;

/// The same for a way a person walks through: a subway, a covered stair, a
/// passage under a building. The mirror of [`WALK_DECK_M`], and needed for
/// the same reason — most of the loop box's tunnel spans are footways, steps
/// and paths, and five metres of tube is not what any of them is.
pub const WALK_TUNNEL_M: f64 = 2.5;

/// How high a standard-gauge railway's bore is inside, in metres, from the
/// track to the crown: the loading gauge and the overhead line over it,
/// which a road's [`TUNNEL_HEIGHT_M`] does not hold. The same headroom
/// [`crate::crossing::RAIL_CLEARANCE_M`] asks of a road over the rails, less
/// the margin a road bridge's soffit keeps.
pub const RAIL_TUNNEL_M: f64 = 6.0;

/// The same for metre gauge and a funicular: a smaller car under a lower
/// wire, as high inside as a road tunnel.
pub const NARROW_RAIL_TUNNEL_M: f64 = 5.0;

/// How far a railway's structure reaches past its track zone on each side,
/// in metres: the edge beam, the cable trough and the walkway a real deck
/// carries, and the same clearance inside a bore. The server's
/// `STRUCTURE_SHOULDER_M`, which its rail comments say the structure sweep
/// adds back and its code never did. Without it a metre-gauge viaduct on a
/// hillside is 2.6 m wide and 1.5 m deep, and reads as a wall rather than a
/// bridge. The track bed over the structure is swept to the same width: on
/// a deck the ballast runs to the parapet.
pub const RAIL_SHOULDER_M: f64 = 1.0;

/// How high a bore is inside for a way of `class`, in metres: a walk's
/// passage, a railway's tube, or a road tunnel. One answer for the
/// structure step, which draws the tube, the partition, which decides
/// where a tube fits, and the crossing, which clears one.
pub fn tube_m(class: &str) -> f64 {
    match (width::family(class), class) {
        (Family::Walk, _) => WALK_TUNNEL_M,
        (Family::Rail, "narrow_gauge" | "funicular") => NARROW_RAIL_TUNNEL_M,
        (Family::Rail, _) => RAIL_TUNNEL_M,
        (Family::Carriageway, _) => TUNNEL_HEIGHT_M,
    }
}

/// Whether a tunnel run of a way of `class`, over `stations`, is a
/// **gallery**: the road goes under the ground somewhere, by more than
/// [`STRUCTURE_MIN_M`], and its tube fits under it nowhere.
///
/// There is no hill to bore through, and still the source says the road is
/// covered: a gallery against a slope, a covered cutting, a road under a
/// deck or a building the terrain model does not have. Drawn as a bore it
/// drew nothing — the tube never fitted, so no tube, and the terrain lay on
/// the road end to end: a road that vanished into the ground with no
/// entrance and no exit (`mouths`). Drawn as a gallery it is what it is: the
/// ground is opened over it ([`crate::bench`]) and the tube stands in the
/// trench, its roof out in the open where the ground is lower than it.
///
/// One rule for the bench, which opens the ground, and for this step, which
/// draws the tube — over the same stations, the run and its abutments.
pub fn is_gallery(class: &str, stations: &[Station]) -> bool {
    let tube = tube_m(class);
    stations.iter().any(|st| st.ground - st.h > STRUCTURE_MIN_M)
        && stations.iter().all(|st| st.ground - st.h - tube < 0.0)
}

/// The station ranges of `p`'s galleries, abutments included: the tunnel
/// runs [`is_gallery`] says are galleries. A walk's spans are not solved and
/// are not here.
pub fn gallery_runs(p: &crate::world::Profile) -> Vec<(usize, usize)> {
    let last = p.stations.len().saturating_sub(1);
    p.runs()
        .into_iter()
        .filter(|r| matches!(r.2, Kind::Tunnel(_)))
        .map(|(k0, k1, _)| (k0.saturating_sub(1), (k1 + 1).min(last)))
        .filter(|&(a, b)| b > a && is_gallery(&p.class, &p.stations[a..=b]))
        .collect()
}

/// Half the width of the structure a way of `class` and `width_m` stands
/// on or runs through: the way's own for a road or a walk, the track zone
/// and [`RAIL_SHOULDER_M`] for a railway.
pub fn half_width_m(class: &str, width_m: f64) -> f64 {
    width_m / 2.0 + if width::family(class) == Family::Rail { RAIL_SHOULDER_M } else { 0.0 }
}

/// How far, in metres, the pavement stands above the carriageway beside
/// it: one kerb face (`data/plans/surface-leaves-the-plane-2026-09-08.md`
/// §5).
pub const KERB_RISE_M: f64 = 0.12;

/// How far past the asphalt's edge, in metres, a carriageway's
/// cross-section carries the pavement level with it. It is the room
/// step's own `WALL_REACH_M`: what that step paves from a kerb — the
/// band, the rungs to a facade, a mapped sidewalk beside them — is what
/// this one lifts with the road, and nothing built there is left behind.
/// It is a *plateau*, so it is not free: on a 30 % flank every metre of
/// it is another metre of bench to cut, and 10 m of it left a wall at
/// its edge where 6 m leaves a face.
pub const ROOM_REACH_M: f64 = 6.0;

/// How far, in metres, the over-a-span mask is grown past the span's own
/// rim before a vertex is asked whether it stands on one.
///
/// A vertex of the meshed sheet lands exactly on the region's ring, and a
/// crossings test on a ring is ambiguous there. A centimetre settles it and
/// is far under anything the answer could be confused with: the nearest
/// real ground to a deck's edge is the abutment, and there the two are at
/// one height anyway.
pub const OVER_RIM_M: f64 = 0.01;

/// Sample spacing along a pedestrian way, in metres.
pub const STATION_M: f64 = 1.0;

/// The narrowest pavement drawn, in metres: what a sidewalk mapped under the
/// asphalt still gets outside the kerb.
pub const WALK_MIN_M: f64 = 0.8;

/// A hole in the pavement smaller than this, in square metres, is a notch
/// between two rungs whose feet jumped from one road to another, or the
/// hairline where two pieces of the ladder met — not a courtyard the
/// footways enclose, which is tens of square metres — and it is filled.
pub const PAVEMENT_HOLE_M2: f64 = 2.0;

/// How far outside the kerb a wall still bounds the street, in metres.
/// Past this the ground in front of a house is its own. Four metres left
/// a bare strip along every block whose fronts stand a little farther
/// back; eight paved whole forecourts and grew the pavement by a third.
pub const WALL_REACH_M: f64 = 6.0;

/// The narrowest hole worth paving, in metres: [`crate::standard::WALK_MIN_M`], the
/// narrowest pavement the model draws anywhere.
///
/// [`ISLAND_M2`] bounds a hole from above — past it the hole is a lawn or a
/// courtyard rather than a traffic island — and nothing bounded it from
/// below, so a boolean's leftover between two ribbons passed every test
/// (small, near a kerb, bordering asphalt) and was paved as an island. On
/// the Montreux junction that was **29 of 40 islands**, and all of them
/// together were 10 m² of the 986.
///
/// **It governs a hole and not the finished pavement.** The same test at
/// the end of the chain is wrong, and four specimens say so: the strip
/// along a house front is deliberately narrow — squeezed between the kerb
/// and a wall two metres off the axis it can be a decimetre wide — and a
/// notch in a facade is a small isolated patch by construction. Narrow,
/// short and isolated are all shapes a *real* pavement takes here, so
/// telling a scrap from a place needs to know which construction made it —
/// which `scraps` on the summary line now records, per source.
///
/// The same missing bound shows at the other end of the box. A
/// boolean between two ribbons leaves scraps, and at the Montreux
/// overbridge two of them, **1.03 m² and 2.84 m², 0.41 m and 0.71 m
/// wide**, sat between the deck's ribbon and the ballast, passed every test
/// (small, near a kerb, bordering asphalt) and were paved as traffic
/// islands. They are not islands: they are places a person could not stand,
/// and up at road level over a cutting the ground then had to wall a square
/// metre of pavement all the way round.
///
/// The test is an **erosion**, not an area: a scrap is thin, not small, and
/// a hole that does not survive being cut back by half the narrowest
/// pavement has no pavement in it to draw.
pub const PAVEMENT_MIN_M: f64 = WALK_MIN_M;

/// Whether `region` can hold a pavement at all: whether anything of it
/// survives being cut back by half [`PAVEMENT_MIN_M`].
///
/// A scrap is **thin, not small** — a traffic island of three square metres
/// is a place and a forty-metre thread of the same area is not — so the
/// test is an erosion and not an area. The cheap ratio first: `2A/P` is a
/// region's width where it is thin, and nothing twice the minimum wide by
/// that measure has ever failed the erosion, so only the suspicious ones
/// cost a boolean.
pub fn wide_enough(region: &poly::Shape) -> bool {
    let area = poly::area(std::slice::from_ref(region));
    let perimeter: f64 = region
        .iter()
        .flat_map(|ring| {
            (0..ring.len()).map(move |i| {
                let (u, v) = (ring[i], ring[(i + 1) % ring.len()]);
                (v[0] - u[0]).hypot(v[1] - u[1])
            })
        })
        .sum();
    if perimeter > 0.0 && 2.0 * area / perimeter >= 2.0 * PAVEMENT_MIN_M {
        return true;
    }
    !poly::erode(&vec![region.clone()], PAVEMENT_MIN_M / 2.0).is_empty()
}

/// Longest break between two attached stretches that is bridged on
/// distance alone, in metres: the corner a pavement wraps, the arc between
/// two legs of a roundabout (the server's `WALK_CORNER_MAX_M`).
pub const SIDEWALK_BRIDGE_M: f64 = 25.0;

/// Half-width of a rung, in metres. A full metre either side of the station
/// so that consecutive rungs overlap even on the outside of a bend of eight
/// metres radius at the full reach.
pub const RUNG_HALF_M: f64 = 1.0;

/// The slope of an earthwork face, as run over rise: 1 in 2.5
/// (`data/plans/surface-leaves-the-plane-2026-09-08.md` §5). Past the
/// room's reach the walk comes down a face of exactly this slope, so it
/// is as wide as the drop it has to close and no wider. A band of fixed
/// width was tried first and rejected: 6 m of it on the box's 30 % flank
/// came out at 103 %, steeper than the wall it was there to avoid, and
/// the `step` check said so before the eye did.
pub const EARTHWORK_BATTER: f64 = 2.5;

/// The tallest earthwork face, in metres, before the bench is walled at
/// its edge instead (`data/plans/surface-leaves-the-plane-2026-09-08.md`
/// §5). It does two things here. A walk band past the room's reach that
/// stands more than one face from the road beside it is not that road's
/// pavement and drapes — without that test a face on the box's flank
/// never daylights at all, because the mountain rises faster than 1 in
/// 2.5, and the first run over the loop box read 225 m of cut where the
/// field had carried a road's height half a kilometre up the hillside.
/// And a vertex of the room itself standing further than this from the
/// ground is counted as `walled`: the ground's answer there is a wall,
/// not a batter.
pub const MAX_BENCH_FACE_M: f64 = 3.0;
