//! The standards: the dimensions and rules more than one step reads.
//!
//! How thick a deck is, how high a tube, how far a kerb stands proud, how
//! narrow a pavement may be. A step module imports no other step module, so
//! a dimension two steps share lives here rather than in either of them:
//! the only dependencies between steps are the layers [`crate::pipeline`]
//! passes.

use crate::poly;
use crate::width::{self, Family};
use crate::world::{Kind, Station};

/// Station spacing along an axis, in metres: the profile's resolution.
/// Fine enough that a 4 m node step at a 6 % ceiling is a quarter of a
/// metre; coarse enough that a town's road network is tens of thousands of
/// stations, not hundreds.
pub const NODE_M: f64 = 4.0;

/// The height off the ground, in metres, at which a station of a mapped
/// structure span is a deck (above) or a bore (below) rather than grade.
/// Below it the annotation degrades to ground: a structure that never
/// leaves the ground by half a metre is a culvert, and a culvert is ground.
pub const STRUCTURE_MIN_M: f64 = 0.5;

/// How far a surface must stand **clear of** the ground, in metres, before a
/// deck is the honest answer rather than an embankment.
///
/// It is [`MAX_BATTER_FACE_M`]: the tallest earthwork face the earthwork
/// builds. Below it the ground closes the gap with a batter; above it the
/// ground is *walled*, and a wall carrying a road across a gully is a deck
/// drawn wrong. So the threshold is read off a construction that already
/// exists rather than fitted to a population (the server, which fitted its
/// own to its at-grade nodes, found 4.0 m).
///
/// Distinct from [`STRUCTURE_MIN_M`], which is half a metre and answers a
/// different question: that one flags a *station* of a run, this one decides
/// whether the run is a structure at all.
pub const DECK_STANDOFF_M: f64 = MAX_BATTER_FACE_M;

/// The headroom a roadway needs over the roadway beneath it, in metres:
/// the Swiss norm's 4.5 m plus a construction margin, and the server's
/// number.
pub const ROAD_CLEARANCE_M: f64 = 5.0;

/// The headroom anything needs over a railway, in metres: more than a
/// road's, for the catenary. The server's `priors::RAIL_CLEARANCE_M`.
pub const RAIL_CLEARANCE_M: f64 = 7.0;

/// How thick a road deck is, in metres: the slab, its beams and its
/// bearings, as one number.
pub const DECK_THICKNESS_M: f64 = 1.5;

/// How thick a footbridge is, in metres. [`DECK_THICKNESS_M`] is a road
/// bridge's: 1.5 m of beam under a 3 m footway is not a footbridge but a
/// wall with a path on it, and most mapped spans are pedestrian.
pub const WALK_DECK_M: f64 = 0.4;

/// How high a bore is inside, in metres, from the roadway to the crown.
pub const TUNNEL_HEIGHT_M: f64 = 5.0;

/// The same for a way a person walks through: a subway, a covered stair, a
/// passage under a building. The mirror of [`WALK_DECK_M`], and needed for
/// the same reason: most mapped tunnel spans are footways, steps and paths,
/// and five metres of tube is not what any of them is.
pub const WALK_TUNNEL_M: f64 = 2.5;

/// How high a standard-gauge railway's bore is inside, in metres, from the
/// track to the crown: the loading gauge and the overhead line over it,
/// which a road's [`TUNNEL_HEIGHT_M`] does not hold. The same headroom
/// [`crate::standard::RAIL_CLEARANCE_M`] asks of a road over the rails, less
/// the margin a road bridge's soffit keeps.
pub const RAIL_TUNNEL_M: f64 = 6.0;

/// The same for metre gauge and a funicular: a smaller car under a lower
/// wire, as high inside as a road tunnel.
pub const NARROW_RAIL_TUNNEL_M: f64 = 5.0;

/// How far a railway's structure reaches past its track zone on each side,
/// in metres: the edge beam, the cable trough and the walkway a real deck
/// carries, and the same clearance inside a bore (the server's
/// `STRUCTURE_SHOULDER_M`). Without it a metre-gauge viaduct on a
/// hillside is 2.6 m wide and 1.5 m deep, and reads as a wall rather than a
/// bridge. The track bed over the structure is swept to the same width: on
/// a deck the ballast runs to the parapet.
pub const RAIL_SHOULDER_M: f64 = 1.0;

/// How high a bore is inside for a way of `class`, in metres: a walk's
/// passage, a railway's tube, or a road tunnel. One answer for the
/// structure step, which draws the tube, the partition step, which decides
/// where a tube fits, and the crossing step, which clears one.
pub fn tube_m(class: &str) -> f64 {
    match (width::family(class), class) {
        (Family::Walk, _) => WALK_TUNNEL_M,
        (Family::Rail, "narrow_gauge" | "funicular") => NARROW_RAIL_TUNNEL_M,
        (Family::Rail, _) => RAIL_TUNNEL_M,
        (Family::Carriageway, _) => TUNNEL_HEIGHT_M,
    }
}

/// Ground cover a bore keeps between its roof and the surface above it, in
/// metres: enough that what rides over it has something to ride on.
pub const TUNNEL_COVER_M: f64 = 0.5;

/// How far a surface of `class` must run **below** the reference before a
/// bore is the honest answer rather than a cutting: the road, its own tube
/// over it ([`tube_m`]), and the cover over that. Shallower
/// there is nothing to drive through, and a cutting is what is there.
///
/// The mirror of [`DECK_STANDOFF_M`], and asymmetric with it *for a reason*
/// rather than by calibration: a fill becomes a wall at the tallest face the
/// earthwork builds, while a cut stays a cutting until a tube fits under it.
pub fn bore_cover_m(class: &str) -> f64 {
    tube_m(class) + TUNNEL_COVER_M
}

/// Whether a tunnel run of a way of `class`, over `stations`, is a
/// **gallery**: the road goes under the ground somewhere, by more than
/// [`STRUCTURE_MIN_M`], and its tube fits under it nowhere.
///
/// There is no hill to bore through, and still the source says the road is
/// covered: a gallery against a slope, a covered cutting, a road under a
/// deck or a building the terrain model does not have. Drawn as a bore it
/// would draw no tube, since none fits, and the terrain would lie on the
/// road end to end: a road vanishing into the ground with no entrance. As a
/// gallery the ground is opened over it ([`crate::portal::Portals`]) and the
/// tube stands in the trench, its roof out in the open where the ground is
/// lower than it.
///
/// One rule for the ground, which is opened, and for the structure step,
/// which draws the tube — over the same stations, the run and its abutments.
pub fn is_gallery(class: &str, stations: &[Station]) -> bool {
    let tube = tube_m(class);
    stations.iter().any(|st| st.ground - st.h > STRUCTURE_MIN_M)
        && stations.iter().all(|st| st.ground - st.h - tube < 0.0)
}

/// The station ranges of `p`'s galleries, abutments included: the tunnel
/// runs [`is_gallery`] says are galleries. A walk's spans are not solved and
/// are not here.
pub fn gallery_runs(p: &crate::world::Profile) -> Vec<(usize, usize)> {
    p.runs()
        .into_iter()
        .filter(|r| matches!(r.2, Kind::Tunnel(_)))
        .map(|(k0, k1, _)| p.with_abutments(k0, k1))
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
/// it: one kerb face.
pub const KERB_RISE_M: f64 = 0.12;

/// How far past the asphalt's edge, in metres, a carriageway's
/// cross-section carries the pavement level with it. It is
/// [`WALL_REACH_M`], the reach the room step paves to: what that step paves
/// from a kerb — the band, the rungs to a facade, a mapped sidewalk beside
/// them — is lifted with the road, and nothing built there is left behind.
/// It is a *plateau*, so it is not free: on a 30 % flank every metre of it
/// is another metre of bench to cut, and a wider one leaves a wall at its
/// edge where this one leaves a face.
pub const ROOM_REACH_M: f64 = WALL_REACH_M;

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
/// Past this the ground in front of a house is its own. Shorter leaves a
/// bare strip along every block whose fronts stand a little farther back;
/// longer paves whole forecourts.
pub const WALL_REACH_M: f64 = 6.0;

/// The narrowest hole worth paving, in metres: [`WALK_MIN_M`], the
/// narrowest pavement the model draws anywhere.
///
/// [`crate::room::ISLAND_M2`] bounds a hole from above — past it the hole is
/// a lawn or a courtyard rather than a traffic island — and this bounds it
/// from below. Without it a boolean's leftover between two ribbons passes
/// every other test (small, near a kerb, bordering asphalt) and is paved as
/// an island: a place a person could not stand, which over a cutting the
/// ground must then wall all the way round. The test is [`wide_enough`].
///
/// **It governs a hole and not the finished pavement.** The strip along a
/// house front is deliberately narrow — squeezed between the kerb and a
/// wall two metres off the axis it can be a decimetre wide — and a notch in
/// a facade is a small isolated patch by construction. Narrow, short and
/// isolated are all shapes a *real* pavement takes, so telling a scrap from
/// a place in the finished pavement needs to know which construction made
/// it, which the room step's `scraps` records per source.
pub const PAVEMENT_MIN_M: f64 = WALK_MIN_M;

/// Whether `region` can hold a pavement at all: whether anything of it
/// survives being cut back by half [`PAVEMENT_MIN_M`].
///
/// A scrap is **thin, not small** — a traffic island of three square metres
/// is a place and a forty-metre thread of the same area is not — so the
/// test is an erosion and not an area. The cheap ratio first: `2A/P` is a
/// region's width where it is uniformly thin, and a region twice the minimum
/// wide by that measure is taken as passing, so only the suspicious ones
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

/// The slope of an earthwork face, as run over rise: 1 in 2.5. Past the
/// room's reach the walk comes down a face of exactly this slope, so it is
/// as wide as the drop it has to close and no wider. A band of fixed width
/// would not be: on a steep flank it comes out steeper than the wall it is
/// there to avoid.
pub const EARTHWORK_BATTER: f64 = 2.5;

/// The tallest earthwork face, in metres, before the bench is walled at
/// its edge instead. It does two things. A walk band past the room's reach
/// that stands more than one face from the road beside it is not that
/// road's pavement and drapes — without that test a face on a flank steeper
/// than 1 in 2.5 never daylights at all, and the field carries a road's
/// height far up the hillside. And a vertex of the room itself standing
/// further than this from the ground is counted as `walled`: the ground's
/// answer there is a wall, not a batter.
pub const MAX_BATTER_FACE_M: f64 = 3.0;
