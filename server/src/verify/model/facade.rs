//! I8 — what the ground under a wall is made of.
//!
//! A road's earthwork declares a footprint and the ground inside it is the
//! road's to decide (`ground.footprint` checks the declaration bounds the
//! influence). Nothing says that footprint may not contain a *building*. It
//! routinely does: the bench is the class half-width plus a shoulder plus a
//! verge, wider than the asphalt it carries, and the batter reaches past that
//! again.
//!
//! **The archive cannot answer this.** It carries the ground that was drawn,
//! never the ground that would have been there, and a building anchors at the
//! highest drawn ground under its footprint — so a wall standing on a road's
//! bench looks exactly like a wall standing on a hill. `contact.building_seat`
//! reads 0.011 % with the defect at full strength, because the building simply
//! rides whatever it is given. What is wrong is not the contact, it is the
//! authority: a road decided where a building stands.

use geo_types::Coord;

use crate::dem::Dem;
use crate::scene::DEG_M;
use crate::verify::dist::Dist;
use crate::verify::{Invariant, Metric, Offender, Sense, Worst};

use super::Model;

/// Plan spacing of the walk along a facade, in metres. A bench is metres wide
/// and a batter face metres more, so nothing this check is looking for can fall
/// between two samples; finer only re-measures the same wall.
const SPACING_M: f64 = 2.0;

/// How far a road earthwork may move the ground at a wall, in metres.
///
/// Reasoned, not read off the distribution, for the same reason
/// `order.building_overlap`'s is: the population's shape is a consequence of
/// the terrain the extract happens to sit on, not of the defect. The number
/// that means something is what the *drawn world* can absorb. A building's
/// foundation is extended past the lowest ground under it by the footprint's
/// own relief (`building_mesh`), and `stamp_elevations` rounds that relief to
/// the metre — so under a metre of movement the foundation covers it and
/// nothing is visible. Past it the wall stands on a terrace the hill does not
/// have.
const FACADE_GROUND_M: f64 = 1.0;

/// Cap on facade samples, so a continental extract still answers in bounded
/// time. When it bites the metric says so rather than reading like full
/// coverage.
const MAX_SAMPLES: usize = 2_000_000;

/// **I8, read from the building's side.**
///
/// Walks every footprint edge in the extract and asks how far the engineered
/// ground stands from the reference surface there. Away from every earthwork that
/// is exactly zero — the stack is the identity — so the population scores zeros
/// wherever a building has the ground to itself, and closing the defect moves
/// the number instead of emptying the population (the Phase 0 lesson from
/// `order.building_overlap`).
///
/// The edge rather than the interior, because a wall is where the building
/// meets the ground and where a terrace under it is visible. A footprint whose
/// *interior* is crossed by a bench has that bench at two of its walls too.
pub fn check(m: &Model<'_>) -> Vec<Metric> {
    let mut dist = Dist::metres();
    let mut worst = Worst::new(Sense::HigherIsWorse, 8);
    let mut scratch: Vec<u32> = Vec::new();
    let edges = m.facades.edges();
    let mut dem = m.terrain.and_then(|p| Dem::open(p).ok());
    if edges.is_empty() || dem.is_none() {
        return vec![Metric {
            id: "authority.facade_ground".into(),
            invariant: Invariant::I8,
            title: "A road earthwork deciding the ground under a building".into(),
            population: "not measured".into(),
            detail: "needs both a building input and a terrain DEM".into(),
            sense: Sense::HigherIsWorse,
            threshold: FACADE_GROUND_M,
            skipped: Some(
                if edges.is_empty() { "no building input" } else { "no terrain DEM" }.into(),
            ),
            dist: Dist::metres(),
            worst: Vec::new(),
        }];
    }
    let dem = dem.as_mut().expect("checked");
    let z = m.solved.z_ref;

    // Long walls contribute more samples than short ones, which is right: the
    // question is how much *wall* stands on a road, not how many buildings have
    // any.
    let t0 = std::time::Instant::now();
    let mut all: Vec<f64> = Vec::new();
    let mut moved = 0u64;
    let mut truncated = false;
    // ARPT_DEBUG_FACADE attribution: a wall standing on a bench and a wall
    // standing on a batter face are two different fixes — narrow the bench, or
    // stop the face. Counting them apart is what says which one the population
    // is made of.
    let debug = std::env::var_os("ARPT_DEBUG_FACADE").is_some();
    let (mut on_bench, mut on_face) = (0u64, 0u64);
    let (mut cut, mut fill) = (0u64, 0u64);
    for e in edges {
        if all.len() >= MAX_SAMPLES {
            truncated = true;
            break;
        }
        for (lon, lat) in walk(e[0], e[1]) {
            if !m.bounds.contains(lon, lat) {
                continue;
            }
            // **`reference_surface`, not `Dem::elevation`.** The reference is
            // the rendered z_ref lattice, which is what the solve and the
            // ground stage both read; a raw point sample of the DEM differs
            // from it by the in-cell interpolation, which on a flank is metres
            // and would be charged to the earthwork. Passing the same surface
            // the stage read as the stack's own base makes that difference
            // cancel and leaves only what the ground stage did — the
            // cancellation `ground.single_source` relies on for the same
            // reason.
            let raw = crate::solve::reference_surface(dem, z, lon, lat);
            let published = m.ground.height(lon, lat, raw, 0.0, &mut scratch);
            let v = (published - raw).abs();
            dist.push(v);
            all.push(v);
            if v > 0.0 {
                moved += 1;
            }
            if debug && v > FACADE_GROUND_M {
                if m.ground.bed_target(lon, lat, &mut scratch).is_some() {
                    on_bench += 1;
                } else {
                    on_face += 1;
                }
                if published < raw {
                    cut += 1;
                } else {
                    fill += 1;
                }
            }
            if v > FACADE_GROUND_M {
                worst.offer(Offender {
                    lon,
                    lat,
                    zoom: z,
                    value: v,
                    note: format!(
                        "the hill is at {raw:.2} m here and the ground under this wall is \
                         {published:.2} m — an earthwork's, not the terrain's"
                    ),
                });
            }
        }
    }
    if std::env::var_os("ARPT_DEBUG_FACADE").is_some() {
        all.sort_by(f64::total_cmp);
        let q = |f: f64| all.get(((all.len().max(1) - 1) as f64 * f) as usize).copied().unwrap_or(0.0);
        eprintln!(
            "[facade] {:.1}s n={} moved={moved} over={} (bench {on_bench} / face {on_face}, \
             cut {cut} / fill {fill}) p50 {:.2} p90 {:.2} p95 {:.2} p98 {:.2} \
             p99 {:.2} max {:.2}",
            t0.elapsed().as_secs_f64(),
            all.len(),
            on_bench + on_face,
            q(0.50),
            q(0.90),
            q(0.95),
            q(0.98),
            q(0.99),
            all.last().copied().unwrap_or(0.0)
        );
    }

    let mut population = format!(
        "Every {SPACING_M:.0} m of every building footprint edge inside the extract bbox \
         ({} walls read), scored as the engineered ground minus the reference surface the solve itself read — \
         zero wherever no earthwork reaches, so the rate is the share of *wall* standing on \
         something a road decided. Of {} samples, {moved} stand on ground an earthwork moved \
         at all.",
        edges.len(),
        all.len()
    );
    if truncated {
        population.push_str(" Coverage: the sample cap bit; a full walk would find more.");
    }
    vec![Metric {
        id: "authority.facade_ground".into(),
        invariant: Invariant::I8,
        title: "A road earthwork deciding the ground under a building".into(),
        population,
        detail: format!(
            "A bench holds the ground flat at road level out to the class half-width plus a \
             shoulder plus a verge, and its batter reaches past that; nothing stops that \
             footprint containing a building. The consequence is invisible to every \
             archive-side check, because a building anchors at the highest drawn ground under \
             it and therefore rides the terrace instead of standing over daylight. Past \
             {FACADE_GROUND_M:.1} m the foundation the mesher extends can no longer absorb it \
             and the wall stands on a shelf the hill does not have."
        ),
        sense: Sense::HigherIsWorse,
        threshold: FACADE_GROUND_M,
        skipped: None,
        dist,
        worst: worst.into_vec(),
    }]
}

/// The sample points along one wall: both ends and every [`SPACING_M`] between,
/// so a wall shorter than the spacing still contributes its own two corners.
fn walk(a: Coord, b: Coord) -> impl Iterator<Item = (f64, f64)> {
    let cos_lat = ((a.y + b.y) * 0.5).to_radians().cos().max(0.1);
    let (dx, dy) = ((b.x - a.x) * DEG_M * cos_lat, (b.y - a.y) * DEG_M);
    let len = dx.hypot(dy);
    let n = (len / SPACING_M).floor() as usize;
    (0..=n).map(move |i| {
        let t = if n == 0 { 0.0 } else { i as f64 / n as f64 };
        (a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_wall_is_walked_end_to_end_at_the_spacing() {
        let a = Coord { x: 6.0, y: 46.0 };
        let b = Coord { x: 6.0, y: 46.0 + 10.0 / DEG_M };
        let pts: Vec<_> = walk(a, b).collect();
        assert_eq!(pts.len(), 6, "10 m at 2 m spacing is five steps and six points");
        assert!((pts[0].1 - a.y).abs() < 1e-12, "starts at one corner");
        assert!((pts[5].1 - b.y).abs() < 1e-12, "ends at the other");
    }

    #[test]
    fn a_wall_shorter_than_the_spacing_still_contributes_a_corner() {
        let a = Coord { x: 6.0, y: 46.0 };
        let b = Coord { x: 6.0, y: 46.0 + 0.4 / DEG_M };
        assert_eq!(walk(a, b).count(), 1);
    }
}
