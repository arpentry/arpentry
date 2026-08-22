//! Census: what allocating the **bench** out of the room the facades leave
//! would cost, before any of it is built.
//!
//! Phase 2 allocated the asphalt (`synth::carriageway::sections_along`). The
//! bench is wider — class half-width + `EARTHWORK_SHOULDER_M` +
//! `EARTHWORK_MARGIN_M` — and its batter reaches past that again, and
//! `authority.facade_ground` says 1.933 % of the world's wall stands on ground
//! one of the two decided, split 48 % bench / 52 % batter face.
//!
//! The question this answers is the one that decides the shape of the fix:
//! clipping a face at a building line leaves a **step** where the face would
//! have daylighted, and `MAX_BENCH_FACE_M` refuses a bench whose face is
//! deeper than it can plausibly be retained. So — how many at-grade nodes lose
//! their bench outright if the room clips it, and how deep are the steps the
//! survivors carry?
//!
//! **Montreux, `6.86,46.40,6.98,46.47`, 73,932 at-grade asphalt node-sides:**
//!
//! ```text
//! bench clipped        13,158  17.80 %   (8,448 of them past the band they carry)
//! face only clipped     1,491   2.02 %
//! bench loss (m)       p50 2.25  p90 2.25  max 3.00   — the verge, and no more
//! step at the wall (m) p50 0.16  p90 0.94  p99 2.52  max 13.25
//! step > 3 m              108   0.15 %    — the whole cost of the ladder
//! by class: service 32.2 %, unknown 36.0 %, residential 21.0 %, tertiary 10.4 %
//! ```
//!
//! So the fix is viable and its cost is bounded: clipping bench *and* face at
//! the building line leaves a plausible step nearly everywhere, and only 108
//! node-sides in the extract fall past `MAX_BENCH_FACE_M` onto the drape rung
//! of the ladder. It is not a driveway-only finding either — `residential`
//! loses its verge on a fifth of its own node-sides.
//!
//! Two things the census settled about the rule itself:
//!
//! - **The floor is phase 2's drawn band, not the class carriageway.** A bench
//!   narrower than the asphalt it carries leaves the drawn surface hanging over
//!   unbenched ground — the one thing `EarthworkEdge::carriageway_m` exists to
//!   prevent. So the bench gives up its verge and no more, and
//!   `MIN_CARRIAGEWAY_HALF_M` becomes its floor too. Read against the class
//!   carriageway instead, the loss came out p50 3.06 m and p90 4.25 m — the
//!   whole bench — which is a bench that would have hung its own road.
//! - **Rail is out**, for the reason `order.building_overlap` leaves it out: a
//!   station roof over its platforms is a level relation the model cannot
//!   state, and narrowing the formation there shaves the platform.
//!
//! Usage: cargo run --release --example bench_room_census -- \
//!            <segment.parquet> <building.parquet> <w,s,e,n> [terrain.pmtiles]

use arpentry_server::assemble;
use arpentry_server::assemble::facades::Facades;
use arpentry_server::priors::{
    self, EARTHWORK_MARGIN_M, EARTHWORK_SHOULDER_M, FACADE_CLEAR_M, MAX_BENCH_FACE_M,
};
use arpentry_server::project::Bounds;
use arpentry_server::scene::DEG_M;
use arpentry_server::solve;

/// How far along the centerline a station looks for the wall beside it — the
/// same rule `sections_along` uses, so the two allocations agree.
fn window_m(gap: f64) -> f64 {
    gap.max(4.0).min(32.0)
}

fn q(v: &[f64], f: f64) -> f64 {
    if v.is_empty() {
        return 0.0;
    }
    v[((v.len() - 1) as f64 * f) as usize]
}

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let seg = std::path::PathBuf::from(&a[0]);
    let bld = std::path::PathBuf::from(&a[1]);
    let b: Vec<f64> = a[2].split(',').map(|s| s.parse().unwrap()).collect();
    let bbox = Bounds { west: b[0], south: b[1], east: b[2], north: b[3] };
    let terrain = a.get(3).map(std::path::PathBuf::from);

    let facades =
        Facades::read(&bld, (bbox.west, bbox.south, bbox.east, bbox.north)).expect("buildings");
    eprintln!("facades: {} footprints, {} edges", facades.footprint_count(), facades.edge_count());

    let mut scene = assemble::run(&seg, None, &bbox).expect("assemble");
    let solved = solve::run(&mut scene, terrain.as_deref(), 16, 8).expect("solve");
    let ground = arpentry_server::ground::derive(&scene, &solved, &facades, terrain.as_deref(), 8);
    eprintln!(
        "corridors {} profiles {} earthworks {}",
        scene.corridors.len(),
        solved.solved_count(),
        ground.earthwork_count()
    );

    // Per at-grade node per side: the bench the class asks for, the room the
    // facades leave, and what the clip would take.
    let mut scratch: Vec<u32> = Vec::new();
    let mut nodes = 0u64; // at-grade node-sides in the bbox
    let mut clipped_bench = 0u64; // the bench itself would be narrowed
    let mut clipped_face = 0u64; // the bench survives, the face is cut short
    let mut to_carriageway = 0u64; // the room is tighter than the asphalt it carries
    let mut bench_loss: Vec<f64> = Vec::new(); // metres taken off the bench
    let mut step: Vec<f64> = Vec::new(); // the step left where a face is cut short
    let mut refused = 0u64; // step past MAX_BENCH_FACE_M: no bench at all
    // Which classes the clip lands on. A driveway running up to a garage door
    // is not the same finding as a residential street in an old town, and a
    // total that is mostly `service` would say the fix is about driveways.
    let mut by_class: std::collections::HashMap<String, (u64, u64)> =
        std::collections::HashMap::new();

    for c in &scene.corridors {
        let Some(p) = solved.profile(c.id) else { continue };
        if c.kind.prior().surface == priors::Surface::None {
            continue;
        }
        let prior = c.kind.prior().half_width_m(c.link).unwrap_or(0.0);
        if prior <= 0.0 {
            continue;
        }
        // Rail is out, for the reason `order.building_overlap` leaves it out:
        // a station roof over its platforms is a level relation the model
        // cannot state, and narrowing the formation there would shave the
        // platform rather than fix anything.
        if c.kind.prior().surface != priors::Surface::Asphalt {
            continue;
        }
        let bench_half = prior + EARTHWORK_SHOULDER_M + EARTHWORK_MARGIN_M;
        // **The floor is phase 2's drawn asphalt, not the class carriageway.**
        // A bench narrower than the band it carries leaves the drawn surface
        // hanging over unbenched ground, which is the one thing
        // `EarthworkEdge::carriageway_m` exists to prevent. So the bench may
        // give up its verge and no more, and the asphalt's own floor
        // (`MIN_CARRIAGEWAY_HALF_M`) becomes the bench's.
        let band_half = prior + priors::STRUCTURE_SHOULDER_M;
        let at_grade = p.at_grade();
        let arcs = p.arc();
        let road = p.road_m();
        let terrain_m = p.terrain_m();
        let m_lon = DEG_M * c.cos_lat;
        for k in 0..arcs.len() {
            if !at_grade[k] {
                continue;
            }
            let pt = p.point_at_arc(arcs[k]);
            if !bbox.contains(pt.x, pt.y) {
                continue;
            }
            let (j, l) = (k.saturating_sub(1), (k + 1).min(arcs.len() - 1));
            if j == l {
                continue;
            }
            let (pj, pl) = (p.point_at_arc(arcs[j]), p.point_at_arc(arcs[l]));
            let (dx, dy) = ((pl.x - pj.x) * m_lon, (pl.y - pj.y) * DEG_M);
            let len = dx.hypot(dy);
            if !(len > 0.0) {
                continue;
            }
            // The reach has to cover the batter too: a face that runs 20 m
            // out and passes under a building is half of what the check found.
            let reach = bench_half + priors::EARTHWORK_MAX_BATTER_M;
            let window = window_m(arcs[l] - arcs[j]);
            let room = facades.room(
                pt,
                c.cos_lat,
                (dx / len, dy / len),
                reach,
                window,
                &mut scratch,
            );
            // What the face is doing here: how deep it is, and how far it runs
            // before it daylights, read from the same numbers the ground stage
            // reads (the profile's road and terrain at this node).
            let rise = (road[k] - terrain_m[k]).abs();
            for (side, r) in [room.left, room.right].into_iter().enumerate() {
                nodes += 1;
                by_class.entry(c.class_key.clone()).or_default().0 += 1;
                let raw_allowed = (r - FACADE_CLEAR_M).max(0.0);
                if raw_allowed >= reach {
                    continue; // nothing close: the class gets what it asks for
                }
                // Phase 2's own allocation, so the bench and the band it
                // carries are read off one cross-section.
                let band =
                    raw_allowed.clamp(priors::MIN_CARRIAGEWAY_HALF_M.min(band_half), band_half);
                let allowed = raw_allowed.max(band);
                if allowed < bench_half {
                    clipped_bench += 1;
                    by_class.entry(c.class_key.clone()).or_default().1 += 1;
                    bench_loss.push(bench_half - allowed);
                    if raw_allowed < band {
                        to_carriageway += 1;
                    }
                    // The bench stops at the building line and the face has
                    // nowhere to go: the whole rise stands as a step.
                    step.push(rise);
                    if rise > MAX_BENCH_FACE_M {
                        refused += 1;
                    }
                } else {
                    // The bench fits; the face is what the room cuts short.
                    // What is left standing is the rise the face had not yet
                    // shed by the time it reached the wall.
                    let run = allowed - bench_half;
                    let shed = run / priors::EARTHWORK_BATTER;
                    let left = (rise - shed).max(0.0);
                    if left > 0.0 {
                        clipped_face += 1;
                        step.push(left);
                        if left > MAX_BENCH_FACE_M {
                            refused += 1;
                        }
                    }
                    let _ = side;
                }
            }
        }
    }

    bench_loss.sort_by(f64::total_cmp);
    step.sort_by(f64::total_cmp);
    println!("at-grade node-sides in bbox: {nodes}");
    println!(
        "  room clips the BENCH:  {clipped_bench} ({:.2} %)  of which the room is tighter than \
         the band it carries: {to_carriageway}",
        100.0 * clipped_bench as f64 / nodes.max(1) as f64
    );
    println!(
        "  room clips only the FACE: {clipped_face} ({:.2} %)",
        100.0 * clipped_face as f64 / nodes.max(1) as f64
    );
    println!(
        "  bench loss (m): n={} p50 {:.2} p90 {:.2} p99 {:.2} max {:.2}",
        bench_loss.len(),
        q(&bench_loss, 0.5),
        q(&bench_loss, 0.9),
        q(&bench_loss, 0.99),
        bench_loss.last().copied().unwrap_or(0.0)
    );
    println!(
        "  step left at the wall (m): n={} p50 {:.2} p90 {:.2} p99 {:.2} max {:.2}",
        step.len(),
        q(&step, 0.5),
        q(&step, 0.9),
        q(&step, 0.99),
        step.last().copied().unwrap_or(0.0)
    );
    let mut classes: Vec<(String, (u64, u64))> = by_class.into_iter().collect();
    classes.sort_by_key(|(_, (_, clipped))| std::cmp::Reverse(*clipped));
    println!("  bench clip by class (clipped / node-sides):");
    for (k, (n, clipped)) in classes.iter().take(10) {
        if *clipped == 0 {
            continue;
        }
        println!("    {k:16} {clipped:>7} / {n:>7}  {:.1} %", 100.0 * *clipped as f64 / *n as f64);
    }
    println!(
        "  step past MAX_BENCH_FACE_M ({MAX_BENCH_FACE_M:.1} m): {refused} ({:.2} % of clipped \
         node-sides, {:.3} % of all)",
        100.0 * refused as f64 / (clipped_bench + clipped_face).max(1) as f64,
        100.0 * refused as f64 / nodes.max(1) as f64
    );
}
