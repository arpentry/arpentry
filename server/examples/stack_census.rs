//! Where do two at-grade bands end up stacked, and which stage put them there?
//!
//! `order.grade_stack` scores the drawn archive and can only say "these two
//! surfaces are 13 m apart with nothing between them". This walks the model
//! side of the same fact — every plan crossing (`crossings::plan_index`) where
//! *both* alignments end the solve as level-0 grade and their solved surfaces
//! are more than [`crossings::SEPARATION_M`] apart — and attributes each site:
//!
//! - **licensed** — the lower side is inside a covered-crossing window
//!   (`crossings::covered_sites`, the gate `relax::seed_bore_ceilings` and
//!   `portals::annex_spans` read) yet was still paved open. Split further into
//!   the annotation stopping short of the window and
//!   `portals::reconcile_spans` shrinking a span back off it.
//! - **near a structure** — neither side is licensed, but a structure span of
//!   one of them ends within `REACH_M` of the crossing: the mapper's span
//!   covers the deck and not the embankment the other line runs through.
//! - **bare** — no structure anywhere near: two lines mapped at grade that the
//!   solve drove metres apart.
//!
//! Usage: cargo run --release --example stack_census -- <segment.parquet> <w,s,e,n> <terrain.pmtiles> [corridor]

use arpentry_server::project::Bounds;
use arpentry_server::scene::{Span, SpanKind};
use arpentry_server::solve::crossings;
use arpentry_server::{assemble, solve};

/// How far from a crossing a structure span end still counts as "the mapper
/// covered this crossing, just not widely enough", metres.
const REACH_M: f64 = 40.0;

/// Arc step when measuring how much of a licensed window is open, metres.
const STEP_M: f64 = 0.5;

fn kind_at(spans: &[Span], arc: f64) -> SpanKind {
    spans.iter().find(|s| arc >= s.arc0 && arc <= s.arc1).map_or(SpanKind::Grade, |s| s.kind)
}

/// Distance from `arc` to the nearest end of a structure span of `spans`.
fn nearest_structure(spans: &[Span], arc: f64) -> Option<(f64, SpanKind)> {
    spans
        .iter()
        .filter(|s| s.kind != SpanKind::Grade)
        .map(|s| ((s.arc0 - arc).abs().min((s.arc1 - arc).abs()), s.kind))
        .min_by(|a, b| a.0.total_cmp(&b.0))
}

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let bb: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let bbox = Bounds { west: bb[0], south: bb[1], east: bb[2], north: bb[3] };
    let terrain = std::path::PathBuf::from(&a[2]);
    let dump: Option<u32> = a.get(3).and_then(|s| s.parse().ok());

    let mut scene = assemble::run(std::path::Path::new(&a[0]), None, &bbox).expect("assemble");
    // The license is stated against the annotation, so both must be taken
    // before the write-back rewrites `scene.corridors[..].spans`.
    let plan = crossings::plan_index(&scene);
    let sites = crossings::covered_sites(&scene, &plan);
    let annotated: Vec<Vec<Span>> = scene.corridors.iter().map(|c| c.spans.clone()).collect();

    let solved = solve::run(&mut scene, Some(&terrain), 16, 0).expect("solve");

    // ---- part 1: the burial license, kept or degraded ----------------------
    let (mut licensed_m, mut kept_m, mut degraded_m) = (0.0f64, 0.0f64, 0.0f64);
    let (mut short_annotation_m, mut shrunk_m) = (0.0f64, 0.0f64);
    for c in &scene.corridors {
        let Some(p) = solved.profile(c.id) else { continue };
        for x in &sites[c.id as usize] {
            let (w0, w1) = (x.arc - x.clear_m, x.arc + x.clear_m);
            let mut arc = w0;
            while arc < w1 {
                let mid = arc + STEP_M * 0.5;
                let step = STEP_M.min(w1 - arc);
                arc += STEP_M;
                let pt = p.point_at_arc(mid);
                if pt.x < bbox.west || pt.x > bbox.east || pt.y < bbox.south || pt.y > bbox.north {
                    continue;
                }
                licensed_m += step;
                if kind_at(&c.spans, mid) == SpanKind::Tunnel {
                    kept_m += step;
                } else if kind_at(&annotated[c.id as usize], mid) == SpanKind::Tunnel {
                    degraded_m += step;
                    shrunk_m += step;
                    println!(
                        "  SHRUNK  #{} {:?} arc {:.1}  at {:.6},{:.6}  drawn {:?}",
                        c.id,
                        c.kind,
                        mid,
                        pt.x,
                        pt.y,
                        kind_at(&c.spans, mid)
                    );
                } else {
                    degraded_m += step;
                    short_annotation_m += step;
                    println!(
                        "  SHORT-ANN  #{} {:?} arc {:.1}  at {:.6},{:.6}  drawn {:?}",
                        c.id,
                        c.kind,
                        mid,
                        pt.x,
                        pt.y,
                        kind_at(&c.spans, mid)
                    );
                }
            }
        }
    }

    // ---- part 2: every stacked crossing, attributed ------------------------
    // (gap, lon, lat, upper, lower, cause, detail)
    let mut stacked: Vec<(f64, f64, f64, u32, u32, &'static str, String)> = Vec::new();
    let mut seen: Vec<(u32, u32, i64)> = Vec::new();
    for c in &scene.corridors {
        let Some(p) = solved.profile(c.id) else { continue };
        for x in &plan[c.id as usize] {
            if x.other <= c.id {
                continue; // each pair once, from the lower id
            }
            let Some(q) = solved.profile(x.other) else { continue };
            if kind_at(&c.spans, x.arc) != SpanKind::Grade
                || kind_at(&scene.corridors[x.other as usize].spans, x.other_arc) != SpanKind::Grade
            {
                continue;
            }
            let pt = p.point_at_arc(x.arc);
            if pt.x < bbox.west || pt.x > bbox.east || pt.y < bbox.south || pt.y > bbox.north {
                continue;
            }
            let (mine, theirs) = (p.road_at_arc(x.arc), q.road_at_arc(x.other_arc));
            let gap = (theirs - mine).abs();
            if gap <= crossings::SEPARATION_M {
                continue;
            }
            let key = (c.id.min(x.other), c.id.max(x.other), (x.arc * 4.0) as i64);
            if seen.contains(&key) {
                continue;
            }
            seen.push(key);
            let (up, lo, up_arc, lo_arc) = if theirs > mine {
                (x.other, c.id, x.other_arc, x.arc)
            } else {
                (c.id, x.other, x.arc, x.other_arc)
            };
            // Was the lower side licensed to be buried here?
            let licensed = sites[lo as usize]
                .iter()
                .any(|s| lo_arc >= s.arc - s.clear_m && lo_arc <= s.arc + s.clear_m);
            let lo_ann = kind_at(&annotated[lo as usize], lo_arc);
            let near_lo = nearest_structure(&scene.corridors[lo as usize].spans, lo_arc);
            let near_up = nearest_structure(&scene.corridors[up as usize].spans, up_arc);
            let nearest = match (near_lo, near_up) {
                (Some(a), Some(b)) if a.0 <= b.0 => Some(("lower", a)),
                (_, Some(b)) => Some(("upper", b)),
                (Some(a), None) => Some(("lower", a)),
                (None, None) => None,
            };
            let cause = if licensed && lo_ann == SpanKind::Tunnel {
                "licensed:shrunk"
            } else if licensed {
                "licensed:short"
            } else if nearest.is_some_and(|(_, (d, _))| d <= REACH_M) {
                "near-structure"
            } else {
                "bare"
            };
            let detail = nearest.map_or("no structure on either side".into(), |(w, (d, k))| {
                format!("{w} {k:?} span end {d:.1} m away")
            });
            stacked.push((gap, pt.x, pt.y, up, lo, cause, detail.clone()));
            if dump == Some(c.id) || dump == Some(x.other) {
                println!(
                    "  #{up} {:.2} over #{lo} {:.2} at {:.6},{:.6}  gap {gap:.2}  {cause}  ({detail})",
                    theirs.max(mine),
                    theirs.min(mine),
                    pt.x,
                    pt.y
                );
            }
        }
    }

    println!("\nCOVERED-CROSSING LICENSE");
    println!("  licensed centerline      {licensed_m:8.1} m");
    println!(
        "  drawn as tube (kept)     {kept_m:8.1} m  ({:.1} %)",
        100.0 * kept_m / licensed_m.max(1e-9)
    );
    println!(
        "  paved open (degraded)    {degraded_m:8.1} m  ({:.1} %)",
        100.0 * degraded_m / licensed_m.max(1e-9)
    );
    println!("    annotation stopped short {short_annotation_m:8.1} m");
    println!("    reconcile shrank it back {shrunk_m:8.1} m");

    // ---- part 3: decks shorter than the crossing they carry ----------------
    // The bore annex's missing twin. A mapped deck ends where a mapper split
    // the way; the band of the feature passing beneath is as wide as that
    // feature, divided by the sine of the crossing angle. Where the second
    // exceeds the first the deck's own formation band is drawn at grade over
    // the lower band — annotation-only, so this is measured with no height.
    // (shortfall_m, lon, lat, carrier, under, deck_len, reach)
    let mut short_decks: Vec<(f64, f64, f64, u32, u32, f64, f64)> = Vec::new();
    for c in &scene.corridors {
        let Some(p) = solved.profile(c.id) else { continue };
        for x in &plan[c.id as usize] {
            let under = &scene.corridors[x.other as usize];
            let my_level = c.spans.iter().find(|s| x.arc >= s.arc0 && x.arc <= s.arc1).map(|s| s.level);
            let their_level = under
                .spans
                .iter()
                .find(|s| x.other_arc >= s.arc0 && x.other_arc <= s.arc1)
                .map_or(0, |s| s.level);
            let Some(0..) = my_level else { continue };
            if my_level.unwrap_or(0) <= their_level {
                continue; // not carried over this one
            }
            // The deck this crossing belongs to, if any: the Bridge span
            // containing it or ending nearest it.
            let Some(deck) = c
                .spans
                .iter()
                .filter(|s| s.kind == SpanKind::Bridge)
                .min_by(|a, b| {
                    let d = |s: &Span| {
                        if x.arc >= s.arc0 && x.arc <= s.arc1 {
                            0.0
                        } else {
                            (s.arc0 - x.arc).abs().min((s.arc1 - x.arc).abs())
                        }
                    };
                    d(a).total_cmp(&d(b))
                })
                // The crossing's own band must reach the deck: a crossing a
                // span-length away is another span's business, or nobody's.
                .filter(|s| x.arc + x.clear_m > s.arc0 && x.arc - x.clear_m < s.arc1)
            else {
                continue;
            };
            let short = (deck.arc0 - (x.arc - x.clear_m)).max(0.0)
                + ((x.arc + x.clear_m) - deck.arc1).max(0.0);
            if short <= 0.0 {
                continue;
            }
            let pt = p.point_at_arc(x.arc);
            if pt.x < bbox.west || pt.x > bbox.east || pt.y < bbox.south || pt.y > bbox.north {
                continue;
            }
            short_decks.push((
                short,
                pt.x,
                pt.y,
                c.id,
                x.other,
                deck.arc1 - deck.arc0,
                2.0 * x.clear_m,
            ));
            if dump == Some(c.id) {
                println!(
                    "  deck #{} [{:.1}, {:.1}] carries #{} at arc {:.1} reach ±{:.1} — short by {short:.1} m",
                    c.id, deck.arc0, deck.arc1, x.other, x.arc, x.clear_m
                );
            }
        }
    }

    stacked.sort_by(|a, b| b.0.total_cmp(&a.0));
    println!(
        "\nSTACKED CROSSINGS ({} sites over {:.1} m, both sides level-0 grade)",
        stacked.len(),
        crossings::SEPARATION_M
    );
    for cause in ["licensed:shrunk", "licensed:short", "near-structure", "bare"] {
        let g: Vec<_> = stacked.iter().filter(|s| s.5 == cause).collect();
        if g.is_empty() {
            continue;
        }
        let worst = g.iter().map(|s| s.0).fold(0.0f64, f64::max);
        println!("  {cause:16} {:3} sites, worst {worst:.2} m", g.len());
    }
    println!("\nTOP 30 BY GAP");
    for (gap, lon, lat, up, lo, cause, detail) in stacked.iter().take(30) {
        println!("  {gap:7.2} m  {lon:.6},{lat:.6}  #{up} over #{lo}  {cause:16} {detail}");
    }

    short_decks.sort_by(|a, b| b.0.total_cmp(&a.0));
    let total: f64 = short_decks.iter().map(|s| s.0).sum();
    println!(
        "\nDECKS SHORTER THAN THE CROSSING THEY CARRY: {} of them, {total:.0} m of shortfall",
        short_decks.len()
    );
    for (short, lon, lat, up, lo, len, reach) in short_decks.iter().take(25) {
        println!(
            "  short {short:6.1} m  {lon:.6},{lat:.6}  deck #{up} ({len:.0} m) over #{lo} \
             (band reach {reach:.0} m)"
        );
    }
}
