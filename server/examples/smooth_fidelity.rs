//! Is the smoothed sweep line a *cleaned* version of the road, or a different
//! road?
//!
//! `Profile::smooth` is what every structure is swept along and what the ground
//! benches beside. `examples/smooth_offset` reported it sitting a median 1.53 m
//! from the raw line the at-grade asphalt is buffered around, and read that as
//! the two-curves problem. That number alone does not say whether the smoother
//! is removing digitising noise (in which case the raw line is the wrong one and
//! the union should move) or cutting corners (in which case the smooth line is
//! the wrong one and no amount of unifying will help) — or, as it turned out,
//! neither.
//!
//! So this splits the displacement two ways and attributes each part:
//!
//! - **across vs along.** Across the road, the deck sits beside its own
//!   approach. *Along* it, the deck is swept at one station carrying the height
//!   solved for another — worth the slide times the grade — and its abutment
//!   lands short of or past the span it belongs to. The second is not a
//!   smoothing question at all: it was the sweep line being sampled at the
//!   densifier's chord fraction on a centripetal Catmull-Rom, whose parameter is
//!   not arc length.
//! - **local curvature radius** at the node, from the raw line over a ±30 m
//!   chord. A quadratic-in-arc fit is a parabola, which follows a road arc only
//!   while the window spans a shallow angle of it, so this is the axis the
//!   corner-cutting shows up on: a fixed ±100 m window saturated its deviation
//!   clamp on every radius under 60 m.
//! - **distance to the nearest corridor end**, because the window is truncated
//!   there. A symmetric window cancels the odd (cubic) term of a curve exactly
//!   at the evaluation point; a one-sided window does not.
//!
//! Both defects are fixed; this is what measured them, and what says so if they
//! come back. On the Montreux extract it read a median 1.59 m of displacement
//! (slide 0.37 m, lateral 0.84 m, 27 % of nodes at the clamp) before, and
//! 0.40 m (slide 0.03 m, lateral 0.37 m) after.
//!
//! Usage: cargo run --release --example smooth_fidelity -- <segment.parquet> <w,s,e,n> <terrain.pmtiles>

use arpentry_server::assemble;
use arpentry_server::project::Bounds;
use arpentry_server::solve;
use geo_types::Coord;

const DEG_M: f64 = 111_320.0;

/// The smoother's own half-window, restated so this can disagree with it.
const WINDOW_M: f64 = 100.0;

/// Menger curvature radius at `b` from the chord `a`–`b`–`c`, in metres
/// (infinite when the three are collinear).
fn radius(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> f64 {
    let (abx, aby) = (b.0 - a.0, b.1 - a.1);
    let (bcx, bcy) = (c.0 - b.0, c.1 - b.1);
    let (acx, acy) = (c.0 - a.0, c.1 - a.1);
    let cross = abx * bcy - aby * bcx;
    if cross.abs() < 1e-9 {
        return f64::INFINITY;
    }
    let la = (abx * abx + aby * aby).sqrt();
    let lb = (bcx * bcx + bcy * bcy).sqrt();
    let lc = (acx * acx + acy * acy).sqrt();
    la * lb * lc / (2.0 * cross.abs())
}

fn pct(v: &mut Vec<f64>, p: f64) -> f64 {
    if v.is_empty() {
        return f64::NAN;
    }
    v.sort_by(f64::total_cmp);
    v[((v.len() - 1) as f64 * p) as usize]
}

fn report(name: &str, v: &mut Vec<f64>) {
    if v.is_empty() {
        println!("{name:<28} —");
        return;
    }
    let n = v.len();
    println!(
        "{name:<28} n={n:<7} p50 {:>5.2}  p90 {:>5.2}  max {:>5.2}  ≥3.9 m {:>5.1} %",
        pct(v, 0.5),
        pct(v, 0.9),
        pct(v, 1.0),
        100.0 * v.iter().filter(|&&d| d >= 3.9).count() as f64 / n as f64,
    );
}

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let bb: Vec<f64> = a[1].split(',').map(|s| s.parse().expect("bbox")).collect();
    let bbox = Bounds { west: bb[0], south: bb[1], east: bb[2], north: bb[3] };
    let terrain = std::path::PathBuf::from(&a[2]);
    let mut scene = assemble::run(std::path::Path::new(&a[0]), None, &bbox).expect("assemble");
    let solved = solve::run(&mut scene, Some(&terrain), 16, 0).expect("solve");

    // Displacement bucketed by what should explain it.
    let (mut tight, mut medium, mut gentle, mut straight) = (vec![], vec![], vec![], vec![]);
    let (mut near_end, mut interior) = (vec![], vec![]);
    let mut all = vec![];
    let mut corridor_len = vec![];
    // The displacement split into the two components that mean different
    // things. *Across* the road it takes the deck off the band beside it;
    // *along* it, the deck is swept at one station carrying the height solved
    // for another — a height error of the slide times the grade, and an
    // abutment that lands short of or past the span it belongs to.
    let (mut lat, mut lon_slide) = (vec![], vec![]);
    let mut worst_slide: Vec<(f64, u32, f64, f64)> = Vec::new();
    let mut worst_lat: Vec<(f64, u32, f64, f64)> = Vec::new();
    for c in &scene.corridors {
        let Some(p) = solved.profile(c.id) else { continue };
        let (raw, sm, arc) = (p.nodes(), p.smooth(), p.arc());
        if raw.len() != sm.len() || raw.len() < 3 {
            continue;
        }
        let cos = c.cos_lat;
        let m = |q: Coord| (q.x * cos * DEG_M, q.y * DEG_M);
        let total = arc[arc.len() - 1];
        corridor_len.push(total);
        for i in 0..raw.len() {
            let (dx, dy) = (m(sm[i]).0 - m(raw[i]).0, m(sm[i]).1 - m(raw[i]).1);
            let d = (dx * dx + dy * dy).sqrt();
            all.push(d);
            // Split along the raw line's own direction at this node.
            let (ja, jb) = (i.saturating_sub(1), (i + 1).min(raw.len() - 1));
            if ja != jb {
                let (ax, ay) = m(raw[ja]);
                let (bx, by) = m(raw[jb]);
                let (ex, ey) = (bx - ax, by - ay);
                let len = (ex * ex + ey * ey).sqrt();
                if len > 1e-9 {
                    let (tx, ty) = (ex / len, ey / len);
                    let along = dx * tx + dy * ty;
                    lon_slide.push(along.abs());
                    let across = (dx * -ty + dy * tx).abs();
                    lat.push(across);
                    worst_slide.push((along.abs(), c.id, raw[i].x, raw[i].y));
                    worst_lat.push((across, c.id, raw[i].x, raw[i].y));
                }
            }
            // Curvature from a ±30 m chord: far enough that node-to-node
            // digitising jitter does not dominate the three-point circle,
            // near enough to still be local.
            let mut lo = i;
            let mut hi = i;
            while lo > 0 && arc[i] - arc[lo - 1] < 30.0 {
                lo -= 1;
            }
            while hi + 1 < raw.len() && arc[hi + 1] - arc[i] < 30.0 {
                hi += 1;
            }
            if lo == i || hi == i {
                continue;
            }
            let r = radius(m(raw[lo]), m(raw[i]), m(raw[hi]));
            match r {
                r if r < 60.0 => tight.push(d),
                r if r < 200.0 => medium.push(d),
                r if r < 1000.0 => gentle.push(d),
                _ => straight.push(d),
            }
            // A window is truncated — and so no longer cancels the curve's odd
            // term — whenever the node is within one half-window of an end.
            let to_end = arc[i].min(total - arc[i]);
            if to_end < WINDOW_M {
                near_end.push(d);
            } else {
                interior.push(d);
            }
        }
    }
    println!("smoothing displacement |smooth − raw|, metres\n");
    report("all nodes", &mut all);
    println!("\nsplit against the road's own direction");
    report("  across (lateral)", &mut lat);
    report("  along (slide)", &mut lon_slide);
    worst_slide.sort_by(|a, b| b.0.total_cmp(&a.0));
    for (d, id, x, y) in worst_slide.iter().take(4) {
        println!("    {d:>8.1} m slid  corridor #{id}  at {x:.6},{y:.6}");
    }
    worst_lat.sort_by(|a, b| b.0.total_cmp(&a.0));
    for (d, id, x, y) in worst_lat.iter().take(8) {
        println!("    {d:>8.1} m across  corridor #{id}  at {x:.6},{y:.6}");
    }
    println!("\nby local curvature radius (±30 m chord)");
    report("  R < 60 m (hairpin/corner)", &mut tight);
    report("  60–200 m", &mut medium);
    report("  200–1000 m", &mut gentle);
    report("  R > 1000 m (near straight)", &mut straight);
    println!("\nby window truncation");
    report("  within 100 m of an end", &mut near_end);
    report("  interior (full window)", &mut interior);
    // The abutment is where a slide is *seen*: the deck's end cross-section is
    // swept at the smooth line's station, while the at-grade band ends at the
    // raw one, so the two meet short of or past each other by exactly this.
    let mut at_span_end: Vec<f64> = Vec::new();
    let mut worst_abut: Vec<(f64, f64, u32, f64, f64)> = Vec::new();
    for c in &scene.corridors {
        let Some(p) = solved.profile(c.id) else { continue };
        let (raw, sm, arc) = (p.nodes(), p.smooth(), p.arc());
        if raw.len() != sm.len() || raw.len() < 3 || c.spans.is_empty() {
            continue;
        }
        let cos = c.cos_lat;
        let m = |q: Coord| (q.x * cos * DEG_M, q.y * DEG_M);
        for s in &c.spans {
            if s.kind == arpentry_server::scene::SpanKind::Grade {
                continue;
            }
            for edge in [s.arc0, s.arc1] {
                let i = arc.partition_point(|&a| a < edge).min(raw.len() - 1);
                if i == 0 || i + 1 >= raw.len() {
                    continue;
                }
                let (dx, dy) = (m(sm[i]).0 - m(raw[i]).0, m(sm[i]).1 - m(raw[i]).1);
                let (ax, ay) = m(raw[i - 1]);
                let (bx, by) = m(raw[i + 1]);
                let (ex, ey) = (bx - ax, by - ay);
                let len = (ex * ex + ey * ey).sqrt();
                if len < 1e-9 {
                    continue;
                }
                let along = (dx * ex / len + dy * ey / len).abs();
                at_span_end.push(along);
                // The height the wrong station costs, at the local grade.
                let road = p.road_m();
                let g = if arc[i + 1] > arc[i] {
                    ((road[i + 1] - road[i]) / (arc[i + 1] - arc[i])).abs()
                } else {
                    0.0
                };
                worst_abut.push((along, along * g, c.id, raw[i].x, raw[i].y));
            }
        }
    }
    println!("\nslide *along* the road at a structure span end (the abutment)");
    report("  at span ends", &mut at_span_end);
    worst_abut.sort_by(|a, b| b.0.total_cmp(&a.0));
    for (d, herr, id, x, y) in worst_abut.iter().take(10) {
        println!("    {d:>7.1} m slid ({herr:>5.2} m of height at the local grade)  corridor #{id}  at {x:.6},{y:.6}");
    }
    println!(
        "\ncorridor length: p50 {:.0} m  p90 {:.0} m  max {:.0} m  ({} corridors)",
        pct(&mut corridor_len, 0.5),
        pct(&mut corridor_len, 0.9),
        pct(&mut corridor_len, 1.0),
        corridor_len.len()
    );
    let short = corridor_len.iter().filter(|&&l| l < 200.0).count();
    println!(
        "  {short} corridors ({:.0} %) are shorter than one full window, so every node in them \
         is fitted from a truncated one",
        100.0 * short as f64 / corridor_len.len() as f64
    );
}
