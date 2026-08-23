//! Census: which pedestrian ways the model attaches to which streets, and how
//! that population compares with the plan-space study the rule came from.
//!
//! Phase 3 of the sidewalk/facade plan states the relation; nothing draws it
//! yet. So the only way to judge the rule is to count what it caught against
//! the ground truth the study established with duckdb over the same window:
//!
//! ```text
//! tagged subclass='sidewalk'   46.0 km    65.7 % of it over 0.8 corridor coverage
//! untagged, detected           16.8 km    248 segments — a third again
//! untagged, off-street        266.0 km    hillside, around Montreux
//! tagged but > 10 m from any street        10.9 % of samples
//! ```
//!
//! What the census must show for the rule to be believed: the tagged
//! population mostly attaches, the detected population is roughly a third
//! again as much length, and the refusals are the far-from-any-street tail
//! rather than a random slice.
//!
//! Usage: cargo run --release --example walk_attach_census -- \
//!            <segment.parquet> <w,s,e,n>

use std::collections::HashMap;

use arpentry_server::assemble;
use arpentry_server::assemble::walks::Evidence;
use arpentry_server::priors;
use arpentry_server::project::Bounds;

fn q(v: &[f64], f: f64) -> f64 {
    if v.is_empty() {
        return 0.0;
    }
    v[((v.len() - 1) as f64 * f) as usize]
}

fn pct(n: f64, d: f64) -> f64 {
    if d > 0.0 {
        100.0 * n / d
    } else {
        0.0
    }
}

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let seg = std::path::PathBuf::from(&a[0]);
    let b: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let bbox = Bounds { west: b[0], south: b[1], east: b[2], north: b[3] };

    let t = std::time::Instant::now();
    let scene = assemble::run(&seg, None, &bbox).expect("assemble");
    let walks = &scene.walks;
    let c = walks.census();
    eprintln!(
        "assembled in {:.1}s: {} corridors, {} pedestrian lines, {} attachments",
        t.elapsed().as_secs_f64(),
        scene.corridors.len(),
        c.lines,
        walks.len()
    );

    println!("\n--- the population ---");
    println!(
        "(the whole read, which is the bbox's row groups and so spills well past it — \
         ratios are comparable to the plan-space study, absolute km are not)"
    );
    println!("pedestrian lines        {:>8}  {:>9.1} km", c.lines, c.line_m / 1000.0);
    println!(
        "  tagged sidewalk       {:>8}  {:>9.1} km  ({:.1} % of length)",
        c.tagged_lines,
        c.tagged_m / 1000.0,
        pct(c.tagged_m, c.line_m)
    );
    println!(
        "  within reach of a street         {:>9.1} km  ({:.1} %)",
        c.covered_m / 1000.0,
        pct(c.covered_m, c.line_m)
    );
    println!(
        "    of the tagged                  {:>9.1} km  ({:.1} %)",
        c.tagged_covered_m / 1000.0,
        pct(c.tagged_covered_m, c.tagged_m)
    );
    println!(
        "  running along one                {:>9.1} km  ({:.1} %)",
        c.alongside_m / 1000.0,
        pct(c.alongside_m, c.line_m)
    );

    println!("\n--- what attached ---");
    println!(
        "lines attached          {:>8}  {:>9.1} km of way, {:.1} km of host arc",
        c.attached_lines,
        c.attached_m / 1000.0,
        c.host_arc_m / 1000.0
    );
    println!(
        "  by tag only           {:>8}   (the geometric test fell short)",
        c.tag_only
    );
    println!(
        "  by geometry only      {:>8}   (untagged, detected; {} of them only because a \
         crosswalk joins them)",
        c.alongside_only, c.joined_only
    );
    println!("  by both               {:>8}", c.both);
    println!(
        "tagged with no street in reach anywhere  {:>5}  ({:.1} % of tagged lines)",
        c.tagged_unhosted,
        pct(c.tagged_unhosted as f64, c.tagged_lines as f64)
    );
    println!(
        "ranges dropped under {:.0} m             {:>5}  {:.2} km",
        priors::WALK_ATTACH_MIN_M,
        c.dropped_short,
        c.dropped_short_m / 1000.0
    );
    // What ends a run ends a band. A fragmented relation is not wrong — a
    // sidewalk really does stop at the mouth of the side street it crosses —
    // but which cause dominates decides whether phase 5 draws one band per
    // block or a chain of stubs.
    let breaks = (c.broke_lost + c.broke_crossed + c.broke_host + c.broke_side + c.broke_turned)
        .max(1) as f64;
    println!(
        "runs end at: a side street {:.0} %, no street at all {:.0} %, a turn off its own host \
         {:.0} %, a change of host {:.0} %, a change of side {:.0} %",
        pct(c.broke_crossed as f64, breaks),
        pct(c.broke_lost as f64, breaks),
        pct(c.broke_turned as f64, breaks),
        pct(c.broke_host as f64, breaks),
        pct(c.broke_side as f64, breaks)
    );

    // The length each evidence carries. The plan's claim is that the geometry
    // finds "a third again" as much as the tag, so the two must be separable.
    let mut by_evidence: HashMap<&str, (u32, f64)> = HashMap::new();
    let mut lens: Vec<f64> = Vec::new();
    let mut offsets: Vec<f64> = Vec::new();
    let mut spreads: Vec<f64> = Vec::new();
    // Offset by host class: a sidewalk stands at a fixed remove from the kerb,
    // so the clear distance should be class-independent even though the
    // centerline distance is not. That is the premise WALK_ATTACH_M rests on.
    let mut by_class: HashMap<String, Vec<f64>> = HashMap::new();
    let mut on_structure_m = 0.0;
    // The window's own share, so a number can be quoted against the duckdb
    // study without the spill in it.
    let (mut in_window, mut in_window_m) = (0u32, 0.0);
    let mut in_window_by_evidence: HashMap<&str, f64> = HashMap::new();
    for a in walks.all() {
        let host = &scene.corridors[a.host as usize];
        let mid = (a.arc0 + a.arc1) * 0.5;
        let k = host.arc.partition_point(|&x| x < mid).min(host.nodes.len() - 1);
        let windowed = bbox.contains(host.nodes[k].x, host.nodes[k].y);
        if windowed {
            in_window += 1;
            in_window_m += a.len_m();
        }
        let key = match a.evidence {
            Evidence::Tag => "tag",
            Evidence::Alongside => "geometry",
            Evidence::Both => "both",
        };
        let e = by_evidence.entry(key).or_default();
        e.0 += 1;
        e.1 += a.len_m();
        if windowed {
            *in_window_by_evidence.entry(key).or_default() += a.len_m();
        }
        lens.push(a.len_m());
        offsets.push(a.offset_m);
        spreads.push(a.spread_m);
        let half = host.width_m.unwrap_or(0.0) * 0.5;
        by_class.entry(host.class_key.clone()).or_default().push(a.offset_m - half);
        // How much of the relation lands on a bridge or in a bore. A carried
        // sidewalk is `synth::carried`'s already; a band drawn there would be
        // a second one.
        on_structure_m += host
            .spans
            .iter()
            .filter(|s| s.kind != arpentry_server::scene::SpanKind::Grade)
            .map(|s| (a.arc1.min(s.arc1) - a.arc0.max(s.arc0)).max(0.0))
            .sum::<f64>();
    }
    println!("\n--- the attachments ---");
    println!(
        "inside the bbox proper  {in_window} ranges  {:.1} km of host arc  ({:.0} % of the read)",
        in_window_m / 1000.0,
        pct(in_window_m, c.host_arc_m)
    );
    let mut keys: Vec<&&str> = by_evidence.keys().collect();
    keys.sort();
    for k in keys {
        let (n, m) = by_evidence[*k];
        println!(
            "{k:<10} {n:>6} ranges  {:>8.1} km of host arc   ({:>5.1} km in the window)",
            m / 1000.0,
            in_window_by_evidence.get(*k).copied().unwrap_or(0.0) / 1000.0
        );
    }
    lens.sort_by(f64::total_cmp);
    offsets.sort_by(f64::total_cmp);
    spreads.sort_by(f64::total_cmp);
    println!(
        "range length (m)   p10 {:.0}  p50 {:.0}  p90 {:.0}  max {:.0}",
        q(&lens, 0.1),
        q(&lens, 0.5),
        q(&lens, 0.9),
        q(&lens, 1.0)
    );
    println!(
        "offset (m)         p10 {:.1}  p50 {:.1}  p90 {:.1}  max {:.1}",
        q(&offsets, 0.1),
        q(&offsets, 0.5),
        q(&offsets, 0.9),
        q(&offsets, 1.0)
    );
    println!(
        "wander ±(m)        p50 {:.2}  p90 {:.2}  p99 {:.2}  max {:.2}",
        q(&spreads, 0.5),
        q(&spreads, 0.9),
        q(&spreads, 0.99),
        q(&spreads, 1.0)
    );
    println!(
        "on a bridge or in a bore  {:.2} km  ({:.1} % of host arc)",
        on_structure_m / 1000.0,
        pct(on_structure_m, c.host_arc_m)
    );

    // What the untagged detections *are*. The plan's claim is that they are
    // sidewalks the tag missed; if they were mostly `path`, they would be
    // hillside tracks that happen to run with a road for a while.
    let mut by_walk_class: HashMap<String, (f64, f64)> = HashMap::new();
    for a in walks.all() {
        let e = by_walk_class.entry(format!("{:?}", a.kind)).or_default();
        match a.evidence {
            Evidence::Alongside => e.1 += a.len_m(),
            _ => e.0 += a.len_m(),
        }
    }
    println!("\n--- attached length by the way's own class (km: tagged / detected) ---");
    let mut wc: Vec<(&String, &(f64, f64))> = by_walk_class.iter().collect();
    wc.sort_by(|a, b| (b.1 .0 + b.1 .1).total_cmp(&(a.1 .0 + a.1 .1)));
    for (class, (tagged, detected)) in wc {
        println!("{class:<22} {:>7.1} / {:>7.1}", tagged / 1000.0, detected / 1000.0);
    }

    println!("\n--- clear of the kerb, by host class ---");
    let mut classes: Vec<(&String, &Vec<f64>)> = by_class.iter().collect();
    classes.sort_by_key(|(_, v)| std::cmp::Reverse(v.len()));
    for (class, v) in classes.iter().take(10) {
        let mut v = (*v).clone();
        v.sort_by(f64::total_cmp);
        println!(
            "{class:<14} {:>6}  p10 {:>5.1}  p50 {:>5.1}  p90 {:>5.1}  max {:>5.1}",
            v.len(),
            q(&v, 0.1),
            q(&v, 0.5),
            q(&v, 0.9),
            q(&v, 1.0)
        );
    }

    // How crowded a host is: two sidewalks (one per side) is the healthy shape.
    let mut per_host: HashMap<u32, [f64; 2]> = HashMap::new();
    for a in walks.all() {
        per_host.entry(a.host).or_default()[a.side as usize] += a.len_m();
    }
    let hosted = per_host.len();
    let both_sides = per_host.values().filter(|s| s[0] > 0.0 && s[1] > 0.0).count();
    let paving = scene
        .corridors
        .iter()
        .filter(|c| c.kind.prior().surface == priors::Surface::Asphalt)
        .count();
    println!("\n--- coverage of the street network ---");
    println!(
        "streets with a sidewalk  {hosted} of {paving} asphalt corridors ({:.1} %), \
         {both_sides} on both sides",
        pct(hosted as f64, paving as f64)
    );
}
