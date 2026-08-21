//! Which alignments earned a grade escape, and did they earn it on a slope or
//! on a notch?
//!
//! `profile::measured_grade` raises a class's grade ceiling to the p90 of the
//! **per-edge** absolute grades of the conditioned reference under its at-grade
//! stretches. That is what lets the Montreux–Glion rack line keep its 11 %
//! bed under a 7 % adhesion prior (S18). The worry the plan records is the
//! other shape: a V-shaped plunge-and-recover inside one notch span has steep
//! *edges* while going nowhere, so it could buy a licence to dive that a road
//! crossing a gully should not have (the Chauderon lanes, "23–41 % under a
//! 15 % cap").
//!
//! This measures the difference. For every corridor whose ceiling was raised,
//! it recomputes the same percentile over **windowed** grades — the reference's
//! net rise over `NOTCH_SPAN_M`, which a V cancels out of and a slope does not
//! — and reports how many escapes survive, how far each would fall, and
//! whether the corridor's solved road ever used the escape it was given.
//!
//! Usage: cargo run --release --example grade_census -- <segment.parquet> <w,s,e,n> <terrain.pmtiles> [corridor]

use arpentry_server::priors::Stratum;
use arpentry_server::project::Bounds;
use arpentry_server::solve::profile::condition_reference;
use arpentry_server::solve::Mode;
use arpentry_server::{assemble, solve};

/// The window a "sustained" grade must hold over, metres. `NOTCH_SPAN_M` —
/// the same width the reference conditioning calls a notch, so a dip the
/// closing would have filled cannot buy an escape.
const WINDOW_M: f64 = 60.0;

/// `profile::MEASURED_GRADE_PCTL`, restated (the constant is private).
const PCTL: f64 = 0.90;

/// The percentile of `grades`, by the same rule `measured_grade` uses.
fn pctl(grades: &mut [f64]) -> Option<f64> {
    if grades.is_empty() {
        return None;
    }
    grades.sort_by(f64::total_cmp);
    Some(grades[((grades.len() - 1) as f64 * PCTL).round() as usize])
}

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let bb: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let bbox = Bounds { west: bb[0], south: bb[1], east: bb[2], north: bb[3] };
    let terrain = std::path::PathBuf::from(&a[2]);
    let dump: Option<u32> = a.get(3).and_then(|s| s.parse().ok());

    let mut scene = assemble::run(std::path::Path::new(&a[0]), None, &bbox).expect("assemble");
    let solved = solve::run(&mut scene, Some(&terrain), 16, 0).expect("solve");

    let (mut raised, mut used, mut survives, mut collapses) = (0usize, 0usize, 0usize, 0usize);
    // (drop, class, edge_pctl, window_pctl, used_m, lon, lat, id, kind)
    let mut sites: Vec<(f64, f64, f64, f64, f64, f64, f64, u32, String)> = Vec::new();

    for c in &scene.corridors {
        if c.kind.stratum() == Stratum::D {
            continue;
        }
        let Some(p) = solved.profile(c.id) else { continue };
        let Some(class) = Mode::for_kind(c.kind).grade() else { continue };
        let Some(ceiling) = p.max_grade() else { continue };
        if ceiling <= class + 1e-9 {
            continue; // no escape
        }
        raised += 1;
        let (arc, at_grade) = (p.arc(), p.at_grade());
        let reference = condition_reference(arc, p.terrain_m());
        // The escape as granted: per-edge absolute grades of the reference.
        let mut edges: Vec<f64> = Vec::new();
        for i in 1..arc.len() {
            if !at_grade[i] || !at_grade[i - 1] || arc[i] <= arc[i - 1] {
                continue;
            }
            edges.push((reference[i] - reference[i - 1]).abs() / (arc[i] - arc[i - 1]));
        }
        // The same percentile over the reference's *net* rise across a window:
        // a plunge-and-recover cancels, a hillside does not. Windows are taken
        // **inside each maximal at-grade run**, and a run shorter than the
        // window contributes its own end-to-end grade — otherwise a line
        // peppered with structures (the rack railway, which is exactly the
        // case the escape exists for) samples almost no windows and looks flat
        // by omission.
        let mut windows: Vec<f64> = Vec::new();
        let mut k = 0;
        while k < arc.len() {
            if !at_grade[k] {
                k += 1;
                continue;
            }
            let f = k;
            while k + 1 < arc.len() && at_grade[k + 1] {
                k += 1;
            }
            let l = k;
            k += 1;
            if l <= f {
                continue;
            }
            if arc[l] - arc[f] <= WINDOW_M {
                windows.push((reference[l] - reference[f]).abs() / (arc[l] - arc[f]));
                continue;
            }
            for i in f..=l {
                let target = (arc[i] + WINDOW_M).min(arc[l]);
                let j = arc[..=l].partition_point(|&x| x < target).min(l);
                if j <= i || arc[j] <= arc[i] {
                    continue;
                }
                windows.push((reference[j] - reference[i]).abs() / (arc[j] - arc[i]));
            }
        }
        let (Some(edge_p), Some(win_p)) = (pctl(&mut edges), pctl(&mut windows)) else { continue };
        // Did the solved road ever spend the escape?
        let road = p.road_m();
        let mut worst_used = 0.0f64;
        for i in 1..arc.len() {
            if arc[i] <= arc[i - 1] {
                continue;
            }
            worst_used = worst_used.max((road[i] - road[i - 1]).abs() / (arc[i] - arc[i - 1]));
        }
        if worst_used > class + 1e-3 {
            used += 1;
        }
        let windowed_ceiling = win_p.max(class);
        let drop = ceiling - windowed_ceiling;
        if drop > 1e-3 {
            collapses += 1;
        } else {
            survives += 1;
        }
        let pt = p.point_at_arc(0.5 * arc[arc.len() - 1]);
        if pt.x < bbox.west || pt.x > bbox.east || pt.y < bbox.south || pt.y > bbox.north {
            continue;
        }
        sites.push((drop, class, edge_p, win_p, worst_used, pt.x, pt.y, c.id, format!("{:?}", c.kind)));
        if dump == Some(c.id) {
            println!(
                "  #{} {:?} class {:.0} % ceiling {:.0} % (edges p90 {:.0} %, windows p90 {:.0} %) \
                 road uses {:.0} %",
                c.id,
                c.kind,
                class * 100.0,
                ceiling * 100.0,
                edge_p * 100.0,
                win_p * 100.0,
                worst_used * 100.0
            );
        }
    }

    println!("\nGRADE ESCAPES over {} corridors", scene.corridors.len());
    println!("  ceiling raised above the class prior   {raised}");
    println!("    and the solved road spends it        {used}");
    println!("  would survive a {WINDOW_M:.0} m window          {survives}");
    println!("  would collapse to the class or below   {collapses}");

    // Per kind, because the escape exists for the rack railway and the steep
    // lanes and the question is whether a rule keeps them.
    let mut kinds: Vec<(String, usize, usize, f64)> = Vec::new();
    for (drop, _, _, w, _, _, _, _, kind) in &sites {
        match kinds.iter_mut().find(|k| &k.0 == kind) {
            Some(k) => {
                k.1 += 1;
                k.2 += usize::from(*drop <= 1e-3);
                k.3 = k.3.max(*w);
            }
            None => kinds.push((kind.clone(), 1, usize::from(*drop <= 1e-3), *w)),
        }
    }
    kinds.sort_by_key(|k| std::cmp::Reverse(k.1));
    println!("\n  {:<28} {:>8} {:>10} {:>12}", "kind", "escapes", "survive", "best window");
    for (kind, n, keep, w) in &kinds {
        println!("  {kind:<28} {n:>8} {keep:>10} {:>11.0} %", w * 100.0);
    }

    sites.sort_by(|a, b| b.0.total_cmp(&a.0));
    println!("\nBIGGEST COLLAPSES UNDER THE WINDOWED RULE (top 25)");
    for (drop, class, e, w, used, lon, lat, id, kind) in sites.iter().take(25) {
        println!(
            "  -{:.0} pp  {lon:.6},{lat:.6}  #{id} {kind} class {:.0} % edges {:.0} % windows \
             {:.0} % road {:.0} %",
            drop * 100.0,
            class * 100.0,
            e * 100.0,
            w * 100.0,
            used * 100.0
        );
    }
}
