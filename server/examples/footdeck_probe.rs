//! Where does a footbridge deck *start*, and is that the bank or the riverbed?
//!
//! A draped feature's deck is chorded between the finished ground at the two
//! ends of the annotated span (`synth::draped`). Against a near-vertical DEM
//! wall — the side of a gorge, a stream cut, a retaining wall — two metres of
//! plan disagreement between the DEM and the vector data is fourteen metres of
//! height error, and the chord starts part way down the wall. That is a bridge
//! beginning in the middle of the riverbed it crosses.
//!
//! The rule this calibrates: **a path cannot descend a cliff.** Marching
//! outward from an abutment, while the ground climbs steeper than the class
//! could walk, the abutment is not standing on ground the path can be on. The
//! first sample where the climb relaxes is the bank, and that is where the deck
//! should have started. Where the climb never relaxes inside the cap the flank
//! is genuine and nothing moves.
//!
//! Reports the population the rule scores, then what each candidate ceiling
//! buys — the table the constant's doc comment needs.
//!
//! Usage: cargo run --release --example footdeck_probe --
//!            <segment.parquet> <w,s,e,n> <terrain.pmtiles> [water.parquet] [n] [grade]

use std::sync::Arc;

use arpentry_server::assemble;
use arpentry_server::dem::Dem;
use arpentry_server::geoparquet::GeoParquet;
use arpentry_server::ground::{self, sampler::GroundSampler, sampler::MeshOptions};
use arpentry_server::levels::LevelRun;
use arpentry_server::priors::{Kind, Stratum};
use arpentry_server::project::Bounds;
use arpentry_server::scene::{metric_len, run_cos_lat};
use arpentry_server::solve;
use arpentry_server::value::Value;
use geo_types::{Coord, Geometry};

/// Reference zoom for the run — the rung the model is solved against.
const Z_REF: u8 = 16;
/// Ground sample spacing along the path, and the baseline the outward climb is
/// measured over.
const STEP_M: f64 = 2.0;
/// How far outward an abutment may be moved looking for the bank.
const CAP_M: f64 = 30.0;

/// Candidate ceilings for "steeper than the path could walk".
const CEILINGS: [f64; 5] = [0.4, 0.6, 0.8, 1.0, 1.5];

struct Span {
    class: String,
    lon: f64,
    lat: f64,
    len_m: f64,
    /// Deck chord at the two abutments (the finished ground there).
    ends: (f64, f64),
    /// Lowest ground under the span.
    floor: f64,
    /// Path available outside the span, per side, in metres.
    room: [f64; 2],
    /// Local outward ground grade at each abutment, over the first `STEP_M`.
    wall: [f64; 2],
    /// What the rule does at each ceiling, per side: `(moved_m, lift_m)`.
    fix: Vec<[Option<(f64, f64)>; 2]>,
    /// Ground along the path from the near abutment, `(arc, ground)`.
    prof: Vec<(f64, f64)>,
}

impl Span {
    /// The chord's own grade. A deck is a structure someone built: past a few
    /// tens of percent the chord is not a bridge, it is two ground samples
    /// taken on opposite flanks of something.
    fn grade(&self) -> f64 {
        (self.ends.1 - self.ends.0).abs() / self.len_m.max(1e-6)
    }
}

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let bb: Vec<f64> = a[1].split(',').map(|s| s.parse().expect("bbox")).collect();
    let bbox = Bounds { west: bb[0], south: bb[1], east: bb[2], north: bb[3] };
    let terrain = std::path::PathBuf::from(&a[2]);
    let water = a.get(3).filter(|s| s.ends_with(".parquet")).map(std::path::PathBuf::from);
    let show: usize = a.iter().find_map(|s| s.parse::<usize>().ok()).unwrap_or(10);

    // The finished world the deck is fitted to: solve the scene, imprint the
    // ground, then read it exactly as the emit path does.
    let mut scene =
        assemble::run(std::path::Path::new(&a[0]), water.as_deref(), &bbox).expect("assemble");
    let solved = solve::run(&mut scene, Some(&terrain), Z_REF, 0).expect("solve");
    let stack = Arc::new(ground::derive(&scene, &solved, Some(&terrain), 0));
    let dem = Dem::open(&terrain).ok();
    let mut sampler = GroundSampler::new(dem, stack, solved.z_ref, MeshOptions::default());

    let mut spans: Vec<Span> = Vec::new();
    let gp = GeoParquet::open(&a[0]).expect("open segments");
    let rgs = gp.row_groups_intersecting((bbox.west, bbox.south, bbox.east, bbox.north));
    for f in gp.features(rgs, ATTRS).expect("features") {
        let Ok(f) = f else { continue };
        let class = prop(&f.properties, "class").unwrap_or_default();
        let kind = Kind::parse(
            prop(&f.properties, "subtype").as_deref(),
            Some(class.as_str()),
            prop(&f.properties, "subclass").as_deref(),
        );
        if kind.stratum() != Stratum::D {
            continue; // solved features get their deck from the solve
        }
        let Geometry::LineString(line) = &f.geometry else { continue };
        if line.0.len() < 2 {
            continue;
        }
        for run in f.level_runs.iter().filter(|r| r.level > 0) {
            // The parquet filter is per row group, so features well outside the
            // bbox arrive with it; a span outside the zone is not this run's.
            if let Some(s) = measure(&line.0, run, &class, &mut sampler, solved.z_ref) {
                if bbox.contains(s.lon, s.lat) {
                    spans.push(s);
                }
            }
        }
    }

    println!("\n{} elevated spans on draped features\n", spans.len());
    hist("span length (m)", spans.iter().map(|s| s.len_m));
    hist("the deck's clear over the ground it spans (m)",
         spans.iter().map(|s| s.ends.0.min(s.ends.1) - s.floor));
    hist("the chord's own grade", spans.iter().map(|s| s.grade()));
    hist("path available outside the span, per side (m)",
         spans.iter().flat_map(|s| s.room));
    hist("the ground's outward grade at an abutment",
         spans.iter().flat_map(|s| s.wall));
    let flush = spans.iter().flat_map(|s| s.room).filter(|&r| r < 1.0).count();
    println!(
        "  abutments with no path beyond them at all: {flush} of {} ({:.1} %)",
        2 * spans.len(),
        100.0 * flush as f64 / (2 * spans.len()) as f64
    );

    println!("\nwhat each ceiling buys — abutments moved out to the bank\n");
    println!("  ceiling   moved          median move   median lift    worst lift");
    for (ci, ceiling) in CEILINGS.iter().enumerate() {
        let moves: Vec<(f64, f64)> =
            spans.iter().flat_map(|s| s.fix[ci]).flatten().collect();
        let n = moves.len();
        let mut d: Vec<f64> = moves.iter().map(|m| m.0).collect();
        let mut l: Vec<f64> = moves.iter().map(|m| m.1).collect();
        d.sort_by(|a, b| a.partial_cmp(b).expect("finite"));
        l.sort_by(|a, b| a.partial_cmp(b).expect("finite"));
        println!(
            "  {:>5.0} %   {n:4} ({:5.1} %)   {:8.1} m   {:8.2} m   {:8.2} m",
            ceiling * 100.0,
            100.0 * n as f64 / (2 * spans.len()).max(1) as f64,
            d.get(n / 2).copied().unwrap_or(0.0),
            l.get(n / 2).copied().unwrap_or(0.0),
            l.last().copied().unwrap_or(0.0),
        );
    }

    let by_grade = a.iter().any(|s| s == "grade");
    spans.sort_by(|x, y| {
        let key = |s: &Span| {
            if by_grade {
                s.grade()
            } else {
                s.fix[1].iter().flatten().map(|m| m.1).fold(0.0, f64::max)
            }
        };
        key(y).partial_cmp(&key(x)).expect("finite")
    });
    println!(
        "\nthe {show} spans with the {}\n",
        if by_grade { "steepest chord" } else { "largest correction at a 60 % ceiling" }
    );
    for s in spans.iter().take(show) {
        println!(
            "  {:<9} {:.6},{:.6}  len {:5.1} m  ends {:.1}/{:.1}  floor {:.1}  grade {:4.0} %  \
             wall {:3.0}/{:3.0} %  room {:4.1}/{:4.1} m  fix {}",
            s.class, s.lon, s.lat, s.len_m, s.ends.0, s.ends.1, s.floor,
            s.grade() * 100.0,
            s.wall[0] * 100.0, s.wall[1] * 100.0,
            s.room[0], s.room[1],
            s.fix[1]
                .iter()
                .map(|m| match m {
                    Some((d, h)) => format!("+{d:.0}m/{h:+.1}m"),
                    None => "-".to_string(),
                })
                .collect::<Vec<_>>()
                .join(" ")
        );
        print_profile(s);
    }
}

const ATTRS: &[&str] = &["id", "type", "subtype", "class", "subclass", "level_rules", "road_flags"];

fn prop(props: &[(String, Value)], key: &str) -> Option<String> {
    props.iter().find(|(k, _)| k == key).and_then(|(_, v)| match v {
        Value::String(s) => Some(s.clone()),
        _ => None,
    })
}

/// One elevated span: its abutments, the ground under it, and what the rule
/// would do to each end at every candidate ceiling.
fn measure(
    nodes: &[Coord],
    run: &LevelRun,
    class: &str,
    sampler: &mut GroundSampler,
    z: u8,
) -> Option<Span> {
    let cos_lat = run_cos_lat(nodes);
    let mut arc = Vec::with_capacity(nodes.len());
    let mut acc = 0.0;
    for (i, &c) in nodes.iter().enumerate() {
        if i > 0 {
            acc += metric_len(nodes[i - 1], c, cos_lat);
        }
        arc.push(acc);
    }
    let total = acc;
    if total <= 1.0 {
        return None;
    }
    let (s0, s1) = ((run.start * total).clamp(0.0, total), (run.end * total).clamp(0.0, total));
    if s1 - s0 < 1.0 {
        return None;
    }
    let at = |d: f64| -> Coord {
        let i = arc.partition_point(|&a| a < d).clamp(1, arc.len() - 1);
        let (a0, a1) = (arc[i - 1], arc[i]);
        let t = if a1 > a0 { ((d - a0) / (a1 - a0)).clamp(0.0, 1.0) } else { 0.0 };
        Coord {
            x: nodes[i - 1].x + (nodes[i].x - nodes[i - 1].x) * t,
            y: nodes[i - 1].y + (nodes[i].y - nodes[i - 1].y) * t,
        }
    };
    let ground_at = |d: f64, sampler: &mut GroundSampler| {
        let c = at(d);
        sampler.ground(c.x, c.y, z)
    };

    let ends = (ground_at(s0, sampler), ground_at(s1, sampler));
    let mut floor = f64::INFINITY;
    let mut d = s0;
    while d <= s1 {
        floor = floor.min(ground_at(d, sampler));
        d += STEP_M;
    }
    let room = [s0, total - s1];
    let wall = [
        (ground_at((s0 - STEP_M).max(0.0), sampler) - ends.0) / STEP_M,
        (ground_at((s1 + STEP_M).min(total), sampler) - ends.1) / STEP_M,
    ];

    // The rule, at each candidate ceiling: march outward while the ground
    // climbs faster than the path could walk; the first relaxation is the bank.
    let bank = |from: f64, dir: f64, ceiling: f64, sampler: &mut GroundSampler| {
        let start = ground_at(from, sampler);
        let (mut prev, mut moved) = (start, 0.0);
        let mut d = STEP_M;
        while d <= CAP_M {
            let s = from + dir * d;
            if s < 0.0 || s > total {
                return None; // the path ends: no bank on this side
            }
            let h = ground_at(s, sampler);
            if (h - prev) / STEP_M < ceiling {
                // The climb relaxed. Something was gained only if the march
                // actually walked a wall first.
                return (moved > 0.0).then_some((moved, prev - start));
            }
            (prev, moved) = (h, d);
            d += STEP_M;
        }
        None // still climbing at the cap: a genuine flank, not a bank
    };
    let fix = CEILINGS
        .iter()
        .map(|&c| [bank(s0, -1.0, c, sampler), bank(s1, 1.0, c, sampler)])
        .collect();

    let mut prof = Vec::new();
    let mut d = -CAP_M;
    while d <= s1 - s0 + CAP_M {
        let s = s0 + d;
        if (0.0..=total).contains(&s) {
            prof.push((d, ground_at(s, sampler)));
        }
        d += STEP_M;
    }

    let mid = at(0.5 * (s0 + s1));
    Some(Span {
        class: class.to_string(),
        lon: mid.x,
        lat: mid.y,
        len_m: s1 - s0,
        ends,
        floor,
        room,
        wall,
        fix,
        prof,
    })
}

/// The ground along the path, with the deck chord marked — the shape of the
/// failure rather than a score for it.
fn print_profile(s: &Span) {
    let base = s.prof.iter().map(|&(_, h)| h).fold(f64::INFINITY, f64::min);
    let top = s.prof.iter().map(|&(_, h)| h).fold(f64::NEG_INFINITY, f64::max);
    let scale = ((top - base) / 60.0).max(0.05);
    for &(d, h) in &s.prof {
        let on_deck = (0.0..=s.len_m).contains(&d);
        let deck = s.ends.0 + (s.ends.1 - s.ends.0) * (d / s.len_m).clamp(0.0, 1.0);
        let col = |v: f64| (((v - base) / scale).round().max(0.0) as usize).min(70);
        let mut row = vec![b' '; 72];
        for c in row.iter_mut().take(col(h)) {
            *c = b'.';
        }
        if on_deck {
            row[col(deck)] = b'#';
        }
        println!(
            "      {d:6.0} m {h:8.2} {:>8} |{}",
            if on_deck { format!("{deck:.2}") } else { String::new() },
            String::from_utf8_lossy(&row).trim_end()
        );
    }
    println!();
}

/// Deciles plus the far tail — enough to see a second mode without a plot.
fn hist(title: &str, values: impl Iterator<Item = f64>) {
    let mut v: Vec<f64> = values.filter(|x| x.is_finite()).collect();
    if v.is_empty() {
        println!("{title}: no samples");
        return;
    }
    v.sort_by(|a, b| a.partial_cmp(b).expect("finite"));
    let q = |p: f64| v[((v.len() - 1) as f64 * p) as usize];
    print!("{title}: n={}", v.len());
    for p in [0.0, 0.05, 0.25, 0.5, 0.75, 0.95, 1.0] {
        print!("  p{:.0}={:.2}", p * 100.0, q(p));
    }
    println!();
}
