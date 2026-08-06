//! Is this footbridge a bridge, or is it the sidewalk on the road bridge?
//!
//! A draped feature's elevated span gets a deck fitted to the finished ground
//! at its two ends (`synth::draped`), on the assumption that it is a structure
//! of its own. Overture maps a road bridge's separated sidewalk as an
//! independently `bridge`-tagged footway, so that assumption is wrong wherever
//! a path runs *along* a road bridge: the fit reads ground under the deck it is
//! standing on, and the result is a second, smaller bridge starting in the
//! riverbed the real one crosses.
//!
//! Measures how often a D-stratum elevated span runs alongside a solved
//! structure span, at what lateral offset, and — where it does — how far the
//! fitted chord ends up from the deck that is actually carrying it. That last
//! number is the size of the defect; the offset histogram is what a threshold
//! would have to separate.
//!
//! Usage: cargo run --release --example carried_probe --
//!            <segment.parquet> <w,s,e,n> <terrain.pmtiles> [water.parquet] [n]

use std::sync::Arc;

use arpentry_server::assemble;
use arpentry_server::dem::Dem;
use arpentry_server::geoparquet::GeoParquet;
use arpentry_server::ground::{self, sampler::GroundSampler, sampler::MeshOptions};
use arpentry_server::levels::LevelRun;
use arpentry_server::priors::{Kind, Stratum};
use arpentry_server::project::Bounds;
use arpentry_server::scene::{metric_len, run_cos_lat, SceneGraph, SpanKind};
use arpentry_server::solve::{self, SolvedModel};
use arpentry_server::value::Value;
use geo_types::{Coord, Geometry};

/// Reference zoom for the run — the rung the model is solved against.
const Z_REF: u8 = 16;
/// Plan spacing along a path while testing it against a carrier.
const STEP_M: f64 = 2.0;
/// How far out to look for a carrier at all. Deliberately wider than any
/// plausible answer, so the histogram shows where the answer actually is
/// instead of being cut off at an assumed one.
const SEARCH_M: f64 = 25.0;

struct Span {
    class: String,
    lon: f64,
    lat: f64,
    len_m: f64,
    /// The fitted chord's two ends (the finished ground there).
    ends: (f64, f64),
    /// Nearest solved *bridge* span, if any: its class, the median lateral
    /// offset over the samples that found it, and the fraction of the path's
    /// samples that did.
    carrier: Option<Carrier>,
    /// Whether the shipping rule (`synth::carried`) claims this span — asked of
    /// the rule itself, not reimplemented here. Where this disagrees with the
    /// columns beside it, the rule is what the archive will show.
    claimed: bool,
}

struct Carrier {
    class: String,
    offset_m: f64,
    covered: f64,
    /// Carrier deck top minus the fitted chord, at the path's two ends and at
    /// its worst sample.
    drop_ends: (f64, f64),
    drop_worst: f64,
}

impl Carrier {
    /// How far the deck stands above the chord at whichever end agrees better.
    fn drop_min_end(&self) -> f64 {
        self.drop_ends.0.abs().min(self.drop_ends.1.abs())
    }
}

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let bb: Vec<f64> = a[1].split(',').map(|s| s.parse().expect("bbox")).collect();
    let bbox = Bounds { west: bb[0], south: bb[1], east: bb[2], north: bb[3] };
    let terrain = std::path::PathBuf::from(&a[2]);
    let water = a.get(3).filter(|s| s.ends_with(".parquet")).map(std::path::PathBuf::from);
    let show: usize = a.iter().find_map(|s| s.parse::<usize>().ok()).unwrap_or(12);

    let mut scene =
        assemble::run(std::path::Path::new(&a[0]), water.as_deref(), &bbox).expect("assemble");
    let solved = solve::run(&mut scene, Some(&terrain), Z_REF, 0).expect("solve");
    let stack = Arc::new(ground::derive(&scene, &solved, Some(&terrain), 0));
    let dem = Dem::open(&terrain).ok();
    let mut sampler = GroundSampler::new(dem, stack, solved.z_ref, MeshOptions::default());

    // Every solved bridge deck in the scene, as the tiler cuts them, and the
    // shipping rule's own index over the same decks.
    let decks = decks(&scene, &solved);
    let carriers = arpentry_server::synth::carried::Carriers::build(&scene, &solved);
    println!("\n{} solved bridge spans to be carried by\n", decks.len());

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
            continue;
        }
        let Geometry::LineString(line) = &f.geometry else { continue };
        if line.0.len() < 2 {
            continue;
        }
        // Seated, not annotated: the abutments the generator will actually use
        // (`synth::draped::seat`). Measuring the raw annotation would report a
        // defect the pipeline has already half-corrected, and would disagree
        // with `contact.deck_carried` reading the emitted archive.
        let runs = arpentry_server::synth::draped::seat(line, &f.level_runs, &mut sampler, Z_REF);
        for run in runs.iter().filter(|r| r.level > 0) {
            if let Some(s) = measure(
                &line.0,
                run,
                &class,
                &decks,
                &scene,
                &solved,
                &carriers,
                &mut sampler,
                solved.z_ref,
            ) {
                if bbox.contains(s.lon, s.lat) {
                    spans.push(s);
                }
            }
        }
    }

    let carried: Vec<&Span> = spans.iter().filter(|s| s.carrier.is_some()).collect();
    println!("{} elevated spans on draped features", spans.len());
    println!(
        "  {} of them run along a solved bridge deck ({:.1} %)\n",
        carried.len(),
        100.0 * carried.len() as f64 / spans.len().max(1) as f64
    );

    hist("lateral offset to the carrier (m)",
         carried.iter().map(|s| s.carrier.as_ref().expect("carried").offset_m));
    hist("fraction of the path's length over that carrier",
         carried.iter().map(|s| s.carrier.as_ref().expect("carried").covered));
    hist("carrier deck above the fitted chord, worst sample (m)",
         carried.iter().map(|s| s.carrier.as_ref().expect("carried").drop_worst));
    // The discriminator: a sidewalk *joins* its road bridge, so wherever the
    // annotation and the DEM happen to agree the chord already lands on the
    // deck. A path passing underneath never touches it at either end.
    hist("carrier deck above the chord at its *closer* end (m)",
         carried.iter().map(|s| s.carrier.as_ref().expect("carried").drop_min_end()));
    hist("span length (m)", spans.iter().map(|s| s.len_m));

    println!("\nwhat a lateral threshold would claim\n");
    println!("  gap     spans claimed    median drop   worst drop");
    for gap in [4.0, 6.0, 8.0, 10.0, 12.0, 16.0] {
        let mut d: Vec<f64> = carried
            .iter()
            .filter_map(|s| s.carrier.as_ref())
            .filter(|c| c.offset_m <= gap)
            .map(|c| c.drop_worst)
            .collect();
        d.sort_by(|a, b| a.partial_cmp(b).expect("finite"));
        println!(
            "  {gap:4.0} m  {:4} ({:5.1} %)    {:8.2} m   {:8.2} m",
            d.len(),
            100.0 * d.len() as f64 / spans.len().max(1) as f64,
            d.get(d.len() / 2).copied().unwrap_or(0.0),
            d.last().copied().unwrap_or(0.0),
        );
    }

    println!("\nand what an end-agreement ceiling adds, at a 10 m lateral gap\n");
    println!("  ceiling   claimed   rejected   median correction");
    for ceil in [1.0, 1.5, 2.0, 3.0, 4.0, 1e9] {
        let claimed: Vec<&Carrier> = carried
            .iter()
            .filter_map(|s| s.carrier.as_ref())
            .filter(|c| c.offset_m <= 10.0 && c.drop_min_end() <= ceil)
            .collect();
        let mut d: Vec<f64> = claimed.iter().map(|c| c.drop_worst).collect();
        d.sort_by(|a, b| a.partial_cmp(b).expect("finite"));
        println!(
            "  {:>6}    {:4}      {:4}       {:8.2} m",
            if ceil > 1e8 { "none".to_string() } else { format!("{ceil:.1} m") },
            claimed.len(),
            carried.len() - claimed.len(),
            d.get(d.len() / 2).copied().unwrap_or(0.0),
        );
    }

    let mut worst: Vec<&&Span> = carried.iter().collect();
    worst.sort_by(|x, y| {
        let k = |s: &Span| s.carrier.as_ref().map_or(0.0, |c| c.drop_worst);
        k(y).partial_cmp(&k(x)).expect("finite")
    });
    println!(
        "\nthe shipping rule (synth::carried) claims {} of the {} spans probed, \
         {} of the {} the lateral search found\n",
        spans.iter().filter(|s| s.claimed).count(),
        spans.len(),
        carried.iter().filter(|s| s.claimed).count(),
        carried.len(),
    );

    println!("\nthe {show} carried spans whose fitted chord is furthest below its carrier\n");
    for s in worst.iter().take(show) {
        let c = s.carrier.as_ref().expect("carried");
        println!(
            "  {:<9} {:.6},{:.6}  len {:5.1} m  chord {:7.2}/{:7.2}  \
             carrier {:<12} off {:4.1} m  cover {:3.0} %  drop {:+.2}/{:+.2} worst {:+.2}",
            s.class, s.lon, s.lat, s.len_m, s.ends.0, s.ends.1,
            c.class, c.offset_m, c.covered * 100.0, c.drop_ends.0, c.drop_ends.1, c.drop_worst,
        );
        println!("            rule: {}", if s.claimed { "carried" } else { "NOT claimed" });
    }
}

const ATTRS: &[&str] = &["id", "type", "subtype", "class", "subclass", "level_rules", "road_flags"];

fn prop(props: &[(String, Value)], key: &str) -> Option<String> {
    props.iter().find(|(k, _)| k == key).and_then(|(_, v)| match v {
        Value::String(s) => Some(s.clone()),
        _ => None,
    })
}

/// One solved bridge span, as the tiler cuts them: the corridor it belongs to
/// and the arc range over which a deck is actually swept.
struct Deck {
    corridor: u32,
    arc0: f64,
    arc1: f64,
}

fn decks(scene: &SceneGraph, solved: &SolvedModel) -> Vec<Deck> {
    let mut out = Vec::new();
    for c in &scene.corridors {
        let Some(p) = solved.profile(c.id) else { continue };
        for s in solve::portals::reconcile_spans(p, &c.spans) {
            if s.kind == SpanKind::Bridge {
                out.push(Deck { corridor: c.id, arc0: s.arc0, arc1: s.arc1 });
            }
        }
    }
    out
}

/// The nearest carrier deck under a plan point, as `(deck index, offset, deck
/// top height)`. Offset is the plan distance from the point to the carrier's
/// centerline; the search is capped at [`SEARCH_M`].
fn carrier_at(
    c: Coord,
    decks: &[Deck],
    scene: &SceneGraph,
    solved: &SolvedModel,
) -> Option<(usize, f64, f64)> {
    let mut best: Option<(usize, f64, f64)> = None;
    for (i, d) in decks.iter().enumerate() {
        let corr = &scene.corridors[d.corridor as usize];
        let Some(p) = solved.profile(d.corridor) else { continue };
        // Cheap plan reject before any projection work.
        let arc = p.arc_of(c.x, c.y);
        if arc < d.arc0 - STEP_M || arc > d.arc1 + STEP_M {
            continue;
        }
        let on = p.point_at_arc(arc);
        let dist = metric_len(c, on, corr.cos_lat);
        if dist > SEARCH_M {
            continue;
        }
        if best.is_none_or(|(_, b, _)| dist < b) {
            best = Some((i, dist, p.deck_at_arc(arc)));
        }
    }
    best
}

#[allow(clippy::too_many_arguments)]
fn measure(
    nodes: &[Coord],
    run: &LevelRun,
    class: &str,
    decks: &[Deck],
    scene: &SceneGraph,
    solved: &SolvedModel,
    carriers: &arpentry_server::synth::carried::Carriers,
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
    let ends = (
        sampler.ground(at(s0).x, at(s0).y, z),
        sampler.ground(at(s1).x, at(s1).y, z),
    );
    let len_m = s1 - s0;
    // The chord the fit would build, so the drop can be read against it.
    let chord = |d: f64| ends.0 + (ends.1 - ends.0) * ((d - s0) / len_m).clamp(0.0, 1.0);

    // Walk the span, asking at each sample which deck (if any) is carrying it.
    let mut hits: std::collections::HashMap<usize, Vec<f64>> = std::collections::HashMap::new();
    let mut tops: std::collections::HashMap<usize, Vec<(f64, f64)>> =
        std::collections::HashMap::new();
    let mut n = 0usize;
    let mut d = s0;
    while d <= s1 {
        n += 1;
        if let Some((i, off, top)) = carrier_at(at(d), decks, scene, solved) {
            hits.entry(i).or_default().push(off);
            tops.entry(i).or_default().push((d, top));
        }
        d += STEP_M;
    }
    // The carrier is the deck that covers most of the path.
    let carrier = hits
        .iter()
        .max_by_key(|(_, offs)| offs.len())
        .map(|(&i, offs)| {
            let mut o = offs.clone();
            o.sort_by(|a, b| a.partial_cmp(b).expect("finite"));
            let samples = &tops[&i];
            let drop_worst = samples
                .iter()
                .map(|&(d, top)| top - chord(d))
                .fold(f64::NEG_INFINITY, f64::max);
            let end_drop = |want: f64| {
                samples
                    .iter()
                    .min_by(|a, b| {
                        (a.0 - want).abs().partial_cmp(&(b.0 - want).abs()).expect("finite")
                    })
                    .map_or(0.0, |&(d, top)| top - chord(d))
            };
            Carrier {
                class: scene.corridors[decks[i].corridor as usize].class_key.clone(),
                offset_m: o[o.len() / 2],
                covered: offs.len() as f64 / n.max(1) as f64,
                drop_ends: (end_drop(s0), end_drop(s1)),
                drop_worst,
            }
        });

    // The piece the tiler cuts for this run — the two interpolated ends with
    // every source vertex between them — put to the rule as it will see it.
    let mut piece = vec![at(s0)];
    piece.extend(
        nodes.iter().enumerate().filter(|&(i, _)| arc[i] > s0 && arc[i] < s1).map(|(_, &c)| c),
    );
    piece.push(at(s1));
    let claimed = carriers.of(&piece, scene, solved, sampler, z).is_some();

    let mid = at(0.5 * (s0 + s1));
    Some(Span { class: class.to_string(), lon: mid.x, lat: mid.y, len_m, ends, carrier, claimed })
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
