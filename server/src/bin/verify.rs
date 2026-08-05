//! `arpentry_verify` — measures an emitted archive against the invariants.
//!
//! ```sh
//! # What is the state of the scene?
//! arpentry_verify data/overture-ch/preview.arpa
//!
//! # Did what I just did make it better?
//! arpentry_verify preview.arpa --baseline verify/baseline-montreux-z16.json
//!
//! # What is happening at the place that looks wrong?
//! arpentry_verify preview.arpa --at 6.9290,46.4200
//!
//! # Where in this data does each canonical situation actually occur?
//! arpentry_verify preview.arpa --mine > verify/scenarios.json
//! ```
//!
//! Exits 1 when a metric regressed against a baseline, so it can gate a commit;
//! exits 0 otherwise, including when defects exist but have not got worse. The
//! archive has known, documented deviations, and a tool that failed on every
//! run would be turned off within a day.

use std::path::PathBuf;
use std::process::ExitCode;

use arpentry_server::verify::checks::{self, Options};
use arpentry_server::verify::report::{self, Move};
use arpentry_server::verify::section::{self, Cut};
use arpentry_server::verify::{corpus, scene::ArchiveScan};

const USAGE: &str = "\
arpentry_verify <archive.arpa> [options]

  --zoom <z>[,<z>…]   Zooms to measure (default: the archive's finest)
  --at <lon,lat>      Only the tile containing this position
  --scenario <Sn>     Only the corpus site for this situation (implies --at)
  --corpus <path>     Corpus file (default: verify/scenarios.json beside the archive's repo)
  --spacing <m>       Plan spacing of surface samples in metres (default: 1.0)
  --worst <k>         Offenders to keep per metric (default: 8)
  --max-tiles <n>     Cap on tiles visited per zoom (default: 4096)
  --model <path>      Merge a model-side scorecard written by
                      `arpentry_tiler --verify-model`. The structural checks
                      (I7 authority, I8 ground footprint, I5 determinism)
                      measure how the scene was computed, which no archive can
                      answer; merged here so one table and one baseline cover
                      both halves.
  --baseline <path>   Diff against a committed scorecard; exit 1 on regression
  --json <path>       Write this run's scorecard as JSON (\"-\" for stdout)
  --mine              Propose corpus sites from this archive and exit
  --list              List the canonical situations and exit

  --section <path>    Cut a vertical section at --at and write it as SVG.
                      A height model is legible in section and barely legible
                      in perspective: a deck ploughing into a hillside or two
                      at-grade regions metres apart are obvious here and a few
                      pixels of shading in a screenshot.
  --bearing <deg>     Section direction, clockwise from north (default 90)
  --length <m>        Section length, centred on --at (default 200)
";

struct Args {
    archive: PathBuf,
    opt: Options,
    baseline: Option<PathBuf>,
    model: Option<PathBuf>,
    json: Option<String>,
    corpus_path: Option<PathBuf>,
    scenario: Option<String>,
    mine: bool,
    list: bool,
    section: Option<String>,
    bearing: f64,
    length_m: f64,
}

fn main() -> ExitCode {
    let args = match parse() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("{e}\n\n{USAGE}");
            return ExitCode::from(2);
        }
    };

    if args.list {
        for s in corpus::catalogue() {
            println!(
                "{:<4} {:<42} {:<34} {}{}",
                s.id,
                s.name,
                s.mechanism,
                s.stresses,
                if s.minable { "" } else { "  [site must be chosen by hand]" }
            );
        }
        return ExitCode::SUCCESS;
    }

    let bytes = match std::fs::read(&args.archive) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("cannot read {}: {e}", args.archive.display());
            return ExitCode::from(2);
        }
    };
    let scan = match ArchiveScan::open(&bytes) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("not an archive: {e}");
            return ExitCode::from(2);
        }
    };

    let zoom = args.opt.zooms.first().copied().unwrap_or_else(|| scan.max_zoom());

    if args.mine {
        let sites = corpus::mine(&scan, zoom, args.opt.max_tiles);
        println!("{}", serde_json::to_string_pretty(&corpus::to_json(&sites)).unwrap_or_default());
        eprintln!(
            "mined {} of {} situations from z{zoom}; the rest need a site chosen by hand",
            sites.len(),
            corpus::catalogue().iter().filter(|s| s.minable).count()
        );
        return ExitCode::SUCCESS;
    }

    let mut opt = args.opt;
    if let Some(id) = &args.scenario {
        // (resolved below, before the section is cut, so --scenario --section
        // works without also passing --at)
        let path = args
            .corpus_path
            .clone()
            .unwrap_or_else(|| default_corpus(&args.archive));
        match corpus::load(&path).get(id) {
            Some(site) => {
                opt.at = Some((site.lon, site.lat));
                opt.zooms = vec![site.zoom];
                eprintln!("{id} at {:.6},{:.6} z{} — {}", site.lon, site.lat, site.zoom, site.source);
            }
            None => {
                eprintln!("no site for {id} in {}; run --mine first", path.display());
                return ExitCode::from(2);
            }
        }
    }

    if let Some(path) = &args.section {
        let Some((lon, lat)) = opt.at else {
            eprintln!("--section needs a place: pass --at lon,lat or --scenario Sn");
            return ExitCode::from(2);
        };
        let cut = Cut {
            lon,
            lat,
            bearing: args.bearing,
            length_m: args.length_m,
            zoom: opt.zooms.first().copied().unwrap_or(zoom),
            ..Cut::default()
        };
        match section::render(&scan, &cut) {
            Some(svg) => match std::fs::write(path, svg) {
                Ok(()) => {
                    eprintln!("section written to {path}");
                    return ExitCode::SUCCESS;
                }
                Err(e) => {
                    eprintln!("cannot write {path}: {e}");
                    return ExitCode::from(2);
                }
            },
            None => {
                eprintln!("nothing to draw at {lon},{lat} z{} — outside the archive?", cut.zoom);
                return ExitCode::from(2);
            }
        }
    }

    let mut card = checks::run(&scan, &opt);
    card.archive = args.archive.display().to_string();
    if let Some(path) = &args.model {
        match read_model(path) {
            Ok(mut metrics) => card.metrics.append(&mut metrics),
            Err(e) => {
                eprintln!("cannot read model scorecard {}: {e}", path.display());
                return ExitCode::from(2);
            }
        }
    }

    print!("{}", report::table(&card));

    if let Some(path) = &args.json {
        let text = serde_json::to_string_pretty(&card.to_json()).unwrap_or_default();
        if path == "-" {
            println!("{text}");
        } else if let Err(e) = std::fs::write(path, text) {
            eprintln!("cannot write {path}: {e}");
            return ExitCode::from(2);
        }
    }

    if let Some(path) = &args.baseline {
        let Ok(text) = std::fs::read_to_string(path) else {
            eprintln!("cannot read baseline {}", path.display());
            return ExitCode::from(2);
        };
        let Ok(base) = serde_json::from_str(&text) else {
            eprintln!("baseline {} is not JSON", path.display());
            return ExitCode::from(2);
        };
        let changes = card.diff(&base);
        println!("\n{}", report::diff_table(&changes));
        let regressed: Vec<&str> =
            changes.iter().filter(|c| c.verdict == Move::Regressed).map(|c| c.id.as_str()).collect();
        if !regressed.is_empty() {
            eprintln!("regressed: {}", regressed.join(", "));
            return ExitCode::FAILURE;
        }
    }

    ExitCode::SUCCESS
}

/// The corpus lives beside the crate, not beside the archive: archives are
/// build products in a data directory, the corpus is source.
fn default_corpus(_archive: &std::path::Path) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("verify/scenarios.json")
}

/// Reads a model-side scorecard back into [`Metric`]s.
///
/// Only what a scorecard diff needs survives the round trip — the distribution
/// itself does not, because a perturbation experiment's "distribution" is a
/// count of features that moved and the diff joins on `worst` and
/// `violation_pct`, both of which are carried explicitly.
fn read_model(path: &std::path::Path) -> Result<Vec<arpentry_server::verify::Metric>, String> {
    use arpentry_server::verify::{dist::Dist, Invariant, Metric, Sense};
    let text = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
    let json: serde_json::Value = serde_json::from_str(&text).map_err(|e| e.to_string())?;
    let list = json.get("metrics").and_then(|m| m.as_array()).ok_or("no `metrics` array")?;
    let mut out = Vec::new();
    for m in list {
        let get = |k: &str| m.get(k);
        let id = get("id").and_then(|v| v.as_str()).ok_or("a metric has no id")?.to_string();
        let invariant = match get("invariant").and_then(|v| v.as_str()).unwrap_or("I1") {
            "I2" => Invariant::I2,
            "I3" => Invariant::I3,
            "I4" => Invariant::I4,
            "I5" => Invariant::I5,
            "I6" => Invariant::I6,
            "I7" => Invariant::I7,
            "I8" => Invariant::I8,
            _ => Invariant::I1,
        };
        let mut dist = Dist::metres();
        // Re-materialize just enough of the distribution that `samples`,
        // `violations` and `worst` print and diff as they did at write time.
        let samples = get("samples").and_then(|v| v.as_u64()).unwrap_or(0);
        let violations = get("violations").and_then(|v| v.as_u64()).unwrap_or(0);
        let worst = get("worst").and_then(|v| v.as_f64()).unwrap_or(0.0);
        for _ in 0..violations.min(samples) {
            dist.push(worst);
        }
        for _ in violations..samples {
            dist.push(0.0);
        }
        out.push(Metric {
            id,
            invariant,
            title: get("title").and_then(|v| v.as_str()).unwrap_or_default().to_string(),
            population: get("population").and_then(|v| v.as_str()).unwrap_or_default().to_string(),
            detail: String::new(),
            sense: Sense::HigherIsWorse,
            threshold: get("threshold").and_then(|v| v.as_f64()).unwrap_or(0.0),
            dist,
            worst: Vec::new(),
            skipped: get("skipped").and_then(|v| v.as_str()).map(str::to_string),
        });
    }
    Ok(out)
}

fn parse() -> Result<Args, String> {
    let mut a = Args {
        archive: PathBuf::new(),
        opt: Options::default(),
        baseline: None,
        model: None,
        json: None,
        corpus_path: None,
        scenario: None,
        mine: false,
        list: false,
        section: None,
        bearing: 90.0,
        length_m: 200.0,
    };
    let mut it = std::env::args().skip(1);
    let mut seen_archive = false;
    while let Some(arg) = it.next() {
        let mut value = || it.next().ok_or_else(|| format!("{arg} needs a value"));
        match arg.as_str() {
            "--help" | "-h" => {
                println!("{USAGE}");
                std::process::exit(0);
            }
            "--list" => a.list = true,
            "--mine" => a.mine = true,
            "--zoom" => {
                a.opt.zooms = value()?
                    .split(',')
                    .map(|s| s.trim().parse::<u8>().map_err(|e| format!("--zoom: {e}")))
                    .collect::<Result<_, _>>()?;
            }
            "--at" => {
                let v = value()?;
                let (lon, lat) = v.split_once(',').ok_or("--at wants lon,lat")?;
                a.opt.at = Some((
                    lon.trim().parse().map_err(|e| format!("--at lon: {e}"))?,
                    lat.trim().parse().map_err(|e| format!("--at lat: {e}"))?,
                ));
            }
            "--scenario" => a.scenario = Some(value()?),
            "--corpus" => a.corpus_path = Some(PathBuf::from(value()?)),
            "--spacing" => {
                a.opt.spacing_m = value()?.parse().map_err(|e| format!("--spacing: {e}"))?
            }
            "--worst" => a.opt.worst_k = value()?.parse().map_err(|e| format!("--worst: {e}"))?,
            "--max-tiles" => {
                a.opt.max_tiles = value()?.parse().map_err(|e| format!("--max-tiles: {e}"))?
            }
            "--section" => a.section = Some(value()?),
            "--bearing" => a.bearing = value()?.parse().map_err(|e| format!("--bearing: {e}"))?,
            "--length" => a.length_m = value()?.parse().map_err(|e| format!("--length: {e}"))?,
            "--baseline" => a.baseline = Some(PathBuf::from(value()?)),
            "--model" => a.model = Some(PathBuf::from(value()?)),
            "--json" => a.json = Some(value()?),
            other if other.starts_with("--") => return Err(format!("unknown option {other}")),
            other => {
                a.archive = PathBuf::from(other);
                seen_archive = true;
            }
        }
    }
    if !seen_archive && !a.list {
        return Err("an archive path is required".into());
    }
    Ok(a)
}
