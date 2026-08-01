//! `arpentry_verify` — measures an emitted archive against the invariants.
//!
//! ```sh
//! # What is the state of the scene?
//! arpentry_verify data/overture-ch/preview.arpa
//!
//! # Did what I just did make it better?
//! arpentry_verify preview.arpa --baseline verify/baseline-montreux.json
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
  --baseline <path>   Diff against a committed scorecard; exit 1 on regression
  --json <path>       Write this run's scorecard as JSON (\"-\" for stdout)
  --mine              Propose corpus sites from this archive and exit
  --list              List the canonical situations and exit
";

struct Args {
    archive: PathBuf,
    opt: Options,
    baseline: Option<PathBuf>,
    json: Option<String>,
    corpus_path: Option<PathBuf>,
    scenario: Option<String>,
    mine: bool,
    list: bool,
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
                "{:<4} {:<42} {}{}",
                s.id,
                s.name,
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

    let mut card = checks::run(&scan, &opt);
    card.archive = args.archive.display().to_string();

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

fn parse() -> Result<Args, String> {
    let mut a = Args {
        archive: PathBuf::new(),
        opt: Options::default(),
        baseline: None,
        json: None,
        corpus_path: None,
        scenario: None,
        mine: false,
        list: false,
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
            "--baseline" => a.baseline = Some(PathBuf::from(value()?)),
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
