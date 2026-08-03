//! Arpentry tiler CLI (TILER.md §4).
//!
//! Hand-rolled argument parsing (no clap) to keep the dependency set minimal.

use std::path::PathBuf;

use arpentry_server::layers;
use arpentry_server::pipeline::{self, Config};
use arpentry_server::project::Bounds;

const USAGE: &str = "\
arpentry_tiler — generate a .arpa tile archive from GeoParquet inputs

USAGE:
  arpentry_tiler --output <path> --input <N:path> [--input <N:path> ...] [options]

OPTIONS:
  --output <path>      Output .arpa archive path (required)
  --input <N:path>     GeoParquet input keyed by layer index N (repeatable)
  --bbox <w,s,e,n>     Geographic bounds in degrees (default: world)
  --min-zoom <z>       Minimum zoom level (default: 0)
  --max-zoom <z>       Maximum zoom level (default: 4)
  --tmp <dir>          Temp directory for external sort (default: system temp)
  --mem <bytes>        Memory budget for external sort (default: 64 MiB)
  --terrain <path>     Terrarium DEM PMTiles (e.g. Mapterhorn planet.pmtiles);
                       gives each tile real elevation instead of a flat mesh
  --threads <n>        Worker threads (default: CPU count)
  --brotli <q>         Brotli quality 0-11 for tile blobs (default: 7)
  --dump <dir>         Write stage-artifact GeoJSON dumps (scene graph,
                       solved profiles) for inspection in QGIS/kepler
  --no-breaklines      Plain lattice terrain: no bench contact lines, and no
                       hole (there is no constrained mesh to cut)
  --no-hole            Draw ground under the asphalt again, so an A/B re-tile
                       of the hole is a flag rather than a patch
  -h, --help           Show this help

Layer indices: 0=terrain 1=land_cover 2=bathymetry 3=water 4=land
               5=transportation 6=land_use 7=building 8=poi 9=boundary";

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.iter().any(|a| a == "-h" || a == "--help") {
        println!("{USAGE}");
        return;
    }
    let cfg = match parse(args) {
        Ok(cfg) => cfg,
        Err(e) => {
            eprintln!("error: {e}\n\n{USAGE}");
            std::process::exit(2);
        }
    };

    match pipeline::run(&cfg) {
        Ok(stats) => {
            eprintln!(
                "done: {} features read, {} records ({} sub-pixel dropped), {} tiles -> {}",
                stats.features_read,
                stats.records,
                stats.dropped_subpixel,
                stats.tiles_written,
                cfg.output.display()
            );
            report_timings(&stats);
        }
        Err(e) => {
            eprintln!("error: {e}");
            std::process::exit(1);
        }
    }
}

/// Prints the per-stage timing breakdown gathered by the pipeline.
fn report_timings(stats: &pipeline::Stats) {
    let t = &stats.timings;
    let total = t.phase1 + t.phase2;
    eprintln!(
        "inputs: {}/{} row groups after bbox pruning, {} worker thread{}",
        stats.row_groups_read,
        stats.row_groups_total,
        stats.threads,
        if stats.threads == 1 { "" } else { "s" },
    );
    eprintln!(
        "model   {:>8}  {} corridors, {} profiles, {} crossings, {} earthwork edges, {} water bodies, {} junction plates",
        secs(t.model),
        stats.corridors,
        stats.profiles,
        stats.crossings,
        stats.earthworks,
        stats.water,
        stats.junction_plates,
    );
    eprintln!(
        "crests            {} segments, {} nodes pulled in by a contending bench, {} dropped",
        stats.crest_segments, stats.crests_pulled, stats.crests_dropped,
    );
    eprintln!(
        "pavement {:>7}  {} chunks, {:.0} m2 paved",
        secs(t.pavement),
        stats.pave_chunks,
        stats.pave_area_m2,
    );
    eprintln!(
        "consistency       junction step max {:.2} m (p99 {:.2} m, {} over 0.5 m), clearance shortfall max {:.2} m",
        stats.max_junction_step_m,
        stats.p99_junction_step_m,
        stats.junction_steps_over,
        stats.max_clearance_violation_m,
    );
    eprintln!(
        "phase 1 {:>8}  cpu: read {}, simplify {}, clip {}, sort {}",
        secs(t.phase1),
        secs(t.read),
        secs(t.simplify),
        secs(t.clip),
        secs(t.sort),
    );
    eprintln!(
        "phase 2 {:>8}  merge {}, decode {}, terrain {}, encode {}, write {}",
        secs(t.phase2),
        secs(t.merge),
        secs(t.decode),
        secs(t.terrain),
        secs(t.encode),
        secs(t.write),
    );
    let total_s = total.as_secs_f64().max(f64::MIN_POSITIVE);
    eprintln!(
        "total   {:>8}  {:.0} features/s, {:.0} tiles/s, sort payload {}",
        secs(total),
        stats.features_read as f64 / total_s,
        stats.tiles_written as f64 / total_s,
        mib(stats.record_bytes),
    );
    eprintln!(
        "dem     {:>8}  {} tile decodes",
        "",
        arpentry_server::dem::DECODES.load(std::sync::atomic::Ordering::Relaxed),
    );
}

fn secs(d: std::time::Duration) -> String {
    format!("{:.1}s", d.as_secs_f64())
}

fn mib(bytes: u64) -> String {
    format!("{:.1} MiB", bytes as f64 / (1024.0 * 1024.0))
}

fn parse(args: Vec<String>) -> Result<Config, String> {
    let mut output: Option<PathBuf> = None;
    let mut inputs: Vec<(u8, PathBuf)> = Vec::new();
    let mut bbox = Bounds::WORLD;
    let mut min_zoom: u8 = 0;
    let mut max_zoom: u8 = 4;
    let mut tmp_dir = std::env::temp_dir();
    let mut mem_budget: usize = 64 * 1024 * 1024;
    let mut terrain: Option<PathBuf> = None;
    let mut threads: usize = 0;
    let mut brotli_quality: i32 = arpentry_server::tile_build::DEFAULT_QUALITY;
    let mut dump: Option<PathBuf> = None;
    let mut breaklines = true;
    let mut hole = true;

    let mut it = args.into_iter();
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--output" => output = Some(PathBuf::from(value(&mut it, "--output")?)),
            "--input" => inputs.push(parse_input(&value(&mut it, "--input")?)?),
            "--bbox" => bbox = parse_bbox(&value(&mut it, "--bbox")?)?,
            "--min-zoom" => min_zoom = parse_num(&value(&mut it, "--min-zoom")?, "--min-zoom")?,
            "--max-zoom" => max_zoom = parse_num(&value(&mut it, "--max-zoom")?, "--max-zoom")?,
            "--tmp" => tmp_dir = PathBuf::from(value(&mut it, "--tmp")?),
            "--mem" => mem_budget = parse_num(&value(&mut it, "--mem")?, "--mem")?,
            "--terrain" => terrain = Some(PathBuf::from(value(&mut it, "--terrain")?)),
            "--threads" => threads = parse_num(&value(&mut it, "--threads")?, "--threads")?,
            "--brotli" => brotli_quality = parse_num(&value(&mut it, "--brotli")?, "--brotli")?,
            "--dump" => dump = Some(PathBuf::from(value(&mut it, "--dump")?)),
            "--no-breaklines" => breaklines = false,
            "--no-hole" => hole = false,
            other => return Err(format!("unknown argument: {other}")),
        }
    }

    let output = output.ok_or("--output is required")?;
    if inputs.is_empty() {
        return Err("at least one --input is required".to_string());
    }
    if min_zoom > max_zoom {
        return Err(format!("--min-zoom ({min_zoom}) exceeds --max-zoom ({max_zoom})"));
    }
    Ok(Config {
        output,
        inputs,
        bbox,
        min_zoom,
        max_zoom,
        tmp_dir,
        mem_budget,
        terrain,
        threads,
        brotli_quality,
        dump,
        breaklines,
        hole: hole && breaklines,
    })
}

fn value(it: &mut impl Iterator<Item = String>, flag: &str) -> Result<String, String> {
    it.next().ok_or_else(|| format!("{flag} requires a value"))
}

fn parse_num<T: std::str::FromStr>(s: &str, flag: &str) -> Result<T, String> {
    s.parse().map_err(|_| format!("invalid value for {flag}: {s}"))
}

/// Parses an `N:path` input. Splits at the first `:` so paths may contain more.
fn parse_input(s: &str) -> Result<(u8, PathBuf), String> {
    let (n, path) = s.split_once(':').ok_or_else(|| format!("--input must be N:path, got {s}"))?;
    let layer: u8 = n.parse().map_err(|_| format!("invalid layer index in --input: {n}"))?;
    if layer as usize >= layers::COUNT {
        return Err(format!("layer index {layer} out of range (0..{})", layers::COUNT));
    }
    if layer as usize == layers::TERRAIN as usize {
        return Err(format!(
            "layer {layer} (terrain) is synthesised by the tiler and cannot be a vector input"
        ));
    }
    Ok((layer, PathBuf::from(path)))
}

fn parse_bbox(s: &str) -> Result<Bounds, String> {
    let parts: Vec<f64> = s
        .split(',')
        .map(|p| p.trim().parse::<f64>())
        .collect::<Result<_, _>>()
        .map_err(|_| format!("invalid --bbox (want w,s,e,n): {s}"))?;
    if parts.len() != 4 {
        return Err(format!("--bbox needs 4 comma-separated values, got {}", parts.len()));
    }
    Ok(Bounds { west: parts[0], south: parts[1], east: parts[2], north: parts[3] })
}
