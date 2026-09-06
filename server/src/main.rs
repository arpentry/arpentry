//! Arpentry tiler CLI (TILER.md §4).
//!
//! Hand-rolled argument parsing (no clap) to keep the dependency set minimal.

use std::path::PathBuf;

use arpentry_server::dem::Field;
use arpentry_server::layers;
use arpentry_server::pipeline::{self, Config};
use arpentry_server::project::Bounds;
use arpentry_server::terrain_cdt;

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
  --terrain <path|spec>
                       Terrarium DEM PMTiles (e.g. Mapterhorn planet.pmtiles),
                       giving each tile real elevation instead of a flat mesh;
                       or an analytic ground, the isolation harness's terrain
                       dial, held fixed while only the features vary:
                         flat[?h=400]
                         ramp?grade=0.03[&bearing=90][&h=][&at=lon,lat]
                         hill?amp=60&radius=400[&h=][&at=]
                         step?rise=3[&width=0][&bearing=90][&h=][&at=]
                       `bearing` is compass degrees (0=N, 90=E) and names the
                       direction the ground rises in; `at` is the origin, and
                       defaults to the centre of --bbox
  --threads <n>        Worker threads (default: CPU count)
  --brotli <q>         Brotli quality 0-11 for tile blobs (default: 7)
  --stage-out <path>   Write the stages this run computed to a snapshot, so a
                       later run can skip them
  --stage-in <path>    Reuse the stages in a snapshot instead of computing
                       them. Refused when the bbox, the ground or an input has
                       moved; a rebuilt tiler only warns, since it cannot tell
                       which stage the rebuild touched
  --no-breaklines      Plain lattice terrain: no bench contact lines, and no
                       hole (there is no constrained mesh to cut)
  --no-hole            Draw ground under the asphalt again, so an A/B re-tile
                       of the hole is a flag rather than a patch
  --plan-lines         Also draw the plan-space network the surfaces were
                       built from: each source segment's centreline
                       (`plan_axis_*`) and the edge the model says it paved
                       (`plan_edge_*`), per surface family. The network view —
                       what a tile normally deletes, because a line over its
                       own asphalt is a second coat of the same paint. Style
                       it with style-network.json
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
            // Which ground this scorecard was measured over. A terrain dial
            // makes the archive's provenance a variable, and a number whose
            // ground nobody can name is the thing this harness exists to stop
            // producing.
            eprintln!(
                "ground  {:>8}  {}",
                "",
                match cfg.terrain.as_deref() {
                    Some(t) => t.display().to_string(),
                    None => "flat mesh (no --terrain)".to_string(),
                }
            );
            // An archive carrying the plan lines is a debugging archive: its
            // transportation layer holds three features per source segment
            // that no map style draws. Say so, because nothing downstream
            // will: it is safe to *score* (`verify::scene` drops every
            // `plan_*` class, so it produces the same scorecard as the same
            // cut without them) and the only way to tell it from a map is to
            // have been told.
            if cfg.plan_lines {
                eprintln!("plan    {:>8}  --plan-lines: network view archive, not a map", "");
            }
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
    // The whole run, not `phase1 + phase2`. The model stage runs before phase 1
    // and the sum left it out entirely: on the day it was first measured that
    // was 10.8 s named as the total of a 25.5 s run, and 5.8 s of 27.2 s at
    // `--min-zoom 13`, with `features/s` and `tiles/s` overstated by the same
    // factor because they divide by it.
    let total = t.wall;
    eprintln!(
        "inputs: {}/{} row groups after bbox pruning, {} worker thread{}",
        stats.row_groups_read,
        stats.row_groups_total,
        stats.threads,
        if stats.threads == 1 { "" } else { "s" },
    );
    eprintln!(
        "model   {:>8}  {} corridors, {} profiles, {} crossings, {} earthwork edges, {} water bodies, {} intersections",
        secs(t.model),
        stats.corridors,
        stats.profiles,
        stats.crossings,
        stats.earthworks,
        stats.water,
        stats.intersections,
    );
    eprintln!(
        "  of which        assemble {}, solve {}, pavement {}, other {}",
        secs(t.assemble),
        secs(t.solve),
        secs(t.pavement),
        secs(t.ground),
    );
    eprintln!(
        "  other is        seniors {}, walk bands {}, crossings {}, walk fit {}, walk graph {}",
        secs(t.seniors),
        secs(t.walk_bands),
        secs(t.crossings_synth),
        secs(t.walk_fit),
        secs(t.walkgraph),
    );
    eprintln!(
        "crests            {} segments, {} nodes pulled in by a contending bench, {} dropped",
        stats.crest_segments, stats.crests_pulled, stats.crests_dropped,
    );
    eprintln!(
        "sidewalks         {} ways attached to a street over {:.1} km of it",
        stats.walks,
        stats.walk_host_m / 1000.0,
    );
    eprintln!(
        "pavement {:>7}  {} chunks, {:.0} m2 paved (inside the model stage above)",
        secs(t.pavement),
        stats.pave_chunks,
        stats.pave_area_m2,
    );
    eprintln!(
        "consistency       junction step max {:.2} m (p99 {:.2} m, {} over 0.5 m), clearance shortfall max {:.2} m, {} demands dropped (worst {:.1} m)",
        stats.max_junction_step_m,
        stats.p99_junction_step_m,
        stats.junction_steps_over,
        stats.max_clearance_violation_m,
        stats.clearance_demands_dropped,
        stats.worst_dropped_demand_m,
    );
    if stats.relax_residuals.iter().any(|(name, _)| !name.is_empty()) {
        eprintln!(
            "residuals         {}",
            stats
                .relax_residuals
                .iter()
                .filter(|(name, _)| !name.is_empty())
                .map(|(name, max)| format!("{name} {max:.3}"))
                .collect::<Vec<_>>()
                .join("  ")
        );
    }
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
    // What the three blocks above account for, and what they do not: the
    // The probes all run between the model
    // stage and phase 1, inside a wall clock and outside every sum. Printed on
    // every run, zero included, for the reason the `cdt` line below is — a
    // remainder that appeared only when it was large would make its own
    // absence unreadable.
    eprintln!(
        "  of which        model {}, phase 1 {}, phase 2 {}, outside them {}",
        secs(t.model),
        secs(t.phase1),
        secs(t.phase2),
        secs(total.saturating_sub(t.model + t.phase1 + t.phase2)),
    );
    eprintln!(
        "dem     {:>8}  {} tile decodes",
        "",
        arpentry_server::dem::DECODES.load(std::sync::atomic::Ordering::Relaxed),
    );
    // The tiles the geometric kernel refused. Always printed, zero included:
    // a fallback is silent everywhere else — the archive checks read a plain
    // lattice as a plain lattice — so a line that appears only on failure
    // would make its own absence unreadable.
    let breaklines = terrain_cdt::BREAKLINES_LOST.load(std::sync::atomic::Ordering::Relaxed);
    let one_mesh = terrain_cdt::ONE_MESH_LOST.load(std::sync::atomic::Ordering::Relaxed);
    eprintln!(
        "cdt     {:>8}  {} tiles lost their breaklines, {} fell back from the one mesh{}",
        "",
        breaklines,
        one_mesh,
        if breaklines + one_mesh > 0 {
            format!(
                " — {} of {} tiles built by a different construction than their neighbours",
                breaklines + one_mesh,
                stats.tiles_written,
            )
        } else {
            String::new()
        },
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
    let mut stage_out: Option<PathBuf> = None;
    let mut stage_in: Option<PathBuf> = None;
    let mut breaklines = true;
    let mut hole = true;
    let mut plan_lines = false;

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
            "--stage-out" => stage_out = Some(PathBuf::from(value(&mut it, "--stage-out")?)),
            "--stage-in" => stage_in = Some(PathBuf::from(value(&mut it, "--stage-in")?)),
            "--no-breaklines" => breaklines = false,
            "--no-hole" => hole = false,
            "--plan-lines" => plan_lines = true,
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
    let terrain = match terrain {
        Some(t) => Some(resolve_terrain(t, &bbox)?),
        None => None,
    };
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
        stage_out,
        stage_in,
        breaklines,
        hole: hole && breaklines,
        plan_lines,
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

/// Settles a `--terrain` value before the run sees it: a DEM path passes
/// through untouched, and a terrain spec is parsed here — so a typo is a usage
/// error rather than an archive that fails to open in fifteen places — and
/// given the centre of `--bbox` as its origin when it named none.
///
/// **The resolved spec is what flows.** Every `Dem::open` downstream, and the
/// `.model.json` the scorecard reads, then carry a self-contained string: the
/// run is reproducible from what it recorded, not from the bbox it happened to
/// be given alongside.
fn resolve_terrain(terrain: PathBuf, bbox: &Bounds) -> Result<PathBuf, String> {
    let Some(spec) = terrain.to_str().filter(|s| Field::is_spec(s)) else {
        return Ok(terrain);
    };
    let field = Field::parse(spec).map_err(|e| format!("invalid --terrain: {e}"))?;
    if field.has_origin() {
        return Ok(terrain);
    }
    if *bbox == Bounds::WORLD {
        return Err("--terrain needs `at=lon,lat`, or a --bbox to take its origin from".to_string());
    }
    let centre = field.at((bbox.west + bbox.east) / 2.0, (bbox.south + bbox.north) / 2.0);
    Ok(PathBuf::from(centre.spec()))
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

#[cfg(test)]
mod tests {
    use super::*;

    fn bbox() -> Bounds {
        Bounds { west: 6.9075, south: 46.4365, east: 6.9110, north: 46.4385 }
    }

    #[test]
    fn a_dem_path_passes_through_untouched() {
        let path = PathBuf::from("data/zones/montreux/terrain.pmtiles");
        assert_eq!(resolve_terrain(path.clone(), &bbox()).unwrap(), path);
    }

    #[test]
    fn a_spec_without_an_origin_takes_the_bbox_centre() {
        let out = resolve_terrain(PathBuf::from("ramp?grade=0.03"), &bbox()).unwrap();
        let spec = out.to_str().unwrap();
        assert!(spec.contains("at=6.909250,46.437500"), "{spec}");
        // …and what it wrote back is a spec the pipeline can read.
        assert!(Field::parse(spec).is_ok(), "{spec}");
    }

    #[test]
    fn an_explicit_origin_is_left_alone() {
        let given = PathBuf::from("hill?amp=60&radius=400&at=6.9091,46.4374");
        assert_eq!(resolve_terrain(given.clone(), &bbox()).unwrap(), given);
    }

    #[test]
    fn a_mistyped_spec_fails_the_usage_check() {
        // Not "a missing file" fifteen `Dem::open`s later, where every caller
        // `.ok()`s it into a silent sea-level ground.
        assert!(resolve_terrain(PathBuf::from("ramp?grade=steep"), &bbox()).is_err());
        assert!(resolve_terrain(PathBuf::from("hill?amp=60"), &bbox()).is_err());
    }

    #[test]
    fn a_spec_without_a_bbox_has_nowhere_to_stand() {
        assert!(resolve_terrain(PathBuf::from("ramp?grade=0.03"), &Bounds::WORLD).is_err());
    }
}
