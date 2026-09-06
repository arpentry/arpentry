//! `arpentry_world`: build the world for one bbox and write it as glTF.
//!
//! ```text
//! arpentry_world --bbox w,s,e,n --zone DIR --output FILE.glb
//!                [--terrain PATH|SPEC] [--segments PATH]
//!                [--spacing M] [--max-vertices N] [--until STEP]
//! ```
//!
//! Runs the steps in order, prints one line per step, stops after `--until`,
//! and writes what was built. The bbox is required and never inferred from the
//! data: a cut zone holds the zone plus a margin.

use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Instant;

use arpentry_server::dem::{Dem, Field};
use arpentry_server::project::Bounds;
use arpentry_world::step::Step;
use arpentry_world::world::World;
use arpentry_world::{drape, gltf, terrain};

struct Args {
    bbox: Bounds,
    terrain: PathBuf,
    segments: PathBuf,
    output: PathBuf,
    spacing: f64,
    max_vertices: usize,
    until: Step,
}

const USAGE: &str = "usage: arpentry_world --bbox w,s,e,n --zone DIR --output FILE.glb
       [--terrain PATH|SPEC] [--segments PATH] [--spacing M] [--max-vertices N] [--until STEP]

  --bbox          the world's bounds in degrees (required; never inferred from the data)
  --zone DIR      a cut zone: DIR/terrain.pmtiles and DIR/segment.parquet
  --terrain       a Mapterhorn PMTiles archive, or a synthetic ground:
                  flat[?h=400], ramp?grade=0.03[&bearing=90][&radius=400],
                  hill?amp=60&radius=400, step?rise=3[&width=0] (overrides --zone)
  --segments      an Overture segment.parquet (overrides --zone)
  --spacing M     terrain lattice spacing in metres (default 2)
  --max-vertices  cap on terrain vertices; the spacing grows to fit (default 2000000)
  --until STEP    stop after this step: terrain | drape (default drape)
  --output FILE   the .glb to write (required)";

fn main() -> ExitCode {
    let args = match parse_args(std::env::args().skip(1)) {
        Ok(args) => args,
        Err(e) => {
            eprintln!("error: {e}\n\n{USAGE}");
            return ExitCode::from(2);
        }
    };
    match run(&args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run(args: &Args) -> Result<(), String> {
    let mut dem = Dem::open(&args.terrain).map_err(|e| format!("{}: {e}", args.terrain.display()))?;
    let mut world = World::new(args.bbox);
    println!("bbox     {:?} -> {:.0} x {:.0} m", args.bbox, world.rect.width(), world.rect.height());
    for step in Step::ALL {
        let t = Instant::now();
        let summary = match step {
            Step::Terrain => terrain::run(&mut world, &mut dem, args.spacing, args.max_vertices),
            Step::Drape => drape::run(&mut world, &args.segments)
                .map_err(|e| format!("{}: {e}", args.segments.display()))?,
        };
        println!("{:<8} {}  {:.2}s", step.name(), summary, t.elapsed().as_secs_f64());
        if step == args.until {
            break;
        }
    }
    let t = Instant::now();
    let glb = gltf::write_glb(&world);
    std::fs::write(&args.output, &glb).map_err(|e| format!("{}: {e}", args.output.display()))?;
    println!("gltf     bytes={}  {:.2}s  {}", glb.len(), t.elapsed().as_secs_f64(), args.output.display());
    Ok(())
}

fn parse_args(mut it: impl Iterator<Item = String>) -> Result<Args, String> {
    let mut bbox = None;
    let mut zone: Option<PathBuf> = None;
    let mut terrain: Option<PathBuf> = None;
    let mut segments: Option<PathBuf> = None;
    let mut output = None;
    let mut spacing = 2.0;
    let mut max_vertices = 2_000_000;
    let mut until = Step::Drape;
    while let Some(flag) = it.next() {
        match flag.as_str() {
            "--bbox" => bbox = Some(parse_bbox(&value(&mut it, "--bbox")?)?),
            "--zone" => zone = Some(PathBuf::from(value(&mut it, "--zone")?)),
            "--terrain" => terrain = Some(PathBuf::from(value(&mut it, "--terrain")?)),
            "--segments" => segments = Some(PathBuf::from(value(&mut it, "--segments")?)),
            "--output" => output = Some(PathBuf::from(value(&mut it, "--output")?)),
            "--spacing" => spacing = parse_num(&value(&mut it, "--spacing")?, "--spacing")?,
            "--max-vertices" => {
                max_vertices = parse_num(&value(&mut it, "--max-vertices")?, "--max-vertices")?
            }
            "--until" => until = value(&mut it, "--until")?.parse()?,
            "-h" | "--help" => return Err("help".into()),
            other => return Err(format!("unknown flag {other}")),
        }
    }
    let bbox = bbox.ok_or("--bbox is required")?;
    let output = output.ok_or("--output is required")?;
    let from_zone = |name: &str| zone.as_ref().map(|z| z.join(name));
    let terrain = terrain
        .or_else(|| from_zone("terrain.pmtiles"))
        .ok_or("--terrain or --zone is required")?;
    let segments = segments
        .or_else(|| from_zone("segment.parquet"))
        .ok_or("--segments or --zone is required")?;
    let terrain = resolve_terrain(terrain, &bbox)?;
    Ok(Args { bbox, terrain, segments, output, spacing, max_vertices, until })
}

/// A synthetic terrain spec without an origin takes the bbox centre, and the
/// resolved spec is what flows: the run is reproducible from what it prints.
/// (The tiler's `resolve_terrain`, kept in step.)
fn resolve_terrain(terrain: PathBuf, bbox: &Bounds) -> Result<PathBuf, String> {
    let Some(spec) = terrain.to_str().filter(|s| Field::is_spec(s)) else {
        return Ok(terrain);
    };
    let field = Field::parse(spec).map_err(|e| format!("invalid --terrain: {e}"))?;
    if field.has_origin() {
        return Ok(terrain);
    }
    let centre = field.at((bbox.west + bbox.east) / 2.0, (bbox.south + bbox.north) / 2.0);
    let resolved = centre.spec();
    println!("terrain  {resolved}");
    Ok(PathBuf::from(resolved))
}

fn parse_bbox(s: &str) -> Result<Bounds, String> {
    let v: Vec<f64> = s
        .split(',')
        .map(|p| p.trim().parse::<f64>().map_err(|_| format!("invalid --bbox: {s}")))
        .collect::<Result<_, _>>()?;
    let [west, south, east, north] = v[..] else {
        return Err(format!("--bbox must be w,s,e,n, got {s}"));
    };
    if !(west < east && south < north) {
        return Err(format!("--bbox is empty: {s}"));
    }
    Ok(Bounds { west, south, east, north })
}

fn value(it: &mut impl Iterator<Item = String>, flag: &str) -> Result<String, String> {
    it.next().ok_or_else(|| format!("{flag} requires a value"))
}

fn parse_num<T: std::str::FromStr>(s: &str, flag: &str) -> Result<T, String> {
    s.parse().map_err(|_| format!("invalid value for {flag}: {s}"))
}

