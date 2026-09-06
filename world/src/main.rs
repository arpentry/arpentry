//! `arpentry_world`: build the world for one bbox and write it as glTF.
//!
//! ```text
//! arpentry_world --bbox w,s,e,n --zone DIR [--output FILE.glb] [--svg FILE.svg]
//!                [--terrain PATH|SPEC] [--segments PATH] [--view x0,y0,x1,y1]
//!                [--spacing M] [--max-vertices N] [--until STEP]
//! ```
//!
//! Runs the steps in order, prints one line per step, stops after `--until`,
//! and writes what was built: the 3D world as glTF, the plan as SVG, or both.
//! The bbox is required and never inferred from the data: a cut zone holds
//! the zone plus a margin.

use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Instant;

use arpentry_server::dem::{Dem, Field};
use arpentry_server::project::Bounds;
use arpentry_world::step::Step;
use arpentry_world::world::World;
use arpentry_world::frame::Rect;
use arpentry_world::net;
use arpentry_world::{drape, fillet, gltf, kerb, ribbon, surface, svg, terrain};

struct Args {
    bbox: Bounds,
    terrain: PathBuf,
    segments: PathBuf,
    output: Option<PathBuf>,
    svg: Option<PathBuf>,
    view: Option<Rect>,
    spacing: f64,
    max_vertices: usize,
    until: Step,
}

const USAGE: &str = "usage: arpentry_world --bbox w,s,e,n --zone DIR [--output FILE.glb] [--svg FILE.svg]
       [--terrain PATH|SPEC] [--segments PATH] [--view x0,y0,x1,y1]
       [--spacing M] [--max-vertices N] [--until STEP]

  --bbox          the world's bounds in degrees (required; never inferred from the data)
  --zone DIR      a cut zone: DIR/terrain.pmtiles and DIR/segment.parquet
  --terrain       a Mapterhorn PMTiles archive, or a synthetic ground:
                  flat[?h=400], ramp?grade=0.03[&bearing=90][&radius=400],
                  hill?amp=60&radius=400, step?rise=3[&width=0] (overrides --zone)
  --segments      an Overture segment.parquet, or a synthetic network:
                  net:straight|tee|cross|hairpin?angle=20|dual?gap=4|
                  sidewalk?d=6|corner?d=5|crossing?d=6 [&len=200&class=residential]
                  (overrides --zone)
  --spacing M     terrain lattice spacing in metres (default 2)
  --max-vertices  cap on terrain vertices; the spacing grows to fit (default 2000000)
  --until STEP    stop after this step: terrain | drape | ribbon | surface | kerb | fillet
                  (default fillet)
  --output FILE   the .glb to write
  --svg FILE      the plan view to write, one SVG group per step
  --view x0,y0,x1,y1  the window the plan shows, in local metres (default: the bbox)
At least one of --output and --svg is required.";

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
            Step::Ribbon => ribbon::run(&mut world),
            Step::Surface => surface::run(&mut world),
            Step::Kerb => kerb::run(&mut world),
            Step::Fillet => fillet::run(&mut world),
        };
        println!("{:<8} {}  {:.2}s", step.name(), summary, t.elapsed().as_secs_f64());
        if step == args.until {
            break;
        }
    }
    if let Some(output) = &args.output {
        let t = Instant::now();
        let glb = gltf::write_glb(&world);
        std::fs::write(output, &glb).map_err(|e| format!("{}: {e}", output.display()))?;
        println!("gltf     bytes={}  {:.2}s  {}", glb.len(), t.elapsed().as_secs_f64(), output.display());
    }
    if let Some(path) = &args.svg {
        let t = Instant::now();
        let text = svg::write_svg(&world, args.view);
        std::fs::write(path, &text).map_err(|e| format!("{}: {e}", path.display()))?;
        println!("svg      bytes={}  {:.2}s  {}", text.len(), t.elapsed().as_secs_f64(), path.display());
    }
    Ok(())
}

fn parse_args(mut it: impl Iterator<Item = String>) -> Result<Args, String> {
    let mut bbox = None;
    let mut zone: Option<PathBuf> = None;
    let mut terrain: Option<PathBuf> = None;
    let mut segments: Option<PathBuf> = None;
    let mut output = None;
    let mut svg = None;
    let mut view = None;
    let mut spacing = 2.0;
    let mut max_vertices = 2_000_000;
    let mut until = Step::Fillet;
    while let Some(flag) = it.next() {
        match flag.as_str() {
            "--bbox" => bbox = Some(parse_bbox(&value(&mut it, "--bbox")?)?),
            "--zone" => zone = Some(PathBuf::from(value(&mut it, "--zone")?)),
            "--terrain" => terrain = Some(PathBuf::from(value(&mut it, "--terrain")?)),
            "--segments" => segments = Some(PathBuf::from(value(&mut it, "--segments")?)),
            "--output" => output = Some(PathBuf::from(value(&mut it, "--output")?)),
            "--svg" => svg = Some(PathBuf::from(value(&mut it, "--svg")?)),
            "--view" => view = Some(parse_view(&value(&mut it, "--view")?)?),
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
    if output.is_none() && svg.is_none() {
        return Err("one of --output and --svg is required".into());
    }
    let from_zone = |name: &str| zone.as_ref().map(|z| z.join(name));
    let terrain = terrain
        .or_else(|| from_zone("terrain.pmtiles"))
        .ok_or("--terrain or --zone is required")?;
    let segments = segments
        .or_else(|| from_zone("segment.parquet"))
        .ok_or("--segments or --zone is required")?;
    let terrain = resolve_terrain(terrain, &bbox)?;
    if let Some(spec) = segments.to_str().filter(|s| net::is_spec(s)) {
        net::parse(spec).map_err(|e| format!("invalid --segments: {e}"))?;
    }
    Ok(Args { bbox, terrain, segments, output, svg, view, spacing, max_vertices, until })
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

fn parse_view(s: &str) -> Result<Rect, String> {
    let v: Vec<f64> = s
        .split(',')
        .map(|p| p.trim().parse::<f64>().map_err(|_| format!("invalid --view: {s}")))
        .collect::<Result<_, _>>()?;
    let [x0, y0, x1, y1] = v[..] else {
        return Err(format!("--view must be x0,y0,x1,y1, got {s}"));
    };
    if !(x0 < x1 && y0 < y1) {
        return Err(format!("--view is empty: {s}"));
    }
    Ok(Rect { x0, y0, x1, y1 })
}

fn value(it: &mut impl Iterator<Item = String>, flag: &str) -> Result<String, String> {
    it.next().ok_or_else(|| format!("{flag} requires a value"))
}

fn parse_num<T: std::str::FromStr>(s: &str, flag: &str) -> Result<T, String> {
    s.parse().map_err(|_| format!("invalid value for {flag}: {s}"))
}

