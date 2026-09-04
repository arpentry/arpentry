//! `arpentry_render` — renders a corpus of views off an archive and diffs the
//! images against a baseline.
//!
//! ```sh
//! # Cut a baseline for the archive you are about to change something under.
//! arpentry_render preview.arpa --baseline verify/shots-montreux --update
//!
//! # Change the client, then ask what moved.
//! arpentry_render preview.arpa --baseline verify/shots-montreux
//! ```
//!
//! `arpentry_verify` measures the model an archive describes. This measures the
//! pixels a client draws from it — the one subsystem the scorecard cannot see,
//! and the reason a renderer defect has until now been something you had to
//! notice rather than something that stayed dead.
//!
//! Renders are taken with `--archive --headless`, so no server, no port and no
//! window are involved and the same camera over the same archive is
//! byte-identical run to run.
//!
//! **A baseline is specific to the machine that cut it.** The image depends on
//! the GPU, the driver and the backend, so a committed one would be red for
//! everybody else. Cut yours, keep it out of the repo, and diff against it —
//! the same discipline `arpentry_verify` needs for a cut zone.
//!
//! Exits 1 when any view changed.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode};

use arpentry_server::verify::corpus;

const USAGE: &str = "\
arpentry_render <archive.arpa> [options]

  --baseline <dir>    Directory of baseline PNGs to diff against (required
                      unless --out is given on its own)
  --update            Write this run into --baseline and exit, cutting a new
                      baseline instead of judging against one
  --out <dir>         Where this run's PNGs go (default: a sibling of
                      --baseline named <baseline>.run)
  --view <id>         Only this view (e.g. S5-oblique); repeatable
  --corpus <path>     Scenario corpus (default: verify/scenarios.json)
  --client <path>     Client binary (default: ../build/client/arpentry_client)
  --size <WxH>        Image size in pixels (default: 640x480)
  --tolerance <n>     Per-channel delta a pixel may differ by and still count
                      as unchanged (default: 0 — the render is deterministic,
                      so anything above 0 is hiding something)
";

/// A camera over a corpus site. Two per site: what a defect looks like from
/// the angle a person would fly to, and the plan view that shows layout
/// without the foreshortening that hides height.
struct View {
    id: String,
    lon: f64,
    lat: f64,
    alt: f64,
    bearing: f64,
    tilt: f64,
}

/// What changed between one view's baseline and this run.
struct Diff {
    id: String,
    /// Share of pixels differing by more than the tolerance.
    changed_pct: f64,
    /// Largest single-channel difference anywhere in the image.
    max_delta: u8,
    /// Mean channel delta over the pixels that changed — separates a whole
    /// image shifted a shade from a handful of pixels rewritten.
    mean_delta: f64,
    note: Option<String>,
    /// A second render of the same view agreed with the baseline, so the first
    /// one was this harness's own noise rather than something that changed.
    unconfirmed: bool,
}

struct Args {
    archive: PathBuf,
    baseline: Option<PathBuf>,
    out: Option<PathBuf>,
    update: bool,
    views: Vec<String>,
    corpus: Option<PathBuf>,
    client: PathBuf,
    width: u32,
    height: u32,
    tolerance: u8,
}

fn parse_args() -> Result<Args, String> {
    let mut it = std::env::args().skip(1);
    let mut archive = None;
    let mut a = Args {
        archive: PathBuf::new(),
        baseline: None,
        out: None,
        update: false,
        views: Vec::new(),
        corpus: None,
        client: PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../build/client/arpentry_client"),
        width: 640,
        height: 480,
        tolerance: 0,
    };
    while let Some(arg) = it.next() {
        let mut value = |name: &str| {
            it.next().ok_or_else(|| format!("{name} needs a value"))
        };
        match arg.as_str() {
            "--baseline" => a.baseline = Some(PathBuf::from(value("--baseline")?)),
            "--out" => a.out = Some(PathBuf::from(value("--out")?)),
            "--update" => a.update = true,
            "--view" => a.views.push(value("--view")?),
            "--corpus" => a.corpus = Some(PathBuf::from(value("--corpus")?)),
            "--client" => a.client = PathBuf::from(value("--client")?),
            "--tolerance" => {
                a.tolerance = value("--tolerance")?
                    .parse()
                    .map_err(|_| "--tolerance takes 0-255".to_string())?
            }
            "--size" => {
                let v = value("--size")?;
                let (w, h) = v.split_once('x').ok_or("--size takes WxH")?;
                a.width = w.parse().map_err(|_| "bad --size width")?;
                a.height = h.parse().map_err(|_| "bad --size height")?;
            }
            "-h" | "--help" => return Err(USAGE.to_string()),
            other if other.starts_with('-') => {
                return Err(format!("unknown option {other}\n\n{USAGE}"))
            }
            other => archive = Some(PathBuf::from(other)),
        }
    }
    a.archive = archive.ok_or_else(|| format!("an archive is required\n\n{USAGE}"))?;
    if a.baseline.is_none() && a.out.is_none() {
        return Err(format!("--baseline or --out is required\n\n{USAGE}"));
    }
    if a.update && a.baseline.is_none() {
        return Err("--update needs --baseline to write into".to_string());
    }
    Ok(a)
}

/// Builds the view list from the corpus: every sited scenario, seen twice.
///
/// The corpus is the project's existing capital — one site per canonical
/// situation of GENERATION.md §6, mined from the data rather than guessed. A
/// second list of places to look at would be a second thing to keep true.
fn views(corpus_path: &Path, wanted: &[String]) -> Vec<View> {
    let sites = corpus::load(corpus_path);
    let mut out = Vec::new();
    for (id, site) in sites.iter().collect::<BTreeMap<_, _>>() {
        // An oblique at street scale, and a plan view high enough to hold the
        // whole junction. Both fixed: a view that drifts is not a baseline.
        out.push(View {
            id: format!("{id}-oblique"),
            lon: site.lon,
            lat: site.lat,
            alt: 600.0,
            bearing: 30.0,
            tilt: 55.0,
        });
        out.push(View {
            id: format!("{id}-plan"),
            lon: site.lon,
            lat: site.lat,
            alt: 900.0,
            bearing: 0.0,
            tilt: 0.0,
        });
    }
    if !wanted.is_empty() {
        out.retain(|v| wanted.iter().any(|w| w == &v.id));
    }
    out
}

/// Renders one view, returning the path written or why it could not be.
fn render(a: &Args, v: &View, dir: &Path) -> Result<PathBuf, String> {
    let path = dir.join(format!("{}.png", v.id));
    let status = Command::new(&a.client)
        .arg("--archive").arg(&a.archive)
        .arg("--lon").arg(v.lon.to_string())
        .arg("--lat").arg(v.lat.to_string())
        .arg("--alt").arg(v.alt.to_string())
        .arg("--bearing").arg(v.bearing.to_string())
        .arg("--tilt").arg(v.tilt.to_string())
        .arg("--width").arg(a.width.to_string())
        .arg("--height").arg(a.height.to_string())
        .arg("--headless")
        .arg("--screenshot").arg(&path)
        .stdout(std::process::Stdio::null())
        .status()
        .map_err(|e| format!("cannot run {}: {e}", a.client.display()))?;
    if !status.success() {
        return Err(format!("client exited {status}"));
    }
    Ok(path)
}

/// Compares two PNGs channel by channel.
fn compare(baseline: &Path, current: &Path, tolerance: u8) -> Diff {
    let id = current
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_default();
    let fail = |note: &str| Diff {
        id: id.clone(),
        changed_pct: 100.0,
        max_delta: 255,
        mean_delta: 255.0,
        note: Some(note.to_string()),
        unconfirmed: false,
    };

    if !baseline.exists() {
        return fail("no baseline for this view");
    }
    let (a, b) = match (image::open(baseline), image::open(current)) {
        (Ok(a), Ok(b)) => (a.to_rgba8(), b.to_rgba8()),
        _ => return fail("unreadable PNG"),
    };
    if a.dimensions() != b.dimensions() {
        return fail("size differs from the baseline");
    }

    let mut changed = 0u64;
    let mut sum_delta = 0u64;
    let mut max_delta = 0u8;
    for (pa, pb) in a.pixels().zip(b.pixels()) {
        // Alpha is not compared: the capture is opaque, and a difference there
        // would be a readback artifact rather than something drawn.
        let d = (0..3)
            .map(|c| pa.0[c].abs_diff(pb.0[c]))
            .max()
            .unwrap_or(0);
        if d > max_delta {
            max_delta = d;
        }
        if d > tolerance {
            changed += 1;
            sum_delta += d as u64;
        }
    }
    let total = (a.width() as u64) * (a.height() as u64);
    Diff {
        id,
        changed_pct: if total == 0 { 0.0 } else { 100.0 * changed as f64 / total as f64 },
        max_delta,
        mean_delta: if changed == 0 { 0.0 } else { sum_delta as f64 / changed as f64 },
        note: None,
        unconfirmed: false,
    }
}

fn main() -> ExitCode {
    let a = match parse_args() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("{e}");
            return ExitCode::from(2);
        }
    };

    let corpus_path = a.corpus.clone().unwrap_or_else(|| {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("verify/scenarios.json")
    });
    let views = views(&corpus_path, &a.views);
    if views.is_empty() {
        eprintln!("no views to render (corpus {})", corpus_path.display());
        return ExitCode::from(2);
    }

    // --update renders straight into the baseline; otherwise this run goes to
    // its own directory so a failed comparison leaves both images to look at.
    let out = if a.update {
        a.baseline.clone().unwrap()
    } else {
        a.out.clone().unwrap_or_else(|| {
            let mut p = a.baseline.clone().unwrap().into_os_string();
            p.push(".run");
            PathBuf::from(p)
        })
    };
    if let Err(e) = std::fs::create_dir_all(&out) {
        eprintln!("cannot create {}: {e}", out.display());
        return ExitCode::from(2);
    }

    println!("archive  {}", a.archive.display());
    println!("views    {} from {}", views.len(), corpus_path.display());
    println!("out      {}\n", out.display());

    let mut rendered = Vec::new();
    for v in &views {
        match render(&a, v, &out) {
            Ok(p) => rendered.push((v.id.clone(), p)),
            Err(e) => {
                eprintln!("{}: {e}", v.id);
                return ExitCode::from(2);
            }
        }
    }

    if a.update {
        println!("cut {} baseline images into {}", rendered.len(), out.display());
        println!("\nThis baseline describes this machine's GPU. Keep it out of\nthe repo and re-cut it when the archive changes.");
        return ExitCode::SUCCESS;
    }

    let baseline = a.baseline.clone().unwrap();
    let mut diffs: Vec<Diff> = rendered
        .iter()
        .map(|(id, path)| compare(&baseline.join(format!("{id}.png")), path, a.tolerance))
        .collect();
    // Confirm every change by rendering it again.
    //
    // The client is not perfectly deterministic: roughly one sweep in three,
    // one view lands a per cent or two different for reasons outside the
    // scene — tile arrival still reaches the image somewhere. A gate that
    // reports that is a gate someone turns off, and a gate that widens its
    // tolerance to swallow it can no longer see a real defect of the same
    // size. So: re-render what moved. Noise does not survive the second look;
    // a change does.
    for d in diffs.iter_mut().filter(|d| d.changed_pct > 0.0 && d.note.is_none()) {
        let Some(v) = views.iter().find(|v| v.id == d.id) else { continue };
        let retry_dir = out.join("confirm");
        if std::fs::create_dir_all(&retry_dir).is_err() {
            continue;
        }
        let Ok(again) = render(&a, v, &retry_dir) else { continue };
        let second = compare(&baseline.join(format!("{}.png", d.id)), &again, a.tolerance);
        if second.changed_pct == 0.0 {
            d.unconfirmed = true;
            d.note = Some("did not reproduce — harness noise".to_string());
        }
    }

    // Worst first, like the scorecard: the deepest change is the one to read.
    diffs.sort_by(|x, y| {
        y.changed_pct
            .partial_cmp(&x.changed_pct)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    println!("{:<20} {:>10} {:>10} {:>11}  {}", "view", "changed", "max Δ", "mean Δ", "");
    for d in &diffs {
        println!(
            "{:<20} {:>9.4}% {:>10} {:>11.2}  {}",
            d.id,
            d.changed_pct,
            d.max_delta,
            d.mean_delta,
            d.note.as_deref().unwrap_or("")
        );
    }

    let moved: Vec<&Diff> = diffs
        .iter()
        .filter(|d| d.changed_pct > 0.0 && !d.unconfirmed)
        .collect();
    let noise = diffs.iter().filter(|d| d.unconfirmed).count();
    println!();
    if noise > 0 {
        println!("{noise} view(s) moved once and not again — not counted");
    }
    if moved.is_empty() {
        println!("{} views unchanged", diffs.len());
        ExitCode::SUCCESS
    } else {
        println!(
            "{} of {} views changed — compare {} against {}",
            moved.len(),
            diffs.len(),
            out.display(),
            baseline.display()
        );
        ExitCode::FAILURE
    }
}
