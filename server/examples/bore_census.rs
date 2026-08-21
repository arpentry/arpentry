//! Where does the bore roof stand out of the ground, and who put it there?
//!
//! `clearance.bore_cover` scores the drawn tube against the drawn terrain and
//! cannot say which of the two moved, nor tell a portal from a wall. This walks
//! every reconciled tunnel span and decomposes each exposed metre of centerline:
//!
//! - **by cause** — the roof already stood above the *reference* surface the
//!   solve read (a portal transition by design, or a profile defect), or the
//!   engineered ground was cut below it afterwards, in which case the layer and
//!   the kind of edge that cut it are named (portal carve, bench, cutting face).
//! - **by owner** — whether the edge that took the cover belongs to the tube's
//!   own corridor (its daylighting approach, which is what a trench is for) or
//!   to somebody else, with the (cutter, victim) pairs listed.
//! - **against the counterfactual** — the whole stack refolded with every
//!   foreign edge floored at the tube's own cover line, which is what a
//!   "nobody may cut through a tunnel" rule would buy, and again with only
//!   foreign *faces* floored, since a bench is somebody's roadbed and raising
//!   the ground to a cover line would bury the road that rides it.
//! - **by position** — distance to the end of the span, and whether the exposed
//!   stretch reaches a run end (the mouth) or is closed off by covered tube.
//! - **by run** — what fraction of each span's tube the finished ground hides,
//!   which is what a majority rule would act on.
//!
//! Written for the S21 family (docs/GENERATION.md §6) and the reason
//! `synth::structure::drawn_runs` cuts on distance from the mouth rather than on
//! depth, connectivity or majority: measured here, connectivity keeps a gallery
//! that is bare end to end, and a majority test deletes the short cut-and-cover
//! spans that are legitimately mostly trench.
//!
//! Usage: cargo run --release --example bore_census -- <segment.parquet> <w,s,e,n> <terrain.pmtiles> [lon,lat] [corridor]

use arpentry_server::dem::Dem;
use arpentry_server::ground;
use arpentry_server::priors::{Stratum, TUNNEL_COVER_M, TUNNEL_HEIGHT_M};
use arpentry_server::project::Bounds;
use arpentry_server::scene::SpanKind;
use arpentry_server::{assemble, solve};

/// Arc step along a tunnel span, metres.
const STEP_M: f64 = 2.0;

/// What the earthworks of one layer do at a point: the covering bench target,
/// the highest fill face, the lowest cutting face, and the lowest carve notch —
/// `Earthworks::height`'s own decomposition, recomputed here so a sample can be
/// attributed to the *kind* of edge that produced it.
fn layer_parts(
    ew: &ground::modifiers::Earthworks,
    lon: f64,
    lat: f64,
    raw: f64,
) -> (Option<f64>, f64, f64, f64) {
    let (b, f, c, cv, _) = layer_parts_owned(ew, lon, lat, raw);
    (b, f, c, cv)
}

/// [`layer_parts`], plus the chain of the edge that produced the lowest of the
/// three cutting answers — the corridor whose earthwork took the cover away.
/// A tube cut by its *own* corridor's trench is the daylighting approach; one
/// cut by somebody else's is a neighbour digging through it.
fn layer_parts_owned(
    ew: &ground::modifiers::Earthworks,
    lon: f64,
    lat: f64,
    raw: f64,
) -> (Option<f64>, f64, f64, f64, Option<(u32, f64)>) {
    let cos_lat = lat.to_radians().cos();
    let (mut bench, mut fill, mut cut, mut carve) =
        (None::<(f64, f64)>, f64::NEG_INFINITY, f64::INFINITY, f64::INFINITY);
    // (chain, height) of the lowest cutting edge seen, whatever its kind.
    let mut lowest: Option<(u32, f64)> = None;
    for e in ew.edges() {
        let px = (lon - e.a.x) * 111_320.0 * cos_lat;
        let py = (lat - e.a.y) * 110_540.0;
        let ex = (e.b.x - e.a.x) * 111_320.0 * cos_lat;
        let ey = (e.b.y - e.a.y) * 110_540.0;
        let len2 = ex * ex + ey * ey;
        let raw_t = if len2 > 0.0 { (px * ex + py * ey) / len2 } else { 0.0 };
        if e.headwall && raw_t < 0.0 {
            continue; // bounded by its own face: no reach behind `a`
        }
        let t = raw_t.clamp(0.0, 1.0);
        let (qx, qy) = (px - t * ex, py - t * ey);
        let d = (qx * qx + qy * qy).sqrt();
        let side = if ex * py - ey * px >= 0.0 { 0 } else { 1 };
        if d >= e.half_width_m + e.batter_m[side] {
            continue;
        }
        let target = e.target_a + (e.target_b - e.target_a) * t;
        let rise = (d - e.half_width_m).max(0.0) / e.batter_run[side];
        let mut note = |h: f64| {
            if lowest.is_none_or(|(_, lh)| h < lh) {
                lowest = Some((e.chain, h));
            }
        };
        if e.carve {
            note(target + rise);
            carve = carve.min(target + rise);
            continue;
        }
        if d <= e.half_width_m {
            note(target);
            if bench.is_none_or(|(bd, _)| d < bd) {
                bench = Some((d, target));
            }
        } else if target > raw {
            fill = fill.max(target - rise);
        } else {
            note(target + rise);
            cut = cut.min(target + rise);
        }
    }
    (bench.map(|(_, t)| t), fill, cut, carve, lowest)
}

/// One layer's height at a point, reproducing [`Earthworks::height`] — bench
/// wins outright, else the cut digs and the fill raises, then carves bound the
/// result from above — with one optional change: every cutting answer from an
/// edge whose chain is **not** `own` is floored at `cover`, which is the
/// counterfactual "a foreign earthwork may not cut below this tube's cover
/// line". The floor never raises the ground above what came in (`base`), so a
/// tube whose cover has already gone at a mouth keeps its trench.
fn layer_height_guarded(
    ew: &ground::modifiers::Earthworks,
    lon: f64,
    lat: f64,
    base: f64,
    guard: Option<(u32, f64)>,
    benches_too: bool,
) -> f64 {
    let cos_lat = lat.to_radians().cos();
    let (mut bench, mut fill, mut cut, mut carve) =
        (None::<(f64, f64)>, f64::NEG_INFINITY, f64::INFINITY, f64::INFINITY);
    for e in ew.edges() {
        let px = (lon - e.a.x) * 111_320.0 * cos_lat;
        let py = (lat - e.a.y) * 110_540.0;
        let ex = (e.b.x - e.a.x) * 111_320.0 * cos_lat;
        let ey = (e.b.y - e.a.y) * 110_540.0;
        let len2 = ex * ex + ey * ey;
        let raw_t = if len2 > 0.0 { (px * ex + py * ey) / len2 } else { 0.0 };
        if e.headwall && raw_t < 0.0 {
            continue;
        }
        let t = raw_t.clamp(0.0, 1.0);
        let (qx, qy) = (px - t * ex, py - t * ey);
        let d = (qx * qx + qy * qy).sqrt();
        let side = if ex * py - ey * px >= 0.0 { 0 } else { 1 };
        if d >= e.half_width_m + e.batter_m[side] {
            continue;
        }
        let target = e.target_a + (e.target_b - e.target_a) * t;
        let rise = (d - e.half_width_m).max(0.0) / e.batter_run[side];
        // The guard: a foreign edge's cutting answer may not go below the
        // cover line, and the cover line may not exceed what came in.
        let floored = |v: f64| match guard {
            Some((own, cover)) if own != e.chain => v.max(cover.min(base)),
            _ => v,
        };
        if e.carve {
            carve = carve.min(floored(target + rise));
            continue;
        }
        if d <= e.half_width_m {
            if bench.is_none_or(|(bd, _)| d < bd) {
                bench = Some((d, if benches_too { floored(target) } else { target }));
            }
        } else if target > base {
            fill = fill.max(target - rise);
        } else {
            cut = cut.min(floored(target + rise));
        }
    }
    let mut h = match bench {
        Some((_, t)) => t,
        None => base.min(cut).max(fill),
    };
    h = h.min(carve);
    h
}

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let bb: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let bbox = Bounds { west: bb[0], south: bb[1], east: bb[2], north: bb[3] };
    let terrain = std::path::PathBuf::from(&a[2]);
    let dump: Option<u32> = a.get(4).and_then(|s| s.parse().ok());
    let at: Option<(f64, f64)> = a.get(3).filter(|s| s.contains(',')).map(|s| {
        let v: Vec<f64> = s.split(',').map(|x| x.parse().unwrap()).collect();
        (v[0], v[1])
    });

    let mut scene = assemble::run(std::path::Path::new(&a[0]), None, &bbox).expect("assemble");
    let solved = solve::run(&mut scene, Some(&terrain), 16, 0).expect("solve");
    let stack = ground::derive(&scene, &solved, Some(&terrain), 0);
    let mut dem = Dem::open(&terrain).expect("dem");
    let mut scratch: Vec<u32> = Vec::new();
    let strata: Vec<Stratum> = stack.layers().iter().map(|l| l.stratum).collect();

    let mut total = 0.0f64;
    let mut model_m = 0.0f64;
    let mut carved_m = 0.0f64;
    let mut by_layer = vec![0.0f64; strata.len()];
    // Per layer: [portal carve, bench, cutting face, other]
    let mut kinds = vec![0.0f64; strata.len() * 4];
    // Exposed metres bucketed by distance to the nearest end of the span:
    // 0-10, 10-25, 25-50, 50-100, 100+ m.
    let mut from_mouth = [0.0f64; 5];
    // Carved metres split by who owns the cutting edge: the tube's own
    // corridor, a peer in the same stratum, or another stratum.
    let mut whose = [0.0f64; 3];
    // whose × [portal carve, bench, cutting face, other]
    let mut whose_kind = [[0.0f64; 4]; 3];
    // The counterfactual guard: exposed metres this instrument reproduces, and
    // how many survive flooring every foreign edge at the tube's cover line.
    let (mut guard_before, mut guard_after, mut guard_faces) = (0.0f64, 0.0f64, 0.0f64);
    // Metres taken by a *neighbour*, per (cutter, victim) pair.
    let mut pairs: std::collections::BTreeMap<(u32, u32), (f64, f64)> =
        std::collections::BTreeMap::new();
    let mut mouth_stretch_m = 0.0f64;
    let mut interior_stretch_m = 0.0f64;
    let mut interior_stretches: Vec<(f64, f64, f64, f64, u32, String)> = Vec::new();
    let mut run_fracs: Vec<(f64, f64, f64, f64, u32, String)> = Vec::new();
    let mut from_mouth_model = [0.0f64; 5];
    // (exposure_m, lon, lat, corridor, kind, road, roof, ref, ground, culprit)
    let mut worst: Vec<(f64, f64, f64, u32, String, f64, f64, f64, f64, String)> = Vec::new();
    let mut per_corridor: Vec<(f64, u32, String)> = Vec::new();

    for c in &scene.corridors {
        let Some(p) = solved.profile(c.id) else { continue };
        let mut corridor_exposed = 0.0;
        for s in c.spans.iter().filter(|s| s.kind == SpanKind::Tunnel) {
            let mut series: Vec<(f64, bool, f64, f64, f64)> = Vec::new();
            let mut arc = s.arc0;
            while arc <= s.arc1 {
                let step = STEP_M.min(s.arc1 - arc + STEP_M);
                arc += STEP_M;
                let a0 = arc - STEP_M;
                let pt = p.point_at_arc(a0);
                if pt.x < bbox.west || pt.x > bbox.east || pt.y < bbox.south || pt.y > bbox.north {
                    continue;
                }
                let road = p.road_at_arc(a0);
                let roof = road + TUNNEL_HEIGHT_M;
                let reference = p.surface_at(pt.x, pt.y);
                let raw = dem.elevation(pt.x, pt.y, 16);
                let ground_m = stack.height(pt.x, pt.y, raw, 0.0, &mut scratch);
                total += step;
                series.push((a0, roof <= ground_m, ground_m - roof, pt.x, pt.y));
                if dump == Some(c.id) {
                    let (bench, fill, cut, carve) = strata
                        .iter()
                        .enumerate()
                        .map(|(n, _)| layer_parts(stack.layers()[n].earthworks(), pt.x, pt.y, raw))
                        .fold((None, f64::NEG_INFINITY, f64::INFINITY, f64::INFINITY), |a, b| {
                            (a.0.or(b.0), a.1.max(b.1), a.2.min(b.2), a.3.min(b.3))
                        });
                    println!(
                        "  arc {a0:7.1}  {:.6},{:.6}  road {road:7.2} roof {roof:7.2} ref \
                         {reference:7.2} raw {raw:7.2} ground {ground_m:7.2}  cover {:+6.2}  \
                         bench {bench:?} fill {fill:.2} cut {cut:.2} carve {carve:.2}",
                        pt.x, pt.y, ground_m - roof
                    );
                }
                if roof <= ground_m {
                    continue;
                }
                corridor_exposed += step;
                let d_mouth = (a0 - s.arc0).min(s.arc1 - a0).max(0.0);
                let b = match d_mouth {
                    d if d < 10.0 => 0,
                    d if d < 25.0 => 1,
                    d if d < 50.0 => 2,
                    d if d < 100.0 => 3,
                    _ => 4,
                };
                from_mouth[b] += step;
                // Which layer's imprint dropped the ground below the roof —
                // the first prefix whose ground goes under, walking juniors in,
                // and which *kind* of edge in it did the cutting.
                let mut culprit = String::from("(none: reference)");
                let mut prev = raw;
                let mut carved_by = None;
                for (n, st) in strata.iter().enumerate() {
                    let h = stack.height_through(n + 1, pt.x, pt.y, raw, 0.0, &mut scratch);
                    if prev > roof && h <= roof {
                        carved_by = Some(n);
                        let (bench, fill, cut, carve, owner) =
                            layer_parts_owned(stack.layers()[n].earthworks(), pt.x, pt.y, prev);
                        // Whose earthwork took the cover: this tube's own
                        // corridor (its daylighting approach), or somebody
                        // else's, in which case record the pair.
                        let owner_id = owner.map(|(id, _)| id);
                        let w = match owner_id {
                            Some(id) if id == c.id => 0,
                            Some(id) => {
                                let peer = scene.corridors.iter().find(|o| o.id == id);
                                let same = peer.is_some_and(|o| o.kind.stratum() == c.kind.stratum());
                                let e = pairs.entry((id, c.id)).or_insert((0.0, 0.0));
                                e.0 += step;
                                e.1 = e.1.max(roof - h);
                                if same { 1 } else { 2 }
                            }
                            None => 2,
                        };
                        whose[w] += step;
                        let kind = if carve <= roof {
                            kinds[n * 4] += step;
                            whose_kind[w][0] += step;
                            format!("portal carve to {carve:.2}")
                        } else if bench.is_some_and(|b| b <= roof) {
                            kinds[n * 4 + 1] += step;
                            whose_kind[w][1] += step;
                            format!("bench at {:.2}", bench.unwrap())
                        } else if cut <= roof {
                            kinds[n * 4 + 2] += step;
                            whose_kind[w][2] += step;
                            format!("cutting face to {cut:.2}")
                        } else {
                            kinds[n * 4 + 3] += step;
                            whose_kind[w][3] += step;
                            format!("other (fill {fill:.2} cut {cut:.2})")
                        };
                        culprit = format!("{st:?} {prev:.2} -> {h:.2}: {kind}");
                    }
                    prev = h;
                }
                if roof > reference && roof > raw {
                    model_m += step;
                    from_mouth_model[b] += step;
                    if carved_by.is_none() {
                        culprit = format!("roof {roof:.2} over reference {reference:.2}");
                    }
                } else {
                    carved_m += step;
                }
                if let Some(n) = carved_by {
                    by_layer[n] += step;
                }
                // The counterfactual: refold the whole stack with every foreign
                // edge floored at this tube's cover line. What is still exposed
                // afterwards is what such a rule would *not* buy — and the
                // difference between the two is the whole case for it.
                {
                    let (mut g, mut faces, mut plain) = (raw, raw, raw);
                    let cover = Some((c.id, roof + TUNNEL_COVER_M));
                    for layer in stack.layers() {
                        let ew = layer.earthworks();
                        g = layer_height_guarded(ew, pt.x, pt.y, g, cover, true);
                        faces = layer_height_guarded(ew, pt.x, pt.y, faces, cover, false);
                        plain = layer_height_guarded(ew, pt.x, pt.y, plain, None, false);
                    }
                    if roof > plain {
                        guard_before += step;
                        if roof > g {
                            guard_after += step;
                        }
                        if roof > faces {
                            guard_faces += step;
                        }
                    }
                }
                worst.push((
                    ground_m - roof,
                    pt.x,
                    pt.y,
                    c.id,
                    format!("{:?}", c.kind),
                    road,
                    roof,
                    reference,
                    ground_m,
                    culprit,
                ));
            }
            // Per-run majority: what fraction of this span's tube the finished
            // ground hides, and what a majority rule would drop.
            if !series.is_empty() {
                let bare = series.iter().filter(|r| !r.1).count();
                let frac = bare as f64 / series.len() as f64;
                let len = series.len() as f64 * STEP_M;
                run_fracs.push((frac, len, series[0].3, series[0].4, c.id, format!("{:?}", c.kind)));
            }
            // Exposed stretches bounded by covered tube on both sides are the
            // interior family; one touching either end of the span is the
            // portal transition, which is drawn on purpose.
            let mut i = 0;
            while i < series.len() {
                if series[i].1 {
                    i += 1;
                    continue;
                }
                let first = i;
                while i < series.len() && !series[i].1 {
                    i += 1;
                }
                let len = series[i - 1].0 - series[first].0 + STEP_M;
                let worst_here = series[first..i].iter().fold(0.0f64, |m, r| m.min(r.2));
                if first == 0 || i == series.len() {
                    mouth_stretch_m += len;
                } else {
                    interior_stretch_m += len;
                    interior_stretches.push((worst_here, len, series[first].3, series[first].4, c.id, format!("{:?}", c.kind)));
                }
            }
        }
        if corridor_exposed > 0.0 {
            per_corridor.push((corridor_exposed, c.id, format!("{:?}", c.kind)));
        }
    }

    println!("BORE CENSUS  bbox {:?}", bb);
    println!("  tunnel centerline in bbox : {total:.0} m");
    let exposed = model_m + carved_m;
    println!(
        "  roof above drawn ground   : {exposed:.0} m ({:.1} %)",
        100.0 * exposed / total.max(1.0)
    );
    println!(
        "    already over reference  : {model_m:.0} m  (mouth transition or profile defect)"
    );
    println!("    carved below the roof   : {carved_m:.0} m  (the junior-cutting family)");
    for (n, st) in strata.iter().enumerate() {
        if by_layer[n] > 0.0 {
            println!(
                "      cut by {st:?}: {:.0} m  (portal carve {:.0}, bench {:.0}, cutting face {:.0}, other {:.0})",
                by_layer[n], kinds[n * 4], kinds[n * 4 + 1], kinds[n * 4 + 2], kinds[n * 4 + 3]
            );
        }
    }
    println!("      whose edge          total   portal carve   bench   cutting face   other");
    for (w, label) in
        ["own corridor", "a peer, same stratum", "another stratum"].iter().enumerate()
    {
        println!(
            "        {label:<22}{:5.0} m {:10.0} m {:7.0} m {:11.0} m {:7.0} m",
            whose[w], whose_kind[w][0], whose_kind[w][1], whose_kind[w][2], whose_kind[w][3]
        );
    }
    println!(
        "\n  counterfactual — floor every FOREIGN edge at the tube's own cover line:\n    \
         {guard_before:.0} m exposed as the instrument refolds it; with every foreign edge \
         floored {guard_after:.0} m remain ({:.0} m recovered), with only foreign *faces* \
         floored (a bench is somebody's roadbed) {guard_faces:.0} m remain ({:.0} m recovered)",
        guard_before - guard_after,
        guard_before - guard_faces
    );
    let mut worst_pairs: Vec<_> = pairs.iter().map(|(k, v)| (v.0, v.1, *k)).collect();
    worst_pairs.sort_by(|a, b| b.0.total_cmp(&a.0));
    for (m, deep, (cutter, victim)) in worst_pairs.iter().take(8) {
        let name = |id: u32| {
            scene
                .corridors
                .iter()
                .find(|o| o.id == id)
                .map(|o| format!("{:?} {}", o.kind, o.id))
                .unwrap_or_else(|| format!("? {id}"))
        };
        println!(
            "        {m:5.0} m, up to {deep:4.2} m deep: {} cuts the cover off {}",
            name(*cutter),
            name(*victim)
        );
    }

    println!(
        "\n  exposed stretches: {mouth_stretch_m:.0} m touch a span end (the portal transition), \
         {interior_stretch_m:.0} m are interior (covered tube on both sides)"
    );
    interior_stretches.sort_by(|a, b| a.0.total_cmp(&b.0));
    for (w, len, lon, lat, id, kind) in interior_stretches.iter().take(10) {
        println!("    {len:5.0} m worst {w:+6.2}  {lon:.6},{lat:.6}  #{id} {kind}");
    }
    run_fracs.sort_by(|a, b| b.0.total_cmp(&a.0));
    let majority: f64 = run_fracs.iter().filter(|r| r.0 > 0.5).map(|r| r.1).sum();
    let minority_bare: f64 = run_fracs
        .iter()
        .filter(|r| r.0 <= 0.5)
        .map(|r| r.1 * r.0)
        .sum();
    println!(
        "\n  per-run majority: {} of {} spans are more bare than hidden, {majority:.0} m of tube \
         (a majority rule would withhold it); {minority_bare:.0} m of exposure stays inside \
         majority-hidden runs",
        run_fracs.iter().filter(|r| r.0 > 0.5).count(),
        run_fracs.len()
    );
    for (f, len, lon, lat, id, kind) in run_fracs.iter().take(12) {
        println!("    {:5.1} % bare of {len:6.0} m  {lon:.6},{lat:.6}  #{id} {kind}", 100.0 * f);
    }
    println!("\n  exposed metres by distance to the nearest span end:");
    for (i, label) in ["0-10 m", "10-25 m", "25-50 m", "50-100 m", "100+ m"].iter().enumerate() {
        println!(
            "    {label:>9}: {:6.0} m total, {:6.0} m of it already over reference",
            from_mouth[i], from_mouth_model[i]
        );
    }

    worst.sort_by(|x, y| x.0.total_cmp(&y.0));
    println!("\nWORST EXPOSURES");
    for w in worst.iter().take(12) {
        println!(
            "  {:+7.2}  {:.6},{:.6}  #{} {}  road {:.2} roof {:.2} ref {:.2} ground {:.2}\n            {}",
            w.0, w.1, w.2, w.3, w.4, w.5, w.6, w.7, w.8, w.9
        );
    }

    per_corridor.sort_by(|x, y| y.0.total_cmp(&x.0));
    println!("\nMOST EXPOSED CORRIDORS");
    for (m, id, kind) in per_corridor.iter().take(12) {
        println!("  {m:6.0} m  #{id} {kind}");
    }

    if let Some((lon, lat)) = at {
        println!("\nSITE {lon:.6},{lat:.6}");
        let raw = dem.elevation(lon, lat, 16);
        println!("  raw DEM z16      {raw:.2}");
        let mut prev = raw;
        for (n, st) in strata.iter().enumerate() {
            let h = stack.height_through(n + 1, lon, lat, raw, 0.0, &mut scratch);
            println!("  after {st:?}         {h:.2}   ({:+.2})", h - prev);
            prev = h;
        }
        for (n, st) in strata.iter().enumerate() {
            let l = &stack.layers()[n];
            let ew = l.earthworks();
            println!(
                "  {st:?} bench target here {:?}  covers {}",
                ew.target_at(lon, lat, &mut scratch),
                ew.covers(lon, lat, &mut scratch)
            );
            let cos_lat = lat.to_radians().cos();
            for (i, e) in ew.edges().iter().enumerate() {
                // Point-to-segment distance and side, in the metric frame.
                let px = (lon - e.a.x) * 111_320.0 * cos_lat;
                let py = (lat - e.a.y) * 110_540.0;
                let ex = (e.b.x - e.a.x) * 111_320.0 * cos_lat;
                let ey = (e.b.y - e.a.y) * 110_540.0;
                let len2 = ex * ex + ey * ey;
                let t = if len2 > 0.0 { ((px * ex + py * ey) / len2).clamp(0.0, 1.0) } else { 0.0 };
                let (qx, qy) = (px - t * ex, py - t * ey);
                let d = (qx * qx + qy * qy).sqrt();
                let cross = ex * py - ey * px;
                let side = if cross >= 0.0 { 0 } else { 1 };
                let reach = e.half_width_m + e.batter_m[side];
                if d >= reach {
                    continue;
                }
                let target = e.target_a + (e.target_b - e.target_a) * t;
                let rise = (d - e.half_width_m).max(0.0) / e.batter_run[side];
                println!(
                    "    edge {i} chain {} d {d:5.1} reach {reach:5.1} side {side}  target {target:.2} \
                     hw {:.1} batter {:.1} run {:.1} carve {}  face {:.2}",
                    e.chain, e.half_width_m, e.batter_m[side], e.batter_run[side], e.carve,
                    target + rise
                );
            }
        }
    }
}
