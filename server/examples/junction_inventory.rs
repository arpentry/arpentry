//! Inventory every drawn object inside a lon/lat window at the archive's
//! finest zoom: each surface mesh's class, level, sheet, band, plan extent and
//! height range; each painted line's class, width and height run; the terrain
//! height field over the window; and every place a `marking` stroke runs
//! through a `crossing` chord's neighbourhood.
//!
//! Usage: cargo run --release --example junction_inventory -- <archive.arpa> <w,s,e,n>

use arpentry_server::verify::scene::ArchiveScan;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let bytes = std::fs::read(&a[0]).unwrap();
    let b: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let (w, s, e, n) = (b[0], b[1], b[2], b[3]);

    let scan = ArchiveScan::open(&bytes).unwrap();
    let z = scan.max_zoom();
    println!("window lon [{w}..{e}] lat [{s}..{n}], zoom {z}");

    // Crossing chords and marking runs, gathered across tiles for the overlap
    // report at the end (in metres via each tile's own scale).
    let mut crossings: Vec<(String, Vec<(f64, f64)>)> = Vec::new(); // (id, metre points)
    let mut markings: Vec<(String, Vec<(f64, f64)>)> = Vec::new();

    for (tz, tx, ty, id) in scan.tiles_at(z) {
        let Some(ts) = scan.decode(tz, tx, ty, id) else { continue };
        if ts.bounds.east < w || ts.bounds.west > e || ts.bounds.north < s || ts.bounds.south > n {
            continue;
        }
        // Window in this tile's unit space.
        let ux = |lon: f64| (lon - ts.bounds.west) / ts.bounds.width();
        let uy = |lat: f64| (lat - ts.bounds.south) / ts.bounds.height();
        let (uw, ue, us, un) = (ux(w), ux(e), uy(s), uy(n));
        let inside = |px: f64, py: f64| px >= uw && px <= ue && py >= us && py <= un;

        println!("\n== tile {tz}/{tx}/{ty} ==");

        if let Some(t) = &ts.terrain {
            let (mut lo, mut hi, mut cnt) = (f64::MAX, f64::MIN, 0u32);
            let steps = 60;
            for i in 0..=steps {
                for j in 0..=steps {
                    let px = uw + (ue - uw) * i as f64 / steps as f64;
                    let py = us + (un - us) * j as f64 / steps as f64;
                    if !ts.owns(px, py) {
                        continue;
                    }
                    if let Some(h) = t.height_at(px, py) {
                        lo = lo.min(h);
                        hi = hi.max(h);
                        cnt += 1;
                    }
                }
            }
            if cnt > 0 {
                println!("terrain: {cnt} samples, z [{lo:.2} .. {hi:.2}] m (span {:.2} m)", hi - lo);
            }
        }

        for r in &ts.roads {
            let (mut lo, mut hi) = (f64::MAX, f64::MIN);
            let (mut bx0, mut bx1, mut by0, mut by1) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
            let mut cnt = 0u32;
            for i in 0..r.mesh.vertex_count() {
                let (px, py, pz) = r.mesh.vertex(i);
                if !inside(px, py) || !ts.owns(px, py) {
                    continue;
                }
                if std::env::var_os("ARPT_INV_VERTS").is_some() && r.class.starts_with("walk_") {
                    let (lon, lat) = ts.lonlat(px, py);
                    println!("  v {:<12} sheet={:?} {lon:.6},{lat:.6} z {pz:.2}", r.class, r.sheet);
                }
                lo = lo.min(pz);
                hi = hi.max(pz);
                bx0 = bx0.min(px);
                bx1 = bx1.max(px);
                by0 = by0.min(py);
                by1 = by1.max(py);
                cnt += 1;
            }
            if cnt == 0 {
                continue;
            }
            let (lon0, lat0) = ts.lonlat(bx0, by0);
            let (lon1, lat1) = ts.lonlat(bx1, by1);
            println!(
                "mesh  {:<14} level={:+} sheet={:?} band={:<12} verts_in={:<5} z [{:.2} .. {:.2}] ({:5.2} m)  plan [{:.6},{:.6} .. {:.6},{:.6}]",
                r.class, r.level, r.sheet, r.band, cnt, lo, hi, hi - lo, lon0, lat0, lon1, lat1
            );
        }

        for (relief, m) in &ts.buildings {
            let (mut lo, mut hi, mut cnt) = (f64::MAX, f64::MIN, 0u32);
            let (mut cx, mut cy) = (0.0, 0.0);
            for i in 0..m.vertex_count() {
                let (px, py, pz) = m.vertex(i);
                if !inside(px, py) || !ts.owns(px, py) {
                    continue;
                }
                lo = lo.min(pz);
                hi = hi.max(pz);
                cx += px;
                cy += py;
                cnt += 1;
            }
            if cnt == 0 {
                continue;
            }
            let (lon, lat) = ts.lonlat(cx / cnt as f64, cy / cnt as f64);
            println!(
                "bldg  relief={relief:.0} m  verts_in={cnt:<5} z [{lo:.2} .. {hi:.2}] ({:.2} m tall)  at {lon:.6},{lat:.6}",
                hi - lo
            );
        }

        for l in &ts.lines {
            for part in &l.parts {
                let pts: Vec<&(f64, f64, f64)> =
                    part.iter().filter(|(px, py, _)| inside(*px, *py) && ts.owns(*px, *py)).collect();
                if pts.is_empty() {
                    continue;
                }
                let (mut lo, mut hi) = (f64::MAX, f64::MIN);
                let mut len = 0.0;
                for p in &pts {
                    lo = lo.min(p.2);
                    hi = hi.max(p.2);
                }
                for pair in pts.windows(2) {
                    len += ts.scale.dist(pair[0].0, pair[0].1, pair[1].0, pair[1].1);
                }
                let (lon0, lat0) = ts.lonlat(pts[0].0, pts[0].1);
                let (lon1, lat1) = ts.lonlat(pts[pts.len() - 1].0, pts[pts.len() - 1].1);
                println!(
                    "line  {:<14} level={:+} width={:.2} m  pts={:<4} len={:6.1} m  z [{:.2} .. {:.2}]  {:.6},{:.6} -> {:.6},{:.6}",
                    l.class, l.level, l.width_m, pts.len(), len, lo, hi, lon0, lat0, lon1, lat1
                );
                let metre: Vec<(f64, f64)> =
                    pts.iter().map(|(px, py, _)| (px * ts.scale.mx, py * ts.scale.my)).collect();
                let tag = format!("{:.6},{:.6}", lon0, lat0);
                if l.class == "crossing" {
                    crossings.push((tag, metre));
                } else if l.class == "marking" {
                    markings.push((tag, metre));
                }
            }
        }
    }

    // Where a marking stroke runs inside a crossing chord's neighbourhood.
    println!("\n== marking-through-crossing (marking point within 2.5 m of a crossing chord) ==");
    for (ctag, chord) in &crossings {
        for (mtag, run) in &markings {
            let mut worst = f64::MAX;
            let mut hits = 0u32;
            for &(mx, my) in run {
                for seg in chord.windows(2) {
                    let d = point_seg(mx, my, seg[0], seg[1]);
                    if d < worst {
                        worst = d;
                    }
                    if d < 2.5 {
                        hits += 1;
                        break;
                    }
                }
            }
            if hits > 0 {
                println!("crossing at {ctag}: marking at {mtag} has {hits} points inside, nearest {worst:.2} m");
            }
        }
    }
}

fn point_seg(px: f64, py: f64, a: (f64, f64), b: (f64, f64)) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let l2 = dx * dx + dy * dy;
    let t = if l2 == 0.0 { 0.0 } else { ((px - a.0) * dx + (py - a.1) * dy) / l2 };
    let t = t.clamp(0.0, 1.0);
    let (qx, qy) = (a.0 + t * dx, a.1 + t * dy);
    ((px - qx).powi(2) + (py - qy).powi(2)).sqrt()
}
