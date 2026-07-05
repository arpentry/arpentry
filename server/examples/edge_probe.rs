//! Probe the smoothness of swept structure meshes in one tile: reconstruct
//! each deck's top-left edge (vertex order is 8 per cross-section) and print
//! per-vertex lateral deviation from the local chord, in metres.
//! Usage: cargo run --example edge_probe -- <archive.arpa> <z> <x> <y>

use arpentry_server::archive::Archive;
use arpentry_server::fb::tile::arpentry::tiles as fbt;
use arpentry_server::project::{dequantize_x, dequantize_y, Bounds};

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let (path, z, x, y) = (
        &a[0],
        a[1].parse::<u8>().unwrap(),
        a[2].parse::<u32>().unwrap(),
        a[3].parse::<u32>().unwrap(),
    );
    let bytes = std::fs::read(path).unwrap();
    let archive = Archive::open(&bytes).unwrap();
    let blob = archive.get(z, x, y).expect("tile present");
    let mut raw = Vec::new();
    let mut input = blob;
    brotli::BrotliDecompress(&mut input, &mut raw).unwrap();
    let tile = fbt::root_as_tile(&raw).unwrap();
    let bounds = Bounds::of_tile(z, x, y);
    let cos_lat = ((bounds.south + bounds.north) * 0.5_f64).to_radians().cos();
    let m_per_deg = 111_320.0 * cos_lat;

    let layers = tile.layers().unwrap();
    for i in 0..layers.len() {
        let layer = layers.get(i);
        if layer.name() != "transportation" {
            continue;
        }
        let Some(feats) = layer.features() else { continue };
        for fi in 0..feats.len() {
            if std::env::var_os("MESH_PROPS").is_some() {
                let f = feats.get(fi);
                if f.geometry_as_mesh_geometry().is_none() {
                    continue;
                }
                let keys = tile.keys().unwrap();
                let values = tile.values().unwrap();
                let mut props = String::new();
                if let Some(kv) = f.properties() {
                    for k in 0..kv.len() {
                        let p = kv.get(k);
                        let key = keys.get(p.key() as usize);
                        let v = values.get(p.value() as usize);
                        let vs = v
                            .string_value()
                            .map(|s| s.to_string())
                            .unwrap_or_else(|| format!("{}", v.int_value()));
                        props.push_str(&format!("{key}={vs} "));
                    }
                }
                let n = f.geometry_as_mesh_geometry().map(|m| m.x().len()).unwrap_or(0);
                println!("mesh feature {fi}: verts={n} props: {props}");
                continue;
            }
            if std::env::var_os("TRI_DUMP").is_some() {
                let f = feats.get(fi);
                let Some(mg) = f.geometry_as_mesh_geometry() else { continue };
                let mut level: i64 = 0;
                if let (Some(kv), Some(keys), Some(values)) =
                    (f.properties(), tile.keys(), tile.values())
                {
                    for k in 0..kv.len() {
                        let p = kv.get(k);
                        if keys.get(p.key() as usize) == "level" {
                            level = values.get(p.value() as usize).int_value();
                        }
                    }
                }
                let (xs, ys, zs) = (mg.x(), mg.y(), mg.z());
                let iv = mg.indices();
                let n = xs.len();
                let vx: Vec<String> = (0..n)
                    .map(|k| format!("{:.7}", dequantize_x(xs.get(k), &bounds)))
                    .collect();
                let vy: Vec<String> = (0..n)
                    .map(|k| format!("{:.7}", dequantize_y(ys.get(k), &bounds)))
                    .collect();
                let vz: Vec<String> = (0..n).map(|k| format!("{}", zs.get(k))).collect();
                let ii: Vec<String> = (0..iv.len()).map(|k| format!("{}", iv.get(k))).collect();
                println!(
                    "{{\"f\":{fi},\"level\":{level},\"x\":[{}],\"y\":[{}],\"z\":[{}],\"i\":[{}]}}",
                    vx.join(","), vy.join(","), vz.join(","), ii.join(",")
                );
                continue;
            }
            if std::env::var_os("LINE_PROBE").is_some() {
                let f = feats.get(fi);
                let Some(lg) = f.geometry_as_line_geometry() else { continue };
                let (xs, ys) = (lg.x(), lg.y());
                let zs = lg.z();
                let n = xs.len();
                if n < 6 {
                    continue;
                }
                let pts: Vec<(f64, f64)> = (0..n)
                    .map(|k| {
                        (
                            dequantize_x(xs.get(k), &bounds) * m_per_deg,
                            dequantize_y(ys.get(k), &bounds) * 111_320.0,
                        )
                    })
                    .collect();
                let (mut mx, mut sum, mut cnt) = (0.0_f64, 0.0, 0usize);
                let mut seg_sum = 0.0;
                for k in 1..n - 1 {
                    let (a, b, c) = (pts[k - 1], pts[k], pts[k + 1]);
                    let (ex, ey) = (c.0 - a.0, c.1 - a.1);
                    let l = (ex * ex + ey * ey).sqrt().max(1e-9);
                    let d = ((b.0 - a.0) * ey - (b.1 - a.1) * ex).abs() / l;
                    mx = mx.max(d);
                    sum += d;
                    cnt += 1;
                    seg_sum += ((b.0 - a.0) * (b.0 - a.0) + (b.1 - a.1) * (b.1 - a.1)).sqrt();
                }
                let lon0 = dequantize_x(xs.get(0), &bounds);
                let lat0 = dequantize_y(ys.get(0), &bounds);
                if lon0 > 6.9285 && lon0 < 6.9320 && lat0 > 6.0 && lat0 < 90.0 && lat0 > 46.4232 && lat0 < 46.4285 {
                    println!(
                        "line {fi}: verts={n} lat_dev max={mx:.2} mean={:.3} seg_mean={:.1}m has_z={} start=({lon0:.5},{lat0:.5})",
                        sum / cnt as f64,
                        seg_sum / (n - 2) as f64,
                        zs.is_some()
                    );
                }
                continue;
            }
            let f = feats.get(fi);
            let Some(mg) = f.geometry_as_mesh_geometry() else { continue };
            let (xs, ys, zs) = (mg.x(), mg.y(), mg.z());
            let n = xs.len();
            if n < 16 {
                continue;
            }
            // Top-left edge: vertex 8k+0 of each section.
            let pts: Vec<(f64, f64, f64)> = (0..n / 8)
                .map(|k| {
                    (
                        dequantize_x(xs.get(8 * k), &bounds) * m_per_deg,
                        dequantize_y(ys.get(8 * k), &bounds) * 111_320.0,
                        zs.get(8 * k) as f64 / 1000.0,
                    )
                })
                .collect();
            let m = pts.len();
            let mut lat_max: f64 = 0.0;
            let mut lat_sum = 0.0;
            let mut vert_max: f64 = 0.0;
            let mut devs = Vec::new();
            for k in 1..m - 1 {
                let (a, b, c) = (pts[k - 1], pts[k], pts[k + 1]);
                let (ex, ey) = (c.0 - a.0, c.1 - a.1);
                let len = (ex * ex + ey * ey).sqrt().max(1e-9);
                // Lateral: perpendicular distance of b from chord a..c.
                let d = ((b.0 - a.0) * ey - (b.1 - a.1) * ex).abs() / len;
                // Vertical: height of b off the a..c chord.
                let t = ((b.0 - a.0) * ex + (b.1 - a.1) * ey) / (len * len);
                let dv = (b.2 - (a.2 + (c.2 - a.2) * t)).abs();
                lat_max = lat_max.max(d);
                lat_sum += d;
                vert_max = vert_max.max(dv);
                devs.push((d * 100.0).round() / 100.0);
            }
            if std::env::var_os("EDGE_GEOJSON").is_some() {
                let m_per = m_per_deg;
                let edge = |off: usize| -> Vec<Vec<f64>> {
                    (0..n / 8)
                        .map(|k| {
                            vec![
                                dequantize_x(xs.get(8 * k + off), &bounds),
                                dequantize_y(ys.get(8 * k + off), &bounds),
                                zs.get(8 * k + off) as f64 / 1000.0,
                            ]
                        })
                        .collect()
                };
                let _ = m_per;
                for (name, off) in [("L", 0usize), ("R", 1usize)] {
                    let coords: Vec<String> = edge(off)
                        .iter()
                        .map(|c| format!("[{:.7},{:.7},{:.2}]", c[0], c[1], c[2]))
                        .collect();
                    println!(
                        "{{\"type\":\"Feature\",\"properties\":{{\"f\":{fi},\"e\":\"{name}\"}},\"geometry\":{{\"type\":\"LineString\",\"coordinates\":[{}]}}}}",
                        coords.join(",")
                    );
                }
                continue;
            }
            let big: Vec<(usize, f64)> = devs
                .iter()
                .enumerate()
                .filter(|(_, &d)| d > 0.3 && d < 20.0)
                .map(|(k, &d)| (k, d))
                .collect();
            println!(
                "feature {fi}: sections={m} verts={n} lat_max={lat_max:.2} mean={:.3} vmax={vert_max:.2} start=({:.5},{:.5}) end=({:.5},{:.5}) jogs={big:?}",
                lat_sum / (m - 2) as f64,
                dequantize_x(xs.get(0), &bounds), dequantize_y(ys.get(0), &bounds),
                dequantize_x(xs.get(n - 8), &bounds), dequantize_y(ys.get(n - 8), &bounds)
            );
        }
    }
}
