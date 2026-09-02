//! Dump the layer/feature structure of one tile in an `.arpa` archive.
//! Usage: cargo run --example dump_tile -- <archive.arpa> <z> <x> <y>

use arpentry_server::archive::Archive;
use arpentry_server::fb::tile::arpentry::tiles as fbt;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let (path, z, x, y) = (&a[0], a[1].parse::<u8>().unwrap(), a[2].parse::<u32>().unwrap(), a[3].parse::<u32>().unwrap());

    let bytes = std::fs::read(path).unwrap();
    let archive = Archive::open(&bytes).unwrap();
    let Some(blob) = archive.get(z, x, y) else {
        println!("tile {z}/{x}/{y}: NOT PRESENT in archive");
        return;
    };

    let mut raw = Vec::new();
    let mut input = blob;
    brotli::BrotliDecompress(&mut input, &mut raw).unwrap();
    println!("tile {z}/{x}/{y}: {} compressed -> {} bytes, id={:?}", blob.len(), raw.len(), &raw[4..8]);

    let tile = fbt::root_as_tile(&raw).unwrap();

    // Print the tile-scope key + string-value dictionaries.
    if let Some(keys) = tile.keys() {
        let ks: Vec<&str> = (0..keys.len()).map(|i| keys.get(i)).collect();
        println!("  keys = {ks:?}");
    }
    if let Some(values) = tile.values() {
        let vs: Vec<String> = (0..values.len())
            .map(|i| {
                let v = values.get(i);
                v.string_value().map(|s| s.to_string()).unwrap_or_else(|| format!("{}", v.int_value()))
            })
            .collect();
        println!("  values = {vs:?}");
    }

    let values = tile.values();
    let layers = tile.layers().unwrap();
    for i in 0..layers.len() {
        let layer = layers.get(i);
        let feats = layer.features();
        let n = feats.map(|f| f.len()).unwrap_or(0);
        let kind = feats
            .filter(|f| f.len() > 0)
            .map(|f| format!("{:?}", f.get(0).geometry_type()))
            .unwrap_or_else(|| "-".into());
        println!("  layer[{i}] {:<16} features={n:<6} first_geom={kind}", layer.name());

        // Transportation: dump every feature's class/level and plan bbox, so a
        // structure solid's drawn extent can be compared with its span arcs.
        if layer.name() == "transportation" {
            if let Some(feats) = feats {
                for j in 0..feats.len() {
                    let f = feats.get(j);
                    let mut cls = String::new();
                    let mut level = String::new();
                    let mut band = String::new();
                    if let (Some(props), Some(keys), Some(vals)) = (f.properties(), tile.keys(), values) {
                        for k in 0..props.len() {
                            let pr = props.get(k);
                            let key = keys.get(pr.key() as usize);
                            let v = vals.get(pr.value() as usize);
                            let vs = v.string_value().map(|s| s.to_string()).unwrap_or_else(|| v.int_value().to_string());
                            match key {
                                "class" => cls = vs,
                                "level" => level = vs,
                                "band_class" => band = vs,
                                _ => {}
                            }
                        }
                    }
                    let (mut xmin, mut xmax, mut ymin, mut ymax, mut nv) = (u16::MAX, 0u16, u16::MAX, 0u16, 0);
                    let mut push = |xs: flatbuffers::Vector<u16>, ys: flatbuffers::Vector<u16>| {
                        for k in 0..xs.len() {
                            xmin = xmin.min(xs.get(k)); xmax = xmax.max(xs.get(k));
                            ymin = ymin.min(ys.get(k)); ymax = ymax.max(ys.get(k));
                        }
                        nv += xs.len();
                    };
                    let gt = format!("{:?}", f.geometry_type());
                    if std::env::var("ARPT_DUMP_FEAT").ok().and_then(|v| v.parse::<usize>().ok()) == Some(j) {
                        if let Some(g) = f.geometry_as_mesh_geometry() {
                            let (xs, ys, zs) = (g.x(), g.y(), g.z());
                            for k in 0..xs.len() {
                                println!("      v[{k}] {} {} {}", xs.get(k), ys.get(k), zs.get(k));
                            }
                            let idx = g.indices();
                            let tris: Vec<u32> = (0..idx.len()).map(|k| idx.get(k)).collect();
                            println!("      indices ({}): {tris:?}", tris.len());
                        }
                    }
                    if let Some(g) = f.geometry_as_mesh_geometry() { push(g.x(), g.y()); }
                    if let Some(g) = f.geometry_as_line_geometry() { push(g.x(), g.y()); }
                    println!("    feat[{j}] {gt} class={cls} level={level} band={band} verts={nv} x=[{xmin}..{xmax}] y=[{ymin}..{ymax}]");
                }
            }
        }

        // For the first polygon feature, dump coordinate bbox + resolved class.
        if let Some(f) = feats.filter(|f| f.len() > 0).map(|f| f.get(0)) {
            if let Some(pg) = f.geometry_as_polygon_geometry() {
                let xs = pg.x();
                let ys = pg.y();
                let (mut xmin, mut xmax, mut ymin, mut ymax) = (u16::MAX, 0u16, u16::MAX, 0u16);
                for k in 0..xs.len() {
                    xmin = xmin.min(xs.get(k)); xmax = xmax.max(xs.get(k));
                    ymin = ymin.min(ys.get(k)); ymax = ymax.max(ys.get(k));
                }
                let ring = pg.ring_offsets().map(|r| r.len()).unwrap_or(0);
                let cls = f.properties().filter(|p| p.len() > 0).map(|p| p.get(0).value());
                let cls_str = cls.and_then(|vi| values.map(|v| {
                    let val = v.get(vi as usize);
                    val.string_value().map(|s| s.to_string()).unwrap_or_else(|| val.int_value().to_string())
                }));
                println!("      poly: verts={} x=[{xmin}..{xmax}] y=[{ymin}..{ymax}] ring_off_len={ring} class={:?}",
                    xs.len(), cls_str);
            }
        }
    }
}
