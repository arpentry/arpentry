//! Temporary probe: what `class` values do MESH features carry, vs LINE
//! features, in a tile archive? Run:
//!   ARPA=/tmp/x.arpa Z=17 cargo test --release --test probe_mesh_classes -- --nocapture
use arpentry_server::archive::Archive;
use arpentry_server::fb::tile::arpentry::tiles as fbt;
use std::collections::BTreeMap;

#[test]
fn probe() {
    let Ok(path) = std::env::var("ARPA") else { return };
    let z_want: u8 = std::env::var("Z").ok().and_then(|s| s.parse().ok()).unwrap_or(17);
    let bytes = std::fs::read(&path).unwrap();
    let archive = Archive::open(&bytes).unwrap();
    let mut mesh: BTreeMap<String, u64> = Default::default();
    let mut line: BTreeMap<String, u64> = Default::default();
    let mut mesh_levels: BTreeMap<(String, i64), u64> = Default::default();
    for entry in archive.entries() {
        if entry.z != z_want {
            continue;
        }
        let raw = {
            let mut out = Vec::new();
            let mut input = archive.get_by_id(entry.hilbert_id).unwrap();
            brotli::BrotliDecompress(&mut input, &mut out).unwrap();
            out
        };
        let tile = fbt::root_as_tile(&raw).unwrap();
        let (Some(layers), Some(keys), Some(values)) = (tile.layers(), tile.keys(), tile.values())
        else {
            continue;
        };
        for li in 0..layers.len() {
            let l = layers.get(li);
            if l.name() != "transportation" {
                continue;
            }
            let Some(feats) = l.features() else { continue };
            for fi in 0..feats.len() {
                let f = feats.get(fi);
                let is_mesh = f.geometry_as_mesh_geometry().is_some();
                let Some(props) = f.properties() else { continue };
                let (mut class, mut level) = (None, 0i64);
                for pi in 0..props.len() {
                    let p = props.get(pi);
                    let v = values.get(p.value() as usize);
                    match keys.get(p.key() as usize) {
                        "class" => class = v.string_value().map(str::to_string),
                        "level" => level = v.int_value(),
                        _ => {}
                    }
                }
                let c = class.unwrap_or_else(|| "<none>".into());
                if is_mesh {
                    *mesh.entry(c.clone()).or_default() += 1;
                    *mesh_levels.entry((c, level)).or_default() += 1;
                } else {
                    *line.entry(c).or_default() += 1;
                }
            }
        }
    }

    eprintln!("MESH features at z{z_want}:");
    for (class, n) in &mesh {
        eprintln!("  {n:>8}  {class}");
    }
    eprintln!("LINE features at z{z_want}:");
    for (class, n) in &line {
        eprintln!("  {n:>8}  {class}");
    }
    eprintln!("MESH features by (class, level):");
    for ((class, level), n) in &mesh_levels {
        eprintln!("  {n:>8}  {class} level {level}");
    }
}
