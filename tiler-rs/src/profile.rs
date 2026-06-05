//! Maps source-feature attributes to tile properties, zoom range, and rank.
//!
//! The layer is assigned per input file on the CLI (`--input N:path`), so the
//! profile only derives the rest: the styling class/subclass, the zoom range
//! (from Overture `cartography` when present, else the global range), and the
//! within-layer rank (from `cartography.sort_key`). Works for both Overture
//! (`class`/`subclass`) and Natural Earth (`type`/`subtype`).

use crate::tileid;
use crate::value::Value;

/// The profiled result for one source feature.
#[derive(Debug, Clone)]
pub struct Profiled {
    /// Normalized reserved-key properties (class, subclass) to encode.
    pub properties: Vec<(String, Value)>,
    pub min_zoom: u8,
    pub max_zoom: u8,
    pub rank: u16,
}

/// Derives tile properties and zoom range from source attributes, falling back
/// to `[default_min, default_max]` when the source has no zoom hints.
pub fn profile(props: &[(String, Value)], default_min: u8, default_max: u8) -> Profiled {
    let class = find_str(props, "class")
        .or_else(|| find_str(props, "type"))
        .or_else(|| find_str(props, "subtype"));
    let subclass = find_str(props, "subclass");

    // Zoom range: prefer Overture's per-feature `cartography` hints; for Natural
    // Earth (which has none) fall back to a per-class minimum so minor features
    // drop out of low-zoom tiles; otherwise the global default.
    let min_zoom = find_int(props, "cartography.min_zoom")
        .and_then(u8_from)
        .or_else(|| class.as_deref().and_then(natural_earth_min_zoom))
        .unwrap_or(default_min);
    let max_zoom = find_int(props, "cartography.max_zoom").and_then(u8_from).unwrap_or(default_max);
    let rank = find_int(props, "cartography.sort_key")
        .map(|i| i.clamp(0, tileid::MAX_RANK as i64) as u16)
        .unwrap_or(0);

    let mut properties = Vec::new();
    if let Some(c) = class {
        properties.push(("class".to_string(), Value::String(c)));
    }
    if let Some(s) = subclass {
        properties.push(("subclass".to_string(), Value::String(s)));
    }

    Profiled { properties, min_zoom, max_zoom, rank }
}

/// Per-class minimum zoom for Natural Earth features.
///
/// Natural Earth has no per-feature zoom hints (unlike Overture's `cartography`
/// fields), so without this every feature would be emitted at every zoom — the
/// whole world's road/river/coastline network lands in the single z0 tile,
/// bloating low-zoom tiles to many megabytes. These minimums mirror the Natural
/// Earth style's per-class `min_level`s, so a feature first appears at the zoom
/// it would first be drawn. Returns `None` for unknown classes (use the default).
fn natural_earth_min_zoom(class: &str) -> Option<u8> {
    Some(match class {
        "land" => 0,
        "boundary" => 1,
        "lake" | "glacier" | "ice_shelf" => 2,
        "admin1_boundary" | "river" | "reef" => 3,
        // Roads, urban land use, and the (un-styled) coastline/graticule lines are
        // the bulk of the data; keeping them out of z0–z3 is what lightens the
        // low-zoom tiles. `road` matches the style's min_level of 4.
        "road" | "urban" | "coastline" | "geographic_line" => 4,
        _ => return None,
    })
}

fn find_str(props: &[(String, Value)], key: &str) -> Option<String> {
    props.iter().find(|(k, _)| k == key).and_then(|(_, v)| match v {
        Value::String(s) => Some(s.clone()),
        _ => None,
    })
}

fn find_int(props: &[(String, Value)], key: &str) -> Option<i64> {
    props.iter().find(|(k, _)| k == key).and_then(|(_, v)| match v {
        Value::Int(i) => Some(*i),
        _ => None,
    })
}

fn u8_from(i: i64) -> Option<u8> {
    (0..=255).contains(&i).then_some(i as u8)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn overture_class_and_cartography() {
        let props = vec![
            ("class".to_string(), Value::String("river".into())),
            ("subclass".to_string(), Value::String("canal".into())),
            ("cartography.min_zoom".to_string(), Value::Int(6)),
            ("cartography.max_zoom".to_string(), Value::Int(14)),
            ("cartography.sort_key".to_string(), Value::Int(40)),
        ];
        let p = profile(&props, 0, 16);
        assert_eq!(p.min_zoom, 6);
        assert_eq!(p.max_zoom, 14);
        assert_eq!(p.rank, 40);
        assert!(p.properties.contains(&("class".into(), Value::String("river".into()))));
        assert!(p.properties.contains(&("subclass".into(), Value::String("canal".into()))));
    }

    #[test]
    fn natural_earth_falls_back_to_type_and_defaults() {
        let props = vec![("type".to_string(), Value::String("land".into()))];
        let p = profile(&props, 0, 8);
        assert_eq!(p.min_zoom, 0);
        assert_eq!(p.max_zoom, 8);
        assert_eq!(p.rank, 0);
        assert!(p.properties.contains(&("class".into(), Value::String("land".into()))));
    }

    #[test]
    fn natural_earth_class_drives_min_zoom() {
        // Roads (the bulk of the data) drop out below z4; land stays at z0.
        let road = profile(&[("type".to_string(), Value::String("road".into()))], 0, 8);
        assert_eq!(road.min_zoom, 4);
        let land = profile(&[("type".to_string(), Value::String("land".into()))], 0, 8);
        assert_eq!(land.min_zoom, 0);
        // Unknown classes fall back to the global default.
        let unknown = profile(&[("type".to_string(), Value::String("mystery".into()))], 0, 8);
        assert_eq!(unknown.min_zoom, 0);
    }

    #[test]
    fn overture_cartography_overrides_class_heuristic() {
        // A feature with explicit cartography hints uses them, not the NE table.
        let props = vec![
            ("class".to_string(), Value::String("road".into())),
            ("cartography.min_zoom".to_string(), Value::Int(7)),
        ];
        assert_eq!(profile(&props, 0, 14).min_zoom, 7);
    }

    #[test]
    fn rank_is_clamped_to_field_width() {
        let props = vec![("cartography.sort_key".to_string(), Value::Int(999_999))];
        assert_eq!(profile(&props, 0, 16).rank, tileid::MAX_RANK);
    }
}
