//! Maps source-feature attributes to tile properties, zoom range, and rank.
//!
//! The layer is assigned per input file on the CLI (`--input N:path`), so the
//! profile only derives the rest: the styling class/subclass, the zoom range
//! (from Overture `cartography` when present, else the global range), and the
//! within-layer rank (from `cartography.sort_key`). Works for both Overture
//! (`class`/`subclass`) and Natural Earth (`type`/`subtype`).

use crate::layers::{self, LayerIndex};
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

/// Default building height in metres for buildings with no height data.
/// Overture CH has measured heights for ~38% of buildings and floor counts for
/// a few percent more; without a fallback the remaining majority would have no
/// height and towns would render as scattered tall buildings.
const DEFAULT_BUILDING_HEIGHT: f64 = 5.0;

/// Assumed storey height in metres when only a floor count is available.
const FLOOR_HEIGHT: f64 = 3.0;

/// First zoom at which buildings are emitted (FORMAT.md §9: building 13–16).
const BUILDING_MIN_ZOOM: u8 = 13;

/// First zoom at which POI labels are emitted.
const POI_MIN_ZOOM: u8 = 13;

/// Derives tile properties and zoom range from source attributes, falling back
/// to `[default_min, default_max]` when the source has no zoom hints. `layer`
/// disambiguates classes that exist in several themes (e.g. `residential` is
/// both a road class and a land-use class).
pub fn profile(
    layer: LayerIndex,
    props: &[(String, Value)],
    default_min: u8,
    default_max: u8,
) -> Profiled {
    match layer {
        layers::BUILDING => return profile_building(props, default_max),
        layers::POI => return profile_poi(props, default_max),
        layers::BOUNDARY => return profile_boundary(props, default_max),
        _ => {}
    }
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
        .or_else(|| class.as_deref().and_then(|c| overture_min_zoom(layer, c)))
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
    // Road names drive the client's line-following street labels; the
    // bridge/tunnel level (from Overture `level_rules`) drives the client's
    // lifted-deck / sunk-tunnel draping. Ground (0) is the implicit default and
    // omitted, so only structures carry the property.
    if layer == layers::TRANSPORTATION {
        if let Some(n) = find_str(props, "names.primary") {
            properties.push(("name".to_string(), Value::String(n)));
        }
        if let Some(lv) = find_int(props, "level_rules").filter(|&l| l != 0) {
            properties.push(("level".to_string(), Value::Int(lv)));
        }
        // Physical carriageway width from the engineering priors — the same
        // numbers the structure sweep uses — so the client can stroke roads
        // at true width at close zooms and meet the decks edge-to-edge.
        if let Some(w) = crate::priors::paint_width_m(
            find_str(props, "class").as_deref(),
            find_str(props, "subclass").as_deref(),
        ) {
            properties.push(("width_m".to_string(), Value::Double(w)));
        }
    }

    Profiled { properties, min_zoom, max_zoom, rank }
}

/// Profiles an Overture building: the broad `subtype` (residential,
/// commercial, …) becomes the styling class with the finer `class` (house,
/// garage, …) as subclass, and every building gets a `height` — measured when
/// available, else floors × [`FLOOR_HEIGHT`], else [`DEFAULT_BUILDING_HEIGHT`].
fn profile_building(props: &[(String, Value)], default_max: u8) -> Profiled {
    let class = find_str(props, "subtype").unwrap_or_else(|| "building".to_string());
    let height = find_f64(props, "height")
        .or_else(|| find_int(props, "num_floors").map(|n| n as f64 * FLOOR_HEIGHT))
        .filter(|h| *h > 0.0)
        .unwrap_or(DEFAULT_BUILDING_HEIGHT);

    let mut properties = vec![
        ("class".to_string(), Value::String(class)),
        ("height".to_string(), Value::Double(height)),
    ];
    if let Some(s) = find_str(props, "class") {
        properties.push(("subclass".to_string(), Value::String(s)));
    }
    // Roof attributes drive server-side 3D meshes at high zoom (see
    // `building_mesh`). Sparse in Overture (most buildings are flat), so only
    // emitted when present.
    if let Some(shape) = find_str(props, "roof_shape") {
        properties.push(("roof_shape".to_string(), Value::String(shape)));
    }
    if let Some(rh) = find_f64(props, "roof_height").filter(|h| *h > 0.0) {
        properties.push(("roof_height".to_string(), Value::Double(rh)));
    }
    Profiled { properties, min_zoom: BUILDING_MIN_ZOOM, max_zoom: default_max, rank: 0 }
}

/// Places below this Overture `confidence` are dropped — they are mostly
/// stale or misplaced POIs, and Switzerland alone has 683k places.
const POI_MIN_CONFIDENCE: f64 = 0.5;

/// Profiles an Overture place into a POI label: `names.primary` becomes the
/// `name`, `basic_category` the class, and `confidence` the within-layer rank
/// (most-confident first, so they win label collision). Nameless and
/// low-confidence places render nothing useful and are dropped.
fn profile_poi(props: &[(String, Value)], default_max: u8) -> Profiled {
    let Some(name) = find_str(props, "names.primary") else {
        return drop_feature();
    };
    let confidence = find_f64(props, "confidence").unwrap_or(0.5);
    if confidence < POI_MIN_CONFIDENCE {
        return drop_feature();
    }
    let mut properties = vec![("name".to_string(), Value::String(name))];
    if let Some(c) = find_str(props, "basic_category") {
        properties.push(("class".to_string(), Value::String(c)));
    }
    let rank = ((1.0 - confidence.clamp(0.0, 1.0)) * 100.0) as u16;
    Profiled { properties, min_zoom: POI_MIN_ZOOM, max_zoom: default_max, rank }
}

/// Profiles an Overture division boundary: the admin level (`subtype` —
/// country, region, county, …) becomes the styling class, with the
/// land/maritime `class` as subclass. Lower levels appear at lower zooms.
fn profile_boundary(props: &[(String, Value)], default_max: u8) -> Profiled {
    let Some(subtype) = find_str(props, "subtype") else {
        return drop_feature();
    };
    let min_zoom = match subtype.as_str() {
        "country" | "dependency" => 1,
        "macroregion" | "region" => 5,
        "macrocounty" | "county" => 9,
        _ => 11, // localadmin, locality, borough, neighborhood, …
    };
    let mut properties = vec![("class".to_string(), Value::String(subtype))];
    if let Some(s) = find_str(props, "class") {
        properties.push(("subclass".to_string(), Value::String(s)));
    }
    Profiled { properties, min_zoom, max_zoom: default_max, rank: 0 }
}

/// A profile whose zoom range is empty, so the pipeline never emits the
/// feature (`min_zoom > max_zoom` — see `process_feature`).
fn drop_feature() -> Profiled {
    Profiled { properties: Vec::new(), min_zoom: u8::MAX, max_zoom: 0, rank: 0 }
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

/// Per-class minimum zoom for Overture features whose theme carries no
/// `cartography` hints.
///
/// Overture's `transportation`, `water`, and `land_use` themes have no
/// `cartography.min_zoom` column (only `land_cover` and `base` do). Without this
/// table their classes fall through to the global `default_min` (0), so every
/// footpath, stream, swimming pool, and meadow is emitted at every zoom — a
/// single z7 tile over Switzerland ends up with ~180k road segments and ~49k
/// water features, ballooning to ~15 MB. These minimums mirror common web-map
/// styles (a feature first appears near the zoom it would first be drawn),
/// keeping fine detail out of low-zoom tiles. Keyed by layer because class
/// names collide across themes (`residential` is a z12 road class and a z10
/// land-use class). Returns `None` for unknown classes (use the default);
/// known overlaps with [`natural_earth_min_zoom`] are matched there first, so
/// this only fires for Overture-specific classes.
fn overture_min_zoom(layer: LayerIndex, class: &str) -> Option<u8> {
    Some(match (layer, class) {
        // --- transportation (Overture road/rail classes) ---
        (layers::TRANSPORTATION, "motorway") => 4,
        (layers::TRANSPORTATION, "trunk") => 5,
        (layers::TRANSPORTATION, "primary") => 7,
        (layers::TRANSPORTATION, "secondary") => 9,
        (layers::TRANSPORTATION, "tertiary") => 11,
        (layers::TRANSPORTATION, "residential" | "unclassified" | "living_street") => 12,
        (layers::TRANSPORTATION, "pedestrian" | "service" | "track") => 13,
        (
            layers::TRANSPORTATION,
            "footway" | "path" | "steps" | "cycleway" | "bridleway" | "sidewalk"
            | "crosswalk" | "unknown",
        ) => 14,
        // Rail (Overture `subtype=rail`, class is the gauge).
        (
            layers::TRANSPORTATION,
            "standard_gauge" | "narrow_gauge" | "broad_gauge" | "monorail" | "subway"
            | "light_rail" | "tram" | "funicular",
        ) => 8,

        // --- water ---
        (layers::WATER, "ocean" | "sea") => 0,
        (layers::WATER, "lake") => 4,
        (layers::WATER, "reservoir") => 6,
        (layers::WATER, "wetland") => 7,
        (layers::WATER, "river" | "water") => 8,
        (layers::WATER, "canal") => 10,
        (layers::WATER, "stream") => 12,
        (layers::WATER, "ditch" | "drain" | "pond" | "basin" | "dock" | "moat") => 13,
        (layers::WATER, "swimming_pool" | "fountain" | "spring" | "waterfall" | "fish_pass") => 14,

        // --- land_use ---
        (layers::LAND_USE, "military") => 8,
        (
            layers::LAND_USE,
            "forest" | "farmland" | "residential" | "commercial" | "industrial"
            | "retail" | "park" | "recreation_ground",
        ) => 10,
        (
            layers::LAND_USE,
            "meadow" | "grass" | "orchard" | "vineyard" | "cemetery" | "downhill"
            | "nordic" | "farmyard",
        ) => 11,
        (layers::LAND_USE, "pitch" | "garden" | "allotments" | "greenhouse_horticulture") => 13,
        (layers::LAND_USE, "playground") => 14,

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

fn find_f64(props: &[(String, Value)], key: &str) -> Option<f64> {
    props.iter().find(|(k, _)| k == key).and_then(|(_, v)| match v {
        Value::Double(d) => Some(*d),
        Value::Int(i) => Some(*i as f64),
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
        let p = profile(layers::WATER, &props, 0, 16);
        assert_eq!(p.min_zoom, 6);
        assert_eq!(p.max_zoom, 14);
        assert_eq!(p.rank, 40);
        assert!(p.properties.contains(&("class".into(), Value::String("river".into()))));
        assert!(p.properties.contains(&("subclass".into(), Value::String("canal".into()))));
    }

    #[test]
    fn natural_earth_falls_back_to_type_and_defaults() {
        let props = vec![("type".to_string(), Value::String("land".into()))];
        let p = profile(layers::LAND, &props, 0, 8);
        assert_eq!(p.min_zoom, 0);
        assert_eq!(p.max_zoom, 8);
        assert_eq!(p.rank, 0);
        assert!(p.properties.contains(&("class".into(), Value::String("land".into()))));
    }

    #[test]
    fn natural_earth_class_drives_min_zoom() {
        // Roads (the bulk of the data) drop out below z4; land stays at z0.
        let road =
            profile(layers::TRANSPORTATION, &[("type".to_string(), Value::String("road".into()))], 0, 8);
        assert_eq!(road.min_zoom, 4);
        let land = profile(layers::LAND, &[("type".to_string(), Value::String("land".into()))], 0, 8);
        assert_eq!(land.min_zoom, 0);
        // Unknown classes fall back to the global default.
        let unknown =
            profile(layers::LAND, &[("type".to_string(), Value::String("mystery".into()))], 0, 8);
        assert_eq!(unknown.min_zoom, 0);
    }

    #[test]
    fn overture_cartography_overrides_class_heuristic() {
        // A feature with explicit cartography hints uses them, not the NE table.
        let props = vec![
            ("class".to_string(), Value::String("road".into())),
            ("cartography.min_zoom".to_string(), Value::Int(7)),
        ];
        assert_eq!(profile(layers::TRANSPORTATION, &props, 0, 14).min_zoom, 7);
    }

    #[test]
    fn overture_classes_without_cartography_get_per_class_min_zoom() {
        // Overture transportation/water/land_use carry no cartography hints, so
        // without the per-class table these would default to 0 and flood every
        // zoom. The fine-detail classes are pushed to high zooms.
        let footway = profile(
            layers::TRANSPORTATION,
            &[("class".to_string(), Value::String("footway".into()))],
            0,
            14,
        );
        assert_eq!(footway.min_zoom, 14);
        let stream =
            profile(layers::WATER, &[("class".to_string(), Value::String("stream".into()))], 0, 14);
        assert_eq!(stream.min_zoom, 12);
        let motorway = profile(
            layers::TRANSPORTATION,
            &[("class".to_string(), Value::String("motorway".into()))],
            0,
            14,
        );
        assert_eq!(motorway.min_zoom, 4);
        // Unknown classes still fall back to the default.
        let mystery =
            profile(layers::WATER, &[("class".to_string(), Value::String("xyzzy".into()))], 0, 14);
        assert_eq!(mystery.min_zoom, 0);
    }

    #[test]
    fn colliding_class_names_resolve_by_layer() {
        // `residential` is both a road class (z12) and a land-use class (z10);
        // the layer decides which table applies.
        let props = vec![("class".to_string(), Value::String("residential".into()))];
        assert_eq!(profile(layers::TRANSPORTATION, &props, 0, 14).min_zoom, 12);
        assert_eq!(profile(layers::LAND_USE, &props, 0, 14).min_zoom, 10);
        // A class outside its own layer's table falls back to the default.
        assert_eq!(profile(layers::WATER, &props, 0, 14).min_zoom, 0);
    }

    #[test]
    fn cartography_hint_overrides_overture_table() {
        // land_cover (and base) carry cartography; that wins over the table.
        let props = vec![
            ("class".to_string(), Value::String("footway".into())),
            ("cartography.min_zoom".to_string(), Value::Int(2)),
        ];
        assert_eq!(profile(layers::TRANSPORTATION, &props, 0, 14).min_zoom, 2);
    }

    #[test]
    fn rank_is_clamped_to_field_width() {
        let props = vec![("cartography.sort_key".to_string(), Value::Int(999_999))];
        assert_eq!(profile(layers::WATER, &props, 0, 16).rank, tileid::MAX_RANK);
    }

    #[test]
    fn building_maps_subtype_to_class_with_measured_height() {
        let props = vec![
            ("subtype".to_string(), Value::String("residential".into())),
            ("class".to_string(), Value::String("house".into())),
            ("height".to_string(), Value::Double(12.5)),
        ];
        let p = profile(layers::BUILDING, &props, 0, 14);
        assert_eq!(p.min_zoom, BUILDING_MIN_ZOOM);
        assert!(p.properties.contains(&("class".into(), Value::String("residential".into()))));
        assert!(p.properties.contains(&("subclass".into(), Value::String("house".into()))));
        assert!(p.properties.contains(&("height".into(), Value::Double(12.5))));
    }

    #[test]
    fn building_height_falls_back_to_floors_then_default() {
        // Floors × storey height when no measured height.
        let floors = vec![("num_floors".to_string(), Value::Int(4))];
        let p = profile(layers::BUILDING, &floors, 0, 14);
        assert!(p.properties.contains(&("height".into(), Value::Double(12.0))));
        // Bare footprint still extrudes at the default height, classed generically.
        let p = profile(layers::BUILDING, &[], 0, 14);
        assert!(p.properties.contains(&("class".into(), Value::String("building".into()))));
        assert!(p.properties.contains(&("height".into(), Value::Double(DEFAULT_BUILDING_HEIGHT))));
    }

    #[test]
    fn transportation_takes_road_name() {
        let props = vec![
            ("class".to_string(), Value::String("residential".into())),
            ("names.primary".to_string(), Value::String("Rue du Lac".into())),
        ];
        let p = profile(layers::TRANSPORTATION, &props, 0, 14);
        assert!(p.properties.contains(&("name".into(), Value::String("Rue du Lac".into()))));
        // Other layers ignore names.primary.
        let p = profile(layers::WATER, &props, 0, 14);
        assert!(!p.properties.iter().any(|(k, _)| k == "name"));
    }

    #[test]
    fn transportation_carries_bridge_tunnel_level() {
        // A bridge (positive level_rules) and a tunnel (negative) become the
        // reserved `level` property; a ground road (no level_rules) omits it.
        let bridge = profile(
            layers::TRANSPORTATION,
            &[
                ("class".to_string(), Value::String("motorway".into())),
                ("level_rules".to_string(), Value::Int(1)),
            ],
            0,
            14,
        );
        assert!(bridge.properties.contains(&("level".into(), Value::Int(1))));
        let tunnel = profile(
            layers::TRANSPORTATION,
            &[("level_rules".to_string(), Value::Int(-1))],
            0,
            14,
        );
        assert!(tunnel.properties.contains(&("level".into(), Value::Int(-1))));
        let ground = profile(
            layers::TRANSPORTATION,
            &[("class".to_string(), Value::String("residential".into()))],
            0,
            14,
        );
        assert!(!ground.properties.iter().any(|(k, _)| k == "level"));
    }

    #[test]
    fn drivable_roads_carry_their_physical_width() {
        // Drivable classes take twice the structure half-width prior; a ramp
        // narrows to one lane; non-drivable classes keep cartographic widths.
        let road = profile(
            layers::TRANSPORTATION,
            &[("class".to_string(), Value::String("motorway".into()))],
            0,
            14,
        );
        assert!(road.properties.contains(&("width_m".into(), Value::Double(9.0))));
        let ramp = profile(
            layers::TRANSPORTATION,
            &[
                ("class".to_string(), Value::String("motorway".into())),
                ("subclass".to_string(), Value::String("link".into())),
            ],
            0,
            14,
        );
        assert!(ramp.properties.contains(&("width_m".into(), Value::Double(5.5))));
        let path = profile(
            layers::TRANSPORTATION,
            &[("class".to_string(), Value::String("pedestrian".into()))],
            0,
            14,
        );
        assert!(!path.properties.iter().any(|(k, _)| k == "width_m"));
    }

    #[test]
    fn poi_takes_name_and_ranks_by_confidence() {
        let props = vec![
            ("names.primary".to_string(), Value::String("Café Fédéral".into())),
            ("basic_category".to_string(), Value::String("restaurant".into())),
            ("confidence".to_string(), Value::Double(0.9)),
        ];
        let p = profile(layers::POI, &props, 0, 14);
        assert_eq!(p.min_zoom, POI_MIN_ZOOM);
        assert!(p.properties.contains(&("name".into(), Value::String("Café Fédéral".into()))));
        assert!(p.properties.contains(&("class".into(), Value::String("restaurant".into()))));
        assert_eq!(p.rank, 9); // (1 - 0.9) × 100, near the front of the layer
    }

    #[test]
    fn nameless_poi_is_dropped() {
        let props = vec![("basic_category".to_string(), Value::String("restaurant".into()))];
        let p = profile(layers::POI, &props, 0, 14);
        assert!(p.min_zoom > p.max_zoom, "empty zoom range drops the feature");
    }

    #[test]
    fn boundary_admin_level_drives_min_zoom() {
        let boundary = |subtype: &str| {
            let props = vec![
                ("subtype".to_string(), Value::String(subtype.into())),
                ("class".to_string(), Value::String("land".into())),
            ];
            profile(layers::BOUNDARY, &props, 0, 14)
        };
        assert_eq!(boundary("country").min_zoom, 1);
        assert_eq!(boundary("region").min_zoom, 5);
        assert_eq!(boundary("county").min_zoom, 9);
        // The admin level is the styling class; land/maritime is the subclass.
        let p = boundary("region");
        assert!(p.properties.contains(&("class".into(), Value::String("region".into()))));
        assert!(p.properties.contains(&("subclass".into(), Value::String("land".into()))));
    }
}
