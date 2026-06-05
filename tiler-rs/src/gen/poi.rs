//! Hardcoded points of interest near the procedural town at (0, 0) (port of
//! `gen/poi.c`). Coordinates are in degrees; every POI lies within the town area.

use crate::project::Bounds;

/// POI key indices into the tile-scope key dictionary.
pub const POI_KEY_NAME: u32 = 2;
pub const POI_KEY_ICON: u32 = 3;

/// Class value for "poi".
pub const POI_VAL_POI: u32 = 18;
/// First POI name string index in the value dictionary. Icon strings follow the
/// name strings: `POI_VAL_NAME_BASE + poi_count()`.
pub const POI_VAL_NAME_BASE: u32 = 19;

// POI area bounding box (slightly larger than the town core).
const POI_WEST: f64 = -0.010;
const POI_EAST: f64 = 0.010;
const POI_SOUTH: f64 = -0.010;
const POI_NORTH: f64 = 0.010;

/// A single point of interest.
pub struct Poi {
    pub lon: f64,
    pub lat: f64,
    pub name: &'static str,
    /// Maki icon name (e.g. "hospital", "school").
    pub icon: &'static str,
}

/// The hardcoded POI list (order is significant — it indexes the value dict).
pub const POIS: &[Poi] = &[
    Poi { lon: 0.0000, lat: 0.0000, name: "Town Hall", icon: "town-hall" },
    Poi { lon: -0.0030, lat: 0.0025, name: "Library", icon: "library" },
    Poi { lon: 0.0035, lat: -0.0020, name: "Market", icon: "grocery" },
    Poi { lon: -0.0050, lat: -0.0040, name: "Park", icon: "park" },
    Poi { lon: 0.0040, lat: 0.0035, name: "School", icon: "school" },
    Poi { lon: 0.0020, lat: -0.0050, name: "Station", icon: "rail" },
    Poi { lon: -0.0045, lat: 0.0050, name: "Hospital", icon: "hospital" },
    Poi { lon: 0.0055, lat: 0.0010, name: "Museum", icon: "museum" },
];

/// Whether the tile bounds overlap the POI area.
pub fn poi_overlaps(bounds: &Bounds) -> bool {
    bounds.east > POI_WEST && bounds.west < POI_EAST && bounds.north > POI_SOUTH && bounds.south < POI_NORTH
}
