//! GeoParquet reader for the tiler's inputs (TILER.md §geoparquet/overture).
//!
//! Overture Maps and Natural Earth store geometry as a WKB binary column
//! (`geometry`), so this decodes the WKB column with [`crate::wkb`] and pulls
//! requested attribute columns by name — including dotted paths into nested
//! structs (`bbox.xmin`, `cartography.min_zoom`). No geoarrow dependency is
//! needed.
//!
//! Built for inputs far larger than memory:
//!
//! - **Footer-only open**: [`GeoParquet::open`] reads just the Parquet footer,
//!   never row data.
//! - **Row-group pruning**: [`GeoParquet::row_groups_intersecting`] uses the
//!   per-row-group statistics of the `bbox` struct column (Overture files are
//!   spatially clustered) to skip row groups outside the tiling bounds without
//!   reading them.
//! - **Projection pushdown**: only the geometry column and the roots of the
//!   requested attribute columns are decoded.
//! - **Streaming**: [`GeoParquet::features`] yields features batch by batch;
//!   nothing is materialized beyond one Arrow record batch.
//!
//! Deep module: callers see `open` + `features` (and the Overture/NE
//! conveniences); Arrow downcasting, nesting, and null handling are internal.
//! Requesting an absent column is not an error — it is simply skipped, so one
//! Overture column list works across its per-theme schema variants.

use std::collections::HashSet;
use std::fs::File;
use std::path::{Path, PathBuf};

use arrow::array::{
    Array, ArrayRef, BinaryArray, BooleanArray, Float32Array, Float64Array, Int32Array,
    Int64Array, LargeBinaryArray, LargeStringArray, StringArray, StructArray,
};
use arrow::datatypes::DataType;
use arrow::record_batch::RecordBatch;
use parquet::arrow::arrow_reader::{
    ArrowReaderMetadata, ArrowReaderOptions, ParquetRecordBatchReader,
    ParquetRecordBatchReaderBuilder,
};
use parquet::arrow::ProjectionMask;
use parquet::file::statistics::Statistics;

use crate::value::Value;
use crate::wkb::{self, WkbError};

/// Default GeoParquet primary geometry column name.
const DEFAULT_GEOMETRY_COLUMN: &str = "geometry";

/// Rows decoded per Arrow record batch while streaming.
const BATCH_SIZE: usize = 4096;

/// One decoded source feature: geometry plus its non-null requested attributes.
#[derive(Debug, Clone)]
pub struct Feature {
    pub geometry: geo_types::Geometry,
    /// Requested attributes that were present and non-null, in request order.
    pub properties: Vec<(String, Value)>,
    /// Overture transportation `level_rules` parsed into constant-level runs
    /// (empty for everything else); the assemble stage resolves these into
    /// corridor spans.
    pub level_runs: Vec<crate::levels::LevelRun>,
    /// Overture transportation `connectors` (empty for everything else); the
    /// assemble stage joins segments into corridors on these.
    pub connectors: Vec<crate::assemble::columns::Connector>,
    /// Overture transportation `subclass_rules` parsed into runs (empty for
    /// everything else). Where the scalar `subclass` is uniform this holds one
    /// full-length run saying the same thing; where it is partial the scalar
    /// is null and this is the only record of it (`docs/SOURCES.md` §2).
    pub subclass_runs: Vec<crate::assemble::columns::SubclassRun>,
    /// Stretches Overture's `road_flags` marks `is_indoor`, as fractions. Not
    /// levels — an indoor way has no ordinal — so they ride separately from
    /// `level_runs` and reach `WalkLine::spans` (`crate::levels::indoor_runs`).
    pub indoor_runs: Vec<(f64, f64)>,
}

/// Errors from opening or decoding a GeoParquet file.
#[derive(Debug)]
pub enum ReadError {
    Io(std::io::Error),
    Parquet(parquet::errors::ParquetError),
    Arrow(arrow::error::ArrowError),
    Wkb(WkbError),
    /// The file lacked an expected column.
    Schema(String),
}

impl std::fmt::Display for ReadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ReadError::Io(e) => write!(f, "io error: {e}"),
            ReadError::Parquet(e) => write!(f, "parquet error: {e}"),
            ReadError::Arrow(e) => write!(f, "arrow error: {e}"),
            ReadError::Wkb(e) => write!(f, "wkb error: {e:?}"),
            ReadError::Schema(s) => write!(f, "schema error: {s}"),
        }
    }
}
impl std::error::Error for ReadError {}
impl From<std::io::Error> for ReadError {
    fn from(e: std::io::Error) -> Self {
        ReadError::Io(e)
    }
}
impl From<parquet::errors::ParquetError> for ReadError {
    fn from(e: parquet::errors::ParquetError) -> Self {
        ReadError::Parquet(e)
    }
}
impl From<arrow::error::ArrowError> for ReadError {
    fn from(e: arrow::error::ArrowError) -> Self {
        ReadError::Arrow(e)
    }
}
impl From<WkbError> for ReadError {
    fn from(e: WkbError) -> Self {
        ReadError::Wkb(e)
    }
}

/// A GeoParquet file opened for streaming reads. Holds only the parsed footer;
/// row data is read on demand by [`GeoParquet::features`]. Cheap to clone-open
/// per thread — each [`features`](GeoParquet::features) call uses its own file
/// handle, so one instance can serve concurrent readers of disjoint row groups.
pub struct GeoParquet {
    path: PathBuf,
    meta: ArrowReaderMetadata,
    geometry_col: String,
}

impl GeoParquet {
    /// Opens a GeoParquet file, reading only its footer metadata.
    pub fn open(path: impl AsRef<Path>) -> Result<Self, ReadError> {
        let path = path.as_ref().to_path_buf();
        let file = File::open(&path)?;
        let meta = ArrowReaderMetadata::load(&file, ArrowReaderOptions::new())?;
        Ok(GeoParquet { path, meta, geometry_col: DEFAULT_GEOMETRY_COLUMN.to_string() })
    }

    /// Number of row groups in the file.
    pub fn num_row_groups(&self) -> usize {
        self.meta.metadata().num_row_groups()
    }

    /// Total row count across the given row groups.
    pub fn num_rows(&self, row_groups: &[usize]) -> u64 {
        let md = self.meta.metadata();
        row_groups.iter().map(|&rg| md.row_group(rg).num_rows() as u64).sum()
    }

    /// Indices of row groups whose `bbox` column statistics intersect the
    /// `(west, south, east, north)` bounds. Pruning is best-effort: row groups
    /// are kept when the file has no `bbox` struct or a row group lacks
    /// statistics, so the result is always a superset of the matching rows.
    pub fn row_groups_intersecting(&self, bounds: (f64, f64, f64, f64)) -> Vec<usize> {
        let (west, south, east, north) = bounds;
        let md = self.meta.metadata();
        let columns = md.file_metadata().schema_descr().columns();
        let find = |name: &str| columns.iter().position(|c| c.path().string() == name);
        let (Some(xmin), Some(xmax), Some(ymin), Some(ymax)) =
            (find("bbox.xmin"), find("bbox.xmax"), find("bbox.ymin"), find("bbox.ymax"))
        else {
            return (0..md.num_row_groups()).collect();
        };

        (0..md.num_row_groups())
            .filter(|&rg| {
                let row_group = md.row_group(rg);
                let lo = |col: usize| stat_f64(row_group.column(col).statistics(), false);
                let hi = |col: usize| stat_f64(row_group.column(col).statistics(), true);
                match (lo(xmin), hi(xmax), lo(ymin), hi(ymax)) {
                    (Some(x0), Some(x1), Some(y0), Some(y1)) => {
                        x0 <= east && x1 >= west && y0 <= north && y1 >= south
                    }
                    // Missing statistics → can't prove disjoint, keep it.
                    _ => true,
                }
            })
            .collect()
    }

    /// Streams features from the given row groups, decoding only the geometry
    /// column and the roots of the requested attribute columns (dotted paths
    /// descend into nested structs). Rows with a null geometry are skipped;
    /// absent columns are skipped silently.
    pub fn features(&self, row_groups: Vec<usize>, attr_columns: &[&str]) -> Result<Features, ReadError> {
        let file = File::open(&self.path)?;
        let builder = ParquetRecordBatchReaderBuilder::new_with_metadata(file, self.meta.clone());
        let mask = self.projection(attr_columns)?;
        let reader = builder
            .with_batch_size(BATCH_SIZE)
            .with_row_groups(row_groups)
            .with_projection(mask)
            .build()?;
        Ok(Features {
            reader,
            geometry_col: self.geometry_col.clone(),
            attrs: attr_columns.iter().map(|s| s.to_string()).collect(),
            cursor: None,
        })
    }

    /// Decodes every row of the file into a [`Feature`]. Materializes the whole
    /// file — prefer [`features`](Self::features) for large inputs.
    pub fn read_features(&self, attr_columns: &[&str]) -> Result<Vec<Feature>, ReadError> {
        self.features((0..self.num_row_groups()).collect(), attr_columns)?.collect()
    }

    /// Projection mask selecting the geometry column plus the root columns of
    /// the requested attributes (a dotted path selects its whole root struct).
    fn projection(&self, attr_columns: &[&str]) -> Result<ProjectionMask, ReadError> {
        let schema = self.meta.metadata().file_metadata().schema_descr();
        let mut want: HashSet<&str> =
            attr_columns.iter().filter_map(|a| a.split('.').next()).collect();
        want.insert(self.geometry_col.as_str());

        let fields = schema.root_schema().get_fields();
        let mut roots = Vec::new();
        let mut have_geometry = false;
        for (i, field) in fields.iter().enumerate() {
            if want.contains(field.name()) {
                roots.push(i);
                have_geometry |= field.name() == self.geometry_col;
            }
        }
        if !have_geometry {
            return Err(ReadError::Schema(format!(
                "missing geometry column '{}'",
                self.geometry_col
            )));
        }
        Ok(ProjectionMask::roots(schema, roots))
    }
}

/// Extracts a row-group statistic as `f64` (`max` when `upper`, else `min`).
fn stat_f64(stats: Option<&Statistics>, upper: bool) -> Option<f64> {
    match stats {
        Some(Statistics::Double(s)) => {
            if upper { s.max_opt().copied() } else { s.min_opt().copied() }
        }
        Some(Statistics::Float(s)) => {
            if upper { s.max_opt().map(|v| *v as f64) } else { s.min_opt().map(|v| *v as f64) }
        }
        _ => None,
    }
}

/// Streaming feature iterator returned by [`GeoParquet::features`]. Holds at
/// most one Arrow record batch at a time.
pub struct Features {
    reader: ParquetRecordBatchReader,
    geometry_col: String,
    attrs: Vec<String>,
    cursor: Option<Cursor>,
}

/// One record batch with its columns resolved, and a row position.
struct Cursor {
    geom: ArrayRef,
    /// Requested attribute columns present in this batch, in request order.
    resolved: Vec<(String, ArrayRef)>,
    rows: usize,
    row: usize,
}

impl Iterator for Features {
    type Item = Result<Feature, ReadError>;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            // Pull the next batch when none is in flight or the current one is
            // exhausted.
            if self.cursor.as_ref().is_none_or(|c| c.row >= c.rows) {
                let batch = match self.reader.next() {
                    None => return None,
                    Some(Ok(b)) => b,
                    Some(Err(e)) => return Some(Err(e.into())),
                };
                let Some(geom) = batch.column_by_name(&self.geometry_col).cloned() else {
                    return Some(Err(ReadError::Schema(format!(
                        "missing geometry column '{}'",
                        self.geometry_col
                    ))));
                };
                let resolved = self
                    .attrs
                    .iter()
                    .filter_map(|name| resolve_column(&batch, name).map(|a| (name.clone(), a)))
                    .collect();
                self.cursor = Some(Cursor { geom, resolved, rows: batch.num_rows(), row: 0 });
                continue;
            }

            let cur = self.cursor.as_mut().expect("cursor in flight");
            let row = cur.row;
            cur.row += 1;
            let Some(bytes) = geometry_bytes(cur.geom.as_ref(), row) else {
                continue; // null geometry → skip the row
            };
            let geometry = match wkb::parse(bytes) {
                Ok(g) => g,
                Err(e) => return Some(Err(e.into())),
            };
            let mut properties = Vec::new();
            let mut level_runs = Vec::new();
            let mut flag_runs = Vec::new();
            let mut connectors = Vec::new();
            let mut subclass_runs = Vec::new();
            let mut indoor_runs = Vec::new();
            for (name, arr) in &cur.resolved {
                // Overture's bridge/tunnel signal is `level_rules`, a
                // linearly-referenced `list<struct<value, between>>` rather than
                // a scalar: parse it into level runs carried on the feature so
                // the assemble stage can resolve the segment's structure spans
                // (see `crate::levels`), and skip the scalar property path.
                if name == "level_rules" {
                    level_runs = crate::levels::parse(arr.as_ref(), row);
                    continue;
                }
                // `road_flags` carries the same structure signal as flags
                // (`is_bridge`/`is_tunnel`); a substantial share of structures
                // have only the flag, no `level_rules` (see `crate::levels`).
                if name == "road_flags" {
                    flag_runs = crate::levels::parse_flags(arr.as_ref(), row);
                    // The same cell also says whether the way is indoors,
                    // which is not a level and is dropped by the level parse.
                    indoor_runs = crate::levels::indoor_runs(arr.as_ref(), row);
                    continue;
                }
                // `connectors` is likewise nested: the graph topology the
                // assemble stage joins corridors on.
                if name == "connectors" {
                    connectors = crate::assemble::columns::parse_connectors(arr.as_ref(), row);
                    continue;
                }
                // `subclass_rules` is the linearly-referenced form of the
                // scalar `subclass` beside it — and the only form when any run
                // is partial, because Overture nulls the scalar then
                // (`docs/SOURCES.md` §2). Parsed into runs here; the assemble
                // stage cuts pedestrian lines on them.
                if name == "subclass_rules" {
                    subclass_runs =
                        crate::assemble::columns::parse_subclass_rules(arr.as_ref(), row);
                    continue;
                }
                // The horizontal road attributes share `level_rules`' nested
                // shape but their consumers need scalars: each is reduced to
                // its dominant value here (see `crate::rules`), riding the
                // property vec like any flat column. `access_restrictions`
                // reduces to the derived one-way verdict.
                if name == "width_rules" {
                    if let Some(w) = crate::rules::dominant_width_m(arr.as_ref(), row) {
                        properties.push((name.clone(), Value::Double(w)));
                    }
                    continue;
                }
                if name == "road_surface" {
                    if let Some(s) = crate::rules::dominant_surface(arr.as_ref(), row) {
                        properties.push((name.clone(), Value::String(s)));
                    }
                    continue;
                }
                if name == "access_restrictions" {
                    if crate::rules::is_oneway(arr.as_ref(), row) {
                        properties.push(("oneway".to_string(), Value::Bool(true)));
                    }
                    continue;
                }
                if let Some(v) = array_value(arr.as_ref(), row) {
                    properties.push((name.clone(), v));
                }
            }
            // Where `level_rules` said nothing, the flags stand in: a
            // flagged-only bridge still earns its structure span. Where both
            // exist the rules win — they carry real ordinals (stacked decks).
            if level_runs.is_empty() {
                level_runs = flag_runs;
            }
            return Some(Ok(Feature {
                geometry,
                properties,
                level_runs,
                connectors,
                subclass_runs,
                indoor_runs,
            }));
        }
    }
}

/// Reads Natural Earth features (geometry + `id`/`type`/`subtype`).
pub fn read_naturalearth(path: impl AsRef<Path>) -> Result<Vec<Feature>, ReadError> {
    GeoParquet::open(path)?.read_features(&["id", "type", "subtype"])
}

/// Reads Overture features. The column list is a superset across themes —
/// columns absent from a given theme are skipped.
pub fn read_overture(path: impl AsRef<Path>) -> Result<Vec<Feature>, ReadError> {
    GeoParquet::open(path)?.read_features(&[
        "id",
        "subtype",
        "class",
        "subclass",
        "depth",
        "level",
        "cartography.min_zoom",
        "cartography.max_zoom",
        "cartography.sort_key",
    ])
}

/// Follows a dotted path to the leaf array, descending struct columns.
fn resolve_column(batch: &RecordBatch, path: &str) -> Option<ArrayRef> {
    let mut parts = path.split('.');
    let mut arr: ArrayRef = batch.column_by_name(parts.next()?)?.clone();
    for field in parts {
        let st = arr.as_any().downcast_ref::<StructArray>()?;
        arr = st.column_by_name(field)?.clone();
    }
    Some(arr)
}

/// Borrows the WKB bytes for a row from a (Large)Binary array.
fn geometry_bytes(array: &dyn Array, row: usize) -> Option<&[u8]> {
    if array.is_null(row) {
        return None;
    }
    if let Some(a) = array.as_any().downcast_ref::<BinaryArray>() {
        return Some(a.value(row));
    }
    if let Some(a) = array.as_any().downcast_ref::<LargeBinaryArray>() {
        return Some(a.value(row));
    }
    None
}

/// Reads a scalar cell as a [`Value`], or `None` if null or unsupported.
fn array_value(array: &dyn Array, row: usize) -> Option<Value> {
    if array.is_null(row) {
        return None;
    }
    let any = array.as_any();
    match array.data_type() {
        DataType::Utf8 => any.downcast_ref::<StringArray>().map(|a| Value::String(a.value(row).to_string())),
        DataType::LargeUtf8 => any.downcast_ref::<LargeStringArray>().map(|a| Value::String(a.value(row).to_string())),
        DataType::Int32 => any.downcast_ref::<Int32Array>().map(|a| Value::Int(a.value(row) as i64)),
        DataType::Int64 => any.downcast_ref::<Int64Array>().map(|a| Value::Int(a.value(row))),
        DataType::Float32 => any.downcast_ref::<Float32Array>().map(|a| Value::Double(a.value(row) as f64)),
        DataType::Float64 => any.downcast_ref::<Float64Array>().map(|a| Value::Double(a.value(row))),
        DataType::Boolean => any.downcast_ref::<BooleanArray>().map(|a| Value::Bool(a.value(row))),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use arrow::array::Float64Array as F64;
    use arrow::datatypes::{Field, Fields, Schema};
    use geo_types::{Geometry, Point};
    use parquet::arrow::ArrowWriter;
    use parquet::file::properties::WriterProperties;
    use std::sync::atomic::{AtomicU32, Ordering};
    use std::sync::Arc;

    fn wkb_point(x: f64, y: f64) -> Vec<u8> {
        let mut b = vec![1u8]; // little-endian
        b.extend_from_slice(&1u32.to_le_bytes()); // type Point
        b.extend_from_slice(&x.to_le_bytes());
        b.extend_from_slice(&y.to_le_bytes());
        b
    }

    fn temp_parquet(name: &str) -> std::path::PathBuf {
        static SEQ: AtomicU32 = AtomicU32::new(0);
        let n = SEQ.fetch_add(1, Ordering::Relaxed);
        std::env::temp_dir().join(format!("arpt-gp-{}-{}-{}.parquet", std::process::id(), name, n))
    }

    #[test]
    fn reads_geometry_and_flat_attributes() {
        let p1 = wkb_point(1.0, 2.0);
        let p2 = wkb_point(3.0, 4.0);
        let geom = BinaryArray::from(vec![Some(p1.as_slice()), Some(p2.as_slice())]);
        let class = StringArray::from(vec![Some("water"), None]);
        let min_zoom = Int32Array::from(vec![Some(5), Some(7)]);

        let schema = Schema::new(vec![
            Field::new("geometry", DataType::Binary, true),
            Field::new("class", DataType::Utf8, true),
            Field::new("min_zoom", DataType::Int32, true),
        ]);
        let batch = RecordBatch::try_new(
            Arc::new(schema),
            vec![Arc::new(geom), Arc::new(class), Arc::new(min_zoom)],
        )
        .unwrap();

        let path = temp_parquet("flat");
        {
            let file = File::create(&path).unwrap();
            let mut w = ArrowWriter::try_new(file, batch.schema(), None).unwrap();
            w.write(&batch).unwrap();
            w.close().unwrap();
        }

        let gp = GeoParquet::open(&path).unwrap();
        // "absent" must be silently ignored, not an error.
        let feats = gp.read_features(&["class", "min_zoom", "absent"]).unwrap();
        std::fs::remove_file(&path).ok();

        assert_eq!(feats.len(), 2);
        assert_eq!(feats[0].geometry, Geometry::Point(Point::new(1.0, 2.0)));
        assert!(feats[0].properties.contains(&("class".to_string(), Value::String("water".into()))));
        assert!(feats[0].properties.contains(&("min_zoom".to_string(), Value::Int(5))));
        // Row 1's class is null → omitted; min_zoom present.
        assert!(!feats[1].properties.iter().any(|(k, _)| k == "class"));
        assert!(feats[1].properties.contains(&("min_zoom".to_string(), Value::Int(7))));
    }

    /// Writes one point per row with an Overture-style `bbox` struct, forcing a
    /// row group per point so each gets its own statistics.
    fn write_bbox_file(name: &str, points: &[(f64, f64)]) -> std::path::PathBuf {
        let bbox_fields = Fields::from(vec![
            Field::new("xmin", DataType::Float64, true),
            Field::new("xmax", DataType::Float64, true),
            Field::new("ymin", DataType::Float64, true),
            Field::new("ymax", DataType::Float64, true),
        ]);
        let schema = Arc::new(Schema::new(vec![
            Field::new("geometry", DataType::Binary, true),
            Field::new("bbox", DataType::Struct(bbox_fields.clone()), true),
        ]));

        let path = temp_parquet(name);
        let props = WriterProperties::builder().set_max_row_group_size(1).build();
        let file = File::create(&path).unwrap();
        let mut w = ArrowWriter::try_new(file, schema.clone(), Some(props)).unwrap();
        for &(x, y) in points {
            let wkb = wkb_point(x, y);
            let geom = BinaryArray::from(vec![Some(wkb.as_slice())]);
            let bbox = StructArray::new(
                bbox_fields.clone(),
                vec![
                    Arc::new(F64::from(vec![x])) as ArrayRef,
                    Arc::new(F64::from(vec![x])) as ArrayRef,
                    Arc::new(F64::from(vec![y])) as ArrayRef,
                    Arc::new(F64::from(vec![y])) as ArrayRef,
                ],
                None,
            );
            let batch =
                RecordBatch::try_new(schema.clone(), vec![Arc::new(geom), Arc::new(bbox)])
                    .unwrap();
            w.write(&batch).unwrap();
        }
        w.close().unwrap();
        path
    }

    #[test]
    fn prunes_row_groups_by_bbox_statistics() {
        // Three single-row row groups: Switzerland, New York, Tokyo.
        let path = write_bbox_file("prune", &[(8.0, 47.0), (-74.0, 40.7), (139.7, 35.7)]);
        let gp = GeoParquet::open(&path).unwrap();
        assert_eq!(gp.num_row_groups(), 3);

        // A Swiss bbox keeps only the first row group.
        let rgs = gp.row_groups_intersecting((5.9, 45.8, 10.5, 47.9));
        assert_eq!(rgs, vec![0]);
        assert_eq!(gp.num_rows(&rgs), 1);

        // Streaming those row groups yields just the Swiss point.
        let feats: Vec<Feature> =
            gp.features(rgs, &[]).unwrap().collect::<Result<_, _>>().unwrap();
        std::fs::remove_file(&path).ok();
        assert_eq!(feats.len(), 1);
        assert_eq!(feats[0].geometry, Geometry::Point(Point::new(8.0, 47.0)));
    }

    #[test]
    fn keeps_all_row_groups_without_bbox_column() {
        let p = wkb_point(1.0, 2.0);
        let geom = BinaryArray::from(vec![Some(p.as_slice())]);
        let schema = Schema::new(vec![Field::new("geometry", DataType::Binary, true)]);
        let batch = RecordBatch::try_new(Arc::new(schema), vec![Arc::new(geom)]).unwrap();
        let path = temp_parquet("nobbox");
        {
            let file = File::create(&path).unwrap();
            let mut w = ArrowWriter::try_new(file, batch.schema(), None).unwrap();
            w.write(&batch).unwrap();
            w.close().unwrap();
        }
        let gp = GeoParquet::open(&path).unwrap();
        let rgs = gp.row_groups_intersecting((100.0, 100.0, 101.0, 101.0));
        std::fs::remove_file(&path).ok();
        assert_eq!(rgs, vec![0], "no bbox column → keep everything");
    }

    #[test]
    fn streaming_matches_materialized_read() {
        let path = write_bbox_file("stream", &[(1.0, 1.0), (2.0, 2.0), (3.0, 3.0)]);
        let gp = GeoParquet::open(&path).unwrap();
        let all: Vec<Feature> = gp
            .features((0..gp.num_row_groups()).collect(), &[])
            .unwrap()
            .collect::<Result<_, _>>()
            .unwrap();
        let materialized = gp.read_features(&[]).unwrap();
        std::fs::remove_file(&path).ok();
        assert_eq!(all.len(), 3);
        assert_eq!(all.len(), materialized.len());
        for (a, b) in all.iter().zip(&materialized) {
            assert_eq!(a.geometry, b.geometry);
        }
    }

    // --- Real-data checks (run with `cargo test -- --ignored`). They exercise
    // nested struct paths and real WKB against the repo's sample files. ---

    #[test]
    #[ignore = "requires repo sample data under ../data"]
    fn reads_real_naturalearth_land() {
        let feats = read_naturalearth("../data/naturalearth/land.parquet").unwrap();
        assert!(!feats.is_empty());
        assert!(feats
            .iter()
            .all(|f| matches!(f.geometry, Geometry::Polygon(_) | Geometry::MultiPolygon(_))));
    }

    #[test]
    #[ignore = "requires repo sample data under ../data"]
    fn reads_real_overture_bathymetry_cartography() {
        let feats = read_overture("../data/overture-globe/bathymetry.parquet").unwrap();
        assert!(!feats.is_empty());
        // The nested cartography.sort_key path resolves to a real (non-null)
        // value. (min_zoom/max_zoom happen to be null in this theme.)
        assert!(feats
            .iter()
            .any(|f| f.properties.iter().any(|(k, _)| k == "cartography.sort_key")));
    }

    #[test]
    #[ignore = "requires repo sample data under ../data"]
    fn prunes_real_overture_row_groups() {
        let gp = GeoParquet::open("../data/overture-ch/water.parquet").unwrap();
        let world = gp.row_groups_intersecting((-180.0, -90.0, 180.0, 90.0));
        assert_eq!(world.len(), gp.num_row_groups(), "world bbox keeps all row groups");
        let nothing = gp.row_groups_intersecting((-179.0, -89.0, -178.0, -88.0));
        assert!(
            nothing.len() < gp.num_row_groups() || gp.num_row_groups() == 1,
            "an empty-ocean bbox should prune row groups when there are several"
        );
    }
}
