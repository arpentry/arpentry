//! GeoParquet reader for the tiler's inputs (TILER.md §geoparquet/overture).
//!
//! Overture Maps and Natural Earth store geometry as a WKB binary column
//! (`geometry`), so this reads the file into Arrow record batches, decodes the
//! WKB column with [`crate::wkb`], and pulls requested attribute columns by
//! name — including dotted paths into nested structs (`bbox.xmin`,
//! `cartography.min_zoom`). No geoarrow dependency is needed.
//!
//! Deep module: callers see `open` + `read_features` (and the Overture/NE
//! conveniences); Arrow downcasting, nesting, and null handling are internal.
//! Requesting an absent column is not an error — it is simply skipped, so one
//! Overture column list works across its per-theme schema variants.

use std::fs::File;
use std::path::Path;

use arrow::array::{
    Array, ArrayRef, BinaryArray, BooleanArray, Float32Array, Float64Array, Int32Array,
    Int64Array, LargeBinaryArray, LargeStringArray, StringArray, StructArray,
};
use arrow::datatypes::DataType;
use arrow::record_batch::RecordBatch;
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use parquet::file::reader::ChunkReader;

use crate::value::Value;
use crate::wkb::{self, WkbError};

/// Default GeoParquet primary geometry column name.
const DEFAULT_GEOMETRY_COLUMN: &str = "geometry";

/// One decoded source feature: geometry plus its non-null requested attributes.
#[derive(Debug, Clone)]
pub struct Feature {
    pub geometry: geo_types::Geometry,
    /// Requested attributes that were present and non-null, in request order.
    pub properties: Vec<(String, Value)>,
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

/// A GeoParquet file loaded into memory as Arrow record batches.
pub struct GeoParquet {
    batches: Vec<RecordBatch>,
    geometry_col: String,
}

impl GeoParquet {
    /// Opens and fully reads a GeoParquet file.
    pub fn open(path: impl AsRef<Path>) -> Result<Self, ReadError> {
        let file = File::open(path)?;
        Ok(GeoParquet {
            batches: collect_batches(file)?,
            geometry_col: DEFAULT_GEOMETRY_COLUMN.to_string(),
        })
    }

    /// Decodes every row into a [`Feature`], pulling the requested attribute
    /// columns (dotted paths descend into nested structs). Rows with a null
    /// geometry are skipped; absent columns are skipped silently.
    pub fn read_features(&self, attr_columns: &[&str]) -> Result<Vec<Feature>, ReadError> {
        let mut out = Vec::new();
        for batch in &self.batches {
            let geom = batch.column_by_name(&self.geometry_col).ok_or_else(|| {
                ReadError::Schema(format!("missing geometry column '{}'", self.geometry_col))
            })?;

            // Resolve attribute columns once per batch (skipping absent ones).
            let resolved: Vec<(&str, ArrayRef)> = attr_columns
                .iter()
                .filter_map(|name| resolve_column(batch, name).map(|a| (*name, a)))
                .collect();

            for row in 0..batch.num_rows() {
                let Some(bytes) = geometry_bytes(geom.as_ref(), row) else {
                    continue;
                };
                let geometry = wkb::parse(bytes)?;
                let mut properties = Vec::new();
                for (name, arr) in &resolved {
                    if let Some(v) = array_value(arr.as_ref(), row) {
                        properties.push(((*name).to_string(), v));
                    }
                }
                out.push(Feature { geometry, properties });
            }
        }
        Ok(out)
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

fn collect_batches<R: ChunkReader + 'static>(reader: R) -> Result<Vec<RecordBatch>, ReadError> {
    let builder = ParquetRecordBatchReaderBuilder::try_new(reader)?;
    let mut batches = Vec::new();
    for batch in builder.build()? {
        batches.push(batch?);
    }
    Ok(batches)
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
    use arrow::datatypes::{Field, Schema};
    use geo_types::{Geometry, Point};
    use parquet::arrow::ArrowWriter;
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
}
