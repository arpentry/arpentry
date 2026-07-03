//! Overture nested-column parsers used by the assemble stage.
//!
//! Overture stores a segment's graph topology as `connectors`, a
//! `list<struct<connector_id: utf8, at: float64>>`: the shared node ids the
//! segment passes through and where along the segment (fraction 0..1) each
//! sits. Two segments sharing an endpoint connector are physically continuous
//! there — the signal corridor joining runs on. Parsed in the same defensive
//! style as `levels::parse`: any shape mismatch yields "no connectors", never
//! an error.

use arrow::array::{Array, Float64Array, LargeStringArray, ListArray, StringArray, StructArray};

use crate::scene::source_hash;

/// One connector reference: the hashed connector id and its fractional
/// position along the segment.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Connector {
    pub id: u64,
    pub at: f64,
}

/// Parses an Overture `connectors` cell. Returns empty for a null cell or a
/// shape other than `list<struct<connector_id, at>>`.
pub fn parse_connectors(array: &dyn Array, row: usize) -> Vec<Connector> {
    parse_inner(array, row).unwrap_or_default()
}

fn parse_inner(array: &dyn Array, row: usize) -> Option<Vec<Connector>> {
    if array.is_null(row) {
        return Some(Vec::new());
    }
    let list = array.as_any().downcast_ref::<ListArray>()?;
    let cell = list.value(row);
    let st = cell.as_any().downcast_ref::<StructArray>()?;
    let ids = st.column_by_name("connector_id")?;
    let ats = st.column_by_name("at")?.as_any().downcast_ref::<Float64Array>()?;

    let id_at = |i: usize| -> Option<&str> {
        if ids.is_null(i) {
            return None;
        }
        if let Some(a) = ids.as_any().downcast_ref::<StringArray>() {
            return Some(a.value(i));
        }
        if let Some(a) = ids.as_any().downcast_ref::<LargeStringArray>() {
            return Some(a.value(i));
        }
        None
    };

    let mut out = Vec::with_capacity(st.len());
    for i in 0..st.len() {
        let Some(id) = id_at(i) else { continue };
        let at = if ats.is_null(i) { continue } else { ats.value(i) };
        out.push(Connector { id: source_hash(id), at });
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use arrow::array::ArrayRef;
    use arrow::buffer::{OffsetBuffer, ScalarBuffer};
    use arrow::datatypes::{DataType, Field, Fields};
    use std::sync::Arc;

    /// Builds a one-row `connectors` list of `{connector_id, at}` structs.
    fn connectors_array(entries: &[(&str, f64)]) -> ListArray {
        let ids: ArrayRef =
            Arc::new(StringArray::from(entries.iter().map(|(s, _)| *s).collect::<Vec<_>>()));
        let ats: ArrayRef =
            Arc::new(Float64Array::from(entries.iter().map(|(_, a)| *a).collect::<Vec<_>>()));
        let fields = Fields::from(vec![
            Field::new("connector_id", DataType::Utf8, true),
            Field::new("at", DataType::Float64, true),
        ]);
        let structs = StructArray::new(fields.clone(), vec![ids, ats], None);
        ListArray::new(
            Arc::new(Field::new("item", DataType::Struct(fields), true)),
            OffsetBuffer::new(ScalarBuffer::from(vec![0, entries.len() as i32])),
            Arc::new(structs),
            None,
        )
    }

    #[test]
    fn parses_ids_and_positions() {
        let arr = connectors_array(&[("a", 0.0), ("b", 0.5), ("c", 1.0)]);
        let got = parse_connectors(&arr, 0);
        assert_eq!(got.len(), 3);
        assert_eq!(got[0].id, source_hash("a"));
        assert_eq!(got[2].at, 1.0);
    }

    #[test]
    fn empty_for_wrong_shape() {
        let arr = Float64Array::from(vec![1.0]);
        assert!(parse_connectors(&arr, 0).is_empty());
    }
}
