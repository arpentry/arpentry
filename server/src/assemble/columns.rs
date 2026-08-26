//! Overture nested-column parsers used by the assemble stage.
//!
//! Overture stores a segment's graph topology as `connectors`, a
//! `list<struct<connector_id: utf8, at: float64>>`: the shared node ids the
//! segment passes through and where along the segment (fraction 0..1) each
//! sits. Two segments sharing an endpoint connector are physically continuous
//! there — the signal corridor joining runs on. Parsed in the same defensive
//! style as `levels::parse`: any shape mismatch yields "no connectors", never
//! an error.
//!
//! [`parse_subclass_rules`] reads the other linearly-referenced column the
//! pedestrian model depends on. See `docs/SOURCES.md` §2: Overture populates
//! the scalar `subclass` only when one value covers the whole segment, and
//! leaves it **null** the moment any rule is partial — so a footway that is a
//! sidewalk for its first 60 % and a crossing for the rest reads, to anyone
//! looking at the scalar alone, as an anonymous footway with no subclass at
//! all. In the Montreux zone that is 45 % of the crossings and a quarter of
//! the sidewalk length.

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

/// One rule from `subclass_rules`: a subclass value over the fractional span
/// `[start, end]` (0..1) of a segment. A rule with no `between` covers the
/// whole segment and parses as `0.0..1.0`, which is exactly the case where
/// Overture also sets the scalar `subclass` — so the common segment yields one
/// full-length run and nothing downstream has to change shape for it.
#[derive(Debug, Clone, PartialEq)]
pub struct SubclassRun {
    pub start: f64,
    pub end: f64,
    pub value: String,
}

/// Parses an Overture `subclass_rules` cell into its runs, in source order.
/// Returns empty for a null cell or a shape other than
/// `list<struct<value, between>>` — every case the caller treats as "the
/// scalar `subclass` is the whole story".
pub fn parse_subclass_rules(array: &dyn Array, row: usize) -> Vec<SubclassRun> {
    parse_subclass_inner(array, row).unwrap_or_default()
}

fn parse_subclass_inner(array: &dyn Array, row: usize) -> Option<Vec<SubclassRun>> {
    if array.is_null(row) {
        return Some(Vec::new());
    }
    let list = array.as_any().downcast_ref::<ListArray>()?;
    let cell = list.value(row);
    let st = cell.as_any().downcast_ref::<StructArray>()?;
    let values = st.column_by_name("value")?;
    let between = st.column_by_name("between")?.as_any().downcast_ref::<ListArray>()?;

    let value_at = |i: usize| -> Option<&str> {
        if values.is_null(i) {
            return None;
        }
        if let Some(a) = values.as_any().downcast_ref::<StringArray>() {
            return Some(a.value(i));
        }
        if let Some(a) = values.as_any().downcast_ref::<LargeStringArray>() {
            return Some(a.value(i));
        }
        None
    };

    let mut out = Vec::with_capacity(st.len());
    for i in 0..st.len() {
        let Some(value) = value_at(i) else { continue };
        // A null or malformed `between` is Overture's "whole segment".
        let (start, end) = span_at(between, i).unwrap_or((0.0, 1.0));
        if end - start <= SPAN_EPS {
            continue;
        }
        out.push(SubclassRun { start, end, value: value.to_string() });
    }
    Some(out)
}

/// Reads one `between` cell as an ordered `[start, end]` fraction pair,
/// clamped to 0..1. `None` when the cell is null or is not a 2-element list.
fn span_at(between: &ListArray, i: usize) -> Option<(f64, f64)> {
    if between.is_null(i) {
        return None;
    }
    let pair = between.value(i);
    let pair = pair.as_any().downcast_ref::<Float64Array>()?;
    if pair.len() != 2 || pair.is_null(0) || pair.is_null(1) {
        return None;
    }
    let (a, b) = (pair.value(0), pair.value(1));
    if !a.is_finite() || !b.is_finite() {
        return None;
    }
    let (lo, hi) = if a <= b { (a, b) } else { (b, a) };
    Some((lo.clamp(0.0, 1.0), hi.clamp(0.0, 1.0)))
}

/// Smallest fraction treated as a non-empty run; anything shorter would cut a
/// walk line into a piece with no length to attach or draw.
pub(crate) const SPAN_EPS: f64 = 1e-9;

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
