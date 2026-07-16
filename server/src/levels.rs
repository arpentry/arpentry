//! Overture transportation `level_rules` — a linearly-referenced z-order over a
//! road segment.
//!
//! Overture encodes bridges and tunnels not as whole segments but as *spans* of
//! one: a `list<struct<value, between>>` where `value` is the relative vertical
//! level (positive = bridge/elevated deck, negative = tunnel, 0/absent = ground)
//! and `between` is the `[start, end]` fraction of the segment the rule covers.
//! A single motorway segment can run at grade, climb onto a bridge, dive into a
//! tunnel, and surface again — all under one geometry.
//!
//! `level_rules` is not the only structure signal: `road_flags` carries
//! `is_bridge`/`is_tunnel` over the same `between` referencing, and a
//! substantial share of structures have *only* the flag (on the Swiss extract
//! ~6 k bridge and ~10 k tunnel road segments carry `road_flags` but no
//! `level_rules`). [`parse_flags`] reads those into the same [`LevelRun`]s —
//! `is_bridge` as level +1, `is_tunnel` as −1 — so a flagged-only bridge still
//! becomes a corridor structure span instead of a ground-hugging bed diving
//! through the river it crosses.
//!
//! The levels are ordinals, not heights, and their edges are not registered to
//! the terrain — resolving them into corridor-wide structure spans is the
//! assemble stage's job (`assemble::corridors::resolve_spans`). This module
//! only parses the Arrow columns into [`LevelRun`]s.

use arrow::array::{Array, Float64Array, Int32Array, ListArray, StringArray, StructArray};

/// Smallest fraction treated as a non-empty span; cuts closer than this
/// collapse rather than spawning a degenerate run.
const EPS: f64 = 1e-9;

/// One rule from `level_rules`: a constant level over the fractional span
/// `[start, end]` (0..1) of a segment. Ground rules (level 0) are dropped at
/// parse time — ground is the implicit default between and around the runs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LevelRun {
    pub start: f64,
    pub end: f64,
    pub level: i64,
}

/// Parses an Overture `level_rules` cell into its non-ground runs, in source
/// order. Returns empty for a null cell, a non-`list<struct<value, between>>`
/// shape, or an all-ground segment — every case the caller treats as "no
/// structure".
pub fn parse(array: &dyn Array, row: usize) -> Vec<LevelRun> {
    parse_inner(array, row).unwrap_or_default()
}

fn parse_inner(array: &dyn Array, row: usize) -> Option<Vec<LevelRun>> {
    if array.is_null(row) {
        return Some(Vec::new());
    }
    let list = array.as_any().downcast_ref::<ListArray>()?;
    let rules = list.value(row);
    let st = rules.as_any().downcast_ref::<StructArray>()?;
    let values = st.column_by_name("value")?.as_any().downcast_ref::<Int32Array>()?;
    let between = st.column_by_name("between")?.as_any().downcast_ref::<ListArray>()?;

    let mut runs = Vec::new();
    for i in 0..st.len() {
        let level = values.value(i) as i64;
        if level == 0 {
            continue; // ground span — implicit, nothing to split on
        }
        push_run(&mut runs, between, i, level);
    }
    Some(runs)
}

/// Parses an Overture `road_flags` cell into level runs: an `is_bridge` rule
/// becomes a level +1 run, an `is_tunnel` rule level −1, over the rule's
/// `between` span (whole segment when absent). Rules carrying neither flag
/// (`is_link`, `is_covered`, …) contribute nothing. The caller uses these only
/// when `level_rules` said nothing — where both exist, the rules carry real
/// ordinals (stacked decks) and win.
pub fn parse_flags(array: &dyn Array, row: usize) -> Vec<LevelRun> {
    parse_flags_inner(array, row).unwrap_or_default()
}

fn parse_flags_inner(array: &dyn Array, row: usize) -> Option<Vec<LevelRun>> {
    if array.is_null(row) {
        return Some(Vec::new());
    }
    let list = array.as_any().downcast_ref::<ListArray>()?;
    let rules = list.value(row);
    let st = rules.as_any().downcast_ref::<StructArray>()?;
    let values = st.column_by_name("values")?.as_any().downcast_ref::<ListArray>()?;
    let between = st.column_by_name("between")?.as_any().downcast_ref::<ListArray>()?;

    let mut runs = Vec::new();
    for i in 0..st.len() {
        if values.is_null(i) {
            continue;
        }
        let flags = values.value(i);
        let flags = flags.as_any().downcast_ref::<StringArray>()?;
        let mut level = 0i64;
        for k in 0..flags.len() {
            if flags.is_null(k) {
                continue;
            }
            match flags.value(k) {
                // A rule flagged both ways is contradictory; the deck reading
                // at least keeps the road out of the ground.
                "is_bridge" => level = 1,
                "is_tunnel" if level == 0 => level = -1,
                _ => {}
            }
        }
        if level == 0 {
            continue; // no structure flag in this rule
        }
        push_run(&mut runs, between, i, level);
    }
    Some(runs)
}

/// Appends the run for rule `i` at `level`, reading its `between` span — a
/// missing one means the rule covers the whole segment. Degenerate spans are
/// dropped.
fn push_run(runs: &mut Vec<LevelRun>, between: &ListArray, i: usize, level: i64) {
    let (start, end) = if between.is_null(i) {
        (0.0, 1.0)
    } else {
        let b = between.value(i);
        match b.as_any().downcast_ref::<Float64Array>() {
            Some(a) if a.len() >= 2 => (a.value(0), a.value(1)),
            _ => (0.0, 1.0),
        }
    };
    let (lo, hi) = (start.min(end).clamp(0.0, 1.0), start.max(end).clamp(0.0, 1.0));
    if hi - lo > EPS {
        runs.push(LevelRun { start: lo, end: hi, level });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use arrow::array::{ArrayRef, Float64Array, Int32Array, ListArray, StructArray};
    use arrow::buffer::{OffsetBuffer, ScalarBuffer};
    use arrow::datatypes::{DataType, Field, Fields};
    use std::sync::Arc;

    fn run(start: f64, end: f64, level: i64) -> LevelRun {
        LevelRun { start, end, level }
    }

    /// Builds an Overture-shaped `level_rules` array — one cell holding a list of
    /// `{value, between:[start,end]}` structs — and checks [`parse`].
    #[test]
    fn parse_reads_value_and_between() {
        let value: ArrayRef = Arc::new(Int32Array::from(vec![1, 0, -5]));
        // `between` as list<float64>, two endpoints per struct.
        let between_values = Float64Array::from(vec![0.1, 0.4, 0.4, 0.6, 0.6, 0.9]);
        let between_offsets = OffsetBuffer::new(ScalarBuffer::from(vec![0, 2, 4, 6]));
        let between: ArrayRef = Arc::new(ListArray::new(
            Arc::new(Field::new("item", DataType::Float64, true)),
            between_offsets,
            Arc::new(between_values),
            None,
        ));
        let struct_fields = Fields::from(vec![
            Field::new("value", DataType::Int32, true),
            Field::new("between", between.data_type().clone(), true),
        ]);
        let structs = StructArray::new(struct_fields.clone(), vec![value, between], None);
        // One outer row containing all three rules.
        let outer = ListArray::new(
            Arc::new(Field::new("item", DataType::Struct(struct_fields), true)),
            OffsetBuffer::new(ScalarBuffer::from(vec![0, 3])),
            Arc::new(structs),
            None,
        );
        let runs = parse(&outer, 0);
        // The ground rule (value 0) is dropped; the bridge and tunnel remain.
        assert_eq!(runs, vec![run(0.1, 0.4, 1), run(0.6, 0.9, -5)]);
    }

    /// Builds an Overture-shaped `road_flags` array — one cell holding a list
    /// of `{values:[flags], between:[start,end]}` structs — and checks
    /// [`parse_flags`]: bridge → +1, tunnel → −1, other flags nothing, a null
    /// `between` covers the whole segment.
    #[test]
    fn parse_flags_reads_bridge_and_tunnel() {
        use arrow::array::StringArray;
        // Three rules: a mid-segment bridge, a whole-segment link (no
        // structure), and a tunnel with no `between` (whole segment).
        let flag_values = StringArray::from(vec!["is_bridge", "is_link", "is_tunnel"]);
        let values_offsets = OffsetBuffer::new(ScalarBuffer::from(vec![0, 1, 2, 3]));
        let values: ArrayRef = Arc::new(ListArray::new(
            Arc::new(Field::new("item", DataType::Utf8, true)),
            values_offsets,
            Arc::new(flag_values),
            None,
        ));
        let between_values = Float64Array::from(vec![0.2, 0.7, 0.0, 1.0]);
        let between_offsets = OffsetBuffer::new(ScalarBuffer::from(vec![0, 2, 4, 4]));
        // The third rule's `between` is null: whole segment.
        let between_nulls = arrow::buffer::NullBuffer::from(vec![true, true, false]);
        let between: ArrayRef = Arc::new(ListArray::new(
            Arc::new(Field::new("item", DataType::Float64, true)),
            between_offsets,
            Arc::new(between_values),
            Some(between_nulls),
        ));
        let struct_fields = Fields::from(vec![
            Field::new("values", values.data_type().clone(), true),
            Field::new("between", between.data_type().clone(), true),
        ]);
        let structs = StructArray::new(struct_fields.clone(), vec![values, between], None);
        let outer = ListArray::new(
            Arc::new(Field::new("item", DataType::Struct(struct_fields), true)),
            OffsetBuffer::new(ScalarBuffer::from(vec![0, 3])),
            Arc::new(structs),
            None,
        );
        let runs = parse_flags(&outer, 0);
        assert_eq!(runs, vec![run(0.2, 0.7, 1), run(0.0, 1.0, -1)]);
    }

    #[test]
    fn parse_handles_null_cell() {
        let value: ArrayRef = Arc::new(Int32Array::from(vec![1]));
        let between: ArrayRef = Arc::new(ListArray::new(
            Arc::new(Field::new("item", DataType::Float64, true)),
            OffsetBuffer::new(ScalarBuffer::from(vec![0, 0])),
            Arc::new(Float64Array::from(Vec::<f64>::new())),
            None,
        ));
        let struct_fields = Fields::from(vec![
            Field::new("value", DataType::Int32, true),
            Field::new("between", between.data_type().clone(), true),
        ]);
        let structs = StructArray::new(struct_fields.clone(), vec![value, between], None);
        // A single null outer row.
        let nulls = arrow::buffer::NullBuffer::from(vec![false]);
        let outer = ListArray::new(
            Arc::new(Field::new("item", DataType::Struct(struct_fields), true)),
            OffsetBuffer::new(ScalarBuffer::from(vec![0, 1])),
            Arc::new(structs),
            Some(nulls),
        );
        assert!(parse(&outer, 0).is_empty());
    }
}
