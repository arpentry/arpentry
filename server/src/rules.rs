//! Overture linearly-referenced road attributes, reduced to per-segment
//! scalars.
//!
//! `width_rules`, `road_surface`, and `access_restrictions` share
//! `level_rules`' shape — a `list<struct<…, between>>` where `between` is the
//! fractional `[start, end]` of the segment a rule covers (see
//! [`crate::levels`]). The paint path needs one value per segment, not runs,
//! so these reducers resolve each column to its *dominant* value: the one
//! covering the largest fraction of the segment, the same resolution the
//! structure sweep applies to level runs. Full cross-section runs are a later
//! milestone (docs/ROADS.md §6.3).

use arrow::array::{Array, Float64Array, LargeStringArray, ListArray, StringArray, StructArray};

/// Smallest coverage fraction that can win dominance; degenerate rules
/// (inverted or empty `between`) never beat a real one.
const EPS: f64 = 1e-9;

/// The dominant mapped carriageway width in metres from a `width_rules` cell,
/// or `None` for a null cell, an unexpected shape, or no positive width — the
/// caller falls back to the class prior either way.
pub fn dominant_width_m(array: &dyn Array, row: usize) -> Option<f64> {
    let (st, between) = rule_list(array, row)?;
    let values = st.column_by_name("value")?.as_any().downcast_ref::<Float64Array>()?;
    let mut best: Option<(f64, f64)> = None; // (coverage, width)
    for i in 0..st.len() {
        if values.is_null(i) {
            continue;
        }
        let w = values.value(i);
        if !w.is_finite() || w <= 0.0 {
            continue;
        }
        let c = coverage(&between, i);
        if c > EPS && best.is_none_or(|(bc, _)| c > bc) {
            best = Some((c, w));
        }
    }
    best.map(|(_, w)| w)
}

/// The dominant surface material from a `road_surface` cell (coverage summed
/// per value, so `paved [0,.4] + paved [.6,1]` beats `gravel [.4,.6]`), or
/// `None` when nothing is mapped.
pub fn dominant_surface(array: &dyn Array, row: usize) -> Option<String> {
    let (st, between) = rule_list(array, row)?;
    let values = st.column_by_name("value")?;
    let mut acc: Vec<(&str, f64)> = Vec::new();
    for i in 0..st.len() {
        let Some(s) = str_at(values.as_ref(), i) else {
            continue;
        };
        let c = coverage(&between, i);
        if c <= EPS {
            continue;
        }
        match acc.iter_mut().find(|(v, _)| *v == s) {
            Some((_, total)) => *total += c,
            None => acc.push((s, c)),
        }
    }
    let (best, _) = acc.into_iter().max_by(|a, b| a.1.total_cmp(&b.1))?;
    Some(best.to_string())
}

/// Whether an `access_restrictions` cell marks the segment one-way: some rule
/// denies travel against a heading, unconditionally. A denial qualified by
/// mode, vehicle, purpose, or time (a bus lane, an HGV ban) restricts *who*
/// may pass, not the carriageway's direction, and is ignored.
pub fn is_oneway(array: &dyn Array, row: usize) -> bool {
    oneway_inner(array, row).unwrap_or(false)
}

fn oneway_inner(array: &dyn Array, row: usize) -> Option<bool> {
    let (st, _) = rule_list(array, row)?;
    let access = st.column_by_name("access_type")?;
    let when = st.column_by_name("when")?.as_any().downcast_ref::<StructArray>()?;
    let heading = when.column_by_name("heading")?;
    for i in 0..st.len() {
        if str_at(access.as_ref(), i) != Some("denied") || when.is_null(i) {
            continue;
        }
        if str_at(heading.as_ref(), i).is_some() && !qualified(when, i) {
            return Some(true);
        }
    }
    Some(false)
}

/// Whether a `when` condition carries any qualifier beyond `heading` — a
/// non-null scalar, or a non-empty list, in any of its other fields.
fn qualified(when: &StructArray, i: usize) -> bool {
    when.fields().iter().zip(when.columns()).any(|(field, col)| {
        field.name() != "heading"
            && !col.is_null(i)
            && col.as_any().downcast_ref::<ListArray>().is_none_or(|l| !l.value(i).is_empty())
    })
}

/// Downcasts one cell of a rules column to its struct rows and their
/// `between` column. `None` for a null cell or an unexpected shape.
fn rule_list(array: &dyn Array, row: usize) -> Option<(StructArray, ListArray)> {
    if array.is_null(row) {
        return None;
    }
    let list = array.as_any().downcast_ref::<ListArray>()?;
    let rules = list.value(row);
    let st = rules.as_any().downcast_ref::<StructArray>()?.clone();
    let between = st.column_by_name("between")?.as_any().downcast_ref::<ListArray>()?.clone();
    Some((st, between))
}

/// Fraction of the segment rule `i` covers: its clamped `between` span, or the
/// whole segment when `between` is absent.
fn coverage(between: &ListArray, i: usize) -> f64 {
    if between.is_null(i) {
        return 1.0;
    }
    let b = between.value(i);
    match b.as_any().downcast_ref::<Float64Array>() {
        Some(a) if a.len() >= 2 => {
            let (lo, hi) = (a.value(0), a.value(1));
            (lo.max(hi).clamp(0.0, 1.0) - lo.min(hi).clamp(0.0, 1.0)).max(0.0)
        }
        _ => 1.0,
    }
}

/// Reads a string cell from a Utf8 or LargeUtf8 array.
fn str_at(array: &dyn Array, i: usize) -> Option<&str> {
    if array.is_null(i) {
        return None;
    }
    let any = array.as_any();
    if let Some(a) = any.downcast_ref::<StringArray>() {
        return Some(a.value(i));
    }
    if let Some(a) = any.downcast_ref::<LargeStringArray>() {
        return Some(a.value(i));
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use arrow::array::{ArrayRef, Float64Array, StringArray};
    use arrow::buffer::{OffsetBuffer, ScalarBuffer};
    use arrow::datatypes::{DataType, Field, Fields};
    use std::sync::Arc;

    /// Builds a `between` list column from optional `[start, end]` pairs.
    fn between_column(spans: &[Option<(f64, f64)>]) -> ArrayRef {
        let mut values = Vec::new();
        let mut offsets = vec![0i32];
        let mut nulls = Vec::new();
        for span in spans {
            match span {
                Some((lo, hi)) => {
                    values.push(*lo);
                    values.push(*hi);
                    nulls.push(true);
                }
                None => nulls.push(false),
            }
            offsets.push(values.len() as i32);
        }
        Arc::new(ListArray::new(
            Arc::new(Field::new("item", DataType::Float64, true)),
            OffsetBuffer::new(ScalarBuffer::from(offsets)),
            Arc::new(Float64Array::from(values)),
            Some(arrow::buffer::NullBuffer::from(nulls)),
        ))
    }

    /// Wraps per-rule columns into a one-row `list<struct<…>>` cell.
    fn one_row(columns: Vec<(&str, ArrayRef)>) -> ListArray {
        let fields = Fields::from(
            columns
                .iter()
                .map(|(name, arr)| Field::new(*name, arr.data_type().clone(), true))
                .collect::<Vec<_>>(),
        );
        let len = columns.first().map_or(0, |(_, a)| a.len());
        let structs =
            StructArray::new(fields.clone(), columns.into_iter().map(|(_, a)| a).collect(), None);
        ListArray::new(
            Arc::new(Field::new("item", DataType::Struct(fields), true)),
            OffsetBuffer::new(ScalarBuffer::from(vec![0, len as i32])),
            Arc::new(structs),
            None,
        )
    }

    #[test]
    fn width_takes_the_longest_rule() {
        let outer = one_row(vec![
            ("value", Arc::new(Float64Array::from(vec![4.0, 9.0])) as ArrayRef),
            ("between", between_column(&[Some((0.0, 0.2)), Some((0.2, 1.0))])),
        ]);
        assert_eq!(dominant_width_m(&outer, 0), Some(9.0));
    }

    #[test]
    fn width_whole_segment_beats_partial_and_junk_is_skipped() {
        // No `between` = whole segment; zero and negative widths are noise.
        let outer = one_row(vec![
            ("value", Arc::new(Float64Array::from(vec![0.0, 6.5, 20.0])) as ArrayRef),
            ("between", between_column(&[None, None, Some((0.4, 0.6))])),
        ]);
        assert_eq!(dominant_width_m(&outer, 0), Some(6.5));
        // All-junk cell → None (the caller keeps the prior).
        let junk = one_row(vec![
            ("value", Arc::new(Float64Array::from(vec![-1.0])) as ArrayRef),
            ("between", between_column(&[None])),
        ]);
        assert_eq!(dominant_width_m(&junk, 0), None);
    }

    #[test]
    fn surface_coverage_sums_per_value() {
        // paved 0.4 + 0.4 beats gravel 0.5.
        let outer = one_row(vec![
            (
                "value",
                Arc::new(StringArray::from(vec!["paved", "gravel", "paved"])) as ArrayRef,
            ),
            (
                "between",
                between_column(&[Some((0.0, 0.4)), Some((0.4, 0.9)), Some((0.6, 1.0))]),
            ),
        ]);
        assert_eq!(dominant_surface(&outer, 0), Some("paved".to_string()));
    }

    /// Builds an `access_restrictions` cell with one rule per entry:
    /// `(access_type, heading, mode)`.
    fn restrictions(rules: &[(&str, Option<&str>, Option<&str>)]) -> ListArray {
        let access: ArrayRef =
            Arc::new(StringArray::from(rules.iter().map(|r| Some(r.0)).collect::<Vec<_>>()));
        let heading: ArrayRef =
            Arc::new(StringArray::from(rules.iter().map(|r| r.1).collect::<Vec<_>>()));
        // `mode` as list<utf8>: one-element list when set, null otherwise.
        let mut mode_values = Vec::new();
        let mut mode_offsets = vec![0i32];
        let mut mode_nulls = Vec::new();
        for r in rules {
            if let Some(m) = r.2 {
                mode_values.push(m.to_string());
                mode_nulls.push(true);
            } else {
                mode_nulls.push(false);
            }
            mode_offsets.push(mode_values.len() as i32);
        }
        let mode: ArrayRef = Arc::new(ListArray::new(
            Arc::new(Field::new("item", DataType::Utf8, true)),
            OffsetBuffer::new(ScalarBuffer::from(mode_offsets)),
            Arc::new(StringArray::from(mode_values)),
            Some(arrow::buffer::NullBuffer::from(mode_nulls)),
        ));
        let when_fields = Fields::from(vec![
            Field::new("heading", DataType::Utf8, true),
            Field::new("mode", mode.data_type().clone(), true),
        ]);
        let when: ArrayRef =
            Arc::new(StructArray::new(when_fields, vec![heading, mode], None));
        one_row(vec![
            ("access_type", access),
            ("when", when),
            ("between", between_column(&vec![None; rules.len()])),
        ])
    }

    /// Exercises the reducers against the real Overture extract's Arrow
    /// shapes (Utf8 widths, nested `when` structs). Run with `--ignored`.
    #[test]
    #[ignore = "requires repo sample data under ../data"]
    fn reduces_real_overture_segments() {
        let gp = crate::geoparquet::GeoParquet::open("../data/overture-ch/segment.parquet")
            .unwrap();
        let row_groups: Vec<usize> = (0..gp.num_row_groups().min(4)).collect();
        let feats: Vec<crate::geoparquet::Feature> = gp
            .features(row_groups, &["class", "width_rules", "road_surface", "access_restrictions"])
            .unwrap()
            .collect::<Result<_, _>>()
            .unwrap();
        let count = |key: &str| {
            feats.iter().filter(|f| f.properties.iter().any(|(k, _)| k == key)).count()
        };
        let (widths, surfaces, oneways) =
            (count("width_rules"), count("road_surface"), count("oneway"));
        eprintln!(
            "{} features: {widths} widths, {surfaces} surfaces, {oneways} oneways",
            feats.len()
        );
        // Surface (~35 %) and oneway (~7 %) coverage cannot plausibly miss a
        // 4-row-group sample; widths (~3 %) are looser but expected too.
        assert!(surfaces > 0, "no road_surface reduced from real data");
        assert!(oneways > 0, "no oneway reduced from real data");
        assert!(widths > 0, "no width_rules reduced from real data");
    }

    #[test]
    fn oneway_is_an_unqualified_heading_denial() {
        // OSM `oneway=yes` → denied against the backward heading.
        let oneway = restrictions(&[("denied", Some("backward"), None)]);
        assert!(is_oneway(&oneway, 0));
        // A bus-only contraflow ban restricts who, not the direction.
        let bus_lane = restrictions(&[("denied", Some("backward"), Some("bus"))]);
        assert!(!is_oneway(&bus_lane, 0));
        // A denial with no heading (private road) is not a oneway.
        let private = restrictions(&[("denied", None, None)]);
        assert!(!is_oneway(&private, 0));
        // An allowed rule never marks one.
        let allowed = restrictions(&[("allowed", Some("forward"), None)]);
        assert!(!is_oneway(&allowed, 0));
        // A qualified denial plus a real oneway rule → still one-way.
        let both = restrictions(&[
            ("denied", Some("backward"), Some("hgv")),
            ("denied", Some("backward"), None),
        ]);
        assert!(is_oneway(&both, 0));
    }
}
