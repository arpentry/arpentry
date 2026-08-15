//! Typed property values (FORMAT.md §4).
//!
//! Mirrors the four `.arpt` value types. All integers are `i64`, all floats
//! `f64` — there are no separate uint or float32 types.

/// A typed feature-property value.
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    String(String),
    Int(i64),
    Double(f64),
    Bool(bool),
}

/// A feature's properties, as every stage carries them.
pub type Props = [(String, Value)];

/// Reads a string property, or `None` when it is absent or another type.
///
/// Borrowed rather than cloned: most callers compare or parse and throw the
/// result away. There were four hand-rolled copies of this scan (two of them —
/// `assemble::prop_string` and `assemble::water::prop_str` — byte-identical
/// bodies under different names in sibling modules), plus inline `find_map`s in
/// `pipeline` and `synth`.
pub fn str_of<'a>(props: &'a Props, key: &str) -> Option<&'a str> {
    props.iter().find(|(k, _)| k == key).and_then(|(_, v)| match v {
        Value::String(s) => Some(s.as_str()),
        _ => None,
    })
}

/// Reads a numeric property. `Int` and `Double` both answer, because the
/// distinction is a wire-format detail and no caller has ever cared.
pub fn f64_of(props: &Props, key: &str) -> Option<f64> {
    props.iter().find(|(k, _)| k == key).and_then(|(_, v)| match v {
        Value::Double(d) => Some(*d),
        Value::Int(i) => Some(*i as f64),
        _ => None,
    })
}

/// The mapped carriageway width in metres, where the input recorded one.
///
/// `Double` only, deliberately: an integer here would be a different Overture
/// encoding rather than a width, and widening the match would silently change
/// which segments use a measured width instead of their class prior. Read the
/// same way by the assemble stage (for the corridor cross-section) and the
/// marking phase (for where the paint goes), which must agree or the paint
/// lands off the asphalt.
pub fn width_rules_m(props: &Props) -> Option<f64> {
    props.iter().find_map(|(k, v)| match (k.as_str(), v) {
        ("width_rules", Value::Double(w)) => Some(*w),
        _ => None,
    })
}
