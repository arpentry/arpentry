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
