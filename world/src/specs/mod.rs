//! Specifications that span several steps, written as checks on synthetic
//! specimens: what the world must be, asserted where a step's own tests
//! cannot see it.
//!
//! - [`spans`]: where a way is a deck, a bore or on the ground is a
//!   consequence of the solved heights, not of the annotation alone.
//! - [`ground`]: one arrangement, one height per vertex, and a step is an
//!   edge that is declared.
//! - [`junction`]: pieces merge into one surface where they meet and stay
//!   apart where they cross.
//!
//! A check whose rule is not built yet is `#[ignore]`d with the rule it
//! names, so `cargo test -- --ignored` is the list of what is still open.

mod ground;
mod junction;
mod spans;
