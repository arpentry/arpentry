# Generated FlatBuffers bindings

`*_generated.rs` here are produced by `flatc` from `../../../schemas/*.fbs` and
are **checked in** — building the crate needs only the `flatbuffers` runtime,
not `flatc`. Regenerate after a schema change, from the crate root:

```sh
flatc --rust --filename-suffix _generated -o src/fb \
    ../schemas/tile.fbs ../schemas/tileset.fbs ../schemas/style.fbs ../schemas/model.fbs
```

Generated with flatc 24.3.25 (keep the `flatbuffers` dependency in `Cargo.toml`
compatible with the `flatc` used here). The output uses the standard FlatBuffers
layout — file identifier at bytes 4..8 — matching the C (flatcc) and web clients
(FORMAT.md §7).

## Required post-generation patch: struct alignment

flatc 24.3.25 emits every struct as `#[repr(transparent)]` (Rust alignment 1)
and does **not** override `Push::alignment()`, so the `flatbuffers` builder
aligns structs to 1. Structs whose true alignment is larger than 1 then land at
the wrong offset, and the C client's stricter **flatcc verifier rejects the
buffer** (`table_field_not_aligned`, rc=12) — e.g. the `.arpi` `Tileset.bounds`
(an `f64` struct, alignment 8). The Rust reader is unaffected because
`follow_cast_ref` requires alignment-1 structs and reads fields by byte copy.

After regenerating, re-apply the `alignment()` override to every struct whose
"aligned to N" comment has N > 1 — keep `#[repr(transparent)]` (the reader needs
it) and add to each `impl flatbuffers::Push`:

```rust
    #[inline]
    fn alignment() -> flatbuffers::PushAlignment {
        flatbuffers::PushAlignment::new(N) // N from the "// struct X, aligned to N" comment
    }
```

Currently patched: `Bounds` (8), `ElevationRange` (8) in `tileset_generated.rs`;
`Part` (4), `Property` (4) in `tile_generated.rs`. `Color`/`RGBA` are alignment 1
and need no change. (Upgrading flatc to a version that emits the override removes
the need for this patch.)
