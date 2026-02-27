# Design Principles

These shape every module. See `STYLE.md` for code conventions.

## Deep modules

Simple interface, complex implementation. A two-function gesture API that hides coordinate transforms, ray intersection, and globe math behind screen-pixel inputs is better than exposing each step to the caller.

## Pull complexity downward

When there's a choice between harder for the implementer vs. harder for the caller, take the hit in the implementation. A manager that handles level-of-detail selection, async I/O, caching, and eviction internally — exposing only `update` and `draw` — is better than pushing those concerns to the caller.

## Define errors out of existence

Design APIs so error conditions can't arise. Clamp instead of rejecting out-of-range values. Return a zero vector for zero-length normalization. Make operations idempotent. When errors are unavoidable (decompression, allocation), use `bool` return + output pointer.

## Information hiding

Each module owns its internals. Opaque types (`typedef struct arpt_foo arpt_foo`) prevent callers from depending on struct layout. Headers expose functions, not fields.

## One width per role

Don't offer multiple ways to do the same thing. One compression codec. One coordinate extent. One tiling scheme. One geometry representation per topology. This eliminates configuration, branching, and the bugs that come with them.

## Somewhat general-purpose

Design interfaces that serve the current need while being usable in adjacent situations. A ray-ellipsoid intersection function that works for both screen picking and visibility culling is better than two specialized routines.
