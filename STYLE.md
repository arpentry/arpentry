# C Style Guide

Follows `DESIGN.md`. When in doubt, read the existing code.

## Principles

1. **Deep modules.** Hide complexity behind a small API. One `.h` + `.c` per module.
2. **Design errors out of existence.** Idempotent ops, wide input ranges, absorbed edge cases. When unavoidable: `NULL`, `bool`, or `goto cleanup`.
3. **Own your memory.** Every `arpt_foo_create` has an `arpt_foo_free`. Use `sizeof(*ptr)`.

## Naming

All public symbols use the `arpt_` prefix. Functions: `arpt_module_verb`. Macros: `ARPT_UPPER_SNAKE`. Static helpers: short, no prefix. Variables: short, lowercase.

## Types

Opaque types hide internals. Value types are transparent.

```c
/* Opaque — declared in header, defined in .c */
typedef struct arpt_camera arpt_camera;
arpt_camera *arpt_camera_create(void);
void arpt_camera_free(arpt_camera *cam);

/* Value — small, transparent, often with static inline ops */
typedef struct { double x, y, z; } arpt_dvec3;
```

Dual precision: `arpt_vec3` / `arpt_mat4` (float32, GPU) and `arpt_dvec3` / `arpt_dmat4` (float64, CPU). Convert at the boundary.

## Functions

Short (10–40 lines). Early returns for guards. Performance-critical helpers go `static inline` in headers.

## Comments

Explain *why*, not *what*.

```c
/* Decompress and verify an .arpt tile. Caller frees *out. */
bool arpt_decode(const uint8_t *data, size_t size,
                 uint8_t **out, size_t *out_size);
```

## Formatting

4 spaces. K&R braces. `void *ptr`. ~80 columns. No braces on single-line guards: `if (!ptr) return NULL;`
