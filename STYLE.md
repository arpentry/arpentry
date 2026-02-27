# C Style Guide

Follows the design principles in `DESIGN.md`.

## Rules

1. **Deep modules, not shallow ones.** A `struct` and a few functions that hide complexity behind a small API. No unnecessary layers — but the right abstraction is worth its weight.
2. **One module = one `.h` + one `.c`.** Self-contained. Design decisions encapsulated behind one interface.
3. **Functions are short, names are short, files are few.** Brevity everywhere except comments that explain *why*.
4. **Design errors out of existence.** Prefer APIs where error conditions cannot arise. When unavoidable, use return values: `NULL`, `-1`, `false`, or an error code.
5. **Own your memory.** Symmetric `create`/`free`, `sizeof(*ptr)`, single-allocation tricks.

## File Organization

One `.h` + `.c` pair per module. Headers are minimal: opaque types and function prototypes. Hide struct bodies in the `.c` file.

Header guards use the `ARPENTRY_MODULE_H` pattern:

```c
#ifndef ARPENTRY_CAMERA_H
#define ARPENTRY_CAMERA_H
...
#endif /* ARPENTRY_CAMERA_H */
```

## Naming

All public symbols use the `arpt_` prefix.

| Element | Convention | Examples |
|---|---|---|
| Public functions | `arpt_module_verb` | `arpt_encode`, `arpt_camera_create`, `arpt_geodetic_to_ecef` |
| Static helpers | Short, no prefix | `bucket_at`, `grow`, `cancel_animation` |
| Opaque types | `typedef struct arpt_foo arpt_foo;` | `arpt_camera`, `arpt_renderer` |
| Value types | `typedef struct { ... } arpt_foo;` | `arpt_dvec3`, `arpt_mat4`, `arpt_bounds_t` |
| Macros / constants | `ARPT_UPPER_SNAKE` or `UPPER_SNAKE` | `ARPT_WGS84_A`, `ARPT_EXTENT`, `CAM_MIN_ALT` |
| Variables | Short, lowercase | `n`, `len`, `buf`, `i` |

## Types

Two type patterns:

- **Opaque types** — declared in header, defined in `.c`. Allocated with `arpt_foo_create()`, freed with `arpt_foo_free()`. Caller never sees struct internals.

```c
/* camera.h */
typedef struct arpt_camera arpt_camera;
arpt_camera *arpt_camera_create(void);
void arpt_camera_free(arpt_camera *cam);
```

- **Value types** — small, transparent structs for math and data. Defined directly in headers, often with `static inline` operations.

```c
typedef struct { double x, y, z; } arpt_dvec3;
typedef struct { float m[16]; } arpt_mat4;
```

Dual-precision convention: `arpt_vec3` / `arpt_mat4` (float32, GPU upload) and `arpt_dvec3` / `arpt_dmat4` (float64, CPU precision). Convert at the boundary with `arpt_dmat4_to_mat4()`.

## Functions

10–40 lines typical, 80 max. One function, one job. Prefer small `static` helpers. Early returns for guard clauses:

```c
arpt_camera *arpt_camera_create(void) {
    arpt_camera *cam = calloc(1, sizeof(*cam));
    if (!cam) return NULL;
    cam->altitude = 10000000.0;
    return cam;
}
```

Performance-critical small functions (math, quantization) go as `static inline` in headers:

```c
static inline double arpt_dequantize(uint16_t q) {
    return ((double)q - 16384.0) / 32768.0;
}
```

## Error Handling

Design errors out of existence first: idempotent operations, wide input ranges, absorbed edge cases. When unavoidable:

| Context | Pattern |
|---|---|
| Constructors | Return `NULL` on failure |
| Operations | Return `bool` or error code |
| Complex cleanup | `goto cleanup` |

```c
int save_file(const char *path, const char *buf, size_t len) {
    int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd == -1) goto err;
    if (write(fd, buf, len) != (ssize_t)len) goto err;
    close(fd);
    return 0;
err:
    if (fd != -1) close(fd);
    return -1;
}
```

## Memory

| Guideline | Detail |
|---|---|
| `sizeof(*ptr)` not `sizeof(Type)` | Survives type changes |
| Symmetric `create`/`free` | Every `arpt_foo_create` has an `arpt_foo_free` |
| Single allocation | Struct + inline data in one `malloc` / `calloc` call |

## API Design

Pull complexity downward — make the caller's life easy. Design interfaces that are somewhat general-purpose without over-engineering.

| Pattern | Description |
|---|---|
| Opaque types | Header declares `typedef struct arpt_foo arpt_foo;` — body in `.c` |
| `const` correctness | Read-only params are always `const` |
| Callback iteration | `bool (*iter)(const void *item, void *udata)` |
| Borrowed pointers | Return `const void *` — caller does not own |

## Comments

Explain *why*, not *what*. Public API uses `/** */` doc comments with `@param` / `@return` tags. Inline comments only for non-obvious logic or edge cases.

```c
/**
 * Decode .arpt wire bytes into a verified FlatBuffer.
 *
 * @param data     Compressed .arpt data.
 * @param size     Size of compressed data in bytes.
 * @param out      Receives malloc'd FlatBuffer (caller must free).
 * @param out_size Receives decompressed size in bytes.
 * @return true on success.
 */
bool arpt_decode(const uint8_t *data, size_t size,
                 uint8_t **out, size_t *out_size);
```

## Formatting

| Aspect | Convention |
|---|---|
| Indentation | 4 spaces |
| Braces | K&R |
| Short guards | No braces: `if (!ptr) return NULL;` |
| Pointer `*` | Attached to variable: `void *ptr` |
| Line length | ~80 characters |

## Data Structures

| Pattern | Detail |
|---|---|
| Power-of-two sizes + bitmask | Fast modulo via `& (size - 1)` |
| Flexible array members | `char data[]` at end of struct |
| SoA layout | Parallel arrays for cache-friendly access |
| Items stored inline | By value, not by pointer |

## Tests

Tests use the [Unity](https://github.com/ThrowTheSwitch/Unity) framework. One test file per module, `test_` prefix on all functions.

```c
#include "unity.h"
#include "globe.h"

void setUp(void) {}
void tearDown(void) {}

void test_ecef_roundtrip(void) {
    arpt_dvec3 ecef = arpt_geodetic_to_ecef(lon, lat, alt);
    double lon_out, lat_out, alt_out;
    arpt_ecef_to_geodetic(ecef, &lon_out, &lat_out, &alt_out);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, lon, lon_out);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_ecef_roundtrip);
    return UNITY_END();
}
```

## Vendored Code

External code (e.g., `hashmap.c`, `buf.c`) is vendored directly into the source tree with a comment noting the origin and any modifications. Vendored code follows its own style — do not reformat it.
