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

```c
/* ========================= Public API ========================= */
/* ========================= Internals ========================== */
```

## Naming

| Element | Convention | Examples |
|---|---|---|
| Public functions | `prefix_verb_noun` | `tile_decode`, `mesh_add_part` |
| Static helpers | Short, no prefix | `bucket_at`, `grow` |
| Types | Opaque in header | `struct tile;` |
| Macros / constants | `UPPER_SNAKE_CASE` | `MAX_EXTENT`, `GROW_AT` |
| Variables | Short, lowercase | `n`, `len`, `buf`, `i` |

## Functions

10–40 lines typical, 80 max. One function, one job. Prefer small `static` helpers. Early returns for guard clauses:

```c
tile_t *tile_copy(const tile_t *t) {
    tile_t *copy = malloc(sizeof(*copy));
    if (!copy) return NULL;
    if (!tile_init(copy, t->size)) {
        free(copy);
        return NULL;
    }
    return copy;
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
| Symmetric `create`/`free` | Every `foo_new` has a `foo_free` |
| Single allocation | Struct + inline data in one `malloc` call |

## API Design

Pull complexity downward — make the caller's life easy. Design interfaces that are somewhat general-purpose without over-engineering.

| Pattern | Description |
|---|---|
| Opaque types | Header declares `struct foo;` — body in `.c` |
| `const` correctness | Read-only params are always `const` |
| Callback iteration | `bool (*iter)(const void *item, void *udata)` |
| Borrowed pointers | Return `const void *` — caller does not own |

## Formatting

| Aspect | Convention |
|---|---|
| Indentation | 4 spaces |
| Braces | K&R |
| Short guards | No braces: `if (!ptr) return NULL;` |
| Pointer `*` | Attached to variable: `void *ptr` |
| Line length | ~80 characters |

## Comments

Explain *why*, not *what*. Doc comments on public API. Inline comments only for non-obvious logic or edge cases.

## Data Structures

| Pattern | Detail |
|---|---|
| Power-of-two sizes + bitmask | Fast modulo via `& (size - 1)` |
| Flexible array members | `char data[]` at end of struct |
| SoA layout | Parallel arrays for cache-friendly access |
| Items stored inline | By value, not by pointer |
