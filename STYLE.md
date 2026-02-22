# C Style Guidelines

Inspired by [Josh Baker](https://github.com/tidwall) (tidwall), [Salvatore Sanfilippo](https://github.com/antirez) (antirez), and [Daniel Lemire](https://github.com/lemire).

## Core Philosophy

Write C that is simple, direct, and minimalist. The code should read almost like pseudocode — short functions, short variable names, minimal abstraction, and zero ceremony. Each `.c` file should be a self-contained module, readable top-to-bottom by a single person.

## Five Rules

1. **Write the simplest code that works.** No abstraction layers, no design patterns, no frameworks. A `struct` and some functions.
2. **One module = one `.h` + one `.c`.** Keep it self-contained. A reader should understand the whole thing in one sitting.
3. **Functions are short, names are short, files are few.** Brevity everywhere except comments that explain *why*.
4. **Errors are return values.** `NULL`, `-1`, `false`, or an error code. No exceptions, no global state.
5. **Own your memory.** Custom allocators, symmetric `create`/`free`, `sizeof(*ptr)`, single-allocation tricks.

## File Organization

Prefer 1-2 files per module: a `.h` + `.c` pair. Avoid splitting into many small files. Use section dividers inside large files to create structure:

```c
/* ========================= Public API ========================= */
/* ========================= Internals ========================== */
/* ========================= Tests ============================== */
```

Headers are minimal: forward declarations, opaque types, function prototypes. Hide struct bodies in the `.c` file. Tests go inline guarded by `#ifdef TESTING`, or in a single adjacent `test_foo.c`.

## Naming

| Element | Convention | Examples |
|---|---|---|
| Public functions | `prefix_verb_noun` | `hashmap_set`, `roaring_bitmap_add` |
| Static helpers | Short, descriptive, no prefix | `bucket_at`, `grow_capacity`, `is_cow` |
| Types (structs) | Opaque in header, body in `.c` | `struct hashmap;` (header only) |
| Macros / constants | `UPPER_SNAKE_CASE` | `SDS_MAX_PREALLOC`, `GROW_AT` |
| Variables | Short, lowercase | `s`, `p`, `len`, `fd`, `i`, `n` |
| Enums | `UPPER_PREFIX_NAME` values | `TG_POINT`, `JSON_NULL` |

## Functions

Keep functions short: 10–40 lines typical, 80 max for complex parsers. One function, one job. Prefer many small `static` helpers over one long function. Use early returns for guard clauses:

```c
roaring_bitmap_t *roaring_bitmap_copy(const roaring_bitmap_t *r) {
    roaring_bitmap_t *ans = roaring_malloc(sizeof(*ans));
    if (!ans) return NULL;
    if (!ra_init(&ans->container, r->container.size)) {
        roaring_free(ans);
        return NULL;
    }
    return ans;
}
```

## Error Handling

Errors flow through return values. No exceptions, no `errno`, no `longjmp`.

| Context | Pattern |
|---|---|
| Constructors | Return `NULL` on failure |
| Operations (libraries) | Return `bool` or integer error code |
| Operations (applications) | Abort on OOM (`perror` + `exit(1)`) |
| Complex cleanup | `goto cleanup` |
| OOM in data structures | Set flag on struct, check with `foo_oom()` |

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

## Memory Management

| Guideline | Detail |
|---|---|
| Custom allocator injection | Every library accepts `malloc`/`realloc`/`free` pointers |
| `sizeof(*ptr)` not `sizeof(Type)` | Survives type changes |
| Symmetric `create`/`free` | Every `foo_new` has a `foo_free` |
| `free(NULL)` is safe | Always guard: `if (!ptr) return;` |
| Single allocation | Allocate struct + inline data in one `malloc` call |
| Return moved pointer | Caller must use return value after realloc |

```c
// Single allocation: struct + data together
size_t total = sizeof(struct hashmap) + bucketsz * 2;
struct hashmap *map = _malloc(total);
map->spare = ((char *)map) + sizeof(struct hashmap);
```

## API Design

| Pattern | Description |
|---|---|
| Opaque types | Header declares `struct foo;` — body hidden in `.c` |
| `const` correctness | Read-only params are always `const` |
| Callback iteration | `bool (*iter)(const void *item, void *udata)` with `void *udata` |
| Extended variants | `_with_` suffix: `foo_new_with_allocator` |
| Operation variants | `_inplace`, `_checked`, `_bulk`, `_safe` suffixes |
| Borrowed pointers | Return `const void *` — caller does not own the memory |

```c
bool hashmap_scan(struct hashmap *map,
    bool (*iter)(const void *item, void *udata), void *udata);
```

## Formatting

| Aspect | Convention |
|---|---|
| Indentation | 4 spaces |
| Braces | K&R (opening brace on same line) |
| Single-statement `if` | No braces for short guards: `if (!ptr) return NULL;` |
| Pointer `*` | Attached to variable: `void *ptr` |
| Line length | ~80 characters soft limit |
| Blank lines | Between functions; sparingly within |

## Comments

Explain *why*, not *what*. The code says what; comments say why.

```c
// Good: explains a non-obvious decision
/* Don't use type 5: the user is appending to the string and type 5 is
 * not able to remember empty space. */
if (type == SDS_TYPE_5 && initlen == 0) type = SDS_TYPE_8;
```

Use doc comments on public API functions. Keep inline comments sparse — only for non-obvious logic, edge cases, or performance tricks. Use section dividers in large files. No boilerplate comments that restate the code.

## Data Structures

| Pattern | Detail |
|---|---|
| Power-of-two sizes + bitmask | Fast modulo via `& (size - 1)` |
| Flexible array members | `char data[]` at end of struct |
| Items stored inline | By value, not by pointer |
| SoA layout | Parallel arrays for cache-friendly access |
| Bit fields | Compact metadata in struct fields |
| Tagged dispatch | Typecode discriminator instead of union |

## Performance

| Pattern | Detail |
|---|---|
| Branchless where it matters | `pos += (c > 32 ? 1 : 0)` instead of `if` |
| SIMD with scalar fallback | Runtime detection, portable default |
| `__restrict__` on hot paths | Helps compiler optimize |
| Galloping search | Exponential + binary search for sorted arrays |
| Precomputed lookup tables | Static arrays for fast dispatch |
