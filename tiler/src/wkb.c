/* WKB geometry parser — full implementation for 2D and 3D geometries. */

#include "wkb.h"

#include <stdlib.h>
#include <string.h>

/* ── Endian helpers ──────────────────────────────────────────────────── */

static inline uint32_t read_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

static inline uint32_t read_u32_be(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24)
         | ((uint32_t)p[1] << 16)
         | ((uint32_t)p[2] << 8)
         | (uint32_t)p[3];
}

static inline double read_f64_le(const uint8_t *p)
{
    double d;
    memcpy(&d, p, 8);
    return d;
}

static inline double read_f64_be(const uint8_t *p)
{
    uint8_t buf[8];
    for (int i = 0; i < 8; i++) buf[i] = p[7 - i];
    double d;
    memcpy(&d, buf, 8);
    return d;
}

/* ── Reader context ─────────────────────────────────────────────────── */

typedef struct {
    const uint8_t *data;
    size_t size;
    size_t pos;
    bool le;  /* true = little-endian */
} wkb_reader;

static bool wkb_check(const wkb_reader *r, size_t n)
{
    return r->pos + n <= r->size;
}

static bool wkb_read_byte(wkb_reader *r, uint8_t *out)
{
    if (!wkb_check(r, 1)) return false;
    *out = r->data[r->pos++];
    return true;
}

static bool wkb_read_u32(wkb_reader *r, uint32_t *out)
{
    if (!wkb_check(r, 4)) return false;
    *out = r->le ? read_u32_le(r->data + r->pos)
                 : read_u32_be(r->data + r->pos);
    r->pos += 4;
    return true;
}

static bool wkb_read_f64(wkb_reader *r, double *out)
{
    if (!wkb_check(r, 8)) return false;
    *out = r->le ? read_f64_le(r->data + r->pos)
                 : read_f64_be(r->data + r->pos);
    r->pos += 8;
    return true;
}

/* ── Type decoding ──────────────────────────────────────────────────── */

/* WKB type IDs */
#define WKB_POINT           1
#define WKB_LINESTRING      2
#define WKB_POLYGON         3
#define WKB_MULTIPOINT      4
#define WKB_MULTILINESTRING 5
#define WKB_MULTIPOLYGON    6

/* ISO Z offset */
#define WKB_Z_ISO   1000

/* OGC Z flag */
#define WKB_Z_OGC   0x80000000u

static bool decode_wkb_type(uint32_t raw, uint32_t *type, bool *has_z)
{
    *has_z = false;
    *type = raw;

    /* ISO Z variants: type + 1000 */
    if (raw > WKB_Z_ISO && raw <= WKB_Z_ISO + 6) {
        *type = raw - WKB_Z_ISO;
        *has_z = true;
    }
    /* OGC Z flag */
    else if (raw & WKB_Z_OGC) {
        *type = raw & ~WKB_Z_OGC;
        *has_z = true;
    }

    return *type >= WKB_POINT && *type <= WKB_MULTIPOLYGON;
}

/* ── Header reading ─────────────────────────────────────────────────── */

static bool read_header(wkb_reader *r, uint32_t *type, bool *has_z)
{
    uint8_t byte_order;
    if (!wkb_read_byte(r, &byte_order)) return false;
    if (byte_order > 1) return false;
    r->le = (byte_order == 1);

    uint32_t raw_type;
    if (!wkb_read_u32(r, &raw_type)) return false;
    return decode_wkb_type(raw_type, type, has_z);
}

/* ── Coordinate reading ─────────────────────────────────────────────── */

static bool read_coords(wkb_reader *r, bool has_z,
                         double *x, double *y, double *z, uint32_t count)
{
    for (uint32_t i = 0; i < count; i++) {
        if (!wkb_read_f64(r, &x[i])) return false;
        if (!wkb_read_f64(r, &y[i])) return false;
        if (has_z) {
            if (!wkb_read_f64(r, &z[i])) return false;
        }
    }
    return true;
}

/* ── Geometry parsers ───────────────────────────────────────────────── */

static bool parse_point(wkb_reader *r, bool has_z, arpt_geom *out)
{
    out->type = WKB_POINT;
    out->n_coords = 1;
    out->x = malloc(sizeof(double));
    out->y = malloc(sizeof(double));
    out->z = has_z ? malloc(sizeof(double)) : NULL;
    if (!out->x || !out->y || (has_z && !out->z)) return false;

    return read_coords(r, has_z, out->x, out->y, out->z, 1);
}

static bool parse_linestring(wkb_reader *r, bool has_z, arpt_geom *out)
{
    uint32_t n;
    if (!wkb_read_u32(r, &n)) return false;

    out->type = WKB_LINESTRING;
    out->n_coords = n;
    out->x = malloc(n * sizeof(double));
    out->y = malloc(n * sizeof(double));
    out->z = has_z ? malloc(n * sizeof(double)) : NULL;
    if (!out->x || !out->y || (has_z && !out->z)) return false;

    return read_coords(r, has_z, out->x, out->y, out->z, n);
}

static bool parse_polygon(wkb_reader *r, bool has_z, arpt_geom *out)
{
    uint32_t num_rings;
    if (!wkb_read_u32(r, &num_rings)) return false;

    /* First pass: count total coordinates */
    size_t saved_pos = r->pos;
    uint32_t total = 0;
    for (uint32_t i = 0; i < num_rings; i++) {
        uint32_t ring_n;
        if (!wkb_read_u32(r, &ring_n)) return false;
        total += ring_n;
        size_t skip = ring_n * (has_z ? 24u : 16u);
        if (!wkb_check(r, skip)) return false;
        r->pos += skip;
    }

    out->type = WKB_POLYGON;
    out->n_coords = total;
    out->n_offsets = num_rings + 1; /* N+1 sentinel style */
    out->x = malloc(total * sizeof(double));
    out->y = malloc(total * sizeof(double));
    out->z = has_z ? malloc(total * sizeof(double)) : NULL;
    out->offsets = malloc((num_rings + 1) * sizeof(uint32_t));
    if (!out->x || !out->y || (has_z && !out->z) || !out->offsets)
        return false;

    /* Second pass: read coordinates */
    r->pos = saved_pos;
    uint32_t coord_idx = 0;
    for (uint32_t i = 0; i < num_rings; i++) {
        uint32_t ring_n;
        if (!wkb_read_u32(r, &ring_n)) return false;
        out->offsets[i] = coord_idx;
        if (!read_coords(r, has_z, out->x + coord_idx, out->y + coord_idx,
                         has_z ? out->z + coord_idx : NULL, ring_n))
            return false;
        coord_idx += ring_n;
    }
    out->offsets[num_rings] = total; /* sentinel */

    return true;
}

static bool parse_multi_point(wkb_reader *r, arpt_geom *out)
{
    uint32_t num_geoms;
    if (!wkb_read_u32(r, &num_geoms)) return false;

    out->type = WKB_MULTIPOINT;
    out->n_coords = num_geoms;
    out->x = malloc(num_geoms * sizeof(double));
    out->y = malloc(num_geoms * sizeof(double));
    if (!out->x || !out->y) return false;

    for (uint32_t i = 0; i < num_geoms; i++) {
        uint32_t sub_type;
        bool sub_z;
        if (!read_header(r, &sub_type, &sub_z)) return false;
        if (sub_type != WKB_POINT) return false;

        /* Allocate z on first Z point */
        if (sub_z && !out->z) {
            out->z = calloc(num_geoms, sizeof(double));
            if (!out->z) return false;
        }

        if (!wkb_read_f64(r, &out->x[i])) return false;
        if (!wkb_read_f64(r, &out->y[i])) return false;
        if (sub_z) {
            if (!wkb_read_f64(r, &out->z[i])) return false;
        }
    }

    return true;
}

static bool parse_multi_linestring(wkb_reader *r, arpt_geom *out)
{
    uint32_t num_geoms;
    if (!wkb_read_u32(r, &num_geoms)) return false;

    /* First pass: count total coordinates */
    size_t saved_pos = r->pos;
    uint32_t total = 0;
    bool any_z = false;
    for (uint32_t i = 0; i < num_geoms; i++) {
        uint32_t sub_type;
        bool sub_z;
        if (!read_header(r, &sub_type, &sub_z)) return false;
        if (sub_type != WKB_LINESTRING) return false;
        if (sub_z) any_z = true;

        uint32_t n;
        if (!wkb_read_u32(r, &n)) return false;
        total += n;
        size_t skip = n * (sub_z ? 24u : 16u);
        if (!wkb_check(r, skip)) return false;
        r->pos += skip;
    }

    out->type = WKB_MULTILINESTRING;
    out->n_coords = total;
    out->n_offsets = num_geoms + 1; /* N+1 sentinel style */
    out->x = malloc(total * sizeof(double));
    out->y = malloc(total * sizeof(double));
    out->z = any_z ? calloc(total, sizeof(double)) : NULL;
    out->offsets = malloc((num_geoms + 1) * sizeof(uint32_t));
    if (!out->x || !out->y || (any_z && !out->z) || !out->offsets)
        return false;

    /* Second pass: read coordinates */
    r->pos = saved_pos;
    uint32_t coord_idx = 0;
    for (uint32_t i = 0; i < num_geoms; i++) {
        uint32_t sub_type;
        bool sub_z;
        if (!read_header(r, &sub_type, &sub_z)) return false;

        uint32_t n;
        if (!wkb_read_u32(r, &n)) return false;
        out->offsets[i] = coord_idx;
        if (!read_coords(r, sub_z, out->x + coord_idx, out->y + coord_idx,
                         sub_z ? out->z + coord_idx : NULL, n))
            return false;
        coord_idx += n;
    }
    out->offsets[num_geoms] = total; /* sentinel */

    return true;
}

static bool parse_multi_polygon(wkb_reader *r, arpt_geom *out)
{
    uint32_t num_geoms;
    if (!wkb_read_u32(r, &num_geoms)) return false;

    /* First pass: count total coordinates and rings */
    size_t saved_pos = r->pos;
    uint32_t total_coords = 0;
    uint32_t total_rings = 0;
    bool any_z = false;
    for (uint32_t i = 0; i < num_geoms; i++) {
        uint32_t sub_type;
        bool sub_z;
        if (!read_header(r, &sub_type, &sub_z)) return false;
        if (sub_type != WKB_POLYGON) return false;
        if (sub_z) any_z = true;

        uint32_t num_rings;
        if (!wkb_read_u32(r, &num_rings)) return false;
        total_rings += num_rings;
        for (uint32_t j = 0; j < num_rings; j++) {
            uint32_t ring_n;
            if (!wkb_read_u32(r, &ring_n)) return false;
            total_coords += ring_n;
            size_t skip = ring_n * (sub_z ? 24u : 16u);
            if (!wkb_check(r, skip)) return false;
            r->pos += skip;
        }
    }

    out->type = WKB_MULTIPOLYGON;
    out->n_coords = total_coords;
    out->n_offsets = total_rings + 1; /* N+1 sentinel style */
    out->n_parts = num_geoms;
    out->x = malloc(total_coords * sizeof(double));
    out->y = malloc(total_coords * sizeof(double));
    out->z = any_z ? calloc(total_coords, sizeof(double)) : NULL;
    out->offsets = malloc((total_rings + 1) * sizeof(uint32_t));
    out->parts = malloc(num_geoms * sizeof(uint32_t));
    if (!out->x || !out->y || (any_z && !out->z) || !out->offsets || !out->parts)
        return false;

    /* Second pass: read coordinates */
    r->pos = saved_pos;
    uint32_t coord_idx = 0;
    uint32_t ring_idx = 0;
    for (uint32_t i = 0; i < num_geoms; i++) {
        uint32_t sub_type;
        bool sub_z;
        if (!read_header(r, &sub_type, &sub_z)) return false;
        out->parts[i] = ring_idx;

        uint32_t num_rings;
        if (!wkb_read_u32(r, &num_rings)) return false;
        for (uint32_t j = 0; j < num_rings; j++) {
            uint32_t ring_n;
            if (!wkb_read_u32(r, &ring_n)) return false;
            out->offsets[ring_idx++] = coord_idx;
            if (!read_coords(r, sub_z, out->x + coord_idx, out->y + coord_idx,
                             sub_z ? out->z + coord_idx : NULL, ring_n))
                return false;
            coord_idx += ring_n;
        }
    }
    out->offsets[total_rings] = total_coords; /* sentinel */

    return true;
}

/* ── Public API ─────────────────────────────────────────────────────── */

bool arpt_wkb_parse(const uint8_t *data, size_t size, arpt_geom *out)
{
    if (!data || size < 5 || !out) return false;

    memset(out, 0, sizeof(*out));

    wkb_reader r = { .data = data, .size = size, .pos = 0, .le = true };

    uint32_t type;
    bool has_z;
    if (!read_header(&r, &type, &has_z)) return false;

    bool ok = false;
    switch (type) {
        case WKB_POINT:           ok = parse_point(&r, has_z, out); break;
        case WKB_LINESTRING:      ok = parse_linestring(&r, has_z, out); break;
        case WKB_POLYGON:         ok = parse_polygon(&r, has_z, out); break;
        case WKB_MULTIPOINT:      ok = parse_multi_point(&r, out); break;
        case WKB_MULTILINESTRING: ok = parse_multi_linestring(&r, out); break;
        case WKB_MULTIPOLYGON:    ok = parse_multi_polygon(&r, out); break;
        default: break;
    }

    if (!ok) {
        arpt_geom_free(out);
        memset(out, 0, sizeof(*out));
    }
    return ok;
}

void arpt_geom_free(arpt_geom *g) {
    if (!g) return;
    free(g->x);
    free(g->y);
    free(g->z);
    free(g->offsets);
    free(g->parts);
}
