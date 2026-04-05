#include "tile_build.h"
#include "layers.h"
#include "tile_builder.h"

#include <brotli/encode.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#if defined(__ARM_NEON)
#include <arm_neon.h>
#endif

/* Coordinate quantization: map geo coords within tile bounds to uint16.
   Tile proper: [16384, 49151], extent = 32768, buffer = 16384 per side. */
#define TILE_EXTENT  32768
#define TILE_BUFFER  16384

/* Brotli compression quality for tile output.
   Level 1 = fastest; good enough for tile delivery since tiles are small. */
#define BROTLI_QUALITY 1

/* String dictionary with open-addressing hash table for O(1) intern. */

/* Hash table slot: maps hash → entry index. Empty slots use UINT32_MAX. */
#define DICT_EMPTY UINT32_MAX

typedef struct {
    char    **entries;
    uint32_t  count;
    uint32_t  entry_cap;
    uint32_t *ht;           /* hash table: slot → entry index */
    uint32_t  ht_cap;       /* always a power of 2 */
} str_dict;

static void str_dict_init(str_dict *d) {
    d->entries = NULL;
    d->count = 0;
    d->entry_cap = 0;
    d->ht = NULL;
    d->ht_cap = 0;
}

static void str_dict_free(str_dict *d) {
    for (uint32_t i = 0; i < d->count; i++) free(d->entries[i]);
    free(d->entries);
    free(d->ht);
}

static uint32_t str_hash(const char *s) {
    uint32_t h = 5381;
    for (; *s; s++)
        h = ((h << 5) + h) ^ (uint32_t)(unsigned char)*s;
    return h;
}

/* Rebuild the hash table after a capacity change. */
static void str_dict_rehash(str_dict *d) {
    for (uint32_t i = 0; i < d->ht_cap; i++)
        d->ht[i] = DICT_EMPTY;
    for (uint32_t i = 0; i < d->count; i++) {
        uint32_t h = str_hash(d->entries[i]) & (d->ht_cap - 1);
        while (d->ht[h] != DICT_EMPTY)
            h = (h + 1) & (d->ht_cap - 1);
        d->ht[h] = i;
    }
}

/* Returns index; adds if not present */
static uint32_t str_dict_intern(str_dict *d, const char *s) {
    /* Grow hash table if needed (keep load < 75%) */
    if (d->ht_cap == 0 || d->count * 4 >= d->ht_cap * 3) {
        uint32_t nc = d->ht_cap ? d->ht_cap * 2 : 16;
        uint32_t *p = realloc(d->ht, nc * sizeof(uint32_t));
        if (!p) return 0;
        d->ht = p;
        d->ht_cap = nc;
        str_dict_rehash(d);
    }

    /* Probe for existing entry */
    uint32_t h = str_hash(s) & (d->ht_cap - 1);
    while (d->ht[h] != DICT_EMPTY) {
        if (strcmp(d->entries[d->ht[h]], s) == 0)
            return d->ht[h];
        h = (h + 1) & (d->ht_cap - 1);
    }

    /* Not found — insert */
    if (d->count == d->entry_cap) {
        uint32_t nc = d->entry_cap ? d->entry_cap * 2 : 16;
        char **p = realloc(d->entries, nc * sizeof(char *));
        if (!p) return 0;
        d->entries = p;
        d->entry_cap = nc;
    }
    d->entries[d->count] = strdup(s);
    if (!d->entries[d->count]) return 0;
    d->ht[h] = d->count;
    return d->count++;
}

/* Stored feature (accumulated before building FlatBuffer) */
typedef struct {
    uint32_t layer;
    uint32_t geom_type;    /* WKB type 1-6 */
    uint16_t *qx, *qy;
    int32_t  *qz;
    uint32_t n_coords;
    uint32_t *offsets;
    uint32_t n_offsets;
    uint32_t *props;       /* [key_idx, val_idx] pairs */
    uint32_t n_props;
} stored_feat;


struct arpt_tile_builder {
    arpt_bounds bounds;

    str_dict keys;
    str_dict vals;

    stored_feat *feats;
    uint32_t n_feats;
    uint32_t feat_cap;
    uint32_t total_coords;
};

static uint16_t quantize_x(const arpt_bounds *b, double x) {
    double t = (x - b->min_x) / (b->max_x - b->min_x);
    double q = t * TILE_EXTENT + TILE_BUFFER;
    if (q < 0.0) q = 0.0;
    if (q > 65535.0) q = 65535.0;
    return (uint16_t)q;
}

static uint16_t quantize_y(const arpt_bounds *b, double y) {
    double t = (y - b->min_y) / (b->max_y - b->min_y);
    double q = t * TILE_EXTENT + TILE_BUFFER;
    if (q < 0.0) q = 0.0;
    if (q > 65535.0) q = 65535.0;
    return (uint16_t)q;
}

/* Remove consecutive duplicate vertices from quantized coordinate arrays.
   For multi-ring/multi-line geometries, each part is deduped independently
   and offsets are updated.  Returns the new total coordinate count.
   min_verts is the minimum surviving vertex count per part (4 for polygon
   rings, 2 for line segments); parts that degenerate below this are
   removed. */
static uint32_t dedup_quantized(uint16_t *qx, uint16_t *qy, int32_t *qz,
                                uint32_t n_coords,
                                uint32_t *offsets, uint32_t *n_offsets,
                                uint32_t min_verts) {
    if (n_coords <= 1) return n_coords;

    /* No offsets: single part */
    if (!offsets || !n_offsets || *n_offsets <= 1) {
        uint32_t out = 1;
        for (uint32_t i = 1; i < n_coords; i++) {
            if (qx[i] != qx[out - 1] || qy[i] != qy[out - 1]) {
                qx[out] = qx[i];
                qy[out] = qy[i];
                if (qz) qz[out] = qz[i];
                out++;
            }
        }
        return out;
    }

    /* Multi-part: dedup each part, compact, update offsets */
    uint32_t n_parts = *n_offsets - 1;
    uint32_t compact = 0;
    uint32_t kept_parts = 0;

    for (uint32_t p = 0; p < n_parts; p++) {
        uint32_t start = offsets[p];
        uint32_t end = offsets[p + 1];
        uint32_t pn = end - start;
        if (pn == 0) continue;

        /* Dedup this part into the compact position */
        qx[compact] = qx[start];
        qy[compact] = qy[start];
        if (qz) qz[compact] = qz[start];
        uint32_t out = 1;
        for (uint32_t i = 1; i < pn; i++) {
            uint32_t si = start + i;
            if (qx[si] != qx[compact + out - 1] ||
                qy[si] != qy[compact + out - 1]) {
                qx[compact + out] = qx[si];
                qy[compact + out] = qy[si];
                if (qz) qz[compact + out] = qz[si];
                out++;
            }
        }

        if (out >= min_verts) {
            offsets[kept_parts] = compact;
            kept_parts++;
            compact += out;
        }
    }

    offsets[kept_parts] = compact;
    *n_offsets = kept_parts + 1;
    return compact;
}

arpt_tile_builder *arpt_tile_builder_create(arpt_bounds bounds) {
    arpt_tile_builder *b = calloc(1, sizeof(*b));
    if (!b) return NULL;
    b->bounds = bounds;
    str_dict_init(&b->keys);
    str_dict_init(&b->vals);
    return b;
}

bool arpt_tile_builder_add_feature(arpt_tile_builder *b,
                                   const arpt_feature *feat) {
    if (!b || !feat || !feat->geom) return false;
    const arpt_geom *g = feat->geom;
    if (g->n_coords == 0) return false;

    /* Grow feature array */
    if (b->n_feats == b->feat_cap) {
        uint32_t nc = b->feat_cap ? b->feat_cap * 2 : 16;
        stored_feat *p = realloc(b->feats, nc * sizeof(stored_feat));
        if (!p) return false;
        b->feats = p;
        b->feat_cap = nc;
    }

    stored_feat *sf = &b->feats[b->n_feats];
    memset(sf, 0, sizeof(*sf));
    sf->layer = feat->layer;
    sf->geom_type = g->type;
    sf->n_coords = g->n_coords;

    /* Quantize coordinates */
    sf->qx = malloc(g->n_coords * sizeof(uint16_t));
    sf->qy = malloc(g->n_coords * sizeof(uint16_t));
    sf->qz = calloc(g->n_coords, sizeof(int32_t));
    if (!sf->qx || !sf->qy || !sf->qz) {
        free(sf->qx); free(sf->qy); free(sf->qz);
        return false;
    }

#if defined(__ARM_NEON)
    {
        double inv_x = 1.0 / (b->bounds.max_x - b->bounds.min_x);
        double inv_y = 1.0 / (b->bounds.max_y - b->bounds.min_y);
        float64x2_t v_min_x  = vdupq_n_f64(b->bounds.min_x);
        float64x2_t v_min_y  = vdupq_n_f64(b->bounds.min_y);
        float64x2_t v_inv_x  = vdupq_n_f64(inv_x);
        float64x2_t v_inv_y  = vdupq_n_f64(inv_y);
        float64x2_t v_extent = vdupq_n_f64((double)TILE_EXTENT);
        float64x2_t v_buffer = vdupq_n_f64((double)TILE_BUFFER);
        float64x2_t v_zero   = vdupq_n_f64(0.0);
        float64x2_t v_max16  = vdupq_n_f64(65535.0);

        uint32_t i = 0;
        uint32_t end2 = g->n_coords & ~1u;
        for (; i < end2; i += 2) {
            /* Quantize x */
            float64x2_t vx = vld1q_f64(g->x + i);
            float64x2_t tx = vmulq_f64(vmulq_f64(vsubq_f64(vx, v_min_x), v_inv_x), v_extent);
            tx = vaddq_f64(tx, v_buffer);
            tx = vmaxq_f64(tx, v_zero);
            tx = vminq_f64(tx, v_max16);
            sf->qx[i]     = (uint16_t)vgetq_lane_f64(tx, 0);
            sf->qx[i + 1] = (uint16_t)vgetq_lane_f64(tx, 1);

            /* Quantize y */
            float64x2_t vy = vld1q_f64(g->y + i);
            float64x2_t ty = vmulq_f64(vmulq_f64(vsubq_f64(vy, v_min_y), v_inv_y), v_extent);
            ty = vaddq_f64(ty, v_buffer);
            ty = vmaxq_f64(ty, v_zero);
            ty = vminq_f64(ty, v_max16);
            sf->qy[i]     = (uint16_t)vgetq_lane_f64(ty, 0);
            sf->qy[i + 1] = (uint16_t)vgetq_lane_f64(ty, 1);

            /* Z */
            if (g->z) {
                float64x2_t vz = vld1q_f64(g->z + i);
                float64x2_t vz_mm = vmulq_f64(vz, vdupq_n_f64(1000.0));
                sf->qz[i]     = (int32_t)vgetq_lane_f64(vz_mm, 0);
                sf->qz[i + 1] = (int32_t)vgetq_lane_f64(vz_mm, 1);
            }
        }
        /* Handle remaining element */
        for (; i < g->n_coords; i++) {
            sf->qx[i] = quantize_x(&b->bounds, g->x[i]);
            sf->qy[i] = quantize_y(&b->bounds, g->y[i]);
            if (g->z) sf->qz[i] = (int32_t)(g->z[i] * 1000.0);
        }
    }
#else
    for (uint32_t i = 0; i < g->n_coords; i++) {
        sf->qx[i] = quantize_x(&b->bounds, g->x[i]);
        sf->qy[i] = quantize_y(&b->bounds, g->y[i]);
        if (g->z) sf->qz[i] = (int32_t)(g->z[i] * 1000.0);
    }
#endif

    /* Copy offsets */
    if (g->offsets && g->n_offsets > 0) {
        sf->offsets = malloc(g->n_offsets * sizeof(uint32_t));
        if (!sf->offsets) {
            free(sf->qx); free(sf->qy); free(sf->qz);
            return false;
        }
        memcpy(sf->offsets, g->offsets, g->n_offsets * sizeof(uint32_t));
        sf->n_offsets = g->n_offsets;
    }

    /* Remove consecutive duplicate vertices that collapsed during
       quantization.  Polygon rings need >= 4 verts (3 unique + closing),
       line strings need >= 2. */
    uint32_t min_verts = (sf->geom_type == 3 || sf->geom_type == 6) ? 4 : 2;
    sf->n_coords = dedup_quantized(sf->qx, sf->qy, sf->qz, sf->n_coords,
                                   sf->offsets, &sf->n_offsets, min_verts);
    if (sf->n_coords < min_verts && sf->geom_type >= 2) {
        free(sf->qx); free(sf->qy); free(sf->qz); free(sf->offsets);
        return false;
    }

    /* Intern properties */
    if (feat->n_props > 0 && feat->prop_keys && feat->prop_vals) {
        sf->props = malloc(feat->n_props * 2 * sizeof(uint32_t));
        if (!sf->props) {
            free(sf->qx); free(sf->qy); free(sf->qz); free(sf->offsets);
            return false;
        }
        for (uint32_t i = 0; i < feat->n_props; i++) {
            sf->props[i * 2]     = str_dict_intern(&b->keys, feat->prop_keys[i]);
            sf->props[i * 2 + 1] = str_dict_intern(&b->vals, feat->prop_vals[i]);
        }
        sf->n_props = feat->n_props;
    }

    b->n_feats++;
    b->total_coords += g->n_coords;
    return true;
}

/* Check if any element of a z array is non-zero. */
static bool has_nonzero_z(const int32_t *qz, uint32_t n) {
    if (!qz) return false;
    for (uint32_t i = 0; i < n; i++) {
        if (qz[i] != 0) return true;
    }
    return false;
}

/* Build geometry for one feature into the flatcc builder.
   Omits the z array when all values are zero to save ~33% of
   per-coordinate storage for 2D features (land, coastlines, etc.). */
static void build_geom(flatcc_builder_t *fb, const stored_feat *sf) {
    bool emit_z = has_nonzero_z(sf->qz, sf->n_coords);

    switch (sf->geom_type) {
    case 1: case 4: { /* Point / MultiPoint */
        arpentry_tiles_PointGeometry_start(fb);
        arpentry_tiles_PointGeometry_x_create(fb, sf->qx, sf->n_coords);
        arpentry_tiles_PointGeometry_y_create(fb, sf->qy, sf->n_coords);
        if (emit_z)
            arpentry_tiles_PointGeometry_z_create(fb, sf->qz, sf->n_coords);
        arpentry_tiles_PointGeometry_ref_t ref = arpentry_tiles_PointGeometry_end(fb);
        arpentry_tiles_Feature_geometry_PointGeometry_add(fb, ref);
        break;
    }
    case 2: case 5: { /* LineString / MultiLineString */
        arpentry_tiles_LineGeometry_start(fb);
        arpentry_tiles_LineGeometry_x_create(fb, sf->qx, sf->n_coords);
        arpentry_tiles_LineGeometry_y_create(fb, sf->qy, sf->n_coords);
        if (emit_z)
            arpentry_tiles_LineGeometry_z_create(fb, sf->qz, sf->n_coords);
        if (sf->offsets && sf->n_offsets > 0) {
            arpentry_tiles_LineGeometry_line_offsets_create(fb, sf->offsets, sf->n_offsets);
        }
        arpentry_tiles_LineGeometry_ref_t ref = arpentry_tiles_LineGeometry_end(fb);
        arpentry_tiles_Feature_geometry_LineGeometry_add(fb, ref);
        break;
    }
    case 3: case 6: { /* Polygon / MultiPolygon */
        arpentry_tiles_PolygonGeometry_start(fb);
        arpentry_tiles_PolygonGeometry_x_create(fb, sf->qx, sf->n_coords);
        arpentry_tiles_PolygonGeometry_y_create(fb, sf->qy, sf->n_coords);
        if (emit_z)
            arpentry_tiles_PolygonGeometry_z_create(fb, sf->qz, sf->n_coords);
        if (sf->offsets && sf->n_offsets > 0) {
            arpentry_tiles_PolygonGeometry_ring_offsets_create(fb, sf->offsets, sf->n_offsets);
        }
        arpentry_tiles_PolygonGeometry_ref_t ref = arpentry_tiles_PolygonGeometry_end(fb);
        arpentry_tiles_Feature_geometry_PolygonGeometry_add(fb, ref);
        break;
    }
    default:
        break;
    }
}

/* Maximum terrain grid subdivisions.  The actual grid is chosen
   adaptively based on tile angular span — see terrain_grid_size(). */
#define TERRAIN_GRID_MAX 64
#define TERRAIN_GRID_MIN 16

/* Choose terrain grid subdivisions based on tile angular span.
   Target: ~2.8° per cell (matching zoom-0 at 64 subdivisions) for
   globe curvature, clamped to [TERRAIN_GRID_MIN, TERRAIN_GRID_MAX]
   and rounded to a power of 2. */
static uint32_t terrain_grid_size(const arpt_bounds *bounds) {
    double lon_span = bounds->max_x - bounds->min_x;
    /* 180° / 64 ≈ 2.8° per cell at zoom 0 */
    uint32_t g = (uint32_t)(lon_span / 2.8);
    /* Round up to next power of 2 */
    if (g < TERRAIN_GRID_MIN) g = TERRAIN_GRID_MIN;
    uint32_t p = TERRAIN_GRID_MIN;
    while (p < g && p < TERRAIN_GRID_MAX) p *= 2;
    return p;
}

/* Encode a unit normal vector into octahedral int8×2.
   Input: (nx, ny, nz) must be normalized. */
static void encode_octahedral(double nx, double ny, double nz,
                               int8_t *out_x, int8_t *out_y) {
    /* Project onto octahedron */
    double inv = 1.0 / (fabs(nx) + fabs(ny) + fabs(nz));
    double ox = nx * inv;
    double oy = ny * inv;

    /* Reflect lower hemisphere */
    if (nz < 0.0) {
        double tmp_x = (1.0 - fabs(oy)) * (ox >= 0.0 ? 1.0 : -1.0);
        double tmp_y = (1.0 - fabs(ox)) * (oy >= 0.0 ? 1.0 : -1.0);
        ox = tmp_x;
        oy = tmp_y;
    }

    /* Quantize to int8 [-127, 127] */
    double sx = ox * 127.0;
    double sy = oy * 127.0;
    if (sx > 127.0) sx = 127.0;
    if (sx < -127.0) sx = -127.0;
    if (sy > 127.0) sy = 127.0;
    if (sy < -127.0) sy = -127.0;
    *out_x = (int8_t)(sx >= 0.0 ? sx + 0.5 : sx - 0.5);
    *out_y = (int8_t)(sy >= 0.0 ? sy + 0.5 : sy - 0.5);
}

/* Emit a flat subdivided terrain mesh covering the tile extent as layer 0.
   Vertices are at z=0 with sphere-surface normals for globe rendering. */
static void emit_terrain(flatcc_builder_t *fb, const arpt_bounds *bounds) {
    const uint32_t gn = terrain_grid_size(bounds);
    const uint32_t n_verts = (gn + 1) * (gn + 1);
    const uint32_t n_tris = gn * gn * 2;
    const uint32_t n_idx = n_tris * 3;

    uint16_t *vx = malloc(n_verts * sizeof(*vx));
    uint16_t *vy = malloc(n_verts * sizeof(*vy));
    int32_t  *vz = calloc(n_verts, sizeof(*vz));
    uint32_t *indices = malloc(n_idx * sizeof(*indices));
    int8_t   *normals = malloc(n_verts * 2 * sizeof(*normals));
    if (!vx || !vy || !vz || !indices || !normals) {
        free(vx); free(vy); free(vz); free(indices); free(normals);
        return;
    }

    double lon_span = bounds->max_x - bounds->min_x;
    double lat_span = bounds->max_y - bounds->min_y;

    /* Generate flat grid vertices */
    for (uint32_t row = 0; row <= gn; row++) {
        for (uint32_t col = 0; col <= gn; col++) {
            uint32_t vi = row * (gn + 1) + col;
            vx[vi] = (uint16_t)(TILE_BUFFER +
                      (uint32_t)((uint64_t)col * (TILE_EXTENT - 1) / gn));
            vy[vi] = (uint16_t)(TILE_BUFFER +
                      (uint32_t)((uint64_t)row * (TILE_EXTENT - 1) / gn));
        }
    }

    /* Compute sphere-surface normals in ECEF (up vector at each vertex) */
    for (uint32_t row = 0; row <= gn; row++) {
        double lat_r = (bounds->min_y + (double)row / gn * lat_span) * M_PI / 180.0;
        double sin_lat = sin(lat_r), cos_lat = cos(lat_r);
        for (uint32_t col = 0; col <= gn; col++) {
            uint32_t vi = row * (gn + 1) + col;
            double lon_r = (bounds->min_x + (double)col / gn * lon_span) * M_PI / 180.0;
            double nx = cos_lat * cos(lon_r);
            double ny = cos_lat * sin(lon_r);
            double nz = sin_lat;
            encode_octahedral(nx, ny, nz,
                              &normals[vi * 2], &normals[vi * 2 + 1]);
        }
    }

    /* Generate triangle indices (two triangles per grid cell) */
    uint32_t ii = 0;
    for (uint32_t row = 0; row < gn; row++) {
        for (uint32_t col = 0; col < gn; col++) {
            uint32_t tl = row * (gn + 1) + col;
            uint32_t tr = tl + 1;
            uint32_t bl = tl + (gn + 1);
            uint32_t br = bl + 1;
            indices[ii++] = tl;
            indices[ii++] = tr;
            indices[ii++] = br;
            indices[ii++] = tl;
            indices[ii++] = br;
            indices[ii++] = bl;
        }
    }

    arpentry_tiles_Tile_layers_push_start(fb);
    arpentry_tiles_Layer_name_create_str(fb, "terrain");

    arpentry_tiles_Layer_features_start(fb);
    arpentry_tiles_Layer_features_push_start(fb);
    arpentry_tiles_Feature_id_add(fb, 0);

    arpentry_tiles_MeshGeometry_start(fb);
    arpentry_tiles_MeshGeometry_x_create(fb, vx, n_verts);
    arpentry_tiles_MeshGeometry_y_create(fb, vy, n_verts);
    arpentry_tiles_MeshGeometry_z_create(fb, vz, n_verts);
    arpentry_tiles_MeshGeometry_indices_create(fb, indices, n_idx);
    arpentry_tiles_MeshGeometry_normals_create(fb, normals, n_verts * 2);

    arpentry_tiles_MeshGeometry_parts_start(fb);
    arpentry_tiles_Part_t part = {0};
    part.first_index = 0;
    part.index_count = n_idx;
    /* color.a = 0 → client-styled */
    arpentry_tiles_MeshGeometry_parts_push(fb, &part);
    arpentry_tiles_MeshGeometry_parts_end(fb);

    arpentry_tiles_MeshGeometry_ref_t mesh_ref = arpentry_tiles_MeshGeometry_end(fb);
    arpentry_tiles_Feature_geometry_MeshGeometry_add(fb, mesh_ref);

    arpentry_tiles_Layer_features_push_end(fb);
    arpentry_tiles_Layer_features_end(fb);
    arpentry_tiles_Tile_layers_push_end(fb);

    free(vx);
    free(vy);
    free(vz);
    free(indices);
    free(normals);
}

void *arpt_tile_builder_finish(arpt_tile_builder *b, size_t *out_size) {
    if (!b) { if (out_size) *out_size = 0; return NULL; }

    flatcc_builder_t fb;
    flatcc_builder_init(&fb);

    arpentry_tiles_Tile_start_as_root(&fb);
    arpentry_tiles_Tile_version_add(&fb, 1);

    /* Keys dictionary */
    if (b->keys.count > 0) {
        arpentry_tiles_Tile_keys_start(&fb);
        for (uint32_t i = 0; i < b->keys.count; i++) {
            arpentry_tiles_Tile_keys_push_create_str(&fb, b->keys.entries[i]);
        }
        arpentry_tiles_Tile_keys_end(&fb);
    }

    /* Values dictionary (all stored as strings for now) */
    if (b->vals.count > 0) {
        arpentry_tiles_Tile_values_start(&fb);
        for (uint32_t i = 0; i < b->vals.count; i++) {
            arpentry_tiles_Tile_values_push_start(&fb);
            arpentry_tiles_Value_type_add(&fb, arpentry_tiles_PropertyValueType_String);
            arpentry_tiles_Value_string_value_create_str(&fb, b->vals.entries[i]);
            arpentry_tiles_Tile_values_push_end(&fb);
        }
        arpentry_tiles_Tile_values_end(&fb);
    }

    /* Group features by layer */
    uint32_t max_layer = 0;
    for (uint32_t i = 0; i < b->n_feats; i++) {
        if (b->feats[i].layer > max_layer) max_layer = b->feats[i].layer;
    }

    arpentry_tiles_Tile_layers_start(&fb);

    /* Emit a flat terrain mesh as layer 0 if no features claim it */
    bool has_layer0 = false;
    for (uint32_t i = 0; i < b->n_feats; i++) {
        if (b->feats[i].layer == 0) { has_layer0 = true; break; }
    }
    if (!has_layer0) {
        emit_terrain(&fb, &b->bounds);
    }

    for (uint32_t layer = 0; layer <= max_layer; layer++) {
        /* Check if any features in this layer */
        bool has_feats = false;
        for (uint32_t i = 0; i < b->n_feats; i++) {
            if (b->feats[i].layer == layer) { has_feats = true; break; }
        }
        if (!has_feats) continue;

        arpentry_tiles_Tile_layers_push_start(&fb);
        const char *name = layer < ARPT_MAX_LAYERS ? arpt_layer_names[layer] : "default";
        arpentry_tiles_Layer_name_create_str(&fb, name);

        arpentry_tiles_Layer_features_start(&fb);
        for (uint32_t i = 0; i < b->n_feats; i++) {
            if (b->feats[i].layer != layer) continue;
            const stored_feat *sf = &b->feats[i];

            arpentry_tiles_Layer_features_push_start(&fb);
            arpentry_tiles_Feature_id_add(&fb, i);

            build_geom(&fb, sf);

            /* Properties */
            if (sf->n_props > 0) {
                arpentry_tiles_Property_t *props =
                    malloc(sf->n_props * sizeof(arpentry_tiles_Property_t));
                if (props) {
                    for (uint32_t j = 0; j < sf->n_props; j++) {
                        props[j].key = sf->props[j * 2];
                        props[j].value = sf->props[j * 2 + 1];
                    }
                    arpentry_tiles_Feature_properties_create(&fb, props, sf->n_props);
                    free(props);
                }
            }

            arpentry_tiles_Layer_features_push_end(&fb);
        }
        arpentry_tiles_Layer_features_end(&fb);
        arpentry_tiles_Tile_layers_push_end(&fb);
    }
    arpentry_tiles_Tile_layers_end(&fb);

    arpentry_tiles_Tile_end_as_root(&fb);

    /* Finalize FlatBuffer */
    size_t fb_size;
    void *fb_buf = flatcc_builder_finalize_buffer(&fb, &fb_size);
    flatcc_builder_clear(&fb);

    if (!fb_buf) {
        if (out_size) *out_size = 0;
        return NULL;
    }

    /* Brotli compress */
    size_t max_compressed = BrotliEncoderMaxCompressedSize(fb_size);
    if (max_compressed == 0) max_compressed = fb_size + 64;

    uint8_t *compressed = malloc(max_compressed);
    if (!compressed) {
        free(fb_buf);
        if (out_size) *out_size = 0;
        return NULL;
    }

    size_t compressed_size = max_compressed;
    if (!BrotliEncoderCompress(BROTLI_QUALITY, BROTLI_DEFAULT_WINDOW, BROTLI_DEFAULT_MODE,
                               fb_size, (const uint8_t *)fb_buf,
                               &compressed_size, compressed)) {
        free(fb_buf);
        free(compressed);
        if (out_size) *out_size = 0;
        return NULL;
    }

    free(fb_buf);
    if (out_size) *out_size = compressed_size;
    return compressed;
}

uint32_t arpt_tile_builder_total_coords(const arpt_tile_builder *b) {
    return b ? b->total_coords : 0;
}

void arpt_tile_builder_free(arpt_tile_builder *b) {
    if (!b) return;
    for (uint32_t i = 0; i < b->n_feats; i++) {
        free(b->feats[i].qx);
        free(b->feats[i].qy);
        free(b->feats[i].qz);
        free(b->feats[i].offsets);
        free(b->feats[i].props);
    }
    free(b->feats);
    str_dict_free(&b->keys);
    str_dict_free(&b->vals);
    free(b);
}
