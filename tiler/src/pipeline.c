#include "pipeline.h"
#include "archive.h"
#include "clip.h"
#include "hilbert.h"
#include "simplify.h"
#include "sort.h"
#include "tile_build.h"
#include "wkb.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Sort key layout: tile_id(48) | layer(4) | rank(12) */
static uint64_t make_sort_key(uint64_t tile_id, uint32_t layer, uint32_t rank) {
    return (tile_id << 16) | ((uint64_t)(layer & 0xF) << 12) | (rank & 0xFFF);
}

static uint64_t sort_key_tile_id(uint64_t key) {
    return key >> 16;
}

/* ---- Compact serialized feature for the sort buffer ----
 *
 * To minimize sort data volume, coordinates are stored as quantized
 * uint16 relative to the tile (computed at clip time). Properties are
 * stored as length-prefixed strings.
 *
 * Format: [geom_type:1][n_coords:4][n_offsets:4][n_props:4]
 *         [tile_w:8][tile_s:8][tile_e:8][tile_n:8]
 *         [x:8*n][y:8*n]
 *         [offsets:4*noff]
 *         [prop_key_len:2 + key_bytes + prop_val_len:2 + val_bytes] * n_props
 *
 * We keep double coords in the sort record so the tile_builder can
 * quantize them relative to the actual tile bounds it receives. The
 * coordinates here are the *clipped* coordinates in WGS84 degrees.
 */

static uint8_t *serialize_feature(const arpt_geom *geom,
                                  const char *const *pkeys,
                                  const char *const *pvals,
                                  uint32_t n_props, size_t *out_size) {
    /* Compute size */
    size_t sz = 1 + 4 + 4 + 4;  /* type + n_coords + n_offsets + n_props */
    sz += geom->n_coords * sizeof(double) * 2;
    uint32_t noff = geom->offsets ? geom->n_offsets : 0;
    if (noff > 0) sz += noff * sizeof(uint32_t);
    for (uint32_t i = 0; i < n_props; i++) {
        sz += 2 + strlen(pkeys[i]) + 2 + strlen(pvals[i]);
    }

    uint8_t *buf = malloc(sz);
    if (!buf) return NULL;

    size_t pos = 0;
    buf[pos++] = (uint8_t)geom->type;
    memcpy(buf + pos, &geom->n_coords, 4); pos += 4;
    memcpy(buf + pos, &noff, 4); pos += 4;
    memcpy(buf + pos, &n_props, 4); pos += 4;

    memcpy(buf + pos, geom->x, geom->n_coords * sizeof(double));
    pos += geom->n_coords * sizeof(double);
    memcpy(buf + pos, geom->y, geom->n_coords * sizeof(double));
    pos += geom->n_coords * sizeof(double);

    if (noff > 0) {
        memcpy(buf + pos, geom->offsets, noff * sizeof(uint32_t));
        pos += noff * sizeof(uint32_t);
    }

    for (uint32_t i = 0; i < n_props; i++) {
        uint16_t klen = (uint16_t)strlen(pkeys[i]);
        uint16_t vlen = (uint16_t)strlen(pvals[i]);
        memcpy(buf + pos, &klen, 2); pos += 2;
        memcpy(buf + pos, pkeys[i], klen); pos += klen;
        memcpy(buf + pos, &vlen, 2); pos += 2;
        memcpy(buf + pos, pvals[i], vlen); pos += vlen;
    }

    *out_size = pos;
    return buf;
}

static bool deserialize_feature(const uint8_t *data, size_t size,
                                arpt_geom *geom, arpt_feature *feat,
                                char ***keys_out, char ***vals_out) {
    if (size < 13) return false;
    size_t pos = 0;

    geom->type = data[pos++];
    memcpy(&geom->n_coords, data + pos, 4); pos += 4;
    uint32_t noff;
    memcpy(&noff, data + pos, 4); pos += 4;
    uint32_t n_props;
    memcpy(&n_props, data + pos, 4); pos += 4;

    geom->x = malloc(geom->n_coords * sizeof(double));
    geom->y = malloc(geom->n_coords * sizeof(double));
    if (!geom->x || !geom->y) return false;

    memcpy(geom->x, data + pos, geom->n_coords * sizeof(double));
    pos += geom->n_coords * sizeof(double);
    memcpy(geom->y, data + pos, geom->n_coords * sizeof(double));
    pos += geom->n_coords * sizeof(double);

    if (noff > 0) {
        geom->offsets = malloc(noff * sizeof(uint32_t));
        if (!geom->offsets) return false;
        memcpy(geom->offsets, data + pos, noff * sizeof(uint32_t));
        pos += noff * sizeof(uint32_t);
        geom->n_offsets = noff;
    }

    char **keys = NULL, **vals = NULL;
    if (n_props > 0) {
        keys = malloc(n_props * sizeof(char *));
        vals = malloc(n_props * sizeof(char *));
        if (!keys || !vals) { free(keys); free(vals); return false; }
        for (uint32_t i = 0; i < n_props; i++) {
            uint16_t klen, vlen;
            memcpy(&klen, data + pos, 2); pos += 2;
            keys[i] = malloc(klen + 1);
            memcpy(keys[i], data + pos, klen); keys[i][klen] = '\0'; pos += klen;
            memcpy(&vlen, data + pos, 2); pos += 2;
            vals[i] = malloc(vlen + 1);
            memcpy(vals[i], data + pos, vlen); vals[i][vlen] = '\0'; pos += vlen;
        }
    }

    feat->geom = geom;
    feat->prop_keys = (const char *const *)keys;
    feat->prop_vals = (const char *const *)vals;
    feat->n_props = n_props;
    *keys_out = keys;
    *vals_out = vals;
    return true;
}

/* ---- Synthetic data generator ---- */

typedef struct {
    arpt_geom geom;
    uint32_t  layer;
} synth_feature;

/* Generate a grid of synthetic features within the bbox.
   Returns features once; the pipeline clips to all zoom levels. */
static synth_feature *generate_synthetic(const double bbox[4], int *out_count) {
    double w = bbox[0], s = bbox[1], e = bbox[2], n = bbox[3];

    /* Generate a moderate grid of points + a few polygons */
    int nx = 16, ny = 16;
    double lon_step = (e - w) / nx;
    double lat_step = (n - s) / ny;

    int cap = nx * ny + 4;  /* points + polygons */
    synth_feature *feats = malloc((size_t)cap * sizeof(synth_feature));
    if (!feats) { *out_count = 0; return NULL; }
    int count = 0;

    /* Points */
    for (int ix = 0; ix < nx; ix++) {
        for (int iy = 0; iy < ny; iy++) {
            synth_feature *sf = &feats[count];
            memset(sf, 0, sizeof(*sf));
            sf->geom.type = 1;
            sf->geom.x = malloc(sizeof(double));
            sf->geom.y = malloc(sizeof(double));
            if (!sf->geom.x || !sf->geom.y) {
                free(sf->geom.x); free(sf->geom.y);
                continue;
            }
            sf->geom.x[0] = w + (ix + 0.5) * lon_step;
            sf->geom.y[0] = s + (iy + 0.5) * lat_step;
            if (sf->geom.y[0] > 85.0) sf->geom.y[0] = 85.0;
            if (sf->geom.y[0] < -85.0) sf->geom.y[0] = -85.0;
            sf->geom.n_coords = 1;
            sf->layer = 0;
            count++;
        }
    }

    /* A few rectangles spanning the bbox */
    for (int i = 0; i < 4 && count < cap; i++) {
        double pw = w + (e - w) * 0.2 * i;
        double pe = pw + (e - w) * 0.15;
        double ps = s + (n - s) * 0.2 * i;
        double pn = ps + (n - s) * 0.15;
        if (pn > 85.0) pn = 85.0;
        if (ps < -85.0) ps = -85.0;

        synth_feature *sf = &feats[count];
        memset(sf, 0, sizeof(*sf));
        sf->geom.type = 3; /* Polygon */
        sf->geom.n_coords = 5;
        sf->geom.x = malloc(5 * sizeof(double));
        sf->geom.y = malloc(5 * sizeof(double));
        sf->geom.offsets = malloc(2 * sizeof(uint32_t));
        if (!sf->geom.x || !sf->geom.y || !sf->geom.offsets) {
            free(sf->geom.x); free(sf->geom.y); free(sf->geom.offsets);
            continue;
        }
        sf->geom.x[0] = pw; sf->geom.y[0] = ps;
        sf->geom.x[1] = pe; sf->geom.y[1] = ps;
        sf->geom.x[2] = pe; sf->geom.y[2] = pn;
        sf->geom.x[3] = pw; sf->geom.y[3] = pn;
        sf->geom.x[4] = pw; sf->geom.y[4] = ps;
        sf->geom.offsets[0] = 0;
        sf->geom.offsets[1] = 5;
        sf->geom.n_offsets = 2;
        sf->layer = 1;
        count++;
    }

    *out_count = count;
    return feats;
}

static void free_synth_features(synth_feature *feats, int count) {
    for (int i = 0; i < count; i++) {
        free(feats[i].geom.x);
        free(feats[i].geom.y);
        free(feats[i].geom.z);
        free(feats[i].geom.offsets);
    }
    free(feats);
}

/* ---- Tile clipping callback context ---- */

typedef struct {
    arpt_sorter *sorter;
    uint32_t     layer;
    uint32_t     rank;
    const char  *const *prop_keys;
    const char  *const *prop_vals;
    uint32_t     n_props;
    int          zoom;  /* current zoom being clipped to */
} clip_ctx;

static void clip_cb(int z, int x, int y,
                    const arpt_geom *clipped, void *ctx) {
    clip_ctx *c = (clip_ctx *)ctx;
    uint64_t tile_id = arpt_hilbert_tile_id(z, x, y);
    uint64_t key = make_sort_key(tile_id, c->layer, c->rank);

    size_t data_size;
    uint8_t *data = serialize_feature(clipped,
                                      c->prop_keys, c->prop_vals,
                                      c->n_props, &data_size);
    if (data) {
        arpt_sorter_add(c->sorter, key, data, data_size);
        free(data);
    }
}

/* ---- Compute tile bounds ---- */

static arpt_bounds compute_tile_bounds(int z, int tx, int ty) {
    double n = (double)(1 << z);
    double w = (double)tx / n * 360.0 - 180.0;
    double e = (double)(tx + 1) / n * 360.0 - 180.0;
    double n_lat = atan(sinh(M_PI * (1.0 - 2.0 * (double)ty / n))) * 180.0 / M_PI;
    double s_lat = atan(sinh(M_PI * (1.0 - 2.0 * (double)(ty + 1) / n))) * 180.0 / M_PI;
    return (arpt_bounds){w, s_lat, e, n_lat};
}

/* Simplification tolerance: roughly 1 pixel at the given zoom */
static double zoom_tolerance(int zoom) {
    return 360.0 / (double)(1 << (zoom + 8));
}

/* ---- Pipeline ----
 *
 * Single-pass design: read features once, clip each feature to every
 * zoom level, push all (tile_id, feature) pairs into the external
 * sorter. Then stream sorted output → tile builder → archive.
 */

bool arpt_pipeline_run(const arpt_pipeline_config *config) {
    if (!config || !config->output) return false;

    arpt_sorter *sorter = arpt_sorter_create(config->tmp_dir, config->mem_budget);
    if (!sorter) return false;

    uint32_t rank = 0;

    if (config->synthetic) {
        int n_feats = 0;
        synth_feature *feats = generate_synthetic(config->bbox, &n_feats);
        if (!feats && n_feats == 0) {
            arpt_sorter_free(sorter);
            return false;
        }

        /* Single pass: for each feature, clip to all zoom levels */
        for (int i = 0; i < n_feats; i++) {
            for (int z = config->min_zoom; z <= config->max_zoom; z++) {
                /* Make a working copy for simplification at this zoom */
                arpt_geom g = feats[i].geom;
                double *sx = NULL, *sy = NULL;

                if (g.type >= 2 && g.n_coords > 2) {
                    /* Copy coords for in-place simplification */
                    sx = malloc(g.n_coords * sizeof(double));
                    sy = malloc(g.n_coords * sizeof(double));
                    if (sx && sy) {
                        memcpy(sx, g.x, g.n_coords * sizeof(double));
                        memcpy(sy, g.y, g.n_coords * sizeof(double));
                        g.x = sx;
                        g.y = sy;
                        g.n_coords = arpt_simplify(g.x, g.y, g.n_coords,
                                                   zoom_tolerance(z));
                    } else {
                        free(sx); free(sy);
                        sx = sy = NULL;
                    }
                }

                clip_ctx ctx = {
                    .sorter = sorter,
                    .layer = feats[i].layer,
                    .rank = rank,
                    .prop_keys = NULL,
                    .prop_vals = NULL,
                    .n_props = 0,
                    .zoom = z,
                };
                arpt_assign_tiles(&g, z, clip_cb, &ctx);

                free(sx);
                free(sy);
            }
            rank++;
            if (rank > 0xFFF) rank = 0xFFF; /* clamp to 12-bit field */
        }

        free_synth_features(feats, n_feats);
    }
    /* TODO: non-synthetic path using OvertureMaps reader:
     *   arpt_overture_reader *reader = arpt_overture_open(path);
     *   while (arpt_overture_next(reader, &feat)) {
     *       for (z = min..max) { simplify copy; clip; push to sorter; }
     *   }
     */

    /* Finalize sort */
    if (!arpt_sorter_finish(sorter)) {
        arpt_sorter_free(sorter);
        return false;
    }

    /* Create archive */
    arpt_archive_writer *writer = arpt_archive_writer_create(config->output);
    if (!writer) {
        arpt_sorter_free(sorter);
        return false;
    }

    arpt_archive_writer_set_zoom(writer,
                                 (uint8_t)config->min_zoom,
                                 (uint8_t)config->max_zoom);
    arpt_archive_writer_set_bounds(writer,
                                   config->bbox[0], config->bbox[1],
                                   config->bbox[2], config->bbox[3]);

    /* Stream sorted records → group by tile → build → write */
    uint64_t key;
    const void *data;
    size_t data_size;

    uint64_t cur_tile_id = UINT64_MAX;
    arpt_tile_builder *builder = NULL;
    int cur_z = 0, cur_x = 0, cur_y = 0;

    while (arpt_sorter_next(sorter, &key, &data, &data_size)) {
        uint64_t tid = sort_key_tile_id(key);

        if (tid != cur_tile_id) {
            /* Flush previous tile */
            if (builder) {
                size_t tile_size;
                void *tile_data = arpt_tile_builder_finish(builder, &tile_size);
                if (tile_data && tile_size > 0) {
                    arpt_archive_writer_add_tile(writer,
                                                 (uint8_t)cur_z, (uint32_t)cur_x,
                                                 (uint32_t)cur_y,
                                                 tile_data, tile_size);
                }
                free(tile_data);
                arpt_tile_builder_free(builder);
            }

            cur_tile_id = tid;
            arpt_hilbert_tile_id_decode(tid, &cur_z, &cur_x, &cur_y);
            arpt_bounds tb = compute_tile_bounds(cur_z, cur_x, cur_y);
            builder = arpt_tile_builder_create(tb);
        }

        if (builder && data && data_size > 0) {
            arpt_geom geom = {0};
            arpt_feature feat = {0};
            char **keys = NULL, **vals = NULL;

            if (deserialize_feature(data, data_size, &geom, &feat, &keys, &vals)) {
                feat.layer = (uint32_t)((key >> 12) & 0xF);
                arpt_tile_builder_add_feature(builder, &feat);
            }

            free(geom.x);
            free(geom.y);
            free(geom.z);
            free(geom.offsets);
            if (keys) {
                for (uint32_t i = 0; i < feat.n_props; i++) free(keys[i]);
                free(keys);
            }
            if (vals) {
                for (uint32_t i = 0; i < feat.n_props; i++) free(vals[i]);
                free(vals);
            }
        }
    }

    /* Flush last tile */
    if (builder) {
        size_t tile_size;
        void *tile_data = arpt_tile_builder_finish(builder, &tile_size);
        if (tile_data && tile_size > 0) {
            arpt_archive_writer_add_tile(writer,
                                         (uint8_t)cur_z, (uint32_t)cur_x,
                                         (uint32_t)cur_y,
                                         tile_data, tile_size);
        }
        free(tile_data);
        arpt_tile_builder_free(builder);
    }

    bool ok = arpt_archive_writer_finish(writer);
    arpt_archive_writer_free(writer);
    arpt_sorter_free(sorter);

    if (ok) {
        fprintf(stderr, "Archive written: %s\n", config->output);
    }

    return ok;
}
