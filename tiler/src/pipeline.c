#include "pipeline.h"
#include "archive.h"
#include "clip.h"
#include "hilbert.h"
#include "overture.h"
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
    double       tolerance; /* simplification tolerance for this zoom */
} clip_ctx;

/* Simplify clipped geometry before serialization.
 * Operates on a mutable copy — caller must free sx, sy, so. */
static void simplify_clipped(arpt_geom *g, double *sx, double *sy,
                              uint32_t *so, double tolerance) {
    if (tolerance <= 0.0) return;
    if (g->n_coords <= 2) return;

    if ((g->type == 3 || g->type == 6) && g->offsets && g->n_offsets > 1) {
        /* Ring-aware simplification */
        uint32_t n_rings = g->n_offsets - 1;
        uint32_t out = 0;
        for (uint32_t ri = 0; ri < n_rings; ri++) {
            uint32_t start = so[ri];
            uint32_t end = so[ri + 1];
            uint32_t ring_n = end - start;
            uint32_t new_n = ring_n;
            if (ring_n > 2) {
                new_n = arpt_simplify_ring(sx + start, sy + start,
                                           ring_n, tolerance);
            }
            if (new_n >= 4) {
                if (out != start) {
                    memmove(sx + out, sx + start, new_n * sizeof(double));
                    memmove(sy + out, sy + start, new_n * sizeof(double));
                }
                so[ri] = out;
                out += new_n;
            } else {
                so[ri] = out; /* degenerate, skip */
            }
        }
        so[n_rings] = out;
        g->n_coords = out;
    } else {
        g->n_coords = arpt_simplify(sx, sy, g->n_coords, tolerance);
    }
}

static void clip_cb(int z, int x, int y,
                    const arpt_geom *clipped, void *ctx) {
    clip_ctx *c = (clip_ctx *)ctx;
    uint64_t tile_id = arpt_hilbert_tile_id(z, x, y);
    uint64_t key = make_sort_key(tile_id, c->layer, c->rank);

    /* Simplify after clipping: make a mutable copy of the clipped
     * coordinates (which point into clip.c's internal buffers). */
    arpt_geom g = *clipped;
    double *sx = NULL, *sy = NULL;
    uint32_t *so = NULL;

    if (c->tolerance > 0.0 && g.type >= 2 && g.n_coords > 2) {
        size_t csz = g.n_coords * sizeof(double);
        sx = malloc(csz);
        sy = malloc(csz);
        if (sx && sy) {
            memcpy(sx, g.x, csz);
            memcpy(sy, g.y, csz);
            g.x = sx;
            g.y = sy;

            if (g.offsets && g.n_offsets > 0) {
                size_t osz = g.n_offsets * sizeof(uint32_t);
                so = malloc(osz);
                if (so) {
                    memcpy(so, g.offsets, osz);
                    g.offsets = so;
                }
            }

            simplify_clipped(&g, sx, sy, so, c->tolerance);
        }
    }

    size_t data_size;
    uint8_t *data = serialize_feature(&g, c->prop_keys, c->prop_vals,
                                      c->n_props, &data_size);
    if (data) {
        arpt_sorter_add(c->sorter, key, data, data_size);
        free(data);
    }

    free(sx);
    free(sy);
    free(so);
}

/* ---- Compute tile bounds ---- */

/* Equirectangular tile bounds: 2^(z+1) columns × 2^z rows, y=0 at south. */
static arpt_bounds compute_tile_bounds(int z, int tx, int ty) {
    int n_cols = 1 << (z + 1);
    int n_rows = 1 << z;
    double lon_span = 360.0 / (double)n_cols;
    double lat_span = 180.0 / (double)n_rows;
    double w = -180.0 + (double)tx * lon_span;
    double s = -90.0 + (double)ty * lat_span;
    return (arpt_bounds){w, s, w + lon_span, s + lat_span};
}

/* Simplification tolerance in degrees.
 * Equirectangular grid: 2^(z+1) columns, each tile 256 px wide.
 * One pixel = 360 / 2^(z+9) degrees.  Use half a pixel as
 * tolerance so coastlines stay visually smooth. */
static double zoom_tolerance(int zoom) {
    return 360.0 / (double)(1 << (zoom + 10));
}

/* Maximum number of tiles a feature may span per axis at a zoom level.
 * Features exceeding this at high zoom are too large to be useful at
 * that detail level. They will still appear at lower zoom levels. */
#define MAX_TILE_SPAN 256

/* Check whether a line/polygon feature is too small to be visible at
 * the given zoom level (sub-pixel).  Returns true if the feature
 * should be skipped.  tile_pixels is the number of pixels per tile
 * side (typically 256). */
static bool feature_subpixel(const double bbox[4], int z, int tile_pixels) {
    double n_cols = (double)(1 << (z + 1));
    double n_rows = (double)(1 << z);
    double lon_span = bbox[2] - bbox[0];
    double lat_span = bbox[3] - bbox[1];
    double tile_lon = 360.0 / n_cols;
    double tile_lat = 180.0 / n_rows;
    double px_x = lon_span / tile_lon * (double)tile_pixels;
    double px_y = lat_span / tile_lat * (double)tile_pixels;
    return px_x < 1.0 && px_y < 1.0;
}

/* Estimate the tile span of a geometry at the given zoom level. */
static int64_t estimate_tile_span(const arpt_geom *geom, int z) {
    double gmin_x = geom->x[0], gmax_x = geom->x[0];
    double gmin_y = geom->y[0], gmax_y = geom->y[0];
    for (uint32_t i = 1; i < geom->n_coords; i++) {
        if (geom->x[i] < gmin_x) gmin_x = geom->x[i];
        if (geom->x[i] > gmax_x) gmax_x = geom->x[i];
        if (geom->y[i] < gmin_y) gmin_y = geom->y[i];
        if (geom->y[i] > gmax_y) gmax_y = geom->y[i];
    }
    double n_cols = (double)(1 << (z + 1));
    double n_rows = (double)(1 << z);
    int64_t tx_span = (int64_t)ceil((gmax_x - gmin_x) / 360.0 * n_cols) + 1;
    int64_t ty_span = (int64_t)ceil((gmax_y - gmin_y) / 180.0 * n_rows) + 1;
    return tx_span * ty_span;
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
            /* Compute feature bbox for sub-pixel filtering */
            double fbbox[4];
            fbbox[0] = fbbox[2] = feats[i].geom.x[0];
            fbbox[1] = fbbox[3] = feats[i].geom.y[0];
            for (uint32_t ci = 1; ci < feats[i].geom.n_coords; ci++) {
                if (feats[i].geom.x[ci] < fbbox[0]) fbbox[0] = feats[i].geom.x[ci];
                if (feats[i].geom.x[ci] > fbbox[2]) fbbox[2] = feats[i].geom.x[ci];
                if (feats[i].geom.y[ci] < fbbox[1]) fbbox[1] = feats[i].geom.y[ci];
                if (feats[i].geom.y[ci] > fbbox[3]) fbbox[3] = feats[i].geom.y[ci];
            }

            for (int z = config->min_zoom; z <= config->max_zoom; z++) {
                /* Skip sub-pixel features at this zoom */
                if (feats[i].geom.type >= 2 && feats[i].geom.n_coords > 1 &&
                    feature_subpixel(fbbox, z, 256)) {
                    continue;
                }

                clip_ctx ctx = {
                    .sorter = sorter,
                    .layer = feats[i].layer,
                    .rank = rank,
                    .prop_keys = NULL,
                    .prop_vals = NULL,
                    .n_props = 0,
                    .zoom = z,
                    .tolerance = zoom_tolerance(z),
                };
                arpt_assign_tiles(&feats[i].geom, z, clip_cb, &ctx);
            }
            rank++;
            if (rank > 0xFFF) rank = 0xFFF; /* clamp to 12-bit field */
        }

        free_synth_features(feats, n_feats);
    }
    /* OvertureMaps GeoParquet input path */
    for (int fi = 0; fi < config->n_inputs; fi++) {
        const arpt_pipeline_input *inp = &config->inputs[fi];
        fprintf(stderr, "Reading %s (layer %u)...\n", inp->path, inp->layer);

        arpt_overture *ov = arpt_overture_open(inp->path);
        if (!ov) {
            fprintf(stderr, "Warning: cannot open %s, skipping\n", inp->path);
            continue;
        }

        uint64_t feat_count = 0;
        arpt_overture_feature feat;
        while (arpt_overture_next(ov, &feat)) {
            arpt_geom *g = &feat.geometry;

            /* Skip features outside the target bbox */
            if (feat.has_bbox) {
                if (feat.bbox[2] < config->bbox[0] ||
                    feat.bbox[0] > config->bbox[2] ||
                    feat.bbox[3] < config->bbox[1] ||
                    feat.bbox[1] > config->bbox[3]) {
                    arpt_geom_free(g);
                    continue;
                }
            }

            /* Build properties: class from subtype or type */
            const char *cls = feat.subtype ? feat.subtype : feat.type;
            const char *pkeys[1] = { "class" };
            const char *pvals[1] = { cls ? cls : "unknown" };
            uint32_t n_props = 1;

            /* Compute feature bbox for sub-pixel filtering */
            double feat_bbox[4];
            if (feat.has_bbox) {
                memcpy(feat_bbox, feat.bbox, sizeof(feat_bbox));
            } else {
                feat_bbox[0] = feat_bbox[2] = g->x[0];
                feat_bbox[1] = feat_bbox[3] = g->y[0];
                for (uint32_t ci = 1; ci < g->n_coords; ci++) {
                    if (g->x[ci] < feat_bbox[0]) feat_bbox[0] = g->x[ci];
                    if (g->x[ci] > feat_bbox[2]) feat_bbox[2] = g->x[ci];
                    if (g->y[ci] < feat_bbox[1]) feat_bbox[1] = g->y[ci];
                    if (g->y[ci] > feat_bbox[3]) feat_bbox[3] = g->y[ci];
                }
            }

            /* Single pass: clip to all zoom levels.
             * Skip zoom levels where the feature spans too many tiles —
             * it will still appear at lower zoom levels. */
            for (int z = config->min_zoom; z <= config->max_zoom; z++) {
                /* Skip sub-pixel features at this zoom */
                if (g->type >= 2 && g->n_coords > 1 &&
                    feature_subpixel(feat_bbox, z, 256)) {
                    continue;
                }

                /* Skip features that span too many tiles at this zoom */
                if (g->type >= 2 && g->n_coords > 1 &&
                    estimate_tile_span(g, z) > (int64_t)MAX_TILE_SPAN * MAX_TILE_SPAN) {
                    continue;
                }

                clip_ctx ctx = {
                    .sorter = sorter,
                    .layer = inp->layer,
                    .rank = rank,
                    .prop_keys = pkeys,
                    .prop_vals = pvals,
                    .n_props = n_props,
                    .zoom = z,
                    .tolerance = zoom_tolerance(z),
                };
                arpt_assign_tiles(g, z, clip_cb, &ctx);
            }

            arpt_geom_free(g);
            rank++;
            if (rank > 0xFFF) rank = 0xFFF;
            feat_count++;

            if (feat_count % 100000 == 0) {
                fprintf(stderr, "  ... %llu features\n",
                        (unsigned long long)feat_count);
            }
        }

        arpt_overture_close(ov);
        fprintf(stderr, "  %llu features from %s\n",
                (unsigned long long)feat_count, inp->path);
    }

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
