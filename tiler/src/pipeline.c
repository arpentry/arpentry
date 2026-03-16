#include "pipeline.h"
#include "archive.h"
#include "clip.h"
#include "feature_io.h"
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
    int          zoom;
} clip_ctx;

static void clip_cb(int z, int x, int y,
                    const arpt_geom *clipped, void *ctx) {
    clip_ctx *c = (clip_ctx *)ctx;
    uint64_t tile_id = arpt_hilbert_tile_id(z, x, y);
    uint64_t key = make_sort_key(tile_id, c->layer, c->rank);

    size_t data_size;
    uint8_t *data = arpt_feature_serialize(clipped, c->prop_keys, c->prop_vals,
                                           c->n_props, &data_size);
    if (data) {
        arpt_sorter_add(c->sorter, key, data, data_size);
        free(data);
    }
}

/* Simplify geometry in-place before clipping.  Returns false if the
 * geometry degenerates (all rings become too small). */
static bool simplify_geom(arpt_geom *g, double tolerance) {
    if (tolerance <= 0.0) return true;
    if (g->n_coords <= 2) return true;

    if ((g->type == 3 || g->type == 6) && g->offsets && g->n_offsets > 1) {
        uint32_t n_rings = g->n_offsets - 1;
        uint32_t out = 0;
        for (uint32_t ri = 0; ri < n_rings; ri++) {
            uint32_t start = g->offsets[ri];
            uint32_t end = g->offsets[ri + 1];
            uint32_t ring_n = end - start;
            uint32_t new_n = ring_n;
            if (ring_n > 2) {
                new_n = arpt_simplify_ring(g->x + start, g->y + start,
                                           ring_n, tolerance);
            }
            if (new_n >= 4) {
                if (out != start) {
                    memmove(g->x + out, g->x + start, new_n * sizeof(double));
                    memmove(g->y + out, g->y + start, new_n * sizeof(double));
                }
                g->offsets[ri] = out;
                out += new_n;
            } else {
                g->offsets[ri] = out;
            }
        }
        g->offsets[n_rings] = out;
        g->n_coords = out;
        return out >= 4;
    } else if (g->type == 2 || g->type == 5) {
        g->n_coords = arpt_simplify(g->x, g->y, g->n_coords, tolerance);
        return g->n_coords >= 2;
    }
    return true;
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

/* Estimate the tile span of a geometry from its bbox at the given zoom. */
static int64_t estimate_tile_span(const double bbox[4], int z) {
    double n_cols = (double)(1 << (z + 1));
    double n_rows = (double)(1 << z);
    int64_t tx_span = (int64_t)ceil((bbox[2] - bbox[0]) / 360.0 * n_cols) + 1;
    int64_t ty_span = (int64_t)ceil((bbox[3] - bbox[1]) / 180.0 * n_rows) + 1;
    return tx_span * ty_span;
}

/* ---- Shared zoom-level processing loop ----
 *
 * For each zoom level: subpixel filter → tile-span filter →
 * copy-simplify-clip → free temporary copy.
 * Both synthetic and Overture paths call this after preparing
 * their feature data. */
static void process_feature_zooms(
    const arpt_geom *geom, const double bbox[4],
    int min_zoom, int max_zoom,
    uint32_t layer, uint32_t rank,
    const char *const *prop_keys, const char *const *prop_vals,
    uint32_t n_props, arpt_sorter *sorter) {

    for (int z = min_zoom; z <= max_zoom; z++) {
        /* Skip sub-pixel features at this zoom */
        if (geom->type >= 2 && geom->n_coords > 1 &&
            feature_subpixel(bbox, z, 256)) {
            continue;
        }

        /* Skip features that span too many tiles at this zoom */
        if (geom->type >= 2 && geom->n_coords > 1 &&
            estimate_tile_span(bbox, z) > (int64_t)MAX_TILE_SPAN * MAX_TILE_SPAN) {
            continue;
        }

        /* Simplify before clipping: make a mutable copy,
         * simplify at this zoom's tolerance, then clip.
         * This prevents DP from removing tile-boundary vertices
         * that the clipper adds, which would create diagonal
         * artifacts across tile interiors. */
        arpt_geom sg = *geom;
        double *sx = NULL, *sy = NULL;
        uint32_t *so = NULL;
        double tol = zoom_tolerance(z);
        bool need_copy = tol > 0.0 && sg.type >= 2 && sg.n_coords > 2;
        if (need_copy) {
            size_t csz = sg.n_coords * sizeof(double);
            sx = malloc(csz);
            sy = malloc(csz);
            if (sx && sy) {
                memcpy(sx, sg.x, csz);
                memcpy(sy, sg.y, csz);
                sg.x = sx;
                sg.y = sy;
                if (sg.offsets && sg.n_offsets > 0) {
                    size_t osz = sg.n_offsets * sizeof(uint32_t);
                    so = malloc(osz);
                    if (so) {
                        memcpy(so, sg.offsets, osz);
                        sg.offsets = so;
                    }
                }
                if (!simplify_geom(&sg, tol)) {
                    free(sx); free(sy); free(so);
                    continue;
                }
            }
        }

        clip_ctx ctx = {
            .sorter = sorter,
            .layer = layer,
            .rank = rank,
            .prop_keys = prop_keys,
            .prop_vals = prop_vals,
            .n_props = n_props,
            .zoom = z,
        };
        arpt_assign_tiles(&sg, z, clip_cb, &ctx);
        free(sx); free(sy); free(so);
    }
}

/* ---- Pipeline ----
 *
 * Single-pass design: read features once, clip each feature to every
 * zoom level, push all (tile_id, feature) pairs into the external
 * sorter. Then stream sorted output → tile builder → archive.
 */

bool arpt_pipeline_run(const arpt_pipeline_config *config) {
    if (!config || !config->output) return false;

    /* Validate config */
    int min_zoom = config->min_zoom;
    int max_zoom = config->max_zoom;
    if (min_zoom < 0) min_zoom = 0;
    if (min_zoom > 15) min_zoom = 15;
    if (max_zoom < min_zoom) max_zoom = min_zoom;
    if (max_zoom > 15) max_zoom = 15;

    if (config->bbox[0] >= config->bbox[2] ||
        config->bbox[1] >= config->bbox[3]) {
        fprintf(stderr, "Invalid bbox: west >= east or south >= north\n");
        return false;
    }

    if (config->n_inputs > 0 && !config->inputs) {
        fprintf(stderr, "n_inputs > 0 but inputs is NULL\n");
        return false;
    }

    const char *tmp_dir = config->tmp_dir ? config->tmp_dir : "/tmp";
    size_t mem_budget = config->mem_budget > 0
        ? config->mem_budget : (size_t)256 * 1024 * 1024;

    arpt_sorter *sorter = arpt_sorter_create(tmp_dir, mem_budget);
    if (!sorter) return false;

    uint32_t rank = 0;

    if (config->synthetic) {
        int n_feats = 0;
        synth_feature *feats = generate_synthetic(config->bbox, &n_feats);
        if (!feats && n_feats == 0) {
            arpt_sorter_free(sorter);
            return false;
        }

        for (int i = 0; i < n_feats; i++) {
            double fbbox[4];
            arpt_geom_bbox(&feats[i].geom, fbbox);

            process_feature_zooms(&feats[i].geom, fbbox,
                                  min_zoom, max_zoom,
                                  feats[i].layer, rank,
                                  NULL, NULL, 0, sorter);
            rank++;
            if (rank > 0xFFF) rank = 0xFFF;
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
                arpt_geom_bbox(g, feat_bbox);
            }

            process_feature_zooms(g, feat_bbox,
                                  min_zoom, max_zoom,
                                  inp->layer, rank,
                                  pkeys, pvals, n_props, sorter);

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
                                 (uint8_t)min_zoom,
                                 (uint8_t)max_zoom);
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
            arpt_bounds tb = arpt_tile_bounds(cur_z, cur_x, cur_y);
            builder = arpt_tile_builder_create(tb);
        }

        if (builder && data && data_size > 0) {
            arpt_geom geom = {0};
            arpt_feature feat = {0};
            char **keys = NULL, **vals = NULL;

            if (arpt_feature_deserialize(data, data_size, &geom, &feat,
                                         &keys, &vals)) {
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
