#include "pipeline.h"
#include "archive.h"
#include "clip.h"
#include "dem.h"
#include "feature_io.h"
#include "hilbert.h"
#include "overture.h"
#include "simplify.h"
#include "sort.h"
#include "tile_build.h"

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

static uint32_t sort_key_layer(uint64_t key) {
    return (uint32_t)((key >> 12) & 0xF);
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

        /* Simplify before clipping to avoid tile-boundary artifacts. */
        arpt_geom sg;
        double tol = zoom_tolerance(z);
        if (!arpt_simplify_geom(geom, tol, &sg))
            continue;

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
        arpt_geom_free(&sg);
    }
}

/* ---- Tile flush / empty-fill helpers ---- */

/* Finish building a tile, write it to the archive, and record its ID.
   builder is freed; caller should set its pointer to NULL after this. */
static void flush_tile(arpt_tile_builder *builder,
                       arpt_archive_writer *writer,
                       int z, int x, int y,
                       uint64_t **written_ids, size_t *written_count,
                       size_t *written_cap) {
    if (!builder) return;
    size_t tile_size;
    void *tile_data = arpt_tile_builder_finish(builder, &tile_size);
    if (tile_data && tile_size > 0) {
        arpt_archive_writer_add_tile(writer,
                                     (uint8_t)z, (uint32_t)x, (uint32_t)y,
                                     tile_data, tile_size);
        if (*written_count == *written_cap) {
            *written_cap *= 2;
            uint64_t *tmp = realloc(*written_ids,
                                    *written_cap * sizeof(**written_ids));
            if (tmp) *written_ids = tmp;
        }
        if (*written_count < *written_cap) {
            (*written_ids)[(*written_count)++] =
                arpt_hilbert_tile_id(z, x, y);
        }
    }
    free(tile_data);
    arpt_tile_builder_free(builder);
}

static int compare_uint64(const uint64_t *a, const uint64_t *b) {
    if (*a < *b) return -1;
    if (*a > *b) return 1;
    return 0;
}

/* Write terrain-only tiles for every grid cell in the bbox that doesn't
   already have feature data. */
static void fill_empty_tiles(arpt_archive_writer *writer,
                             const arpt_dem *dem, const double bbox[4],
                             int min_zoom, int max_zoom,
                             uint64_t *written_ids, size_t written_count) {
    qsort(written_ids, written_count, sizeof(*written_ids),
          (int (*)(const void *, const void *))compare_uint64);

    uint64_t empty_count = 0;

    for (int z = min_zoom; z <= max_zoom; z++) {
        int n_cols = 1 << (z + 1);
        int n_rows = 1 << z;
        double lon_span = 360.0 / (double)n_cols;
        double lat_span = 180.0 / (double)n_rows;

        int x_min = (int)floor((bbox[0] + 180.0) / lon_span);
        int x_max = (int)floor((bbox[2] + 180.0) / lon_span);
        int y_min = (int)floor((bbox[1] + 90.0) / lat_span);
        int y_max = (int)floor((bbox[3] + 90.0) / lat_span);
        if (x_min < 0) x_min = 0;
        if (x_max >= n_cols) x_max = n_cols - 1;
        if (y_min < 0) y_min = 0;
        if (y_max >= n_rows) y_max = n_rows - 1;

        for (int y = y_min; y <= y_max; y++) {
            for (int x = x_min; x <= x_max; x++) {
                uint64_t tid = arpt_hilbert_tile_id(z, x, y);

                /* Binary search in written_ids */
                size_t lo = 0, hi = written_count;
                while (lo < hi) {
                    size_t mid = lo + (hi - lo) / 2;
                    if (written_ids[mid] < tid) lo = mid + 1;
                    else hi = mid;
                }
                if (lo < written_count && written_ids[lo] == tid)
                    continue;

                arpt_bounds tb = arpt_tile_bounds(z, x, y);
                arpt_tile_builder *eb = arpt_tile_builder_create(tb, dem);
                if (!eb) continue;
                size_t tile_size;
                void *tile_data = arpt_tile_builder_finish(eb, &tile_size);
                if (tile_data && tile_size > 0) {
                    arpt_archive_writer_add_tile(
                        writer, (uint8_t)z, (uint32_t)x, (uint32_t)y,
                        tile_data, tile_size);
                    empty_count++;
                }
                free(tile_data);
                arpt_tile_builder_free(eb);
            }
        }
    }

    if (empty_count > 0) {
        fprintf(stderr, "Added %llu empty tiles\n",
                (unsigned long long)empty_count);
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

    /* Load DEM if provided */
    arpt_dem *dem = NULL;
    if (config->dem_path) {
        dem = arpt_dem_open(config->dem_path);
        if (!dem) {
            fprintf(stderr, "Warning: cannot load DEM %s, using flat terrain\n",
                    config->dem_path);
        }
    }

    arpt_sorter *sorter = arpt_sorter_create(tmp_dir, mem_budget);
    if (!sorter) { arpt_dem_free(dem); return false; }

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

            /* Build properties: class from type, name from subtype */
            const char *cls = feat.subtype ? feat.type : (feat.type ? feat.type : "unknown");
            const char *pkeys[2] = { "class", "name" };
            const char *pvals[2] = { cls, feat.subtype };
            uint32_t n_props = feat.subtype ? 2 : 1;

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
    arpt_archive_config arc = {
        .path = config->output,
        .min_zoom = (uint8_t)min_zoom,
        .max_zoom = (uint8_t)max_zoom,
        .bounds = { config->bbox[0], config->bbox[1],
                    config->bbox[2], config->bbox[3] },
    };
    arpt_archive_writer *writer = arpt_archive_writer_create(&arc);
    if (!writer) {
        arpt_sorter_free(sorter);
        return false;
    }

    /* Track which tiles have features so we can fill empty ones later. */
    size_t written_cap = 4096, written_count = 0;
    uint64_t *written_ids = malloc(written_cap * sizeof(*written_ids));
    if (!written_ids) {
        arpt_archive_writer_free(writer);
        arpt_sorter_free(sorter);
        arpt_dem_free(dem);
        return false;
    }

    /* Stream sorted records → group by tile → build → write */
    uint64_t cur_tile_id = UINT64_MAX;
    arpt_tile_builder *builder = NULL;
    int cur_z = 0, cur_x = 0, cur_y = 0;

    uint64_t key;
    const void *data;
    size_t data_size;

    while (arpt_sorter_next(sorter, &key, &data, &data_size)) {
        uint64_t tid = sort_key_tile_id(key);

        if (tid != cur_tile_id) {
            /* Flush previous tile */
            flush_tile(builder, writer, cur_z, cur_x, cur_y,
                       &written_ids, &written_count, &written_cap);
            builder = NULL;

            cur_tile_id = tid;
            arpt_hilbert_tile_id_decode(tid, &cur_z, &cur_x, &cur_y);
            arpt_bounds tb = arpt_tile_bounds(cur_z, cur_x, cur_y);
            builder = arpt_tile_builder_create(tb, dem);
        }

        if (builder && data && data_size > 0) {
            arpt_geom geom = {0};
            arpt_feature feat = {0};
            char **keys = NULL, **vals = NULL;

            if (arpt_feature_deserialize(data, data_size, &geom, &feat,
                                         &keys, &vals)) {
                feat.layer = sort_key_layer(key);
                arpt_tile_builder_add_feature(builder, &feat);
            }

            arpt_feature_deserialize_free(&geom, &feat, keys, vals);
        }
    }

    /* Flush last tile */
    flush_tile(builder, writer, cur_z, cur_x, cur_y,
               &written_ids, &written_count, &written_cap);

    /* Fill empty tiles for all grid cells within the bbox */
    fill_empty_tiles(writer, dem, config->bbox,
                     min_zoom, max_zoom,
                     written_ids, written_count);

    free(written_ids);

    bool ok = arpt_archive_writer_finish(writer);
    arpt_archive_writer_free(writer);
    arpt_sorter_free(sorter);
    arpt_dem_free(dem);

    if (ok) {
        fprintf(stderr, "Archive written: %s\n", config->output);
    }

    return ok;
}
