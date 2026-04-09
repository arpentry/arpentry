#include "pipeline.h"
#include "archive.h"
#include "clip.h"
#include "feature_io.h"
#include "hilbert.h"
#include "overture.h"
#include "sort.h"
#include "tile_build.h"
#include "wkb.h"
#include "workqueue.h"

#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef __APPLE__
#include <sys/sysctl.h>
#else
#include <unistd.h>
#endif

/* ---- Timing helpers ---- */

static double now_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

/* Sort key layout (64 bits): tile_id | layer | rank
 *   tile_id:  48 bits  (6-bit zoom + 42-bit Hilbert distance)
 *   layer:     4 bits  (0–15)
 *   rank:     12 bits  (0–4095, feature draw order within a layer)
 */
#define SORT_KEY_RANK_BITS   12
#define SORT_KEY_LAYER_BITS   4
#define SORT_KEY_RANK_MASK   ((1u << SORT_KEY_RANK_BITS) - 1)    /* 0xFFF */
#define SORT_KEY_LAYER_MASK  ((1u << SORT_KEY_LAYER_BITS) - 1)   /* 0xF */
#define SORT_KEY_SUB_BITS    (SORT_KEY_RANK_BITS + SORT_KEY_LAYER_BITS)  /* 16 */

static uint64_t make_sort_key(uint64_t tile_id, uint32_t layer, uint32_t rank) {
    return (tile_id << SORT_KEY_SUB_BITS)
         | ((uint64_t)(layer & SORT_KEY_LAYER_MASK) << SORT_KEY_RANK_BITS)
         | (rank & SORT_KEY_RANK_MASK);
}

static uint64_t sort_key_tile_id(uint64_t key) {
    return key >> SORT_KEY_SUB_BITS;
}

static uint32_t sort_key_layer(uint64_t key) {
    return (uint32_t)((key >> SORT_KEY_RANK_BITS) & SORT_KEY_LAYER_MASK);
}

static int detect_cpu_count(void) {
#ifdef __APPLE__
    int count = 0;
    size_t size = sizeof(count);
    if (sysctlbyname("hw.logicalcpu", &count, &size, NULL, 0) == 0 && count > 0)
        return count;
    return 4;
#else
    long n = sysconf(_SC_NPROCESSORS_ONLN);
    return (n > 0) ? (int)n : 4;
#endif
}

/* Callback context: serialize each clipped geometry into the sorter. */
typedef struct {
    arpt_sorter *sorter;
    uint32_t     layer;
    uint32_t     rank;
    const char  *const *prop_keys;
    const char  *const *prop_vals;
    uint32_t     n_props;
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

/* ---- Parallel pipeline data types ---- */

/* A raw feature queued from a reader thread to worker threads. */
typedef struct {
    uint8_t  *wkb;        /* Owned copy of raw WKB bytes */
    size_t    wkb_len;
    double    bbox[4];
    bool      has_bbox;
    char     *type;       /* Owned copy, may be NULL */
    char     *subtype;    /* Owned copy, may be NULL */
    uint32_t  layer;
    uint32_t  rank;
    int32_t   min_zoom;   /* cartography.min_zoom (-1 = no limit) */
    int32_t   max_zoom;   /* cartography.max_zoom (-1 = no limit) */
    int32_t   depth;      /* depth in meters (-1 = not bathymetry) */
} raw_feature;

/* Batch size for the work queue (number of features per batch).
   Smaller batches improve worker load balance at the cost of slightly
   more queue contention — 256 is a good balance. */
#define FEATURE_BATCH_SIZE 256

static void raw_feature_free(raw_feature *f) {
    free(f->wkb);
    free(f->type);
    free(f->subtype);
}

/* ---- Worker thread: process features → per-thread sorter ---- */

typedef struct {
    arpt_workqueue *queue;
    arpt_sorter    *sorter;
    int             min_zoom;
    int             max_zoom;
    uint64_t        feat_count;   /* Features processed by this worker */
    uint64_t        batch_count;  /* Batches processed by this worker */
} worker_ctx;

static void *worker_fn(void *arg) {
    worker_ctx *wc = (worker_ctx *)arg;
    arpt_batch batch;

    while (arpt_workqueue_pop(wc->queue, &batch)) {
        raw_feature *feats = (raw_feature *)batch.items;
        for (size_t i = 0; i < batch.count; i++) {
            raw_feature *f = &feats[i];

            /* Parse WKB in worker thread (deferred from reader) */
            arpt_geom geometry = {0};
            if (!arpt_wkb_parse(f->wkb, f->wkb_len, &geometry)) {
                raw_feature_free(f);
                continue;
            }

            double feat_bbox[4];
            if (f->has_bbox) {
                memcpy(feat_bbox, f->bbox, sizeof(feat_bbox));
            } else {
                arpt_geom_bbox(&geometry, feat_bbox);
            }

            /* Determine class: use depth for bathymetry, otherwise
               subtype (land_cover classification) or type. */
            char depth_cls[16];
            const char *cls;
            if (f->depth >= 0) {
                snprintf(depth_cls, sizeof(depth_cls), "%d", f->depth);
                cls = depth_cls;
            } else {
                cls = f->subtype ? f->subtype
                    : (f->type ? f->type : "unknown");
            }
            const char *pkeys[1] = { "class" };
            const char *pvals[1] = { cls };

            clip_ctx ctx = {
                .sorter = wc->sorter,
                .layer = f->layer,
                .rank = f->rank,
                .prop_keys = pkeys,
                .prop_vals = pvals,
                .n_props = 1,
            };

            /* Constrain zoom range using cartography metadata */
            int feat_min = wc->min_zoom;
            int feat_max = wc->max_zoom;
            if (f->min_zoom >= 0 && f->min_zoom > feat_min)
                feat_min = f->min_zoom;
            if (f->max_zoom >= 0 && f->max_zoom < feat_max)
                feat_max = f->max_zoom;

            if (feat_min <= feat_max) {
                arpt_process_feature_zooms(&geometry, feat_bbox,
                                           feat_min, feat_max,
                                           clip_cb, &ctx);
            }

            arpt_geom_free(&geometry);
            raw_feature_free(f);
        }
        wc->feat_count += batch.count;
        wc->batch_count++;
        free(feats);
    }

    return NULL;
}

/* ---- Reader: read features from one input file into the work queue ---- */

static void read_overture_input(const arpt_pipeline_input *inp,
                                const double config_bbox[4],
                                int min_zoom, int max_zoom,
                                arpt_workqueue *queue,
                                uint32_t *rank) {
    double t0 = now_sec();
    fprintf(stderr, "Reading %s (layer %u)...\n", inp->path, inp->layer);

    arpt_overture *ov = arpt_overture_open(inp->path);
    if (!ov) {
        fprintf(stderr, "Warning: cannot open %s, skipping\n", inp->path);
        return;
    }

    uint64_t feat_count = 0;
    uint64_t skip_bbox = 0, skip_zoom = 0;
    uint64_t depth_hist[32] = {0};  /* histogram by depth/1000 */
    uint64_t depth_count = 0;
    raw_feature *batch_buf = malloc(FEATURE_BATCH_SIZE * sizeof(raw_feature));
    if (!batch_buf) { arpt_overture_close(ov); return; }
    size_t batch_pos = 0;

    arpt_overture_feature feat;
    while (arpt_overture_next(ov, &feat)) {
        /* Bbox filter — skip features outside the pipeline bounds */
        if (feat.has_bbox) {
            if (feat.bbox[2] < config_bbox[0] ||
                feat.bbox[0] > config_bbox[2] ||
                feat.bbox[3] < config_bbox[1] ||
                feat.bbox[1] > config_bbox[3]) {
                skip_bbox++;
                continue;
            }
        }

        /* Zoom filter — skip features whose cartography zoom range
         * doesn't overlap the pipeline zoom range.  Avoids copying
         * WKB for features that will never be processed. */
        if (feat.min_zoom >= 0 && feat.min_zoom > max_zoom) {
            skip_zoom++;
            continue;
        }
        if (feat.max_zoom >= 0 && feat.max_zoom < min_zoom) {
            skip_zoom++;
            continue;
        }

        /* Copy raw WKB bytes — parsing is deferred to worker threads */
        raw_feature *rf = &batch_buf[batch_pos];
        rf->wkb = malloc(feat.wkb_len);
        if (!rf->wkb) continue;
        memcpy(rf->wkb, feat.wkb, feat.wkb_len);
        rf->wkb_len = feat.wkb_len;
        memcpy(rf->bbox, feat.bbox, sizeof(rf->bbox));
        rf->has_bbox = feat.has_bbox;
        rf->type = feat.type ? strdup(feat.type) : NULL;
        rf->subtype = feat.subtype ? strdup(feat.subtype) : NULL;
        rf->layer = inp->layer;
        rf->min_zoom = feat.min_zoom;
        rf->max_zoom = feat.max_zoom;
        rf->depth = feat.depth;
        if (feat.depth >= 0) {
            int bucket = feat.depth < 32000 ? feat.depth / 1000 : 31;
            depth_hist[bucket]++;
            depth_count++;
        }

        /* Use cartography.sort_key as rank when available. */
        if (feat.sort_key > 0) {
            rf->rank = (uint32_t)feat.sort_key & SORT_KEY_RANK_MASK;
        } else {
            rf->rank = *rank;
            (*rank)++;
            if (*rank > SORT_KEY_RANK_MASK) *rank = SORT_KEY_RANK_MASK;
        }
        batch_pos++;
        feat_count++;

        if (batch_pos == FEATURE_BATCH_SIZE) {
            arpt_batch b = { .items = batch_buf, .count = batch_pos };
            if (!arpt_workqueue_push(queue, &b)) {
                /* Queue closed — free remaining features */
                for (size_t i = 0; i < batch_pos; i++)
                    raw_feature_free(&batch_buf[i]);
                free(batch_buf);
                batch_buf = NULL;
                break;
            }
            /* Push took ownership; allocate new buffer */
            batch_buf = malloc(FEATURE_BATCH_SIZE * sizeof(raw_feature));
            if (!batch_buf) break;
            batch_pos = 0;
        }

        if (feat_count % 100000 == 0) {
            fprintf(stderr, "  ... %llu features\n",
                    (unsigned long long)feat_count);
        }
    }

    /* Flush remaining features */
    if (batch_buf && batch_pos > 0) {
        arpt_batch b = { .items = batch_buf, .count = batch_pos };
        if (!arpt_workqueue_push(queue, &b)) {
            for (size_t i = 0; i < batch_pos; i++)
                raw_feature_free(&batch_buf[i]);
            free(batch_buf);
        }
    } else {
        free(batch_buf);
    }

    arpt_overture_close(ov);
    double t1 = now_sec();
    fprintf(stderr, "  %llu features from %s (%.3fs, skipped: %llu bbox, %llu zoom)\n",
            (unsigned long long)feat_count, inp->path, t1 - t0,
            (unsigned long long)skip_bbox, (unsigned long long)skip_zoom);
    if (depth_count > 0) {
        fprintf(stderr, "  Depth histogram (%llu features with depth):\n",
                (unsigned long long)depth_count);
        for (int i = 0; i < 32; i++) {
            if (depth_hist[i] > 0)
                fprintf(stderr, "    depth %d-%d: %llu\n",
                        i * 1000, (i + 1) * 1000 - 1,
                        (unsigned long long)depth_hist[i]);
        }
    }
}

/* ---- Finalize sort threads ---- */

static void *finish_sorter_fn(void *arg) {
    arpt_sorter *s = (arpt_sorter *)arg;
    arpt_sorter_finish(s);
    return NULL;
}

/* ---- Phase 3 parallel tile encoding types ---- */

/* A tile ready to be encoded (pushed from grouper to encoder threads). */
typedef struct {
    arpt_tile_builder *builder;
    uint64_t           sequence;
    int                z;
    uint32_t           x, y;
} tile_job;

/* Push a tile builder to the encoder queue.  Takes ownership of builder
   (frees it on failure).  Returns the next sequence number. */
static uint64_t push_tile_job(arpt_workqueue *queue,
                              arpt_tile_builder *builder,
                              uint64_t sequence,
                              int z, uint32_t x, uint32_t y) {
    tile_job *job = malloc(sizeof(tile_job));
    if (job) {
        job->builder = builder;
        job->sequence = sequence;
        job->z = z;
        job->x = x;
        job->y = y;
        arpt_batch b = { .items = job, .count = 1 };
        if (!arpt_workqueue_push(queue, &b)) {
            arpt_tile_builder_free(builder);
            free(job);
        }
    } else {
        arpt_tile_builder_free(builder);
    }
    return sequence + 1;
}

/* An encoded tile ready to be written (pushed from encoder to writer). */
typedef struct {
    uint64_t  sequence;
    uint8_t   z;
    uint32_t  x, y;
    void     *data;    /* Compressed tile blob (owned) */
    size_t    size;
} encoded_tile;

/* Encoder thread context */
typedef struct {
    arpt_workqueue *tile_queue;    /* Input: tile_job batches */
    arpt_workqueue *write_queue;   /* Output: encoded_tile batches */
} encoder_ctx;

static void *encoder_fn(void *arg) {
    encoder_ctx *ec = (encoder_ctx *)arg;
    arpt_batch batch;

    while (arpt_workqueue_pop(ec->tile_queue, &batch)) {
        tile_job *jobs = (tile_job *)batch.items;

        /* Encode each tile and push results one at a time */
        for (size_t i = 0; i < batch.count; i++) {
            tile_job *j = &jobs[i];
            size_t tile_size = 0;
            void *tile_data = NULL;

            if (j->builder) {
                uint32_t coords = arpt_tile_builder_total_coords(j->builder);
                tile_data = arpt_tile_builder_finish(j->builder, &tile_size);
                if (coords > 500000 || tile_size > 512 * 1024) {
                    fprintf(stderr, "[TILER] tile %d/%u/%u: "
                            "coords=%u, compressed=%zu bytes\n",
                            j->z, j->x, j->y, coords, tile_size);
                }
                arpt_tile_builder_free(j->builder);
            }

            encoded_tile *et = malloc(sizeof(encoded_tile));
            if (et) {
                et->sequence = j->sequence;
                et->z = (uint8_t)j->z;
                et->x = j->x;
                et->y = j->y;
                et->data = tile_data;
                et->size = tile_size;

                arpt_batch out = { .items = et, .count = 1 };
                if (!arpt_workqueue_push(ec->write_queue, &out)) {
                    free(tile_data);
                    free(et);
                }
            } else {
                free(tile_data);
            }
        }
        free(jobs);
    }

    return NULL;
}

/* Writer thread context — receives encoded tiles and writes in sequence order */
typedef struct {
    arpt_workqueue      *write_queue;
    arpt_archive_writer *writer;
    uint64_t             tile_count;
    bool                 write_failed;
} writer_ctx;

/* Min-heap for reordering encoded tiles by sequence number */
typedef struct {
    encoded_tile **tiles;
    size_t         size;
    size_t         cap;
} reorder_heap;

static void reorder_sift_up(reorder_heap *h, size_t pos) {
    while (pos > 0) {
        size_t parent = (pos - 1) / 2;
        if (h->tiles[pos]->sequence >= h->tiles[parent]->sequence) break;
        encoded_tile *tmp = h->tiles[pos];
        h->tiles[pos] = h->tiles[parent];
        h->tiles[parent] = tmp;
        pos = parent;
    }
}

static void reorder_sift_down(reorder_heap *h, size_t pos) {
    while (1) {
        size_t smallest = pos;
        size_t left = 2 * pos + 1;
        size_t right = 2 * pos + 2;
        if (left < h->size &&
            h->tiles[left]->sequence < h->tiles[smallest]->sequence)
            smallest = left;
        if (right < h->size &&
            h->tiles[right]->sequence < h->tiles[smallest]->sequence)
            smallest = right;
        if (smallest == pos) break;
        encoded_tile *tmp = h->tiles[pos];
        h->tiles[pos] = h->tiles[smallest];
        h->tiles[smallest] = tmp;
        pos = smallest;
    }
}

static void reorder_push(reorder_heap *h, encoded_tile *et) {
    if (h->size == h->cap) {
        size_t nc = h->cap ? h->cap * 2 : 64;
        encoded_tile **p = realloc(h->tiles, nc * sizeof(*p));
        if (!p) return;
        h->tiles = p;
        h->cap = nc;
    }
    h->tiles[h->size] = et;
    reorder_sift_up(h, h->size);
    h->size++;
}

static encoded_tile *reorder_peek(reorder_heap *h) {
    return h->size > 0 ? h->tiles[0] : NULL;
}

static encoded_tile *reorder_pop(reorder_heap *h) {
    if (h->size == 0) return NULL;
    encoded_tile *top = h->tiles[0];
    h->size--;
    if (h->size > 0) {
        h->tiles[0] = h->tiles[h->size];
        reorder_sift_down(h, 0);
    }
    return top;
}

static void write_tile_to_archive(writer_ctx *wc, encoded_tile *et) {
    if (et->data && et->size > 0) {
        if (!arpt_archive_writer_add_tile(wc->writer, et->z, et->x, et->y,
                                          et->data, et->size)) {
            wc->write_failed = true;
        }
        wc->tile_count++;
    }
    free(et->data);
    free(et);
}

static void *writer_fn(void *arg) {
    writer_ctx *wc = (writer_ctx *)arg;
    reorder_heap heap = {0};
    uint64_t next_seq = 0;

    arpt_batch batch;
    while (arpt_workqueue_pop(wc->write_queue, &batch)) {
        encoded_tile *et = (encoded_tile *)batch.items;
        reorder_push(&heap, et);

        /* Drain any tiles that are now in order */
        while (reorder_peek(&heap) &&
               reorder_peek(&heap)->sequence == next_seq) {
            encoded_tile *ready = reorder_pop(&heap);
            write_tile_to_archive(wc, ready);
            next_seq++;
        }
    }

    /* Drain remaining tiles in the heap */
    while (heap.size > 0) {
        encoded_tile *ready = reorder_pop(&heap);
        write_tile_to_archive(wc, ready);
    }

    free(heap.tiles);
    return NULL;
}

/* ---- Pipeline ----
 *
 * Three-phase parallel pipeline:
 *   Phase 1: Reader thread(s) → work queue → N worker threads (per-thread sorters)
 *   Phase 2: Parallel sorter finalization → k-way merge across all sorters
 *   Phase 3: Merged stream → group by tile → build → compress → write archive
 */

bool arpt_pipeline_run(const arpt_pipeline_config *config) {
    if (!config || !config->output) return false;

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

    int n_threads = config->n_threads > 0 ? config->n_threads : detect_cpu_count();
    if (n_threads < 1) n_threads = 1;

    const char *tmp_dir = config->tmp_dir ? config->tmp_dir : "/tmp";
    size_t mem_budget = config->mem_budget > 0
        ? config->mem_budget : (size_t)256 * 1024 * 1024;

    fprintf(stderr, "Pipeline: %d worker threads, %zu MB sort budget\n",
            n_threads, mem_budget / (1024 * 1024));

    double t_start = now_sec();

    /* ---- Phase 1: Read & Process ---- */

    /* Create per-worker sorters with budget split across workers */
    size_t per_worker_budget = mem_budget / (size_t)n_threads;
    if (per_worker_budget < 64 * 1024) per_worker_budget = 64 * 1024;

    arpt_sorter **sorters = calloc((size_t)n_threads, sizeof(arpt_sorter *));
    worker_ctx *wctxs = calloc((size_t)n_threads, sizeof(worker_ctx));
    pthread_t *workers = calloc((size_t)n_threads, sizeof(pthread_t));
    bool *worker_started = NULL;
    arpt_workqueue *queue = NULL;
    if (!sorters || !wctxs || !workers) goto fail_alloc;

    queue = arpt_workqueue_create((size_t)(2 * n_threads));
    if (!queue) goto fail_alloc;

    worker_started = calloc((size_t)n_threads, sizeof(bool));
    if (!worker_started) goto fail_alloc;

    for (int i = 0; i < n_threads; i++) {
        sorters[i] = arpt_sorter_create(tmp_dir, per_worker_budget);
        if (!sorters[i]) goto fail_phase1;
    }

    for (int i = 0; i < n_threads; i++) {
        wctxs[i] = (worker_ctx){
            .queue = queue,
            .sorter = sorters[i],
            .min_zoom = min_zoom,
            .max_zoom = max_zoom,
        };
        if (pthread_create(&workers[i], NULL, worker_fn, &wctxs[i]) == 0) {
            worker_started[i] = true;
        }
    }

    /* Read GeoParquet inputs (reader on main thread, workers process) */
    uint32_t rank = 0;
    for (int fi = 0; fi < config->n_inputs; fi++) {
        read_overture_input(&config->inputs[fi], config->bbox,
                           min_zoom, max_zoom, queue, &rank);
    }

    /* Close the queue — workers will drain remaining batches and exit */
    arpt_workqueue_close(queue);

    /* Join worker threads */
    for (int i = 0; i < n_threads; i++) {
        if (worker_started[i])
            pthread_join(workers[i], NULL);
    }

    double t_phase1 = now_sec();
    {
        uint64_t total_feats = 0;
        for (int i = 0; i < n_threads; i++)
            total_feats += wctxs[i].feat_count;
        fprintf(stderr, "Phase 1 (read+process): %.3fs, %llu features\n",
                t_phase1 - t_start, (unsigned long long)total_feats);
        if (n_threads > 1) {
            for (int i = 0; i < n_threads; i++) {
                fprintf(stderr, "  worker %d: %llu features, %llu batches\n",
                        i, (unsigned long long)wctxs[i].feat_count,
                        (unsigned long long)wctxs[i].batch_count);
            }
        }
    }

    free(worker_started);
    worker_started = NULL;

    arpt_workqueue_free(queue);
    queue = NULL;

    /* ---- Phase 2: Parallel sort finalization ---- */

    fprintf(stderr, "Phase 2 (sort): finalizing %d sorters...\n", n_threads);

    if (n_threads == 1) {
        if (!arpt_sorter_finish(sorters[0])) goto fail_sort;
    } else {
        pthread_t *finish_threads = calloc((size_t)n_threads, sizeof(pthread_t));
        if (!finish_threads) goto fail_sort;

        for (int i = 0; i < n_threads; i++) {
            pthread_create(&finish_threads[i], NULL, finish_sorter_fn, sorters[i]);
        }
        for (int i = 0; i < n_threads; i++) {
            pthread_join(finish_threads[i], NULL);
        }
        free(finish_threads);
    }

    /* Create k-way merger across all per-thread sorters */
    arpt_sort_merger *merger = arpt_sort_merger_create(sorters, n_threads);
    if (!merger) goto fail_sort;

    double t_phase2 = now_sec();
    fprintf(stderr, "Phase 2 (sort): %.3fs\n", t_phase2 - t_phase1);

    /* ---- Phase 3: Parallel Build & Write ----
     *
     * Grouper (main thread) reads from merger, groups records by tile_id,
     * builds tile_builder with deserialized features, pushes to tile queue.
     * N encoder threads pop builders, finish+compress, push to write queue.
     * Writer thread receives encoded tiles and writes in sequence order. */

    arpt_archive_config arc = {
        .path = config->output,
        .min_zoom = (uint8_t)min_zoom,
        .max_zoom = (uint8_t)max_zoom,
        .bounds = { config->bbox[0], config->bbox[1],
                    config->bbox[2], config->bbox[3] },
    };
    arpt_archive_writer *writer = arpt_archive_writer_create(&arc);
    if (!writer) {
        arpt_sort_merger_free(merger);
        goto fail_sort;
    }

    /* Create tile queue and write queue */
    arpt_workqueue *tile_queue = arpt_workqueue_create((size_t)(2 * n_threads));
    arpt_workqueue *write_queue = arpt_workqueue_create((size_t)(4 * n_threads));
    if (!tile_queue || !write_queue) {
        if (tile_queue) arpt_workqueue_free(tile_queue);
        if (write_queue) arpt_workqueue_free(write_queue);
        arpt_archive_writer_free(writer);
        arpt_sort_merger_free(merger);
        goto fail_sort;
    }

    /* Start encoder threads */
    encoder_ctx *ectxs = calloc((size_t)n_threads, sizeof(encoder_ctx));
    pthread_t *encoders = calloc((size_t)n_threads, sizeof(pthread_t));
    if (!ectxs || !encoders) {
        free(ectxs); free(encoders);
        arpt_workqueue_free(tile_queue);
        arpt_workqueue_free(write_queue);
        arpt_archive_writer_free(writer);
        arpt_sort_merger_free(merger);
        goto fail_sort;
    }

    for (int i = 0; i < n_threads; i++) {
        ectxs[i] = (encoder_ctx){
            .tile_queue = tile_queue,
            .write_queue = write_queue,
        };
        pthread_create(&encoders[i], NULL, encoder_fn, &ectxs[i]);
    }

    /* Start writer thread */
    writer_ctx wctx = {
        .write_queue = write_queue,
        .writer = writer,
        .tile_count = 0,
    };
    pthread_t writer_thread;
    pthread_create(&writer_thread, NULL, writer_fn, &wctx);

    /* Grouper: read from merger, group by tile, push to tile queue.
     * Also collect feature tile IDs (already sorted by the merger)
     * for Phase 3b empty-tile detection. */
    uint64_t cur_tile_id = UINT64_MAX;
    arpt_tile_builder *builder = NULL;
    int cur_z = 0, cur_x = 0, cur_y = 0;
    uint64_t record_count = 0;
    uint64_t sequence = 0;

    /* Feature tile ID set — built during grouping, already sorted. */
    uint64_t *feature_tile_ids = NULL;
    size_t feature_tile_count = 0;
    size_t feature_tile_cap = 0;

    uint64_t key;
    const void *data;
    size_t data_size;

    while (arpt_sort_merger_next(merger, &key, &data, &data_size)) {
        record_count++;
        uint64_t tid = sort_key_tile_id(key);

        if (tid != cur_tile_id) {
            /* Push previous tile to encoder queue */
            if (builder) {
                sequence = push_tile_job(tile_queue, builder, sequence,
                                         cur_z, (uint32_t)cur_x, (uint32_t)cur_y);
            }
            builder = NULL;

            cur_tile_id = tid;
            arpt_hilbert_tile_id_decode(tid, &cur_z, &cur_x, &cur_y);
            arpt_bounds tb = arpt_tile_bounds(cur_z, cur_x, cur_y);
            builder = arpt_tile_builder_create(tb);

            /* Track feature tile ID (already in sorted order from merger) */
            if (feature_tile_count == feature_tile_cap) {
                size_t nc = feature_tile_cap ? feature_tile_cap * 2 : 4096;
                uint64_t *tmp = realloc(feature_tile_ids, nc * sizeof(*tmp));
                if (tmp) { feature_tile_ids = tmp; feature_tile_cap = nc; }
            }
            if (feature_tile_count < feature_tile_cap)
                feature_tile_ids[feature_tile_count++] = tid;
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

    /* Push last tile */
    if (builder) {
        sequence = push_tile_job(tile_queue, builder, sequence,
                                 cur_z, (uint32_t)cur_x, (uint32_t)cur_y);
    }

    arpt_sort_merger_free(merger);
    merger = NULL;

    double t_phase3_tiles = now_sec();
    fprintf(stderr, "Phase 3a (build tiles): %.3fs, %llu records -> %llu tiles (queued)\n",
            t_phase3_tiles - t_phase2,
            (unsigned long long)record_count,
            (unsigned long long)sequence);

    /* Drain the feature tile pipeline before writing empty tiles. */

    arpt_workqueue_close(tile_queue);
    for (int i = 0; i < n_threads; i++)
        pthread_join(encoders[i], NULL);
    arpt_workqueue_close(write_queue);
    pthread_join(writer_thread, NULL);

    arpt_workqueue_free(tile_queue);
    arpt_workqueue_free(write_queue);
    free(ectxs);
    free(encoders);

    fprintf(stderr, "Phase 3a (encode+write): %.3fs, %llu tiles written\n",
            now_sec() - t_phase2, (unsigned long long)wctx.tile_count);

    /* ---- Phase 3b: Fill empty tiles (terrain-only) ----
     *
     * Empty tiles are pre-compressed, so write them directly to the
     * archive — no encoder or writer threads needed.  The feature tile
     * ID set was built during grouping (already sorted by the merger). */

    double t_phase3b_start = now_sec();
    uint64_t empty_count = 0;

    /* Cache one empty tile per zoom (flat terrain, no features) */
    void  *cached_data = NULL;
    size_t cached_size = 0;
    int    cached_zoom = -1;

    for (int z = min_zoom; z <= max_zoom; z++) {
        int n_cols = 1 << z;
        int n_rows = 1 << z;
        double lon_span = 360.0 / (double)n_cols;
        double lat_span = 180.0 / (double)n_rows;

        int x_min = (int)floor((config->bbox[0] + 180.0) / lon_span);
        int x_max = (int)floor((config->bbox[2] + 180.0) / lon_span);
        int y_min = (int)floor((config->bbox[1] + 90.0) / lat_span);
        int y_max = (int)floor((config->bbox[3] + 90.0) / lat_span);
        if (x_min < 0) x_min = 0;
        if (x_max >= n_cols) x_max = n_cols - 1;
        if (y_min < 0) y_min = 0;
        if (y_max >= n_rows) y_max = n_rows - 1;

        if (cached_zoom != z) {
            free(cached_data);
            cached_data = NULL;
            cached_size = 0;
            cached_zoom = z;
            arpt_bounds tb = arpt_tile_bounds(z, x_min, y_min);
            arpt_tile_builder *eb = arpt_tile_builder_create(tb);
            if (eb) {
                cached_data = arpt_tile_builder_finish(eb, &cached_size);
                arpt_tile_builder_free(eb);
            }
        }

        if (!cached_data || cached_size == 0)
            continue;

        for (int y = y_min; y <= y_max; y++) {
            for (int x = x_min; x <= x_max; x++) {
                uint64_t tid = arpt_hilbert_tile_id(z, x, y);

                /* Binary search in the sorted feature tile ID set */
                size_t lo = 0, hi = feature_tile_count;
                while (lo < hi) {
                    size_t mid = lo + (hi - lo) / 2;
                    if (feature_tile_ids[mid] < tid) lo = mid + 1;
                    else hi = mid;
                }
                if (lo < feature_tile_count && feature_tile_ids[lo] == tid)
                    continue;

                arpt_archive_writer_add_tile(writer, (uint8_t)z,
                                             (uint32_t)x, (uint32_t)y,
                                             cached_data, cached_size);
                empty_count++;
            }
        }
    }

    free(cached_data);
    free(feature_tile_ids);

    if (empty_count > 0) {
        fprintf(stderr, "Added %llu empty tiles\n",
                (unsigned long long)empty_count);
    }

    double t_phase3_fill = now_sec();
    fprintf(stderr, "Phase 3b (fill empty): %.3fs\n", t_phase3_fill - t_phase3b_start);

    if (wctx.write_failed) {
        fprintf(stderr, "Warning: some tile writes failed during encoding\n");
    }

    bool ok = arpt_archive_writer_finish(writer);
    arpt_archive_writer_free(writer);

    double t_end = now_sec();
    fprintf(stderr, "Phase 3c (finalize): %.3fs\n", t_end - t_phase3_fill);
    fprintf(stderr, "Total: %.3fs\n", t_end - t_start);

    for (int i = 0; i < n_threads; i++)
        arpt_sorter_free(sorters[i]);
    free(sorters);
    free(wctxs);
    free(workers);

    if (ok && !wctx.write_failed) {
        fprintf(stderr, "Archive written: %s\n", config->output);
    } else if (!ok) {
        fprintf(stderr, "Error: archive finalization failed\n");
    }

    return ok && !wctx.write_failed;

    /* ---- Error paths ---- */

fail_phase1:
    if (queue) arpt_workqueue_close(queue);
    if (worker_started) {
        for (int i = 0; i < n_threads; i++) {
            if (worker_started[i])
                pthread_join(workers[i], NULL);
        }
    }
    if (queue) arpt_workqueue_free(queue);
    /* fall through */

fail_alloc:
    free(worker_started);
    if (sorters) {
        for (int i = 0; i < n_threads; i++) {
            if (sorters[i]) arpt_sorter_free(sorters[i]);
        }
    }
    free(sorters);
    free(wctxs);
    free(workers);
    return false;

fail_sort:
    for (int i = 0; i < n_threads; i++) {
        if (sorters[i]) arpt_sorter_free(sorters[i]);
    }
    free(sorters);
    free(wctxs);
    free(workers);
    return false;
}
