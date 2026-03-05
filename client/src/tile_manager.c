#include "tile_manager.h"
#include "renderer.h"
#include "style.h"
#include "tile_decode.h"
#include "tile_fetch.h"
#include "globe.h"
#include "hashmap.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_VISIBLE_TILES 256
#define MAX_RETRIES 3

/* Internal types */

typedef enum {
    TILE_EMPTY = 0,
    TILE_LOADING,
    TILE_READY,
    TILE_FAILED,
} tile_state_t;

typedef struct {
    arpt_tile_key key;
    tile_state_t state;
    arpt_tile_gpu *gpu;
    uint64_t last_used;
    arpt_bounds bounds;
    double center_lon_rad;
    double center_lat_rad;
    double avg_elevation;
    int retries;
    uint64_t retry_after;
} tile_entry;

struct arpt_tile_manager {
    arpt_tile_manager_config config;
    arpt_renderer *renderer;
    struct hashmap *cache;
    uint64_t frame;
    int active_fetches;

    /* Tree class names from style (for tree decode). Copies stored inline. */
    char tree_class_names_buf[ARPT_MAX_TREE_STYLES][32];
    const char *tree_class_names[ARPT_MAX_TREE_STYLES];
    int tree_class_count;

    /* Cached per-frame visible tile list (computed once in update, reused in
     * draw) */
    arpt_tile_key visible[MAX_VISIBLE_TILES];
    int visible_count;
    int visible_level;

    /* Ground elevation at camera position (updated each frame) */
    double ground_elevation;
};

/* Hashmap callbacks */

static uint64_t tile_hash(const void *item, uint64_t seed0, uint64_t seed1) {
    const tile_entry *e = item;
    /* Pack key into bytes for hashing */
    uint8_t buf[12];
    memcpy(buf + 0, &e->key.level, 4);
    memcpy(buf + 4, &e->key.x, 4);
    memcpy(buf + 8, &e->key.y, 4);
    return hashmap_xxhash3(buf, sizeof(buf), seed0, seed1);
}

static int tile_compare(const void *a, const void *b, void *udata) {
    (void)udata;
    const tile_entry *ea = a;
    const tile_entry *eb = b;
    if (ea->key.level != eb->key.level) return ea->key.level - eb->key.level;
    if (ea->key.x != eb->key.x) return ea->key.x - eb->key.x;
    return ea->key.y - eb->key.y;
}

static void tile_entry_free(void *item) {
    tile_entry *e = item;
    if (e->gpu) arpt_tile_gpu_free(e->gpu);
}

static void tm_hashmap_set(arpt_tile_manager *tm, const tile_entry *entry) {
    const tile_entry *replaced = hashmap_set(tm->cache, entry);
    if (replaced && replaced->gpu && replaced->gpu != entry->gpu) {
        arpt_tile_gpu_free(replaced->gpu);
    }
}

static void tm_hashmap_delete(arpt_tile_manager *tm, const tile_entry *entry) {
    const tile_entry *removed = hashmap_delete(tm->cache, entry);
    if (removed && removed->gpu) {
        arpt_tile_gpu_free(removed->gpu);
    }
}

/* Fetch callback */

typedef struct {
    arpt_tile_manager *tm;
    arpt_tile_key key;
    int retries;
} fetch_ctx;

static void on_tile_fetched(bool success, uint8_t *flatbuf, size_t size,
                            void *userdata) {
    fetch_ctx *ctx = userdata;
    arpt_tile_manager *tm = ctx->tm;
    arpt_tile_key key = ctx->key;
    int retries = ctx->retries;
    free(ctx);

    tm->active_fetches--;

    /* Look up the entry; it may have been evicted while loading */
    tile_entry lookup = {.key = key};
    const tile_entry *existing = hashmap_get(tm->cache, &lookup);
    if (!existing || existing->state != TILE_LOADING) {
        free(flatbuf);
        return;
    }

    tile_entry updated = *existing;

    if (!success) {
        updated.state = TILE_FAILED;
        updated.retries = retries + 1;
        updated.retry_after = tm->frame + (1u << updated.retries);
        tm_hashmap_set(tm, &updated);
        return;
    }

    arpt_terrain_mesh mesh = {0};
    if (!arpt_decode_terrain(flatbuf, size, &mesh)) {
        updated.state = TILE_FAILED;
        updated.retries = MAX_RETRIES; /* decode error is permanent */
        tm_hashmap_set(tm, &updated);
        free(flatbuf);
        return;
    }

    /* Compute average elevation (mm → m) for terrain awareness */
    if (mesh.vertex_count > 0 && mesh.z) {
        double sum = 0.0;
        for (size_t v = 0; v < mesh.vertex_count; v++)
            sum += mesh.z[v];
        updated.avg_elevation = (sum / (double)mesh.vertex_count) * 0.001;
    }

    arpt_surface_data surface = {0};
    arpt_decode_surface(flatbuf, size, &surface);

    arpt_highway_data highways = {0};
    arpt_decode_highways(flatbuf, size, &highways);

    arpt_surface_data buildings = {0};
    if (key.level >= 13)
        arpt_decode_buildings(flatbuf, size, &buildings);

    arpt_tree_data trees = {0};
    if (key.level >= 13)
        arpt_decode_trees(flatbuf, size, tm->tree_class_names,
                          tm->tree_class_count, &trees);

    /* wgpuQueueWriteBuffer copies synchronously, safe to free after */
    updated.gpu = arpt_renderer_upload_tile(tm->renderer, &mesh, &surface,
                                            &highways, &buildings, &trees,
                                            updated.bounds);
    updated.state = updated.gpu ? TILE_READY : TILE_FAILED;
    tm_hashmap_set(tm, &updated);
    arpt_surface_data_free(&surface);
    arpt_highway_data_free(&highways);
    arpt_surface_data_free(&buildings);
    arpt_tree_data_free(&trees);
    free(flatbuf);
}

/* LRU eviction */

static void evict_oldest(arpt_tile_manager *tm) {
    size_t count = hashmap_count(tm->cache);
    if ((int)count <= tm->config.max_tiles) return;

    size_t to_evict = count - (size_t)tm->config.max_tiles;
    for (size_t e = 0; e < to_evict; e++) {
        /* Find the least recently used READY or FAILED entry */
        uint64_t oldest_frame = UINT64_MAX;
        arpt_tile_key oldest_key = {0};
        bool found = false;

        size_t iter = 0;
        void *item;
        while (hashmap_iter(tm->cache, &iter, &item)) {
            tile_entry *entry = item;
            if (entry->state == TILE_LOADING)
                continue; /* don't evict in-flight */
            if (entry->last_used < oldest_frame) {
                oldest_frame = entry->last_used;
                oldest_key = entry->key;
                found = true;
            }
        }

        if (!found) break;
        tile_entry del = {.key = oldest_key};
        tm_hashmap_delete(tm, &del);
    }
}

/* Public API */

arpt_tile_manager *arpt_tile_manager_create(arpt_tile_manager_config config,
                                            arpt_renderer *r,
                                            const arpt_style *style) {
    arpt_tile_manager *tm = calloc(1, sizeof(*tm));
    if (!tm) return NULL;

    tm->config = config;
    tm->renderer = r;

    /* Cache tree class names from style for decode-time mapping. */
    if (style) {
        tm->tree_class_count = style->tree_style_count;
        for (int i = 0; i < style->tree_style_count; i++) {
            strncpy(tm->tree_class_names_buf[i], style->trees[i].class_name,
                    sizeof(tm->tree_class_names_buf[i]) - 1);
            tm->tree_class_names_buf[i][31] = '\0';
            tm->tree_class_names[i] = tm->tree_class_names_buf[i];
        }
    }
    tm->cache = hashmap_new(sizeof(tile_entry), 64, 0, 0, tile_hash,
                            tile_compare, tile_entry_free, NULL);
    if (!tm->cache) {
        free(tm);
        return NULL;
    }

    if (!arpt_fetch_init(config.max_concurrent)) {
        hashmap_free(tm->cache);
        free(tm);
        return NULL;
    }

    return tm;
}

void arpt_tile_manager_free(arpt_tile_manager *tm) {
    if (!tm) return;
    arpt_fetch_shutdown();
    hashmap_free(tm->cache);
    free(tm);
}

/* Start a fetch for a tile key, inserting a LOADING entry into the cache.
   prev_retries is the retry count carried from a previous failed attempt (0 for
   new). */
static void start_fetch(arpt_tile_manager *tm, arpt_tile_key key,
                        int prev_retries) {
    arpt_bounds bounds = arpt_tile_bounds(key.level, key.x, key.y);
    double center_lon = (bounds.west + bounds.east) / 2.0 * M_PI / 180.0;
    double center_lat = (bounds.south + bounds.north) / 2.0 * M_PI / 180.0;

    tile_entry new_entry = {
        .key = key,
        .state = TILE_LOADING,
        .gpu = NULL,
        .last_used = tm->frame,
        .bounds = bounds,
        .center_lon_rad = center_lon,
        .center_lat_rad = center_lat,
        .retries = prev_retries,
    };
    tm_hashmap_set(tm, &new_entry);

    fetch_ctx *ctx = malloc(sizeof(*ctx));
    if (!ctx) return;
    ctx->tm = tm;
    ctx->key = key;
    ctx->retries = prev_retries;

    tm->active_fetches++;
    if (!arpt_fetch_tile(tm->config.base_url, key.level, key.x, key.y,
                         on_tile_fetched, ctx)) {
        tm->active_fetches--;
        tile_entry failed = new_entry;
        failed.state = TILE_FAILED;
        failed.retries = prev_retries + 1;
        failed.retry_after = tm->frame + (1u << failed.retries);
        tm_hashmap_set(tm, &failed);
        free(ctx);
    }
}

void arpt_tile_manager_update(arpt_tile_manager *tm, const arpt_camera *cam) {
    arpt_fetch_drain();
    tm->frame++;

    /* Pre-fetch level-0 root tiles on the first frame so that
       draw always has a fallback level to render. */
    if (tm->frame == 1) {
        arpt_tile_key roots[2] = {{0, 0, 0}, {0, 1, 0}};
        for (int r = 0; r < 2; r++) {
            tile_entry lookup = {.key = roots[r]};
            if (!hashmap_get(tm->cache, &lookup)) {
                start_fetch(tm, roots[r], 0);
            }
        }
    }

    int level = arpt_camera_zoom_level(
        cam, tm->config.root_error, tm->config.min_level, tm->config.max_level);

    tm->visible_level = level;
    tm->visible_count = arpt_enumerate_visible_tiles(cam, level, tm->visible,
                                                     MAX_VISIBLE_TILES);

    for (int i = 0; i < tm->visible_count; i++) {
        tile_entry lookup = {.key = tm->visible[i]};
        const tile_entry *existing = hashmap_get(tm->cache, &lookup);

        if (existing) {
            if (existing->state == TILE_FAILED) {
                if (existing->retries >= MAX_RETRIES) {
                    /* Permanently failed — stop retrying */
                    tile_entry updated = *existing;
                    updated.last_used = tm->frame;
                    tm_hashmap_set(tm, &updated);
                    continue;
                }
                if (tm->frame < existing->retry_after) {
                    /* Backoff not elapsed yet */
                    tile_entry updated = *existing;
                    updated.last_used = tm->frame;
                    tm_hashmap_set(tm, &updated);
                    continue;
                }
                /* Backoff elapsed — delete and re-fetch */
                int prev_retries = existing->retries;
                tm_hashmap_delete(tm, &lookup);

                if (tm->active_fetches < tm->config.max_concurrent)
                    start_fetch(tm, tm->visible[i], prev_retries);
                continue;
            }

            /* LOADING or READY — touch for LRU */
            tile_entry updated = *existing;
            updated.last_used = tm->frame;
            tm_hashmap_set(tm, &updated);
            continue;
        }

        /* New tile: initiate fetch if under concurrency limit */
        if (tm->active_fetches >= tm->config.max_concurrent) continue;

        start_fetch(tm, tm->visible[i], 0);
    }

    evict_oldest(tm);

    /* Update ground elevation from the highest-level READY tile under the
       camera.  We search the visible list (already computed) instead of
       scanning the full cache — O(visible) rather than O(cache). */
    double cam_lon_deg = arpt_camera_lon(cam) * 180.0 / M_PI;
    double cam_lat_deg = arpt_camera_lat(cam) * 180.0 / M_PI;
    int best_level = -1;
    for (int i = 0; i < tm->visible_count; i++) {
        if (tm->visible[i].level <= best_level) continue;
        tile_entry lookup = {.key = tm->visible[i]};
        const tile_entry *e = hashmap_get(tm->cache, &lookup);
        if (!e || e->state != TILE_READY) continue;
        if (cam_lon_deg < e->bounds.west || cam_lon_deg > e->bounds.east)
            continue;
        if (cam_lat_deg < e->bounds.south || cam_lat_deg > e->bounds.north)
            continue;
        best_level = e->key.level;
        tm->ground_elevation = e->avg_elevation;
    }
}

int arpt_tile_manager_active_fetches(const arpt_tile_manager *tm) {
    return tm ? tm->active_fetches : 0;
}

/* Ground elevation query */

double arpt_tile_manager_ground_elevation(const arpt_tile_manager *tm,
                                          double lon_rad, double lat_rad) {
    (void)lon_rad;
    (void)lat_rad;
    return tm ? tm->ground_elevation : 0.0;
}

/* Draw helpers */

static void draw_entry(arpt_renderer *r, const arpt_camera *cam,
                       const tile_entry *e) {
    arpt_mat4 model =
        arpt_camera_tile_model(cam, e->center_lon_rad, e->center_lat_rad, 0.0);
    float bounds_rad[4] = {
        (float)(e->bounds.west * M_PI / 180.0),
        (float)(e->bounds.south * M_PI / 180.0),
        (float)(e->bounds.east * M_PI / 180.0),
        (float)(e->bounds.north * M_PI / 180.0),
    };
    arpt_tile_gpu_set_uniforms((arpt_tile_gpu *)e->gpu, model, bounds_rad,
                               (float)e->center_lon_rad,
                               (float)e->center_lat_rad);
    arpt_renderer_draw_tile(r, (arpt_tile_gpu *)e->gpu);
}

void arpt_tile_manager_draw(arpt_tile_manager *tm, arpt_renderer *r,
                            const arpt_camera *cam) {
    int ph_slot = 0;
    for (int i = 0; i < tm->visible_count; i++) {
        tile_entry lookup = {.key = tm->visible[i]};
        const tile_entry *e = hashmap_get(tm->cache, &lookup);

        if (e && e->state == TILE_READY && e->gpu) {
            draw_entry(r, cam, e);
        } else if (ph_slot < ARPT_MAX_PLACEHOLDERS) {
            /* Draw flat placeholder quad while tile loads */
            arpt_bounds bounds = arpt_tile_bounds(
                tm->visible[i].level, tm->visible[i].x, tm->visible[i].y);
            double clon = (bounds.west + bounds.east) / 2.0 * M_PI / 180.0;
            double clat = (bounds.south + bounds.north) / 2.0 * M_PI / 180.0;
            arpt_mat4 model = arpt_camera_tile_model(cam, clon, clat, 0.0);
            float bounds_rad[4] = {
                (float)(bounds.west * M_PI / 180.0),
                (float)(bounds.south * M_PI / 180.0),
                (float)(bounds.east * M_PI / 180.0),
                (float)(bounds.north * M_PI / 180.0),
            };
            arpt_renderer_draw_placeholder(r, ph_slot++, model, bounds_rad,
                                           (float)clon, (float)clat);
        }
    }
}
