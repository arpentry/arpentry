#include "manager.h"
#include "renderer.h"
#include "prepare.h"
#include "style.h"
#include "decode.h"
#include "fetch.h"
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

    /* Retained terrain heightfield (moved out of the prepared tile, owned by
       this entry) so the camera's ground height can be sampled at its exact
       lon/lat.  At street-level overzoom the tile-average elevation diverges
       too far from the real surface for interaction to track it. */
    uint16_t *terr_x, *terr_y;
    int32_t *terr_z;
    uint32_t *terr_indices;
    size_t terr_vert_count, terr_index_count;
} tile_entry;

struct arpt_tile_manager {
    arpt_tile_manager_config config;
    arpt_renderer *renderer;
    arpt_style style;
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

    /* Set when a tile upload completes; cleared by needs_redraw query. */
    bool needs_redraw;
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

static void tile_entry_free_terrain(tile_entry *e) {
    free(e->terr_x);
    free(e->terr_y);
    free(e->terr_z);
    free(e->terr_indices);
    e->terr_x = NULL;
    e->terr_y = NULL;
    e->terr_z = NULL;
    e->terr_indices = NULL;
}

static void tile_entry_free(void *item) {
    tile_entry *e = item;
    if (e->gpu) arpt_tile_gpu_free(e->gpu);
    tile_entry_free_terrain(e);
}

static void tm_hashmap_set(arpt_tile_manager *tm, const tile_entry *entry) {
    const tile_entry *replaced = hashmap_set(tm->cache, entry);
    if (!replaced) return;
    /* Free the replaced entry's resources only when the new entry doesn't carry
       the same pointers (an LRU touch re-sets a copy with the same handles). */
    if (replaced->gpu && replaced->gpu != entry->gpu)
        arpt_tile_gpu_free(replaced->gpu);
    if (replaced->terr_x && replaced->terr_x != entry->terr_x) {
        tile_entry tmp = *replaced;
        tile_entry_free_terrain(&tmp);
    }
}

static void tm_hashmap_delete(arpt_tile_manager *tm, const tile_entry *entry) {
    const tile_entry *removed = hashmap_delete(tm->cache, entry);
    if (!removed) return;
    if (removed->gpu) arpt_tile_gpu_free(removed->gpu);
    if (removed->terr_x) {
        tile_entry tmp = *removed;
        tile_entry_free_terrain(&tmp);
    }
}

/* Fetch callback */

typedef struct {
    arpt_tile_manager *tm;
    arpt_tile_key key;
    int retries;
} fetch_ctx;

/* Self-contained prepared tile: the product of the worker-side decode +
   preparation pipeline, ready for GPU upload on the main thread. All arrays
   are owned by this struct so the worker can release the flatbuf before
   handing the result over. */
typedef struct {
    double avg_elevation;
    /* Terrain arrays (copied out of flatbuf so they outlive it). */
    uint16_t *terrain_x;
    uint16_t *terrain_y;
    int32_t *terrain_z;
    int8_t *terrain_normals;
    uint32_t *terrain_indices;
    size_t terrain_vertex_count;
    size_t terrain_index_count;
    /* Render primitives. prims.terrain aliases the arrays above. */
    arpt_tile_prims prims;
} prepared_tile;

static void prepared_tile_free(prepared_tile *p) {
    if (!p) return;
    arpt_tile_prims_free(&p->prims);
    free(p->terrain_x);
    free(p->terrain_y);
    free(p->terrain_z);
    free(p->terrain_normals);
    free(p->terrain_indices);
    free(p);
}

static int compare_surface_cls(const void *a, const void *b) {
    const arpt_surface_polygon *pa = (const arpt_surface_polygon *)a;
    const arpt_surface_polygon *pb = (const arpt_surface_polygon *)b;
    if (pa->cls != pb->cls) return (int)pa->cls - (int)pb->cls;
    return (int)pa->poly_id - (int)pb->poly_id;
}

static int compare_line_cls(const void *a, const void *b) {
    const arpt_line_feature *la = (const arpt_line_feature *)a;
    const arpt_line_feature *lb = (const arpt_line_feature *)b;
    return (int)la->cls - (int)lb->cls;
}

/* Guard against tiles that would exceed WebGPU buffer limits.
   Each vertex needs 4 bytes in the largest per-vertex buffer. */
#define ARPT_MAX_BUFFER_BYTES (200u * 1024u * 1024u)

/* Diagnostic (env ARPT_DUMP_BRIDGE=<path>): append a decoded bridge prim —
   the exact vertex/index arrays uploaded to the GPU — as text, so the
   client-side geometry can be rasterized offline and compared against the
   archive. One "tile" header line per prim, then "v qx qy zmm" and
   "t i0 i1 i2" lines. */
static void dump_bridge_prim(arpt_tile_key key, const arpt_building_prim *p) {
    const char *path = getenv("ARPT_DUMP_BRIDGE");
    if (!path || !p || p->vertex_count == 0) return;
    /* One file per tile: decode runs on concurrent worker threads, so a
       shared append-mode file would interleave records. */
    char full[1024];
    int n = snprintf(full, sizeof(full), "%s.%d.%d.%d", path, key.level,
                     key.x, key.y);
    if (n < 0 || (size_t)n >= sizeof(full)) return;
    FILE *f = fopen(full, "w");
    if (!f) return;
    arpt_bounds b = arpt_tile_bounds(key.level, key.x, key.y);
    fprintf(f, "tile %d %d %d %.17g %.17g %.17g %.17g %zu %zu\n",
            key.level, key.x, key.y, b.west, b.south, b.east, b.north,
            p->vertex_count, p->index_count);
    for (size_t v = 0; v < p->vertex_count; v++)
        fprintf(f, "v %u %u %d\n", (unsigned)p->xy[2 * v],
                (unsigned)p->xy[2 * v + 1], p->z[v]);
    for (size_t k = 0; k + 2 < p->index_count; k += 3)
        fprintf(f, "t %u %u %u\n", p->indices[k], p->indices[k + 1],
                p->indices[k + 2]);
    fclose(f);
}

/* Runs on a fetch worker thread: decode, prepare, and copy terrain into
   self-contained buffers. Returns a heap prepared_tile (NULL on failure).
   Consumes `flatbuf` (always freed before returning).

   Thread safety: reads only immutable fields of tm (style, tree class
   names) and of tm->renderer (font/icon glyph tables), all populated at
   init. No writes to shared state — all mutation happens later on the
   main thread in tile_finish_main. */
static void *tile_prepare_worker(uint8_t *flatbuf, size_t size,
                                  void *userdata) {
    fetch_ctx *ctx = userdata;
    arpt_tile_manager *tm = ctx->tm;
    arpt_tile_key key = ctx->key;

    arpt_terrain_mesh mesh = {0};
    if (!arpt_decode_terrain(flatbuf, size, &mesh)) {
        free(flatbuf);
        return NULL;
    }

    size_t max_vert_buf = mesh.vertex_count * 4;
    size_t max_idx_buf = mesh.index_count * sizeof(uint32_t);
    size_t max_buf = max_vert_buf > max_idx_buf ? max_vert_buf : max_idx_buf;
    if (max_buf > ARPT_MAX_BUFFER_BYTES) {
        fprintf(stderr, "[TILE] %d/%d/%d SKIPPED: oversized "
                "(verts=%zu, indices=%zu, max_buf=%zu bytes)\n",
                key.level, key.x, key.y,
                mesh.vertex_count, mesh.index_count, max_buf);
        free(flatbuf);
        return NULL;
    }
    fprintf(stderr, "[TILE] %d/%d/%d loaded: verts=%zu indices=%zu "
            "(%zu bytes decompressed)\n",
            key.level, key.x, key.y,
            mesh.vertex_count, mesh.index_count, size);

    prepared_tile *p = calloc(1, sizeof(*p));
    if (!p) {
        free(flatbuf);
        return NULL;
    }

    if (mesh.vertex_count > 0 && mesh.z) {
        double sum = 0.0;
        for (size_t v = 0; v < mesh.vertex_count; v++)
            sum += mesh.z[v];
        p->avg_elevation = (sum / (double)mesh.vertex_count) * 0.001;
    }

    arpt_surface_data surface = {0};
    arpt_line_data lines = {0};
    arpt_tree_data trees = {0};
    arpt_poi_data pois = {0};
    arpt_line_label_data line_labels = {0};

    for (int li = 0; li < tm->style.layer_count; li++) {
        const arpt_layer_entry *le = &tm->style.layers[li];
        if (key.level < le->min_level) continue;
        switch (le->type) {
        case ARPT_LAYER_TERRAIN:
            /* Already decoded above */
            break;
        case ARPT_LAYER_TEXTURE: {
            arpt_surface_data extra = {0};
            arpt_decode_surface_layer(flatbuf, size, le->source_layer,
                                      tm->style.class_names,
                                      tm->style.class_count, &extra);
            if (extra.count > 0) {
                size_t new_count = surface.count + extra.count;
                arpt_surface_polygon *merged = realloc(
                    surface.polygons,
                    new_count * sizeof(arpt_surface_polygon));
                if (merged) {
                    memcpy(merged + surface.count, extra.polygons,
                           extra.count * sizeof(arpt_surface_polygon));
                    surface.polygons = merged;
                    surface.count = new_count;
                }
            }
            arpt_surface_data_free(&extra);
            break;
        }
        case ARPT_LAYER_BUILDING:
            /* Buildings are server-baked 3D meshes, decoded straight into the
               building primitive. */
            arpt_decode_building_mesh(flatbuf, size, le->source_layer,
                                      &p->prims.buildings);
            break;
        case ARPT_LAYER_INSTANCE:
            arpt_decode_trees(flatbuf, size, le->source_layer,
                              tm->tree_class_names,
                              tm->tree_class_count, &trees);
            break;
        case ARPT_LAYER_LINE: {
            /* Bridges, tunnels, and at-grade junction plates ride this line
               layer as server-baked box/plate prisms (MeshGeometry) alongside
               the road lines; decode them once into per-kind primitives (split
               by `level`) so each colours differently and the coplanar plates
               render as decals. The line decoder below skips the meshes. */
            if (p->prims.bridges.vertex_count == 0) {
                arpt_decode_bridge_mesh(flatbuf, size, le->source_layer,
                                        tm->style.class_names,
                                        tm->style.class_count, tm->style.colors,
                                        &p->prims.bridges);
                dump_bridge_prim(key, &p->prims.bridges);
            }
            if (p->prims.tunnels.vertex_count == 0)
                arpt_decode_tunnel_mesh(flatbuf, size, le->source_layer,
                                        tm->style.class_names,
                                        tm->style.class_count, tm->style.colors,
                                        &p->prims.tunnels);
            arpt_line_data extra = {0};
            arpt_decode_lines(flatbuf, size, le->source_layer,
                              tm->style.class_names,
                              tm->style.class_count, &extra);
            if (extra.count > 0) {
                size_t new_count = lines.count + extra.count;
                arpt_line_feature *merged = realloc(
                    lines.lines,
                    new_count * sizeof(arpt_line_feature));
                if (merged) {
                    memcpy(merged + lines.count, extra.lines,
                           extra.count * sizeof(arpt_line_feature));
                    lines.lines = merged;
                    lines.count = new_count;
                }
            }
            arpt_line_data_free(&extra);
            break;
        }
        case ARPT_LAYER_LABEL:
            arpt_decode_pois(flatbuf, size, le->source_layer, &pois);
            break;
        case ARPT_LAYER_LINE_LABEL: {
            arpt_line_label_data extra = {0};
            arpt_decode_line_labels(flatbuf, size, le->source_layer, &extra);
            if (extra.count > 0) {
                size_t new_count = line_labels.count + extra.count;
                arpt_line_label_feature *merged = realloc(
                    line_labels.features,
                    new_count * sizeof(arpt_line_label_feature));
                if (merged) {
                    memcpy(merged + line_labels.count, extra.features,
                           extra.count * sizeof(arpt_line_label_feature));
                    line_labels.features = merged;
                    line_labels.count = new_count;
                }
            }
            arpt_line_label_data_free(&extra);
            break;
        }
        }
    }

    /* Filter out features whose class min_level > tile level */
    {
        const uint8_t *ml = tm->style.class_min_levels;
        size_t dst = 0;
        for (size_t i = 0; i < surface.count; i++) {
            if (key.level >= ml[surface.polygons[i].cls])
                surface.polygons[dst++] = surface.polygons[i];
        }
        surface.count = dst;
        dst = 0;
        for (size_t i = 0; i < lines.count; i++) {
            if (key.level >= ml[lines.lines[i].cls])
                lines.lines[dst++] = lines.lines[i];
        }
        lines.count = dst;
    }

    /* Sort polygons by class index so that earlier paint entries in the
       style (lower class index) draw first (bottom) and later entries
       draw on top, following MapLibre-style layer ordering. */
    if (surface.count > 1) {
        qsort(surface.polygons, surface.count,
              sizeof(arpt_surface_polygon), compare_surface_cls);
    }

    /* Same ordering for lines: list minor road classes before major ones
       in the style so the important roads draw on top. */
    if (lines.count > 1) {
        qsort(lines.lines, lines.count,
              sizeof(arpt_line_feature), compare_line_cls);
    }

    arpt_bounds bounds = arpt_tile_bounds(key.level, key.x, key.y);

    /* Copy terrain arrays out of the flatbuf so the prepared tile is fully
       self-contained and the flatbuf can be freed here on the worker. */
    p->terrain_vertex_count = mesh.vertex_count;
    p->terrain_index_count = mesh.index_count;
    if (mesh.vertex_count > 0) {
        p->terrain_x = malloc(mesh.vertex_count * sizeof(uint16_t));
        p->terrain_y = malloc(mesh.vertex_count * sizeof(uint16_t));
        p->terrain_z = malloc(mesh.vertex_count * sizeof(int32_t));
        if (!p->terrain_x || !p->terrain_y || !p->terrain_z) goto oom;
        memcpy(p->terrain_x, mesh.x, mesh.vertex_count * sizeof(uint16_t));
        memcpy(p->terrain_y, mesh.y, mesh.vertex_count * sizeof(uint16_t));
        memcpy(p->terrain_z, mesh.z, mesh.vertex_count * sizeof(int32_t));
        if (mesh.normals) {
            p->terrain_normals = malloc(mesh.vertex_count * 2);
            if (!p->terrain_normals) goto oom;
            memcpy(p->terrain_normals, mesh.normals, mesh.vertex_count * 2);
        }
    }
    if (mesh.index_count > 0) {
        p->terrain_indices = malloc(mesh.index_count * sizeof(uint32_t));
        if (!p->terrain_indices) goto oom;
        memcpy(p->terrain_indices, mesh.indices,
               mesh.index_count * sizeof(uint32_t));
    }

    p->prims.bounds = bounds;
    p->prims.terrain.x = p->terrain_x;
    p->prims.terrain.y = p->terrain_y;
    p->prims.terrain.z = p->terrain_z;
    p->prims.terrain.normals = p->terrain_normals;
    p->prims.terrain.indices = p->terrain_indices;
    p->prims.terrain.vertex_count = p->terrain_vertex_count;
    p->prims.terrain.index_count = p->terrain_index_count;

    /* The tiler writes POIs most-confident first; keep only the head of the
       list so dense city tiles don't wallpaper the view with shop labels. */
    #define ARPT_MAX_POIS_PER_TILE 24
    if (pois.count > ARPT_MAX_POIS_PER_TILE)
        pois.count = ARPT_MAX_POIS_PER_TILE;

    arpt_prepare_polygons(&surface, &tm->style, &p->prims.polygons);
    arpt_prepare_lines(&lines, &tm->style, key.level, bounds, &p->prims.lines);
    arpt_prepare_instances(&trees, arpt_renderer_model_count(tm->renderer),
                           &p->prims.instances);
    arpt_prepare_labels(&pois, arpt_renderer_font_glyphs(tm->renderer),
                        arpt_renderer_font_height(tm->renderer),
                        arpt_renderer_icon_glyphs(tm->renderer),
                        arpt_renderer_icon_count(tm->renderer),
                        arpt_renderer_icon_height(tm->renderer),
                        &p->prims.labels);
    arpt_prepare_line_labels(&line_labels, &mesh,
                             arpt_renderer_font_glyphs(tm->renderer),
                             &p->prims.line_labels);

    /* The terrain guard above only covers the mesh. A pathological tile (e.g. a
       low-zoom tile holding the whole world's line network) can tessellate into
       line/polygon vertex or index buffers far beyond WebGPU limits; skip it
       rather than let the GPU upload fail and render nothing. */
    {
        size_t worst = p->prims.lines.vert_count * sizeof(arpt_line_vertex);
        size_t cand[] = {
            p->prims.lines.index_count * sizeof(uint32_t),
            p->prims.polygons.vert_count * sizeof(arpt_poly_vertex),
            p->prims.polygons.index_count * sizeof(uint32_t),
        };
        for (size_t i = 0; i < sizeof(cand) / sizeof(cand[0]); i++)
            if (cand[i] > worst) worst = cand[i];
        if (worst > ARPT_MAX_BUFFER_BYTES) {
            fprintf(stderr, "[TILE] %d/%d/%d SKIPPED: oversized vector buffers "
                    "(lines %zu verts/%zu idx, polys %zu verts/%zu idx)\n",
                    key.level, key.x, key.y,
                    p->prims.lines.vert_count, p->prims.lines.index_count,
                    p->prims.polygons.vert_count, p->prims.polygons.index_count);
            goto oom;
        }
    }

    arpt_surface_data_free(&surface);
    arpt_line_data_free(&lines);
    arpt_tree_data_free(&trees);
    arpt_poi_data_free(&pois);
    arpt_line_label_data_free(&line_labels);
    free(flatbuf);

    return p;

oom:
    arpt_surface_data_free(&surface);
    arpt_line_data_free(&lines);
    arpt_tree_data_free(&trees);
    arpt_poi_data_free(&pois);
    arpt_line_label_data_free(&line_labels);
    free(flatbuf);
    prepared_tile_free(p);
    return NULL;
}

/* Runs on the main thread via arpt_fetch_drain: only the GPU upload and
   cache-state update remain here, so even a burst of completed fetches
   keeps the frame cheap.

   `success` is set by the fetch layer: true ⇒ HTTP + verify + prepare all
   succeeded and payload is a prepared_tile*; false ⇒ either HTTP failed
   (retry) or prepare returned NULL (permanent decode failure). */
static void tile_finish_main(bool success, void *payload, void *userdata) {
    fetch_ctx *ctx = userdata;
    arpt_tile_manager *tm = ctx->tm;
    arpt_tile_key key = ctx->key;
    int retries = ctx->retries;
    free(ctx);

    prepared_tile *p = payload;
    tm->active_fetches--;

    tile_entry lookup = {.key = key};
    const tile_entry *existing = hashmap_get(tm->cache, &lookup);
    if (!existing || existing->state != TILE_LOADING) {
        prepared_tile_free(p);
        return;
    }

    tile_entry updated = *existing;

    if (!success) {
        updated.state = TILE_FAILED;
        if (p) {
            /* HTTP succeeded but decode/prepare failed — permanent. */
            updated.retries = MAX_RETRIES;
        } else {
            /* HTTP failure — retry with backoff. */
            updated.retries = retries + 1;
            updated.retry_after = tm->frame + (1u << updated.retries);
        }
        tm_hashmap_set(tm, &updated);
        prepared_tile_free(p);
        return;
    }

    updated.avg_elevation = p->avg_elevation;
    updated.gpu = arpt_renderer_upload_tile(tm->renderer, &p->prims);
    if (updated.gpu) {
        updated.state = TILE_READY;
        tm->needs_redraw = true;
        /* Move the terrain heightfield into the cache entry (zero-copy); the
           GPU upload above already consumed it.  NULL the source so
           prepared_tile_free leaves these for the entry to own. */
        updated.terr_x = p->terrain_x;
        updated.terr_y = p->terrain_y;
        updated.terr_z = p->terrain_z;
        updated.terr_indices = p->terrain_indices;
        updated.terr_vert_count = p->terrain_vertex_count;
        updated.terr_index_count = p->terrain_index_count;
        p->terrain_x = NULL;
        p->terrain_y = NULL;
        p->terrain_z = NULL;
        p->terrain_indices = NULL;
    } else {
        /* GPU upload failed (likely memory pressure) — back off before
           retrying rather than refetching every frame. */
        updated.state = TILE_FAILED;
        updated.retries = retries + 1;
        updated.retry_after = tm->frame + (1u << updated.retries);
    }
    tm_hashmap_set(tm, &updated);

    prepared_tile_free(p);
}

/* LRU eviction */

/* True when key is in the current frame's visible tile list. */
static bool tile_visible_now(const arpt_tile_manager *tm, arpt_tile_key key) {
    for (int i = 0; i < tm->visible_count; i++) {
        if (tm->visible[i].level == key.level && tm->visible[i].x == key.x &&
            tm->visible[i].y == key.y)
            return true;
    }
    return false;
}

static void evict_oldest(arpt_tile_manager *tm) {
    size_t count = hashmap_count(tm->cache);
    if ((int)count <= tm->config.max_tiles) return;

    size_t to_evict = count - (size_t)tm->config.max_tiles;
    for (size_t e = 0; e < to_evict; e++) {
        /* Find the least recently used READY or FAILED entry.  Visible tiles
           are never evicted: they would be refetched on the very next frame,
           and once the visible set exceeds max_tiles that turns into an
           endless reload loop of whichever tiles iterate first.  The cache
           may exceed max_tiles while the visible set itself is larger. */
        uint64_t oldest_frame = UINT64_MAX;
        arpt_tile_key oldest_key = {0};
        bool found = false;

        size_t iter = 0;
        void *item;
        while (hashmap_iter(tm->cache, &iter, &item)) {
            tile_entry *entry = item;
            if (entry->state == TILE_LOADING)
                continue; /* don't evict in-flight */
            if (tile_visible_now(tm, entry->key))
                continue;
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
    if (style) tm->style = *style;

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
                         tile_prepare_worker, tile_finish_main, ctx)) {
        tm->active_fetches--;
        tile_entry failed = new_entry;
        failed.state = TILE_FAILED;
        failed.retries = prev_retries + 1;
        failed.retry_after = tm->frame + (1u << failed.retries);
        tm_hashmap_set(tm, &failed);
        free(ctx);
    }
}

/* Per-frame cap on tile uploads.  Each completed fetch triggers flatbuffer
   decode, polygon triangulation, buffer uploads, and surface-texture
   rasterization + mipmap generation — several ms of CPU + GPU work.  Doing
   that for every tile in one frame causes visible hitching while panning,
   so we process a few per frame and let the rest catch up over subsequent
   frames. */
#define ARPT_TILE_UPLOAD_BUDGET_PER_FRAME 2

/* Interpolate the terrain height (metres) at geographic (lon_rad, lat_rad)
   from a tile's retained heightfield.  Returns false if the point isn't inside
   any triangle (e.g. the camera is over the tile's buffer zone, not its
   proper area).  O(triangles), called once per frame for the tile under the
   camera. */
static bool sample_terrain_height(const tile_entry *e, double lon_rad,
                                  double lat_rad, double *out_h) {
    if (!e->terr_x || !e->terr_y || !e->terr_z || e->terr_index_count < 3)
        return false;

    double lon = lon_rad * 180.0 / M_PI;
    double lat = lat_rad * 180.0 / M_PI;
    double lon_span = e->bounds.east - e->bounds.west;
    double lat_span = e->bounds.north - e->bounds.south;
    if (lon_span <= 0.0 || lat_span <= 0.0) return false;

    /* Geographic → tile quantised coords (inverse of the shader dequant:
       u = (qx - 16384) / 32768, lon = west + u * span). */
    double u = (lon - e->bounds.west) / lon_span;
    double v = (lat - e->bounds.south) / lat_span;
    double px = 16384.0 + u * 32768.0;
    double py = 16384.0 + v * 32768.0;

    const uint16_t *X = e->terr_x, *Y = e->terr_y;
    const int32_t *Z = e->terr_z;
    const uint32_t *idx = e->terr_indices;
    for (size_t t = 0; t + 2 < e->terr_index_count; t += 3) {
        uint32_t ia = idx[t], ib = idx[t + 1], ic = idx[t + 2];
        double ax = X[ia], ay = Y[ia];
        double bx = X[ib], by = Y[ib];
        double cx = X[ic], cy = Y[ic];
        /* Barycentric coordinates of (px, py) in triangle a,b,c. */
        double d = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy);
        if (fabs(d) < 1e-9) continue; /* degenerate */
        double w0 = ((by - cy) * (px - cx) + (cx - bx) * (py - cy)) / d;
        double w1 = ((cy - ay) * (px - cx) + (ax - cx) * (py - cy)) / d;
        double w2 = 1.0 - w0 - w1;
        const double eps = -1e-6;
        if (w0 < eps || w1 < eps || w2 < eps) continue; /* outside */
        double z_mm = w0 * Z[ia] + w1 * Z[ib] + w2 * Z[ic];
        *out_h = z_mm * 0.001;
        return true;
    }
    return false;
}

/* Terrain height at a geodetic point, taken from the highest-level READY tile
   that contains it.  Falls back to that tile's average elevation when the
   point lies outside the mesh triangles.  Returns false when no READY tile
   covers the point, so callers keep their prior value rather than snapping to
   zero.  Scans the full cache (not just the visible set) because the queried
   point — e.g. the ground under a tilted eye — may sit just outside the
   frustum. */
static bool query_ground_at(const arpt_tile_manager *tm, double lon_rad,
                            double lat_rad, double *out_h) {
    double lon_deg = lon_rad * 180.0 / M_PI;
    double lat_deg = lat_rad * 180.0 / M_PI;
    int best_level = -1;
    const tile_entry *best_e = NULL;

    size_t iter = 0;
    void *item;
    while (hashmap_iter(tm->cache, &iter, &item)) {
        const tile_entry *e = item;
        if (e->state != TILE_READY) continue;
        if (e->key.level <= best_level) continue;
        if (lon_deg < e->bounds.west || lon_deg > e->bounds.east) continue;
        if (lat_deg < e->bounds.south || lat_deg > e->bounds.north) continue;
        best_level = e->key.level;
        best_e = e;
    }
    if (!best_e) return false;
    if (!sample_terrain_height(best_e, lon_rad, lat_rad, out_h))
        *out_h = best_e->avg_elevation;
    return true;
}

void arpt_tile_manager_update(arpt_tile_manager *tm, const arpt_camera *cam) {
    arpt_fetch_drain(ARPT_TILE_UPLOAD_BUDGET_PER_FRAME);
    tm->frame++;

    /* Pre-fetch level-0 root tiles on the first frame so that
       draw always has a fallback level to render. */
    if (tm->frame == 1) {
        arpt_tile_key roots[1] = {{0, 0, 0}};
        for (int r = 0; r < 1; r++) {
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

    /* Sample the real terrain height under the camera's interest point so
       interaction (pan, zoom anchor) and the camera's own height track the
       surface at street-level overzoom, where the tile average is too coarse.
       Keep the previous value when no READY tile covers the point. */
    double h;
    if (query_ground_at(tm, arpt_camera_lon(cam), arpt_camera_lat(cam), &h))
        tm->ground_elevation = h;

    /* Overzoom: when the view resolves finer than the tileset's deepest level,
       visible tiles are clamped to max_level and their baked surface fill
       texture is magnified, going pixelated.  Re-rasterize that texture at a
       higher resolution proportional to the overzoom amount so fills stay
       crisp.  Re-rasterizing is GPU work, so cap it per frame (as with tile
       uploads); the rest catch up over subsequent frames. */
    int desired = arpt_camera_zoom_level_desired(cam, tm->config.root_error,
                                                  tm->config.min_level);
    int reraster_budget = ARPT_TILE_UPLOAD_BUDGET_PER_FRAME;
    for (int i = 0; i < tm->visible_count && reraster_budget > 0; i++) {
        tile_entry lookup = {.key = tm->visible[i]};
        const tile_entry *e = hashmap_get(tm->cache, &lookup);
        if (!e || e->state != TILE_READY || !e->gpu) continue;
        if (arpt_renderer_tile_set_overzoom(tm->renderer, e->gpu,
                                            desired - e->key.level)) {
            reraster_budget--;
            tm->needs_redraw = true;
        }
    }
}

int arpt_tile_manager_active_fetches(const arpt_tile_manager *tm) {
    return tm ? tm->active_fetches : 0;
}

bool arpt_tile_manager_needs_redraw(arpt_tile_manager *tm) {
    if (!tm) return false;
    bool v = tm->needs_redraw;
    tm->needs_redraw = false;
    return v;
}

/* Ground elevation query */

double arpt_tile_manager_camera_ground_elevation(const arpt_tile_manager *tm) {
    return tm ? tm->ground_elevation : 0.0;
}

bool arpt_tile_manager_sample_ground(const arpt_tile_manager *tm,
                                     double lon_rad, double lat_rad,
                                     double *out_h) {
    if (!tm) return false;
    return query_ground_at(tm, lon_rad, lat_rad, out_h);
}

/* Draw helpers */

static void draw_entry(arpt_renderer *r, const arpt_camera *cam,
                       const tile_entry *e) {
    /* Diagnostic (env ARPT_ONLY_TILE="level/x/y"): draw only that tile. */
    static int only_tile = -2; /* -2 unparsed, -1 off */
    static int oz, ox, oy;
    if (only_tile == -2) {
        const char *s = getenv("ARPT_ONLY_TILE");
        only_tile = (s && sscanf(s, "%d/%d/%d", &oz, &ox, &oy) == 3) ? 1 : -1;
    }
    if (only_tile == 1 &&
        (e->key.level != oz || e->key.x != ox || e->key.y != oy))
        return;

    arpt_mat4 model =
        arpt_camera_tile_model(cam, e->center_lon_rad, e->center_lat_rad, 0.0);
    double bounds_rad[4] = {
        e->bounds.west * M_PI / 180.0,
        e->bounds.south * M_PI / 180.0,
        e->bounds.east * M_PI / 180.0,
        e->bounds.north * M_PI / 180.0,
    };
    arpt_tile_gpu_set_uniforms((arpt_tile_gpu *)e->gpu, model, bounds_rad,
                               e->center_lon_rad, e->center_lat_rad);
    arpt_renderer_draw_tile(r, (arpt_tile_gpu *)e->gpu);
}

void arpt_tile_manager_debug_info(const arpt_tile_manager *tm) {
    if (!tm) return;

    static const char *state_names[] = {"EMPTY", "LOADING", "READY", "FAILED"};

    printf("[DEBUG] zoom_level=%d  visible_tiles=%d  active_fetches=%d  "
           "cached=%zu\n",
           tm->visible_level, tm->visible_count, tm->active_fetches,
           hashmap_count(tm->cache));

    for (int i = 0; i < tm->visible_count; i++) {
        arpt_tile_key k = tm->visible[i];
        tile_entry lookup = {.key = k};
        const tile_entry *e = hashmap_get(tm->cache, &lookup);

        const char *state = "MISSING";
        double elev = 0.0;
        if (e) {
            state = (e->state <= TILE_FAILED) ? state_names[e->state] : "?";
            elev = e->avg_elevation;
        }

        arpt_bounds b = arpt_tile_bounds(k.level, k.x, k.y);
        printf("  tile %d/%d/%d  state=%-8s  bounds=[%.4f,%.4f,%.4f,%.4f]  "
               "elev=%.1fm\n",
               k.level, k.x, k.y, state, b.west, b.south, b.east, b.north,
               elev);
    }
}

void arpt_tile_manager_draw(arpt_tile_manager *tm, arpt_renderer *r,
                            const arpt_camera *cam) {
    /* Phase 1: draw ancestor fallbacks for tiles that are not yet ready.
       Track which ancestors we've already drawn to avoid duplicates. */
    arpt_tile_key drawn_ancestors[MAX_VISIBLE_TILES];
    int drawn_count = 0;

    for (int i = 0; i < tm->visible_count; i++) {
        tile_entry lookup = {.key = tm->visible[i]};
        const tile_entry *e = hashmap_get(tm->cache, &lookup);
        if (e && e->state == TILE_READY && e->gpu) continue;

        /* Walk up the hierarchy to find the nearest READY ancestor */
        int al = tm->visible[i].level;
        int ax = tm->visible[i].x;
        int ay = tm->visible[i].y;
        while (arpt_tile_ancestor(al, ax, ay, &al, &ax, &ay)) {
            tile_entry alookup = {.key = {al, ax, ay}};
            const tile_entry *ancestor = hashmap_get(tm->cache, &alookup);
            if (!ancestor || ancestor->state != TILE_READY || !ancestor->gpu)
                continue;

            /* Check if we already drew this ancestor */
            bool already = false;
            for (int d = 0; d < drawn_count; d++) {
                if (drawn_ancestors[d].level == al &&
                    drawn_ancestors[d].x == ax &&
                    drawn_ancestors[d].y == ay) {
                    already = true;
                    break;
                }
            }
            if (!already) {
                draw_entry(r, cam, ancestor);
                if (drawn_count < MAX_VISIBLE_TILES)
                    drawn_ancestors[drawn_count++] =
                        (arpt_tile_key){al, ax, ay};
            }
            break;
        }
    }

    /* Phase 2: draw READY tiles on top of ancestors */
    for (int i = 0; i < tm->visible_count; i++) {
        tile_entry lookup = {.key = tm->visible[i]};
        const tile_entry *e = hashmap_get(tm->cache, &lookup);
        if (e && e->state == TILE_READY && e->gpu)
            draw_entry(r, cam, e);
    }
}
