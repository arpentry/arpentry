#ifndef ARPENTRY_TILE_MANAGER_H
#define ARPENTRY_TILE_MANAGER_H

#include "camera.h"
#include "coords.h"
#include <stdbool.h>
#include <stdint.h>

/* Tile key */

typedef struct {
    int level;
    int x;
    int y;
} arpt_tile_key;

/* Pure geometry functions (testable without GPU) */

/**
 * Enumerate tiles visible from the current camera at the given zoom level.
 * Casts rays from screen corners/edges, intersects with the WGS84 ellipsoid,
 * and returns the tile grid cells covering the visible region.
 *
 * Returns the number of tiles written to out (at most max_count).
 */
int arpt_enumerate_visible_tiles(const arpt_camera *cam, int level,
                                 arpt_tile_key *out, int max_count);

/**
 * Get the parent tile key. Returns false at level 0 (no parent).
 */
static inline bool arpt_tile_ancestor(int level, int x, int y, int *plevel,
                                      int *px, int *py) {
    if (level <= 0) return false;
    *plevel = level - 1;
    *px = x / 2;
    *py = y / 2;
    return true;
}

/* Tile manager (requires renderer) */

typedef struct arpt_tile_manager arpt_tile_manager;
typedef struct arpt_renderer arpt_renderer;

typedef struct {
    const char *base_url;
    double root_error;
    int min_level;
    int max_level;
    int max_tiles;      /* LRU cache capacity */
    int max_concurrent; /* max in-flight fetches */
} arpt_tile_manager_config;

/** Return a config with sensible defaults. Only base_url must be set. */
static inline arpt_tile_manager_config arpt_tile_manager_config_defaults(
    const char *base_url) {
    return (arpt_tile_manager_config){
        .base_url = base_url,
        .root_error = 400000.0,
        .min_level = 0,
        .max_level = 19,
        .max_tiles = 200,
        .max_concurrent = 6,
    };
}

typedef struct arpt_style arpt_style;

arpt_tile_manager *arpt_tile_manager_create(arpt_tile_manager_config config,
                                            arpt_renderer *r,
                                            const arpt_style *style);
void arpt_tile_manager_free(arpt_tile_manager *tm);

/**
 * Compute visible tiles, initiate fetches for missing tiles, evict old tiles.
 */
void arpt_tile_manager_update(arpt_tile_manager *tm, const arpt_camera *cam);

/** Returns the number of in-flight tile fetches. */
int arpt_tile_manager_active_fetches(const arpt_tile_manager *tm);

/** Returns true (once) after a tile upload completes. Clear-on-read. */
bool arpt_tile_manager_needs_redraw(arpt_tile_manager *tm);

/**
 * Ground elevation (meters) at the camera position from the best loaded tile.
 * Returns 0.0 if no tile is loaded yet.
 */
double arpt_tile_manager_camera_ground_elevation(const arpt_tile_manager *tm);

/**
 * Terrain height (meters) at an arbitrary geodetic point, sampled from the
 * highest-level READY tile that covers it.  Returns false (leaving *out_h
 * untouched) when no loaded tile covers the point.  Used to keep the camera
 * eye above terrain it flies over when tilted.
 */
bool arpt_tile_manager_sample_ground(const arpt_tile_manager *tm,
                                     double lon_rad, double lat_rad,
                                     double *out_h);

/**
 * Draw visible tiles at the target zoom level.  READY tiles are drawn
 * normally; tiles still loading fall back to the nearest READY ancestor,
 * providing smooth visual continuity while tiles load.
 */
void arpt_tile_manager_draw(arpt_tile_manager *tm, arpt_renderer *r,
                            const arpt_camera *cam);

/**
 * Print debug info to stdout: zoom level, visible tiles with states.
 */
void arpt_tile_manager_debug_info(const arpt_tile_manager *tm);

#endif /* ARPENTRY_TILE_MANAGER_H */
