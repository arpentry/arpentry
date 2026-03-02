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

arpt_tile_manager *arpt_tile_manager_create(arpt_tile_manager_config config,
                                            arpt_renderer *r);
void arpt_tile_manager_free(arpt_tile_manager *tm);

/**
 * Compute visible tiles, initiate fetches for missing tiles, evict old tiles.
 */
void arpt_tile_manager_update(arpt_tile_manager *tm, const arpt_camera *cam);

/**
 * Query ground elevation (meters) at a geodetic position.
 * Finds the highest-level READY tile containing the position and returns
 * its average terrain elevation. Returns 0.0 if no tile covers the point.
 */
double arpt_tile_manager_ground_elevation(const arpt_tile_manager *tm,
                                          double lon_rad, double lat_rad);

/**
 * Draw visible tiles at the target zoom level.  READY tiles are drawn
 * normally; tiles still loading are shown as flat placeholder quads.
 * All tiles belong to the same zoom level — no ancestor mixing.
 */
void arpt_tile_manager_draw(arpt_tile_manager *tm, arpt_renderer *r,
                            const arpt_camera *cam);

#endif /* ARPENTRY_TILE_MANAGER_H */
