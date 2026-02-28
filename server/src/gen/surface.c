#include "surface.h"
#include "terrain.h"

/* Biome classification thresholds */
#define BIOME_ELEV_ICE  3000.0  /* above -> ice */
#define BIOME_ELEV_MID  1500.0  /* above -> shrub / forest */
#define BIOME_ELEV_LOW   400.0  /* above -> grassland / forest */
#define BIOME_MOIST_WET    0.55 /* above -> forest */
#define BIOME_MOIST_DRY    0.25 /* below -> desert */

/* Phase 4: Classify surface type at each vertex of the marching squares grid.
 *
 * Two independent noise fields drive biome placement:
 *   - terrain elevation  (from terrain_elevation)
 *   - moisture          (from terrain_moisture)
 *
 * 2D biome lookup (elevation bands x moisture bands) following the
 * Red Blob Games terrain-from-noise approach.  No latitude overrides --
 * biomes are distributed evenly across the globe.
 *
 *   Elev / Moisture      Dry            Medium          Wet
 *   High (>ICE)         ice            ice             ice
 *   Mid-high (MID-ICE)  shrub          shrub           forest
 *   Mid (LOW-MID)       grassland      grassland       forest
 *   Low (0-LOW)         desert         cropland        forest
 */
void classify_surface(arpt_bounds bounds, double lon_span, double lat_span,
                      int *vert_class) {
    for (int vr = 0; vr < SURFACE_VERTS; vr++) {
        double v = (double)(vr - SURFACE_BUFFER) / SURFACE_GRID;
        double lat = bounds.south + v * lat_span;

        for (int vc = 0; vc < SURFACE_VERTS; vc++) {
            double u = (double)(vc - SURFACE_BUFFER) / SURFACE_GRID;
            double lon = bounds.west + u * lon_span;
            double e = terrain_elevation(lon, lat);
            double m = terrain_moisture(lon, lat);

            int cls;
            if (e < 0.0) {
                cls = SURFACE_VAL_WATER;
            } else if (e > BIOME_ELEV_ICE) {
                cls = SURFACE_VAL_ICE;
            } else if (e > BIOME_ELEV_MID) {
                cls = (m > BIOME_MOIST_WET) ? SURFACE_VAL_FOREST
                                            : SURFACE_VAL_SHRUB;
            } else if (e > BIOME_ELEV_LOW) {
                cls = (m > BIOME_MOIST_WET) ? SURFACE_VAL_FOREST
                                            : SURFACE_VAL_GRASSLAND;
            } else {
                if (m > BIOME_MOIST_WET)
                    cls = SURFACE_VAL_FOREST;
                else if (m > BIOME_MOIST_DRY)
                    cls = SURFACE_VAL_CROPLAND;
                else
                    cls = SURFACE_VAL_DESERT;
            }

            vert_class[vr * SURFACE_VERTS + vc] = cls;
        }
    }
}

/* Phase 5: Generate surface polygon patches via marching squares. */
int generate_surface_patches(const int *vert_class, ms_patch *patches) {
    int patch_count = 0;

    for (int r = 0; r < SURFACE_TOTAL; r++) {
        for (int c = 0; c < SURFACE_TOTAL; c++) {
            int cl_tl = vert_class[r * SURFACE_VERTS + c];
            int cl_tr = vert_class[r * SURFACE_VERTS + c + 1];
            int cl_bl = vert_class[(r + 1) * SURFACE_VERTS + c];
            int cl_br = vert_class[(r + 1) * SURFACE_VERTS + c + 1];

            /* Find unique classes in this cell */
            int unique[4], n_unique = 0;
            int corners[4] = {cl_tl, cl_tr, cl_bl, cl_br};
            for (int i = 0; i < 4; i++) {
                int found = 0;
                for (int j = 0; j < n_unique; j++)
                    if (unique[j] == corners[i]) {
                        found = 1;
                        break;
                    }
                if (!found) unique[n_unique++] = corners[i];
            }

            /* Quantized cell corner and edge-midpoint coordinates */
            uint16_t xl =
                arpt_quantize((double)(c - SURFACE_BUFFER) / SURFACE_GRID);
            uint16_t xm =
                arpt_quantize((c - SURFACE_BUFFER + 0.5) / SURFACE_GRID);
            uint16_t xr =
                arpt_quantize((double)(c - SURFACE_BUFFER + 1) / SURFACE_GRID);
            uint16_t yt =
                arpt_quantize((double)(r - SURFACE_BUFFER) / SURFACE_GRID);
            uint16_t ym =
                arpt_quantize((r - SURFACE_BUFFER + 0.5) / SURFACE_GRID);
            uint16_t yb =
                arpt_quantize((double)(r - SURFACE_BUFFER + 1) / SURFACE_GRID);

            for (int ui = 0; ui < n_unique; ui++) {
                int cls = unique[ui];

                /* Perimeter walk (clockwise from TL) */
                ms_patch *p = &patches[patch_count];
                p->cls = cls;
                int n = 0;

#define MS_V(px, py)    \
    do {                \
        p->x[n] = (px); \
        p->y[n] = (py); \
        n++;            \
    } while (0)
                if (cl_tl == cls) MS_V(xl, yt);
                if (cl_tl != cl_tr && (cl_tl == cls || cl_tr == cls))
                    MS_V(xm, yt);
                if (cl_tr == cls) MS_V(xr, yt);
                if (cl_tr != cl_br && (cl_tr == cls || cl_br == cls))
                    MS_V(xr, ym);
                if (cl_br == cls) MS_V(xr, yb);
                if (cl_bl != cl_br && (cl_bl == cls || cl_br == cls))
                    MS_V(xm, yb);
                if (cl_bl == cls) MS_V(xl, yb);
                if (cl_tl != cl_bl && (cl_tl == cls || cl_bl == cls))
                    MS_V(xl, ym);
#undef MS_V

                if (n < 3) continue;
                p->x[n] = p->x[0];
                p->y[n] = p->y[0];
                p->count = n + 1;
                patch_count++;
            }
        }
    }

    return patch_count;
}
