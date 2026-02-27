#include "terrain_gen.h"
#include "noise.h"
#include "coords.h"
#include "tile.h"
#include "tile_builder.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Terrain parameters */

#define TERRAIN_GRID        64
#define TERRAIN_VERTS       (TERRAIN_GRID + 1)  /* 65x65 = 4225 vertices */
#define TERRAIN_OCTAVES     16
#define TERRAIN_BASE_FREQ   4.0
#define TERRAIN_AMPLITUDE   8000.0   /* peak ~8000m */
#define TERRAIN_LACUNARITY  2.0
#define TERRAIN_PERSISTENCE 0.65
#define TERRAIN_REDISTRIBUTE 0.45  /* power curve exponent: < 1 pushes toward extremes */
#define BROTLI_QUALITY      4

#define LANDUSE_GRID        64      /* 64x64 marching squares grid (matches terrain) */
#define LANDUSE_BUFFER      8       /* extra cells of buffer on each side */
#define LANDUSE_TOTAL       (LANDUSE_GRID + 2 * LANDUSE_BUFFER)  /* 80 */
#define LANDUSE_VERTS       (LANDUSE_TOTAL + 1)  /* 81x81 classification vertices */

/* Landuse class indices into the tile-scope value dictionary */
#define LANDUSE_VAL_GRASS   0
#define LANDUSE_VAL_FOREST  1
#define LANDUSE_VAL_SAND    2

#define PI 3.14159265358979323846

/* Helpers */

static double terrain_elevation(double lon_deg, double lat_deg) {
    double lon_r = lon_deg * (PI / 180.0);
    double lat_r = lat_deg * (PI / 180.0);
    double cos_lat = cos(lat_r);
    double sx = cos_lat * cos(lon_r);
    double sy = cos_lat * sin(lon_r);
    double sz = sin(lat_r);
    double n = arpt_fbm3(sx * TERRAIN_BASE_FREQ, sy * TERRAIN_BASE_FREQ,
                         sz * TERRAIN_BASE_FREQ, TERRAIN_OCTAVES,
                         TERRAIN_LACUNARITY, TERRAIN_PERSISTENCE);
    /* Redistribute: sign(n) * |n|^exp pushes values away from zero,
     * creating more pronounced mountains and valleys. */
    double sign = n >= 0.0 ? 1.0 : -1.0;
    n = sign * pow(fabs(n), TERRAIN_REDISTRIBUTE);
    return n * TERRAIN_AMPLITUDE;
}

/* Octahedral encoding: unit normal -> int8x2 */
static void encode_octahedral(double nx, double ny, double nz,
                               int8_t *ox, int8_t *oy) {
    double ax = fabs(nx), ay = fabs(ny), az = fabs(nz);
    double sum = ax + ay + az;
    if (sum < 1e-15) { *ox = 0; *oy = 127; return; }

    double u = nx / sum;
    double v = ny / sum;

    /* Reflect lower hemisphere */
    if (nz < 0.0) {
        double old_u = u;
        u = (1.0 - fabs(v)) * (old_u >= 0.0 ? 1.0 : -1.0);
        v = (1.0 - fabs(old_u)) * (v >= 0.0 ? 1.0 : -1.0);
    }

    /* Quantize to int8 [-127, 127] */
    double cu = u * 127.0;
    double cv = v * 127.0;
    *ox = (int8_t)(cu >= 0.0 ? cu + 0.5 : cu - 0.5);
    *oy = (int8_t)(cv >= 0.0 ? cv + 0.5 : cv - 0.5);
}

/* Public API */

bool arpt_generate_terrain(int level, int x, int y,
                           uint8_t **out, size_t *out_size)
{
    if (!out || !out_size) return false;

    arpt_bounds bounds = arpt_tile_bounds(level, x, y);
    double lon_span = bounds.east - bounds.west;
    double lat_span = bounds.north - bounds.south;

    /* Approximate cell size in meters (for normal computation) */
    double mid_lat = (bounds.south + bounds.north) * 0.5;
    double cos_lat = cos(mid_lat * PI / 180.0);
    double meters_per_deg_lon = 111319.5 * cos_lat;
    double meters_per_deg_lat = 111319.5;
    double cell_w_m = (lon_span / TERRAIN_GRID) * meters_per_deg_lon;
    double cell_h_m = (lat_span / TERRAIN_GRID) * meters_per_deg_lat;

    /* Compute elevations on a padded grid: (VERTS+2) x (VERTS+2).
     * The 1-cell border beyond the tile boundary allows centered finite
     * differences at every output vertex, eliminating normal seams between
     * adjacent tiles. */
    int nv = TERRAIN_VERTS * TERRAIN_VERTS;
    int pad_w = TERRAIN_VERTS + 2;    /* 35 */
    int pad_n = pad_w * pad_w;        /* 35x35 = 1225 */
    double *elev = malloc((size_t)pad_n * sizeof(double));
    if (!elev) return false;

    double cell_lon = lon_span / TERRAIN_GRID;
    double cell_lat = lat_span / TERRAIN_GRID;

    for (int row = -1; row <= TERRAIN_VERTS; row++) {
        double lat = bounds.south + (double)row * cell_lat;
        for (int col = -1; col <= TERRAIN_VERTS; col++) {
            double lon = bounds.west + (double)col * cell_lon;
            elev[(row + 1) * pad_w + (col + 1)] = terrain_elevation(lon, lat);
        }
    }

    /* Build vertex arrays (only the inner VERTS x VERTS) */
    uint16_t *vx = malloc((size_t)nv * sizeof(uint16_t));
    uint16_t *vy = malloc((size_t)nv * sizeof(uint16_t));
    int32_t  *vz = malloc((size_t)nv * sizeof(int32_t));
    int8_t   *normals = malloc((size_t)nv * 2 * sizeof(int8_t));
    struct ms_patch { uint16_t x[9], y[9]; int count, cls; };
    struct ms_patch *patches = NULL;
    int patch_count = 0;
    if (!vx || !vy || !vz || !normals) goto fail;

    for (int row = 0; row < TERRAIN_VERTS; row++) {
        double lat = bounds.south + (double)row * cell_lat;
        for (int col = 0; col < TERRAIN_VERTS; col++) {
            int idx = row * TERRAIN_VERTS + col;
            double lon = bounds.west + (double)col * cell_lon;

            /* Padded grid index: offset by +1 for the border */
            int pi = (row + 1) * pad_w + (col + 1);

            vx[idx] = arpt_quantize_lon(lon, bounds);
            vy[idx] = arpt_quantize_lat(lat, bounds);
            vz[idx] = arpt_meters_to_mm(elev[pi]);

            /* Centered finite differences using padded neighbors */
            double dz_dx = (elev[pi + 1] - elev[pi - 1]) / (2.0 * cell_w_m);
            double dz_dy = (elev[pi + pad_w] - elev[pi - pad_w]) / (2.0 * cell_h_m);

            /* Compute normal in ECEF coordinates.
             * The shader transforms normals by the tile model matrix which
             * operates in ECEF space, so we need ECEF normals (not tangent-plane).
             * N = normalize(up - dz_dx * east - dz_dy * north) */
            double lon_r = lon * (PI / 180.0);
            double lat_r = lat * (PI / 180.0);
            double sin_lon = sin(lon_r), cos_lon = cos(lon_r);
            double sin_lat = sin(lat_r), cos_lat = cos(lat_r);

            /* ENU basis vectors in ECEF */
            double ex = -sin_lon,           ey = cos_lon,            ez = 0.0;
            double nx_e = -sin_lat*cos_lon, ny_e = -sin_lat*sin_lon, nz_e = cos_lat;
            double ux = cos_lat*cos_lon,    uy = cos_lat*sin_lon,    uz = sin_lat;

            /* N_ecef = up - dz_dx * east - dz_dy * north */
            double nx = ux - dz_dx * ex - dz_dy * nx_e;
            double ny = uy - dz_dx * ey - dz_dy * ny_e;
            double nz = uz - dz_dx * ez - dz_dy * nz_e;
            double len = sqrt(nx*nx + ny*ny + nz*nz);
            nx /= len; ny /= len; nz /= len;

            encode_octahedral(nx, ny, nz, &normals[idx * 2], &normals[idx * 2 + 1]);
        }
    }

    /* Build triangle indices: 2 triangles per quad, TERRAIN_GRID^2 quads */
    int num_triangles = TERRAIN_GRID * TERRAIN_GRID * 2;
    int num_indices = num_triangles * 3;
    uint32_t *indices = malloc((size_t)num_indices * sizeof(uint32_t));
    if (!indices) goto fail;

    int ii = 0;
    for (int row = 0; row < TERRAIN_GRID; row++) {
        for (int col = 0; col < TERRAIN_GRID; col++) {
            uint32_t tl = (uint32_t)(row * TERRAIN_VERTS + col);
            uint32_t tr = tl + 1;
            uint32_t bl = tl + TERRAIN_VERTS;
            uint32_t br = bl + 1;

            indices[ii++] = tl;
            indices[ii++] = bl;
            indices[ii++] = tr;

            indices[ii++] = tr;
            indices[ii++] = bl;
            indices[ii++] = br;
        }
    }

    /* Classify landuse at each vertex of the extended marching squares grid.
     * Uses terrain_elevation() directly so buffer-zone vertices beyond
     * the tile proper are classified identically by adjacent tiles. */
    int vert_class[LANDUSE_VERTS * LANDUSE_VERTS];
    {
        for (int vr = 0; vr < LANDUSE_VERTS; vr++) {
            double v = (double)(vr - LANDUSE_BUFFER) / LANDUSE_GRID;
            double lat = bounds.south + v * lat_span;
            for (int vc = 0; vc < LANDUSE_VERTS; vc++) {
                double u = (double)(vc - LANDUSE_BUFFER) / LANDUSE_GRID;
                double lon = bounds.west + u * lon_span;
                double e = terrain_elevation(lon, lat);

                int cls;
                if (e < 200.0)       cls = LANDUSE_VAL_GRASS;
                else if (e < 800.0)  cls = LANDUSE_VAL_FOREST;
                else if (e < 1500.0) cls = LANDUSE_VAL_GRASS;
                else                 cls = LANDUSE_VAL_SAND;

                vert_class[vr * LANDUSE_VERTS + vc] = cls;
            }
        }
    }

    /* Generate landuse polygon patches using marching squares.
     * For each cell, walk the perimeter clockwise collecting vertices where
     * the target class appears: cell corners that match + edge midpoints
     * where the boundary crosses. Saddle cases (diagonal same-class) are
     * split into two separate triangles per class. */
    patches = malloc((size_t)(LANDUSE_TOTAL * LANDUSE_TOTAL * 4)
                     * sizeof(struct ms_patch));
    if (!patches) goto fail;

    for (int r = 0; r < LANDUSE_TOTAL; r++) {
        for (int c = 0; c < LANDUSE_TOTAL; c++) {
            int cl_tl = vert_class[r * LANDUSE_VERTS + c];
            int cl_tr = vert_class[r * LANDUSE_VERTS + c + 1];
            int cl_bl = vert_class[(r+1) * LANDUSE_VERTS + c];
            int cl_br = vert_class[(r+1) * LANDUSE_VERTS + c + 1];

            /* Find unique classes in this cell */
            int unique[4], n_unique = 0;
            int corners[4] = {cl_tl, cl_tr, cl_bl, cl_br};
            for (int i = 0; i < 4; i++) {
                int found = 0;
                for (int j = 0; j < n_unique; j++)
                    if (unique[j] == corners[i]) { found = 1; break; }
                if (!found) unique[n_unique++] = corners[i];
            }

            /* Quantized cell corner and edge-midpoint coordinates.
             * Offset by -LANDUSE_BUFFER so tile proper is at [0, 1]. */
            uint16_t xl = arpt_quantize((double)(c - LANDUSE_BUFFER) / LANDUSE_GRID);
            uint16_t xm = arpt_quantize((c - LANDUSE_BUFFER + 0.5) / LANDUSE_GRID);
            uint16_t xr = arpt_quantize((double)(c - LANDUSE_BUFFER + 1) / LANDUSE_GRID);
            uint16_t yt = arpt_quantize((double)(r - LANDUSE_BUFFER) / LANDUSE_GRID);
            uint16_t ym = arpt_quantize((r - LANDUSE_BUFFER + 0.5) / LANDUSE_GRID);
            uint16_t yb = arpt_quantize((double)(r - LANDUSE_BUFFER + 1) / LANDUSE_GRID);

            for (int ui = 0; ui < n_unique; ui++) {
                int cls = unique[ui];

                /* Perimeter walk (clockwise from TL).
                 * For saddle cases this produces overlapping hexagons —
                 * acceptable since the rasterizer paints over the overlap. */
                struct ms_patch *p = &patches[patch_count];
                p->cls = cls;
                int n = 0;

                #define MS_V(px,py) do { p->x[n]=(px); p->y[n]=(py); n++; } while(0)
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
                p->x[n] = p->x[0]; p->y[n] = p->y[0];
                p->count = n + 1;
                patch_count++;
            }
        }
    }

    /* Build FlatBuffer */
    flatcc_builder_t builder;
    flatcc_builder_init(&builder);

    arpentry_tiles_Tile_start_as_root(&builder);
    arpentry_tiles_Tile_version_add(&builder, 1);

    /* Tile-scope property dictionary for landuse "class" key */
    arpentry_tiles_Tile_keys_start(&builder);
    arpentry_tiles_Tile_keys_push_create_str(&builder, "class");
    arpentry_tiles_Tile_keys_end(&builder);

    /* Value dictionary: index 0="grass", 1="forest", 2="sand" */
    arpentry_tiles_Tile_values_start(&builder);
    {
        arpentry_tiles_Tile_values_push_start(&builder);
        arpentry_tiles_Value_type_add(&builder, arpentry_tiles_PropertyValueType_String);
        arpentry_tiles_Value_string_value_create_str(&builder, "grass");
        arpentry_tiles_Tile_values_push_end(&builder);

        arpentry_tiles_Tile_values_push_start(&builder);
        arpentry_tiles_Value_type_add(&builder, arpentry_tiles_PropertyValueType_String);
        arpentry_tiles_Value_string_value_create_str(&builder, "forest");
        arpentry_tiles_Tile_values_push_end(&builder);

        arpentry_tiles_Tile_values_push_start(&builder);
        arpentry_tiles_Value_type_add(&builder, arpentry_tiles_PropertyValueType_String);
        arpentry_tiles_Value_string_value_create_str(&builder, "sand");
        arpentry_tiles_Tile_values_push_end(&builder);
    }
    arpentry_tiles_Tile_values_end(&builder);

    arpentry_tiles_Tile_layers_start(&builder);
    {
        /* Layer 0: terrain (existing) */
        arpentry_tiles_Tile_layers_push_start(&builder);
        arpentry_tiles_Layer_name_create_str(&builder, "terrain");

        arpentry_tiles_Layer_features_start(&builder);
        {
            arpentry_tiles_Layer_features_push_start(&builder);
            arpentry_tiles_Feature_id_add(&builder, 1);

            /* Build MeshGeometry */
            arpentry_tiles_MeshGeometry_ref_t mesh_ref;
            arpentry_tiles_MeshGeometry_start(&builder);
            arpentry_tiles_MeshGeometry_x_create(&builder, vx, (size_t)nv);
            arpentry_tiles_MeshGeometry_y_create(&builder, vy, (size_t)nv);
            arpentry_tiles_MeshGeometry_z_create(&builder, vz, (size_t)nv);
            arpentry_tiles_MeshGeometry_indices_create(&builder, indices,
                                                        (size_t)num_indices);
            arpentry_tiles_MeshGeometry_normals_create(&builder, normals,
                                                        (size_t)(nv * 2));

            /* Single part covering all triangles, client-styled (a=0) */
            arpentry_tiles_MeshGeometry_parts_start(&builder);
            arpentry_tiles_Part_t part = {0};
            part.first_index = 0;
            part.index_count = (uint32_t)num_indices;
            part.color.r = 0;
            part.color.g = 0;
            part.color.b = 0;
            part.color.a = 0;
            part.roughness = 0;
            part.metalness = 0;
            arpentry_tiles_MeshGeometry_parts_push(&builder, &part);
            arpentry_tiles_MeshGeometry_parts_end(&builder);

            mesh_ref = arpentry_tiles_MeshGeometry_end(&builder);
            arpentry_tiles_Feature_geometry_MeshGeometry_add(&builder, mesh_ref);

            arpentry_tiles_Layer_features_push_end(&builder);
        }
        arpentry_tiles_Layer_features_end(&builder);
        arpentry_tiles_Tile_layers_push_end(&builder);

        /* Layer 1: landuse (marching squares patches) */
        arpentry_tiles_Tile_layers_push_start(&builder);
        arpentry_tiles_Layer_name_create_str(&builder, "landuse");

        arpentry_tiles_Layer_features_start(&builder);
        for (int pi = 0; pi < patch_count; pi++) {
            struct ms_patch *p = &patches[pi];
            int32_t pz[9] = {0};
            uint32_t ring_off[2] = {0, (uint32_t)p->count};

            arpentry_tiles_Layer_features_push_start(&builder);
            arpentry_tiles_Feature_id_add(&builder, (uint64_t)(pi + 2));

            arpentry_tiles_PolygonGeometry_ref_t poly_ref;
            arpentry_tiles_PolygonGeometry_start(&builder);
            arpentry_tiles_PolygonGeometry_x_create(&builder, p->x,
                                                     (size_t)p->count);
            arpentry_tiles_PolygonGeometry_y_create(&builder, p->y,
                                                     (size_t)p->count);
            arpentry_tiles_PolygonGeometry_z_create(&builder, pz,
                                                     (size_t)p->count);
            arpentry_tiles_PolygonGeometry_ring_offsets_create(
                &builder, ring_off, 2);
            poly_ref = arpentry_tiles_PolygonGeometry_end(&builder);
            arpentry_tiles_Feature_geometry_PolygonGeometry_add(
                &builder, poly_ref);

            arpentry_tiles_Feature_properties_start(&builder);
            arpentry_tiles_Property_t prop;
            prop.key = 0;
            prop.value = (uint32_t)p->cls;
            arpentry_tiles_Feature_properties_push(&builder, &prop);
            arpentry_tiles_Feature_properties_end(&builder);

            arpentry_tiles_Layer_features_push_end(&builder);
        }
        arpentry_tiles_Layer_features_end(&builder);
        arpentry_tiles_Tile_layers_push_end(&builder);
    }
    arpentry_tiles_Tile_layers_end(&builder);

    arpentry_tiles_Tile_end_as_root(&builder);

    size_t fb_size;
    void *fb = flatcc_builder_finalize_buffer(&builder, &fb_size);
    flatcc_builder_clear(&builder);

    /* Clean up vertex/index/patch arrays */
    free(elev);
    free(vx);
    free(vy);
    free(vz);
    free(normals);
    free(indices);
    free(patches);

    if (!fb) return false;

    bool ok = arpt_encode(fb, fb_size, out, out_size, BROTLI_QUALITY);
    free(fb);
    return ok;

fail:
    free(elev);
    free(vx);
    free(vy);
    free(vz);
    free(normals);
    free(indices);
    free(patches);
    return false;
}
