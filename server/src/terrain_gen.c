#include "terrain_gen.h"
#include "noise.h"
#include "coords.h"
#include "tile.h"
#include "tile_builder.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ── Terrain parameters ────────────────────────────────────────────── */

#define TERRAIN_GRID        32
#define TERRAIN_VERTS       (TERRAIN_GRID + 1)  /* 33x33 = 1089 vertices */
#define TERRAIN_OCTAVES     16
#define TERRAIN_BASE_FREQ   4.0
#define TERRAIN_AMPLITUDE   8000.0   /* peak ~8000m */
#define TERRAIN_LACUNARITY  2.0
#define TERRAIN_PERSISTENCE 0.65
#define TERRAIN_REDISTRIBUTE 0.45  /* power curve exponent: < 1 pushes toward extremes */
#define BROTLI_QUALITY      4

#define PI 3.14159265358979323846

/* ── Helpers ────────────────────────────────────────────────────────── */

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

/* ── Public API ────────────────────────────────────────────────────── */

bool arpt_generate_terrain(int level, int x, int y,
                           uint8_t **out, size_t *out_size)
{
    if (!out || !out_size) return false;

    arpt_bounds_t bounds = arpt_tile_bounds(level, x, y);
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

    /* Build FlatBuffer */
    flatcc_builder_t builder;
    flatcc_builder_init(&builder);

    arpentry_tiles_Tile_start_as_root(&builder);
    arpentry_tiles_Tile_version_add(&builder, 1);

    /* No properties needed for terrain */

    arpentry_tiles_Tile_layers_start(&builder);
    {
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
    }
    arpentry_tiles_Tile_layers_end(&builder);

    arpentry_tiles_Tile_end_as_root(&builder);

    size_t fb_size;
    void *fb = flatcc_builder_finalize_buffer(&builder, &fb_size);
    flatcc_builder_clear(&builder);

    /* Clean up vertex/index arrays */
    free(elev);
    free(vx);
    free(vy);
    free(vz);
    free(normals);
    free(indices);

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
    return false;
}
