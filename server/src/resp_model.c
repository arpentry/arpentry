#include "resp_model.h"
#include "model_builder.h"
#include "tile.h"

#include <math.h>
#include <stdlib.h>

#define BROTLI_QUALITY 4

#define SIDES 8

/* Model-space center in millimetres.  Must exceed the largest crown radius
   so that all vertex coordinates stay positive (uint16). */
#define CX 10000
#define CY 10000

/* Generate an 8-sided cylinder.
   Returns updated index count; writes updated vertex count to *out_vi.
   part_idx is stored in the w component for material lookup. */
static int gen_cylinder(uint16_t *vx, uint16_t *vy, uint16_t *vz,
                        uint16_t *vw, uint32_t *indices, int vi, int ii,
                        uint16_t radius, uint16_t z_bot, uint16_t z_top,
                        uint16_t part_idx, int *out_vi) {
    int base = vi;
    /* Bottom ring */
    for (int i = 0; i < SIDES; i++) {
        double a = 2.0 * M_PI * i / SIDES;
        vx[vi] = (uint16_t)(CX + radius * cos(a));
        vy[vi] = (uint16_t)(CY + radius * sin(a));
        vz[vi] = z_bot;
        vw[vi] = part_idx;
        vi++;
    }
    /* Top ring */
    for (int i = 0; i < SIDES; i++) {
        double a = 2.0 * M_PI * i / SIDES;
        vx[vi] = (uint16_t)(CX + radius * cos(a));
        vy[vi] = (uint16_t)(CY + radius * sin(a));
        vz[vi] = z_top;
        vw[vi] = part_idx;
        vi++;
    }
    int bot0 = base;
    int top0 = base + SIDES;
    /* Side quads (2 tris each) */
    for (int i = 0; i < SIDES; i++) {
        int n = (i + 1) % SIDES;
        indices[ii++] = (uint32_t)(bot0 + i);
        indices[ii++] = (uint32_t)(bot0 + n);
        indices[ii++] = (uint32_t)(top0 + i);
        indices[ii++] = (uint32_t)(top0 + i);
        indices[ii++] = (uint32_t)(bot0 + n);
        indices[ii++] = (uint32_t)(top0 + n);
    }
    /* Bottom cap */
    for (int i = 1; i < SIDES - 1; i++) {
        indices[ii++] = (uint32_t)(bot0);
        indices[ii++] = (uint32_t)(bot0 + i + 1);
        indices[ii++] = (uint32_t)(bot0 + i);
    }
    /* Top cap */
    for (int i = 1; i < SIDES - 1; i++) {
        indices[ii++] = (uint32_t)(top0);
        indices[ii++] = (uint32_t)(top0 + i);
        indices[ii++] = (uint32_t)(top0 + i + 1);
    }
    *out_vi = vi;
    return ii;
}

/* Generate an 8-sided cone.  Returns updated index count. */
static int gen_cone(uint16_t *vx, uint16_t *vy, uint16_t *vz, uint16_t *vw,
                    uint32_t *indices, int vi, int ii, uint16_t radius,
                    uint16_t z_base, uint16_t z_apex, uint16_t part_idx,
                    int *out_vi) {
    int base = vi;
    /* Base ring */
    for (int i = 0; i < SIDES; i++) {
        double a = 2.0 * M_PI * i / SIDES;
        vx[vi] = (uint16_t)(CX + radius * cos(a));
        vy[vi] = (uint16_t)(CY + radius * sin(a));
        vz[vi] = z_base;
        vw[vi] = part_idx;
        vi++;
    }
    /* Apex */
    vx[vi] = CX;
    vy[vi] = CY;
    vz[vi] = z_apex;
    vw[vi] = part_idx;
    int apex = vi;
    vi++;
    /* Side triangles */
    for (int i = 0; i < SIDES; i++) {
        int n = (i + 1) % SIDES;
        indices[ii++] = (uint32_t)apex;
        indices[ii++] = (uint32_t)(base + i);
        indices[ii++] = (uint32_t)(base + n);
    }
    /* Base cap */
    for (int i = 1; i < SIDES - 1; i++) {
        indices[ii++] = (uint32_t)base;
        indices[ii++] = (uint32_t)(base + i + 1);
        indices[ii++] = (uint32_t)(base + i);
    }
    *out_vi = vi;
    return ii;
}

/* Generate a UV sphere approximation.  Returns updated index count. */
#define SPHERE_LAT 4
#define SPHERE_LON 8

static int gen_sphere(uint16_t *vx, uint16_t *vy, uint16_t *vz, uint16_t *vw,
                      uint32_t *indices, int vi, int ii, uint16_t radius,
                      uint16_t cz, uint16_t part_idx, int *out_vi) {
    int base = vi;
    /* Top pole */
    vx[vi] = CX;
    vy[vi] = CY;
    vz[vi] = (uint16_t)(cz + radius);
    vw[vi] = part_idx;
    vi++;
    /* Latitude rings */
    for (int lat = 1; lat < SPHERE_LAT; lat++) {
        double phi = M_PI * lat / SPHERE_LAT;
        double sp = sin(phi);
        double cp = cos(phi);
        for (int lon = 0; lon < SPHERE_LON; lon++) {
            double theta = 2.0 * M_PI * lon / SPHERE_LON;
            vx[vi] = (uint16_t)(int)(CX + radius * sp * cos(theta));
            vy[vi] = (uint16_t)(int)(CY + radius * sp * sin(theta));
            vz[vi] = (uint16_t)(cz + (int16_t)(radius * cp));
            vw[vi] = part_idx;
            vi++;
        }
    }
    /* Bottom pole */
    vx[vi] = CX;
    vy[vi] = CY;
    vz[vi] = (uint16_t)(cz - radius);
    vw[vi] = part_idx;
    int bot_pole = vi;
    vi++;

    int top_pole = base;
    /* Top cap triangles */
    for (int i = 0; i < SPHERE_LON; i++) {
        int n = (i + 1) % SPHERE_LON;
        indices[ii++] = (uint32_t)top_pole;
        indices[ii++] = (uint32_t)(base + 1 + i);
        indices[ii++] = (uint32_t)(base + 1 + n);
    }
    /* Middle quads */
    for (int lat = 0; lat < SPHERE_LAT - 2; lat++) {
        int row0 = base + 1 + lat * SPHERE_LON;
        int row1 = row0 + SPHERE_LON;
        for (int i = 0; i < SPHERE_LON; i++) {
            int n = (i + 1) % SPHERE_LON;
            indices[ii++] = (uint32_t)(row0 + i);
            indices[ii++] = (uint32_t)(row1 + i);
            indices[ii++] = (uint32_t)(row0 + n);
            indices[ii++] = (uint32_t)(row0 + n);
            indices[ii++] = (uint32_t)(row1 + i);
            indices[ii++] = (uint32_t)(row1 + n);
        }
    }
    /* Bottom cap triangles */
    int last_row = base + 1 + (SPHERE_LAT - 2) * SPHERE_LON;
    for (int i = 0; i < SPHERE_LON; i++) {
        int n = (i + 1) % SPHERE_LON;
        indices[ii++] = (uint32_t)bot_pole;
        indices[ii++] = (uint32_t)(last_row + n);
        indices[ii++] = (uint32_t)(last_row + i);
    }
    *out_vi = vi;
    return ii;
}

/* Write one model into the builder: name, x/y/z/w arrays, indices, 2 Parts.
   Roughness 200/255 ≈ 0.78 gives a matte natural look; metalness 0 = dielectric. */
static void emit_model(flatcc_builder_t *b, const char *name,
                        const uint16_t *vx, const uint16_t *vy,
                        const uint16_t *vz, const uint16_t *vw, int nv,
                        const uint32_t *indices, int ni, int trunk_last_idx,
                        arpentry_tiles_Color_t trunk_col,
                        arpentry_tiles_Color_t crown_col) {
    arpentry_tiles_ModelLibrary_models_push_start(b);
    arpentry_tiles_Model_name_create_str(b, name);

    arpentry_tiles_Model_x_create(b, vx, (size_t)nv);
    arpentry_tiles_Model_y_create(b, vy, (size_t)nv);
    arpentry_tiles_Model_z_create(b, vz, (size_t)nv);
    arpentry_tiles_Model_w_create(b, vw, (size_t)nv);
    arpentry_tiles_Model_indices_create(b, indices, (size_t)ni);

    arpentry_tiles_Model_parts_start(b);
    arpentry_tiles_Part_t trunk_part = {
        .first_index = 0,
        .index_count = (uint32_t)trunk_last_idx,
        .roughness = 200,
        .metalness = 0,
    };
    trunk_part.color = trunk_col;
    arpentry_tiles_Model_parts_push(b, &trunk_part);
    arpentry_tiles_Part_t crown_part = {
        .first_index = (uint32_t)trunk_last_idx,
        .index_count = (uint32_t)(ni - trunk_last_idx),
        .roughness = 200,
        .metalness = 0,
    };
    crown_part.color = crown_col;
    arpentry_tiles_Model_parts_push(b, &crown_part);
    arpentry_tiles_Model_parts_end(b);

    arpentry_tiles_ModelLibrary_models_push_end(b);
}

/* Max vertices/indices per model: trunk(16v) + sphere(26v) = 42v, ~600 idx */
#define MAX_MODEL_V 64
#define MAX_MODEL_I 1024

bool resp_build_models(uint8_t **out, size_t *out_size) {
    flatcc_builder_t builder;
    flatcc_builder_init(&builder);

    arpentry_tiles_ModelLibrary_start_as_root(&builder);
    arpentry_tiles_ModelLibrary_version_add(&builder, 1);
    arpentry_tiles_ModelLibrary_models_start(&builder);

    uint16_t vx[MAX_MODEL_V], vy[MAX_MODEL_V], vz[MAX_MODEL_V],
        vw[MAX_MODEL_V];
    uint32_t indices[MAX_MODEL_I];

    /* Dimensions in millimetres.  CX/CY (10000 mm) centre the model so all
       coordinates stay within uint16 range. */

    /* --- Oak: short trunk (2 m, r=400 mm) + wide sphere crown (r=6 m, cz=8 m) */
    {
        int vi = 0, ii = 0;
        ii = gen_cylinder(vx, vy, vz, vw, indices, vi, ii, 400, 0, 2000, 0, &vi);
        int trunk_ii = ii;
        ii = gen_sphere(vx, vy, vz, vw, indices, vi, ii, 6000, 8000, 1, &vi);
        arpentry_tiles_Color_t trunk = {.r = 101, .g = 67, .b = 33, .a = 255};
        arpentry_tiles_Color_t crown = {.r = 34, .g = 85, .b = 25, .a = 255};
        emit_model(&builder, "oak", vx, vy, vz, vw, vi, indices, ii, trunk_ii,
                   trunk, crown);
    }

    /* --- Pine: medium trunk (4 m, r=300 mm) + tall cone crown (base 4 m, apex 18 m, r=3 m) */
    {
        int vi = 0, ii = 0;
        ii = gen_cylinder(vx, vy, vz, vw, indices, vi, ii, 300, 0, 4000, 0, &vi);
        int trunk_ii = ii;
        ii = gen_cone(vx, vy, vz, vw, indices, vi, ii, 3000, 4000, 18000, 1,
                      &vi);
        arpentry_tiles_Color_t trunk = {.r = 101, .g = 67, .b = 33, .a = 255};
        arpentry_tiles_Color_t crown = {.r = 20, .g = 70, .b = 20, .a = 255};
        emit_model(&builder, "pine", vx, vy, vz, vw, vi, indices, ii,
                   trunk_ii, trunk, crown);
    }

    /* --- Birch: slender trunk (5 m, r=200 mm) + small sphere crown (r=3 m, cz=10 m) */
    {
        int vi = 0, ii = 0;
        ii = gen_cylinder(vx, vy, vz, vw, indices, vi, ii, 200, 0, 5000, 0, &vi);
        int trunk_ii = ii;
        ii = gen_sphere(vx, vy, vz, vw, indices, vi, ii, 3000, 10000, 1, &vi);
        arpentry_tiles_Color_t trunk = {.r = 200, .g = 200, .b = 195, .a = 255};
        arpentry_tiles_Color_t crown = {.r = 80, .g = 160, .b = 50, .a = 255};
        emit_model(&builder, "birch", vx, vy, vz, vw, vi, indices, ii,
                   trunk_ii, trunk, crown);
    }

    arpentry_tiles_ModelLibrary_models_end(&builder);
    arpentry_tiles_ModelLibrary_end_as_root(&builder);

    size_t fb_size;
    void *fb = flatcc_builder_finalize_buffer(&builder, &fb_size);
    flatcc_builder_clear(&builder);
    if (!fb) return false;

    bool ok = arpt_encode(fb, fb_size, out, out_size, BROTLI_QUALITY);
    free(fb);
    return ok;
}
