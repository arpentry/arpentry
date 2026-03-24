#include "tile_build.h"
#include "tile_builder.h"

#include <brotli/encode.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Coordinate quantization: map geo coords within tile bounds to uint16.
   Tile proper: [16384, 49151], extent = 32768, buffer = 16384 per side. */
#define TILE_EXTENT  32768
#define TILE_BUFFER  16384

/* String dictionary with open-addressing hash table for O(1) intern. */

/* Hash table slot: maps hash → entry index. Empty slots use UINT32_MAX. */
#define DICT_EMPTY UINT32_MAX

typedef struct {
    char    **entries;
    uint32_t  count;
    uint32_t  entry_cap;
    uint32_t *ht;           /* hash table: slot → entry index */
    uint32_t  ht_cap;       /* always a power of 2 */
} str_dict;

static void str_dict_init(str_dict *d) {
    d->entries = NULL;
    d->count = 0;
    d->entry_cap = 0;
    d->ht = NULL;
    d->ht_cap = 0;
}

static void str_dict_free(str_dict *d) {
    for (uint32_t i = 0; i < d->count; i++) free(d->entries[i]);
    free(d->entries);
    free(d->ht);
}

static uint32_t str_hash(const char *s) {
    uint32_t h = 5381;
    for (; *s; s++)
        h = ((h << 5) + h) ^ (uint32_t)(unsigned char)*s;
    return h;
}

/* Rebuild the hash table after a capacity change. */
static void str_dict_rehash(str_dict *d) {
    for (uint32_t i = 0; i < d->ht_cap; i++)
        d->ht[i] = DICT_EMPTY;
    for (uint32_t i = 0; i < d->count; i++) {
        uint32_t h = str_hash(d->entries[i]) & (d->ht_cap - 1);
        while (d->ht[h] != DICT_EMPTY)
            h = (h + 1) & (d->ht_cap - 1);
        d->ht[h] = i;
    }
}

/* Returns index; adds if not present */
static uint32_t str_dict_intern(str_dict *d, const char *s) {
    /* Grow hash table if needed (keep load < 75%) */
    if (d->ht_cap == 0 || d->count * 4 >= d->ht_cap * 3) {
        uint32_t nc = d->ht_cap ? d->ht_cap * 2 : 16;
        uint32_t *p = realloc(d->ht, nc * sizeof(uint32_t));
        if (!p) return 0;
        d->ht = p;
        d->ht_cap = nc;
        str_dict_rehash(d);
    }

    /* Probe for existing entry */
    uint32_t h = str_hash(s) & (d->ht_cap - 1);
    while (d->ht[h] != DICT_EMPTY) {
        if (strcmp(d->entries[d->ht[h]], s) == 0)
            return d->ht[h];
        h = (h + 1) & (d->ht_cap - 1);
    }

    /* Not found — insert */
    if (d->count == d->entry_cap) {
        uint32_t nc = d->entry_cap ? d->entry_cap * 2 : 16;
        char **p = realloc(d->entries, nc * sizeof(char *));
        if (!p) return 0;
        d->entries = p;
        d->entry_cap = nc;
    }
    d->entries[d->count] = strdup(s);
    if (!d->entries[d->count]) return 0;
    d->ht[h] = d->count;
    return d->count++;
}

/* Stored feature (accumulated before building FlatBuffer) */
typedef struct {
    uint32_t layer;
    uint32_t geom_type;    /* WKB type 1-6 */
    uint16_t *qx, *qy;
    int32_t  *qz;
    uint32_t n_coords;
    uint32_t *offsets;
    uint32_t n_offsets;
    uint32_t *props;       /* [key_idx, val_idx] pairs */
    uint32_t n_props;
} stored_feat;

/* Layer names matching style.json */
static const char *layer_names[] = {
    "terrain", "surface", "highway", "building",
    "tree", "poi", "layer6", "layer7",
    "layer8", "layer9", "layer10", "layer11",
    "layer12", "layer13", "layer14", "layer15"
};

struct arpt_tile_builder {
    arpt_bounds bounds;
    const arpt_dem *dem;

    str_dict keys;
    str_dict vals;

    stored_feat *feats;
    uint32_t n_feats;
    uint32_t feat_cap;
};

static uint16_t quantize_x(const arpt_bounds *b, double x) {
    double t = (x - b->min_x) / (b->max_x - b->min_x);
    double q = t * TILE_EXTENT + TILE_BUFFER;
    if (q < 0.0) q = 0.0;
    if (q > 65535.0) q = 65535.0;
    return (uint16_t)q;
}

static uint16_t quantize_y(const arpt_bounds *b, double y) {
    double t = (y - b->min_y) / (b->max_y - b->min_y);
    double q = t * TILE_EXTENT + TILE_BUFFER;
    if (q < 0.0) q = 0.0;
    if (q > 65535.0) q = 65535.0;
    return (uint16_t)q;
}

arpt_tile_builder *arpt_tile_builder_create(arpt_bounds bounds,
                                            const arpt_dem *dem) {
    arpt_tile_builder *b = calloc(1, sizeof(*b));
    if (!b) return NULL;
    b->bounds = bounds;
    b->dem = dem;
    str_dict_init(&b->keys);
    str_dict_init(&b->vals);
    return b;
}

bool arpt_tile_builder_add_feature(arpt_tile_builder *b,
                                   const arpt_feature *feat) {
    if (!b || !feat || !feat->geom) return false;
    const arpt_geom *g = feat->geom;
    if (g->n_coords == 0) return false;

    /* Grow feature array */
    if (b->n_feats == b->feat_cap) {
        uint32_t nc = b->feat_cap ? b->feat_cap * 2 : 16;
        stored_feat *p = realloc(b->feats, nc * sizeof(stored_feat));
        if (!p) return false;
        b->feats = p;
        b->feat_cap = nc;
    }

    stored_feat *sf = &b->feats[b->n_feats];
    memset(sf, 0, sizeof(*sf));
    sf->layer = feat->layer;
    sf->geom_type = g->type;
    sf->n_coords = g->n_coords;

    /* Quantize coordinates */
    sf->qx = malloc(g->n_coords * sizeof(uint16_t));
    sf->qy = malloc(g->n_coords * sizeof(uint16_t));
    sf->qz = calloc(g->n_coords, sizeof(int32_t));
    if (!sf->qx || !sf->qy || !sf->qz) {
        free(sf->qx); free(sf->qy); free(sf->qz);
        return false;
    }

    for (uint32_t i = 0; i < g->n_coords; i++) {
        sf->qx[i] = quantize_x(&b->bounds, g->x[i]);
        sf->qy[i] = quantize_y(&b->bounds, g->y[i]);
        if (g->z) sf->qz[i] = (int32_t)(g->z[i] * 1000.0);
    }

    /* Copy offsets */
    if (g->offsets && g->n_offsets > 0) {
        sf->offsets = malloc(g->n_offsets * sizeof(uint32_t));
        if (!sf->offsets) {
            free(sf->qx); free(sf->qy); free(sf->qz);
            return false;
        }
        memcpy(sf->offsets, g->offsets, g->n_offsets * sizeof(uint32_t));
        sf->n_offsets = g->n_offsets;
    }

    /* Intern properties */
    if (feat->n_props > 0 && feat->prop_keys && feat->prop_vals) {
        sf->props = malloc(feat->n_props * 2 * sizeof(uint32_t));
        if (!sf->props) {
            free(sf->qx); free(sf->qy); free(sf->qz); free(sf->offsets);
            return false;
        }
        for (uint32_t i = 0; i < feat->n_props; i++) {
            sf->props[i * 2]     = str_dict_intern(&b->keys, feat->prop_keys[i]);
            sf->props[i * 2 + 1] = str_dict_intern(&b->vals, feat->prop_vals[i]);
        }
        sf->n_props = feat->n_props;
    }

    b->n_feats++;
    return true;
}

/* Build geometry for one feature into the flatcc builder */
static void build_geom(flatcc_builder_t *fb, const stored_feat *sf) {
    switch (sf->geom_type) {
    case 1: case 4: { /* Point / MultiPoint */
        arpentry_tiles_PointGeometry_start(fb);
        arpentry_tiles_PointGeometry_x_create(fb, sf->qx, sf->n_coords);
        arpentry_tiles_PointGeometry_y_create(fb, sf->qy, sf->n_coords);
        arpentry_tiles_PointGeometry_z_create(fb, sf->qz, sf->n_coords);
        arpentry_tiles_PointGeometry_ref_t ref = arpentry_tiles_PointGeometry_end(fb);
        arpentry_tiles_Feature_geometry_PointGeometry_add(fb, ref);
        break;
    }
    case 2: case 5: { /* LineString / MultiLineString */
        arpentry_tiles_LineGeometry_start(fb);
        arpentry_tiles_LineGeometry_x_create(fb, sf->qx, sf->n_coords);
        arpentry_tiles_LineGeometry_y_create(fb, sf->qy, sf->n_coords);
        arpentry_tiles_LineGeometry_z_create(fb, sf->qz, sf->n_coords);
        if (sf->offsets && sf->n_offsets > 0) {
            arpentry_tiles_LineGeometry_line_offsets_create(fb, sf->offsets, sf->n_offsets);
        }
        arpentry_tiles_LineGeometry_ref_t ref = arpentry_tiles_LineGeometry_end(fb);
        arpentry_tiles_Feature_geometry_LineGeometry_add(fb, ref);
        break;
    }
    case 3: case 6: { /* Polygon / MultiPolygon */
        arpentry_tiles_PolygonGeometry_start(fb);
        arpentry_tiles_PolygonGeometry_x_create(fb, sf->qx, sf->n_coords);
        arpentry_tiles_PolygonGeometry_y_create(fb, sf->qy, sf->n_coords);
        arpentry_tiles_PolygonGeometry_z_create(fb, sf->qz, sf->n_coords);
        if (sf->offsets && sf->n_offsets > 0) {
            arpentry_tiles_PolygonGeometry_ring_offsets_create(fb, sf->offsets, sf->n_offsets);
        }
        arpentry_tiles_PolygonGeometry_ref_t ref = arpentry_tiles_PolygonGeometry_end(fb);
        arpentry_tiles_Feature_geometry_PolygonGeometry_add(fb, ref);
        break;
    }
    default:
        break;
    }
}

/* Grid subdivisions for the terrain mesh.  Must be high enough
   that the vertex shader can deform the quad into a curved globe
   surface.  64×64 = 4225 vertices, 2×64×64 = 8192 triangles. */
#define TERRAIN_GRID 64

/* Encode a unit normal vector into octahedral int8×2.
   Input: (nx, ny, nz) must be normalized. */
static void encode_octahedral(double nx, double ny, double nz,
                               int8_t *out_x, int8_t *out_y) {
    /* Project onto octahedron */
    double inv = 1.0 / (fabs(nx) + fabs(ny) + fabs(nz));
    double ox = nx * inv;
    double oy = ny * inv;

    /* Reflect lower hemisphere */
    if (nz < 0.0) {
        double tmp_x = (1.0 - fabs(oy)) * (ox >= 0.0 ? 1.0 : -1.0);
        double tmp_y = (1.0 - fabs(ox)) * (oy >= 0.0 ? 1.0 : -1.0);
        ox = tmp_x;
        oy = tmp_y;
    }

    /* Quantize to int8 [-127, 127] */
    double sx = ox * 127.0;
    double sy = oy * 127.0;
    if (sx > 127.0) sx = 127.0;
    if (sx < -127.0) sx = -127.0;
    if (sy > 127.0) sy = 127.0;
    if (sy < -127.0) sy = -127.0;
    *out_x = (int8_t)(sx >= 0.0 ? sx + 0.5 : sx - 0.5);
    *out_y = (int8_t)(sy >= 0.0 ? sy + 0.5 : sy - 0.5);
}

/* Emit a subdivided terrain mesh covering the tile extent as layer 0.
   If a DEM is provided, vertices get real elevation and computed normals.
   Otherwise the mesh is flat (z=0, normals pointing up). */
static void emit_terrain(flatcc_builder_t *fb, const arpt_bounds *bounds,
                          const arpt_dem *dem) {
    const uint32_t gn = TERRAIN_GRID;
    const uint32_t n_verts = (gn + 1) * (gn + 1);
    const uint32_t n_tris = gn * gn * 2;
    const uint32_t n_idx = n_tris * 3;

    uint16_t *vx = malloc(n_verts * sizeof(*vx));
    uint16_t *vy = malloc(n_verts * sizeof(*vy));
    int32_t  *vz = calloc(n_verts, sizeof(*vz));
    uint32_t *indices = malloc(n_idx * sizeof(*indices));
    int8_t   *normals = malloc(n_verts * 2 * sizeof(*normals));
    if (!vx || !vy || !vz || !indices || !normals) {
        free(vx); free(vy); free(vz); free(indices); free(normals);
        return;
    }

    double lon_span = bounds->max_x - bounds->min_x;
    double lat_span = bounds->max_y - bounds->min_y;

    /* Generate grid vertices with elevation */
    for (uint32_t row = 0; row <= gn; row++) {
        for (uint32_t col = 0; col <= gn; col++) {
            uint32_t vi = row * (gn + 1) + col;
            vx[vi] = (uint16_t)(TILE_BUFFER +
                      (uint32_t)((uint64_t)col * (TILE_EXTENT - 1) / gn));
            vy[vi] = (uint16_t)(TILE_BUFFER +
                      (uint32_t)((uint64_t)row * (TILE_EXTENT - 1) / gn));

            if (dem) {
                double lon = bounds->min_x + (double)col / gn * lon_span;
                double lat = bounds->min_y + (double)row / gn * lat_span;
                double elev = arpt_dem_sample(dem, lon, lat);
                /* Clamp ocean (negative elevation) to 0 for terrain mesh */
                if (elev < 0.0) elev = 0.0;
                vz[vi] = (int32_t)(elev * 1000.0);
            }
        }
    }

    /* Compute normals in ECEF using ENU basis vectors.
       The terrain shader transforms normals by tile.model which operates
       in ECEF space, so normals must be in ECEF — not tile-local. */
    {
        double dx_deg = lon_span / gn;
        double dy_deg = lat_span / gn;

        for (uint32_t row = 0; row <= gn; row++) {
            for (uint32_t col = 0; col <= gn; col++) {
                uint32_t vi = row * (gn + 1) + col;

                double lon = bounds->min_x + (double)col / gn * lon_span;
                double lat = bounds->min_y + (double)row / gn * lat_span;
                double lon_r = lon * M_PI / 180.0;
                double lat_r = lat * M_PI / 180.0;
                double sin_lon = sin(lon_r), cos_lon = cos(lon_r);
                double sin_lat = sin(lat_r), cos_lat = cos(lat_r);

                /* ENU basis vectors in ECEF */
                double e_x = -sin_lon,           e_y = cos_lon,            e_z = 0.0;
                double n_x = -sin_lat * cos_lon, n_y = -sin_lat * sin_lon, n_z = cos_lat;
                double u_x =  cos_lat * cos_lon, u_y =  cos_lat * sin_lon, u_z = sin_lat;

                double dzdx = 0.0, dzdy = 0.0;
                if (dem) {
                    double z_xp = arpt_dem_sample(dem, lon + dx_deg, lat);
                    double z_xm = arpt_dem_sample(dem, lon - dx_deg, lat);
                    double z_yp = arpt_dem_sample(dem, lon, lat + dy_deg);
                    double z_ym = arpt_dem_sample(dem, lon, lat - dy_deg);

                    if (z_xp < 0.0) z_xp = 0.0;
                    if (z_xm < 0.0) z_xm = 0.0;
                    if (z_yp < 0.0) z_yp = 0.0;
                    if (z_ym < 0.0) z_ym = 0.0;

                    double cell_w = dx_deg * 111320.0 * cos_lat * 2.0;
                    double cell_h = dy_deg * 111320.0 * 2.0;
                    if (cell_w > 0.0) dzdx = (z_xp - z_xm) / cell_w;
                    if (cell_h > 0.0) dzdy = (z_yp - z_ym) / cell_h;
                }

                /* ECEF normal: up - dzdx*east - dzdy*north */
                double nx = u_x - dzdx * e_x - dzdy * n_x;
                double ny = u_y - dzdx * e_y - dzdy * n_y;
                double nz = u_z - dzdx * e_z - dzdy * n_z;
                double len = sqrt(nx * nx + ny * ny + nz * nz);
                nx /= len; ny /= len; nz /= len;

                encode_octahedral(nx, ny, nz,
                                  &normals[vi * 2], &normals[vi * 2 + 1]);
            }
        }
    }

    /* Generate triangle indices (two triangles per grid cell) */
    uint32_t ii = 0;
    for (uint32_t row = 0; row < gn; row++) {
        for (uint32_t col = 0; col < gn; col++) {
            uint32_t tl = row * (gn + 1) + col;
            uint32_t tr = tl + 1;
            uint32_t bl = tl + (gn + 1);
            uint32_t br = bl + 1;
            indices[ii++] = tl;
            indices[ii++] = tr;
            indices[ii++] = br;
            indices[ii++] = tl;
            indices[ii++] = br;
            indices[ii++] = bl;
        }
    }

    arpentry_tiles_Tile_layers_push_start(fb);
    arpentry_tiles_Layer_name_create_str(fb, "terrain");

    arpentry_tiles_Layer_features_start(fb);
    arpentry_tiles_Layer_features_push_start(fb);
    arpentry_tiles_Feature_id_add(fb, 0);

    arpentry_tiles_MeshGeometry_start(fb);
    arpentry_tiles_MeshGeometry_x_create(fb, vx, n_verts);
    arpentry_tiles_MeshGeometry_y_create(fb, vy, n_verts);
    arpentry_tiles_MeshGeometry_z_create(fb, vz, n_verts);
    arpentry_tiles_MeshGeometry_indices_create(fb, indices, n_idx);
    arpentry_tiles_MeshGeometry_normals_create(fb, normals, n_verts * 2);

    arpentry_tiles_MeshGeometry_parts_start(fb);
    arpentry_tiles_Part_t part = {0};
    part.first_index = 0;
    part.index_count = n_idx;
    /* color.a = 0 → client-styled */
    arpentry_tiles_MeshGeometry_parts_push(fb, &part);
    arpentry_tiles_MeshGeometry_parts_end(fb);

    arpentry_tiles_MeshGeometry_ref_t mesh_ref = arpentry_tiles_MeshGeometry_end(fb);
    arpentry_tiles_Feature_geometry_MeshGeometry_add(fb, mesh_ref);

    arpentry_tiles_Layer_features_push_end(fb);
    arpentry_tiles_Layer_features_end(fb);
    arpentry_tiles_Tile_layers_push_end(fb);

    free(vx);
    free(vy);
    free(vz);
    free(indices);
    free(normals);
}

void *arpt_tile_builder_finish(arpt_tile_builder *b, size_t *out_size) {
    if (!b) { if (out_size) *out_size = 0; return NULL; }

    flatcc_builder_t fb;
    flatcc_builder_init(&fb);

    arpentry_tiles_Tile_start_as_root(&fb);
    arpentry_tiles_Tile_version_add(&fb, 1);

    /* Keys dictionary */
    if (b->keys.count > 0) {
        arpentry_tiles_Tile_keys_start(&fb);
        for (uint32_t i = 0; i < b->keys.count; i++) {
            arpentry_tiles_Tile_keys_push_create_str(&fb, b->keys.entries[i]);
        }
        arpentry_tiles_Tile_keys_end(&fb);
    }

    /* Values dictionary (all stored as strings for now) */
    if (b->vals.count > 0) {
        arpentry_tiles_Tile_values_start(&fb);
        for (uint32_t i = 0; i < b->vals.count; i++) {
            arpentry_tiles_Tile_values_push_start(&fb);
            arpentry_tiles_Value_type_add(&fb, arpentry_tiles_PropertyValueType_String);
            arpentry_tiles_Value_string_value_create_str(&fb, b->vals.entries[i]);
            arpentry_tiles_Tile_values_push_end(&fb);
        }
        arpentry_tiles_Tile_values_end(&fb);
    }

    /* Group features by layer */
    uint32_t max_layer = 0;
    for (uint32_t i = 0; i < b->n_feats; i++) {
        if (b->feats[i].layer > max_layer) max_layer = b->feats[i].layer;
    }

    arpentry_tiles_Tile_layers_start(&fb);

    /* Emit a flat terrain mesh as layer 0 if no features claim it */
    bool has_layer0 = false;
    for (uint32_t i = 0; i < b->n_feats; i++) {
        if (b->feats[i].layer == 0) { has_layer0 = true; break; }
    }
    if (!has_layer0) {
        emit_terrain(&fb, &b->bounds, b->dem);
    }

    for (uint32_t layer = 0; layer <= max_layer; layer++) {
        /* Check if any features in this layer */
        bool has_feats = false;
        for (uint32_t i = 0; i < b->n_feats; i++) {
            if (b->feats[i].layer == layer) { has_feats = true; break; }
        }
        if (!has_feats) continue;

        arpentry_tiles_Tile_layers_push_start(&fb);
        const char *name = layer < 16 ? layer_names[layer] : "default";
        arpentry_tiles_Layer_name_create_str(&fb, name);

        arpentry_tiles_Layer_features_start(&fb);
        for (uint32_t i = 0; i < b->n_feats; i++) {
            if (b->feats[i].layer != layer) continue;
            const stored_feat *sf = &b->feats[i];

            arpentry_tiles_Layer_features_push_start(&fb);
            arpentry_tiles_Feature_id_add(&fb, i);

            build_geom(&fb, sf);

            /* Properties */
            if (sf->n_props > 0) {
                arpentry_tiles_Property_t *props =
                    malloc(sf->n_props * sizeof(arpentry_tiles_Property_t));
                if (props) {
                    for (uint32_t j = 0; j < sf->n_props; j++) {
                        props[j].key = sf->props[j * 2];
                        props[j].value = sf->props[j * 2 + 1];
                    }
                    arpentry_tiles_Feature_properties_create(&fb, props, sf->n_props);
                    free(props);
                }
            }

            arpentry_tiles_Layer_features_push_end(&fb);
        }
        arpentry_tiles_Layer_features_end(&fb);
        arpentry_tiles_Tile_layers_push_end(&fb);
    }
    arpentry_tiles_Tile_layers_end(&fb);

    arpentry_tiles_Tile_end_as_root(&fb);

    /* Finalize FlatBuffer */
    size_t fb_size;
    void *fb_buf = flatcc_builder_finalize_buffer(&fb, &fb_size);
    flatcc_builder_clear(&fb);

    if (!fb_buf) {
        if (out_size) *out_size = 0;
        return NULL;
    }

    /* Brotli compress */
    size_t max_compressed = BrotliEncoderMaxCompressedSize(fb_size);
    if (max_compressed == 0) max_compressed = fb_size + 64;

    uint8_t *compressed = malloc(max_compressed);
    if (!compressed) {
        free(fb_buf);
        if (out_size) *out_size = 0;
        return NULL;
    }

    size_t compressed_size = max_compressed;
    if (!BrotliEncoderCompress(4, BROTLI_DEFAULT_WINDOW, BROTLI_DEFAULT_MODE,
                               fb_size, (const uint8_t *)fb_buf,
                               &compressed_size, compressed)) {
        free(fb_buf);
        free(compressed);
        if (out_size) *out_size = 0;
        return NULL;
    }

    free(fb_buf);
    if (out_size) *out_size = compressed_size;
    return compressed;
}

void arpt_tile_builder_free(arpt_tile_builder *b) {
    if (!b) return;
    for (uint32_t i = 0; i < b->n_feats; i++) {
        free(b->feats[i].qx);
        free(b->feats[i].qy);
        free(b->feats[i].qz);
        free(b->feats[i].offsets);
        free(b->feats[i].props);
    }
    free(b->feats);
    str_dict_free(&b->keys);
    str_dict_free(&b->vals);
    free(b);
}
