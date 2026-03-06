#include "prepare.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Terrain — trivial zero-copy assignment */

void arpt_prepare_terrain(const arpt_terrain_mesh *mesh, arpt_mesh_prim *out) {
    out->x = mesh->x;
    out->y = mesh->y;
    out->z = mesh->z;
    out->normals = mesh->normals;
    out->vertex_count = mesh->vertex_count;
    out->indices = mesh->indices;
    out->index_count = mesh->index_count;
}

/* Texture — tessellate surface polygons and highway SDF quads */

static void count_polygon_geom(const arpt_surface_data *data,
                                size_t *out_verts, size_t *out_indices) {
    if (!data) return;
    for (size_t i = 0; i < data->count; i++) {
        size_t vc = data->polygons[i].vertex_count;
        if (vc < 3) continue;
        *out_verts += vc;
        *out_indices += (vc - 2) * 3;
    }
}

static void emit_polygons(const arpt_surface_data *data,
                           const arpt_style *style,
                           arpt_poly_vertex *verts, uint32_t *idxs,
                           size_t *vi, size_t *ii) {
    if (!data) return;
    for (size_t i = 0; i < data->count; i++) {
        const arpt_surface_polygon *p = &data->polygons[i];
        if (p->vertex_count < 3) continue;

        const float *c = style->colors[p->cls];
        uint32_t base = (uint32_t)*vi;

        for (size_t v = 0; v < p->vertex_count; v++) {
            verts[*vi] = (arpt_poly_vertex){p->x[v], p->y[v],
                                             c[0], c[1], c[2], c[3]};
            (*vi)++;
        }
        for (size_t v = 1; v + 1 < p->vertex_count; v++) {
            idxs[(*ii)++] = base;
            idxs[(*ii)++] = base + (uint32_t)v;
            idxs[(*ii)++] = base + (uint32_t)v + 1;
        }
    }
}

static void emit_highway_sdf_quads(const arpt_highway_data *data,
                                    const arpt_style *style,
                                    arpt_line_vertex *verts, uint32_t *idxs,
                                    size_t *vi, size_t *ii) {
    if (!data) return;
    for (size_t i = 0; i < data->count; i++) {
        const arpt_highway_line *line = &data->lines[i];
        double hw = style->stroke_widths[line->cls];
        const float *c = style->colors[line->cls];

        for (size_t s = 0; s + 1 < line->vertex_count; s++) {
            double x1 = line->x[s], y1 = line->y[s];
            double x2 = line->x[s + 1], y2 = line->y[s + 1];
            double dx = x2 - x1, dy = y2 - y1;
            double len = sqrt(dx * dx + dy * dy);
            if (len < 1.0) continue;

            double ux = dx / len, uy = dy / len;
            double nx = -uy, ny = ux;

            double ex1 = x1 - ux * hw, ey1 = y1 - uy * hw;
            double ex2 = x2 + ux * hw, ey2 = y2 + uy * hw;

#define CLAMP16(v) ((uint16_t)((v) < 0 ? 0 : (v) > 65535 ? 65535 : (v)))
            uint32_t base = (uint32_t)*vi;
            verts[*vi] = (arpt_line_vertex){
                CLAMP16(ex1 - nx * hw), CLAMP16(ey1 - ny * hw),
                c[0], c[1], c[2], c[3],
                (float)(-hw), (float)(-hw), (float)hw, (float)len};
            (*vi)++;
            verts[*vi] = (arpt_line_vertex){
                CLAMP16(ex1 + nx * hw), CLAMP16(ey1 + ny * hw),
                c[0], c[1], c[2], c[3],
                (float)(-hw), (float)(hw), (float)hw, (float)len};
            (*vi)++;
            verts[*vi] = (arpt_line_vertex){
                CLAMP16(ex2 + nx * hw), CLAMP16(ey2 + ny * hw),
                c[0], c[1], c[2], c[3],
                (float)(len + hw), (float)(hw), (float)hw, (float)len};
            (*vi)++;
            verts[*vi] = (arpt_line_vertex){
                CLAMP16(ex2 - nx * hw), CLAMP16(ey2 - ny * hw),
                c[0], c[1], c[2], c[3],
                (float)(len + hw), (float)(-hw), (float)hw, (float)len};
            (*vi)++;
#undef CLAMP16

            idxs[(*ii)++] = base;
            idxs[(*ii)++] = base + 1;
            idxs[(*ii)++] = base + 2;
            idxs[(*ii)++] = base;
            idxs[(*ii)++] = base + 2;
            idxs[(*ii)++] = base + 3;
        }
    }
}

void arpt_prepare_texture(const arpt_surface_data *surface,
                          const arpt_highway_data *highways,
                          const arpt_style *style, arpt_texture_prim *out) {
    memset(out, 0, sizeof(*out));

    /* Count polygon geometry */
    size_t poly_verts = 0, poly_indices = 0;
    count_polygon_geom(surface, &poly_verts, &poly_indices);

    /* Count highway geometry */
    size_t hw_verts = 0, hw_indices = 0;
    if (highways) {
        for (size_t i = 0; i < highways->count; i++) {
            size_t segs = highways->lines[i].vertex_count - 1;
            hw_verts += segs * 4;
            hw_indices += segs * 6;
        }
    }

    /* Tessellate polygons */
    if (poly_verts > 0 && poly_indices > 0) {
        out->poly_verts = malloc(poly_verts * sizeof(arpt_poly_vertex));
        out->poly_indices = malloc(poly_indices * sizeof(uint32_t));
        if (out->poly_verts && out->poly_indices) {
            size_t vi = 0, ii = 0;
            emit_polygons(surface, style, out->poly_verts, out->poly_indices,
                          &vi, &ii);
            out->poly_vert_count = vi;
            out->poly_index_count = ii;
        } else {
            free(out->poly_verts);
            free(out->poly_indices);
            out->poly_verts = NULL;
            out->poly_indices = NULL;
        }
    }

    /* Tessellate highways */
    if (hw_verts > 0 && hw_indices > 0) {
        out->line_verts = malloc(hw_verts * sizeof(arpt_line_vertex));
        out->line_indices = malloc(hw_indices * sizeof(uint32_t));
        if (out->line_verts && out->line_indices) {
            size_t vi = 0, ii = 0;
            emit_highway_sdf_quads(highways, style, out->line_verts,
                                   out->line_indices, &vi, &ii);
            out->line_vert_count = vi;
            out->line_index_count = ii;
        } else {
            free(out->line_verts);
            free(out->line_indices);
            out->line_verts = NULL;
            out->line_indices = NULL;
        }
    }
}

/* Extrusion — building wall + roof geometry */

#define DEG_TO_RAD (M_PI / 180.0)

static void encode_octahedral(double nx, double ny, double nz, int8_t *ox,
                               int8_t *oy) {
    double ax = fabs(nx), ay = fabs(ny), az = fabs(nz);
    double sum = ax + ay + az;
    if (sum < 1e-15) {
        *ox = 0;
        *oy = 127;
        return;
    }
    double u = nx / sum;
    double v = ny / sum;
    if (nz < 0.0) {
        double old_u = u, old_v = v;
        u = (1.0 - fabs(old_v)) * (old_u >= 0.0 ? 1.0 : -1.0);
        v = (1.0 - fabs(old_u)) * (old_v >= 0.0 ? 1.0 : -1.0);
    }
    double cu = u * 127.0;
    double cv = v * 127.0;
    *ox = (int8_t)(cu >= 0.0 ? cu + 0.5 : cu - 0.5);
    *oy = (int8_t)(cv >= 0.0 ? cv + 0.5 : cv - 0.5);
}

static bool building_in_tile_proper(const arpt_surface_polygon *b) {
    size_t n = b->vertex_count - 1;
    uint32_t sx = 0, sy = 0;
    for (size_t v = 0; v < n; v++) {
        sx += b->x[v];
        sy += b->y[v];
    }
    uint16_t cx = (uint16_t)(sx / n);
    uint16_t cy = (uint16_t)(sy / n);
    return cx >= ARPT_BUFFER && cx < (ARPT_BUFFER + ARPT_EXTENT) &&
           cy >= ARPT_BUFFER && cy < (ARPT_BUFFER + ARPT_EXTENT);
}

static void count_building_extrusion(const arpt_surface_data *buildings,
                                     size_t *extra_verts,
                                     size_t *extra_indices) {
    if (!buildings) return;
    for (size_t i = 0; i < buildings->count; i++) {
        const arpt_surface_polygon *b = &buildings->polygons[i];
        if (b->height_m <= 0 || b->vertex_count < 4) continue;
        if (!building_in_tile_proper(b)) continue;
        size_t n = b->vertex_count - 1;
        *extra_verts += n * 4 + n;
        *extra_indices += n * 6 + (n - 2) * 3;
    }
}

static void emit_building_extrusion(const arpt_surface_data *buildings,
                                    double east[3], double north[3],
                                    double up[3], arpt_bounds bounds,
                                    uint16_t *xy, int32_t *z, int8_t *norms,
                                    uint32_t *indices, size_t *vi,
                                    size_t *ii) {
    if (!buildings) return;

    int8_t roof_ox, roof_oy;
    encode_octahedral(up[0], up[1], up[2], &roof_ox, &roof_oy);

    for (size_t bi = 0; bi < buildings->count; bi++) {
        const arpt_surface_polygon *b = &buildings->polygons[bi];
        if (b->height_m <= 0 || b->vertex_count < 4) continue;
        if (!building_in_tile_proper(b)) continue;

        size_t n = b->vertex_count - 1;
        int32_t base_z = (b->z && b->vertex_count > 0) ? b->z[0] : 0;
        int32_t height_mm = base_z + b->height_m * 1000;

        /* Wall quads */
        for (size_t e = 0; e < n; e++) {
            size_t next = (e + 1) % n;
            uint16_t ax = b->x[e], ay = b->y[e];
            uint16_t bx = b->x[next], by = b->y[next];

            double dx = arpt_dequantize(bx) - arpt_dequantize(ax);
            double dy = arpt_dequantize(by) - arpt_dequantize(ay);
            double len = sqrt(dx * dx + dy * dy);
            if (len < 1e-12) len = 1e-12;

            double lon_span = bounds.east - bounds.west;
            double lat_span = bounds.north - bounds.south;
            double perp_e = (dy / len) * lon_span;
            double perp_n = (-dx / len) * lat_span;
            double plen = sqrt(perp_e * perp_e + perp_n * perp_n);
            if (plen < 1e-12) plen = 1e-12;
            perp_e /= plen;
            perp_n /= plen;

            double wnx = perp_e * east[0] + perp_n * north[0];
            double wny = perp_e * east[1] + perp_n * north[1];
            double wnz = perp_e * east[2] + perp_n * north[2];
            double wnlen = sqrt(wnx * wnx + wny * wny + wnz * wnz);
            if (wnlen > 1e-12) {
                wnx /= wnlen;
                wny /= wnlen;
                wnz /= wnlen;
            }
            int8_t wall_ox, wall_oy;
            encode_octahedral(wnx, wny, wnz, &wall_ox, &wall_oy);

            uint32_t base = (uint32_t)*vi;

            xy[*vi * 2] = ax; xy[*vi * 2 + 1] = ay;
            z[*vi] = base_z;
            norms[*vi * 2] = wall_ox; norms[*vi * 2 + 1] = wall_oy;
            (*vi)++;

            xy[*vi * 2] = ax; xy[*vi * 2 + 1] = ay;
            z[*vi] = height_mm;
            norms[*vi * 2] = wall_ox; norms[*vi * 2 + 1] = wall_oy;
            (*vi)++;

            xy[*vi * 2] = bx; xy[*vi * 2 + 1] = by;
            z[*vi] = height_mm;
            norms[*vi * 2] = wall_ox; norms[*vi * 2 + 1] = wall_oy;
            (*vi)++;

            xy[*vi * 2] = bx; xy[*vi * 2 + 1] = by;
            z[*vi] = base_z;
            norms[*vi * 2] = wall_ox; norms[*vi * 2 + 1] = wall_oy;
            (*vi)++;

            indices[(*ii)++] = base;
            indices[(*ii)++] = base + 1;
            indices[(*ii)++] = base + 2;
            indices[(*ii)++] = base;
            indices[(*ii)++] = base + 2;
            indices[(*ii)++] = base + 3;
        }

        /* Roof: triangle fan (CW winding) */
        uint32_t roof_base = (uint32_t)*vi;
        for (size_t v = 0; v < n; v++) {
            xy[*vi * 2] = b->x[v];
            xy[*vi * 2 + 1] = b->y[v];
            z[*vi] = height_mm;
            norms[*vi * 2] = roof_ox;
            norms[*vi * 2 + 1] = roof_oy;
            (*vi)++;
        }
        for (size_t v = 1; v + 1 < n; v++) {
            indices[(*ii)++] = roof_base;
            indices[(*ii)++] = roof_base + (uint32_t)(v + 1);
            indices[(*ii)++] = roof_base + (uint32_t)v;
        }
    }
}

void arpt_prepare_extrusion(const arpt_surface_data *buildings,
                            arpt_bounds bounds, arpt_extrusion_prim *out) {
    memset(out, 0, sizeof(*out));

    size_t nv = 0, ni = 0;
    count_building_extrusion(buildings, &nv, &ni);
    if (nv == 0 || ni == 0) return;

    out->xy = malloc(nv * 4);
    out->z = malloc(nv * sizeof(int32_t));
    out->normals = calloc(nv, 2);
    out->indices = malloc(ni * sizeof(uint32_t));
    if (!out->xy || !out->z || !out->normals || !out->indices) {
        free(out->xy);
        free(out->z);
        free(out->normals);
        free(out->indices);
        memset(out, 0, sizeof(*out));
        return;
    }

    /* Compute ENU basis at tile center */
    double clon = (bounds.west + bounds.east) * 0.5 * DEG_TO_RAD;
    double clat = (bounds.south + bounds.north) * 0.5 * DEG_TO_RAD;
    double slon = sin(clon), clon_c = cos(clon);
    double slat = sin(clat), clat_c = cos(clat);

    double east_v[3] = {-slon, clon_c, 0.0};
    double north_v[3] = {-slat * clon_c, -slat * slon, clat_c};
    double up_v[3] = {clat_c * clon_c, clat_c * slon, slat};

    size_t vi = 0, ii = 0;
    emit_building_extrusion(buildings, east_v, north_v, up_v, bounds,
                            out->xy, out->z, out->normals, out->indices,
                            &vi, &ii);

    out->vertex_count = vi;
    out->index_count = ii;
}

/* Instances — pack tree points into per-model batches */

void arpt_prepare_instances(const arpt_tree_data *trees, int model_count,
                            arpt_instance_prim *out) {
    memset(out, 0, sizeof(*out));
    if (!trees || trees->count == 0 || model_count == 0) return;

    /* Count instances per model */
    size_t counts[ARPT_MAX_MODELS] = {0};
    for (size_t i = 0; i < trees->count; i++) {
        int mi = trees->points[i].model_index;
        if (mi >= 0 && mi < model_count) counts[mi]++;
    }

    /* Count non-empty batches */
    int batch_count = 0;
    for (int mi = 0; mi < model_count; mi++) {
        if (counts[mi] > 0) batch_count++;
    }
    if (batch_count == 0) return;

    out->batches = calloc((size_t)batch_count, sizeof(arpt_instance_batch));
    if (!out->batches) return;
    out->batch_count = batch_count;

    int bi = 0;
    for (int mi = 0; mi < model_count; mi++) {
        if (counts[mi] == 0) continue;

        arpt_instance_batch *batch = &out->batches[bi++];
        batch->model_index = mi;
        batch->count = counts[mi];
        batch->instances = malloc(counts[mi] * sizeof(arpt_instance_pt));
        if (!batch->instances) { batch->count = 0; continue; }

        size_t idx = 0;
        for (size_t i = 0; i < trees->count; i++) {
            if (trees->points[i].model_index != mi) continue;

            batch->instances[idx].qx = trees->points[i].qx;
            batch->instances[idx].qy = trees->points[i].qy;
            batch->instances[idx].qz = trees->points[i].z;

            uint32_t hash = trees->points[i].id * 2654435761u;
            float yaw_01 = (float)(hash & 0xFF) / 255.0f;
            float scale_01 = (float)((hash >> 8) & 0xFF) / 255.0f;
            batch->instances[idx].yaw_scale =
                (float)((int)(yaw_01 * 256.0f)) + scale_01;
            idx++;
        }
    }
}

/* Labels — lay out POI glyph instances */

void arpt_prepare_labels(const arpt_poi_data *pois, const font_glyph *glyphs,
                         float font_height, arpt_label_prim *out) {
    memset(out, 0, sizeof(*out));
    if (!pois || pois->count == 0 || !glyphs) return;

    float font_size = font_height;
    if (font_size < 1.0f) font_size = 40.0f;

    /* Count total renderable glyphs */
    size_t total_glyphs = 0;
    for (size_t i = 0; i < pois->count; i++) {
        const char *name = pois->points[i].name;
        for (size_t c = 0; name[c]; c++) {
            int ch = (unsigned char)name[c];
            if (ch < FONT_FIRST_CHAR || ch > FONT_LAST_CHAR) continue;
            int gi = ch - FONT_FIRST_CHAR;
            if (glyphs[gi].width > 0) total_glyphs++;
        }
    }
    if (total_glyphs == 0) return;

    out->glyphs = malloc(total_glyphs * sizeof(arpt_glyph_inst));
    out->labels = malloc(pois->count * sizeof(arpt_label_meta));
    if (!out->glyphs || !out->labels) {
        free(out->glyphs);
        free(out->labels);
        memset(out, 0, sizeof(*out));
        return;
    }

    size_t idx = 0;
    int label_count = 0;

    for (size_t i = 0; i < pois->count; i++) {
        const arpt_poi_point *p = &pois->points[i];
        const char *name = p->name;
        size_t len = strlen(name);

        /* Compute total string width in pixels */
        float total_w = 0;
        float max_h = 0;
        for (size_t c = 0; c < len; c++) {
            int ch = (unsigned char)name[c];
            if (ch < FONT_FIRST_CHAR || ch > FONT_LAST_CHAR)
                ch = FONT_FIRST_CHAR;
            const font_glyph *g = &glyphs[ch - FONT_FIRST_CHAR];
            total_w += g->advance;
            if (g->height > max_h) max_h = g->height;
        }
        float half_w = total_w * 0.5f;

        uint32_t first_inst = (uint32_t)idx;

        /* Emit glyph instances */
        float cursor = 0;
        for (size_t c = 0; c < len; c++) {
            int ch = (unsigned char)name[c];
            if (ch < FONT_FIRST_CHAR || ch > FONT_LAST_CHAR)
                ch = FONT_FIRST_CHAR;
            int gi = ch - FONT_FIRST_CHAR;
            const font_glyph *g = &glyphs[gi];

            if (g->width > 0) {
                out->glyphs[idx].qx = p->qx;
                out->glyphs[idx].qy = p->qy;
                out->glyphs[idx].qz = p->z;
                out->glyphs[idx].u0 = g->u0;
                out->glyphs[idx].v0 = g->v0;
                out->glyphs[idx].u1 = g->u1;
                out->glyphs[idx].v1 = g->v1;
                out->glyphs[idx].ox =
                    (cursor + g->bearing_x - half_w) / font_size;
                out->glyphs[idx].oy = 0.3f - g->bearing_y / font_size;
                idx++;
            }
            cursor += g->advance;
        }

        uint32_t glyph_count = (uint32_t)idx - first_inst;
        if (glyph_count > 0) {
            arpt_label_meta *lm = &out->labels[label_count++];
            lm->qx = p->qx;
            lm->qy = p->qy;
            lm->qz = p->z;
            lm->w_px = total_w;
            lm->h_px = max_h;
            lm->first = first_inst;
            lm->count = glyph_count;
        }
    }

    out->glyph_count = idx;
    out->label_count = label_count;
}

/* Cleanup */

void arpt_tile_prims_free(arpt_tile_prims *p) {
    if (!p) return;
    /* terrain: zero-copy, nothing to free */

    /* texture */
    free(p->texture.poly_verts);
    free(p->texture.poly_indices);
    free(p->texture.line_verts);
    free(p->texture.line_indices);

    /* extrusion */
    free(p->extrusion.xy);
    free(p->extrusion.z);
    free(p->extrusion.normals);
    free(p->extrusion.indices);

    /* instances */
    if (p->instances.batches) {
        for (int i = 0; i < p->instances.batch_count; i++)
            free(p->instances.batches[i].instances);
        free(p->instances.batches);
    }

    /* labels */
    free(p->labels.glyphs);
    free(p->labels.labels);
}
