#include "internal.h"

#include <stdlib.h>

void arpt__building_upload(arpt_renderer *r, arpt_tile_gpu *t,
                           const arpt_building_prim *prim) {
    if (!prim || prim->vertex_count == 0 || prim->index_count == 0 ||
        !prim->normals)
        return;

    size_t nv = prim->vertex_count;
    size_t ni = prim->index_count;

    t->bldg_buf_xy =
        create_buffer(r->device, r->queue, WGPUBufferUsage_Vertex,
                      prim->xy, nv * 4);

    t->bldg_buf_z = create_buffer(r->device, r->queue, WGPUBufferUsage_Vertex,
                                  prim->z, nv * sizeof(int32_t));

    /* Pad normals to 4-byte stride */
    {
        int8_t *padded = pad_normals_2to4(prim->normals, NULL, NULL, nv);
        if (!padded) return;
        t->bldg_buf_normals = create_buffer(
            r->device, r->queue, WGPUBufferUsage_Vertex, padded, nv * 4);
        free(padded);
    }

    t->bldg_buf_indices = create_buffer(r->device, r->queue,
                                        WGPUBufferUsage_Index, prim->indices,
                                        ni * sizeof(uint32_t));

    /* Mark drawable only once every buffer exists, so a partial upload
       (allocation failure above) is skipped by the draw path. */
    if (t->bldg_buf_xy && t->bldg_buf_z && t->bldg_buf_normals &&
        t->bldg_buf_indices)
        t->bldg_index_count = (uint32_t)ni;
}
