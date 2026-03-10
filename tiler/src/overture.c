/* OvertureMaps GeoParquet feature reader. */

#include "overture.h"
#include "geoparquet.h"
#include <carquet/carquet.h>
#include <stdlib.h>
#include <string.h>

/* Column indices discovered from schema. -1 = not found. */
struct arpt_overture {
    arpt_parquet *pq;
    arpt_parquet_cursor *cursor;

    int32_t col_geometry;   /* projected index for geometry column */
    int32_t col_id;         /* projected index for id column */
    int32_t col_type;       /* projected index for type column */
    int32_t col_subtype;    /* projected index for subtype column */
    int32_t col_bbox_xmin;
    int32_t col_bbox_ymin;
    int32_t col_bbox_xmax;
    int32_t col_bbox_ymax;

    int64_t row_in_batch;   /* current row within current batch */
    int64_t batch_rows;     /* rows in current batch */
};

arpt_overture *arpt_overture_open(const char *path)
{
    if (!path) return NULL;

    arpt_parquet *pq = arpt_parquet_open(path);
    if (!pq) return NULL;

    arpt_overture *ov = calloc(1, sizeof(*ov));
    if (!ov) {
        arpt_parquet_close(pq);
        return NULL;
    }
    ov->pq = pq;

    /* Read GeoParquet "geo" metadata to find geometry column name */
    const char *geo_json = arpt_parquet_key_value(pq, "geo");
    char geom_col_name[64] = "geometry";
    if (geo_json) {
        arpt_geoparquet_meta meta;
        if (arpt_geoparquet_parse(geo_json, &meta)) {
            strncpy(geom_col_name, meta.primary_column, sizeof(geom_col_name) - 1);
            geom_col_name[sizeof(geom_col_name) - 1] = '\0';
        }
    }

    /* Discover columns in file */
    int32_t file_col_geom = arpt_parquet_find_column(pq, geom_col_name);
    int32_t file_col_id = arpt_parquet_find_column(pq, "id");
    int32_t file_col_type = arpt_parquet_find_column(pq, "type");
    int32_t file_col_subtype = arpt_parquet_find_column(pq, "subtype");
    int32_t file_col_bbox_xmin = arpt_parquet_find_column_path(pq, "bbox.xmin");
    int32_t file_col_bbox_ymin = arpt_parquet_find_column_path(pq, "bbox.ymin");
    int32_t file_col_bbox_xmax = arpt_parquet_find_column_path(pq, "bbox.xmax");
    int32_t file_col_bbox_ymax = arpt_parquet_find_column_path(pq, "bbox.ymax");

    /* Must have geometry column */
    if (file_col_geom < 0) {
        arpt_parquet_close(pq);
        free(ov);
        return NULL;
    }

    /* Build projection list */
    int32_t proj_cols[8];
    int32_t n_proj = 0;

    ov->col_geometry = n_proj;
    proj_cols[n_proj++] = file_col_geom;

    if (file_col_id >= 0) {
        ov->col_id = n_proj;
        proj_cols[n_proj++] = file_col_id;
    } else {
        ov->col_id = -1;
    }

    if (file_col_type >= 0) {
        ov->col_type = n_proj;
        proj_cols[n_proj++] = file_col_type;
    } else {
        ov->col_type = -1;
    }

    if (file_col_subtype >= 0) {
        ov->col_subtype = n_proj;
        proj_cols[n_proj++] = file_col_subtype;
    } else {
        ov->col_subtype = -1;
    }

    if (file_col_bbox_xmin >= 0 && file_col_bbox_ymin >= 0 &&
        file_col_bbox_xmax >= 0 && file_col_bbox_ymax >= 0) {
        ov->col_bbox_xmin = n_proj; proj_cols[n_proj++] = file_col_bbox_xmin;
        ov->col_bbox_ymin = n_proj; proj_cols[n_proj++] = file_col_bbox_ymin;
        ov->col_bbox_xmax = n_proj; proj_cols[n_proj++] = file_col_bbox_xmax;
        ov->col_bbox_ymax = n_proj; proj_cols[n_proj++] = file_col_bbox_ymax;
    } else {
        ov->col_bbox_xmin = ov->col_bbox_ymin = -1;
        ov->col_bbox_xmax = ov->col_bbox_ymax = -1;
    }

    /* Create cursor with projection */
    ov->cursor = arpt_parquet_cursor_create(pq, proj_cols, n_proj, 0);
    if (!ov->cursor) {
        arpt_parquet_close(pq);
        free(ov);
        return NULL;
    }

    ov->row_in_batch = 0;
    ov->batch_rows = 0;

    return ov;
}

/* Count set bits in a null bitmap up to (but not including) position pos.
 * In carquet's batch reader, set bits mark NULL values and the data array
 * is packed (only non-null values). The sparse index of a non-null row is
 * its row number minus the number of nulls (set bits) before it. */
static int64_t count_nulls_before(const uint8_t *nulls, int64_t pos)
{
    int64_t count = 0;
    for (int64_t i = 0; i < pos; i++) {
        if (nulls[i / 8] & (1u << (i % 8)))
            count++;
    }
    return count;
}

/* Read a BYTE_ARRAY string at row index as a C string pointer.
 * The pointer is valid until the next batch advance. */
static const char *read_string(arpt_overture *ov, int32_t proj_col, int64_t row)
{
    if (proj_col < 0) return NULL;
    const arpt_parquet_bytes *arr =
        (const arpt_parquet_bytes *)arpt_parquet_cursor_data(ov->cursor, proj_col);
    if (!arr) return NULL;

    /* Check null bitmap — in carquet's batch reader, a set bit means NULL
     * and the data array is sparse (packed non-null values only). */
    const uint8_t *nulls = arpt_parquet_cursor_nulls(ov->cursor, proj_col);
    if (nulls) {
        uint8_t bit = nulls[row / 8] & (1u << (row % 8));
        if (bit) return NULL;  /* bit set = value is null */
    }

    /* Compute sparse data index: row minus nulls before it */
    int64_t data_idx = nulls ? row - count_nulls_before(nulls, row) : row;

    const arpt_parquet_bytes *ba = &arr[data_idx];
    if (!ba->data || ba->length <= 0) return NULL;

    return (const char *)ba->data;
}

bool arpt_overture_next(arpt_overture *ov, arpt_overture_feature *out)
{
    if (!ov || !out) return false;

    memset(out, 0, sizeof(*out));

    /* Advance to next row, fetching new batch if needed */
    while (ov->row_in_batch >= ov->batch_rows) {
        if (!arpt_parquet_cursor_next(ov->cursor))
            return false;
        ov->batch_rows = arpt_parquet_cursor_num_rows(ov->cursor);
        ov->row_in_batch = 0;
    }

    int64_t row = ov->row_in_batch;

    /* Parse WKB geometry */
    const arpt_parquet_bytes *geom_arr =
        (const arpt_parquet_bytes *)arpt_parquet_cursor_data(ov->cursor, ov->col_geometry);
    if (!geom_arr) return false;

    const arpt_parquet_bytes *wkb = &geom_arr[row];
    if (!wkb->data || wkb->length <= 0) {
        ov->row_in_batch++;
        return false;
    }

    if (!arpt_wkb_parse(wkb->data, (size_t)wkb->length, &out->geometry)) {
        ov->row_in_batch++;
        return false;
    }

    /* String columns */
    out->id = read_string(ov, ov->col_id, row);
    out->type = read_string(ov, ov->col_type, row);
    out->subtype = read_string(ov, ov->col_subtype, row);

    /* Bbox columns — Overture uses FLOAT (32-bit) for bbox fields */
    if (ov->col_bbox_xmin >= 0) {
        const float *xmin = arpt_parquet_cursor_data(ov->cursor, ov->col_bbox_xmin);
        const float *ymin = arpt_parquet_cursor_data(ov->cursor, ov->col_bbox_ymin);
        const float *xmax = arpt_parquet_cursor_data(ov->cursor, ov->col_bbox_xmax);
        const float *ymax = arpt_parquet_cursor_data(ov->cursor, ov->col_bbox_ymax);
        if (xmin && ymin && xmax && ymax) {
            out->bbox[0] = (double)xmin[row];
            out->bbox[1] = (double)ymin[row];
            out->bbox[2] = (double)xmax[row];
            out->bbox[3] = (double)ymax[row];
            out->has_bbox = true;
        }
    }

    ov->row_in_batch++;
    return true;
}

void arpt_overture_close(arpt_overture *ov)
{
    if (!ov) return;
    arpt_parquet_cursor_free(ov->cursor);
    arpt_parquet_close(ov->pq);
    free(ov);
}
