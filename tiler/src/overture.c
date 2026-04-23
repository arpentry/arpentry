/* OvertureMaps GeoParquet feature reader. */

#include "overture.h"
#include "geoparquet.h"
#include "wkb.h"
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
    int32_t col_class;      /* projected index for class column */
    int32_t col_bbox_xmin;
    int32_t col_bbox_ymin;
    int32_t col_bbox_xmax;
    int32_t col_bbox_ymax;
    bool    bbox_is_double; /* true if bbox columns are DOUBLE, false if FLOAT */

    int32_t col_cart_min_zoom;  /* cartography.min_zoom */
    int32_t col_cart_max_zoom;  /* cartography.max_zoom */
    int32_t col_cart_sort_key;  /* cartography.sort_key */
    int32_t col_depth;          /* depth (meters) */

    int64_t row_in_batch;   /* current row within current batch */
    int64_t batch_rows;     /* rows in current batch */

    /* Owned null-terminated copies of string columns (freed each iteration) */
    char *owned_id;
    char *owned_type;
    char *owned_subtype;
    char *owned_cls;
};

/* Find a flat string leaf column by name.
 * Rejects columns that are not BYTE_ARRAY or that live inside a repeated
 * group (LIST/MAP), which would have multiple values per row and cannot
 * be indexed by a simple row offset. */
static int32_t find_string_column(const arpt_parquet *pq, const char *name)
{
    int32_t col = arpt_parquet_find_column(pq, name);
    if (col < 0) return -1;
    if (arpt_parquet_column_type(pq, col) != ARPT_PARQUET_BYTES) return -1;
    if (arpt_parquet_column_is_repeated(pq, col)) return -1;
    return col;
}

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

    /* Discover columns in file.
     * Overture schemas vary by theme:
     *   - transportation has "type"+"subtype"+"class" (e.g. segment/road/motorway)
     *   - base/land has "subtype"+"class" with no "type" column
     * When "type" is absent, promote "subtype" → type, "class" → subtype
     * so the downstream pipeline always gets the broad category in ->type
     * and the detail in ->subtype.
     * When all three exist, ->cls carries the finest classification. */
    int32_t file_col_geom = arpt_parquet_find_column(pq, geom_col_name);
    int32_t file_col_id = find_string_column(pq, "id");
    int32_t file_col_type = find_string_column(pq, "type");
    int32_t file_col_subtype = find_string_column(pq, "subtype");
    int32_t file_col_class = -1;
    if (file_col_type < 0 && file_col_subtype >= 0) {
        /* No "type" column — promote "subtype" → type, "class" → subtype */
        file_col_type = file_col_subtype;
        file_col_subtype = find_string_column(pq, "class");
    } else {
        /* All three may exist (e.g. transportation/segment) */
        file_col_class = find_string_column(pq, "class");
    }
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

    /* Discover cartography columns */
    int32_t file_col_cart_min_zoom = arpt_parquet_find_column_path(pq, "cartography.min_zoom");
    int32_t file_col_cart_max_zoom = arpt_parquet_find_column_path(pq, "cartography.max_zoom");
    int32_t file_col_cart_sort_key = arpt_parquet_find_column_path(pq, "cartography.sort_key");

    /* Discover depth column (bathymetry) */
    int32_t file_col_depth = arpt_parquet_find_column(pq, "depth");

    /* Print discovered columns for diagnostics */
    fprintf(stderr, "  Columns: geom=%d id=%d type=%d subtype=%d class=%d\n",
            file_col_geom, file_col_id, file_col_type, file_col_subtype,
            file_col_class);
    fprintf(stderr, "  Columns: bbox=%d/%d/%d/%d cart=%d/%d/%d depth=%d\n",
            file_col_bbox_xmin, file_col_bbox_ymin,
            file_col_bbox_xmax, file_col_bbox_ymax,
            file_col_cart_min_zoom, file_col_cart_max_zoom,
            file_col_cart_sort_key, file_col_depth);
    fprintf(stderr, "  Total leaf columns in file: %d\n",
            arpt_parquet_num_columns(pq));

    /* Build projection list */
    int32_t proj_cols[14];
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

    if (file_col_class >= 0) {
        ov->col_class = n_proj;
        proj_cols[n_proj++] = file_col_class;
    } else {
        ov->col_class = -1;
    }

    if (file_col_bbox_xmin >= 0 && file_col_bbox_ymin >= 0 &&
        file_col_bbox_xmax >= 0 && file_col_bbox_ymax >= 0) {
        ov->col_bbox_xmin = n_proj; proj_cols[n_proj++] = file_col_bbox_xmin;
        ov->col_bbox_ymin = n_proj; proj_cols[n_proj++] = file_col_bbox_ymin;
        ov->col_bbox_xmax = n_proj; proj_cols[n_proj++] = file_col_bbox_xmax;
        ov->col_bbox_ymax = n_proj; proj_cols[n_proj++] = file_col_bbox_ymax;
        ov->bbox_is_double =
            arpt_parquet_column_type(pq, file_col_bbox_xmin) == ARPT_PARQUET_DOUBLE;
    } else {
        ov->col_bbox_xmin = ov->col_bbox_ymin = -1;
        ov->col_bbox_xmax = ov->col_bbox_ymax = -1;
    }

    if (file_col_cart_min_zoom >= 0) {
        ov->col_cart_min_zoom = n_proj;
        proj_cols[n_proj++] = file_col_cart_min_zoom;
    } else {
        ov->col_cart_min_zoom = -1;
    }
    if (file_col_cart_max_zoom >= 0) {
        ov->col_cart_max_zoom = n_proj;
        proj_cols[n_proj++] = file_col_cart_max_zoom;
    } else {
        ov->col_cart_max_zoom = -1;
    }
    if (file_col_cart_sort_key >= 0) {
        ov->col_cart_sort_key = n_proj;
        proj_cols[n_proj++] = file_col_cart_sort_key;
    } else {
        ov->col_cart_sort_key = -1;
    }
    if (file_col_depth >= 0) {
        ov->col_depth = n_proj;
        proj_cols[n_proj++] = file_col_depth;
    } else {
        ov->col_depth = -1;
    }

    /* Create cursor with projection */
    fprintf(stderr, "  Projecting %d columns, creating cursor...\n", n_proj);
    ov->cursor = arpt_parquet_cursor_create(pq, proj_cols, n_proj, 0);
    if (!ov->cursor) {
        fprintf(stderr, "  ERROR: cursor creation failed\n");
        arpt_parquet_close(pq);
        free(ov);
        return NULL;
    }
    fprintf(stderr, "  Cursor created, %lld total rows\n",
            (long long)arpt_parquet_num_rows(pq));

    ov->row_in_batch = 0;
    ov->batch_rows = 0;

    return ov;
}

/* Count set bits in a null bitmap up to (but not including) position pos.
 * In carquet's batch reader, set bits mark NULL values and the data array
 * is packed (only non-null values). The sparse index of a non-null row is
 * its row number minus the number of nulls (set bits) before it.
 *
 * Uses popcount on full bytes for O(pos/8) instead of O(pos). */
static int64_t count_nulls_before(const uint8_t *nulls, int64_t pos)
{
    int64_t count = 0;
    int64_t full_bytes = pos / 8;
    for (int64_t i = 0; i < full_bytes; i++) {
        count += __builtin_popcount(nulls[i]);
    }
    /* Count remaining bits in the partial byte */
    int rem = (int)(pos % 8);
    if (rem > 0) {
        uint8_t mask = (uint8_t)((1u << rem) - 1);
        count += __builtin_popcount(nulls[full_bytes] & mask);
    }
    return count;
}

/* Read a BYTE_ARRAY string at row index as a null-terminated C string.
 * Returns a malloc'd copy that the caller must free. */
static char *read_string(arpt_overture *ov, int32_t proj_col, int64_t row)
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

    /* Treat pandas' stringified null ("nan") as NULL.  Some input parquet
       files — notably Natural Earth exports via pandas — serialise missing
       string values as the literal text "nan" instead of a real SQL null. */
    if (ba->length == 3 && memcmp(ba->data, "nan", 3) == 0) return NULL;

    /* Parquet BYTE_ARRAY values are NOT null-terminated; make a copy */
    return strndup((const char *)ba->data, (size_t)ba->length);
}

/* Check if row is null in a column's null bitmap. Returns true if null. */
static bool is_null(arpt_overture *ov, int32_t proj_col, int64_t row)
{
    if (proj_col < 0) return true;
    const uint8_t *nulls = arpt_parquet_cursor_nulls(ov->cursor, proj_col);
    if (!nulls) return false;  /* no nulls bitmap = all non-null */
    return (nulls[row / 8] & (1u << (row % 8))) != 0;
}

/* Compute the data array index for a non-null row.
 * In carquet's batch reader, set bits in the null bitmap mark NULL values
 * and the data array is packed (only non-null values). */
static int64_t sparse_index(arpt_overture *ov, int32_t proj_col, int64_t row)
{
    const uint8_t *nulls = arpt_parquet_cursor_nulls(ov->cursor, proj_col);
    if (!nulls) return row;  /* no nulls → dense array */
    return row - count_nulls_before(nulls, row);
}

/* Read a nullable INT32 column at the given row.
 * Returns default_val if the column is absent or the value is null. */
static int32_t read_int32(arpt_overture *ov, int32_t proj_col, int64_t row,
                           int32_t default_val) {
    if (proj_col < 0) return default_val;
    if (is_null(ov, proj_col, row)) return default_val;
    const int32_t *arr =
        (const int32_t *)arpt_parquet_cursor_data(ov->cursor, proj_col);
    if (!arr) return default_val;
    int64_t idx = sparse_index(ov, proj_col, row);
    return arr[idx];
}

bool arpt_overture_next(arpt_overture *ov, arpt_overture_feature *out)
{
    if (!ov || !out) return false;

    for (;;) {
        /* Free previous iteration's owned strings */
        free(ov->owned_id);      ov->owned_id = NULL;
        free(ov->owned_type);    ov->owned_type = NULL;
        free(ov->owned_subtype); ov->owned_subtype = NULL;
        free(ov->owned_cls);     ov->owned_cls = NULL;

        memset(out, 0, sizeof(*out));

        /* Advance to next row, fetching new batch if needed */
        while (ov->row_in_batch >= ov->batch_rows) {
            if (!arpt_parquet_cursor_next(ov->cursor))
                return false;
            ov->batch_rows = arpt_parquet_cursor_num_rows(ov->cursor);
            if (ov->row_in_batch == 0 && ov->batch_rows > 0)
                fprintf(stderr, "  First batch: %lld rows\n",
                        (long long)ov->batch_rows);
            ov->row_in_batch = 0;
        }

        int64_t row = ov->row_in_batch;
        ov->row_in_batch++;

        /* Bbox columns — check null bitmap and use sparse index because
         * the bbox group may be OPTIONAL, making the data array packed. */
        if (ov->col_bbox_xmin >= 0 &&
            !is_null(ov, ov->col_bbox_xmin, row)) {
            const void *xmin = arpt_parquet_cursor_data(ov->cursor, ov->col_bbox_xmin);
            const void *ymin = arpt_parquet_cursor_data(ov->cursor, ov->col_bbox_ymin);
            const void *xmax = arpt_parquet_cursor_data(ov->cursor, ov->col_bbox_xmax);
            const void *ymax = arpt_parquet_cursor_data(ov->cursor, ov->col_bbox_ymax);
            if (xmin && ymin && xmax && ymax) {
                int64_t bi = sparse_index(ov, ov->col_bbox_xmin, row);
                if (ov->bbox_is_double) {
                    out->bbox[0] = ((const double *)xmin)[bi];
                    out->bbox[1] = ((const double *)ymin)[bi];
                    out->bbox[2] = ((const double *)xmax)[bi];
                    out->bbox[3] = ((const double *)ymax)[bi];
                } else {
                    out->bbox[0] = (double)((const float *)xmin)[bi];
                    out->bbox[1] = (double)((const float *)ymin)[bi];
                    out->bbox[2] = (double)((const float *)xmax)[bi];
                    out->bbox[3] = (double)((const float *)ymax)[bi];
                }
                out->has_bbox = true;
            }
        }

        /* Skip rows with null geometry */
        if (is_null(ov, ov->col_geometry, row))
            continue;

        /* Get raw WKB bytes — defer parsing to worker threads */
        const arpt_parquet_bytes *geom_arr =
            (const arpt_parquet_bytes *)arpt_parquet_cursor_data(
                ov->cursor, ov->col_geometry);
        if (!geom_arr) continue;

        int64_t gi = sparse_index(ov, ov->col_geometry, row);
        const arpt_parquet_bytes *wkb = &geom_arr[gi];
        if (!wkb->data || wkb->length <= 0)
            continue;

        out->wkb = wkb->data;
        out->wkb_len = (size_t)wkb->length;

        /* String columns — read_string returns owned null-terminated copies */
        ov->owned_id = read_string(ov, ov->col_id, row);
        ov->owned_type = read_string(ov, ov->col_type, row);
        ov->owned_subtype = read_string(ov, ov->col_subtype, row);
        ov->owned_cls = read_string(ov, ov->col_class, row);
        out->id = ov->owned_id;
        out->type = ov->owned_type;
        out->subtype = ov->owned_subtype;
        out->cls = ov->owned_cls;

        /* Cartography fields */
        out->min_zoom = read_int32(ov, ov->col_cart_min_zoom, row, -1);
        out->max_zoom = read_int32(ov, ov->col_cart_max_zoom, row, -1);
        out->sort_key = read_int32(ov, ov->col_cart_sort_key, row, 0);

        /* Depth (bathymetry) */
        out->depth = read_int32(ov, ov->col_depth, row, -1);

        return true;
    }
}

void arpt_overture_close(arpt_overture *ov)
{
    if (!ov) return;
    free(ov->owned_id);
    free(ov->owned_type);
    free(ov->owned_subtype);
    free(ov->owned_cls);
    arpt_parquet_cursor_free(ov->cursor);
    arpt_parquet_close(ov->pq);
    free(ov);
}
