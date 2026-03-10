/* Parquet file reader — wraps carquet for read-only access. */

#include "parquet.h"
#include <carquet/carquet.h>
#include <stdlib.h>
#include <string.h>

/* ── File handle ────────────────────────────────────────────────────── */

struct arpt_parquet {
    carquet_reader_t *reader;
    /* Cached metadata value strings (owned by carquet reader). */
};

arpt_parquet *arpt_parquet_open(const char *path) {
    if (!path) return NULL;

    arpt_parquet *pq = calloc(1, sizeof(*pq));
    if (!pq) return NULL;

    carquet_error_t err = CARQUET_ERROR_INIT;
    carquet_reader_options_t opts;
    carquet_reader_options_init(&opts);
    opts.use_mmap = true;

    pq->reader = carquet_reader_open(path, &opts, &err);
    if (!pq->reader) {
        free(pq);
        return NULL;
    }
    return pq;
}

void arpt_parquet_close(arpt_parquet *pq) {
    if (!pq) return;
    carquet_reader_close(pq->reader);
    free(pq);
}

int64_t arpt_parquet_num_rows(const arpt_parquet *pq) {
    if (!pq) return 0;
    return carquet_reader_num_rows(pq->reader);
}

int32_t arpt_parquet_num_columns(const arpt_parquet *pq) {
    if (!pq) return 0;
    return carquet_reader_num_columns(pq->reader);
}

int32_t arpt_parquet_num_row_groups(const arpt_parquet *pq) {
    if (!pq) return 0;
    return carquet_reader_num_row_groups(pq->reader);
}

/* Find the schema node for the nth leaf (column) by scanning elements. */
static const carquet_schema_node_t *find_leaf_node(
    const carquet_schema_t *schema, int32_t col)
{
    int32_t n = carquet_schema_num_elements(schema);
    int32_t leaf = 0;
    for (int32_t i = 0; i < n; i++) {
        const carquet_schema_node_t *node = carquet_schema_get_element(schema, i);
        if (!node) continue;
        if (carquet_schema_node_is_leaf(node)) {
            if (leaf == col) return node;
            leaf++;
        }
    }
    return NULL;
}

const char *arpt_parquet_column_name(const arpt_parquet *pq, int32_t col) {
    if (!pq) return NULL;
    const carquet_schema_t *schema = carquet_reader_schema(pq->reader);
    if (col < 0 || col >= carquet_schema_num_columns(schema)) return NULL;
    const carquet_schema_node_t *node = find_leaf_node(schema, col);
    if (!node) return NULL;
    return carquet_schema_node_name(node);
}

arpt_parquet_type arpt_parquet_column_type(const arpt_parquet *pq, int32_t col) {
    if (!pq) return ARPT_PARQUET_INT32;
    const carquet_schema_t *schema = carquet_reader_schema(pq->reader);
    if (col < 0 || col >= carquet_schema_num_columns(schema))
        return ARPT_PARQUET_INT32;
    const carquet_schema_node_t *node = find_leaf_node(schema, col);
    if (!node) return ARPT_PARQUET_INT32;
    carquet_physical_type_t pt = carquet_schema_node_physical_type(node);
    switch (pt) {
        case CARQUET_PHYSICAL_BOOLEAN:    return ARPT_PARQUET_BOOLEAN;
        case CARQUET_PHYSICAL_INT32:      return ARPT_PARQUET_INT32;
        case CARQUET_PHYSICAL_INT64:      return ARPT_PARQUET_INT64;
        case CARQUET_PHYSICAL_FLOAT:      return ARPT_PARQUET_FLOAT;
        case CARQUET_PHYSICAL_DOUBLE:     return ARPT_PARQUET_DOUBLE;
        case CARQUET_PHYSICAL_BYTE_ARRAY: return ARPT_PARQUET_BYTES;
        case CARQUET_PHYSICAL_FIXED_LEN_BYTE_ARRAY: return ARPT_PARQUET_BYTES;
        default: return ARPT_PARQUET_BYTES;
    }
}

int32_t arpt_parquet_find_column(const arpt_parquet *pq, const char *name) {
    if (!pq || !name) return -1;
    const carquet_schema_t *schema = carquet_reader_schema(pq->reader);
    return carquet_schema_find_column(schema, name);
}

/* ── Key-Value Metadata ─────────────────────────────────────────────── */

int32_t arpt_parquet_num_key_values(const arpt_parquet *pq) {
    if (!pq) return 0;
    return carquet_reader_num_key_values(pq->reader);
}

const char *arpt_parquet_key_value(const arpt_parquet *pq, const char *key) {
    if (!pq || !key) return NULL;
    int32_t n = carquet_reader_num_key_values(pq->reader);
    for (int32_t i = 0; i < n; i++) {
        const char *k = NULL;
        const char *v = NULL;
        if (carquet_reader_key_value(pq->reader, i, &k, &v) == CARQUET_OK) {
            if (k && strcmp(k, key) == 0) return v;
        }
    }
    return NULL;
}

/* ── Dot-Path Column Finder ────────────────────────────────────────── */

int32_t arpt_parquet_find_column_path(const arpt_parquet *pq, const char *dotpath)
{
    if (!pq || !dotpath) return -1;
    /* carquet_schema_find_column supports dot-separated paths for nested schemas */
    const carquet_schema_t *schema = carquet_reader_schema(pq->reader);
    return carquet_schema_find_column(schema, dotpath);
}

/* ── Cursor ─────────────────────────────────────────────────────────── */

struct arpt_parquet_cursor {
    carquet_batch_reader_t *batch_reader;
    carquet_row_batch_t    *batch;
};

arpt_parquet_cursor *arpt_parquet_cursor_create(
    arpt_parquet *pq,
    const int32_t *columns, int32_t num_columns,
    int32_t batch_size)
{
    if (!pq) return NULL;

    arpt_parquet_cursor *cur = calloc(1, sizeof(*cur));
    if (!cur) return NULL;

    carquet_batch_reader_config_t config;
    carquet_batch_reader_config_init(&config);
    if (batch_size > 0) config.batch_size = batch_size;
    if (columns && num_columns > 0) {
        config.column_indices = columns;
        config.num_columns = num_columns;
    }

    carquet_error_t err = CARQUET_ERROR_INIT;
    cur->batch_reader = carquet_batch_reader_create(pq->reader, &config, &err);
    if (!cur->batch_reader) {
        free(cur);
        return NULL;
    }
    return cur;
}

bool arpt_parquet_cursor_next(arpt_parquet_cursor *cur) {
    if (!cur) return false;
    if (cur->batch) {
        carquet_row_batch_free(cur->batch);
        cur->batch = NULL;
    }
    carquet_status_t st = carquet_batch_reader_next(cur->batch_reader, &cur->batch);
    return st == CARQUET_OK && cur->batch != NULL;
}

int64_t arpt_parquet_cursor_num_rows(const arpt_parquet_cursor *cur) {
    if (!cur || !cur->batch) return 0;
    return carquet_row_batch_num_rows(cur->batch);
}

const void *arpt_parquet_cursor_data(const arpt_parquet_cursor *cur, int32_t col) {
    if (!cur || !cur->batch) return NULL;
    const void *data = NULL;
    const uint8_t *nulls = NULL;
    int64_t count = 0;
    carquet_status_t st = carquet_row_batch_column(cur->batch, col, &data, &nulls, &count);
    if (st != CARQUET_OK) return NULL;
    return data;
}

const uint8_t *arpt_parquet_cursor_nulls(const arpt_parquet_cursor *cur, int32_t col) {
    if (!cur || !cur->batch) return NULL;
    const void *data = NULL;
    const uint8_t *nulls = NULL;
    int64_t count = 0;
    carquet_status_t st = carquet_row_batch_column(cur->batch, col, &data, &nulls, &count);
    if (st != CARQUET_OK) return NULL;
    return nulls;
}

void arpt_parquet_cursor_free(arpt_parquet_cursor *cur) {
    if (!cur) return;
    if (cur->batch) carquet_row_batch_free(cur->batch);
    carquet_batch_reader_free(cur->batch_reader);
    free(cur);
}
