/* Parquet file reader. */

#ifndef ARPT_PARQUET_H
#define ARPT_PARQUET_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Opaque parquet file handle. */
typedef struct arpt_parquet arpt_parquet;

/* Opaque row cursor for iterating batches. */
typedef struct arpt_parquet_cursor arpt_parquet_cursor;

/* Column physical types (mirrors Parquet spec). */
typedef enum {
    ARPT_PARQUET_BOOLEAN = 0,
    ARPT_PARQUET_INT32   = 1,
    ARPT_PARQUET_INT64   = 2,
    ARPT_PARQUET_FLOAT   = 4,
    ARPT_PARQUET_DOUBLE  = 5,
    ARPT_PARQUET_BYTES   = 6,   /* variable-length byte array */
} arpt_parquet_type;

/* Variable-length byte array (for BYTES columns). */
typedef struct {
    uint8_t *data;
    int32_t  length;
} arpt_parquet_bytes;

/* Open a parquet file for reading. Returns NULL on error. */
arpt_parquet *arpt_parquet_open(const char *path);

/* Close a parquet file and free resources. */
void arpt_parquet_close(arpt_parquet *pq);

/* Number of rows across all row groups. */
int64_t arpt_parquet_num_rows(const arpt_parquet *pq);

/* Number of leaf columns. */
int32_t arpt_parquet_num_columns(const arpt_parquet *pq);

/* Number of row groups. */
int32_t arpt_parquet_num_row_groups(const arpt_parquet *pq);

/* Column name by index. Returns NULL if out of range. */
const char *arpt_parquet_column_name(const arpt_parquet *pq, int32_t col);

/* Column type by index. */
arpt_parquet_type arpt_parquet_column_type(const arpt_parquet *pq, int32_t col);

/* Find column index by name. Returns -1 if not found. */
int32_t arpt_parquet_find_column(const arpt_parquet *pq, const char *name);

/* Create a cursor that reads selected columns in batches.
   columns: array of column indices, num_columns: length.
   If columns is NULL, reads all columns.
   batch_size: number of rows per batch (0 for default 65536).
   Returns NULL on error. */
arpt_parquet_cursor *arpt_parquet_cursor_create(
    arpt_parquet *pq,
    const int32_t *columns, int32_t num_columns,
    int32_t batch_size);

/* Advance to the next batch. Returns true if a batch is available. */
bool arpt_parquet_cursor_next(arpt_parquet_cursor *cur);

/* Number of rows in the current batch. */
int64_t arpt_parquet_cursor_num_rows(const arpt_parquet_cursor *cur);

/* Get column data from the current batch.
   col: index within the projected column list (not the file column index).
   Returns pointer to typed array (cast according to column type).
   For BYTES columns, returns arpt_parquet_bytes*. */
const void *arpt_parquet_cursor_data(const arpt_parquet_cursor *cur, int32_t col);

/* Get null bitmap for a column in the current batch.
   Bit i is set if value i is NOT null. Returns NULL if column is non-nullable. */
const uint8_t *arpt_parquet_cursor_nulls(const arpt_parquet_cursor *cur, int32_t col);

/* Free a cursor and its resources. */
void arpt_parquet_cursor_free(arpt_parquet_cursor *cur);

#endif
