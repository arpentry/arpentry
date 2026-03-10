/* GeoParquet metadata parser. */

#ifndef ARPT_GEOPARQUET_H
#define ARPT_GEOPARQUET_H

#include <stdbool.h>
#include <stddef.h>

/* Parsed GeoParquet "geo" metadata. */
typedef struct {
    char primary_column[64];  /* geometry column name, default "geometry" */
    char encoding[16];        /* "WKB" */
    double bbox[4];           /* [xmin, ymin, xmax, ymax] or NAN */
    bool has_bbox;
} arpt_geoparquet_meta;

/* Parse the "geo" JSON key-value from file metadata.
   Returns true on success, false on parse error. */
bool arpt_geoparquet_parse(const char *json, arpt_geoparquet_meta *out);

#endif
