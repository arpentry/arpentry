/* OvertureMaps GeoParquet feature reader. */

#ifndef ARPT_OVERTURE_H
#define ARPT_OVERTURE_H

#include "geom.h"
#include "parquet.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Opaque overture reader handle. */
typedef struct arpt_overture arpt_overture;

/* A single OvertureMaps feature. */
typedef struct {
    const uint8_t *wkb;     /* Raw WKB bytes (valid until next call) */
    size_t wkb_len;
    const char *id;         /* Feature ID (NULL if missing) */
    const char *type;       /* Feature type (NULL if missing) */
    const char *subtype;    /* Feature subtype (NULL if missing) */
    double bbox[4];         /* [xmin, ymin, xmax, ymax] from bbox columns */
    bool has_bbox;
    int32_t min_zoom;       /* cartography.min_zoom (-1 if missing) */
    int32_t max_zoom;       /* cartography.max_zoom (-1 if missing) */
    int32_t sort_key;       /* cartography.sort_key (0 if missing) */
} arpt_overture_feature;

/* Open an OvertureMaps GeoParquet file.
   Reads "geo" metadata, discovers geometry/id/type/subtype/bbox columns.
   Returns NULL on error. */
arpt_overture *arpt_overture_open(const char *path);

/* Read the next feature. Returns true if a feature was read.
   The geometry in `out` is owned by the caller (call arpt_geom_free).
   String pointers (id, type, subtype) are valid until next call. */
bool arpt_overture_next(arpt_overture *ov, arpt_overture_feature *out);

/* Close the reader and free resources. */
void arpt_overture_close(arpt_overture *ov);

#endif
