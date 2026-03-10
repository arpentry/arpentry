/* GeoParquet metadata parser — extracts "geo" JSON metadata. */

#include "geoparquet.h"
#include <json.h>
#include <string.h>
#include <math.h>

bool arpt_geoparquet_parse(const char *json_str, arpt_geoparquet_meta *out)
{
    if (!json_str || !out) return false;

    memset(out, 0, sizeof(*out));
    out->bbox[0] = out->bbox[1] = out->bbox[2] = out->bbox[3] = NAN;

    /* Extract primary_column (default: "geometry") */
    struct json pc = json_get(json_str, "primary_column");
    if (json_type(pc) == JSON_STRING) {
        json_string_copy(pc, out->primary_column, sizeof(out->primary_column));
    } else {
        strcpy(out->primary_column, "geometry");
    }

    /* Walk into columns.<primary_column> */
    struct json root = json_parse(json_str);
    struct json columns = json_object_get(root, "columns");
    if (json_type(columns) != JSON_OBJECT) {
        return true;  /* No columns metadata, but still valid */
    }

    struct json col = json_object_get(columns, out->primary_column);
    if (json_type(col) != JSON_OBJECT) {
        return true;
    }

    /* encoding */
    struct json enc = json_object_get(col, "encoding");
    if (json_type(enc) == JSON_STRING) {
        json_string_copy(enc, out->encoding, sizeof(out->encoding));
    } else {
        strcpy(out->encoding, "WKB");
    }

    /* bbox array */
    struct json bbox = json_object_get(col, "bbox");
    if (json_type(bbox) == JSON_ARRAY) {
        struct json v0 = json_array_get(bbox, 0);
        struct json v1 = json_array_get(bbox, 1);
        struct json v2 = json_array_get(bbox, 2);
        struct json v3 = json_array_get(bbox, 3);

        if (json_type(v0) == JSON_NUMBER && json_type(v1) == JSON_NUMBER &&
            json_type(v2) == JSON_NUMBER && json_type(v3) == JSON_NUMBER) {
            out->bbox[0] = json_double(v0);
            out->bbox[1] = json_double(v1);
            out->bbox[2] = json_double(v2);
            out->bbox[3] = json_double(v3);
            out->has_bbox = true;
        }
    }

    return true;
}
