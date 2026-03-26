/* WKB geometry parser. */

#ifndef ARPT_WKB_H
#define ARPT_WKB_H

#include "geom.h"

#include <stdbool.h>
#include <stddef.h>

/* Parse a WKB blob into an arpt_geom. Returns false on error. */
bool arpt_wkb_parse(const uint8_t *data, size_t size, arpt_geom *out);

#endif
