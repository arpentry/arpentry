/* Feature serialization for the sort buffer. */

#ifndef ARPT_FEATURE_IO_H
#define ARPT_FEATURE_IO_H

#include "geom.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Serialize a clipped geometry and its properties into a compact binary
   record suitable for the external sorter.  Caller frees the returned
   buffer.  Returns NULL on allocation failure. */
uint8_t *arpt_feature_serialize(const arpt_geom *geom,
                                const char *const *pkeys,
                                const char *const *pvals,
                                uint32_t n_props, size_t *out_size);

/* Deserialize a binary record back into geometry + feature.  Allocates
   coordinate arrays and property strings; caller must free them with
   arpt_feature_deserialize_free().
   Returns false on truncated or corrupt data. */
bool arpt_feature_deserialize(const uint8_t *data, size_t size,
                              arpt_geom *geom, arpt_feature *feat,
                              char ***keys_out, char ***vals_out);

/* Free all memory allocated by arpt_feature_deserialize. */
void arpt_feature_deserialize_free(arpt_geom *geom, arpt_feature *feat,
                                   char **keys, char **vals);

#endif
