#ifndef ARPENTRY_RESP_MODEL_H
#define ARPENTRY_RESP_MODEL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Build a Brotli-compressed ModelLibrary FlatBuffer for the /models.arpm
   response.  Contains procedurally generated tree models (oak, pine, birch).
   Caller frees *out. */
bool resp_build_models(uint8_t **out, size_t *out_size);

#endif /* ARPENTRY_RESP_MODEL_H */
