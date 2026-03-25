#ifndef ARPENTRY_RESP_STYLE_H
#define ARPENTRY_RESP_STYLE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Build a Brotli-compressed Style FlatBuffer for the /style.arps response.
   Reloads the JSON file on every call so edits take effect without restart.
   Caller frees *out. */
bool resp_build_style(const char *style_file, uint8_t **out, size_t *out_size);

#endif /* ARPENTRY_RESP_STYLE_H */
