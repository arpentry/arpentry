#ifndef ARPENTRY_RESP_TILESET_H
#define ARPENTRY_RESP_TILESET_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct arpt_archive_reader;

/* Build a Brotli-compressed Tileset FlatBuffer for the /index.arpi response.
   Caller frees *out. */
bool resp_build_tileset(const struct arpt_archive_reader *archive,
                        uint8_t **out, size_t *out_size);

#endif /* ARPENTRY_RESP_TILESET_H */
