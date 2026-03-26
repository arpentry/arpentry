/* Feature serialization for the sort buffer.
 *
 * Format: [geom_type:1][n_coords:4][n_offsets:4][n_props:4]
 *         [x:8*n][y:8*n]
 *         [offsets:4*noff]
 *         [prop_key_len:2 + key_bytes + prop_val_len:2 + val_bytes] * n_props
 */

#include "feature_io.h"

#include <stdlib.h>
#include <string.h>

/* Fixed header size: type(1) + n_coords(4) + n_offsets(4) + n_props(4) */
#define HEADER_SIZE 13

uint8_t *arpt_feature_serialize(const arpt_geom *geom,
                                const char *const *pkeys,
                                const char *const *pvals,
                                uint32_t n_props, size_t *out_size) {
    size_t sz = HEADER_SIZE;
    sz += geom->n_coords * sizeof(double) * 2;
    uint32_t noff = geom->offsets ? geom->n_offsets : 0;
    if (noff > 0) sz += noff * sizeof(uint32_t);
    for (uint32_t i = 0; i < n_props; i++) {
        sz += 2 + strlen(pkeys[i]) + 2 + strlen(pvals[i]);
    }

    uint8_t *buf = malloc(sz);
    if (!buf) return NULL;

    size_t pos = 0;
    buf[pos++] = (uint8_t)geom->type;
    memcpy(buf + pos, &geom->n_coords, 4); pos += 4;
    memcpy(buf + pos, &noff, 4); pos += 4;
    memcpy(buf + pos, &n_props, 4); pos += 4;

    memcpy(buf + pos, geom->x, geom->n_coords * sizeof(double));
    pos += geom->n_coords * sizeof(double);
    memcpy(buf + pos, geom->y, geom->n_coords * sizeof(double));
    pos += geom->n_coords * sizeof(double);

    if (noff > 0) {
        memcpy(buf + pos, geom->offsets, noff * sizeof(uint32_t));
        pos += noff * sizeof(uint32_t);
    }

    for (uint32_t i = 0; i < n_props; i++) {
        uint16_t klen = (uint16_t)strlen(pkeys[i]);
        uint16_t vlen = (uint16_t)strlen(pvals[i]);
        memcpy(buf + pos, &klen, 2); pos += 2;
        memcpy(buf + pos, pkeys[i], klen); pos += klen;
        memcpy(buf + pos, &vlen, 2); pos += 2;
        memcpy(buf + pos, pvals[i], vlen); pos += vlen;
    }

    *out_size = pos;
    return buf;
}

bool arpt_feature_deserialize(const uint8_t *data, size_t size,
                              arpt_geom *geom, arpt_feature *feat,
                              char ***keys_out, char ***vals_out) {
    if (size < HEADER_SIZE) return false;
    size_t pos = 0;

    geom->type = data[pos++];
    memcpy(&geom->n_coords, data + pos, 4); pos += 4;
    uint32_t noff;
    memcpy(&noff, data + pos, 4); pos += 4;
    uint32_t n_props;
    memcpy(&n_props, data + pos, 4); pos += 4;

    /* Bounds-check coordinate data */
    size_t coords_size = (size_t)geom->n_coords * sizeof(double) * 2;
    if (pos + coords_size > size) return false;

    geom->x = malloc(geom->n_coords * sizeof(double));
    geom->y = malloc(geom->n_coords * sizeof(double));
    if (!geom->x || !geom->y) return false;

    memcpy(geom->x, data + pos, geom->n_coords * sizeof(double));
    pos += geom->n_coords * sizeof(double);
    memcpy(geom->y, data + pos, geom->n_coords * sizeof(double));
    pos += geom->n_coords * sizeof(double);

    if (noff > 0) {
        /* Bounds-check offset data */
        size_t off_size = (size_t)noff * sizeof(uint32_t);
        if (pos + off_size > size) return false;

        geom->offsets = malloc(off_size);
        if (!geom->offsets) return false;
        memcpy(geom->offsets, data + pos, off_size);
        pos += off_size;
        geom->n_offsets = noff;
    }

    char **keys = NULL, **vals = NULL;
    if (n_props > 0) {
        keys = malloc(n_props * sizeof(char *));
        vals = malloc(n_props * sizeof(char *));
        if (!keys || !vals) { free(keys); free(vals); return false; }
        memset(keys, 0, n_props * sizeof(char *));
        memset(vals, 0, n_props * sizeof(char *));
        for (uint32_t i = 0; i < n_props; i++) {
            /* Bounds-check key length */
            if (pos + 2 > size) goto prop_fail;
            uint16_t klen;
            memcpy(&klen, data + pos, 2); pos += 2;
            if (pos + klen > size) goto prop_fail;
            keys[i] = malloc(klen + 1);
            if (!keys[i]) goto prop_fail;
            memcpy(keys[i], data + pos, klen); keys[i][klen] = '\0'; pos += klen;

            /* Bounds-check value length */
            if (pos + 2 > size) goto prop_fail;
            uint16_t vlen;
            memcpy(&vlen, data + pos, 2); pos += 2;
            if (pos + vlen > size) goto prop_fail;
            vals[i] = malloc(vlen + 1);
            if (!vals[i]) goto prop_fail;
            memcpy(vals[i], data + pos, vlen); vals[i][vlen] = '\0'; pos += vlen;
        }
        goto prop_ok;
prop_fail:
        for (uint32_t j = 0; j < n_props; j++) {
            free(keys[j]);
            free(vals[j]);
        }
        free(keys); free(vals);
        return false;
prop_ok:;
    }

    feat->geom = geom;
    feat->prop_keys = (const char *const *)keys;
    feat->prop_vals = (const char *const *)vals;
    feat->n_props = n_props;
    *keys_out = keys;
    *vals_out = vals;
    return true;
}

void arpt_feature_deserialize_free(arpt_geom *geom, arpt_feature *feat,
                                   char **keys, char **vals) {
    if (geom) arpt_geom_free(geom);
    if (keys) {
        for (uint32_t i = 0; i < feat->n_props; i++) free(keys[i]);
        free(keys);
    }
    if (vals) {
        for (uint32_t i = 0; i < feat->n_props; i++) free(vals[i]);
        free(vals);
    }
}
