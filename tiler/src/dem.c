/* Minimal GeoTIFF reader for single-band int16 uncompressed DEMs.
 *
 * Supports only what ETOPO1 needs: single-band, int16, uncompressed,
 * strip layout, GeoTIFF ModelTiePoint + ModelPixelScale tags.
 */

#include "dem.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* TIFF tag IDs */
#define TAG_IMAGE_WIDTH      256
#define TAG_IMAGE_LENGTH     257
#define TAG_BITS_PER_SAMPLE  258
#define TAG_COMPRESSION      259
#define TAG_STRIP_OFFSETS    273
#define TAG_ROWS_PER_STRIP   278
#define TAG_STRIP_BYTE_COUNTS 279
#define TAG_SAMPLE_FORMAT    339
#define TAG_MODEL_PIXEL_SCALE 33550
#define TAG_MODEL_TIE_POINT   33922

/* TIFF data types */
#define TIFF_SHORT   3
#define TIFF_LONG    4
#define TIFF_DOUBLE  12

struct arpt_dem {
    int16_t *data;       /* Row-major, north-to-south */
    uint32_t width;
    uint32_t height;
    double   origin_lon; /* Longitude of pixel (0,0) */
    double   origin_lat; /* Latitude of pixel (0,0) */
    double   scale_lon;  /* Degrees per pixel (positive eastward) */
    double   scale_lat;  /* Degrees per pixel (positive southward) */
};

/* ---- Endian-aware reads ---- */

typedef struct {
    const uint8_t *buf;
    size_t         len;
    int            big;  /* 1 = big-endian (MM), 0 = little-endian (II) */
} tiff_reader;

static uint16_t rd16(const tiff_reader *r, size_t off) {
    if (off + 2 > r->len) return 0;
    const uint8_t *p = r->buf + off;
    return r->big ? (uint16_t)((p[0] << 8) | p[1])
                  : (uint16_t)(p[0] | (p[1] << 8));
}

static uint32_t rd32(const tiff_reader *r, size_t off) {
    if (off + 4 > r->len) return 0;
    const uint8_t *p = r->buf + off;
    return r->big ? ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                    ((uint32_t)p[2] << 8)  | (uint32_t)p[3]
                  : (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                    ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static double rd64f(const tiff_reader *r, size_t off) {
    if (off + 8 > r->len) return 0.0;
    uint8_t tmp[8];
    if (r->big) {
        for (int i = 0; i < 8; i++) tmp[i] = r->buf[off + 7 - i];
    } else {
        memcpy(tmp, r->buf + off, 8);
    }
    double v;
    memcpy(&v, tmp, 8);
    return v;
}

/* Read a tag value that fits in a uint32 (SHORT or LONG). */
static uint32_t tag_value_u32(const tiff_reader *r, size_t entry_off,
                              uint16_t type) {
    if (type == TIFF_SHORT) return rd16(r, entry_off + 8);
    if (type == TIFF_LONG)  return rd32(r, entry_off + 8);
    return 0;
}

/* Read an array of uint32 offsets from a tag (may be inline or pointed). */
static uint32_t *read_offset_array(const tiff_reader *r, size_t entry_off,
                                   uint16_t type, uint32_t count) {
    uint32_t *arr = malloc(count * sizeof(uint32_t));
    if (!arr) return NULL;

    size_t val_size = (type == TIFF_SHORT) ? 2 : 4;
    size_t total = count * val_size;
    size_t data_off;

    if (total <= 4) {
        data_off = entry_off + 8;
    } else {
        data_off = rd32(r, entry_off + 8);
    }

    for (uint32_t i = 0; i < count; i++) {
        if (type == TIFF_SHORT)
            arr[i] = rd16(r, data_off + i * 2);
        else
            arr[i] = rd32(r, data_off + i * 4);
    }
    return arr;
}

arpt_dem *arpt_dem_open(const char *path) {
    if (!path) return NULL;

    /* Read entire file into memory */
    FILE *f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "DEM: cannot open %s\n", path);
        return NULL;
    }

    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    if (fsize <= 0) { fclose(f); return NULL; }
    fseek(f, 0, SEEK_SET);

    uint8_t *buf = malloc((size_t)fsize);
    if (!buf) { fclose(f); return NULL; }
    if (fread(buf, 1, (size_t)fsize, f) != (size_t)fsize) {
        free(buf);
        fclose(f);
        return NULL;
    }
    fclose(f);

    tiff_reader r = { .buf = buf, .len = (size_t)fsize, .big = 0 };

    /* Parse TIFF header */
    if (r.len < 8) goto fail;

    if (buf[0] == 'I' && buf[1] == 'I') r.big = 0;
    else if (buf[0] == 'M' && buf[1] == 'M') r.big = 1;
    else { fprintf(stderr, "DEM: not a TIFF file\n"); goto fail; }

    uint16_t magic = rd16(&r, 2);
    if (magic != 42) { fprintf(stderr, "DEM: bad TIFF magic\n"); goto fail; }

    uint32_t ifd_off = rd32(&r, 4);
    if (ifd_off + 2 > r.len) goto fail;

    uint16_t n_entries = rd16(&r, ifd_off);
    if (ifd_off + 2 + (size_t)n_entries * 12 > r.len) goto fail;

    /* Scan IFD entries */
    uint32_t width = 0, height = 0;
    uint32_t bits_per_sample = 0, compression = 1;
    uint32_t rows_per_strip = 0;
    uint32_t strip_count = 0;
    uint32_t *strip_offsets = NULL;
    uint32_t *strip_byte_counts = NULL;
    double pixel_scale[3] = {0};
    double tie_point[6] = {0};
    int has_scale = 0, has_tie = 0;

    for (uint16_t i = 0; i < n_entries; i++) {
        size_t eoff = ifd_off + 2 + (size_t)i * 12;
        uint16_t tag   = rd16(&r, eoff);
        uint16_t type  = rd16(&r, eoff + 2);
        uint32_t count = rd32(&r, eoff + 4);

        switch (tag) {
        case TAG_IMAGE_WIDTH:
            width = tag_value_u32(&r, eoff, type);
            break;
        case TAG_IMAGE_LENGTH:
            height = tag_value_u32(&r, eoff, type);
            break;
        case TAG_BITS_PER_SAMPLE:
            bits_per_sample = tag_value_u32(&r, eoff, type);
            break;
        case TAG_COMPRESSION:
            compression = tag_value_u32(&r, eoff, type);
            break;
        case TAG_ROWS_PER_STRIP:
            rows_per_strip = tag_value_u32(&r, eoff, type);
            break;
        case TAG_STRIP_OFFSETS:
            strip_count = count;
            strip_offsets = read_offset_array(&r, eoff, type, count);
            break;
        case TAG_STRIP_BYTE_COUNTS:
            strip_byte_counts = read_offset_array(&r, eoff, type, count);
            break;
        case TAG_MODEL_PIXEL_SCALE:
            if (count >= 3 && type == TIFF_DOUBLE) {
                uint32_t doff = rd32(&r, eoff + 8);
                for (int k = 0; k < 3; k++)
                    pixel_scale[k] = rd64f(&r, doff + (size_t)k * 8);
                has_scale = 1;
            }
            break;
        case TAG_MODEL_TIE_POINT:
            if (count >= 6 && type == TIFF_DOUBLE) {
                uint32_t doff = rd32(&r, eoff + 8);
                for (int k = 0; k < 6; k++)
                    tie_point[k] = rd64f(&r, doff + (size_t)k * 8);
                has_tie = 1;
            }
            break;
        default:
            break;
        }
    }

    /* Validate */
    if (width == 0 || height == 0) {
        fprintf(stderr, "DEM: missing image dimensions\n");
        goto fail_strips;
    }
    if (bits_per_sample != 16) {
        fprintf(stderr, "DEM: expected 16-bit samples, got %u\n",
                bits_per_sample);
        goto fail_strips;
    }
    if (compression != 1) {
        fprintf(stderr, "DEM: only uncompressed TIFF supported "
                "(compression=%u)\n", compression);
        goto fail_strips;
    }
    if (!strip_offsets || !strip_byte_counts || strip_count == 0) {
        fprintf(stderr, "DEM: missing strip data\n");
        goto fail_strips;
    }
    if (!has_scale || !has_tie) {
        fprintf(stderr, "DEM: missing GeoTIFF transform tags\n");
        goto fail_strips;
    }
    if (rows_per_strip == 0) rows_per_strip = height;

    /* Allocate elevation grid */
    size_t n_pixels = (size_t)width * height;
    int16_t *data = malloc(n_pixels * sizeof(int16_t));
    if (!data) goto fail_strips;

    /* Read strips into the elevation grid */
    size_t row = 0;
    for (uint32_t s = 0; s < strip_count && row < height; s++) {
        uint32_t soff = strip_offsets[s];
        uint32_t slen = strip_byte_counts[s];
        uint32_t strip_rows = rows_per_strip;
        if (row + strip_rows > height) strip_rows = height - (uint32_t)row;

        size_t expected = (size_t)strip_rows * width * 2;
        if (slen < expected) {
            fprintf(stderr, "DEM: strip %u too short (%u < %zu)\n",
                    s, slen, expected);
            free(data);
            goto fail_strips;
        }
        if ((size_t)soff + expected > r.len) {
            fprintf(stderr, "DEM: strip %u exceeds file\n", s);
            free(data);
            goto fail_strips;
        }

        /* Copy and fix byte order */
        int16_t *dst = data + row * width;
        const uint8_t *src = buf + soff;
        for (size_t px = 0; px < (size_t)strip_rows * width; px++) {
            if (r.big)
                dst[px] = (int16_t)((src[px * 2] << 8) | src[px * 2 + 1]);
            else
                dst[px] = (int16_t)(src[px * 2] | (src[px * 2 + 1] << 8));
        }
        row += strip_rows;
    }

    free(strip_offsets);
    free(strip_byte_counts);
    free(buf);

    /* Build DEM struct */
    arpt_dem *dem = calloc(1, sizeof(*dem));
    if (!dem) { free(data); return NULL; }

    dem->data = data;
    dem->width = width;
    dem->height = height;

    /* GeoTIFF transform: pixel (I,J) → geo (X,Y)
       X = tie_point[3] + (I - tie_point[0]) * pixel_scale[0]
       Y = tie_point[4] - (J - tie_point[1]) * pixel_scale[1]
       Origin = pixel (0,0) → (tie_point[3], tie_point[4]) */
    dem->origin_lon = tie_point[3] - tie_point[0] * pixel_scale[0];
    dem->origin_lat = tie_point[4] + tie_point[1] * pixel_scale[1];
    dem->scale_lon  = pixel_scale[0];
    dem->scale_lat  = pixel_scale[1];

    fprintf(stderr, "DEM: loaded %s (%ux%u, %.4f°×%.4f° per pixel)\n",
            path, width, height, dem->scale_lon, dem->scale_lat);
    fprintf(stderr, "DEM: origin (%.2f, %.2f) to (%.2f, %.2f)\n",
            dem->origin_lon, dem->origin_lat,
            dem->origin_lon + (width - 1) * dem->scale_lon,
            dem->origin_lat - (height - 1) * dem->scale_lat);

    return dem;

fail_strips:
    free(strip_offsets);
    free(strip_byte_counts);
fail:
    free(buf);
    return NULL;
}

double arpt_dem_sample(const arpt_dem *dem, double lon, double lat) {
    if (!dem || !dem->data) return 0.0;

    /* Convert lon/lat to fractional pixel coordinates.
       Row 0 is at origin_lat (north), increasing row goes south. */
    double px = (lon - dem->origin_lon) / dem->scale_lon;
    double py = (dem->origin_lat - lat) / dem->scale_lat;

    /* Clamp to grid */
    if (px < 0.0) px = 0.0;
    if (py < 0.0) py = 0.0;
    if (px >= (double)(dem->width - 1))  px = (double)(dem->width - 1) - 1e-6;
    if (py >= (double)(dem->height - 1)) py = (double)(dem->height - 1) - 1e-6;

    /* Bilinear interpolation */
    uint32_t x0 = (uint32_t)px;
    uint32_t y0 = (uint32_t)py;
    uint32_t x1 = x0 + 1;
    uint32_t y1 = y0 + 1;
    if (x1 >= dem->width)  x1 = dem->width - 1;
    if (y1 >= dem->height) y1 = dem->height - 1;

    double fx = px - (double)x0;
    double fy = py - (double)y0;

    double z00 = (double)dem->data[(size_t)y0 * dem->width + x0];
    double z10 = (double)dem->data[(size_t)y0 * dem->width + x1];
    double z01 = (double)dem->data[(size_t)y1 * dem->width + x0];
    double z11 = (double)dem->data[(size_t)y1 * dem->width + x1];

    double z = z00 * (1.0 - fx) * (1.0 - fy)
             + z10 * fx * (1.0 - fy)
             + z01 * (1.0 - fx) * fy
             + z11 * fx * fy;

    return z;
}

void arpt_dem_free(arpt_dem *dem) {
    if (!dem) return;
    free(dem->data);
    free(dem);
}
