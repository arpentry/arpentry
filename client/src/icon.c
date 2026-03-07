#include "icon.h"
#include "icon_map.h"
#include "stb_truetype.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Embedded Maki icon font (generated at build time) */
#include "maki-icons.ttf.h"

/*
 * Generate an SDF icon atlas using stb_truetype.
 *
 * Uses stbtt_GetGlyphSDF() to produce signed distance fields from the
 * Maki icon font. Icons are assigned codepoints in the Private Use Area
 * starting at U+E000.
 */

/* SDF generation parameters — icons are chunkier than text, so we use
 * a larger render size and spread. */
#define ICON_SIZE       64.0f  /* render size in pixels */
#define SDF_PADDING      8     /* extra pixels around each icon for SDF spread */
#define SDF_ON_EDGE    128     /* SDF value at the icon edge */
#define SDF_PIXEL_DIST   8.0f  /* distance in pixels for full SDF range */
#define GLYPH_PAD        2     /* spacing between icons in atlas */

float icon_generate_atlas(uint8_t *rgba_out, icon_glyph *glyphs_out,
                           int *icon_count_out) {
    int atlas_w = ICON_ATLAS_SIZE;
    int atlas_h = ICON_ATLAS_SIZE;
    int count = ICON_COUNT;

    if (icon_count_out) *icon_count_out = count;

    /* Clear atlas to transparent */
    memset(rgba_out, 0, (size_t)(atlas_w * atlas_h * 4));
    memset(glyphs_out, 0, (size_t)count * sizeof(icon_glyph));

    /* Initialize stb_truetype */
    stbtt_fontinfo font;
    if (!stbtt_InitFont(&font, maki_icons_ttf,
                        stbtt_GetFontOffsetForIndex(maki_icons_ttf, 0))) {
        return ICON_SIZE;
    }

    float scale = stbtt_ScaleForPixelHeight(&font, ICON_SIZE);

    /* Pack icons into atlas row by row */
    int cursor_x = GLYPH_PAD;
    int cursor_y = GLYPH_PAD;
    int row_height = 0;

    for (int i = 0; i < count; i++) {
        int codepoint = icon_map[i].codepoint;
        int glyph_idx = stbtt_FindGlyphIndex(&font, codepoint);

        /* Generate SDF bitmap */
        int gw, gh, xoff, yoff;
        unsigned char *sdf = stbtt_GetGlyphSDF(
            &font, scale, glyph_idx, SDF_PADDING, SDF_ON_EDGE,
            SDF_PIXEL_DIST, &gw, &gh, &xoff, &yoff);

        if (!sdf || gw == 0 || gh == 0) {
            if (sdf) stbtt_FreeSDF(sdf, NULL);
            continue;
        }

        /* Advance to next row if needed */
        if (cursor_x + gw + GLYPH_PAD > atlas_w) {
            cursor_x = GLYPH_PAD;
            cursor_y += row_height + GLYPH_PAD;
            row_height = 0;
        }

        /* Check if we've run out of atlas space */
        if (cursor_y + gh + GLYPH_PAD > atlas_h) {
            stbtt_FreeSDF(sdf, NULL);
            break;
        }

        /* Copy SDF data into RGBA atlas */
        for (int y = 0; y < gh; y++) {
            for (int x = 0; x < gw; x++) {
                int ai = ((cursor_y + y) * atlas_w + (cursor_x + x)) * 4;
                uint8_t val = sdf[y * gw + x];
                rgba_out[ai + 0] = val;
                rgba_out[ai + 1] = val;
                rgba_out[ai + 2] = val;
                rgba_out[ai + 3] = 255;
            }
        }

        /* Fill icon metrics */
        glyphs_out[i].u0 = (float)cursor_x / (float)atlas_w;
        glyphs_out[i].v0 = (float)cursor_y / (float)atlas_h;
        glyphs_out[i].u1 = (float)(cursor_x + gw) / (float)atlas_w;
        glyphs_out[i].v1 = (float)(cursor_y + gh) / (float)atlas_h;
        glyphs_out[i].width = (float)gw;
        glyphs_out[i].height = (float)gh;

        /* Advance cursor */
        if (gh > row_height) row_height = gh;
        cursor_x += gw + GLYPH_PAD;

        stbtt_FreeSDF(sdf, NULL);
    }

    return ICON_SIZE;
}

int icon_find(const char *name) {
    if (!name) return -1;
    for (int i = 0; i < ICON_COUNT; i++) {
        if (strcmp(icon_map[i].name, name) == 0) return i;
    }
    return -1;
}

int icon_count(void) {
    return ICON_COUNT;
}
