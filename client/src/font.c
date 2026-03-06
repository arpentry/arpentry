#include "font.h"
#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Embedded Inter font (generated at build time) */
#include "Inter-Regular.ttf.h"

/*
 * Generate an SDF font atlas for ASCII 32-126 using stb_truetype.
 *
 * Uses stbtt_GetGlyphSDF() to produce high-quality signed distance fields
 * from the Inter font. The atlas is packed row-by-row with padding between
 * glyphs.
 */

/* SDF generation parameters */
#define FONT_SIZE      40.0f  /* render size in pixels */
#define SDF_PADDING     6     /* extra pixels around each glyph for SDF spread */
#define SDF_ON_EDGE   128     /* SDF value at the glyph edge */
#define SDF_PIXEL_DIST  6.0f  /* distance in pixels for full SDF range */
#define GLYPH_PAD       2     /* spacing between glyphs in atlas */

float font_generate_atlas(uint8_t *rgba_out, font_glyph *glyphs_out) {
    int atlas_w = FONT_ATLAS_SIZE;
    int atlas_h = FONT_ATLAS_SIZE;

    /* Clear atlas to transparent */
    memset(rgba_out, 0, (size_t)(atlas_w * atlas_h * 4));

    /* Initialize stb_truetype */
    stbtt_fontinfo font;
    if (!stbtt_InitFont(&font, inter_regular_ttf,
                        stbtt_GetFontOffsetForIndex(inter_regular_ttf, 0))) {
        /* Fallback: leave atlas blank */
        memset(glyphs_out, 0, FONT_CHAR_COUNT * sizeof(font_glyph));
        return FONT_SIZE;
    }

    float scale = stbtt_ScaleForPixelHeight(&font, FONT_SIZE);

    /* Get font vertical metrics */
    int ascent, descent, line_gap;
    stbtt_GetFontVMetrics(&font, &ascent, &descent, &line_gap);

    /* Pack glyphs into atlas row by row */
    int cursor_x = GLYPH_PAD;
    int cursor_y = GLYPH_PAD;
    int row_height = 0;

    for (int ci = 0; ci < FONT_CHAR_COUNT; ci++) {
        int ch = FONT_FIRST_CHAR + ci;
        int glyph_idx = stbtt_FindGlyphIndex(&font, ch);

        /* Get glyph metrics */
        int advance_w, left_bearing;
        stbtt_GetGlyphHMetrics(&font, glyph_idx, &advance_w, &left_bearing);

        /* Generate SDF bitmap */
        int gw, gh, xoff, yoff;
        unsigned char *sdf = stbtt_GetGlyphSDF(
            &font, scale, glyph_idx, SDF_PADDING, SDF_ON_EDGE,
            SDF_PIXEL_DIST, &gw, &gh, &xoff, &yoff);

        if (!sdf || gw == 0 || gh == 0) {
            /* Space or empty glyph */
            glyphs_out[ci].u0 = 0;
            glyphs_out[ci].v0 = 0;
            glyphs_out[ci].u1 = 0;
            glyphs_out[ci].v1 = 0;
            glyphs_out[ci].advance = (float)advance_w * scale;
            glyphs_out[ci].bearing_x = (float)left_bearing * scale;
            glyphs_out[ci].bearing_y = 0;
            glyphs_out[ci].width = 0;
            glyphs_out[ci].height = 0;
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
            /* Fill remaining glyphs with zeros */
            for (int j = ci; j < FONT_CHAR_COUNT; j++)
                memset(&glyphs_out[j], 0, sizeof(font_glyph));
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

        /* Fill glyph metrics */
        glyphs_out[ci].u0 = (float)cursor_x / (float)atlas_w;
        glyphs_out[ci].v0 = (float)cursor_y / (float)atlas_h;
        glyphs_out[ci].u1 = (float)(cursor_x + gw) / (float)atlas_w;
        glyphs_out[ci].v1 = (float)(cursor_y + gh) / (float)atlas_h;
        glyphs_out[ci].advance = (float)advance_w * scale;
        glyphs_out[ci].bearing_x = (float)xoff;
        glyphs_out[ci].bearing_y = (float)yoff;
        glyphs_out[ci].width = (float)gw;
        glyphs_out[ci].height = (float)gh;

        /* Advance cursor */
        if (gh > row_height) row_height = gh;
        cursor_x += gw + GLYPH_PAD;

        stbtt_FreeSDF(sdf, NULL);
    }

    return FONT_SIZE;
}
