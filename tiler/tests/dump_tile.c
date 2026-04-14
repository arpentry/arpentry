/* Quick diagnostic: dump tile content from an archive. */
#include "archive.h"
#include "tile_reader.h"
#include <brotli/decode.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv) {
    if (argc < 4) {
        fprintf(stderr, "Usage: dump_tile <archive> <z> <x> [y]\n");
        return 1;
    }
    const char *path = argv[1];
    int z = atoi(argv[2]);
    int x = atoi(argv[3]);
    int y = argc > 4 ? atoi(argv[4]) : 0;

    arpt_archive_reader *r = arpt_archive_reader_open(path);
    if (!r) { fprintf(stderr, "Cannot open %s\n", path); return 1; }

    printf("Archive: %llu tiles\n", (unsigned long long)arpt_archive_reader_tile_count(r));

    size_t comp_size;
    const void *comp = arpt_archive_reader_get_tile(r, (uint8_t)z, (uint32_t)x, (uint32_t)y, &comp_size);
    if (!comp) { fprintf(stderr, "Tile %d/%d/%d not found\n", z, x, y); arpt_archive_reader_close(r); return 1; }
    printf("Tile %d/%d/%d: %zu compressed bytes\n", z, x, y, comp_size);

    /* Decompress */
    size_t dec_size = comp_size * 20;
    uint8_t *dec = malloc(dec_size);
    if (BrotliDecoderDecompress(comp_size, comp, &dec_size, dec) != BROTLI_DECODER_RESULT_SUCCESS) {
        fprintf(stderr, "Brotli decode failed\n");
        free(dec); arpt_archive_reader_close(r); return 1;
    }
    printf("Decompressed: %zu bytes\n", dec_size);

    /* Parse FlatBuffer */
    arpentry_tiles_Tile_table_t tile = arpentry_tiles_Tile_as_root(dec);
    if (!tile) { fprintf(stderr, "FlatBuffer parse failed\n"); free(dec); arpt_archive_reader_close(r); return 1; }

    /* Keys/values dictionaries */
    flatbuffers_string_vec_t keys = arpentry_tiles_Tile_keys(tile);
    arpentry_tiles_Value_vec_t vals = arpentry_tiles_Tile_values(tile);
    size_t n_keys = keys ? flatbuffers_string_vec_len(keys) : 0;
    size_t n_vals = vals ? arpentry_tiles_Value_vec_len(vals) : 0;
    printf("Keys (%zu):", n_keys);
    for (size_t i = 0; i < n_keys; i++)
        printf(" [%zu]=\"%s\"", i, flatbuffers_string_vec_at(keys, i));
    printf("\nValues (%zu):", n_vals);
    for (size_t i = 0; i < n_vals && i < 20; i++) {
        arpentry_tiles_Value_table_t v = arpentry_tiles_Value_vec_at(vals, i);
        const char *sv = arpentry_tiles_Value_string_value(v);
        printf(" [%zu]=\"%s\"", i, sv ? sv : "(null)");
    }
    if (n_vals > 20) printf(" ... +%zu more", n_vals - 20);
    printf("\n\n");

    /* Layers */
    arpentry_tiles_Layer_vec_t layers = arpentry_tiles_Tile_layers(tile);
    size_t n_layers = layers ? arpentry_tiles_Layer_vec_len(layers) : 0;
    printf("Layers: %zu\n", n_layers);
    for (size_t li = 0; li < n_layers; li++) {
        arpentry_tiles_Layer_table_t layer = arpentry_tiles_Layer_vec_at(layers, li);
        const char *name = arpentry_tiles_Layer_name(layer);
        arpentry_tiles_Feature_vec_t feats = arpentry_tiles_Layer_features(layer);
        size_t n_feats = feats ? arpentry_tiles_Feature_vec_len(feats) : 0;
        printf("  Layer %zu: \"%s\" (%zu features)\n", li, name ? name : "(null)", n_feats);

        /* Count features per class */
        size_t class_counts[64] = {0};
        for (size_t fi = 0; fi < n_feats; fi++) {
            arpentry_tiles_Feature_table_t f2 = arpentry_tiles_Feature_vec_at(feats, fi);
            arpentry_tiles_Property_vec_t p2 = arpentry_tiles_Feature_properties(f2);
            if (p2 && arpentry_tiles_Property_vec_len(p2) > 0) {
                const arpentry_tiles_Property_struct_t pp = arpentry_tiles_Property_vec_at(p2, 0);
                if (pp->value < 64) class_counts[pp->value]++;
            }
        }
        printf("    Class distribution:");
        for (size_t ci = 0; ci < n_vals && ci < 64; ci++) {
            if (class_counts[ci] > 0) {
                arpentry_tiles_Value_table_t vt = arpentry_tiles_Value_vec_at(vals, ci);
                const char *sv = arpentry_tiles_Value_string_value(vt);
                printf(" %s=%zu", sv ? sv : "?", class_counts[ci]);
            }
        }
        printf("\n");

        /* Print first 50 features' properties */
        for (size_t fi = 0; fi < n_feats && fi < 50; fi++) {
            arpentry_tiles_Feature_table_t feat = arpentry_tiles_Feature_vec_at(feats, fi);
            arpentry_tiles_Property_vec_t props = arpentry_tiles_Feature_properties(feat);
            size_t n_props = props ? arpentry_tiles_Property_vec_len(props) : 0;
            printf("    feat[%zu]: %zu props", fi, n_props);
            for (size_t pi = 0; pi < n_props; pi++) {
                const arpentry_tiles_Property_struct_t p = arpentry_tiles_Property_vec_at(props, pi);
                uint32_t ki = p->key, vi = p->value;
                const char *k = ki < n_keys ? flatbuffers_string_vec_at(keys, ki) : "?";
                const char *v = "(?)";
                if (vi < n_vals) {
                    arpentry_tiles_Value_table_t vt = arpentry_tiles_Value_vec_at(vals, vi);
                    v = arpentry_tiles_Value_string_value(vt);
                    if (!v) v = "(null)";
                }
                printf(" %s=%s", k, v);
            }

            /* Geometry type + coordinates */
            if (arpentry_tiles_Feature_geometry_type(feat) == arpentry_tiles_Geometry_PolygonGeometry)
                printf(" [polygon]");
            else if (arpentry_tiles_Feature_geometry_type(feat) == arpentry_tiles_Geometry_MeshGeometry)
                printf(" [mesh]");
            else if (arpentry_tiles_Feature_geometry_type(feat) == arpentry_tiles_Geometry_LineGeometry) {
                arpentry_tiles_LineGeometry_table_t lg =
                    (arpentry_tiles_LineGeometry_table_t)arpentry_tiles_Feature_geometry(feat);
                flatbuffers_uint16_vec_t lx = arpentry_tiles_LineGeometry_x(lg);
                flatbuffers_uint16_vec_t ly = arpentry_tiles_LineGeometry_y(lg);
                flatbuffers_uint32_vec_t lo = arpentry_tiles_LineGeometry_line_offsets(lg);
                size_t lvc = lx ? flatbuffers_uint16_vec_len(lx) : 0;
                size_t lon = lo ? flatbuffers_uint32_vec_len(lo) : 0;
                printf(" [line vc=%zu offsets=%zu]", lvc, lon);
                /* Print first and last coordinates */
                if (lvc > 0) {
                    printf("\n      first=(%u,%u) last=(%u,%u)",
                           lx[0], ly[0], lx[lvc-1], ly[lvc-1]);
                    if (lx[0] == lx[lvc-1] && ly[0] == ly[lvc-1]) {
                        printf(" ** CLOSED **");
                        /* Print all vertices for closed lines */
                        printf("\n      all_coords:");
                        for (size_t ci = 0; ci < lvc; ci++)
                            printf(" (%u,%u)", lx[ci], ly[ci]);
                    }
                }
            } else if (arpentry_tiles_Feature_geometry_type(feat) == arpentry_tiles_Geometry_PointGeometry)
                printf(" [point]");
            printf("\n");
        }
        if (n_feats > 50) printf("    ... +%zu more\n", n_feats - 50);
    }

    free(dec);
    arpt_archive_reader_close(r);
    return 0;
}
