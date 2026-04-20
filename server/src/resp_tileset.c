#include "resp_tileset.h"
#include "archive.h"
#include "tile.h"
#include "tileset_builder.h"

#include <stdlib.h>

#define BROTLI_QUALITY 4

bool resp_build_tileset(const struct arpt_archive_reader *archive,
                        uint8_t **out, size_t *out_size) {
    flatcc_builder_t builder;
    flatcc_builder_init(&builder);

    arpentry_tiles_Tileset_start_as_root(&builder);
    arpentry_tiles_Tileset_version_add(&builder, 1);
    arpentry_tiles_Tileset_name_create_str(&builder, "Generated Terrain");

    arpentry_tiles_Bounds_t bounds = {
        .west = -180.0, .south = -90.0, .east = 180.0, .north = 90.0};
    arpentry_tiles_Tileset_bounds_add(&builder, &bounds);

    arpentry_tiles_ElevationRange_t elev = {.min = -500.0, .max = 4800.0};
    arpentry_tiles_Tileset_elevation_range_add(&builder, &elev);

    /* Data availability limit.  19 matches the Overture Maps zoom range.
       The tile path parser accepts up to 21 (address-space limit), but the
       client should not request beyond what the tileset metadata advertises.
       Clamped to the archive's zoom range when serving from an .arpa file. */
    int max_level = 19;
    if (archive) {
        int archive_max = (int)arpt_archive_reader_max_zoom(archive);
        if (archive_max < max_level) max_level = archive_max;
    }

    arpentry_tiles_Tileset_min_level_add(&builder, 0);
    arpentry_tiles_Tileset_max_level_add(&builder, max_level);
    arpentry_tiles_Tileset_root_error_add(&builder, 400000.0);

    /* Layers in decode-priority order (Section 9) */
    arpentry_tiles_Tileset_layers_start(&builder);

    /* terrain: Mesh, 0-19 */
    arpentry_tiles_Tileset_layers_push_start(&builder);
    arpentry_tiles_LayerInfo_name_create_str(&builder, "terrain");
    arpentry_tiles_GeometryType_enum_t mesh_types[] = {
        arpentry_tiles_GeometryType_Mesh};
    arpentry_tiles_LayerInfo_geometry_types_create(&builder, mesh_types, 1);
    arpentry_tiles_LayerInfo_min_level_add(&builder, 0);
    arpentry_tiles_LayerInfo_max_level_add(&builder, 19);
    arpentry_tiles_Tileset_layers_push_end(&builder);

    /* surface: Polygon, 0-19 */
    arpentry_tiles_Tileset_layers_push_start(&builder);
    arpentry_tiles_LayerInfo_name_create_str(&builder, "surface");
    arpentry_tiles_GeometryType_enum_t poly_types[] = {
        arpentry_tiles_GeometryType_Polygon};
    arpentry_tiles_LayerInfo_geometry_types_create(&builder, poly_types, 1);
    arpentry_tiles_LayerInfo_min_level_add(&builder, 0);
    arpentry_tiles_LayerInfo_max_level_add(&builder, 19);
    arpentry_tiles_Tileset_layers_push_end(&builder);

    /* transportation: Line, 8-19 */
    arpentry_tiles_Tileset_layers_push_start(&builder);
    arpentry_tiles_LayerInfo_name_create_str(&builder, "transportation");
    arpentry_tiles_GeometryType_enum_t line_types[] = {
        arpentry_tiles_GeometryType_Line};
    arpentry_tiles_LayerInfo_geometry_types_create(&builder, line_types, 1);
    arpentry_tiles_LayerInfo_min_level_add(&builder, 8);
    arpentry_tiles_LayerInfo_max_level_add(&builder, 19);
    arpentry_tiles_Tileset_layers_push_end(&builder);

    /* building: Polygon, 13-19 */
    arpentry_tiles_Tileset_layers_push_start(&builder);
    arpentry_tiles_LayerInfo_name_create_str(&builder, "building");
    arpentry_tiles_LayerInfo_geometry_types_create(&builder, poly_types, 1);
    arpentry_tiles_LayerInfo_min_level_add(&builder, 13);
    arpentry_tiles_LayerInfo_max_level_add(&builder, 19);
    arpentry_tiles_Tileset_layers_push_end(&builder);

    /* tree: Point, 13-19 */
    arpentry_tiles_Tileset_layers_push_start(&builder);
    arpentry_tiles_LayerInfo_name_create_str(&builder, "tree");
    arpentry_tiles_GeometryType_enum_t point_types[] = {
        arpentry_tiles_GeometryType_Point};
    arpentry_tiles_LayerInfo_geometry_types_create(&builder, point_types, 1);
    arpentry_tiles_LayerInfo_min_level_add(&builder, 13);
    arpentry_tiles_LayerInfo_max_level_add(&builder, 19);
    arpentry_tiles_Tileset_layers_push_end(&builder);

    arpentry_tiles_Tileset_layers_end(&builder);
    arpentry_tiles_Tileset_end_as_root(&builder);

    size_t fb_size;
    void *fb = flatcc_builder_finalize_buffer(&builder, &fb_size);
    flatcc_builder_clear(&builder);
    if (!fb) return false;

    bool ok = arpt_encode(fb, fb_size, out, out_size, BROTLI_QUALITY);
    free(fb);
    return ok;
}
