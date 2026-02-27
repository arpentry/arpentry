#include "demo_tile.h"
#include "tile.h"
#include "tile_builder.h"

#include <stdlib.h>

/* Property dictionary indices */

/* Keys (Tile.keys) */
enum {
    KEY_CLASS = 0,
    KEY_NAME = 1,
    KEY_POP = 2,
    KEY_ELEV = 3,
    KEY_ONEWAY = 4,
    KEY_COUNT = 5
};

/* Values (Tile.values) */
enum {
    VAL_PEAK = 0,       /* String "peak"           */
    VAL_SUMMIT = 1,     /* String "Summit Point"   */
    VAL_VIEWPOINT = 2,  /* String "viewpoint"      */
    VAL_LOOKOUT = 3,    /* String "The Lookout"    */
    VAL_SADDLE = 4,     /* String "saddle"         */
    VAL_LOW_PASS = 5,   /* String "Low Pass"       */
    VAL_POP_1200 = 6,   /* Int 1200                */
    VAL_ELEV_850 = 7,   /* Double 850.5            */
    VAL_TRUE = 8,       /* Bool true               */
    VAL_ROAD = 9,       /* String "road"           */
    VAL_MAIN_ST = 10,   /* String "Main Street"    */
    VAL_BUILDING = 11,  /* String "building"       */
    VAL_TOWN_HALL = 12, /* String "Town Hall"      */
    VAL_COUNT = 13
};

/* Helpers */

static void build_keys(flatcc_builder_t *b) {
    arpentry_tiles_Tile_keys_start(b);
    arpentry_tiles_Tile_keys_push_create_str(b, "class");
    arpentry_tiles_Tile_keys_push_create_str(b, "name");
    arpentry_tiles_Tile_keys_push_create_str(b, "population");
    arpentry_tiles_Tile_keys_push_create_str(b, "elevation");
    arpentry_tiles_Tile_keys_push_create_str(b, "oneway");
    arpentry_tiles_Tile_keys_end(b);
}

static void push_string_value(flatcc_builder_t *b, const char *s) {
    arpentry_tiles_Tile_values_push_start(b);
    arpentry_tiles_Value_type_add(b, arpentry_tiles_PropertyValueType_String);
    arpentry_tiles_Value_string_value_create_str(b, s);
    arpentry_tiles_Tile_values_push_end(b);
}

static void push_int_value(flatcc_builder_t *b, int64_t v) {
    arpentry_tiles_Tile_values_push_start(b);
    arpentry_tiles_Value_type_add(b, arpentry_tiles_PropertyValueType_Int);
    arpentry_tiles_Value_int_value_add(b, v);
    arpentry_tiles_Tile_values_push_end(b);
}

static void push_double_value(flatcc_builder_t *b, double v) {
    arpentry_tiles_Tile_values_push_start(b);
    arpentry_tiles_Value_type_add(b, arpentry_tiles_PropertyValueType_Double);
    arpentry_tiles_Value_double_value_add(b, v);
    arpentry_tiles_Tile_values_push_end(b);
}

static void push_bool_value(flatcc_builder_t *b, bool v) {
    arpentry_tiles_Tile_values_push_start(b);
    arpentry_tiles_Value_type_add(b, arpentry_tiles_PropertyValueType_Bool);
    arpentry_tiles_Value_bool_value_add(b, v);
    arpentry_tiles_Tile_values_push_end(b);
}

static void build_values(flatcc_builder_t *b) {
    arpentry_tiles_Tile_values_start(b);
    push_string_value(b, "peak");         /* 0  */
    push_string_value(b, "Summit Point"); /* 1  */
    push_string_value(b, "viewpoint");    /* 2  */
    push_string_value(b, "The Lookout");  /* 3  */
    push_string_value(b, "saddle");       /* 4  */
    push_string_value(b, "Low Pass");     /* 5  */
    push_int_value(b, 1200);              /* 6  */
    push_double_value(b, 850.5);          /* 7  */
    push_bool_value(b, true);             /* 8  */
    push_string_value(b, "road");         /* 9  */
    push_string_value(b, "Main Street");  /* 10 */
    push_string_value(b, "building");     /* 11 */
    push_string_value(b, "Town Hall");    /* 12 */
    arpentry_tiles_Tile_values_end(b);
}

/* Layer 1: points */

static void build_points_layer(flatcc_builder_t *b) {
    arpentry_tiles_Tile_layers_push_start(b);
    arpentry_tiles_Layer_name_create_str(b, "points");

    arpentry_tiles_Layer_features_start(b);

    /* Feature 0: peak at center, elev 2000m */
    {
        arpentry_tiles_Layer_features_push_start(b);
        arpentry_tiles_Feature_id_add(b, 1);

        uint16_t xs[] = {32768};
        uint16_t ys[] = {32768};
        int32_t zs[] = {2000000}; /* 2000m in mm */

        arpentry_tiles_PointGeometry_ref_t ref;
        arpentry_tiles_PointGeometry_start(b);
        arpentry_tiles_PointGeometry_x_create(b, xs, 1);
        arpentry_tiles_PointGeometry_y_create(b, ys, 1);
        arpentry_tiles_PointGeometry_z_create(b, zs, 1);
        ref = arpentry_tiles_PointGeometry_end(b);
        arpentry_tiles_Feature_geometry_PointGeometry_add(b, ref);

        arpentry_tiles_Property_t props[] = {
            {KEY_CLASS, VAL_PEAK},
            {KEY_NAME, VAL_SUMMIT},
            {KEY_POP, VAL_POP_1200},
            {KEY_ELEV, VAL_ELEV_850},
        };
        arpentry_tiles_Feature_properties_create(b, props, 4);

        arpentry_tiles_Layer_features_push_end(b);
    }

    /* Feature 1: viewpoint, elev 500m */
    {
        arpentry_tiles_Layer_features_push_start(b);
        arpentry_tiles_Feature_id_add(b, 2);

        uint16_t xs[] = {24576};
        uint16_t ys[] = {40960};
        int32_t zs[] = {500000};

        arpentry_tiles_PointGeometry_ref_t ref;
        arpentry_tiles_PointGeometry_start(b);
        arpentry_tiles_PointGeometry_x_create(b, xs, 1);
        arpentry_tiles_PointGeometry_y_create(b, ys, 1);
        arpentry_tiles_PointGeometry_z_create(b, zs, 1);
        ref = arpentry_tiles_PointGeometry_end(b);
        arpentry_tiles_Feature_geometry_PointGeometry_add(b, ref);

        arpentry_tiles_Property_t props[] = {
            {KEY_CLASS, VAL_VIEWPOINT},
            {KEY_NAME, VAL_LOOKOUT},
        };
        arpentry_tiles_Feature_properties_create(b, props, 2);

        arpentry_tiles_Layer_features_push_end(b);
    }

    /* Feature 2: saddle, elev 0m, oneway=true (Bool value type) */
    {
        arpentry_tiles_Layer_features_push_start(b);
        arpentry_tiles_Feature_id_add(b, 3);

        uint16_t xs[] = {40960};
        uint16_t ys[] = {24576};
        int32_t zs[] = {0};

        arpentry_tiles_PointGeometry_ref_t ref;
        arpentry_tiles_PointGeometry_start(b);
        arpentry_tiles_PointGeometry_x_create(b, xs, 1);
        arpentry_tiles_PointGeometry_y_create(b, ys, 1);
        arpentry_tiles_PointGeometry_z_create(b, zs, 1);
        ref = arpentry_tiles_PointGeometry_end(b);
        arpentry_tiles_Feature_geometry_PointGeometry_add(b, ref);

        arpentry_tiles_Property_t props[] = {
            {KEY_CLASS, VAL_SADDLE},
            {KEY_NAME, VAL_LOW_PASS},
            {KEY_ONEWAY, VAL_TRUE},
        };
        arpentry_tiles_Feature_properties_create(b, props, 3);

        arpentry_tiles_Layer_features_push_end(b);
    }

    arpentry_tiles_Layer_features_end(b);
    arpentry_tiles_Tile_layers_push_end(b);
}

/* Layer 2: roads (LineGeometry) */

static void build_roads_layer(flatcc_builder_t *b) {
    arpentry_tiles_Tile_layers_push_start(b);
    arpentry_tiles_Layer_name_create_str(b, "roads");

    arpentry_tiles_Layer_features_start(b);

    /* A road with 2 linestrings (5 vertices total: 3 + 2) */
    {
        arpentry_tiles_Layer_features_push_start(b);
        arpentry_tiles_Feature_id_add(b, 10);

        uint16_t xs[] = {16384, 24576, 32768, 40960, 49151};
        uint16_t ys[] = {32768, 36864, 32768, 28672, 32768};
        int32_t zs[] = {0, 0, 0, 0, 0};
        uint32_t offsets[] = {0, 3,
                              5}; /* line 0: verts 0-2, line 1: verts 3-4 */

        arpentry_tiles_LineGeometry_ref_t ref;
        arpentry_tiles_LineGeometry_start(b);
        arpentry_tiles_LineGeometry_x_create(b, xs, 5);
        arpentry_tiles_LineGeometry_y_create(b, ys, 5);
        arpentry_tiles_LineGeometry_z_create(b, zs, 5);
        arpentry_tiles_LineGeometry_line_offsets_create(b, offsets, 3);
        ref = arpentry_tiles_LineGeometry_end(b);
        arpentry_tiles_Feature_geometry_LineGeometry_add(b, ref);

        arpentry_tiles_Property_t props[] = {
            {KEY_CLASS, VAL_ROAD},
            {KEY_NAME, VAL_MAIN_ST},
            {KEY_ONEWAY, VAL_TRUE},
        };
        arpentry_tiles_Feature_properties_create(b, props, 3);

        arpentry_tiles_Layer_features_push_end(b);
    }

    arpentry_tiles_Layer_features_end(b);
    arpentry_tiles_Tile_layers_push_end(b);
}

/* Layer 3: buildings (PolygonGeometry) */

static void build_buildings_layer(flatcc_builder_t *b) {
    arpentry_tiles_Tile_layers_push_start(b);
    arpentry_tiles_Layer_name_create_str(b, "buildings");

    arpentry_tiles_Layer_features_start(b);

    /* A building with exterior ring + hole */
    {
        arpentry_tiles_Layer_features_push_start(b);
        arpentry_tiles_Feature_id_add(b, 20);

        /* Exterior ring: 5 verts (closed square), hole: 5 verts (closed inner
         * square) */
        uint16_t xs[] = {
            /* exterior */ 28672, 36864, 36864, 28672, 28672,
            /* hole */ 30720,     34816, 34816, 30720, 30720};
        uint16_t ys[] = {
            /* exterior */ 28672, 28672, 36864, 36864, 28672,
            /* hole */ 30720,     30720, 34816, 34816, 30720};
        int32_t zs[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

        uint32_t ring_offsets[] = {0, 5,
                                   10}; /* ring 0: exterior, ring 1: hole */
        uint32_t polygon_offsets[] = {0, 2}; /* polygon 0 uses rings 0-1 */

        arpentry_tiles_PolygonGeometry_ref_t ref;
        arpentry_tiles_PolygonGeometry_start(b);
        arpentry_tiles_PolygonGeometry_x_create(b, xs, 10);
        arpentry_tiles_PolygonGeometry_y_create(b, ys, 10);
        arpentry_tiles_PolygonGeometry_z_create(b, zs, 10);
        arpentry_tiles_PolygonGeometry_ring_offsets_create(b, ring_offsets, 3);
        arpentry_tiles_PolygonGeometry_polygon_offsets_create(
            b, polygon_offsets, 2);
        ref = arpentry_tiles_PolygonGeometry_end(b);
        arpentry_tiles_Feature_geometry_PolygonGeometry_add(b, ref);

        arpentry_tiles_Property_t props[] = {
            {KEY_CLASS, VAL_BUILDING},
            {KEY_NAME, VAL_TOWN_HALL},
        };
        arpentry_tiles_Feature_properties_create(b, props, 2);

        arpentry_tiles_Layer_features_push_end(b);
    }

    arpentry_tiles_Layer_features_end(b);
    arpentry_tiles_Tile_layers_push_end(b);
}

/* Layer 4: terrain (MeshGeometry) */

static void build_terrain_layer(flatcc_builder_t *b) {
    arpentry_tiles_Tile_layers_push_start(b);
    arpentry_tiles_Layer_name_create_str(b, "terrain");

    arpentry_tiles_Layer_features_start(b);

    /* 2-triangle quad with 2 Parts */
    {
        arpentry_tiles_Layer_features_push_start(b);
        arpentry_tiles_Feature_id_add(b, 30);

        uint16_t xs[] = {16384, 49151, 49151, 16384};
        uint16_t ys[] = {16384, 16384, 49151, 49151};
        int32_t zs[] = {0, 100000, 200000, 50000}; /* varying elevation */
        uint32_t indices[] = {0, 1, 2, 0, 2, 3};   /* 2 triangles */

        /* Octahedral normals: int8x2 per vertex (8 values for 4 verts) */
        int8_t normals[] = {0, 127, 0, 127, 0, 127, 0, 127};

        arpentry_tiles_MeshGeometry_ref_t ref;
        arpentry_tiles_MeshGeometry_start(b);
        arpentry_tiles_MeshGeometry_x_create(b, xs, 4);
        arpentry_tiles_MeshGeometry_y_create(b, ys, 4);
        arpentry_tiles_MeshGeometry_z_create(b, zs, 4);
        arpentry_tiles_MeshGeometry_indices_create(b, indices, 6);
        arpentry_tiles_MeshGeometry_normals_create(b, normals, 8);

        /* Part 0: first triangle, inline brown material */
        arpentry_tiles_MeshGeometry_parts_start(b);

        arpentry_tiles_Part_t part0 = {0};
        part0.first_index = 0;
        part0.index_count = 3;
        part0.color.r = 139;
        part0.color.g = 119;
        part0.color.b = 101;
        part0.color.a = 255;
        part0.roughness = 220;
        part0.metalness = 10;
        arpentry_tiles_MeshGeometry_parts_push(b, &part0);

        /* Part 1: second triangle, client-styled (a=0) */
        arpentry_tiles_Part_t part1 = {0};
        part1.first_index = 3;
        part1.index_count = 3;
        part1.color.r = 0;
        part1.color.g = 0;
        part1.color.b = 0;
        part1.color.a = 0;
        part1.roughness = 0;
        part1.metalness = 0;
        arpentry_tiles_MeshGeometry_parts_push(b, &part1);

        arpentry_tiles_MeshGeometry_parts_end(b);

        ref = arpentry_tiles_MeshGeometry_end(b);
        arpentry_tiles_Feature_geometry_MeshGeometry_add(b, ref);

        arpentry_tiles_Layer_features_push_end(b);
    }

    arpentry_tiles_Layer_features_end(b);
    arpentry_tiles_Tile_layers_push_end(b);
}

/* Public API */

bool arpt_build_demo_tile(uint8_t **out, size_t *out_size) {
    if (!out || !out_size) return false;

    flatcc_builder_t builder;
    flatcc_builder_init(&builder);

    arpentry_tiles_Tile_start_as_root(&builder);
    arpentry_tiles_Tile_version_add(&builder, 1);

    build_keys(&builder);
    build_values(&builder);

    arpentry_tiles_Tile_layers_start(&builder);
    build_points_layer(&builder);
    build_roads_layer(&builder);
    build_buildings_layer(&builder);
    build_terrain_layer(&builder);
    arpentry_tiles_Tile_layers_end(&builder);

    arpentry_tiles_Tile_end_as_root(&builder);

    size_t fb_size;
    void *fb = flatcc_builder_finalize_buffer(&builder, &fb_size);
    flatcc_builder_clear(&builder);

    if (!fb) return false;

    bool ok = arpt_encode(fb, fb_size, out, out_size, 4);
    free(fb);
    return ok;
}
