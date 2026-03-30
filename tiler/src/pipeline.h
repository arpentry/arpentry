/* Top-level tiler pipeline orchestration. */

#ifndef ARPT_PIPELINE_H
#define ARPT_PIPELINE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* An input GeoParquet file mapped to a tile layer. */
typedef struct {
    const char *path;         /* Path to GeoParquet file */
    uint32_t    layer;        /* Layer index (0-15) */
} arpt_pipeline_input;

typedef struct {
    const char *output;       /* Output .arpa path */
    const char *tmp_dir;      /* Temp directory for sort runs */
    size_t      mem_budget;   /* Memory budget for external sort */
    double      bbox[4];      /* West, south, east, north (degrees) */
    int         min_zoom;
    int         max_zoom;
    bool        synthetic;    /* Use synthetic test data */

    const arpt_pipeline_input *inputs;  /* Input GeoParquet files */
    int                        n_inputs;

    const char *dem_path;     /* Path to GeoTIFF DEM (optional) */

    int         n_threads;    /* Worker threads (0 = auto-detect) */
} arpt_pipeline_config;

/* Run the full tiling pipeline. Returns false on error. */
bool arpt_pipeline_run(const arpt_pipeline_config *config);

#endif
