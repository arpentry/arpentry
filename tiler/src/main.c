/* arpentry_tiler CLI entry point. */

#include "pipeline.h"
#include "layers.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DEFAULT_MEM_BUDGET (64 * 1024 * 1024)
#define MAX_INPUTS 64

static void usage(void) {
    fprintf(stderr,
        "Usage: arpentry_tiler [options]\n"
        "\n"
        "  --output <path>        Output .arpa archive path (required)\n"
        "  --input <layer>:<path> Add a GeoParquet input file mapped to layer\n"
        "  --synthetic            Use synthetic test data\n"
        "  --bbox <w,s,e,n>       Geographic bounds in degrees (default: world)\n"
        "  --min-zoom <z>         Minimum zoom level (default: 0)\n"
        "  --max-zoom <z>         Maximum zoom level (default: 4)\n"
        "  --tmp <dir>            Temp directory for sort runs (default: /tmp)\n"
        "  --mem <bytes>          Memory budget for external sort (default: 64 MB)\n"
        "  --dem <path>           GeoTIFF DEM for terrain elevation (optional)\n"
        "\n"
        "Layer indices (matching style.json):\n"
        "  1=surface  2=highway  3=building  4=tree  5=poi\n"
    );
}

int main(int argc, char **argv) {
    arpt_pipeline_input inputs[MAX_INPUTS];
    int n_inputs = 0;

    arpt_pipeline_config config = {
        .output     = NULL,
        .tmp_dir    = "/tmp",
        .mem_budget = DEFAULT_MEM_BUDGET,
        .bbox       = {-180.0, -90.0, 180.0, 90.0},
        .min_zoom   = 0,
        .max_zoom   = 4,
        .synthetic  = false,
        .inputs     = inputs,
        .n_inputs   = 0,
    };

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
            config.output = argv[++i];
        } else if (strcmp(argv[i], "--input") == 0 && i + 1 < argc) {
            i++;
            if (n_inputs >= MAX_INPUTS) {
                fprintf(stderr, "Error: too many inputs (max %d)\n", MAX_INPUTS);
                return 1;
            }
            /* Parse "layer:path" */
            const char *arg = argv[i];
            const char *colon = strchr(arg, ':');
            if (!colon) {
                fprintf(stderr, "Error: --input requires <layer>:<path> format\n");
                return 1;
            }
            inputs[n_inputs].layer = (uint32_t)atoi(arg);
            inputs[n_inputs].path = colon + 1;
            n_inputs++;
        } else if (strcmp(argv[i], "--synthetic") == 0) {
            config.synthetic = true;
        } else if (strcmp(argv[i], "--bbox") == 0 && i + 1 < argc) {
            if (sscanf(argv[++i], "%lf,%lf,%lf,%lf",
                       &config.bbox[0], &config.bbox[1],
                       &config.bbox[2], &config.bbox[3]) != 4) {
                fprintf(stderr, "Error: invalid --bbox format\n");
                return 1;
            }
        } else if (strcmp(argv[i], "--min-zoom") == 0 && i + 1 < argc) {
            config.min_zoom = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--max-zoom") == 0 && i + 1 < argc) {
            config.max_zoom = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--tmp") == 0 && i + 1 < argc) {
            config.tmp_dir = argv[++i];
        } else if (strcmp(argv[i], "--mem") == 0 && i + 1 < argc) {
            config.mem_budget = (size_t)atol(argv[++i]);
        } else if (strcmp(argv[i], "--dem") == 0 && i + 1 < argc) {
            config.dem_path = argv[++i];
        } else {
            usage();
            return 1;
        }
    }

    config.n_inputs = n_inputs;

    if (!config.output) {
        fprintf(stderr, "Error: --output is required\n");
        usage();
        return 1;
    }

    if (!config.synthetic && config.n_inputs == 0) {
        fprintf(stderr, "Error: need --synthetic or at least one --input\n");
        usage();
        return 1;
    }

    if (!arpt_pipeline_run(&config)) {
        fprintf(stderr, "Error: pipeline failed\n");
        return 1;
    }

    return 0;
}
