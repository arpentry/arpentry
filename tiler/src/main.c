/* arpentry_tiler CLI entry point. */

#include "layers.h"
#include "pipeline.h"

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
        "  --bbox <w,s,e,n>       Geographic bounds in degrees (default: world)\n"
        "  --min-zoom <z>         Minimum zoom level (default: 0)\n"
        "  --max-zoom <z>         Maximum zoom level (default: 10)\n"
        "  --tmp <dir>            Temp directory for sort runs (default: /tmp)\n"
        "  --mem <bytes>          Memory budget for external sort (default: 64 MB)\n"
        "  --threads <n>          Worker threads (default: auto-detect CPU count)\n"
        "\n"
        "Layer indices (<layer>:<path>):\n");
    for (int i = 0; i < ARPT_NAMED_LAYERS; i++) {
        fprintf(stderr, "  %d=%s%s\n", i, arpt_layer_names[i],
                i == ARPT_LAYER_TERRAIN ? " (auto-generated, not a valid input)" : "");
    }
}

/* Parse "<layer>:<path>" strictly: digits, ':', path.  Returns false on any
 * malformed spec or out-of-range layer index, with a clear error written to
 * stderr.  On success, fills *layer_out and *path_out. */
static bool parse_input_spec(const char *arg, uint32_t *layer_out,
                             const char **path_out) {
    char *endptr = NULL;
    long n = strtol(arg, &endptr, 10);
    if (endptr == arg || *endptr != ':') {
        fprintf(stderr, "Error: --input %s: expected <layer>:<path>\n", arg);
        return false;
    }
    if (n < 0 || n >= ARPT_NAMED_LAYERS) {
        fprintf(stderr,
                "Error: --input %s: layer index %ld out of range [0, %d)\n",
                arg, n, ARPT_NAMED_LAYERS);
        return false;
    }
    if (n == ARPT_LAYER_TERRAIN) {
        fprintf(stderr,
                "Error: --input %s: layer %d (\"%s\") is auto-generated, "
                "cannot be used as input\n",
                arg, ARPT_LAYER_TERRAIN, arpt_layer_names[ARPT_LAYER_TERRAIN]);
        return false;
    }
    const char *path = endptr + 1;
    if (*path == '\0') {
        fprintf(stderr, "Error: --input %s: path is empty\n", arg);
        return false;
    }
    *layer_out = (uint32_t)n;
    *path_out = path;
    return true;
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
        .max_zoom   = 10,
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
            uint32_t layer;
            const char *path;
            if (!parse_input_spec(argv[i], &layer, &path)) {
                return 1;
            }
            for (int j = 0; j < n_inputs; j++) {
                if (inputs[j].layer == layer) {
                    fprintf(stderr,
                            "Warning: layer %u (%s) already assigned to %s; "
                            "new input %s will be appended\n",
                            layer, arpt_layer_names[layer], inputs[j].path, path);
                    break;
                }
            }
            inputs[n_inputs].layer = layer;
            inputs[n_inputs].path = path;
            n_inputs++;
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
        } else if (strcmp(argv[i], "--threads") == 0 && i + 1 < argc) {
            config.n_threads = atoi(argv[++i]);
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

    if (config.n_inputs == 0) {
        fprintf(stderr, "Error: at least one --input is required\n");
        usage();
        return 1;
    }

    if (!arpt_pipeline_run(&config)) {
        fprintf(stderr, "Error: pipeline failed\n");
        return 1;
    }

    return 0;
}
