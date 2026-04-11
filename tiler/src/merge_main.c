/* arpentry_merge CLI entry point. */

#include "merge.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void usage(void) {
    fprintf(stderr,
        "Usage: arpentry_merge [options]\n"
        "\n"
        "  --input <path>         Input segment.parquet (required)\n"
        "  --output <path>        Output merged.parquet (required)\n"
        "  --bbox <w,s,e,n>       Geographic bounds in degrees (default: world)\n"
        "\n"
        "Merges connected major-road segments (motorway, trunk, primary)\n"
        "into longer linestrings to prevent gaps at low zoom levels.\n"
        "Only these classes are included in the output.\n"
    );
}

int main(int argc, char **argv) {
    const char *input = NULL;
    const char *output = NULL;
    double bbox[4] = {-180.0, -90.0, 180.0, 90.0};
    bool has_bbox = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--input") == 0 && i + 1 < argc) {
            input = argv[++i];
        } else if (strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
            output = argv[++i];
        } else if (strcmp(argv[i], "--bbox") == 0 && i + 1 < argc) {
            if (sscanf(argv[++i], "%lf,%lf,%lf,%lf",
                       &bbox[0], &bbox[1], &bbox[2], &bbox[3]) != 4) {
                fprintf(stderr, "Error: invalid --bbox format\n");
                return 1;
            }
            has_bbox = true;
        } else {
            usage();
            return 1;
        }
    }

    if (!input) {
        fprintf(stderr, "Error: --input is required\n");
        usage();
        return 1;
    }
    if (!output) {
        fprintf(stderr, "Error: --output is required\n");
        usage();
        return 1;
    }

    if (!arpt_merge_run(input, output, has_bbox ? bbox : NULL)) {
        fprintf(stderr, "Error: merge failed\n");
        return 1;
    }

    return 0;
}
