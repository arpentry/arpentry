/* Merge connected transportation segments into longer linestrings.
 *
 * Reads an Overture segment.parquet file, builds a connectivity graph
 * from the embedded connector references, merges chains of same-class
 * segments through degree-2 connectors, and writes a new GeoParquet
 * file with merged linestrings.  This prevents gaps at low zoom levels
 * where short individual segments would be filtered out.
 *
 * Named road classes (motorway, trunk, primary, secondary, tertiary
 * and their _link variants) are included in the output, each tagged
 * with a min_zoom appropriate for its importance.  Other classes
 * (residential, service, etc.) are omitted — the tiler should receive
 * the original segment.parquet alongside the merged output if those
 * higher-detail classes are needed at higher zoom levels. */

#ifndef ARPT_MERGE_H
#define ARPT_MERGE_H

#include <stdbool.h>

/* Run the segment merge pipeline.
 *   input_path  — path to Overture segment.parquet
 *   output_path — path for merged output GeoParquet
 *   bbox        — optional geographic filter [w,s,e,n] (NULL for world)
 * Returns true on success. */
bool arpt_merge_run(const char *input_path,
                    const char *output_path,
                    const double *bbox);

#endif
