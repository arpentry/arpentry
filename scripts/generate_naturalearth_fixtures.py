#!/usr/bin/env python3
"""Download Natural Earth 110m shapefiles and convert to GeoParquet fixtures.

Produces Overture-compatible parquet files for the tiler integration tests.
Output: tiler/tests/fixtures/naturalearth/{land,coastline,boundary,places}.parquet
"""

import json
import os
import tempfile
import zipfile
from io import BytesIO
from pathlib import Path
from urllib.request import urlopen

import geopandas as gpd
import pyarrow as pa
import pyarrow.parquet as pq
from shapely import wkb

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
OUTPUT_DIR = PROJECT_ROOT / "tiler" / "tests" / "fixtures" / "naturalearth"

LAYERS = [
    {
        "url": "https://naciscdn.org/naturalearth/110m/physical/ne_110m_land.zip",
        "shapefile": "ne_110m_land.shp",
        "output": "land.parquet",
        "type_value": "land",
    },
    {
        "url": "https://naciscdn.org/naturalearth/110m/physical/ne_110m_coastline.zip",
        "shapefile": "ne_110m_coastline.shp",
        "output": "coastline.parquet",
        "type_value": "coastline",
    },
    {
        "url": "https://naciscdn.org/naturalearth/110m/cultural/ne_110m_admin_0_boundary_lines_land.zip",
        "shapefile": "ne_110m_admin_0_boundary_lines_land.shp",
        "output": "boundary.parquet",
        "type_value": "boundary",
    },
    {
        "url": "https://naciscdn.org/naturalearth/110m/cultural/ne_110m_populated_places_simple.zip",
        "shapefile": "ne_110m_populated_places_simple.shp",
        "output": "places.parquet",
        "type_value": "place",
    },
]

GEO_META = json.dumps(
    {
        "primary_column": "geometry",
        "columns": {"geometry": {"encoding": "WKB"}},
    }
)


def download_and_read(url: str, shapefile: str) -> gpd.GeoDataFrame:
    print(f"  Downloading {url} ...")
    resp = urlopen(url)
    data = resp.read()
    with tempfile.TemporaryDirectory() as tmpdir:
        zpath = os.path.join(tmpdir, "data.zip")
        with open(zpath, "wb") as f:
            f.write(data)
        with zipfile.ZipFile(zpath) as zf:
            zf.extractall(tmpdir)
        return gpd.read_file(os.path.join(tmpdir, shapefile))


def gdf_to_overture_parquet(gdf: gpd.GeoDataFrame, output: Path, type_value: str):
    n = len(gdf)

    # Geometry as WKB
    geometry_wkb = [g.wkb for g in gdf.geometry]

    # IDs
    ids = [f"ne_{type_value}_{i}" for i in range(n)]

    # Type column
    types = [type_value] * n

    # Subtype: use NAME column for places if available, else None
    if "name" in gdf.columns:
        subtypes = [str(v) if v is not None else None for v in gdf["name"]]
    elif "NAME" in gdf.columns:
        subtypes = [str(v) if v is not None else None for v in gdf["NAME"]]
    else:
        subtypes = [None] * n

    # Bbox
    bounds = gdf.geometry.bounds  # minx, miny, maxx, maxy
    bbox_xmin = bounds["minx"].astype("float32").tolist()
    bbox_ymin = bounds["miny"].astype("float32").tolist()
    bbox_xmax = bounds["maxx"].astype("float32").tolist()
    bbox_ymax = bounds["maxy"].astype("float32").tolist()

    # Build Arrow table
    bbox_struct = pa.StructArray.from_arrays(
        [
            pa.array(bbox_xmin, type=pa.float32()),
            pa.array(bbox_ymin, type=pa.float32()),
            pa.array(bbox_xmax, type=pa.float32()),
            pa.array(bbox_ymax, type=pa.float32()),
        ],
        names=["xmin", "ymin", "xmax", "ymax"],
    )

    table = pa.table(
        {
            "geometry": pa.array(geometry_wkb, type=pa.binary()),
            "id": pa.array(ids, type=pa.string()),
            "type": pa.array(types, type=pa.string()),
            "subtype": pa.array(subtypes, type=pa.string()),
            "bbox": bbox_struct,
        }
    )

    # Add geo metadata
    existing_meta = table.schema.metadata or {}
    existing_meta[b"geo"] = GEO_META.encode("utf-8")
    table = table.replace_schema_metadata(existing_meta)

    # Write with SNAPPY compression
    pq.write_table(table, str(output), compression="snappy")
    print(f"  Wrote {output} ({n} features)")


def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    for layer in LAYERS:
        print(f"Processing {layer['type_value']}...")
        gdf = download_and_read(layer["url"], layer["shapefile"])
        output = OUTPUT_DIR / layer["output"]
        gdf_to_overture_parquet(gdf, output, layer["type_value"])

    print("Done.")


if __name__ == "__main__":
    main()
