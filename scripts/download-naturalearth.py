#!/usr/bin/env python3
"""Download Natural Earth 10m shapefiles and convert to GeoParquet.

Produces Overture-compatible parquet files for the naturalearth demo.
Output: data/naturalearth/{land,coastline,lake,glacier,river,boundary,places}.parquet

Uses 10m (highest resolution) datasets. The tiler handles simplification
at lower zoom levels automatically.
"""

import json
import os
import tempfile
import zipfile
from pathlib import Path
from urllib.request import urlopen

import geopandas as gpd
import pyarrow as pa
import pyarrow.parquet as pq

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
OUTPUT_DIR = PROJECT_ROOT / "data" / "naturalearth"

LAYERS = [
    {
        "url": "https://naciscdn.org/naturalearth/10m/physical/ne_10m_land.zip",
        "shapefile": "ne_10m_land.shp",
        "output": "land.parquet",
        "type_value": "land",
    },
    {
        "url": "https://naciscdn.org/naturalearth/10m/physical/ne_10m_coastline.zip",
        "shapefile": "ne_10m_coastline.shp",
        "output": "coastline.parquet",
        "type_value": "coastline",
    },
    {
        "url": "https://naciscdn.org/naturalearth/10m/physical/ne_10m_lakes.zip",
        "shapefile": "ne_10m_lakes.shp",
        "output": "lake.parquet",
        "type_value": "lake",
    },
    {
        "url": "https://naciscdn.org/naturalearth/10m/physical/ne_10m_glaciated_areas.zip",
        "shapefile": "ne_10m_glaciated_areas.shp",
        "output": "glacier.parquet",
        "type_value": "glacier",
    },
    {
        "url": "https://naciscdn.org/naturalearth/10m/physical/ne_10m_rivers_lake_centerlines.zip",
        "shapefile": "ne_10m_rivers_lake_centerlines.shp",
        "output": "river.parquet",
        "type_value": "river",
    },
    {
        "url": "https://naciscdn.org/naturalearth/10m/cultural/ne_10m_admin_0_boundary_lines_land.zip",
        "shapefile": "ne_10m_admin_0_boundary_lines_land.shp",
        "output": "boundary.parquet",
        "type_value": "boundary",
    },
    {
        "url": "https://naciscdn.org/naturalearth/10m/cultural/ne_10m_populated_places_simple.zip",
        "shapefile": "ne_10m_populated_places_simple.shp",
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
    # Explode Multi* geometries into individual parts so each feature has
    # a tight bounding box.  Without this, world-spanning MultiPolygons
    # get dropped by the tiler's tile-span filter at higher zoom levels.
    gdf = gdf.explode(index_parts=False).reset_index(drop=True)

    n = len(gdf)

    geometry_wkb = [g.wkb for g in gdf.geometry]
    ids = [f"ne_{type_value}_{i}" for i in range(n)]
    types = [type_value] * n

    if "name" in gdf.columns:
        subtypes = [str(v) if v is not None else None for v in gdf["name"]]
    elif "NAME" in gdf.columns:
        subtypes = [str(v) if v is not None else None for v in gdf["NAME"]]
    else:
        subtypes = [None] * n

    bounds = gdf.geometry.bounds
    bbox_xmin = bounds["minx"].astype("float32").tolist()
    bbox_ymin = bounds["miny"].astype("float32").tolist()
    bbox_xmax = bounds["maxx"].astype("float32").tolist()
    bbox_ymax = bounds["maxy"].astype("float32").tolist()

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

    existing_meta = table.schema.metadata or {}
    existing_meta[b"geo"] = GEO_META.encode("utf-8")
    table = table.replace_schema_metadata(existing_meta)

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
