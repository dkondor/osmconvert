# osmconvert

Program to convert OpenStreetMap Data to TSV or Shapefile, creating a simplified graph.

## Main functionality

This program reads OSM data in either XML or PBF format and creates a directed graph from it where nodes correspond to intersections and edges are connections between them. In this process, ways are filtered (to only include relevant ones) and split or merged to form meaningful edges. Practically, this means creating a graph from OSM nodes and then removing all nodes whose degree is exactly 2.

Output can be written in text files or as a shapefile. In the latter case, the original geometry of edges is preserved and the start and end node of each edge is saved as an additional attribute.

The resulting graph can be used e.g. to perform routing in a more convenient maaner than working with the original data.

## Requirements

To use this program, you will need to compile with a C++11 compatible compiler. For reading OSM data, it also depends on the [Osmium](https://osmcode.org/libosmium/) library. Optionally, shapefile output depends on the [GDAL](https://gdal.org/) library.

On Ubuntu, these dependencies can be satisfied by installing the packages ``libosmium2-dev`` and ``libgdal-dev``. Other distributions will likely have similarly named packages.

## Compiling

Example compilation with gcc on Ubuntu:

``g++ -o osmconvert osmconvert.cpp osmreader.cpp way_filter.cpp shp_output.cpp -I/usr/include/gdal -O3 -lm -lexpat -lz -lbz2 -lgdal -pthread``

Alternatively, if shapefile output is not needed:

``g++ -o osmconvert osmconvert.cpp osmreader.cpp way_filter.cpp -O3 -lm -lexpat -lz -lbz2 -pthread -DNO_SHP``

## Running

Example:

``./osmc -i Singapore.osm.pbf -s -t -r -e -C -S -o osm_sg_new``

Description of command line options:

| Option | Meaning |
| ------ | ------- |
| `-i`   | Input file name (required). Can be PBF or OSM XML. |
| `-o`   | Output base file name (required). For text output, two files will be created with the suffixes ``_nodes.dat`` and ``_edges.dat`` appended for the file containing nodes and edges respectively. For shapefile output, a directory with this name will be create with the file ``ways.shp`` in it. |
| `-s`   | If given, shapefile output is produced. It is an error if a shapefile with the same name already exists. |
| `-t`   | If given, text output is produced. Any files with the same name are overwritten. |
| `-r`   | If given, OSM node and way IDs in the output are replaced by smaller integer IDs. This is mainly useful if you would like to process the output with a program that does not accept 64-bit numbers. In this case, the maximum number of OSM ways and nodes to be processed is 2^31-1. |
| `-e`   | Only applies to shapefile output. If given, each edge in the shapefile is given a unique ID and an extra file, ``edge_ids.csv`` is created in the output folder that lists edge IDs and the corresponding OSM nodes. |
| `-b`   | If given, the output is a network for cycling. This means removing motorways and adding paths that might be suitable for cycling. See the code in ``way_filter.cpp`` for explanation of the tags included / excluded. |
| `-d`   | If given, the output is a road network which should be the same as the network produced by the [osmnx downloader](https://osmnx.readthedocs.io/en/stable/osmnx.html#osmnx.core.graph_from_place) using the [drive+service](https://github.com/gboeing/osmnx/blob/master/osmnx/downloader.py#L47) filter.
| `-C`   | If give, only the largest connected component is included in the output. Connected components are calculated based on a directed graph, so the result will be fully routable. |
| `-c`   | Like `-C`, but the connected components are calculated assuming an undirected graph. |
| `-S`   | If given, use an extra filter to specifically exclude roads that connect Singapore and Malaysia and roads that connect Hong Kong and Shenzhen. Combined with the `-C` switch, this can be useful to quickly cut a map centered on Singapore, Hong Kong or Shenzhen to exclude the network on the other side of the border. See the code in ``way_filter.cpp`` for a list of roads explicitely excluded by this option. |

## Output

Shapefile output includes one linestring feature for each edge in the processed network. The fields ``from`` and ``to`` are added and contain the OSM IDs of the nodes at the start and end of the edge (with the `-r` command line option, these are replaced by smaller integer IDs). OSM IDs and interesting tags for each way that is included in an edge are included as fields. Note that these will likely be truncated if an edge is made up of many OSM ways. With the `-e` option, each edge will have an `id` field which is a unique integer. Edges are considered oneway, corresponding to the direction of traffic (i.e. traffic flows from the ``from`` node to the ``to`` node). Bidirectional road segments are included as two separate edges, once for each direction.

Text output is written to two separate files: ``{}_nodes.dat`` contains a list of nodes with coordinates (i.e. columns are OSM node ID, longitude, latitude). Only nodes that are part of the graph (have degree != 2) are included. A separate file, ``{}_edges.dat`` contains a list of directed edges. Columns are the two node IDs (direction of an edge is from the first to the second ID), distance (in meters) and travel time (in seconds, based on an estimate of the speed limits along the way -- note that speed limits are based on the typical values in Singapore, these can be different for other cities). Again, all edges are considered directed; for bidirectional road segments, two edges will be included in the output, corresponding to both directions. If the ``-r`` option is given, all OSM node IDs are replaced with smaller integers.


## Limitations

 - There is not an option to download data directly from OpenStreetMap. To use this program, you have to download an OSM map file manually first.
 - It is not possible to cut a map to a specific boundary, all ways and nodes in the input file are processed. It is only possible to filter for the largest connected component. It is possible to manually exclude segments by modifying the filters in ``way_filter.cpp``; an example is provided for the cases of Singapore, Hong Kong and Shenzhen, where the border crossings are explicitely filtered out.
 - OSM tags in shapefile output are only saved for ways and might get truncated if the total length is above 254 bytes. In the text output, no metadata is included.






