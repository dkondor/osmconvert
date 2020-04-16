/*
 * shp_output.hpp -- output OSM data in shapefile format
 * 
 * Copyright 2020 Daniel Kondor <kondor.dani@gmail.com>
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following disclaimer
 *   in the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name of the  nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * 
 */

#ifndef SHP_OUTPUT_H
#define SHP_OUTPUT_H

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ogr_core.h>
#include <ogrsf_frmts.h>
#include <ogr_feature.h>
#include <gdal_priv.h>

#include <osmium/osm/way.hpp>
#include <osmium/osm/node.hpp>

namespace OSMReader {

	/** Class to output OSM data as shapefile */
	struct shp_output {
			
		/** Set of interesting tags that are saved with the edges.
		 * Note: shapefiles only support a total of 254 bytes, so
		 * these might be truncated */
		const std::unordered_set<std::string> way_tags {"bridge", "tunnel", "oneway", "lanes", "ref", "name",
				"highway", "maxspeed", "service", "access", "area",
				"landuse", "width", "est_width", "junction", "surface", "oneway"};
		
		std::string current_way_ids;
		std::unordered_map<std::string, std::unordered_set<std::string> > current_way_tags;
		int dir = 0;
		osmium::unsigned_object_id_type first_node;
		osmium::unsigned_object_id_type last_node;
		int first_node_alt;
		int last_node_alt;
		bool use_alternate_id;
		bool have_first_node = false;
		
		OGRFeature* of = 0;
		OGRLineString* ls = 0;
		
		bool registered = false;
		GDALDriver* driver = 0;
		GDALDataset* data_source = 0;
		
		OGRSpatialReference wgs84;
		
		OGRLayer* layer_ways = 0;
		FILE* edges_ids_output = 0;
		long long int edge_id = 0;
		
		/** helper function to create a new field in a shapefile layer */
		static bool shp_create_field(OGRLayer* layer, const char* name, OGRFieldType type, int width);
		
		explicit shp_output(const char* base_dir, bool replace_ids_ = false, bool write_edge_ids = false);
		
		~shp_output();
		
		static void str_append_with_sep(std::string& s, const char* value, const char* sep = ", ");
		static void str_append_with_sep(std::string& s, const std::string& value, const char* sep = ", ");
		
		void begin_edge(int dir1);
		void new_way(const osmium::Way& way, int alt_id);
		void new_node(const osmium::Node& node, int alt_id);
		void end_edge();
	};
	
}

#endif


