/*
 * shp_output.cpp
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


#include "shp_output.hpp"

namespace OSMReader {

	/** helper function to create a new field in a shapefile layer */
	bool shp_output::shp_create_field(OGRLayer* layer, const char* name, OGRFieldType type, int width) {
		OGRFieldDefn field(name, type);
		if(width > 0) field.SetWidth(width);
		if(layer->CreateField(&field) != OGRERR_NONE) return false;
		return true;
	}
	
	shp_output::shp_output(const char* base_dir, bool replace_ids_, bool write_edge_ids):use_alternate_id(replace_ids_) {
		OGRRegisterAll();
		registered = true;
		driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
		if(!driver) throw std::runtime_error("shp_output: Error getting GDAL driver!\n");
		data_source = driver->Create(base_dir,0,0,0,GDT_Unknown,0);
		if(!data_source) throw std::runtime_error("shp_output: Error creating data source!\n");
		wgs84.SetWellKnownGeogCS("WGS84");
		char o1[] = "OVERWRITE=YES";
		char o2[] = "ENCODING=UTF-8";
		char* options[] = {o1,o2,0};
		
		layer_ways = data_source->CreateLayer("ways",&wgs84,wkbLineString,options);
		if(!layer_ways) throw std::runtime_error("shp_output: Error creating layers!\n");
		
		/* note: all fields are lists, so attributes from multiple ways can be stored
		 * also, shapefile ony supports string list, so use that for way IDs */
		if(!shp_create_field(layer_ways,"way_ids",OFTString,254))
			throw std::runtime_error("shp_output: Error creating fields!\n");
		if(write_edge_ids) if(!shp_create_field(layer_ways,"id",OFTInteger64,0))
			throw std::runtime_error("shp_output: Error creating fields!\n");
		for(const std::string& tag : way_tags) if(!shp_create_field(layer_ways,tag.c_str(),OFTString,254))
			throw std::runtime_error("shp_output: Error creating fields!\n");
		/* create field for to / from IDs */
		if(use_alternate_id) {
			if(!(shp_create_field(layer_ways,"from",OFTInteger,0) && shp_create_field(layer_ways,"to",OFTInteger,0)))
				throw std::runtime_error("shp_output: Error creating fields!\n");
		}
		else if(!(shp_create_field(layer_ways,"from",OFTInteger64,0) && shp_create_field(layer_ways,"to",OFTInteger64,0)))
			throw std::runtime_error("shp_output: Error creating fields!\n");
			
		/* create file to write out matching between newly assigned edge IDs and edge endpoints */
		if(write_edge_ids) {
			char* tmp = (char*)malloc(sizeof(char)*(strlen(base_dir) + 20));
			if(!tmp) throw std::runtime_error("shp_output: Cannot allocate memory!\n");
			sprintf(tmp,"%s/edge_ids.csv",base_dir);
			edges_ids_output = fopen(tmp,"w");
			free(tmp);
			if(!edges_ids_output) throw std::runtime_error("shp_output: Cannot open output file!\n");
		}
	}
	
	shp_output::~shp_output() {
		if(ls) delete ls;
		if(of) OGRFeature::DestroyFeature(of);
		if(data_source) GDALClose(data_source);
		if(registered) OGRCleanupAll();
		if(edges_ids_output) fclose(edges_ids_output);
	}
	
	void shp_output::str_append_with_sep(std::string& s, const char* value, const char* sep) {
		if(s.size()) s.append(sep);
		s.append(value);
	}
	void shp_output::str_append_with_sep(std::string& s, const std::string& value, const char* sep) {
		if(s.size()) s.append(sep);
		s.append(value);
	}
	
	void shp_output::begin_edge(int dir1) {
		dir = dir1;
		have_first_node = false;
		of = OGRFeature::CreateFeature(layer_ways->GetLayerDefn());
		ls = new OGRLineString();
		if( ! (ls && of) ) throw std::runtime_error("shp_output::begin_edge(): Error creating new feature!\n");
	}
	void shp_output::new_way(const osmium::Way& way, int alt_id) {
		str_append_with_sep(current_way_ids, std::to_string(way.id()) );
		for(const osmium::Tag& tag : way.tags()) {
			std::string key(tag.key());
			if(way_tags.count(key) > 0) current_way_tags[key].insert(std::string(tag.value()));
		}
	}
	void shp_output::new_node(const osmium::Node& node, int alt_id) {
		osmium::unsigned_object_id_type id = node.id();
		if(!have_first_node) {
			first_node = id;
			first_node_alt = alt_id;
			have_first_node = true;
		}
		last_node = id;
		last_node_alt = alt_id;
		auto loc = node.location();
		ls->addPoint(loc.lon(),loc.lat());
	}
	void shp_output::end_edge() {
		/* create reverse linestring and feature if needed */
		OGRFeature* ofr = 0;
		OGRLineString* lsr = 0;
		std::vector<std::pair<const char*, std::string> > tags;
		tags.reserve(current_way_tags.size());
		for(const auto& x : current_way_tags) {
			std::string y;
			for(const auto& z : x.second) str_append_with_sep(y,z);
			tags.push_back(std::pair<const char*, std::string>(x.first.c_str(),std::move(y)));
		}
		
		bool err = false;
		if(dir == -1) {
			lsr = ls;
			lsr->reversePoints();
			ofr = of;
			ls = 0;
			of = 0;
		}
		if(dir == 0) {
			lsr = dynamic_cast<OGRLineString*>(ls->clone());
			ofr = OGRFeature::CreateFeature(layer_ways->GetLayerDefn());
			if( ! (lsr && ofr) ) { err = true; goto end_edge_end; }
			lsr->reversePoints();
		}
		
		if(dir >= 0) {
			if(of->SetGeometryDirectly(ls) != OGRERR_NONE) { err = true; goto end_edge_end; }
			ls = 0; /* note: ls is owned by of now, so setting it 0 will avoid calling delete on it later */
			/* set fields */
			of->SetField("way_ids",current_way_ids.c_str());
			for(const auto& x : tags) of->SetField(x.first,x.second.c_str());
			
			if(use_alternate_id) {
				of->SetField("from",first_node_alt);
				of->SetField("to",last_node_alt);
			}
			else {
				of->SetField("from",(long long int)first_node);
				of->SetField("to",(long long int)last_node);
			}
			if(edges_ids_output) {
				of->SetField("id",edge_id);
				fprintf(edges_ids_output,"%lld,%lu,%lu",edge_id,first_node,last_node);
				if(use_alternate_id) fprintf(edges_ids_output,",%d,%d\n",first_node_alt,last_node_alt);
				else fputc('\n',edges_ids_output);
				edge_id++;
			}
			if(layer_ways->CreateFeature(of) != OGRERR_NONE) { err = true; goto end_edge_end; }
		}
		if(dir <= 0) {
			if(ofr->SetGeometryDirectly(lsr) != OGRERR_NONE) { err = true; goto end_edge_end; }
			lsr = 0; /* note: lsr is owned by ofr now, so setting it 0 will avoid calling delete on it later */
			ofr->SetField("way_ids",current_way_ids.c_str());
			for(const auto& x : tags) ofr->SetField(x.first,x.second.c_str());
			
			if(use_alternate_id) {
				ofr->SetField("from",last_node_alt);
				ofr->SetField("to",first_node_alt);
			}
			else {
				ofr->SetField("from",(long long int)last_node);
				ofr->SetField("to",(long long int)first_node);
			}
			if(edges_ids_output) {
				ofr->SetField("id",edge_id);
				fprintf(edges_ids_output,"%lld,%lu,%lu",edge_id,last_node,first_node);
				if(use_alternate_id) fprintf(edges_ids_output,",%d,%d\n",last_node_alt,first_node_alt);
				else fputc('\n',edges_ids_output);
				edge_id++;
			}
			if(layer_ways->CreateFeature(ofr) != OGRERR_NONE) { err = true; goto end_edge_end; }
		}
end_edge_end:
		if(of) { OGRFeature::DestroyFeature(of); of = 0; }
		if(ofr) { OGRFeature::DestroyFeature(ofr); ofr = 0; }
		if(ls) { delete ls; ls = 0; }
		if(lsr) { delete lsr; lsr = 0; }
		current_way_ids.clear();
		current_way_tags.clear();
		if(err) throw std::runtime_error("simplified_shp_output::end_edge(): Error creating features in layer!\n");
	}
	
}

