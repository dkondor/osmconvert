/*
 * way_filter.cpp
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

#include "way_filter.hpp"

namespace OSMReader {

	/** test if the given way matches the filters */
	bool way_filter_tags::operator () (const osmium::Way& way) const {
		/* test include_tags first -- note that this is an O(n*m) operation where n is the size of include_tags, and 
		 * m is the number of tags the way has; since the size of include_tags will be typically small (1),
		 * this will not be a practical problem*/
		for(const auto& p : include_tags) {
			const char* value = way.get_value_by_key(p.first.c_str());
			if(!value) return false;
			if(p.second.size() && p.second.count(std::string(value)) == 0) return false;
		}
		/* test exclude_tags next -- iterate over all tags for the way for this */
		for(const osmium::Tag& tag : way.tags()) {
			const auto& it = exclude_tags.find(std::string(tag.key()));
			if(it != exclude_tags.end()) {
				if(it->second.count(std::string(tag.value())) > 0) return false;
			}
		}
		return true;
	}
	
	
	/** default filter for cycling in Singapore*/
	way_filter_tags way_filter_tags::default_bike_filter_sg() {
		way_filter_tags wf;
		/* way has to be highway */
		wf.include_tags.insert(std::make_pair(std::string("highway"),std::unordered_set<std::string>()));
		
		/* exclude ways that are impassable or prohibited for cycling */
		wf.exclude_tags.insert(std::make_pair(std::string("highway"),std::unordered_set<std::string>{"steps","corridor","motorway",
			"motorway_link","proposed","construction","abandoned","platform","raceway",
			"primary_link","secondary_link","trunk_link","tertiary_link"}));
		/* exclude ways whose surface is not suitable (note: some of these could be OK for MTB;
		 * this filter is meant for cycling as commute) */
		wf.exclude_tags.insert(std::make_pair(std::string("surface"),std::unordered_set<std::string>{"wood","unpaved","bricks","grass",
			"sand","dirt","gravel","ground","dirt/sand","grass_paver","mud","rocky","sett","pebblestone","pebblestones"}));
		/* exclude ways marking polygons */
		wf.exclude_tags.insert(std::make_pair(std::string("area"),std::unordered_set<std::string>{"yes"}));
		/* exclude ways with explicit prohibition for cycling */
		wf.exclude_tags.insert(std::make_pair(std::string("bicycle"),std::unordered_set<std::string>{"no"}));
		/* exclude pricate roads */
		wf.exclude_tags.insert(std::make_pair(std::string("service"),std::unordered_set<std::string>{"private"}));
		wf.exclude_tags.insert(std::make_pair(std::string("access"),std::unordered_set<std::string>{"private","no"}));
		
		/* Singapore specific exclude for roads that are not suitable for cycling, but are not filtered out by the previous */
		wf.exclude_tags.insert(std::make_pair(std::string("name"),std::unordered_set<std::string>{"Keppel Way","Laluan Tambak Johor–Singapura"}));
		
		return wf;
	}
	
	/** filter for roads */
	way_filter_tags way_filter_tags::default_road_filter() {
		way_filter_tags wf;
		/* must be highway */
		wf.include_tags.insert(std::make_pair(std::string("highway"),std::unordered_set<std::string>()));
		/* exclude ways unsuitable for cars */
		wf.exclude_tags.insert(std::make_pair(std::string("highway"),std::unordered_set<std::string>{"steps","corridor",
			"pedestrian","proposed","construction","abandoned","platform","raceway","track","bus_guideway","escape",
			"footway","bridleway","path","cycleway","elevator","platform"}));
		
		/* exclude unsuitable surface */
		wf.exclude_tags.insert(std::make_pair(std::string("surface"),std::unordered_set<std::string>{"wood","unpaved","bricks",
			"grass","sand","dirt","gravel","ground","dirt/sand","grass_paver","mud","rocky","sett","pebblestone","pebblestones"}));
		wf.exclude_tags.insert(std::make_pair(std::string("area"),std::unordered_set<std::string>{"yes"}));
		/* exclude private roads */
		wf.exclude_tags.insert(std::make_pair(std::string("service"),std::unordered_set<std::string>{"private"}));
		wf.exclude_tags.insert(std::make_pair(std::string("access"),std::unordered_set<std::string>{"private","no"}));
		
		return wf;
	}
		
	/** filter for roads with some additions for Singapore and Hong Kong / Shenzhen 
	 * 
	 * note: border crossings from Singapore to Malaysia and between Hong Kong and Shenzhen are
	 * explicitely excluded; this way, the OSM network can be cut to one of these cities base
	 * on finding the largest connected component in a box centered on it */
	way_filter_tags way_filter_tags::default_road_filter_sg_hk_sz() {
		way_filter_tags wf = default_road_filter();
		/* for Shenzhen, exclude the connection to Hong Kong */
		/* for Singapore, exclude connections to Malaysia */
		wf.exclude_tags.insert(std::make_pair(std::string("name"),std::unordered_set<std::string>{"落馬洲橋 Lok Ma Chau Birdge",
			"文錦渡路 Man Kam To Road","深圳灣公路大橋 Shenzhen Bay Bridge","沙河路 Sha Ho Road","Laluan Tambak Johor–Singapura",
			"Laluan Kedua Malaysia–Singapura","羅芳橋  Lo Fong Bridge"}));
		/* note: Lo Fong Bridge does not have name:en tag */
		wf.exclude_tags.insert(std::make_pair(std::string("name:en"),std::unordered_set<std::string>{"Lok Ma Chau Birdge",
			"Man Kam To Road","Shenzhen Bay Bridge","Sha Ho Road","Johor–Singapore Causeway",
			"Malaysia–Singapore Second Link"}));
				
		return wf;
	}
	
}

