/*
 * way_filter.hpp -- filte OSM ways according to attributes
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


#ifndef WAY_FILTER_H
#define WAY_FILTER_H

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <osmium/osm/way.hpp>

namespace OSMReader {

/** generic interface for filtering ways */
class way_filter {
	public:
		virtual bool operator () (const osmium::Way& way) const = 0;
	protected:
		~way_filter() { }
};

/** specific implementation using maps of tags to include / exclude */
class way_filter_tags : public way_filter {
	protected:
		/** key -- value pairs; if any of the keys is present with any of the corresponding values, this item is skipped */
		std::unordered_map<std::string, std::unordered_set<std::string> > exclude_tags;
		/** key -- value pairs; all of the keys here need to be present with any of the values given -- if no value given, the key
		 * has to be present with any value */
		std::unordered_map<std::string, std::unordered_set<std::string> > include_tags;
	
	public:
		/** test if the given way matches the filters */
		bool operator () (const osmium::Way& way) const override;
		
		/** default filter for cycling in Singapore*/
		static way_filter_tags default_bike_filter_sg();
		
		/** filter for roads */
		static way_filter_tags default_road_filter();
		
		/** filter equivalent to osmnx's drive_service filter */
		static way_filter_tags road_filter_drive_service();
		
		/** add filter for roads for border crossings for Singapore and Hong Kong / Shenzhen 
		 * 
		 * note: border crossings from Singapore to Malaysia and between Hong Kong and Shenzhen are
		 * explicitely excluded; this way, the OSM network can be cut to one of these cities based
		 * on finding the largest connected component in a box centered on it */
		static void filter_sg_hk_sz(way_filter_tags& wf);
};

}

#endif

