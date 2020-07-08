/*
 * osmreader.cpp
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


#include "osmreader.hpp"

namespace OSMReader {
	int OSMMap::is_oneway(const osmium::Way& way) {
		int oneway = 0;
		const char* value = way.get_value_by_key("oneway");
		if(value) {
			if(!strcmp(value,"yes") || !strcmp(value,"1") || !strcmp(value,"true")) oneway = 1;
			if(!strcmp(value,"-1")) oneway = -1;
		}
		if(!oneway) {
			value = way.get_value_by_key("highway");
			if(value && !strcmp(value,"motorway")) oneway = 1;
			else {
				value = way.get_value_by_key("junction");
				if(value && !strcmp(value,"roundabout")) oneway = 1;
			}
		}
		return oneway;
	}
	
	/** Function to handle a node read from OSM data (part of the osmium::handler::Handler interface) */
	void OSMMap::node(const osmium::Node& node) {
		auto id = node.id();
		auto handle = istash.add_item(node);
		nodes[node.id()] = handle;
		if(replace_ids) get_id(id); /* adds ID to replacement map if not there already */
	}
	
	/** Function to handle a way read from OSM data (part of the osmium::handler::Handler interface) */
	void OSMMap::way(const osmium::Way& way) {
		if(!wf(way)) return;
		/* check that the way has more than 1 node */
		if(way.nodes().size() < 2) return;
		/* store full way in istash */
		osmium::unsigned_object_id_type way_id = way.id();
		auto handle = istash.add_item(way);
		ways[way_id] = handle;
		
		/* optionally add the ID of this way to the replacement map */
		if(replace_ids) get_id(way_id);
		
		/* check the direction of this way */
		int dir = ignore_dir ? 0 : is_oneway(way);
		
		/* process all nodes in this way */
		osmium::unsigned_object_id_type last_node_id = 18446744073709551615UL;
		for(auto& nr : way.nodes()) {
			auto node_id = nr.ref();
			if(last_node_id != 18446744073709551615UL) {
				/* add all links to the graph */
				node_graph[last_node_id].insert(std::make_pair(node_id,std::make_pair(way_id,dir)));
				node_graph[node_id].insert(std::make_pair(last_node_id,std::make_pair(way_id,-1*dir)));
			}
			last_node_id = node_id;
		}
	}
	
	/** find connected components considering an undirected graph */
	std::pair<unsigned int, unsigned int> OSMMap::find_sccs() {
		sccs.clear();
		sccs_size.clear();
		std::unordered_set<osmium::unsigned_object_id_type> nodes_to_add;
		std::unordered_set<osmium::unsigned_object_id_type> nodes_to_add2;
		unsigned int sccid = 0;
		unsigned int scc_size = 0;
		for(const auto& x : node_graph) {
			auto n1 = x.first;
			if(sccs.count(n1)) continue;
			nodes_to_add.insert(n1);
			do {
				for(auto n2 : nodes_to_add) {
					auto r = sccs.insert(std::make_pair(n2,sccid));
					if(r.second) {
						for(const auto& y : node_graph.at(n2)) nodes_to_add2.insert(y.first);
						scc_size++;
					}
					else if(r.first->second != sccid) throw std::runtime_error("OSMMap::find_sccs(): node already present in different component!\n");
				}
				nodes_to_add.clear();
				nodes_to_add.swap(nodes_to_add2);
			} while(nodes_to_add.size());
			sccs_size.push_back(std::make_pair(sccid,scc_size));
			sccid++;
			scc_size = 0;
		}
		std::sort(sccs_size.begin(),sccs_size.end(),[](const auto& x, const auto& y) { return x.second > y.second; });
		return sccs_size[0];
	}
	
	/** Find connected components in the directed graph.
	 * 
	 * Note: in the worst case scenario (long one-way chain),
	 * this can have O(n^2) runtime -- for a map, we don't expect
	 * this, since nodes should be reachable from almost everywhere.
	 * 
	 * Returns the size and ID of the largest connected component.
	 */
	std::pair<unsigned int, unsigned int> OSMMap::find_sccs_dir() {
		sccs.clear();
		sccs_size.clear();
		
		unsigned int sccid = 0;
		unsigned int scc_size = 0;
		
		size_t nnodes = 0;
		for(const auto& x : node_graph) {
			auto n1 = x.first;
			nnodes++;
			if(sccs.count(n1)) continue;
			std::unordered_set<osmium::unsigned_object_id_type> out_nodes; /* nodes reachable from the current */
			std::unordered_set<osmium::unsigned_object_id_type> in_nodes; /* nodes the current one is reachable from */
			
			std::unordered_set<osmium::unsigned_object_id_type> in_nodes_add;
			std::unordered_set<osmium::unsigned_object_id_type> in_nodes_add2;
			std::unordered_set<osmium::unsigned_object_id_type> out_nodes_add;
			std::unordered_set<osmium::unsigned_object_id_type> out_nodes_add2;
			
			in_nodes_add.insert(n1);
			out_nodes_add.insert(n1);
			
			do {
				for(auto n2 : out_nodes_add) {
					auto r = out_nodes.insert(n2);
					if(r.second) for(const auto& y : node_graph.at(n2)) {
						auto n3 = y.first;
						int dir = y.second.second;
						if(dir >= 0 && sccs.count(n3) == 0) out_nodes_add2.insert(n3);
					}
				}
				for(auto n2 : in_nodes_add) {
					auto r = in_nodes.insert(n2);
					if(r.second) for(const auto& y : node_graph.at(n2)) {
						auto n3 = y.first;
						int dir = y.second.second;
						if(dir <= 0 && sccs.count(n3) == 0) in_nodes_add2.insert(n3);
					}
				}
				in_nodes_add.clear();
				in_nodes_add.swap(in_nodes_add2);
				out_nodes_add.clear();
				out_nodes_add.swap(out_nodes_add2);
			} while(out_nodes_add.size() || in_nodes_add.size());
			
			/* elements in the scc are the intersection of out_nodes and in_nodes */
			for(auto n : out_nodes) if(in_nodes.count(n)) {
				sccs[n] = sccid;
				scc_size++;
			}
			sccs_size.push_back(std::make_pair(sccid,scc_size));
			sccid++;
			scc_size = 0;
			
			//~ fprintf(stderr,"Nodes processed: %lu, sccs found: %u\n",nnodes,sccid);
		}
		std::sort(sccs_size.begin(),sccs_size.end(),[](const auto& x, const auto& y) { return x.second > y.second; });
		return sccs_size[0];
	}	
			
	
	static const double RADIUS = 6371000.0;

	static inline double dist_m(double lon1, double lat1, double lon2, double lat2) {
		lon1 = M_PI * lon1 / 180.0;
		lat1 = M_PI * lat1 / 180.0;
		lon2 = M_PI * lon2 / 180.0;
		lat2 = M_PI * lat2 / 180.0;

		double s1 = sin ((lat1 - lat2) / 2.0);
		double s2 = sin ((lon1 - lon2) / 2.0);
		double r1 = s1 * s1 + s2 * s2 * cos (lat1) * cos (lat2);
		return RADIUS * 2.0 * asin (sqrt (r1));
	}

	static inline double dist(const osmium::Location& x, const osmium::Location& y) {
		return dist_m(x.lon(),x.lat(),y.lon(),y.lat());
	}

			
	/** start of a new edge */
	void OSMMap::tsv_output::begin_edge(int dir1) {
		dir = dir1;
		have_first_node = false;
		total_dist = 0.0;
		total_time = 0.0;
	}
	/** new way -- here, we only care about the travel speed on it */
	void OSMMap::tsv_output::new_way(const osmium::Way& way, int alternate_id) {
		if(sc) current_speed = get_segment_speed(sc,way);
	}
	/** new node -- calculate distance along the edge here */
	void OSMMap::tsv_output::new_node(const osmium::Node& node, int alternate_id) {
		osmium::unsigned_object_id_type id = node.id();
		auto loc = node.location();
		if(have_first_node) {
			double d1 = dist(last_loc,loc);
			total_dist += d1;
			if(sc) total_time += d1/current_speed;
		}
		if(!have_first_node) {
			first_node = id;
			first_node_alt = alternate_id;
			have_first_node = true;
			if(nodes_seen.insert(first_node).second) {
				if(use_alternate_id) fprintf(out_nodes,"%d",first_node_alt);
				else fprintf(out_nodes,"%lu",first_node);
				fprintf(out_nodes,"\t%f\t%f\n",loc.lon(),loc.lat());
			}
		}
		last_node = id;
		last_node_alt = alternate_id;
		last_loc = loc;
	}
	/** end of an edge, output it */
	void OSMMap::tsv_output::end_edge() {
		if(dir >= 0) {
			/* output in forward direction */
			if(use_alternate_id) fprintf(out_edges,"%d\t%d",first_node_alt,last_node_alt);
			else fprintf(out_edges,"%lu\t%lu",first_node,last_node);
			fprintf(out_edges,"\t%f",total_dist);
			if(sc) fprintf(out_edges,"\t%f\n",total_time);
			else fputc('\n',out_edges);
		}
		if(dir <= 0) {
			/* output in backward direction */
			if(use_alternate_id) fprintf(out_edges,"%d\t%d",last_node_alt,first_node_alt);
			else fprintf(out_edges,"%lu\t%lu",last_node,first_node);
			fprintf(out_edges,"\t%f",total_dist);
			if(sc) fprintf(out_edges,"\t%f\n",total_time);
			else fputc('\n',out_edges);
		}
		if(nodes_seen.insert(last_node).second) {
			/* output last node coordinates in case we haven't seen it yet */
			if(use_alternate_id) fprintf(out_nodes,"%d",last_node_alt);
			else fprintf(out_nodes,"%lu",last_node);
			fprintf(out_nodes,"\t%f\t%f\n",last_loc.lon(),last_loc.lat());
		}
	}
	
	/** Helper function to determine to speed limit on an OSM Way */
	double OSMMap::tsv_output::get_segment_speed(const speed_converter* sc, const osmium::Way& way) {
		const char* s1 = way.get_value_by_key("maxspeed");
		double r = -1.0;
		if(s1) {
			char* s2;
			r = strtod(s1,&s2);
			if(s1 != s2) {
				bool found = true;
				for(;*s2;++s2) if( ! (*s2 == ' ' || *s2 == '\t' || *s2 == '\n' || *s2 == '\r') ) { found = false; break; }
				if(found) return r;
			}
		}
		return (*sc)( way.get_value_by_key("highway") );
	}
	
	OSMMap::tsv_output::speed_converter OSMMap::tsv_output::default_speeds_sg() {
		speed_converter sc;
		sc.default_speed = 50.0;
		sc.speed_dict.insert(std::make_pair(std::string("motorway"),90.0));
		sc.speed_dict.insert(std::make_pair(std::string("motorway_link"),50.0));
		sc.speed_dict.insert(std::make_pair(std::string("primary"),60.0));
		sc.speed_dict.insert(std::make_pair(std::string("service"),40.0));
		sc.speed_dict.insert(std::make_pair(std::string("trunk"),70.0));
		return sc;
	}
	
	
	double OSMMap::tsv_output::speed_converter::operator()(const std::string& type) const {
		auto it = speed_dict.find(type);
		if(it != speed_dict.end()) return it->second;
		else return default_speed;
	}
}


