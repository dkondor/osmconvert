/*
 * osmreader.hpp -- read osm data and generate a network from it directly
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
 
#ifndef OSM_READER_H
#define OSM_READER_H 

#include <stdio.h>
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <iostream>
#include <fstream>
#include <iomanip>		
#include <type_traits>
#include <limits.h>

#include <osmium/index/map/sparse_mem_array.hpp>
#include <osmium/handler.hpp>
#include <osmium/storage/item_stash.hpp>
#include <osmium/visitor.hpp>

#include "way_filter.hpp"


namespace OSMReader {
/** 
 * Class for to read and process OpenStreetMap road network
 */
class OSMMap : public osmium::handler::Handler {
	protected:
		osmium::ItemStash istash; /**< Store nodes and ways here during processing */
		std::unordered_map<osmium::unsigned_object_id_type,osmium::ItemStash::handle_type> nodes; /**< Mapping for node IDs in istash */
		std::unordered_map<osmium::unsigned_object_id_type,osmium::ItemStash::handle_type> ways; /**< Mapping for way IDs in istash */
		
		/** Store the network in a graph format for processing.
		 * For each edge, it stores the ID of the OSM way it is part of,
		 * and a flag about direction (1: forward, 0: both ways, -1: backward) */
		std::unordered_map<osmium::unsigned_object_id_type,
			std::unordered_map<osmium::unsigned_object_id_type, 
				std::pair<osmium::unsigned_object_id_type, int > > > node_graph;
		/** filter used to select which ways to include */
		const way_filter& wf;
		
		/** Optional: find connected components, store the mapping from node IDs here */
		std::unordered_map<osmium::unsigned_object_id_type, unsigned int> sccs;
		/** Size of connected components found, to be used to select the largest.
		 * Always sorted in decreasing order. */
		std::vector<std::pair<unsigned int,unsigned int> > sccs_size;
		
		/** facility to replace OSM IDs with smaller numeric IDs */
		std::unordered_map<osmium::unsigned_object_id_type,int> ids_replace;
		int max_id; /**< Next free ID to assign */
		const bool replace_ids; /**< Flag whether IDs are replaced with smaller integers */
		/** Convenience wrapper to find a replacement ID */
		int get_id(osmium::unsigned_object_id_type id) {
			auto r = ids_replace.insert(std::make_pair(id,max_id));
			if(r.second) {
				if(max_id == INT_MAX) throw std::runtime_error("OSMMap: too many object IDs!\n");
				max_id++;
			}
			return r.first->second;
		}
		/** Convenience wrapper to find a replacement ID -- const version, throws an exception if the ID was not assigned a replacement previously. */
		int get_id(osmium::unsigned_object_id_type id) const { return ids_replace.at(id); }
		
		/** Function to determine if an OSM way's direction. Returns 0 for two-way, 1 for one-way following the nodes' order, -1 for reversed case. */
		static int is_oneway(const osmium::Way& way);
		
		/** Function to determine if an OSM way's direction, based on it's OSM ID. Returns 0 for two-way, 1 for one-way following the nodes' order, -1 for reversed case. 
		 * !!NOT USED!! */
		/* int is_oneway(osmium::unsigned_object_id_type way_id) const { return is_oneway(istash.get<osmium::Way>(ways.at(way_id))); } */
		
	public:
		/** Initialize a new instance based on the way_filter provided */
		explicit OSMMap(const way_filter& wf_, bool replace_ids_ = false) : wf(wf_), max_id(0), replace_ids(replace_ids_), ignore_dir(false) { }
		
		/** Flag whether to ignore direction of way. If set, all ways are addded as bidirectional edges. Can be useful when creating cycling network or if oneway tags are not reliable. */
		bool ignore_dir;
		
		/** Function to handle a node read from OSM data (part of the osmium::handler::Handler interface) */
		void node(const osmium::Node& node);
		
		/** Function to handle a way read from OSM data (part of the osmium::handler::Handler interface) */
		void way(const osmium::Way& way);		
		
		/** Function that generates a simplified graph.
		 * 
		 * Result is returned by calling the callback functions of the
		 * given output_class object. This should define the following methods:
		 * 
		 * begin_edge(int dir) -- called when a new edge is started to be processed
		 * new_way(const osmium::Way& w, int alternate_id) -- called when a new OSM Way object is encountered while travering an adge
		 * new_node(const osmium::Node& n, int alternate_id) -- called in sequence for each OSM node in the edge
		 * end_edge() -- called when finishing processing for each edge (i.e. after all OSM ways and nodes have been added
		 * 
		 * note: the alternate id parameter is only used if replace_ids == true,
		 * in this case it is a unique integer for each node and way, otherwise it is always zero
		 * 
		 * One edge is defined by a path between any two OSM nodes with degree != 2.
		 * Typically, an edge contains multiple OSM nodes which have the purpose
		 * of determining the correct geometry of the edge.
		 * Also, there is not a one-to-one correspondence between edges and OSM ways:
		 * an edge can contain multiple OSM ways (e.g. if some property changes on the way),
		 * and an OSM way can belong to multiple edge (an OSM way can be created for a road
		 * that spans multiple intersections). This function identifies the edges and notifies
		 * the callback function of all the nodes (points) that belong to it and all the OSM
		 * ways that are included in it.
		 * 
		 * The sccid parameter determines which connected component to include in the output.
		 * This only has effect if the find_sccs() or find_sccs_dir() function has been called before.
		 */
		template<class output_class>
		void generate_simplified_graph(output_class& out, unsigned int sccid = 0) const;
		
		/** Example interface for generate_simplified_graph(): output graph in a TSV file */
		struct tsv_output {
			
			/** Helper struct to find out speed limits on road segments */
			struct speed_converter {
				std::unordered_map<std::string,double> speed_dict;
				double default_speed;
				speed_converter():default_speed(50.0) { }
				double operator()(const std::string& type) const;
				double operator()(const char* type) const { return operator()(std::string(type)); }
			};

			
			FILE* out_nodes;
			FILE* out_edges;
			int dir;
			const speed_converter* sc;
			osmium::unsigned_object_id_type first_node;
			osmium::unsigned_object_id_type last_node;
			int first_node_alt;
			int last_node_alt;
			double total_dist;
			double total_time;
			double current_speed;
			osmium::Location last_loc;
			bool have_first_node;
			bool use_alternate_id;
			
			std::unordered_set<osmium::unsigned_object_id_type> nodes_seen;
			explicit tsv_output(FILE* out_nodes_, FILE* out_edges_,
					bool use_alternate_id_ = false, const speed_converter* sc_ = 0) :
				out_nodes(out_nodes_), out_edges(out_edges_), sc(sc_), use_alternate_id(use_alternate_id_) { }
			
			/** start of a new edge */
			void begin_edge(int dir1);
			/** new way -- here, we only care about the travel speed on it */
			void new_way(const osmium::Way& way, int alternate_id);
			/** new node -- calculate distance along the edge here */
			void new_node(const osmium::Node& node, int alternate_id);
			/** end of an edge, output it */
			void end_edge();
			
			/** Helper function to determine to speed limit on an OSM Way */
			static double get_segment_speed(const speed_converter* sc, const osmium::Way& way);
			
			/** Create speed limit converter with defaults for Singapore */
			static speed_converter default_speeds_sg();
		};
		
		/** Save OSM data in TSV format (separate files for nodes and edges) */
		void save_tsv(FILE* out_nodes, FILE* out_edges, unsigned int sccid = 0, const tsv_output::speed_converter* sc = 0) const {
			tsv_output out1(out_nodes, out_edges, replace_ids, sc);
			generate_simplified_graph(out1, sccid);
		}
		
		/** find connected components considering an undirected graph */
		std::pair<unsigned int, unsigned int> find_sccs();
		
		/** Find connected components in the directed graph.
		 * 
		 * Note: in the worst case scenario (long one-way chain),
		 * this can have O(n^2) runtime -- for a map, we don't expect
		 * this, since nodes should be reachable from almost everywhere.
		 * 
		 * Returns the size of the largest connected component.
		 */
		std::pair<unsigned int, unsigned int> find_sccs_dir();
		
};



template<class output_class>
void OSMMap::generate_simplified_graph(output_class& out, unsigned int sccid) const {
	std::unordered_set<osmium::unsigned_object_id_type> nodes_seen;
	std::unordered_set<osmium::unsigned_object_id_type> extra_start;
	
	std::unordered_set<osmium::unsigned_object_id_type> ways_seen;
	
	/* two passes of the main loop, 2nd pass needed only if oneway == true for intermediate nodes */
	for(int i=0;i<2;i++) {
		auto loop_it1 = node_graph.begin();
		while(true) {
			std::unordered_map<osmium::unsigned_object_id_type,
				std::unordered_map<osmium::unsigned_object_id_type,
				std::pair<osmium::unsigned_object_id_type,int> > >::const_iterator node_it;
			
			if(i == 0) {
				if(loop_it1 == node_graph.end()) break;
				node_it = loop_it1;
				++loop_it1;
				if( !(node_it->second.size() == 1 || node_it->second.size() > 2) ) continue;
			}
			else {
				auto loop_it2 = extra_start.begin();
				if(loop_it2 == extra_start.end()) break;
				node_it = node_graph.find(*loop_it2);
				if(node_it == node_graph.end()) throw std::runtime_error("OSMMap::save_tsv(): node not found!\n");
				extra_start.erase(loop_it2);
			}
		
			const auto& n = *node_it;
			osmium::unsigned_object_id_type n1 = n.first;
			if(sccs.size()) if(sccs.at(n1) != sccid) continue;
			
			/* check all possible paths going out from this node */
			for(const auto& ps : n.second) {
				/* edge properties */
				osmium::unsigned_object_id_type last_node = n1; /* from node */
				osmium::unsigned_object_id_type end_node = ps.first; /* to node */
				osmium::unsigned_object_id_type way_id = ps.second.first; /* OSM way */
				int dir = ps.second.second; /* edge directions */
				
				/* potential filters */
				if(nodes_seen.count(end_node)) continue; /* if the first node on the path has been seen already, there is no need to look further */
				if(sccs.size()) if(sccs.at(end_node) != sccid) continue; /* filter for connected components */
				
				/* start of a new edge */
				out.begin_edge(dir);
				/* add first way and first two nodes */
				if (replace_ids) {
					out.new_way(istash.get<osmium::Way>(ways.at(way_id)),get_id(way_id));
					out.new_node(istash.get<osmium::Node>(nodes.at(n1)),get_id(n1));
					out.new_node(istash.get<osmium::Node>(nodes.at(end_node)),get_id(end_node));
				}
				else {
					out.new_way(istash.get<osmium::Way>(ways.at(way_id)),0);
					out.new_node(istash.get<osmium::Node>(nodes.at(n1)),0);
					out.new_node(istash.get<osmium::Node>(nodes.at(end_node)),0);
				}
				ways_seen.insert(way_id);
				
				while(true) {
					osmium::unsigned_object_id_type end_way;
					const auto& nn = node_graph.at(end_node);
					if(nn.size() != 2) break; /* break if current node's degree != 2 */
					
					/* find the next node -- note that this loop will have maximum two iterations, since the node has degree 2 */
					int dir2;
					for(const auto& x : nn) {
						if(x.first != last_node) {
							last_node = end_node;
							end_node = x.first;
							end_way = x.second.first;
							dir2 = x.second.second;
							break;
						}
					}
					
					/* check that direction is still consistent */
					if(dir2 != dir) {
						if(dir2 == 0 || dir == 0) {
							/* OK, partly one-way segment, treat this as two segments */
							end_node = last_node;
							extra_start.insert(end_node);
							break;
						}
						/* this is error, "source" or "sink" node detected */
						fprintf(stderr,"OSMMap::generate_simplified_graph(): edges change direction in path chain!\n");
						fprintf(stderr,"nodes: %lu -- %lu\n",last_node,end_node);
						throw std::runtime_error("OSMMap::save_csv(): edges change direction in path chain!\n");
					}
					
					if(replace_ids) {
						if(ways_seen.insert(end_way).second) out.new_way(istash.get<osmium::Way>(ways.at(end_way)),get_id(end_way));
						out.new_node(istash.get<osmium::Node>(nodes.at(end_node)),get_id(end_node));
					}
					else {
						if(ways_seen.insert(end_way).second) out.new_way(istash.get<osmium::Way>(ways.at(end_way)),0);
						out.new_node(istash.get<osmium::Node>(nodes.at(end_node)),0);
					}
					nodes_seen.insert(last_node); /* at this point, only add middle nodes as seen */
				}
				/* end of edge, we have added all nodes */
				out.end_edge();
				ways_seen.clear();
			}
			nodes_seen.insert(n1);
		}
	}
}


}


#endif // OSM_READER_H

