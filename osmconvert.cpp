/*
 * osmconvert.cpp -- convert OSM data to TSV format
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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <osmium/io/any_input.hpp>
#include "osmreader.hpp"
#ifndef NO_SHP
#include "shp_output.hpp"
#endif


int main(int argc, char **argv)
{
	char* infn = 0; /* input OSM or PBF filename */
	char* out_base = 0; /* output base filename */
	bool out_tsv = false;
	bool bike_filter = false;
	bool only_scc = false;
	bool sccs_dir = false;
	bool replace_ids = false;
	bool out_shapefile = false;
	bool sg_sz_filter = false;
	bool write_edge_ids_shp = false;
	bool drive_filter = false;
	
	for(int i=1;i<argc;i++) {
		if(argv[i][0] == '-') switch(argv[i][1]) {
			case 'i':
				infn = argv[i+1];
				i++;
				break;
			case 'o':
				out_base = argv[i+1];
				i++;
				break;
			case 't':
				out_tsv = true;
				break;
#ifndef NO_SHP
			case 's':
				out_shapefile = true;
				break;
#endif
			case 'b':
				bike_filter = true;
				break;
			case 'd':
				drive_filter = true;
				break;
			case 'C':
				sccs_dir = true;
			case 'c':
				only_scc = true;
				break;
			case 'r':
				replace_ids = true;
				break;
			case 'S':
				sg_sz_filter = true;
				break;
			case 'e':
				write_edge_ids_shp = true;
				break;
			default:
				fprintf(stderr,"Unknown parameter: %s!\n",argv[i]);
				break;
		}
		else fprintf(stderr,"Unknown parameter: %s!\n",argv[i]);
	}
	
	if(! (infn && out_base) ) {
		fprintf(stderr,"No input or output filename given!\n");
		return 1;
	}
	
	OSMReader::way_filter_tags wf;
	if(bike_filter) wf = OSMReader::way_filter_tags::default_bike_filter_sg();
	else {
		if(drive_filter) wf = OSMReader::way_filter_tags::road_filter_drive_service();
		else wf = OSMReader::way_filter_tags::default_road_filter();
	}
	if(sg_sz_filter) OSMReader::way_filter_tags::filter_sg_hk_sz(wf);
	
	OSMReader::OSMMap map(wf,replace_ids);
	
	{
		osmium::io::Reader reader(infn);
		osmium::apply(reader,map);
	}
	
	unsigned int sccid = 0;
	if(only_scc) {
		if(sccs_dir) sccid = map.find_sccs_dir().first;
		else sccid = map.find_sccs().first;
	}
	int ret = 0;
	
	if(out_tsv) {
		FILE* fnodes = 0;
		FILE* fedges = 0;
		char* tmp1 = (char*)malloc(sizeof(char)*(strlen(out_base) + 12));
		if(!tmp1) {
			fprintf(stderr,"Error allocating memory!\n");
			return 1;
		}
		sprintf(tmp1,"%s_nodes.dat",out_base);
		fnodes = fopen(tmp1,"w");
		sprintf(tmp1,"%s_edges.dat",out_base);
		fedges = fopen(tmp1,"w");
		if( ! (fnodes && fedges) ) {
			fprintf(stderr,"Error opening output files!\n");
			ret = 1;
		}
		else {
			auto sc = OSMReader::OSMMap::tsv_output::default_speeds_sg();
			map.save_tsv(fnodes,fedges,sccid,&sc);
		}
		if(fnodes) fclose(fnodes);
		if(fedges) fclose(fedges);
		free(tmp1);
		if(ret) return ret;
	}
#ifndef NO_SHP
	if(out_shapefile) {
		OSMReader::shp_output out1(out_base,replace_ids,write_edge_ids_shp);
		map.generate_simplified_graph(out1,sccid);
	}
#endif
	
	return 0;
}

