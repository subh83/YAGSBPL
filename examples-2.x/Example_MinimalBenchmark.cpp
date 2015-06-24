/******************************************************************************************
*                                                                                        *
*    Part of                                                                             *
*    Yet Another Graph-Search Based Planning Library (YAGSBPL)                           *
*    A template-based C++ library for graph search and planning                          *
*    Version 1.0                                                                         *
*    ----------------------------------------------------------                          *
*    Copyright (C) 2010  Subhrajit Bhattacharya                                          *
*                                                                                        *
*    This program is free software: you can redistribute it and/or modify                *
*    it under the terms of the GNU General Public License as published by                *
*    the Free Software Foundation, either version 3 of the License, or                   *
*    (at your option) any later version.                                                 *
*                                                                                        *
*    This program is distributed in the hope that it will be useful,                     *
*    but WITHOUT ANY WARRANTY; without even the implied warranty of                      *
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                       *
*    GNU General Public License for more details <http://www.gnu.org/licenses/>.         *
*                                                                                        *
*                                                                                        *
*    Contact: subhrajit@gmail.com, http://fling.seas.upenn.edu/~subhrabh/                *
*                                                                                        *
*                                                                                        *
******************************************************************************************/
//    For a detailed tutorial and download, visit 
//    http://fling.seas.upenn.edu/~subhrabh/cgi-bin/wiki/index.php?n=Projects.ProgrammingLibraries-YAGSBPL


#include <stdio.h>
// =======================
// YAGSBPL libraries
// constants
#define _YAGSBPL_A_STAR__VIEW_PROGRESS 1
// headers
#include "yagsbpl_base.h"
#include "planners/A_star.h"

// A node of the graph
class myNode
{
public:
	int x, y;
	bool operator==(const myNode& n)
	    { return (x==n.x && y==n.y); }
};

int getHashBin(myNode& n) 
{ return (abs(n.x) % 200); }

// -----

void getSuccessors(myNode& n, std::vector<myNode>* s, std::vector<double>* c) 
{
	myNode tn;
	for (int a=-1; a<=1; a++)
		for (int b=-1; b<=1; b++) {
			if (a==0 && b==0) continue;
			
			tn.x = n.x + a;
			tn.y = n.y + b;
			// TODO
		    s->push_back(tn);
		    c->push_back(sqrt((double)(a*a+b*b))); 
		}
}

// =============================================================================

int main(int argc, char *argv[])
{
    // Init
	GenericSearchGraphDescriptor<myNode,double> myGraph;
	
	myGraph.getHashBin_fp = &getHashBin;
	myGraph.getSuccessors_fp = &getSuccessors;
	myGraph.hashTableSize = 201;
	
	myNode tempNode;
	tempNode.x = -150; tempNode.y = -150; // Start node
	myGraph.SeedNode = tempNode;
	
	
	// Planning
	A_star_planner<myNode,double>  planner;
	planner.init(myGraph);
	planner.plan();
	
	std::vector< std::vector< myNode > > paths = planner.getPlannedPaths();
	printf("\nNumber of paths: %d \n", paths.size());
	printf("\nCoordinates of path 0: ");
	for (int a=0; a<paths[0].size(); a++)
		printf("(%d, %d)-", paths[0][a].x, paths[0][a].y);
}

