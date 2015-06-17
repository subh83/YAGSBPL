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
#include "yagsbpl_base.h"
#include "planners/A_star.h"

// A node of the graph
class myNode
{
public:
	int x, y; // Profiling observation: integer coordinates, hence operator==,
	          //  makes the search significantly faster (almost 10 folds than double)
	bool operator==(const myNode& n) { return (x==n.x && y==n.y); }; // This must be defined for the node
};

// ============================================================
// Functions that describe the graph, placed inside a class
// Here the class "GraphFunctionContainer" is redefining the virtual functions declared in "SearchGraphDescriptorFunctionContainer"
// Thus the names of the functions are important here.

class GraphFunctionContainer : public SearchGraphDescriptorFunctionContainer<myNode,double>
{
public:

	int getHashBin(myNode& n) // Use the absolute value of x coordinate as hash bin counter. Not a good choice though!
	{
		return ((int)fabs(n.x));
	}

	bool isAccessible(myNode& n)
	{
		// Environment with (0,0) at center, and a circular obstacle of radius 'circleRadius' centered at (0,0)
		return ( n.x*n.x + n.y*n.y > circleRadius*circleRadius 
					&& n.x>=Xmin && n.x<=Xmax && n.y>=Ymin && n.y<=Ymax );
	}

	void getSuccessors(myNode& n, std::vector<myNode>* s, std::vector<double>* c) // Define a 8-connected graph
	{
		// This function needn't account for obstacles or size of environment. That's done by "isAccessible"
		myNode tn;
		s->clear(); c->clear(); // Planner is supposed to clear these. Still, for safety we clear it again.
		for (int a=-1; a<=1; a++)
			for (int b=-1; b<=1; b++) {
				if (a==0 && b==0) continue;
				tn.x = n.x + a;
				tn.y = n.y + b;
				s->push_back(tn);
				c->push_back(sqrt((double)(a*a+b*b)));
			}
	}

	double getHeuristics(myNode& n1, myNode& n2)
	{
		int dx = abs(n1.x - n2.x);
		int dy = abs(n1.y - n2.y);
		return (sqrt((double)(dx*dx + dy*dy))); // Euclidean distance as heuristics
	}

	// -------------------------------
	// constructors
	GraphFunctionContainer (int cr, std::vector<int> ranges) 
		{ circleRadius = cr; Xmin = ranges[0]; Xmax = ranges[1]; Ymin = ranges[2]; Ymax = ranges[3]; }
	GraphFunctionContainer () 
		{ circleRadius = 10; Xmin = -20; Xmax = 20; Ymin = -20; Ymax = 20; }

private:

	int circleRadius;
	int Xmin, Xmax, Ymin, Ymax;

};


// =============================================================================

int main(int argc, char *argv[])
{
	// Profiling observation: Using int instead of double cost provides marginal improvement (~10%)
	GenericSearchGraphDescriptor<myNode,double> myGraph;
	
	// We describe the graph, cost function, heuristics, and the start & goal in this block
	// ------------------------------------------------------------------------------------
	// Create an instance of "GraphFunctionContainer_derived", and set is as the function container.
	std::vector<int> ranges(4);
	ranges[0] = -200; ranges[1] = 200; ranges[2] = -200; ranges[3] = 200;
	GraphFunctionContainer fun_cont(100, ranges);
	myGraph.func_container = &fun_cont;
	// Set other variables
	myGraph.hashTableSize = 212; // Since in this problem, "getHashBin" can return a max of value 201.
	myGraph.hashBinSizeIncreaseStep = 512; // By default it's 128. For this problem, we choose a higher value.
	
	myNode tempNode;
	tempNode.x = -150; tempNode.y = -150; // Start node
	myGraph.SeedNode = tempNode;
	tempNode.x = 150; tempNode.y = 150; // Goal node
	myGraph.TargetNode = tempNode;
	// ------------------------------------------------------------------------------------
	
	// Planning
	A_star_planner<myNode,double>  planner;
	planner.setParams(1.0, 10); // optional.
	planner.init(myGraph);
	planner.plan();
	
	std::vector< std::vector< myNode > > paths = planner.getPlannedPaths();
	printf("\nNumber of paths: %d\nPath coordinates: \n[ ", paths.size());
	for (int a=0; a<paths[0].size(); a++)
		printf("[%d, %d]; ", paths[0][a].x, paths[0][a].y);
	printf(" ]\n\n");
	
	tempNode.x = -100; tempNode.y = -100;
	printf("Testing 'getNodeInfo': g-value of (%d,%d): %f\n\n", tempNode.x, tempNode.y, planner.getNodeInfo(tempNode).g);
}

