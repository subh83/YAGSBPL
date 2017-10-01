<pre>
**************************************************************************
*    The latest version of YAGSBPL has a new name and a new home:        *
*        DOSL: Discrete Optimal Search Library                           *
*        Available at: <a href="https://github.com/subh83/DOSL">https://github.com/subh83/DOSL</a>                    *
*    <b><span style="color:red">YAGSBPL is deprecated and support for it has been discontinued.</span></b>     *
*        It has been superseded by DOSL.                                 *
*        Please vist <a href="https://github.com/subh83/DOSL">https://github.com/subh83/DOSL</a> to download DOSL.    *
**************************************************************************
</pre>

==========================================================================


YAGSBPL (deprecated)...

```
*******************************************************************************
*                                                                             *
* Yet Another Graph-Search Based Planning Library (YAGSBPL)                   *
* A template-based C++ library for graph search and planning                  *
* Version 2.1                                                                 *
* ----------------------------------------------------------                  *
* Copyright (C) 2013  Subhrajit Bhattacharya                                  *
*                                                                             *
* This program is free software: you can redistribute it and/or modify        *
* it under the terms of the GNU General Public License as published by        *
* the Free Software Foundation, either version 3 of the License, or           *
* (at your option) any later version.                                         *
*                                                                             *
* This program is distributed in the hope that it will be useful,             *
* but WITHOUT ANY WARRANTY; without even the implied warranty of              *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the               *
* GNU General Public License for more details <http://www.gnu.org/licenses/>. *
*                                                                             *
*                                                                             *
* Contact: subhrajit@gmail.com, http://subhrajit.net/                         *
*                                                                             *
*                                                                             *
*******************************************************************************

This is the newer page for YAGSBPL, previously available at
    https://code.google.com/p/yagsbpl/

Important notice: Versions 1.x and 2.x of YAGSBPL are deprecated (although fully functional) and support for them will be discontinued. Version 3.x has now been renamed as DOSL (Discrete Optimal Search Library) and is available at https://github.com/subh83/DOSL 

For the latest documentation, detailed tutorial and download, visit
    http://subhrajit.net/index.php?WPage=yagsbpl


Features:
--------

Description: "Yet Another Graph-Search Based Planning Library (YAGSBPL)" is a fast, efficient and easy-to-use graph construction and search (using algorithms like A-star, Dijkstra's, etc.) library written in C++, designed specifically for searching medium to large scale graphs for optimal paths. 

YAGSBPL is designed to be:
* Fast (e.g., with integer coordinates for nodes but floating point cost as well as cost function needing to perform floating point operations online, and an average degree of the graph being 8, the library can expand about 70,000 nodes in the graph in just 1 second on a 2.1GHz processor machine with 2GB RAM.)
* Easy to use (being template-based, defining new arbitrary node-types, cost types, etc. is made easy. For graph connectivity, node accessibility tests, etc, pointers to user-defined functions can be used, which makes defining the graph structure very easy, yet highly flexible.) 

YAGSBPL supports:
- Directed graphs with complex cost functions.
- On the fly graph construction (i.e. no need to construct and store a complete graph before starting the search/planning process - makes it highly suitable for RRT-like graph construction).
- Arbitrary graph node type declaration.
- Arbitrary edge cost type declaration.
- Intermediate storage of paths during graph search.
- Multiple goals (or goal manifold) that determine when to stop search.
- Multiple seed nodes (start nodes) for wave-front expansion type graph exploration, and ability to trace seed lineage for any node.
- Event handling (i.e., call to user-defined functions upon generation, g-score updating or expansion of a node during the search process).
- Ability to write new planners with much ease. Comes with a weighted A-star (that includes Dijkstra's and normal A-star) planner by default.

*******************************************************************************

Getting Started:
---------------

Compile and run the examples in the 'examples-2.x' folder. To compile the basic examples run the 'makeExamples.sh' script. To compile examples requiring OpenCV library run the 'makeVis.sh' script.

Basic Usage:
-----------

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

// Hash function for storing a node
int getHashBin(myNode& n) 
{ return (abs(n.x) % 200); }

// .....

// Function describing graph connectivity
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

// .....

// Entry point
int main(int argc, char *argv[])
{
    // Initiate the graph structure
    GenericSearchGraphDescriptor<myNode,double> myGraph;
    
    // set function pointers
    myGraph.getHashBin_fp = &getHashBin;
    myGraph.getSuccessors_fp = &getSuccessors;
    myGraph.hashTableSize = 201;
    
    myNode tempNode;
    tempNode.x = -150; tempNode.y = -150; // Start node
    myGraph.SeedNode = tempNode;
    
    
    // Planning/searching
    A_star_planner<myNode,double>  planner;
    planner.init(myGraph);
    planner.plan();
    
    // retrieve the found shortest path(s)
    std::vector< std::vector< myNode > > paths = planner.getPlannedPaths();
    printf("\nNumber of paths: %d \n", paths.size());
    printf("\nCoordinates of path 0: ");
    for (int a=0; a<paths[0].size(); a++)
        printf("(%d, %d)-", paths[0][a].x, paths[0][a].y);
}


Detailed Tutorial/Documentation:
-------------------------------
http://subhrajit.net/index.php?WPage=yagsbpl


*******************************************************************************

Version history:
---------------

* Dec 2010: v1.0 released

* Jan 2011: v1.5 released
 - More efficient heap (keyed heap) introduced
 - Easier seed node declaration
 - Multiple seed nodes introduced
 
* Apr 2011: v2.0 released
 - Event handling introduced for A* planner
 - Some structural modifications in 'SearchGraphNode' class
 - Ability to keep track of lineage when multiple seed nodes are present

* May 2013: v2.1 released
 - Some bug fixes
 - Binary heap as priority list
 
```
