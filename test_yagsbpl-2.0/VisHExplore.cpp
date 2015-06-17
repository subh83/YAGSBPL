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

// Related publication: http://dx.doi.org/10.1007/s10514-012-9304-1 

#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h> 
#include <vector>
// =======================
// YAGSBPL libraries
#define _YAGSBPL_A_STAR__VIEW_PROGRESS 1

#include "yagsbpl_base.h"
#include "planners/A_star.h"



#define PROB_HMTPY 1 // 0: homology; 1: homotopy
#define N_EXPLORE 10

// Environment description
#define MAX_X 100
#define MIN_X -100
#define MAX_Y 100
#define MIN_Y -100
#define DIAG_LEN (sqrt((double) ((MAX_X-MIN_X)*(MAX_X-MIN_X) + (MAX_Y-MIN_Y)*(MAX_Y-MIN_Y)) ))

// Rectangles: {x1, y1, x2, y2}, x1<x2, y1<y2
#define OBS_RECT_COUNT 2
int OBS_RECT[OBS_RECT_COUNT][4] = { {-60, -10, -20, 20} , {20, -10, 60, 20} };

int OBS_CENTERS[OBS_RECT_COUNT][2];

// Start and end
int START_COORD[2] = {-2, -80};
int GOAL_COORD[2] = {2, 80};

int STORED_PATH_COUNT;

#define PLOT_SCALE 1
#define _cv_plot_coord(x,y) (cvPoint(PLOT_SCALE*((x)-MIN_X), PLOT_SCALE*((y)-MIN_Y)))

double A_star_eps = 1.0;
#define SAVE_IMG 1
#if SAVE_IMG
	std::string imgPrefix = "Homotopy_";
#endif

// ------------------------------

IplImage* ipl_image_p;
int frameno = 0;

// A node of the graph
class myNode
{
public:
	int x, y;
	std::vector<int> h;
	bool operator==(const myNode& n) { 
	    if (x!=n.x || y!=n.y) return (false);
	    if (h.size()!=n.h.size()) return (false);
	    for (int a=0; a<h.size(); a++)
	        if (h[a]!=n.h[a]) return (false);
	    //printf("(%d,%d,%d)==(%d,%d,%d) ; %d\n", x,y,h[0], n.x,n.y,n.h[0], h.size());
	    return (true);
	}
	void print(std::string pre) {  printf("%s: (%d,%d,%d); %d\n", pre.c_str(), x,y,h[0], h.size()); }
	
#if	PROB_HMTPY
    myNode() { h.resize(0); } // homotopy
#else
    myNode() { h.resize(OBS_RECT_COUNT, (int)0); } // homology
#endif
	// Optional event handlers
	//void event_NodeExpanded (double gVal, double fVal, int seedLineage);
	//void event_SuccUpdated (myNode nn, double edgeCost, double gVal, double fVal, int seedLineage); 
};


// ============================================================
// Functions that describe the graph

int getHashBin(myNode& n) // Use the absolute value of x coordinate as hash bin counter. Not a good choice though!
{
	return ((int)fabs(n.x));
}

bool isAccessible(myNode& n)
{
	//bool insideObs = false;
	for (int a=0; a<OBS_RECT_COUNT; a++)
		if ( OBS_RECT[a][0]<=n.x && OBS_RECT[a][1]<=n.y && OBS_RECT[a][2]>=n.x && OBS_RECT[a][3]>=n.y )
		    return false;
	
	if (!(n.x>=MIN_X && n.x<=MAX_X && n.y>=MIN_Y && n.y<=MAX_Y) )
		return false;
		
    return true;
}

void getSuccessors(myNode& n, std::vector<myNode>* s, std::vector<double>* c) // Define a 8-connected graph
{
	// This function needn't account for obstacles or size of environment. That's done by "isAccessible"
	myNode tn;
	int pm;
	s->clear(); c->clear(); // Planner is supposed to clear these. Still, for safety we clear it again.
	for (int a=-1; a<=1; a++)
		for (int b=-1; b<=1; b++) {
			if (a==0 && b==0) continue;
			tn.x = n.x + a;
			tn.y = n.y + b;
			tn.h = n.h;
			for (int p=0; p<OBS_RECT_COUNT; p++) {
			    pm = 0;
			    if (n.y>OBS_CENTERS[p][1] && tn.y>OBS_CENTERS[p][1] && n.x!=tn.x) {
			        if (n.x<=OBS_CENTERS[p][0] && tn.x>OBS_CENTERS[p][0]) 
			            pm = 1;
			        else if (n.x>OBS_CENTERS[p][0] && tn.x<=OBS_CENTERS[p][0]) 
			            pm = -1;
			    }
			    if (pm)
			    {
#if	PROB_HMTPY
                    if (tn.h.size()>0 && tn.h[tn.h.size()-1]==-pm*(p+1))
                        tn.h.pop_back();
                    else
                        tn.h.push_back(pm*(p+1));
#else
                    tn.h[p] += pm;
#endif
			    }
            }
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


bool storePath(myNode& n)
{
    if ( n.x==GOAL_COORD[0] && n.y==GOAL_COORD[1] ) {
        STORED_PATH_COUNT++;
        return true;
    }
	return false;
}

bool stopSearch(myNode& n)
{
    if ( n.x==GOAL_COORD[0] && n.y==GOAL_COORD[1] && STORED_PATH_COUNT>=(N_EXPLORE-1)) 
        return true;
	return false;
}

// =============================================================================

int main(int argc, char *argv[])
{
    // Some pre-computations
    for (int a=0; a<OBS_RECT_COUNT; a++) {
        OBS_CENTERS[a][0] = (OBS_RECT[a][0] + OBS_RECT[a][2]) / 2;
        OBS_CENTERS[a][1] = (OBS_RECT[a][1] + OBS_RECT[a][3]) / 2;
    }
    STORED_PATH_COUNT = 0;
    // -----------------------------------------------------------------------------------
    
	// Profiling observation: Using int instead of double cost provides marginal improvement (~10%)
	GenericSearchGraphDescriptor<myNode,double> myGraph;
	
	// We describe the graph, cost function, heuristics, and the start & goal in this block
	// ------------------------------------------------------------------------------------
	// Set the functions
	myGraph.getHashBin_fp = &getHashBin;
	myGraph.isAccessible_fp = &isAccessible;
	myGraph.getSuccessors_fp = &getSuccessors;
	myGraph.getHeuristics_fp = &getHeuristics;
	myGraph.storePath_fp = &storePath;
	myGraph.stopSearch_fp = &stopSearch;
	// Set other variables
	myGraph.hashTableSize = MAX_X - MIN_X + 1;
	
	myNode tempNode;
	tempNode.x = START_COORD[0]; tempNode.y = START_COORD[1]; // Start node
	myGraph.SeedNode = tempNode;
	tempNode.x = GOAL_COORD[0]; tempNode.y = GOAL_COORD[1]; // Goal node
	myGraph.TargetNode = tempNode;
	// ------------------------------------------------------------------------------------
	
	// Initiate visualization
	CvSize map_size;
	cvNamedWindow("display", CV_WINDOW_AUTOSIZE);
	map_size.width = PLOT_SCALE * (1 + MAX_X - MIN_X);
	map_size.height = PLOT_SCALE * (1 + MAX_Y - MIN_Y);
	ipl_image_p = cvCreateImage(map_size, IPL_DEPTH_8U, 3);
	ipl_image_p->origin = 1;
	ipl_image_p->widthStep = ipl_image_p->width * 3;
	/* cvSet(ipl_image_p, CV_RGB(255.0,255.0,255.0));
	for (int a=0; a<OBS_RECT_COUNT; a++)
		cvRectangle(ipl_image_p, _cv_plot_coord(OBS_RECT[a][0],OBS_RECT[a][1]), _cv_plot_coord(OBS_RECT[a][2],OBS_RECT[a][3]), 
					cvScalar(200.0,200.0,200.0), -1 );
	cvCircle(ipl_image_p, _cv_plot_coord(myGraph.TargetNode.x,myGraph.TargetNode.y), 0.3*PLOT_SCALE, cvScalar(0, 255.0, 0), -1);
	cvShowImage("display", ipl_image_p); */
	
	// Planning
	A_star_planner<myNode,double>  planner;
	planner.setParams(A_star_eps, 10); // optional.
	//planner.event_NodeExpanded_nm = &myNode::event_NodeExpanded; // optional event handler.
	//planner.event_SuccUpdated_nm = &myNode::event_SuccUpdated; // optional event handler.
	planner.init(myGraph);
	planner.plan();
	
	// Display result
	std::vector< myNode > goals = planner.getGoalNodes();
	std::vector< std::vector< myNode > > paths = planner.getPlannedPaths();
	printf("\nNumber of paths: %d\n", paths.size());
	for (int p=0; p<paths.size(); p++)
	{
		cvSet(ipl_image_p, CV_RGB(255.0,255.0,255.0));
	    for (int a=0; a<OBS_RECT_COUNT; a++)
	    {
		    cvRectangle(ipl_image_p, _cv_plot_coord(OBS_RECT[a][0],OBS_RECT[a][1]), _cv_plot_coord(OBS_RECT[a][2],OBS_RECT[a][3]), 
					    cvScalar(200.0,200.0,200.0), -1 );
			cvLine(ipl_image_p, _cv_plot_coord(OBS_CENTERS[a][0], OBS_CENTERS[a][1]), _cv_plot_coord(OBS_CENTERS[a][0],MAX_Y), cvScalar(0.0,180.0,0.0), 1);
		}
	    cvCircle(ipl_image_p, _cv_plot_coord(myGraph.TargetNode.x,myGraph.TargetNode.y), 0.3*PLOT_SCALE, cvScalar(0, 255.0, 0), -1);
	    printf("\tPath %d: ", p);
	    for (int a=0; a<goals[p].h.size(); a++)
	        printf(" %d ", goals[p].h[a]);
	    printf("\n");
	    
	    double r=(double)(rand()%200), g=(double)(rand()%200), b=(double)(rand()%200);
	    for (int a=0; a<paths[p].size()-1; a++)
		    cvLine(ipl_image_p, _cv_plot_coord(paths[p][a].x, paths[p][a].y), _cv_plot_coord(paths[p][a+1].x, paths[p][a+1].y), 
				    cvScalar(r,g,b), 2);
	    #if SAVE_IMG
		    char imgFname[1024];
		    sprintf(imgFname, "outfiles/%s%03d.png", imgPrefix.c_str(), p);
		    cvSaveImage(imgFname, ipl_image_p);
	    #endif
	    cvShowImage("display", ipl_image_p);
	    cvWaitKey();
    }
	
	cvWaitKey();
}

