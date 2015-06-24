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

// Related publication: http://ijr.sagepub.com/content/33/1/113


#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h> 
// =======================
// YAGSBPL libraries
#include "yagsbpl_base.h"
#include "planners/A_star.h"

// Environment description
#define MAX_X 100
#define MIN_X -100
#define MAX_Y 100
#define MIN_Y -100
#define DIAG_LEN (sqrt((double) ((MAX_X-MIN_X)*(MAX_X-MIN_X) + (MAX_Y-MIN_Y)*(MAX_Y-MIN_Y)) ))

// Rectangles: {x1, y1, x2, y2}, x1<x2, y1<y2
#define OBS_RECT_COUNT 2
int OBS_RECT[OBS_RECT_COUNT][4] = { {-50, 30, 50, 50},
								    {30, -20, 50, 50} };

#define PLOT_SCALE 1
#define _cv_plot_coord(x,y) (cvPoint(PLOT_SCALE*((x)-MIN_X), PLOT_SCALE*((y)-MIN_Y)))

double A_star_eps = 0.0;
#define SAVE_IMG 1
#if SAVE_IMG
	char imgPrefix = 'T';
#endif
int progress = 0;

// ------------------------------

IplImage* ipl_image_p;
int frameno = 0;

// A node of the graph
class myNode
{
public:
	int x, y; // Profiling observation: integer coordinates, hence operator==,
	          //  makes the search significantly faster (almost 10 folds than double)
	bool operator==(const myNode& n) { return (x==n.x && y==n.y); }; // This must be defined for the node
	
	// Optional event handlers
	void event_NodeExpanded (double gVal, double fVal, int seedLineage);
	void event_SuccUpdated (myNode nn, double edgeCost, double gVal, double fVal, int seedLineage);
};

A_star_planner<myNode,double>  planner;
GenericSearchGraphDescriptor<myNode,double> myGraph;
void getSuccessors(myNode& n, std::vector<myNode>* s, std::vector<double>* c);

void myNode::event_NodeExpanded (double gVal, double fVal, int seedLineage)
{
    CvScalar col = cvScalar(0, (int)(255.0*gVal/DIAG_LEN), (int)(255.0*(1.0-gVal/DIAG_LEN)));
    //A_star_planner<myNode,double>::GraphNode_p thisGraphNode = planner.hash->getNodeInHash(*this);
    std::vector<myNode> s;
    std::vector<double> c;
    getSuccessors(*this, &s, &c);
    for (int a=0; a<s.size(); a++) {
        //if( (thisGraphNode->successors.getLinkSearchGraphNode(a))->plannerVars.seedLineage == seedLineage )
        //printf("%d,%d\n", (planner.hash->getNodeInHash(s[a]))->plannerVars.seedLineage, seedLineage);
        if ( (planner.hash->getNodeInHash(s[a]))->plannerVars.seedLineage != seedLineage  &&  seedLineage>=0  &&  (planner.hash->getNodeInHash(s[a]))->plannerVars.seedLineage>=0 )
            col = cvScalar(150.0, 50.0, 50.0);
    }
    
    cvCircle(ipl_image_p, _cv_plot_coord(x,y), 0.3*PLOT_SCALE, col, -1);
    
	if (progress%100 == 0) {
	    for (int a=0; a<myGraph.SeedNodes.size(); a++)
	        cvCircle(ipl_image_p, _cv_plot_coord(myGraph.SeedNodes[a].x,myGraph.SeedNodes[a].y), 1.0*PLOT_SCALE, cvScalar(0, 255.0, 0), -1);
	    cvShowImage("D3N2_MovingObstacleIn2D", ipl_image_p);
	    #if SAVE_IMG
		    char imgFname[1024];
		    sprintf(imgFname, "outfiles/%c%03d.png", imgPrefix, frameno++);
		    cvSaveImage(imgFname, ipl_image_p);
	    #endif
	
	    cvWaitKey(10);
	}
	
	progress++;
}

void myNode::event_SuccUpdated (myNode nn, double edgeCost, double gVal, double fVal, int seedLineage)
{
/*	double xm=((double)(x+nn.x))/2.0, ym=((double)(y+nn.y))/2.0;
	CvScalar col=cvScalar(200.0, 200.0, 200.0);
	int linewidth=1;
	 int otherLineage = (planner.hash->getNodeInHash(*this))->plannerVars.seedLineage;
	if (seedLineage!=otherLineage && seedLineage && otherLineage) {
	    col = cvScalar(50.0, 100.0, 50.0);
	    linewidth=3;
	} 

	cvLine(ipl_image_p, _cv_plot_coord(xm+0.5*((double)x-xm),ym+0.5*((double)y-ym)), 
				_cv_plot_coord(xm+0.5*((double)nn.x-xm),ym+0.5*((double)nn.y-ym)), col, linewidth);
	cvCircle(ipl_image_p, _cv_plot_coord(nn.x,nn.y), 0.3*PLOT_SCALE, cvScalar(255.0, 0, 0), 1); */
}

// ============================================================
// Functions that describe the graph

int getHashBin(myNode& n) // Use the absolute value of x coordinate as hash bin counter. Not a good choice though!
{
	return ((int)fabs(n.x));
}

bool isAccessible(myNode& n)
{
	bool insideObs = false;
	for (int a=0; a<OBS_RECT_COUNT; a++)
		insideObs = insideObs | ( OBS_RECT[a][0]<=n.x && OBS_RECT[a][1]<=n.y && OBS_RECT[a][2]>=n.x && OBS_RECT[a][3]>=n.y );
	
	if ( !insideObs && n.x>=MIN_X && n.x<=MAX_X && n.y>=MIN_Y && n.y<=MAX_Y )
		return true;
	else
	{
		cvCircle(ipl_image_p, _cv_plot_coord(n.x,n.y), 0.4*PLOT_SCALE, cvScalar(0,0,0), -1);
		return false;
	}
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

/*
double getHeuristics(myNode& n1, myNode& n2)
{
	int dx = abs(n1.x - n2.x);
	int dy = abs(n1.y - n2.y);
	return (sqrt((double)(dx*dx + dy*dy))); // Euclidean distance as heuristics
} */

bool stopSearch(myNode& n)
{
	return false;
}

// =============================================================================

int main(int argc, char *argv[])
{
	// Profiling observation: Using int instead of double cost provides marginal improvement (~10%)
	//GenericSearchGraphDescriptor<myNode,double> myGraph;
	
	// We describe the graph, cost function, heuristics, and the start & goal in this block
	// ------------------------------------------------------------------------------------
	// Set the functions
	myGraph.getHashBin_fp = &getHashBin;
	myGraph.isAccessible_fp = &isAccessible;
	myGraph.getSuccessors_fp = &getSuccessors;
	//myGraph.getHeuristics_fp = &getHeuristics;
	// Set other variables
	myGraph.hashTableSize = MAX_X - MIN_X + 1; // Since in this problem, "getHashBin" can return a max of value 201.
	
	myNode tempNode;
	tempNode.x = (int)(0.8*MIN_X); tempNode.y = (int)(0.8*MIN_Y); // Start node
	myGraph.SeedNodes.push_back(tempNode);
	tempNode.x = (int)(0.4*MAX_X); tempNode.y = (int)(0.7*MIN_Y);
	myGraph.SeedNodes.push_back(tempNode);
	tempNode.x = (int)(0.65*MAX_X); tempNode.y = (int)(0.7*MAX_Y);
	myGraph.SeedNodes.push_back(tempNode);
	
	//tempNode.x = (int)(0.8*MAX_X); tempNode.y = (int)(0.8*MAX_Y); // Goal node
	//myGraph.TargetNode = tempNode;
	myGraph.stopSearch_fp = &stopSearch;
	// ------------------------------------------------------------------------------------
	
	// Initiate visualization
	CvSize map_size;
	cvNamedWindow("D3N2_MovingObstacleIn2D", CV_WINDOW_AUTOSIZE);
	map_size.width = PLOT_SCALE * (1 + MAX_X - MIN_X);
	map_size.height = PLOT_SCALE * (1 + MAX_Y - MIN_Y);
	ipl_image_p = cvCreateImage(map_size, IPL_DEPTH_8U, 3);
	ipl_image_p->origin = 1;
	ipl_image_p->widthStep = ipl_image_p->width * 3;
	cvSet(ipl_image_p, CV_RGB(255.0,255.0,255.0));
	for (int a=0; a<OBS_RECT_COUNT; a++)
		cvRectangle(ipl_image_p, _cv_plot_coord(OBS_RECT[a][0],OBS_RECT[a][1]), _cv_plot_coord(OBS_RECT[a][2],OBS_RECT[a][3]), 
					cvScalar(200.0,200.0,200.0), -1 );
	/* for (int a=0; a<myGraph.SeedNodes.size(); a++)
	    cvCircle(ipl_image_p, _cv_plot_coord(myGraph.SeedNodes[a].x,myGraph.SeedNodes[a].y), 0.3*PLOT_SCALE, cvScalar(0, 255.0, 0), -1); */
	cvShowImage("D3N2_MovingObstacleIn2D", ipl_image_p);
	
	// Planning
	planner.setParams(A_star_eps, 10); // optional.
	planner.event_NodeExpanded_nm = &myNode::event_NodeExpanded; // optional event handler.
	planner.event_SuccUpdated_nm = &myNode::event_SuccUpdated; // optional event handler.
	planner.init(myGraph);
	planner.plan();
	
	// Display result
	/* std::vector< std::vector< myNode > > paths = planner.getPlannedPaths();
	for (int a=0; a<paths[0].size()-1; a++)
		cvLine(ipl_image_p, _cv_plot_coord(paths[0][a].x, paths[0][a].y), _cv_plot_coord(paths[0][a+1].x, paths[0][a+1].y), 
				cvScalar(100.0,250.0,200.0), 4); */
	cvShowImage("D3N2_MovingObstacleIn2D", ipl_image_p);
	
	#if SAVE_IMG
		char imgFname[1024];
		sprintf(imgFname, "outfiles/%c%03d.png", imgPrefix, frameno++);
		cvSaveImage(imgFname, ipl_image_p);
	#endif
	
	cvWaitKey();
}

