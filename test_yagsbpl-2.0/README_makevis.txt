******************************************************************************************
*                                                                                        *
*    Part of                                                                             *
*    Yet Another Graph-Search Based Planning Library (YAGSBPL)                           *
*    A template-based C++ library for graph search and planning                          *
*    Version 2.0                                                                         *
*    ----------------------------------------------------------                          *
*    Copyright (C) 2011  Subhrajit Bhattacharya                                          *
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
*    Contact: subhrajit@gmail.com, http://subhrajit.net/                                 *
*                                                                                        *
*                                                                                        *
******************************************************************************************
For a detailed tutorial and download, visit 
http://subhrajit.net/index.php?WPage=yagsbpl

VisEvents:
----------

This example program demonstrates the use of event handlers introduced in v2.0.
Requirements: C++ compiler, OpenCV library (including 'cvaux' and 'highgui')

The program VisEvents.cpp makes use of the event handlers "event_NodeExpanded_nm" and "event_SuccUpdated_nm" of the A* planner in order to give an animated visual representation of the progress of a plan.

VisHExplore:
-----------

This example program demonstrates planning with topological consideration. See http://dx.doi.org/10.1007/s10514-012-9304-1 for related publication.
Requirements: C++ compiler, OpenCV library (including 'cvaux' and 'highgui')

VisTessellation:
---------------

Shows computation of voronoi tesselation in a graph. Related publication: http://ijr.sagepub.com/content/33/1/113


To compile the program:
1. Open "makeVis.sh" and change the value of PathToYAGSBPL to point to the folder containing the YAGSBPL library.
2. In a terminal run "sh makeVis.sh".

At the end of successful compilation you should have an executable named main_VisEvents. To run one just do ./VisEvents, etc. in a terminal.

