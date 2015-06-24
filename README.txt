This is the new page for YAGSBPL, previously available at https://code.google.com/p/yagsbpl/

Important notice: Versions 1.x and 2.x of YAGSBPL are depreciated and support for it will be discontinued. Version 3.x is under development, and will be available soon.


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

*************************************************************************
*  For the latest documentation, detailed tutorial and download, visit  *
*  http://subhrajit.net/index.php?WPage=yagsbpl                         *
**************************************************************************

Features:
--------

YAGSBPL is designed to be:

* Fast (e.g., with integer coordinates for nodes but floating point cost as well as cost function needing to perform floating point operations online, and an average degree of the graph being 8, the library can expand about 70,000 nodes in the graph in just 1 second on a 2.1GHz processor machine with 2GB RAM.)
* Easy to use (being template-based, defining new arbitrary node-types, cost types, etc. is made easy. For graph connectivity, node accessibility tests, etc, pointers to user-defined functions can be used, which makes defining the graph structure very easy, yet highly flexible.) 


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
 
