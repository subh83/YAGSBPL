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


This is a collection of example programs for demonstrating the basic use of the YAGSBPL C++ library.
Requirements: C++ compiler with STL

The following programs demonstrate the use of the 3 different methods for describing a graph (as detailed at http://subhrajit.net/index.php?WPage=yagsbpl ).
    Example_GlobalFunctionPointers.cpp : Uses global function pointers
    Example_VirtualFunctionDerivation.cpp : Uses member function by inheriting and redefining virtual functions
    Example_ClassMemberFunctionPointers.cpp : Uses member function by using function container

To compile all the files in this example:
1. Open "makeExample.sh" and change the value of PathToYAGSBPL to point to the folder containing the YAGSBPL library.
2. In a terminal run "sh makeExample.sh". That should compile all the 3 files.


At the end of successful compilation you should have 3 executables named main_XXX. To run one just do ./Example_XXX in a terminal. Each of the programs employ different methods for the graph construction as detailed in the tutorial at http://subhrajit.net/index.php?WPage=yagsbpl .

