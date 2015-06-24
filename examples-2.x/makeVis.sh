#!/bin/bash 

PathToYAGSBPL=../yagsbpl-2.1
echo
echo "IMPORTANT: Path to YAGSBPL is set to '$PathToYAGSBPL'. Make sure this points to the folder containing yagsbpl_base.h"
echo "           You also need OpenCV (including 'cvaux' and 'highgui') for compiling these. Make sure those are in search path of g++"
echo

opts="-O3 -g -w -I. -I$PathToYAGSBPL"
libs="-lm -lopencv_core -lopencv_highgui" 

for examplename in "VisTessellation" "VisHExplore" "VisEvents"
do
	echo "Now compiling $examplename..."
	g++ $opts -c $examplename.cpp
	g++ $opts -o $examplename $examplename.o $libs
done

echo

