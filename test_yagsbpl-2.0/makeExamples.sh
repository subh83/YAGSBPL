#!/bin/bash 

PathToYAGSBPL=../yagsbpl-2.1
echo
echo "IMPORTANT: Path to YAGSBPL is set to '$PathToYAGSBPL'. Make sure this points to the folder containing yagsbpl_base.h"
echo

opts="-O3 -g -w -I. -I$PathToYAGSBPL"
libs="-lm" 

for examplename in "Example_GlobalFunctionPointers" "Example_VirtualFunctionDerivation" "Example_ClassMemberFunctionPointers" "Example_MinimalBenchmark"
do
	echo "Now compiling $examplename..."
	g++ $opts -c $examplename.cpp
	g++ $opts -o $examplename $examplename.o $libs
done

echo

