// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Cecily Sunday
// =============================================================================
//
// Convert first and last binary files in a folder to csv format
// 
// =============================================================================

#include "chrono_multicore/physics/ChSystemMulticore.h"

#include "ProjProc.h"

#include <iostream> 
#include <string.h>

using namespace chrono;


int main(int argc, char* argv[]) {
	// Check that the user supplied the name of a filepath
	if (argc != 2) {
		fprintf(stderr, "\nERR: Please provide a filepath.\n");
		return -1;
	}
	std::string configdir = argv[1];
	
	// Convert select binary files to csv format
	if (ConvertBinaries(configdir) != 0) {
		fprintf(stderr, "\nERR: Could not convert binary files.\n");
		return -1;
	}

	return 0;
}