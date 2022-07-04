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
// Convert first and last binary files in a folder to csv format and find
// the packing statics for the given simulation
// 
// =============================================================================

#include "chrono_multicore/physics/ChSystemMulticore.h"

#include "ProjConfig.h"
#include "ProjProc.h"
#include "ProjWriteData.h"

using namespace chrono;


int main(int argc, char* argv[]) {
	// Check that the user has supplied the name of a filepath
	if (argc != 2) {
		fprintf(stderr, "\nERR: Please provide a filepath.\n");
		return -1;
	}
	std::string configdir = argv[1];

	// Import the configuration file
	ConfigParameters cp;
	const std::string paramfile = configdir + "/system_params.txt";
	if (ImportConfigFile(&cp, paramfile) != 0) {
		fprintf(stderr, "\nERR: Could not import parameter file.\n");
		return -1;
	}

	// Convert select binary files to csv format
	if (ConvertBinaries(configdir) != 0) {
		fprintf(stderr, "\nERR: Could not convert binary files.\n");
		return -1;
	}

	// Calculate the packing statistics for the simulations
	if (CalcPacking(configdir, cp) != 0) {
		fprintf(stderr, "\nERR: Could not calculate sim packing stats.\n");
		return -1;
	}

	return 0;
}