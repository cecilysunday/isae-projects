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
// Authors: Cecily Sunday and Adam Yp-Tcha
// =============================================================================
//
// Calculate the packing density from the last file in a set simulation or 
// from the first file in a run simulation.
//
// =============================================================================

#include "chrono_multicore/physics/ChSystemMulticore.h"

#include "ProjConfig.h"
#include "ProjProc.h"
#include "ProjWriteData.h"

using namespace chrono;


// Calculate the packing density associated with any simulation
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

	// Calculate the packing statistics for the simulations
	if (CalcPacking(configdir, cp) != 0) {
		fprintf(stderr, "\nERR: Could not calculate sim packing stats.\n");
		return -1;
	}

	return 0;
}