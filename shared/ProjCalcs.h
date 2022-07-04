// =============================================================================
// 
// Helper functions for placing and tracking particles in a simulation
// 
// =============================================================================

#ifndef PROJCALCS_H
#define PROJCALCS_H

using namespace chrono;



// Generate a vector of random offsets in the X, Y, and Z directions using ChRandom
ChVector<> ChRandomXYZ(double minmax);

// Generate a vector of random offsets in the X, Y, and Z directions using a random seed
ChVector<> SeedRandomXYZ(double minmax);

// Get the highest point in the particle bed, assuming that all particles have positive ID numbers
double GetSurfaceHeight(ChSystemMulticoreSMC* msystem);

// Function to calculate and log the kinetic energy and velocity of the particles in a system 
bool CalcAverageKE(const ChSystemMulticoreSMC &msystem, const std::string &path, const size_t &num,
	const size_t &start_id, const double &time, const double &threshold);

#endif 
