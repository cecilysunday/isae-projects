// =============================================================================
//
// Helper functions for processing simulation data
//
// =============================================================================

#ifndef PROJPROC_H
#define PROJPROC_H

using namespace chrono;

typedef struct PackingData {
	std::string input_file;
	std::string container_shape;
	int num_grains = 0;
	double container_volume = 0;
	double grain_mass = 0;
	double grain_volume = 0;
	double packing_fraction = 0;
	double bulk_density = 0;
	double xlim_min = 0;
	double xlim_max = 0;
	double ylim_min = 0;
	double ylim_max = 0;
	double zlim_min = 0;
	double zlim_max = 0;
} PackingData;

struct ParticleData;

struct ConfigParameters;


// Print the packing fraction and bulk density to a file in the specified data directory
void PrintPackingStats(const std::string& configdir, const std::string& units, const PackingData& packing);

// Verify that the simulation has either cylinder or box geometry. Otherwise, the packing fraction cannot be determined.
std::string GetContainerShape(const std::string& proj_container);

// Return the name of first bin file in a directory for a 'run' simulation and the name of the last bin file for a 'set' simulation
std::string GetFileToProcess(const std::string& configdir, const std::string& proj_type);

// Calculate packing fraction and bulk density based on simulation configuration
int CalcPackingStats(const std::string& configdir, const ConfigParameters& config, PackingData* packing);

// Wrapper function to calculate and print the packing configuration of a simulation
int CalcPacking(const std::string& configdir, const ConfigParameters& config);

// Convert first and last binary files in a folder to csv format
int ConvertBinaries(const std::string& configdir);

#endif