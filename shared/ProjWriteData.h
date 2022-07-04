// =============================================================================
//
// Helper functions for reading and writing simulation data
//
// =============================================================================

#ifndef PROJWRITEDATA_H
#define PROJWRITEDATA_H

using namespace chrono;

typedef struct ParticleData {
	long long int id;
	int collision_state;
	double time;
	double radius;
	double pos_x;
	double pos_y;
	double pos_z;
	double rot_0; //Type Real 
	double rot_1; //Type Real 
	double rot_2; //Type Real 
	double rot_3; //Type Real 
	double vel_x;
	double vel_y;
	double vel_z;
	double rot_vel_x;
	double rot_vel_y;
	double rot_vel_z;
	double acc_x;
	double acc_y;
	double acc_z;
	double rot_acc_x;
	double rot_acc_y;
	double rot_acc_z;
	double force_x;
	double force_y;
	double force_z;
	double torque_x;
	double torque_y;
	double torque_z;
} ParticleData;

typedef struct SystemData {
	size_t num_walls = 0;
	size_t num_particles = 0;
	size_t num_bodies = 0;
	size_t num_shapes = 0;
	double sim_timestep = 0.0;
	double sim_duration = 0.0;
	double timer_total_sim = 0.0;
	double timer_total_collision = 0.0;
	double timer_broad_collision = 0.0;
	double timer_narrow_collision = 0.0;
	double timer_custom_collision = 0.0;
	double timer_total_update = 0.0;
	double timer_total_advance = 0.0;
	double timer_calcf_advance = 0.0;
} SystemData;


// Print all contact information for the system in order to generate force chain plots
int PrintContactInfo(ChSystemMulticoreSMC& msystem, const std::string& dpath, const double& time, const bool final);

// Read a file containing ParticleData objects and count the number of objects in the file
size_t CountStateBinObjects(const std::string& path);

// Import data from final state binary file and save in a data array
int ImportStateBin(ParticleData* data, const std::string &fname);

// Write header line of csv files
int WriteCsvHeader(const std::string &fname);

// Search through a given directory and return a vector containing the names of all files with a .bin extension 
std::vector<std::string> GetFileList(const std::string& path, bool bin_only = false);

// Convert all binary files in the given directory to csv files, assuming that binary files consist only of ParticleData objects
int ConvertBinToCsv(const std::string& path);

// Print system information, including number of walls and particles and timer totals
int WriteSystemStats(const SystemData& stats, const std::string& path);

// Write contents of a 2D array of ParticleData structures to a binary file
int WriteBinFile(ParticleData** data, const std::string& fname, const size_t& num_bodies, const int& index);

// Populate a 2D data array with state information for every particle in the system for one timestep index
int StoreData(const ChSystemMulticoreSMC& msystem, ParticleData** data, const size_t& num_bodies, const size_t& start_list, const int& index);

// Store and write data for a single body
int ArchiveSingleBody(const ChSystemMulticoreSMC& msystem, const std::string& path, const size_t& index);

// Store and write data for a single timestep
int ArchiveState(const ChSystemMulticoreSMC& msystem, const SystemData& stats, const std::string& path, const size_t& num_bodies,
	const size_t& start_index, const double& time, bool final = false);

// Keep track of simulation run time by proces type
void IncrementTimerTotals(const ChSystemMulticoreSMC& msystem, SystemData* systats, const double& time);

// Get the total number of shapes in a system
size_t GetNumShapes(const ChSystemMulticoreSMC& msystem);

// Store basic system stats, like number of particles in the system
int SetSystemStats(SystemData& mstats, const ChSystemMulticoreSMC& msystem, const double& timestep, 
	const std::pair<size_t, size_t>& wlist, const std::pair<size_t, size_t>& glist);


#endif