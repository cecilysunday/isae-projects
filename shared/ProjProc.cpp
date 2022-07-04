// =============================================================================
//
// Helper functions for processing simulation data
//
// =============================================================================

#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "ProjConfig.h"
#include "ProjWriteData.h"
#include "ProjProc.h"

#include <iostream> 
#include <string.h>


using namespace chrono;


// Print the packing fraction and bulk density to a file in the specified data directory
void PrintPackingStats(const std::string &configdir, const std::string &units, const PackingData &packing) {
	const std::string new_file = configdir + "/system_packing.txt";
	if (filesystem::path(new_file).exists()) filesystem::path(new_file).remove_file();

	std::ofstream txt_out(new_file, std::ofstream::out | std::ofstream::app);
	txt_out << "\n" << "# Project packing and density statistics"
		<< "\n" << "input_file " << packing.input_file
		<< "\n" << "units " << units
		<< "\n" << "container_shape " << packing.container_shape
		<< "\n" << "container_volume " << packing.container_volume
		<< "\n" << "xlim_min " << packing.xlim_min
		<< "\n" << "xlim_max " << packing.xlim_max
		<< "\n" << "ylim_min " << packing.ylim_min
		<< "\n" << "ylim_max " << packing.ylim_max
		<< "\n" << "zlim_min " << packing.zlim_min
		<< "\n" << "zlim_max " << packing.zlim_max
		<< "\n" << "num_grains " << packing.num_grains
		<< "\n" << "grain_mass " << packing.grain_mass
		<< "\n" << "grain_volume " << packing.grain_volume
		<< "\n" << "packing_fraction " << packing.packing_fraction
		<< "\n" << "bulk_density " << packing.bulk_density;
	txt_out.close();
}


// Verify that the simulation has either cylinder or box geometry. Otherwise, the packing fraction cannot be determined.
std::string GetContainerShape(const std::string& proj_container) {
	try {
		if (proj_container != "box" & proj_container != "cyl") throw 42;
	}
	catch (int) {
		fprintf(stderr, "\nERR: Configuration file contains an invalid container geometry! Container must be a box or a cyl.\n");
		exit(EXIT_FAILURE);
	}

	return proj_container;
}


// Return the name of first bin file in a directory for a 'run' simulation and the name of the last bin file for a 'set' simulation
std::string GetFileToProcess(const std::string& configdir, const std::string& proj_type) {
	std::string ifile;
	try {
		std::vector<std::string> bin_list = GetFileList(configdir, true);
		if (bin_list.size() == 0) throw 42;
		else {
			std::vector<std::string>::iterator i = bin_list.begin();
			if (proj_type == "setbox" || proj_type == "setcyl") {
				i = bin_list.end() - 1;
			}
			ifile = *i;
		}
	}
	catch (int) {
		fprintf(stderr, "\nERR: Directory does not contain any binary files.\n");
		exit(EXIT_FAILURE);
	}

	return ifile;
}


// Calculate packing fraction and bulk density based on simulation configuration
int CalcPackingStats(const std::string& configdir, const ConfigParameters& config, PackingData* packing) {
	/// Get the file that should be processed and the container shape
	packing->input_file = GetFileToProcess(configdir, config.proj_type);
	packing->container_shape = GetContainerShape(config.proj_container);
	
	/// Import the particle data from the appropriate bin file
	size_t num = CountStateBinObjects(packing->input_file);
	ParticleData* data = new ParticleData[num];
	if (ImportStateBin(data, packing->input_file) != 0) {
		fprintf(stderr, "\nERR: Could not import data file.\n");
		return -1;
	}

	/// Initialize sizing variables
	std::pair<double, double> lim_x(0, 0);
	std::pair<double, double> lim_y(0, 0);
	std::pair<double, double> lim_z(0, 0);
	std::pair<double, double> lim_r(0, 0);

	/// Find the limits of the container and the mass, volume, and number of grains in the system
	for (size_t i = 0; i < num; ++i) {
		if (data[i].id >= 0) {
			double rad = data[i].radius;
			double pos_x = data[i].pos_x;
			double pos_y = data[i].pos_y;
			double pos_z = data[i].pos_z;
			double pos_r = Sqrt(Pow(pos_x, 2.0) + Pow(pos_y, 2.0));

			if (pos_x - rad < lim_x.first) lim_x.first = pos_x - rad;
			if (pos_x + rad > lim_x.second) lim_x.second = pos_x + rad;
			if (pos_y - rad < lim_y.first) lim_y.first = pos_y - rad;
			if (pos_y + rad > lim_y.second) lim_y.second = pos_y + rad;
			if (pos_z - rad < lim_z.first) lim_z.first = pos_z - rad;
			if (pos_z + rad > lim_z.second) lim_z.second = pos_z + rad;
			if (pos_r + rad > lim_r.second) lim_r.second = pos_r + rad;

			double vol = (4.0 / 3.0 * CH_C_PI * rad * rad * rad);

			packing->num_grains += 1;
			packing->grain_volume += vol;
			packing->grain_mass += vol * config.grho;
		}
	}

	packing->xlim_min = lim_x.first;
	packing->xlim_max = lim_x.second;
	packing->ylim_min = lim_y.first;
	packing->ylim_max = lim_y.second;
	packing->zlim_min = lim_z.first;
	packing->zlim_max = lim_z.second;
	
	// Find container volume based on container shape
	if (packing->container_shape == "box") packing->container_volume = (lim_x.second - lim_x.first) * (lim_y.second - lim_y.first) * (lim_z.second - lim_z.first);
	if (packing->container_shape == "cyl") packing->container_volume = (lim_z.second - lim_z.first) * Pow(lim_r.second, 2.0) * CH_C_PI;

	// Find bulk density and packing fraction excluding the top surface of the grains bed
	double grain_mass_int = 0;
	double grain_volume_int = 0;
	double container_volume_int = (lim_x.second - lim_x.first) * (lim_y.second - lim_y.first) * (lim_z.second - lim_z.first - 2.0 * config.grad);

	for (size_t i = 0; i < num; ++i) {
		if (data[i].id >= 0) {
			double rad = data[i].radius;
			double pos_z = data[i].pos_z;

			if (pos_z + rad <= lim_z.second - 2.0 * config.grad) {
				double vol = (4.0 / 3.0) * CH_C_PI * rad * rad * rad;
				grain_volume_int += vol;
				grain_mass_int += vol * config.grho;
			}

		}
	}

	packing->packing_fraction = grain_volume_int / container_volume_int;
	packing->bulk_density = grain_mass_int / container_volume_int;

	return 0;
}


// Wrapper function to calculate and print the packing configuration of a simulation
int CalcPacking(const std::string &configdir, const ConfigParameters& cp) {
	/// Calculate packing stats using particle positions and configuration file
	PackingData packing;
	if (CalcPackingStats(configdir, cp, &packing) != 0) {
		fprintf(stderr, "\nERR: Could not calculate packing for given project.\n");
		return -1;
	}
	else {
		fprintf(stdout, "\nSucessfully determined simulation packing statistics\n");
	}

	/// Print packing stats
	PrintPackingStats(configdir, cp.proj_units, packing);

	return 0;
}


// Convert first and last binary files in a folder to csv format
int ConvertBinaries(const std::string & configdir) {
	int convert_base = ConvertBinToCsv(configdir);
	if (convert_base > 0) {
		fprintf(stdout, "\nConverted %i binary files to CSV format\n", convert_base);
	} 
	else {
		fprintf(stderr, "\nERR: Error converting binary files to CSV format\n");
		return -1;
	}
	return 0;
}