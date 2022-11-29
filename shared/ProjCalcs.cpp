// =============================================================================
// 
// Helper functions for placing and tracking particles in a simulation
// 
// =============================================================================

#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "ProjCalcs.h"

#include <fstream>
#include <iostream>
#include <random>

using namespace chrono;



ChVector<> ChRandomXYZ(double minmax) {
	double rx = -minmax / 2 + minmax * ChRandom();
	double ry = -minmax / 2 + minmax * ChRandom();
	double rz = -minmax / 2 + minmax * ChRandom();
	
	return ChVector<>(rx, ry, rz);
}


ChVector<> SeedRandomXYZ(double minmax) {
	auto seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator((unsigned int)seed);
	std::uniform_real_distribution<double> distribution(-minmax, minmax);

	double rx = distribution(generator);
	double ry = distribution(generator);
	double rz = distribution(generator);

	return ChVector<>(rx, ry, rz);
}


double GetSurfaceHeight(ChSystemMulticoreSMC* msystem) {
	double highest = 0;
	for (auto body : msystem->Get_bodylist()) {
		if (body->GetIdentifier() >= 0) {
			double rad = body->GetCollisionModel().get()->GetShapeDimensions(0).at(0);
			double h = body->GetPos().z() + rad;
			if (h > highest) highest = h;
		}
	}

	return highest + 1.0E-5;
}


bool CalcAverageKE(const ChSystemMulticoreSMC &msystem, const std::string &path, 
	const size_t &num, const size_t &start_id, const double &time, const double &threshold) {
	// Create a stream to log file and determine if this is a new file or not
	const std::string fname = path + "/system_energy.txt";

	bool include_header = false;
	if (!filesystem::path(fname).exists()) include_header = true;

	std::ofstream txt_out(fname, std::ofstream::out | std::ofstream::app);

	// If this is the first time that the file is being created, write the header
	if (include_header) txt_out 
		<< "time\t" 
		<< "mean_KE_trn\t" << "mean_KE_rot\t" << "mean_KE\t" 
		<< "rms_vel\t" << "max_vel\n";

	// Find the kinetic energy and sum the particle velocities in the system
	double KE_trn = 0;
	double KE_rot = 0;
	double vel_sum = 0;
	double vel_max = 0;

	int num_adjust = 0;

	for (size_t i = start_id; i < start_id + num; ++i) {
		std::shared_ptr<ChBody> body = msystem.Get_bodylist().at(i);
		
		double vel = body->GetPos_dt().Length();
		ChVector<> trn = 0.5 * body->GetMass() * body->GetPos_dt() * body->GetPos_dt();
		ChVector<> rot = 0.5 * body->GetInertiaXX() * body->GetWvel_par() * body->GetWvel_par();
		
		if (std::isnan(trn.Length()) || std::isnan(rot.Length())) {
			num_adjust += 1;
			GetLog() << "\nWARNING: KE_tot = NAN for particle " << body->GetIdentifier() << " at t = " << time;
		}
		else {
			KE_trn += trn.x() + trn.y() + trn.z();
			KE_rot += rot.x() + rot.y() + rot.z();

			if (vel > vel_max) vel_max = vel;
			vel_sum += vel * vel;
		}
	}

	// Calculate and write the average kinetic energy and velocity
	double KE_trn_avg = KE_trn / (num - num_adjust);
	double KE_rot_avg = KE_rot / (num - num_adjust);
	double KE_tot_avg = KE_trn_avg + KE_rot_avg;
	double RMS_vel = Sqrt(vel_sum / (num - num_adjust));
	txt_out << time << "\t" 
		    << KE_trn_avg << "\t" << KE_rot_avg << "\t" << KE_tot_avg << "\t" 
		    << RMS_vel << "\t" << vel_max << "\n";
	txt_out.close();

	// Return true if the max particle velocity falls below the given threshold
	std::cout<<"Vel_max = "<<vel_max<<", RMS_vel = "<<RMS_vel<<"\n";
	if (vel_max < threshold && RMS_vel < threshold / 100) return true;
	return false;
}