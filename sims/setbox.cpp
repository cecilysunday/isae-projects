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
// Fill a rectangular-shaped container - create a particle cloud, give each
// particle in the cloud an initial velocity in a random direction, and let 
// the particles settle under gravity.
//
// Additional notes:
// - Gravity aligned with z-axis
// - Walls have negetive ID numbers (ID < 0) 
// - Particles have positive ID numbers (ID >= 0)
// 
// =============================================================================

// Header and include files
#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "ProjConfig.h"
#include "ProjCalcs.h"
#include "ProjWriteData.h"
#include "ProjProc.h"
        
#include <random>

// If IRRLICHT is enabled, add irrlicht headers and namespaces
#ifdef CHRONO_IRRLICHT
#include <irrlicht.h>
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

// Other namespace declatartions
using namespace chrono;
using namespace chrono::collision;


void AddSphere(int id, ChSystemMulticoreSMC* msystem, const ConfigParameters& cp,
	double radius, ChVector<> pos, ChVector<> init_v) {
	// Calculate derived parameters
	double mass = cp.grho * (4.0 / 3.0) * CH_C_PI * Pow(radius, 3.0);
	ChVector<> inertia = 0.4 * mass * Pow(radius, 2.0) * ChVector<>(1, 1, 1);

	// Create a spherical body. Set parameters and collision model
	auto body = std::shared_ptr<ChBody>(msystem->NewBody());
	body->SetIdentifier(id);
	body->SetMass(mass);
	body->SetPos(pos);
	body->SetRot(QUNIT);
	body->SetPos_dt(init_v);
	body->SetWvel_par(VNULL);
	body->SetInertiaXX(inertia);
	body->SetBodyFixed(false);
	body->SetCollide(true);

	body->GetCollisionModel()->ClearModel();
	utils::AddSphereGeometry(body.get(), cp.mat_pp, radius);
	body->GetCollisionModel()->BuildModel();

	// Attach a random color to the sphere
	#ifdef CHRONO_IRRLICHT
	auto mvisual = chrono_types::make_shared<ChSphereShape>();
	mvisual->GetSphereGeometry().rad = radius;
	mvisual->SetColor(ChColor(0.0f, 0.28f, 0.67f));
	body->AddVisualShape(mvisual);
	#endif

	// Add the sphere to the system 
	msystem->AddBody(body);

	return;
}


void AddWall(int id, ChSystemMulticoreSMC* msystem, const ConfigParameters &cp,
	ChVector<> size, ChVector<> pos, bool vis) {
	// Create container walls. Set parameters and collision model.
	auto body = std::shared_ptr<ChBody>(msystem->NewBody());
	body->SetIdentifier(id);
	body->SetMass(cp.cmass);
	body->SetPos(pos);
	body->SetRot(QUNIT);
	body->SetBodyFixed(true);
	body->SetCollide(true);

	body->GetCollisionModel()->ClearModel();
	utils::AddBoxGeometry(body.get(), cp.mat_pw, size, VNULL, QUNIT, vis);
	body->GetCollisionModel()->BuildModel();

	// Attach a color to the visible container
	#ifdef CHRONO_IRRLICHT
	auto mvisual = chrono_types::make_shared<ChBoxShape>();
	mvisual->GetBoxGeometry().Size = size;
	mvisual->SetColor(ChColor(0.35f, 0.85f, 0.15f));
	if (vis) body->AddVisualShape(mvisual);
	#endif

	// Add the wall to the system
	msystem->AddBody(body);
}


std::pair<size_t, size_t> FillContainer(ChSystemMulticoreSMC* msystem, const ConfigParameters& cp) {
	// Get start index of particle list
	int id = 0;
	std::pair<size_t, size_t> glist;
	glist.first = msystem->Get_bodylist().size();

	// Define temporary container dimensions
	double temp_height = cp.clength_z * 3.3;
	double marg = cp.grad * 1.5;

	// Calculate offsets required for constructing the particle cloud
	double sft_x = marg;
	double sft_y = 2.0 * marg * sin(CH_C_PI / 3.0);
	double sft_z = marg * Sqrt(4.0 - (1.0 / Pow(cos(CH_C_PI / 6.0), 2.0)));
	double sft_w = marg * tan(CH_C_PI / 6.0);

	// Set the max number of beads along the X, Y, and Z axis
	double numx = ceil(cp.clength_x / (marg * 2.0)) + 1;
	double numy = ceil(cp.clength_y / sft_y) + 1;
	double numz = ceil(temp_height / sft_z) + 1;

	// Create a generator for computing random sphere radius values that follow a normal distribution 
	auto seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator((unsigned int)seed);
	std::normal_distribution<double> distribution(cp.grad, cp.gdia_std*cp.grad);

	// Construct a particle cloud inside of the box limits
	ChVector<> pos_ref = ChVector<>(-cp.clength_x / 2.0 + marg, -cp.clength_y / 2.0 + marg, marg);
	for (int iz = 0; iz < numz; ++iz) {
		for (int iy = 0; iy < numy; ++iy) {
			double posx = sft_x * (iy % 2) + sft_x * ((iz + 1) % 2);
			double posy = sft_y * iy + sft_w * (iz % 2);
			double posz = sft_z * iz;

			ChVector<> pos_next = pos_ref + ChVector<>(posx, posy, posz);
			if (pos_next.z() <= temp_height - marg && pos_next.z() >= marg){
				if (pos_next.y() <= cp.clength_y / 2.0 - marg && pos_next.y() >= -cp.clength_y / 2.0 + marg) {
					for (int ix = 0; ix < numx; ++ix) {
						if (pos_next.x() <= cp.clength_x / 2.0 - marg && pos_next.x() >= -cp.clength_x / 2.0 + marg) {
							AddSphere(id++, msystem, cp, distribution(generator), pos_next, ChRandomXYZ(cp.gvel));
						}
						pos_next += ChVector<>(2.0 * sft_x, 0, 0);
					}
				}
			}
		}
	}

	// Get the end index of the particle list and return
	glist.second = msystem->Get_bodylist().size() - 1;
	return glist;
}


std::pair<size_t, size_t> AddContainer(ChSystemMulticoreSMC* msystem, const ConfigParameters& cp) {
	// Get start index of the wall list
	int id = 0;
	std::pair<size_t, size_t> wlist;
	wlist.first = msystem->Get_bodylist().size();

	// Adjust box dimensions for 'set' sim
	double temph = cp.clength_z * 3.3;
	double tempt = cp.cthickness * 0.999;

	// Add box walls
	ChVector<> sbase = ChVector<>(cp.clength_x, cp.clength_y, tempt) / 2.0;
	ChVector<> srght = ChVector<>(tempt, cp.clength_y, temph + 2.0 * cp.cthickness) / 2.0;
	ChVector<> sback = ChVector<>(cp.clength_x + 2.0 * cp.cthickness, tempt, temph + 2.0 * cp.cthickness) / 2.0;

	ChVector<> pbase = -ChVector<>(0, 0, cp.cthickness) / 2.0;
	ChVector<> proof = ChVector<>(0, 0, temph * 2.0 + cp.cthickness) / 2.0;
	ChVector<> prght = ChVector<>(cp.clength_x + cp.cthickness, 0, temph) / 2.0;
	ChVector<> pleft = -ChVector<>(cp.clength_x + cp.cthickness, 0, -temph) / 2.0;
	ChVector<> pfrnt = ChVector<>(0, cp.clength_y + cp.cthickness, temph) / 2.0;
	ChVector<> pback = -ChVector<>(0, cp.clength_y + cp.cthickness, -temph) / 2.0;

	AddWall(--id, msystem, cp, sbase, pbase, true);
	AddWall(--id, msystem, cp, srght, prght, true);
	AddWall(--id, msystem, cp, srght, pleft, true);
	AddWall(--id, msystem, cp, sback, pback, true);
	AddWall(--id, msystem, cp, sback, pfrnt, false);
	AddWall(--id, msystem, cp, sbase, proof, true);

	// Find and return index range of full wall list 
	wlist.second = msystem->Get_bodylist().size() - 1;
	return wlist;
}


int main(int argc, char* argv[]) {
	// Print Chrono version to userlog
	GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n";

	// Get the location of the config file from user input
	const std::string cfile = GetConfigFile(argc, argv);

	// Initialize simulation. Import parameters from the config file and set input/output directories.
	ConfigParameters cp;
	if (int err = InitializeSimulation(&cp, cfile) != 0) {
		fprintf(stderr, "\nERR: Error configuring simulation at return %i\n", err);
		return -1;
	}

	// Create a Multicore SMC system and set the system parameters
	ChSystemMulticoreSMC msystem;
	if (SetSimulationParameters(&msystem, cp) != 0) {
		fprintf(stderr, "\nERR: Error setting simulation parameters\n");
		return -1;
	}

	// Add the container and grains to the simulation
	std::pair<size_t, size_t> wlist = AddContainer(&msystem, cp);
	std::pair<size_t, size_t> glist = FillContainer(&msystem, cp);

	// Create an object to track certain system info and stats
	SystemData mstats;
	if (SetSystemStats(mstats, msystem, cp.time_step, wlist, glist) != 0) {
		fprintf(stderr, "\nERR: Error setting system stats\n");
		return -1;
	}

	GetLog() << "\nNumber of particles: " << mstats.num_particles;

	// Create and configure the irrlicht visualizer 
	#ifdef CHRONO_IRRLICHT
	auto application = chrono_types::make_shared<ChVisualSystemIrrlicht>();
	application->AttachSystem(&msystem);
	application->SetWindowSize(800, 600);
	application->SetWindowTitle("BOX_SET");
	application->SetCameraVertical(CameraVerticalDir::Z);
	application->Initialize();
	application->AddTypicalLights();
	application->AddSkyBox();
	application->AddCamera(ChVector<>(0, cp.clength_x * 2, cp.clength_z), ChVector<>(0, 0, cp.clength_z));
	application->AddLight(ChVector<>(0, cp.clength_z * 2, cp.clength_z), cp.clength_z * 2);
	#endif

	// Set the soft run-time parameters
	bool settled = false;
	bool flattened = false;

	double time = 0.0;
	double time_loop = cp.time_loop;
	double time_save = cp.time_save;

	// Iterate through the simulation until the time limit is reached.
	while (time < cp.sim_duration) {
		// Start irrlicht visualization scene
		#ifdef CHRONO_IRRLICHT
		application->BeginScene();
		application->Render();
		application->GetDevice()->run();
		application->EndScene();
		#endif

		// Calculate dynamics for (cp.time_loop / time_step) continuous steps  
		while (time < time_loop) {
			msystem.DoStepDynamics(cp.time_step);
			time += cp.time_step;

			IncrementTimerTotals(msystem, &mstats, time);
			if (time > cp.sim_duration) break;
		}
		time_loop = time - cp.time_step + cp.time_loop;

		// Save and write data every (cp.time_save / time_step) steps (particles only, not walls)
		if (time > time_save) {
			if (ArchiveState(msystem, mstats, cp.proj_path, mstats.num_particles, glist.first, time) != 0) {
				fprintf(stderr, "\nERR: Error archiving run-time data \n");
				return -1;
			}
			time_save += cp.time_save;
		}

		// Calculate the average kinetic energy of all particles. If the max particle vel < threshold, exit the loop.
		if (CalcAverageKE(msystem, cp.proj_path, mstats.num_particles, glist.first, time, 1)) break;
	}

	// Export the final state data for all bodies in the system
	if (ArchiveState(msystem, mstats, cp.proj_path, mstats.num_bodies, wlist.first, time, true) != 0) {
		fprintf(stderr, "\nERR: Error archiving final state data \n");
		return -1;
	}

	// Post-process step 1: convert select binaries to .csv format
	if (ConvertBinaries(cp.proj_path) != 0) {
		fprintf(stderr, "\nERR: Could not convert binary files.\n");
		return -1;
	}

	// Post-process step 2: calculate the packing properties of the system
	if (CalcPacking(cp.proj_path, cp) != 0) {
		fprintf(stderr, "\nERR: Could not calculate sim packing stats.\n");
		return -1;
	}

	return 0;
}