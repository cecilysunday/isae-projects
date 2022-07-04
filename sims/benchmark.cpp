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
// Fill a rectangular-shaped container - create a particle cloud and let 
// the particles settle under gravity with no mixing. This is a benchmarking test
// that can be used to verify that the performance of Chrono::Multicore has not
// degraded during recent code updates.
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

// Other namespace declatartions
using namespace chrono;
using namespace chrono::collision;



// Change the container size to ajust the number of particles in the system
// I do not change any of the other parameters, and I set the number of threads
// directly in my cluster SLURM script
// 
// 0220 x 0220 x 220 container = 007,702 particles when gdia = 10
// 0315 x 0315 x 220 container = 016,670 particles when gdia = 10
// 0500 x 0500 x 300 container = 058,830 particles when gdia = 10
// 0750 x 0750 x 300 container = 134,766 particles when gdia = 10
// 1000 x 1000 x 300 container = 239,862 particles when gdia = 10
// 1500 x 1500 x 300 container = 545,478 particles when gdia = 10
// 2000 x 2000 x 300 container = 971,262 particles when gdia = 10
double clength = 500;
double cwidth = 500;
double cheight = 300;
double gdia = 10;
double grho = 0.00248;


void AddSphere(int id, ChSystemMulticoreSMC* msystem, std::shared_ptr<ChMaterialSurfaceSMC> mat,
	double radius, ChVector<> pos) {
	// Define certain shared properties and calculate derived parameters
	double mass = grho * (4.0 / 3.0) * CH_C_PI * Pow(radius, 3.0);
	ChVector<> inertia = 0.4 * mass * Pow(radius, 2.0) * ChVector<>(1, 1, 1);

	// Create a spherical body. Set parameters and collision model
	auto body = std::shared_ptr<ChBody>(msystem->NewBody());
	body->SetIdentifier(id);
	body->SetMass(mass);
	body->SetPos(pos);
	body->SetRot(QUNIT);
	body->SetPos_dt(VNULL);
	body->SetWvel_par(VNULL);
	body->SetInertiaXX(inertia);
	body->SetBodyFixed(false);
	body->SetCollide(true);

	body->GetCollisionModel()->ClearModel();
	utils::AddSphereGeometry(body.get(), mat, radius);
	body->GetCollisionModel()->BuildModel();

	// Add the sphere to the system 
	msystem->AddBody(body);
	return;
}


void AddWall(int id, ChSystemMulticoreSMC* msystem, std::shared_ptr<ChMaterialSurfaceSMC> mat,
	ChVector<> size, ChVector<> pos) {
	// Create container walls. Set parameters and collision model
	auto body = std::shared_ptr<ChBody>(msystem->NewBody());
	body->SetIdentifier(id);
	body->SetMass(1000);
	body->SetPos(pos);
	body->SetRot(QUNIT);
	body->SetBodyFixed(true);
	body->SetCollide(true);

	body->GetCollisionModel()->ClearModel();
	utils::AddBoxGeometry(body.get(), mat, size, VNULL, QUNIT);
	body->GetCollisionModel()->BuildModel();

	// Add the wall to the system
	msystem->AddBody(body);
	return;
}


void FillContainer(ChSystemMulticoreSMC* msystem, std::shared_ptr<ChMaterialSurfaceSMC> mat) {
	// Define temporary container dimensions
	int id = 0;
	double grad = gdia / 2.0;
	double marg = grad * 1.5;
	double temp_height = cheight * 2.0;

	// Calculate offsets required for constructing the particle cloud
	double sft_x = marg;
	double sft_y = 2.0 * marg * sin(CH_C_PI / 3.0);
	double sft_z = marg * Sqrt(4.0 - (1.0 / Pow(cos(CH_C_PI / 6.0), 2.0)));
	double sft_w = marg * tan(CH_C_PI / 6.0);

	// Set the max number of beads along the X, Y, and Z axis
	double numx = ceil(clength / (marg * 2.0)) + 1;
	double numy = ceil(cwidth / sft_y) + 1;
	double numz = ceil(temp_height / sft_z) + 1;

	// Construct a particle cloud inside of the box limits
	ChVector<> pos_ref = ChVector<>(-clength / 2.0 + marg, -cwidth / 2.0 + marg, marg);
	for (int iz = 0; iz < numz; ++iz) {
		for (int iy = 0; iy < numy; ++iy) {
			double posx = sft_x * (iy % 2) + sft_x * ((iz + 1) % 2);
			double posy = sft_y * iy + sft_w * (iz % 2);
			double posz = sft_z * iz;

			ChVector<> pos_next = pos_ref + ChVector<>(posx, posy, posz);
			if (pos_next.z() <= temp_height - marg && pos_next.z() >= marg){
				if (pos_next.y() <= cwidth / 2.0 - marg && pos_next.y() >= -cwidth / 2.0 + marg) {
					for (int ix = 0; ix < numx; ++ix) {
						if (pos_next.x() <= clength / 2.0 - marg && pos_next.x() >= -clength / 2.0 + marg) {
							AddSphere(id++, msystem, mat, grad, pos_next);
						}
						pos_next += ChVector<>(2.0 * sft_x, 0, 0);
					}
				}
			}
		}
	}

	return;
}


void AddContainer(ChSystemMulticoreSMC* msystem, std::shared_ptr<ChMaterialSurfaceSMC> mat) {
	// Adjust box dimensions for 'set' sim
	int id = 0;
	double cthickness = 10;
	double temph = 2.0 * cheight;
	double tempt = cthickness * 0.999;

	// Add box walls
	ChVector<> sbase = ChVector<>(clength, cwidth, tempt) / 2.0;
	ChVector<> srght = ChVector<>(tempt, cwidth, temph + 2.0 * cthickness) / 2.0;
	ChVector<> sback = ChVector<>(clength + 2.0 * cthickness, tempt, temph + 2.0 * cthickness) / 2.0;

	ChVector<> pbase = -ChVector<>(0, 0, cthickness) / 2.0;
	ChVector<> proof = ChVector<>(0, 0, temph * 2.0 + cthickness) / 2.0;
	ChVector<> prght = ChVector<>(clength + cthickness, 0, temph) / 2.0;
	ChVector<> pleft = -ChVector<>(clength + cthickness, 0, -temph) / 2.0;
	ChVector<> pfrnt = ChVector<>(0, cwidth + cthickness, temph) / 2.0;
	ChVector<> pback = -ChVector<>(0, cwidth + cthickness, -temph) / 2.0;

	AddWall(--id, msystem, mat, sbase, pbase);
	AddWall(--id, msystem, mat, srght, prght);
	AddWall(--id, msystem, mat, srght, pleft);
	AddWall(--id, msystem, mat, sback, pback);
	AddWall(--id, msystem, mat, sback, pfrnt);
	AddWall(--id, msystem, mat, sbase, proof);

	return;
}


void SetSimulationParameters(ChSystemMulticoreSMC* msystem) {
	// Solver and integrator settings
	msystem->GetSettings()->solver.max_iteration_bilateral = 100;
	msystem->GetSettings()->solver.tolerance = 0.001;
	msystem->GetSettings()->solver.min_roll_vel = 1e-05;
	msystem->GetSettings()->solver.min_spin_vel = 1e-05;
	msystem->GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::Hertz;
	msystem->GetSettings()->solver.adhesion_force_model = ChSystemSMC::AdhesionForceModel::Constant;
	msystem->GetSettings()->solver.tangential_displ_mode = ChSystemSMC::TangentialDisplacementModel::MultiStep;

	// Collision detection parameters and gravity
	int bpa_x = (int)std::ceil(clength / gdia / 3.0);
	int bpa_y = (int)std::ceil(cwidth / gdia / 3.0);
	int bpa_z = (int)std::ceil(cheight / gdia / 3.0);

	msystem->SetCollisionSystemType(ChCollisionSystemType::CHRONO);
	msystem->GetSettings()->collision.bins_per_axis = vec3(bpa_x, bpa_y, bpa_z);
	msystem->GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;
	msystem->Set_G_acc(ChVector<>(0, 0, -9810));
	
	return;
}


int main(int argc, char* argv[]) {
	// Print Chrono version to userlog
	GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n";

	// Create a Multicore SMC system and set the system parameters
	ChSystemMulticoreSMC msystem;
	SetSimulationParameters(&msystem);

	// Create a shared material of the container and particles
	auto mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
	mat->SetYoungModulus(7.0e7f);
	mat->SetPoissonRatio(0.24f);
	mat->SetSfriction(0.45f);
	mat->SetKfriction(0.45f);
	mat->SetRollingFriction(0.09f);
	mat->SetSpinningFriction(0.01f);
	mat->SetRestitution(0.5f);
	mat->SetAdhesion(0.0f);

	// Add the container and grains to the simulation
	AddContainer(&msystem, mat);
	FillContainer(&msystem, mat);

	// Set the soft run-time parameters
	double time = 0.0;
	double time_step = 5.0E-6;
	double time_sim = 0.5;

	double timer_sim = 0;
	double timer_total_collision = 0;
	double timer_broad_collision = 0;
	double timer_narrow_collision = 0;
	double timer_total_update = 0;
	double timer_total_advance = 0;
	double timer_calcf_advance = 0;

	// Iterate through the simulation until the time limit is reached.
	while (time < time_sim) {
		msystem.DoStepDynamics(time_step);
		time += time_step;

		timer_sim += msystem.GetTimerStep();
		timer_total_collision += msystem.GetTimerCollision();
		timer_broad_collision += msystem.GetTimerCollisionBroad();
		timer_narrow_collision += msystem.GetTimerCollisionNarrow();
		timer_total_update += msystem.GetTimerUpdate();
		timer_total_advance += msystem.GetTimerAdvance();
		timer_calcf_advance += msystem.GetTimerProcessContact();
	}

	// Print the timer info to the log file
	GetLog() << "\n" << "## Project run-time statistics"
			 << "\n" << "num_bodies " << (int)msystem.Get_bodylist().size()
			 << "\n" << "timer_total_sim " << timer_sim
			 << "\n" << "timer_total_collision " << timer_total_collision
			 << "\n" << "timer_broad_collision " << timer_broad_collision
			 << "\n" << "timer_narrow_collision " << timer_narrow_collision
			 << "\n" << "timer_total_update " << timer_total_update
			 << "\n" << "timer_total_advance " << timer_total_advance
			 << "\n" << "timer_calcf_advance " << timer_calcf_advance;

	return 0;
}