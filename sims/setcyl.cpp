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
// Fill a cylindrical container - create a particle cloud, give each
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
#include "chrono_irrlicht/ChIrrApp.h"
using namespace chrono::irrlicht;
#endif

// Other namespace declatartions
using namespace chrono;
using namespace chrono::collision;
using namespace chrono::geometry;



// Custom collision detection callback class.
// One instance of this class handles potential collision between a given sphere and the cylinder
// by manually adding one or two contacts to the system for the current timestep.
// The 'OnCustomCollision function is called at each simulation timestep, for every instance of the class (i.e. for every sphere). 
class MyCustomCollisionDetection : public ChSystem::CustomCollisionCallback {
public:
	MyCustomCollisionDetection(std::shared_ptr<ChBody> sph,
							   std::shared_ptr<ChBody> cyl,
							   std::shared_ptr<ChMaterialSurface> sph_mat,
							   std::shared_ptr<ChMaterialSurface> cyl_mat,
							   double sph_rad,
							   double cyl_rad)
		: m_sph(sph),
		  m_cyl(cyl),
		  m_sph_mat(sph_mat),
		  m_cyl_mat(cyl_mat),
		  m_sph_rad(sph_rad),
		  m_cyl_rad(cyl_rad) {}

	virtual void OnCustomCollision(ChSystem* msys) override {
		// Get the current ball and cylinder positions
		ChVector<> sph_pos = m_sph->GetPos();
		ChVector<> cyl_pos = m_cyl->GetPos();
		ChVector<> delta = sph_pos - cyl_pos;

		// If the sphere is not in contact with the sides of cylinder or its base, return
		double dist_hplane = Sqrt(Pow(delta.x(), 2.0) + Pow(delta.y(), 2.0));
		if (dist_hplane + m_sph_rad < m_cyl_rad) return;

		// Find spheres contacting the sides of the cylinder
		if (dist_hplane + m_sph_rad >= m_cyl_rad) {
			// Find collision points on the ball and the sides of the cylinder
			ChVector<> normal_side = ChVector<>(delta.x() / dist_hplane, delta.y() / dist_hplane, 0.0);
			ChVector<> pt_sph_side = ChVector<>(sph_pos.x(), sph_pos.y(), sph_pos.z()) + m_sph_rad * normal_side;
			ChVector<> pt_cyl_side = ChVector<>(cyl_pos.x(), sph_pos.y(), cyl_pos.z()) + m_cyl_rad * normal_side;

			// Populate the collision info object (express all vectors in 3D). We pass null pointers to collision shapes.
			ChCollisionInfo contact;
			contact.modelA = m_sph->GetCollisionModel().get();
			contact.modelB = m_cyl->GetCollisionModel().get();
			contact.shapeA = nullptr;
			contact.shapeB = nullptr;
			contact.vN = normal_side;
			contact.vpA = pt_sph_side;
			contact.vpB = pt_cyl_side;
			contact.distance = m_cyl_rad - (dist_hplane + m_sph_rad);
			contact.eff_radius = m_sph_rad;
			msys->GetContactContainer()->AddContact(contact, m_sph_mat, m_cyl_mat);
		}
	}

	double GetSphereRadius() { return m_sph_rad; }
	double GetCylinderRadius() { return m_cyl_rad; }
	std::shared_ptr<ChBody> GetSphereBody() { return m_sph; }
	std::shared_ptr<ChBody> GetCylinderBody() { return m_cyl; }

private:
	std::shared_ptr<ChBody> m_sph;
	std::shared_ptr<ChBody> m_cyl;
	std::shared_ptr<ChMaterialSurface> m_sph_mat;
	std::shared_ptr<ChMaterialSurface> m_cyl_mat;
	double m_sph_rad;
	double m_cyl_rad;
};


std::shared_ptr<MyCustomCollisionDetection> AddSphere(int id, ChSystemMulticoreSMC* msystem, const ConfigParameters& cp,
	double radius, ChVector<> pos, ChVector<> init_v, std::shared_ptr<ChBody> cwall) {
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
	std::shared_ptr<ChColorAsset> mvisual = chrono_types::make_shared<ChColorAsset>();
	mvisual->SetColor(ChColor(0.0f, 0.28f, 0.67f));
	body->AddAsset(mvisual);
	#endif

	// Add the sphere to the system 
	msystem->AddBody(body);

	// Register the custom collision for the sphere
	auto collision_ptr = chrono_types::make_shared<MyCustomCollisionDetection>(body, cwall, cp.mat_pp, cp.mat_pw, radius, cp.clength_r);
	msystem->RegisterCustomCollisionCallback(collision_ptr);
	return collision_ptr;
}


void AddWall(int id, ChSystemMulticoreSMC* msystem, const ConfigParameters &cp, ChVector<> size, ChVector<> pos, bool sidewall, bool vis) {
	// Adjust switch values for sidewalls
	bool collide = true;
	bool rings = vis;
	if (sidewall) {
		collide = false;
		vis = false;
	}

	// Create container walls. Set parameters and collision model.
	auto body = std::shared_ptr<ChBody>(msystem->NewBody());
	body->SetIdentifier(id);
	body->SetMass(cp.cmass);
	body->SetPos(pos);
	body->SetRot(Q_ROTATE_Y_TO_Z);
	body->SetBodyFixed(true);
	body->SetCollide(collide);

	body->GetCollisionModel()->ClearModel();
	utils::AddCylinderGeometry(body.get(), cp.mat_pw, size.x(), size.z(), VNULL, QUNIT, vis);
	body->GetCollisionModel()->BuildModel();

	// Attach a color to the visible container
	#ifdef CHRONO_IRRLICHT
	if (sidewall & rings) {
		int rnum = 10;
		double rgap = size.z() * 2.0 / rnum;
		for (int i = 0; i <= rnum; i++) {
			ChLineArc circle(ChCoordsys<>(ChVector<>(pos.x(), pos.z() - 2.0 * size.z() + i*rgap, pos.y()), Q_ROTATE_Y_TO_Z), size.x());
			auto mpath = chrono_types::make_shared<ChPathShape>();
			mpath->GetPathGeometry()->AddSubLine(circle);
			mpath->SetColor(ChColor(0.35f, 0.85f, 0.15f));
			body->AddAsset(mpath);
		}
	}
	else {
		auto mvisual = chrono_types::make_shared<ChColorAsset>();
		mvisual->SetColor(ChColor(0.35f, 0.85f, 0.15f));
		body->AddAsset(mvisual);
	}
	#endif

	// Add the wall to the system
	msystem->AddBody(body);
}


std::pair<size_t, size_t> FillContainer(ChSystemMulticoreSMC* msystem, const ConfigParameters& cp,
	std::vector<std::shared_ptr<MyCustomCollisionDetection>>* my_collisions_ptr) {
	// Get start index of particle list
	int id = 0;
	std::pair<size_t, size_t> glist;
	glist.first = msystem->Get_bodylist().size();

	// Get the cylinder wall body, assumin the index of the wall is 1 (0 = base, 1 = wall, 2 = roof)
	std::shared_ptr<ChBody> cwall = msystem->Get_bodylist().at(1);
	
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
	std::normal_distribution<double> distribution(cp.grad, cp.gmarg / 6.0);

	// Construct a particle cloud inside of the box limits
	ChVector<> pos_ref = ChVector<>(-cp.clength_x / 2.0 + marg, -cp.clength_y / 2.0 + marg, marg);
	for (int iz = 0; iz < numz; ++iz) {
		for (int iy = 0; iy < numy; ++iy) {
			double posx = sft_x * (iy % 2) + sft_x * ((iz + 1) % 2);
			double posy = sft_y * iy + sft_w * (iz % 2);
			double posz = sft_z * iz;

			ChVector<> pos_next = pos_ref + ChVector<>(posx, posy, posz);
			if (pos_next.z() <= temp_height - marg && pos_next.z() >= marg){
				for (int ix = 0; ix < numx; ++ix) {
					if (Sqrt(Pow(pos_next.x(), 2) + Pow(pos_next.y(), 2)) < cp.clength_r - marg) {
						my_collisions_ptr->push_back(AddSphere(id++, msystem, cp, distribution(generator), pos_next, ChRandomXYZ(cp.gvel), cwall));
					}
					pos_next += ChVector<>(2.0 * sft_x, 0, 0);
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

	// Adjust cylinder dimensions for 'set' sim
	double temph = cp.clength_z * 3.3;

	// Add cylinder base, rool, and walls
	ChVector<> sbase = ChVector<>(cp.clength_r, cp.clength_r, cp.cthickness / 2.0);
	ChVector<> swall = ChVector<>(cp.clength_r, cp.clength_r, temph / 2.0);
	ChVector<> sroof = ChVector<>(cp.clength_r, cp.clength_r, cp.cthickness / 2.0);

	ChVector<> pbase = ChVector<>(0, 0, -cp.cthickness) / 2.0;
	ChVector<> pwall = ChVector<>(0, 0, temph) / 2.0;
	ChVector<> proof = ChVector<>(0, 0, temph * 2.0 + cp.cthickness) / 2.0;

	AddWall(--id, msystem, cp, sbase, pbase, false, true);
	AddWall(--id, msystem, cp, swall, pwall, true, true);
	AddWall(--id, msystem, cp, sbase, proof, false, true);

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

	// Initialize the list of all custom collisions
	std::vector<std::shared_ptr<MyCustomCollisionDetection>> my_collisions_ptr;

	// Add the container and grains to the simulation
	std::pair<size_t, size_t> wlist = AddContainer(&msystem, cp);
	std::pair<size_t, size_t> glist = FillContainer(&msystem, cp, &my_collisions_ptr);

	// Create an object to track certain system info and stats
	SystemData mstats;
	if (SetSystemStats(mstats, msystem, cp.time_step, wlist, glist) != 0) {
		fprintf(stderr, "\nERR: Error setting system stats\n");
		return -1;
	}

	// Create and configure the irrlicht visualizer 
	#ifdef CHRONO_IRRLICHT
	ChIrrApp application(&msystem, L"CYL_SET", irr::core::dimension2d<irr::u32>(800, 600), VerticalDir::Z);
	application.AddTypicalSky();
	application.AddTypicalLights();
	application.AddTypicalCamera(irr::core::vector3df(0, cp.clength_x * 2, cp.clength_z), irr::core::vector3df(0, 0, cp.clength_z));
	application.AssetUpdateAll();
	#endif

	// Set the soft run-time parameters
	double time = 0.0;
	double time_loop = cp.time_loop;
	double time_save = cp.time_save;

	// Iterate through the simulation until the time limit is reached.
	while (time < cp.sim_duration) {
		// Start irrlicht visualization scene
		#ifdef CHRONO_IRRLICHT
		application.BeginScene(true, true);
		application.GetDevice()->run();
		application.DrawAll();
		application.EndScene();
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