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

void AddSphereWall(int id, ChSystemMulticoreSMC* msystem, const ConfigParameters& cp,
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
	body->SetPos_dt(ChVector<>(0));
	body->SetWvel_par(VNULL);
	body->SetInertiaXX(inertia);
	body->SetBodyFixed(true);
	body->SetCollide(true);

	body->GetCollisionModel()->ClearModel();
	utils::AddSphereGeometry(body.get(), cp.mat_pp, radius);
	body->GetCollisionModel()->BuildModel();

	// Attach a random color to the sphere
	#ifdef CHRONO_IRRLICHT
	auto mvisual = chrono_types::make_shared<ChSphereShape>();
	mvisual->GetSphereGeometry().rad = radius;
	mvisual->SetColor(ChColor(0.35f, 0.85f, 0.15f));
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

	// Get the cylinder wall body, assumin the index of the wall is 1 (0 = base, 1 = wall, 2 = roof)
	std::shared_ptr<ChBody> cwall = msystem->Get_bodylist().at(1);
	
	// Define temporary container dimensions
	double temp_height = cp.clength_z * 3.3;
	double marg = cp.grad * 1.5;

	double grad = cp.grad;
	double sdia = cp.funnel_small_dia;
	double shgt = cp.height_stem;
	double bdia = cp.funnel_large_dia ;
	double rang = cp.angle_funnel*CH_C_PI/180;
	double rlength = (bdia - sdia) / (2.0 * cos(rang));
	double dist_funnel_platform=cp.dist_funnel_platform; //ISO : 75
	double size_platform =cp.platform_size; //ISO : 100

	int nb_particles = round(0.25*pow(size_platform/2.0/grad,3)*tan(CH_C_PI/3.0));
	double brad=bdia/2.0;
	
	double bhgt=(2*pow(dist_funnel_platform,3))/(3*pow(brad*tan(rang),2));
	double crad_stem = sdia / 2.0 + grad;									/// Estimated radius of funnel stem particles-ring
	double crad_body = bdia / 2.0 + grad;									/// Estimated radius of funnel body particles-ring

	double sftz = grad * sqrt(4.0 - (1.0 / pow(cos(CH_C_PI / 6.0), 2.0)));	/// Shift between z-layers
	double numd_stem = round(CH_C_PI / asin(grad / crad_stem));				/// Num particles needed to construct funnel stem particle-ring
	double numd_body = round(CH_C_PI / asin(grad / crad_body));				/// Num particles needed to construct funnel stem particle-ring
	double numh_stem = round((shgt - 2.0 * grad) / sftz + 1);				/// Num particles needed to preserve the funnel stem height
	
	double numl_ramp = round(rlength / (2.0 * grad));						/// Num particles along the ramped portion of the funnel

	crad_stem = grad / sin(CH_C_PI / numd_stem);							/// Actual radius of funnel stem particle-ring
	crad_body = grad / sin(CH_C_PI / numd_body);							/// Actual radius of funnel body particle-ring


	// Calculate offsets required for constructing the particle cloud
	double sft_x = marg;
	double sft_y = 2.0 * marg * sin(CH_C_PI / 3.0);
	double sft_z = marg * Sqrt(4.0 - (1.0 / Pow(cos(CH_C_PI / 6.0), 2.0)));
	double sft_w = marg * tan(CH_C_PI / 6.0);

	// Set the max number of beads along the X, Y, and Z axis
	double numx = ceil(2*crad_body / (marg * 2.0)) + 1;
	double numy = ceil(2*crad_body / sft_y) + 1;
	double numz = ceil(nb_particles/(numx*numy)) + 1;

	std::cout<<"numx = "<<numx<<", numy = "<<numy<<", numz = "<<numz<<"\n";
	// Create a generator for computing random sphere radius values that follow a normal distribution 
	auto seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator((unsigned int)seed);
	std::normal_distribution<double> distribution(cp.grad, cp.gmarg / 6.0);

	double height=cp.dist_funnel_platform+(numh_stem-1)*sftz+grad+2.0*grad*sin(rang)*(numl_ramp-1)+2.0*grad*cos(rang);

	// Construct a particle cloud inside of the box limits
	ChVector<> pos_ref = ChVector<>(-crad_body + marg, -crad_body + marg, height);
	for (int iz = 0; iz < numz; ++iz) {
		for (int iy = 0; iy < numy; ++iy) {
			double posx = sft_x * (iy % 2) + sft_x * ((iz + 1) % 2);
			double posy = sft_y * iy + sft_w * (iz % 2);
			double posz = sft_z * iz;

			ChVector<> pos_next = pos_ref + ChVector<>(posx, posy, posz);
			if (pos_next.z() >= height){
				for (int ix = 0; ix < numx; ++ix) {
					if (Sqrt(Pow(pos_next.x(), 2) + Pow(pos_next.y(), 2)) < crad_body - marg) {
						AddSphere(id++, msystem, cp, distribution(generator), pos_next, ChRandomXYZ(cp.gvel));
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

std::pair<size_t, size_t> AddPlatform(ChSystemMulticoreSMC* msystem, const ConfigParameters &cp,const std::string cfile){
	std::pair<size_t, size_t> plist;
	plist.first = msystem->Get_bodylist().size();

	int id =-1;

	double marg=1.5*cp.grad;
	double sft_x = marg;
	double sft_y = 2.0 * marg * sin(CH_C_PI / 3.0);
	double sft_z = marg * Sqrt(4.0 - (1.0 / Pow(cos(CH_C_PI / 6.0), 2.0)));
	double sft_w = marg * tan(CH_C_PI / 6.0);

	// Set the max number of beads along the X, Y, and Z axis
	double numx = ceil(2*cp.platform_size / (marg * 2.0)) + 1;
	double numy = ceil(2*cp.platform_size / sft_y) + 1;

	// Create a generator for computing random sphere radius values that follow a normal distribution 
	auto seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator((unsigned int)seed);
	std::normal_distribution<double> distribution(cp.grad, cp.gmarg / 6.0);


	// Construct a particle cloud inside of the box limits
	ChVector<> pos_ref = ChVector<>(-cp.platform_size + marg, -cp.platform_size + marg, 0);

	for (int iy = 0; iy < numy; ++iy) {
		double posx = sft_x * (iy % 2) + sft_x ;
		double posy = sft_y * iy ;
		double posz = 0;

		ChVector<> pos_next = pos_ref + ChVector<>(posx, posy, posz);

		for (int ix = 0; ix < numx; ++ix) {
			if (Sqrt(Pow(pos_next.x(), 2) + Pow(pos_next.y(), 2)) < cp.platform_size - marg) {
				AddSphereWall(id++, msystem, cp, distribution(generator), pos_next, ChRandomXYZ(cp.gvel));
			}
			pos_next += ChVector<>(2.0 * sft_x, 0, 0);
		}
		
	}
	


	// const std::string cfolder = cfile.substr(0,cfile.rfind("/")+1);
	// const std::string path_platform_config_file = cfolder+cp.config_platform_file;
	// std::cout<<"cfolder = "<<path_platform_config_file<<"\n";

	// std::ifstream file(path_platform_config_file);
	// if (!file) {
	// 	fprintf(stderr, "\nERR: Platform file does not exist!\n");
	// }
	// std::string line;
  
	// std::vector<double> platform_x_pos, platform_y_pos,platform_rad;
	// while (std::getline(file, line)) {
	// 	// Ignore empy line or lines that start with #. Otherwise, parse words.
	// 	if (line.empty()) continue;

	// 	// Parse each line into a vector of words
	// 	std::vector<std::string> parsed_line;
	// 	std::vector<std::string> parsed_word;

	// 	char* cline = const_cast<char*>(line.c_str());
	// 	char* delim = strtok(cline, " ");
	// 	while (delim != NULL) {
	// 		std::string parsed_word(delim);
	// 		parsed_line.push_back(parsed_word);
	// 		delim = strtok(NULL, " ");
	// 		// std::cout<<"delim = "<<delim<<"\n";
	// 	}
	// 	if (parsed_line.size() != 3) {
	// 		std::cout<<"Problem for reading platform file \n"; 
	// 	}

	// 	platform_x_pos.push_back(std::stod(parsed_line[0]));
	// 	platform_y_pos.push_back(std::stod(parsed_line[1]));
	// 	platform_rad.push_back(std::stod(parsed_line[2]));

	// }

	// int n=platform_x_pos.size();



	// for(int i=0;i<n;i++){
	// 	std::cout<<"id dans addplatform = "<<id<<"\n";
	// 	double x = platform_x_pos[i];
	// 	double y = platform_y_pos[i];
	// 	double radius = platform_rad[i];
	// 	// utils::AddSphereGeometry(platform.get(), cp.mat_pp, radius, ChVector<>(x, y, 0), ChQuaternion<>(1, 0, 0, 0), true);
	// 	AddSphereWall(id--,msystem,cp,radius,ChVector<>(x, y, 0),ChVector<>(0));

	// }

	plist.second = msystem->Get_bodylist().size() - 1;
	return plist;
}

std::pair<size_t, size_t> AddContainer(ChSystemMulticoreSMC* msystem, const ConfigParameters& cp, int id_base) {
	// Get start index of the wall list
	int id = id_base;
	std::pair<size_t, size_t> wlist;
	wlist.first = msystem->Get_bodylist().size();

	// Adjust box dimensions for 'set' sim
	double temph = cp.clength_z * 3.3;
	double tempt = cp.cthickness * 0.999;

	double grad = cp.grad;
	double sdia = cp.funnel_small_dia;
	double shgt = cp.height_stem;
	double bdia = cp.funnel_large_dia ;
	double rang = cp.angle_funnel*CH_C_PI/180;
	double rlength = (bdia - sdia) / (2.0 * cos(rang));
	double dist_funnel_platform=cp.dist_funnel_platform; //ISO : 75
	double size_platform =cp.platform_size; //ISO : 100

	int nb_particles = round(0.25*pow(size_platform/2.0/grad,3)*tan(CH_C_PI/3.0));
	double brad=bdia/2.0;
	
	double bhgt=(2*pow(dist_funnel_platform,3))/(3*pow(brad*tan(rang),2));
	double crad_stem = sdia / 2.0 + grad;									/// Estimated radius of funnel stem particles-ring
	double crad_body = bdia / 2.0 + grad;									/// Estimated radius of funnel body particles-ring

	double sftz = grad * sqrt(4.0 - (1.0 / pow(cos(CH_C_PI / 6.0), 2.0)));	/// Shift between z-layers
	double numd_stem = round(CH_C_PI / asin(grad / crad_stem));				/// Num particles needed to construct funnel stem particle-ring
	double numd_body = round(CH_C_PI / asin(grad / crad_body));				/// Num particles needed to construct funnel stem particle-ring
	double numh_stem = round((shgt - 2.0 * grad) / sftz + 1);				/// Num particles needed to preserve the funnel stem height
	
	double numl_ramp = round(rlength / (2.0 * grad));						/// Num particles along the ramped portion of the funnel

	crad_stem = grad / sin(CH_C_PI / numd_stem);							/// Actual radius of funnel stem particle-ring
	crad_body = grad / sin(CH_C_PI / numd_body);							/// Actual radius of funnel body particle-ring

	// Add and calculate additional funnel properties
	double theta_stem = 2.0 * asin(grad / crad_stem);						/// Angle between spheres on the funnel stem ring
	double theta_body = 2.0 * asin(grad / crad_body);						/// Angle between spheres on the funnel body ring

	// Set parent position and rotation vectors 
	// ChVector<> npos = ChVector<>(0, (cp.clength_z - cp.cthickness) / 2.0, -(cp.cthickness + cp.clength_y) / 2.0);
	ChVector<> npos(0,0,dist_funnel_platform+grad) ;
	ChVector<> ramp_pos, body_pos;
	ChVector<> stem_pos(0,0,dist_funnel_platform);

	double height=cp.dist_funnel_platform+(numh_stem-1)*sftz+grad+2.0*grad*sin(rang)*(numl_ramp-1)+2.0*grad*cos(rang);

	double marg_fill = cp.grad*1.5;
	double sft_y = 2.0 * marg_fill * sin(CH_C_PI / 3.0);
	double sft_z = marg_fill * Sqrt(4.0 - (1.0 / Pow(cos(CH_C_PI / 6.0), 2.0)));
	double numx = ceil(2*(crad_body-marg_fill) / (marg_fill * 2.0)) + 1;
	double numy = ceil(2*(crad_body-marg_fill) / sft_y) + 1;
	double numz = ceil(nb_particles / (numx*numy)) + 1;
	
	height+=3*marg_fill+sft_z*numz;
	double size_z = height+4*cp.grad-dist_funnel_platform;
	double pos_center_wall_z=cp.dist_funnel_platform+size_z/2.0;

	wlist.first = msystem->Get_bodylist().size();

	// Add box walls
	ChVector<> sbase = ChVector<>(2*crad_body, 2*crad_body, tempt) / 2.0;
	ChVector<> srght = ChVector<>(tempt, 2*crad_body, size_z+ 2.0 * cp.cthickness) / 2.0;
	ChVector<> sback = ChVector<>(2*crad_body+ 2.0 * cp.cthickness, tempt, size_z+ 2.0 * cp.cthickness) / 2.0;

	ChVector<> pbase = ChVector<>(0, 0, -cp.cthickness/ 2.0 +cp.dist_funnel_platform);
	ChVector<> proof = ChVector<>(0, 0, height+4*cp.grad + cp.cthickness / 2.0);
	ChVector<> prght = ChVector<>(2*crad_body + cp.cthickness, 0, 2.0*pos_center_wall_z) / 2.0;
	ChVector<> pleft = -ChVector<>(2*crad_body + cp.cthickness, 0, -2.0*pos_center_wall_z) / 2.0;
	ChVector<> pfrnt = ChVector<>(0, 2*crad_body + cp.cthickness, 2.0*pos_center_wall_z) / 2.0;
	ChVector<> pback = -ChVector<>(0, 2*crad_body + cp.cthickness, -2.0*pos_center_wall_z) / 2.0;

	std::cout<<"id dans add container,avant = "<<id<<"\n";
	AddWall(--id, msystem, cp, sbase, pbase, true);
	AddWall(--id, msystem, cp, srght, prght, true);
	AddWall(--id, msystem, cp, srght, pleft, true);
	AddWall(--id, msystem, cp, sback, pback, true);
	AddWall(--id, msystem, cp, sback, pfrnt, false);
	AddWall(--id, msystem, cp, sbase, proof, true);
	std::cout<<"id dans add container,après = "<<id<<"\n";

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
	std::pair<size_t, size_t> plist = AddPlatform(&msystem,cp,cfile);
	std::pair<size_t, size_t> wlist = AddContainer(&msystem, cp,-plist.second-1);
	wlist.first=plist.first;
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
	// application->AddCamera(ChVector<>(0, cp.clength_x * 2, cp.clength_z), ChVector<>(0, 0, cp.clength_z));
	application->AddCamera(ChVector<>(0, cp.clength_x * 2, cp.dist_funnel_platform), ChVector<>(0, 0, cp.dist_funnel_platform));
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