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
// 
// 
//
// =============================================================================

// Header and include files
#include "chrono/ChConfig.h"
#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "ProjConfig.h"
#include "ProjWriteData.h"
#include "ProjCalcs.h"
#include "ProjProc.h"

#include <ctime>        
#include <random>
#include <random>
#include <vector>
#include <numeric>
#include <iostream>

// If IRRLICHT is enabled, add irrlicht headers and namespaces
#ifdef CHRONO_IRRLICHT
#include <irrlicht.h>
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

// Other namespace declatartions
using namespace chrono;
using namespace chrono::collision;
using namespace chrono::geometry;


// Nozzel and box geometric properties that by sim type
const double NOZZEL_DIA = 5.0;  // 6 for half scale, 8.0 for full
const double NOZZEL_HEIGHT = 5; // 22.5 for half scale, 110 (85) for fullscale, 67 for level changing



ChVector<> RotatePoint(ChVector<> pivot, double theta, ChVector<> point) {
	double s = sin(theta);
	double c = cos(theta);

	// translate point back to origin:
	point.x() -= pivot.x();
	point.y() -= pivot.y();

	// rotate point
	double xnew = point.x() * c - point.y() * s;
	double ynew = point.x() * s + point.y() * c;

	// translate point back:
	point.x() = xnew + pivot.x();
	point.y() = ynew + pivot.y();
	return point;
}


void AddSphere(int id, ChSystemMulticoreSMC* msystem, const ConfigParameters& cp,
	double radius, ChVector<> pos, ChVector<> init_v,bool iswall) {
	// Shared parameters for the falling ball
	double mass = cp.grho * (4.0 / 3.0) * CH_C_PI * pow(radius, 3.0);
	ChVector<> inertia = 0.4 * mass * pow(radius, 2.0) * ChVector<>(1, 1, 1);

	// Create a spherical body. Set body parameters and sphere collision model
	auto body = std::shared_ptr<ChBody>(msystem->NewBody());
	body->SetIdentifier(id);
	body->SetMass(mass);
	body->SetPos(pos);
	body->SetRot(QUNIT);
	if (iswall){
		body->SetPos_dt(ChVector<>(0,0,0));
	}
	else{
		body->SetPos_dt(init_v);
	}
	body->SetWvel_par(VNULL);
	body->SetInertiaXX(inertia);
	body->SetBodyFixed(iswall);
	body->SetCollide(true);


	body->GetCollisionModel()->ClearModel();
	utils::AddSphereGeometry(body.get(), cp.mat_pp, radius);
	body->GetCollisionModel()->BuildModel();

	// Attach a color to the sphere
	#ifdef CHRONO_IRRLICHT
	auto mvisual = chrono_types::make_shared<ChSphereShape>();
	mvisual->GetSphereGeometry().rad = radius;
	if (iswall) mvisual->SetColor(ChColor(1.0f, 1.0f, 1.0f));
	else mvisual->SetColor(ChColor(0.0f, 1.0f, 1.0f));
	body->AddVisualShape(mvisual);
	#endif

	// Add the body to the system
	msystem->AddBody(body);
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


std::pair<size_t, size_t> RecreateParticles(ChSystemMulticoreSMC* msystem, const ConfigParameters &cp, ParticleData* data,
	const size_t &num_bodies, const bool iswall) {
	// Get start index and if of the particle list
	std::pair<size_t, size_t> glist;
	glist.first = msystem->Get_bodylist().size();

	// Do I need to fix this?!?!?!
	for (size_t i = glist.first; i < num_bodies; ++i) {
		if (iswall && data[i].id >= 0) break;

		ChVector<> pos = ChVector<>(data[i].pos_x, data[i].pos_y, data[i].pos_z);
		ChQuaternion<> rot = ChQuaternion<>(data[i].rot_0, data[i].rot_1, data[i].rot_2, data[i].rot_3);

		ChVector<> init_v = ChVector<>(data[i].vel_x, data[i].vel_y, data[i].vel_z);
		ChVector<> init_w = ChVector<>(data[i].rot_vel_x, data[i].rot_vel_y, data[i].rot_vel_z);
		
		bool realnum = true;
		if (std::isnan(init_v.Length()) || std::isnan(init_w.Length())) realnum = false;
		if (data[i].radius < cp.grad * 0.5) realnum = false; // This is really ugly...it's a hack to avoid double counting the funnel

		bool collide = true;
		// if (iswall && (pos.y() < cp.cmisc)) collide = false;
		if (iswall && (pos.y() < cp.clength_y)) collide = false;

		if (realnum) AddSphere(data[i].id, msystem, cp, data[i].radius, pos, init_v,iswall);
	}

	// Find and return index range of the particle list 
	glist.second = msystem->Get_bodylist().size() - 1;
	return glist;
}


void AddRoughness(ChSystemMulticoreSMC* msystem, const ConfigParameters &cp, ChVector<> size,
	ChVector<> ref, double theta, bool lid, bool collide) {
	// Get start index of particle list
	std::pair<size_t, size_t> plist;
	plist.first = msystem->Get_bodylist().size();

	// Set shared sphere properties
	auto wall = msystem->Get_bodylist().at(plist.first - 1);
	int id = wall->GetIdentifier();

	// Create some shared particle properties
	ChQuaternion<> rot = ChQuaternion<>(1, 0, 0, 0);
	ChVector<> init_v = ChVector<>(0, 0, 0);
	ChVector<> init_w = ChVector<>(0, 0, 0);

	double marg = cp.grad * 1.5;

	// Find the number of particles per x and z row, and the position shift between particles
	double sftx = 2.0 * marg * cos(CH_C_PI / 3.0);
	double sftz = 2.0 * marg * sin(CH_C_PI / 3.0);
	double numx = floor(size.x() / marg);
	double numz = floor(((size.z() * 2.0) - (2.0 * marg)) / sftz + 1.0);

	// Create a generator for determining random radii following a normal distribution 
	auto seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator((unsigned int)seed);
	std::normal_distribution<double> distribution(cp.grad, cp.gmarg / 4.0);

	// Glue the particles to the board in a randomized lattice structure
	for (int iz = 0; iz < numz; ++iz) {
		// Apply a shift to odd index z layers
		int add_sft = 0;
		if (iz % 2) add_sft = 1;

		double posix = sftx * add_sft;
		double posiz = -sftz * iz;

		// Create the first particle in the z layer
		double radi = distribution(generator);
		ChVector<> posi = ref + ChRandomXYZ(marg) + ChVector<>(posix, 0, posiz);
		ChVector<> posif = RotatePoint(ref, theta, posi + ChVector<>(0, radi, 0));

		double nrad = NOZZEL_DIA / 2.0 + marg * 2.0; 
		ChVector<> nozzel_center = ChVector<>(0, 0, -(cp.cthickness + cp.clength_y) / 2.0);

		if (lid) {
			ChVector<> temp_posi = posif - nozzel_center;
			double temp_radi = Sqrt(temp_posi.x() * temp_posi.x() + temp_posi.z() * temp_posi.z());
			if (temp_radi > nrad) AddSphere(--id, msystem, cp, radi, posif, init_v, true);
		}
		else AddSphere(--id, msystem, cp, radi, posif, init_v, true);

		// Create particles to the right and left of the initial sphere 
		for (int ix = 1; ix < numx / 2; ++ix) {
			double radr = distribution(generator);
			ChVector<> posr = posi + ChRandomXYZ(cp.gmarg) + ChVector<>(2.0 * sftx * ix, radr, 0);
			ChVector<> posrf = RotatePoint(ref, theta, posr);

			double radl = distribution(generator);
			ChVector<> posl = posi + ChRandomXYZ(cp.gmarg) + ChVector<>(-1 * 2.0 * sftx * ix, radl, 0);
			ChVector<> poslf = RotatePoint(ref, theta, posl);

			if (lid) {
				ChVector<> temp_posr = posrf - nozzel_center;
				double temp_radr = Sqrt(temp_posr.x() * temp_posr.x() + temp_posr.z() * temp_posr.z());
				if (temp_radr > nrad) AddSphere(--id, msystem, cp, radr, posr, init_v, true);

				ChVector<> temp_posl = poslf - nozzel_center;
				double temp_radl = Sqrt(temp_posl.x() * temp_posl.x() + temp_posl.z() * temp_posl.z());
				if (temp_radl > nrad) AddSphere(--id, msystem, cp, radl, posl, init_v,  true);
			}
			else {
				AddSphere(--id, msystem, cp, radr, posrf,  init_v,  true);
				AddSphere(--id, msystem, cp, radl, poslf,  init_v,  true);
			}
		}
	}

	// Get the end index of the particle list
	plist.second = msystem->Get_bodylist().size() - 1;
}


std::pair<size_t, size_t> FillFunnel(ChSystemMulticoreSMC* msystem, const ConfigParameters &cp, double crad, int num_particles,double* height,ChVector<> cpos) {
	// Get start index of particle list
	int id = 0;
	std::pair<size_t, size_t> glist;
	glist.first = msystem->Get_bodylist().size();

	// Set shared sphere and cylinder properties
	ChVector<> init_w = ChVector<>(0, 0, 0);
	ChQuaternion<> rot = ChQuaternion<>(1, 0, 0, 0);
	
	double gmarg_kd = cp.gmarg;
	double gmargrad_kd = cp.grad * (1.0 + gmarg_kd);

	double orad = crad - 2.0 * gmargrad_kd;

	double sft_xx = 2.0 * gmargrad_kd;
	double sft_xy = 2.0 * gmargrad_kd * cos(CH_C_PI / 3.0);
	double sft_zy = 2.0 * gmargrad_kd * sin(CH_C_PI / 3.0);
	double sft_xz = gmargrad_kd * tan(CH_C_PI / 6.0);
	double sft_yz = gmargrad_kd * sqrt(4.0 - (1.0 / pow(cos(CH_C_PI / 6.0), 2.0)));

	double numx = round(2.0 * orad / sft_xx);
	// double numz = floor((height - 2.0 * gmargrad_kd) / sft_zy + 1.0);
	double numy = round(2.0 * orad / sft_zy);

	ChVector<> pos_zero = cpos + ChVector<>(-orad, -orad,0.0);
	// std::cout<<"numx = "<<numx<<", numy = "<<numy<<", numz = "<<numz<<"\n";
	// Create a generator for determining random radii following a normal distribution 
	auto seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator((unsigned int)seed);
	std::normal_distribution<double> distribution(cp.grad, cp.bead_rad_std);

	// Construct particle lattice cloud
	// for (int iz = 0; iz < numz; ++iz) {
	int iz=0;
	int n_particles=0;
	ChVector<> pos_next = pos_zero;

	while (n_particles<num_particles){
		int add_sft = 0;
		if (iz % 2) add_sft = 1;

		double posz = sft_yz * iz; 

		for (int iy = 0; iy < numy; ++iy) {
			// Set the position of the first particle in the row
			double posx = sft_xz * add_sft + sft_xy * (iy % 2);
			double posy = sft_yz * add_sft + sft_zy * iy;

			pos_next = pos_zero + ChVector<>(posx, posy, posz);

			for (int ix = 0; ix < numx; ++ix) {
				//Add particles that lie inside of the cylinder
				ChVector<> temp_pos = pos_next - cpos;
				double temp_rad = Sqrt(temp_pos.x() * temp_pos.x() + temp_pos.y() * temp_pos.y());
				if (temp_rad <= orad) {
					AddSphere(id++, msystem, cp, distribution(generator), pos_next, ChRandomXYZ(cp.gvel), false);
					// AddSphere(id++, msystem, cp, distribution(generator), pos_next, rot, ChRandomXYZ(1.0E2), init_w, false, false);
					n_particles+=1;
					// std::cout<<"n_particles = "<<n_particles<<"\n";
				}
				pos_next += ChVector<>(sft_xx, 0, 0);
			}
		}
		iz+=1;
	}
	*height=pos_next.z();
	// Get the end index of the particle list and return
	glist.second = msystem->Get_bodylist().size() - 1;
	return glist;
}

std::pair<size_t, size_t> FillFunnel_setbox(ChSystemMulticoreSMC* msystem, const ConfigParameters& cp) {
	// Get start index of particle list
	int id = 0;
	std::pair<size_t, size_t> glist;
	glist.first = msystem->Get_bodylist().size();

	// Define temporary container dimensions
	double temp_height = cp.clength_z * 3.3;
	double marg = cp.grad * 1.5;
	double crad=cp.funnel_large_dia/2.0+cp.grad;
	int numd_body=round(CH_C_PI/asin(cp.grad/crad));
	crad=cp.grad/sin(CH_C_PI/numd_body);
	int num_particles = round(0.25*pow(cp.platform_size/2.0/cp.grad,3)*tan(CH_C_PI/3.0));

	double rang = cp.angle_funnel*CH_C_PI/180;
	double sftz = cp.grad * sqrt(4.0 - (1.0 / pow(cos(CH_C_PI / 6.0), 2.0)));
	double numh_stem = round((cp.height_stem - 2.0 * cp.grad) / sftz + 1);				
	double rlength = (cp.funnel_large_dia - cp.funnel_small_dia) / (2.0 * cos(rang));
	double numl_ramp = round(rlength / (2.0 * cp.grad));
	double height=cp.dist_funnel_platform+(numh_stem-1)*sftz+cp.grad+2.0*cp.grad*sin(rang)*(numl_ramp-1)+2.0*cp.grad*cos(rang);

	// Calculate offsets required for constructing the particle cloud
	double sft_x = marg;
	double sft_y = 2.0 * marg * sin(CH_C_PI / 3.0);
	double sft_z = marg * Sqrt(4.0 - (1.0 / Pow(cos(CH_C_PI / 6.0), 2.0)));
	double sft_w = marg * tan(CH_C_PI / 6.0);

	// Set the max number of beads along the X, Y, and Z axis
	double numx = ceil(2*(crad-marg) / (marg * 2.0)) + 1;
	double numy = ceil(2*(crad-marg) / sft_y) + 1;
	double numz = ceil(num_particles / (numx*numy)) + 1;

	std::cout<<"numx ="<<numx<<", numy ="<<numy<<",numz ="<<numz<<"\n";
	// Create a generator for computing random sphere radius values that follow a normal distribution 
	auto seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator((unsigned int)seed);
	std::normal_distribution<double> distribution(cp.grad, cp.gmarg / 6.0);

	// Construct a particle cloud inside of the box limits
	ChVector<> pos_ref = ChVector<>(-crad - marg, -crad- marg, height+marg);
	int nb_beads=0;
	//int iz=0;
	for (int iz = 0; iz < numz; ++iz) {
	//while (nb_beads<num_particles){
		//iz+=1;
		for (int iy = 0; iy < numy; ++iy) {
			double posx = sft_x * (iy % 2) + sft_x * ((iz + 1) % 2);
			double posy = sft_y * iy + sft_w * (iz % 2);
			double posz = sft_z * iz;

			ChVector<> pos_next = pos_ref + ChVector<>(posx, posy, posz);
			if (pos_next.z() >= height+marg){
				if (pos_next.y() <= crad - marg && pos_next.y() >= -crad + marg) {
					for (int ix = 0; ix < numx; ++ix) {
						//if (pos_next.x() <= cp.clength_x / 2.0 - marg && pos_next.x() >= -cp.clength_x / 2.0 + marg) {
						if (pos_next.x() <= 2*crad - marg && pos_next.x() >= -2*crad + marg) {
						//if (Sqrt(Pow(pos_next.x(), 2) + Pow(pos_next.y(), 2)) < crad - marg){
							AddSphere(id++, msystem, cp, distribution(generator), pos_next, ChRandomXYZ(cp.gvel),false);
							++nb_beads;
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

void AddPlatform(ChSystemMulticoreSMC* msystem, const ConfigParameters &cp, 
	std::vector<double> x_pos, std::vector<double> y_pos,std::vector<double> rad,int* id){

	int n=x_pos.size();



	for(int i=0;i<n;i++){
		double x = x_pos[i];
		double y = y_pos[i];
		double radius = rad[i];
		// utils::AddSphereGeometry(platform.get(), cp.mat_pp, radius, ChVector<>(x, y, 0), ChQuaternion<>(1, 0, 0, 0), true);
		AddSphere(*id--,msystem,cp,cp.grad,ChVector<>(x, y, 0),ChVector<>(0),true);

	}
}


std::pair<std::pair<size_t, size_t>,std::pair<size_t, size_t>> AddFunnel(ChSystemMulticoreSMC* msystem, const ConfigParameters &cp, 
	ParticleData* data, size_t num_objects, std::vector<double> x_pos, std::vector<double> y_pos,std::vector<double> rad,bool rerun) {
	// Get start index and if of the wall list. If this is a re-run, some information was already 
	// added for the funnel during re-import. Delete that information so that i can be replaced correctly.
	// const std::shared_ptr<ChBody> wall = msystem->Get_bodylist().at(msystem->Get_bodylist().size() - 1);
	// int id = wall->GetIdentifier();
	int id = 0;

	// Calculate some geometric properties of the funnel
	double grad = cp.grad;
	double sdia = cp.funnel_small_dia;
	double shgt = cp.height_stem;
	double bdia = cp.funnel_large_dia ;
	// double bhgt = NOZZEL_HEIGHT;
	double rang = cp.angle_funnel*CH_C_PI/180;
	double rlength = (bdia - sdia) / (2.0 * cos(rang));
	double dist_funnel_platform=cp.dist_funnel_platform; //ISO : 75
	double size_platform =cp.platform_size; //ISO : 100
	
	// int nb_particles = round(pow(dist_funnel_platform,3)/(4*pow(grad,3)*tan(CH_C_PI/4)));
	int nb_particles = round(0.25*pow(size_platform/2.0/grad,3)*tan(CH_C_PI/3.0));
	double brad=bdia/2.0;
	
	// double bhgt=1.5*(brad/3)*(4*nb_particles*pow(grad/brad,3)-tan(rang)*(1-pow(sdia/2/brad,3)));
	// double bhgt=4*nb_particles*pow(grad/brad,3)/3;
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
	std::cout<<"crad_stem = "<<crad_stem<<"\n";


	// Add and calculate additional funnel properties
	double theta_stem = 2.0 * asin(grad / crad_stem);						/// Angle between spheres on the funnel stem ring
	double theta_body = 2.0 * asin(grad / crad_body);						/// Angle between spheres on the funnel body ring

	// Set parent position and rotation vectors 
	// ChVector<> npos = ChVector<>(0, (cp.clength_z - cp.cthickness) / 2.0, -(cp.cthickness + cp.clength_y) / 2.0);
	ChVector<> npos(0,0,dist_funnel_platform+grad) ;
	ChVector<> ramp_pos, body_pos;
	ChVector<> stem_pos(0,0,dist_funnel_platform);

	double height=cp.dist_funnel_platform+(numh_stem-1)*sftz+grad+2.0*grad*sin(rang)*(numl_ramp-1)+2.0*grad*cos(rang);
	// double* cyl_height=&height;

	double marg_fill = cp.grad*1.5;
	double sft_y = 2.0 * marg_fill * sin(CH_C_PI / 3.0);
	double sft_z = marg_fill * Sqrt(4.0 - (1.0 / Pow(cos(CH_C_PI / 6.0), 2.0)));
	double numx = ceil(2*(crad_body-marg_fill) / (marg_fill * 2.0)) + 1;
	double numy = ceil(2*(crad_body-marg_fill) / sft_y) + 1;
	double numz = ceil(nb_particles / (numx*numy)) + 1;
	
	std::pair<size_t, size_t> glist;
	//glist = FillFunnel(msystem, cp, crad_body, nb_particles,cyl_height,ChVector<>(0,0,height));
	std::cout<<"AddFunnel : nb particles = "<<nb_particles<<"\n";
	glist = FillFunnel_setbox(msystem, cp);
	height+=3*marg_fill+sft_z*numz;
	std::pair<size_t, size_t> wlist;
	wlist.first = msystem->Get_bodylist().size();
	
	// Add funnel
	// auto funnel= std::shared_ptr<ChBody>(msystem->NewBody());
	// funnel->SetIdentifier(--id);
	// funnel->SetMass(cp.cmass);
	// funnel->SetBodyFixed(true);
	// funnel->SetCollide(true);

	// funnel->GetCollisionModel()->ClearModel();
 
	/*	
	for (int i = 0; i < numd_stem; ++i) {
		for (int j = 0; j < numh_stem; ++j) {
			int add_sft = 0;
			if (j % 2) add_sft = 1;

			double posx = crad_stem * sin(theta_stem * i + theta_stem / 2.0 * add_sft);
			double posz = j * sftz;
			double posy = crad_stem * cos(theta_stem * i + theta_stem / 2.0 * add_sft);
			stem_pos = npos + ChVector<>(0,0, posz);
			// utils::AddSphereGeometry(funnel.get(), cp.mat_pp, grad, npos + ChVector<>(posx, posy, posz), ChQuaternion<>(1, 0, 0, 0), true);
			AddSphere(id--,msystem,cp,grad,npos + ChVector<>(posx, posy, posz),ChVector<>(0),true);
		}
	}
	
	double ramp_pos_estim=cp.dist_funnel_platform+(numh_stem-1)*sftz+grad;
	std::cout<<"stem_pos fin 1ere boucle = "<<stem_pos.z()<<", estim = "<<ramp_pos_estim<<"\n";
	for (int i = 1; i < numl_ramp; ++i) {
		double sftx = 2.0 * grad * cos(rang) * i;
		double sftz = 2.0 * grad * sin(rang) * i;
		double crad_ramp = crad_stem + sftx;
		double numd_ramp = round(CH_C_PI / asin(grad / crad_ramp));
		double temp_grad = sin(CH_C_PI / numd_ramp) * crad_ramp;
		double theta_ramp = 2.0 * asin(temp_grad / crad_ramp);
	
		for (int j = 0; j < numd_ramp; ++j) {
			double posx = crad_ramp * sin(j*theta_ramp + theta_ramp / 2.0);
			double posz = sftz;
			double posy = crad_ramp * cos(j*theta_ramp + theta_ramp / 2.0);
			ramp_pos = stem_pos + ChVector<>(0,0,posz + 2.0 * grad * cos(rang));
			// utils::AddSphereGeometry(funnel.get(), cp.mat_pp, grad, stem_pos + ChVector<>(posx, posy, posz), ChQuaternion<>(1, 0, 0, 0), true);
			AddSphere(id--,msystem,cp,grad,stem_pos + ChVector<>(posx, posy, posz),ChVector<>(0),true);
		}
	}
	
	ramp_pos_estim+=2.0*grad*sin(rang)*(numl_ramp-1)+2.0*grad*cos(rang);
	std::cout<<"ramp_pos.z() à la fin : "<<ramp_pos.z()<<", estimation = "<<ramp_pos_estim<<"\n";


	
	std::cout<<"Final height = "<<*cyl_height<<"\n";
	*cyl_height+=4*cp.grad;
	
	double numh_body = round((*cyl_height - 2.0 * grad) / sftz + 1);				/// Num particles needed to preserve the funnel body height
	
	double posz=0;
	
	for (int i = 0; i < numd_body; ++i) {
		int j=0;
		posz=0;
			// for (int j = 0; j < numh_body; ++j) {
		while (posz +ramp_pos.z()< *cyl_height) {
			int add_sft = 0;
			if (j % 2) add_sft = 1;

			double posx = crad_body * sin(i*theta_body + theta_body / 2.0 * add_sft);
			posz = j * sftz;
			double posy = crad_body * cos(i*theta_body + theta_body / 2.0 * add_sft);
			body_pos = ramp_pos + ChVector<>(0, posz, 0);
			// utils::AddSphereGeometry(funnel.get(), cp.mat_pp, grad, ramp_pos + ChVector<>(posx, posy, posz), ChQuaternion<>(1, 0, 0, 0), true);
			AddSphere(id--,msystem,cp,grad,ramp_pos + ChVector<>(posx, posy, posz),ChVector<>(0),true);
			j++;
		}
	}*/
	// utils::AddCylinderGeometry(funnel.get(), cp.mat_pp, crad_body + cp.gdia, grad, body_pos + ChVector<>(0, cp.gdia, 0), ChQuaternion<>(1, 1, 0, 0), true);
	// utils::AddCylinderGeometry(funnel.get(), cp.mat_pp, crad_body + cp.gdia, grad, body_pos + ChVector<>(0, cp.gdia, 0), Q_from_AngX(CH_C_PI/2.0), true);
	// funnel->GetCollisionModel()->BuildModel();

	// msystem->AddBody(funnel);
  
	
	msystem->GetSettings()->collision.use_aabb_active = true;
	msystem->GetSettings()->collision.aabb_min = real3(-10000 * size_platform, -10000 * size_platform, -dist_funnel_platform);
	msystem->GetSettings()->collision.aabb_max = real3(10000 * size_platform, 10000 * size_platform, height*3+dist_funnel_platform+shgt+height);

  /*
  //Adding the standard platform and walls
	ChVector<> pbase(0,0,dist_funnel_platform+cp.cthickness/2.0);
	ChVector<> proof(0,0,*cyl_height+cp.cthickness/2.0);
	ChVector<> pplatform(0,0,-cp.cthickness/2.0);

	ChVector<> sbase(1.25*crad_stem,1.25*crad_stem,cp.cthickness/2.0);
	ChVector<> sroof(1.25*crad_body,1.25*crad_body,cp.cthickness/2.0);
	ChVector<> splatform(size_platform/2.0,size_platform/2.0,cp.cthickness/2.0);

	// AddWall(--id,msystem,cp,splatform,pplatform,true,true);
	AddPlatform(msystem,cp,x_pos,y_pos,rad,&id);
	AddWall(--id,msystem,cp,sroof,proof,true);
	AddWall(--id,msystem,cp,sbase,pbase,true);
	*/
	// *cyl_height+=4*cp.grad;
 
	
  double size_z = height+4*cp.grad-dist_funnel_platform;


	ChVector<> pbase(0,0,dist_funnel_platform-cp.cthickness/2.0);
	ChVector<> proof(0,0,height+4*cp.grad+cp.cthickness/2.0);
	ChVector<> pleft(-cp.cthickness/2.0-cp.funnel_large_dia/2.0,0,dist_funnel_platform+size_z/2.0);
	ChVector<> pright(cp.cthickness/2.0+cp.funnel_large_dia/2.0,0,dist_funnel_platform+size_z/2.0);
	ChVector<> pfront(0,-cp.cthickness/2.0-cp.funnel_large_dia/2.0,dist_funnel_platform+size_z/2.0);
	ChVector<> pback(0,cp.cthickness/2.0+cp.funnel_large_dia/2.0,dist_funnel_platform+size_z/2.0);
	
	ChVector<> sbase(cp.funnel_large_dia/2.0,cp.funnel_large_dia/2.0,cp.cthickness/2.0);
	ChVector<> sroof(cp.funnel_large_dia/2.0,cp.funnel_large_dia/2.0,cp.cthickness/2.0);
	ChVector<> sleft(cp.cthickness/2.0,cp.funnel_large_dia/2.0,size_z/2.0);
	ChVector<> sright(cp.cthickness/2.0,cp.funnel_large_dia/2.0,size_z/2.0);
	ChVector<> sfront(cp.funnel_large_dia/2.0,cp.cthickness/2.0,size_z/2.0);
	ChVector<> sback(cp.funnel_large_dia/2.0,cp.cthickness/2.0,size_z/2.0);

	bool vis=true;

	AddWall(id--,msystem,cp,sbase,pbase,vis);
	AddWall(id--,msystem,cp,sroof,proof,vis);
	AddWall(id--,msystem,cp,sleft,pleft, vis);
	AddWall(id--,msystem,cp,sright,pright,not vis);
	AddWall(id--,msystem,cp,sback,pback,not vis);
	AddWall(id--,msystem,cp,sfront,pfront, vis);
  //AddPlatform(msystem,cp,x_pos,y_pos,rad,&id);



	wlist.second = msystem->Get_bodylist().size() - 1;
	std::pair<std::pair<size_t, size_t>,std::pair<size_t, size_t>> to_return;
	to_return.first=glist;
	to_return.second=wlist;
	return to_return;
}


std::pair<size_t, size_t> CreateContainer(ChSystemMulticoreSMC* msystem, const ConfigParameters &cp, 
	ParticleData* data, size_t num_objects, bool rerun) {
	// Get start index and if of the wall list
	int id = 0;
	std::pair<size_t, size_t> wlist;
	wlist.first = msystem->Get_bodylist().size();

	// Define some additional geometric properties
	const double rimw = 12.0 * 0.5;
	const double tabw = 13.0; // Divide by 2 for half scale sim

	double length = cp.clength_x - rimw;
	double height = cp.clength_z - rimw;
	double theta = atan(cp.clength_z / cp.clength_x);
	// double hlev = - cp.cthickness / 2.0 + cp.cmisc;
	double hlev = - cp.cthickness / 2.0 + cp.clength_y;

	double htx = (cp.cthickness / 2.0) * sin(theta);
	double hty = (cp.cthickness / 2.0) * cos(theta);

	double lr = ((length - tabw) / cos(theta)) / 2.0;  
	double xr = (lr * cos(theta)) / 2.0 + tabw / 2.0 + htx;                       
	double yr = (lr * sin(theta)) / 2.0 + hty / 4.0 + hlev;
	
	// Define wall rotation angles 
	ChQuaternion<> rot_flat = ChQuaternion<>(1, 0, 0, 0);
	ChQuaternion<> rot_ramp_r = ChQuaternion<>(Q_from_AngAxis( theta, ChVector<>(0.0, 0.0, 1.0)));
	ChQuaternion<> rot_ramp_l = ChQuaternion<>(Q_from_AngAxis(-theta, ChVector<>(0.0, 0.0, 1.0)));

	// Define wall sizes and positions
	ChVector<> sz_lid = ChVector<>(length + 2.0 * cp.cthickness, cp.cthickness, cp.clength_y) / 2.0;
	ChVector<> sz_base = ChVector<>(length, cp.cthickness, cp.clength_y) / 2.0;
	ChVector<> sz_side = ChVector<>(cp.cthickness, (height + hty) / 2.0, cp.clength_y + cp.cthickness) / 2.0;
	ChVector<> sz_face = ChVector<>(length, (height + hty) / 2.0, cp.cthickness) / 2.0;
	ChVector<> sz_wall = ChVector<>(lr * cos(theta), cp.cthickness, cp.clength_y) / 2.0;
	ChVector<> sz_sprt = ChVector<>(tabw, cp.cthickness, cp.clength_y) / 2.0;
	ChVector<> sz_ramp = ChVector<>(lr, cp.cthickness, cp.clength_y) / 2.0;

	ChVector<> pos_lid = ChVector<>(0, cp.cthickness + height, -cp.cthickness - cp.clength_y) / 2.0;
	ChVector<> pos_base = ChVector<>(0, cp.cthickness - height, -cp.cthickness - cp.clength_y) / 2.0;
	ChVector<> pos_side_l = ChVector<>(-(cp.cthickness + length), height / 2.0 + hty, -cp.clength_y) / 2.0;
	ChVector<> pos_side_r = ChVector<>(length + cp.cthickness, height / 2.0 + hty, -cp.clength_y) / 2.0;
	ChVector<> pos_side_f = ChVector<>(0, height / 4.0 + hty / 2.0, -(cp.cthickness + cp.clength_y));
	ChVector<> pos_side_b = ChVector<>(0, height / 4.0 + hty / 2.0, 0);
	
	ChVector<> pos_wall_m = ChVector<>(0, hlev, -(cp.cthickness + cp.clength_y) / 2.0);
	ChVector<> pos_wall_l = ChVector<>(xr - htx, hlev, -(cp.cthickness + cp.clength_y) / 2.0);
	ChVector<> pos_wall_r = ChVector<>(-xr + htx, hlev, -(cp.cthickness + cp.clength_y) / 2.0);
	ChVector<> pos_ramp_l = ChVector<>(-xr, yr, -(cp.cthickness + cp.clength_y) / 2.0);
	ChVector<> pos_ramp_r = ChVector<>(xr, yr, -(cp.cthickness + cp.clength_y) / 2.0);

	// Add walls to system. The collide and size properties of some walls have been altered to optimize calc time during set
	// AddWall(--id, msystem, cp, sz_lid, pos_lid, rot_flat, false, true);
	// AddWall(--id, msystem, cp, sz_base, pos_base, rot_flat, false, true);
	// AddWall(--id, msystem, cp, sz_side, pos_side_l, rot_flat, true, true);
	// AddWall(--id, msystem, cp, sz_side, pos_side_r, rot_flat, true, true);
	// AddWall(--id, msystem, cp, sz_face, pos_side_f, rot_flat, true, false);
	// AddWall(--id, msystem, cp, sz_face, pos_side_b, rot_flat, true, true);
	
	// AddWall(--id, msystem, cp, sz_sprt, pos_wall_m, rot_flat, true, true);
	// AddWall(--id, msystem, cp, sz_wall, pos_wall_l, rot_flat, true, true);
	// AddWall(--id, msystem, cp, sz_wall, pos_wall_r, rot_flat, true, true);
	
	// AddWall(--id, msystem, cp, sz_ramp, pos_ramp_l, rot_ramp_l, true, true);
	// AddWall(--id, msystem, cp, sz_ramp, pos_ramp_r, rot_ramp_r, true, true);
	
	AddWall(--id, msystem, cp, sz_lid, pos_lid, false);
	AddWall(--id, msystem, cp, sz_base, pos_base, false);
	AddWall(--id, msystem, cp, sz_side, pos_side_l,false);
	AddWall(--id, msystem, cp, sz_side, pos_side_r,false);
	AddWall(--id, msystem, cp, sz_face, pos_side_f, false);
	AddWall(--id, msystem, cp, sz_face, pos_side_b,false);
	
	AddWall(--id, msystem, cp, sz_sprt, pos_wall_m,false);
	AddWall(--id, msystem, cp, sz_wall, pos_wall_l,false);
	AddWall(--id, msystem, cp, sz_wall, pos_wall_r,false);
	
	AddWall(--id, msystem, cp, sz_ramp, pos_ramp_l,false);
	AddWall(--id, msystem, cp, sz_ramp, pos_ramp_r,false);

	// Add the particles that are glued to the walls of the container
	if (rerun) std::pair<size_t, size_t> pwlist = RecreateParticles(msystem, cp, data, num_objects, true);
	else {
		ChVector<> ref_lid = ChVector<>(0, height / 2.0, -(cp.cthickness / 2.0 + cp.gmarg));
		ChVector<> ref_base = ChVector<>(0, cp.cthickness - height / 2.0, -(cp.cthickness / 2.0 + cp.gmarg));
		ChVector<> ref_ramp_r = ChVector<>(xr - htx, yr + hty, -(cp.cthickness / 2.0 + cp.gmarg));
		ChVector<> ref_ramp_l = ChVector<>(-xr + htx, yr + hty, -(cp.cthickness / 2.0 + cp.gmarg));

		// AddRoughness(msystem, cp, sz_base, ref_lid, 0, true, true);
		// AddRoughness(msystem, cp, sz_base, ref_base, 0, false, false);
		// AddRoughness(msystem, cp, sz_ramp, ref_ramp_r, theta, false, true);
		// AddRoughness(msystem, cp, sz_ramp, ref_ramp_l, -theta, false, true);
	}

	// Find and return index range of full wall list 
	wlist.second = msystem->Get_bodylist().size() - 1;
	
	// std::pair<size_t, size_t> wlist;
	// wlist.first = msystem->Get_bodylist().size();
	// wlist.second = msystem->Get_bodylist().size();
	return wlist;
}

double mass_rate(ChSystemMulticoreSMC &msystem, const ConfigParameters& cp,std::pair<size_t, size_t> glist, int num_particles){
	double total_mass=0.0;
	for (size_t i = glist.first; i < glist.first + num_particles; ++i) {
		std::shared_ptr<ChBody> bead_tested = msystem.Get_bodylist().at(i);
		ChVector<> pos_bead_tested=bead_tested->GetPos();

		if (abs(pos_bead_tested.z()-cp.dist_funnel_platform-cp.height_stem/2.0)<=cp.height_stem/2.0 && 
			(pow(pos_bead_tested.x(),2)+pow(pos_bead_tested.y(),2)<=pow(cp.funnel_small_dia/2.0,2))){
			total_mass+=bead_tested->GetMass();
		}
	}
	return total_mass;
}

bool detect_beads_in_funnel(ChSystemMulticoreSMC &msystem, const ConfigParameters& cp,std::pair<size_t, size_t> glist, int num_particles){
	bool beads_in_funnel= false;
	for (size_t i = glist.first; i < glist.first + num_particles; ++i) {
		std::shared_ptr<ChBody> bead_tested = msystem.Get_bodylist().at(i);
		ChVector<> pos_bead_tested=bead_tested->GetPos();

		if (pos_bead_tested.z()>cp.dist_funnel_platform && 
			(pow(pos_bead_tested.x(),2)+pow(pos_bead_tested.y(),2)<=pow(cp.funnel_large_dia/2.0,2))){
			beads_in_funnel=true;
			break;
		}
	}
	return beads_in_funnel;
}


int main(int argc, char* argv[]) {
	// Print Chrono version to userlog
	GetLog() << "\nCopyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n";

	// Update accordingly. The rest of main should remain largely the same between projects.
	// const std::string projname = "vsim_shadow_box";
	// const std::string projtype = "set";
	// const std::string configdir = GetConfigDir(argc, argv);

	// bool rerun = false;
	// if (argc == 2) rerun = true;

	// Get the location of the config file from user input
	const std::string cfile = GetConfigFile(argc, argv);

	// Initialize simulation. Import sim parameters and set input/output directories.
	ConfigParameters cp;
	int err = InitializeSimulation(&cp, cfile);
	if (err != 0) {
		fprintf(stderr, "Error configuring simulation at return %i\n", err);
		return -1;
	}


	const std::string cfolder = cfile.substr(0,cfile.rfind("/")+1);
	const std::string path_platform_config_file = cfolder+cp.config_platform_file;
	std::cout<<"cfolder = "<<path_platform_config_file<<"\n";

	std::ifstream file(path_platform_config_file);
	if (!file) {
		fprintf(stderr, "\nERR: Platform file does not exist!\n");
		return -1;
	}
	std::string line;
  
	std::vector<double> platform_x_pos, platform_y_pos,platform_rad;
	while (std::getline(file, line)) {
		// Ignore empy line or lines that start with #. Otherwise, parse words.
		if (line.empty()) continue;

		// Parse each line into a vector of words
		std::vector<std::string> parsed_line;
		std::vector<std::string> parsed_word;

		char* cline = const_cast<char*>(line.c_str());
		char* delim = strtok(cline, " ");
		while (delim != NULL) {
			std::string parsed_word(delim);
			parsed_line.push_back(parsed_word);
			delim = strtok(NULL, " ");
			// std::cout<<"delim = "<<delim<<"\n";
		}
		if (parsed_line.size() != 3) {
			std::cout<<"Problem for reading platform file \n"; 
			return -1;
		}

		platform_x_pos.push_back(std::stod(parsed_line[0]));
		platform_y_pos.push_back(std::stod(parsed_line[1]));
		platform_rad.push_back(std::stod(parsed_line[2]));

	}
 /*
	std::cout<<"taille x_pos"<<platform_x_pos.size()<<"\n";
	for (int i=0;i<platform_x_pos.size();i++){
	 	std::cout<<"x_pos="<<platform_x_pos[i]<<"\n";
	 	std::cout<<"y_pos="<<platform_y_pos[i]<<"\n";
	 	std::cout<<"rad="<<platform_rad[i]<<"\n";
	}
 */
	// Create a Multicore SMC system and set the system parameters
	ChSystemMulticoreSMC msystem;
	if (SetSimulationParameters(&msystem, cp) != 0) {
		fprintf(stderr, "Error setting simulation parameters\n");
		return -1;
	}

	// // If this is a re-run sum, import final state data previous sim
	// std::string fname = configdir + "/state_final.b.bin";
	// if (cp.projstruct == "full") fname = configdir + "/state_final.f.bin";

	size_t num_imported = 0;
	// if (rerun) num_imported = CountStateBinObjects(fname, cp.projstruct);
	ParticleData *istate_data = new ParticleData[num_imported];
	bool rerun = false;
	double time = 0.0;
	// if (rerun) {
	// 	if (ImportStateBin(istate_data, fname, cp.projstruct) != 0) {
	// 		fprintf(stderr, "Error importing initial state data.\n");
	// 		return -1;
	// 	}
	// 	time = istate_data[num_imported-1].time;
	// 	msystem.SetChTime(time);
	// }

	// Add the shadowbox to the simulation. If this is a sim reboot, reconstruct the previous scene
	// std::pair<size_t, size_t> wlist = CreateContainer(&msystem, cp, istate_data, num_imported, rerun);
	std::pair<std::pair<size_t, size_t>,std::pair<size_t, size_t>> lists = AddFunnel(&msystem, cp, istate_data, num_imported, 
		platform_x_pos,platform_y_pos,platform_rad,rerun);
	std::pair<size_t, size_t> glist=lists.first;
	std::pair<size_t, size_t> wlist=lists.second;
	std::cout<<"wlist = ("<<wlist.first<<","<<wlist.second<<"), glist = ("<<glist.first<<","<<glist.second<<") \n";
	// Delte the array containig the initial state information, as it is no longer needed
	delete[] istate_data;

	// Create an object to track system info and stats, such as element indicies and totals
	// SystemData mstats;
	// mystats.num_walls = wlist.second - wlist.first + 1 + 1; // Add 1 for funnel
	// mystats.num_particles = glist.second - glist.first + 1 - 1; // Subtract 1 for funnel
	// mystats.num_shapes = GetNumShapes(msystem);

	// size_t num_bodies = mystats.num_particles + mystats.num_walls;
	// size_t start_index = wlist.first;

	SystemData mstats;
	if (SetSystemStats(mstats, msystem, cp.time_step, wlist, glist) != 0) {
		fprintf(stderr, "\nERR: Error setting system stats\n");
		return -1;
	}


	GetLog() << "\nNumber of walls: " << mstats.num_walls;
	GetLog() << "\nNumber of particles: " << mstats.num_particles;
	GetLog() << "\nNumber of shapes: " << mstats.num_shapes << "\n\n";

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
	// application->AddCamera(ChVector<>(80, 80, cp.dist_funnel_platform/2.0+cp.height_stem/2.0), ChVector<>(0,0,cp.dist_funnel_platform/2.0+cp.height_stem/2.0));
	application->AddCamera(ChVector<>(80, 80, cp.dist_funnel_platform/2.0+cp.height_stem/2.0), ChVector<>(0,0,cp.dist_funnel_platform/2.0+cp.height_stem/2.0+30));
	application->AddLight(ChVector<>(0, cp.clength_z * 2, cp.clength_z), cp.clength_z * 2);
	#endif

	// Set the soft-real-time cycle parameters 
	double start_time = time;
	double time_loop = cp.time_loop;
	double time_save = cp.time_save;
	double threshold = 400;

	
	// Iterate through simulation. Calculate resultant forces and motion for each timestep
	while (time < cp.sim_duration) {
		// Start irrlicht visualization scene
		#ifdef CHRONO_IRRLICHT
		application->BeginScene();
		application->Render();
		application->GetDevice()->run();
		application->EndScene();
		#endif
		
		// Calculate dynamics for (cp.out_step / time_step.cp) continuous steps. Create new particles according to the fill_step interval
		while (time < time_loop) {
			msystem.DoStepDynamics(cp.time_step);
			time += cp.time_step;
			
			IncrementTimerTotals(msystem, &mstats,time);
			if (time > cp.sim_duration) break;

		}
		
		time_loop = time - cp.time_step + cp.time_loop;
	
		// // Calculate dynamics for (cp.out_step / time_step.cp) continuous steps  
		// #ifdef CHRONO_IRRLICHT
		// if (cp.irrlicht_vis) {
		// 	application->AssetBindAll();
		// 	application->AssetUpdateAll();
		// 	application->EndScene();
		// }
		// #endif

		// Save and write data every (cp.save_step / time_step.cp) steps 

		if (time > time_save) {
			/*
      if (ArchiveState(msystem, mstats, cp.proj_path, mstats.num_particles, glist.first, time) != 0) {
				fprintf(stderr, "\nERR: Error archiving run-time data \n");
				return -1;
			}
      */
			time_save += cp.time_save;
		}
	
		// Calculate the average velocity of all particles and exit the loop if KE < threshold
		//if(CalcAverageKE(msystem, cp.proj_path, mstats.num_particles, glist.first, time, threshold)) break;
   std::cout<<"Fin iteration \n";
		
	}
	/*
	//LOOP 2 : Break the dam ! Release the river ! 
	// std::shared_ptr<ChBody> roof = msystem.SearchBodyID(-4);
  
	std::shared_ptr<ChBody> roof = msystem.Get_bodylist().back();
	roof->SetPos(ChVector<>(100,100,100));
	std::vector<double> mass_rate_vector;
	int nb_it_stucked=0;
	int it_max_stucked=10000;
	bool stucked=false;
	// Iterate through simulation. Calculate resultant forces and motion for each timestep
	while (time < cp.sim_duration) {
		// Start irrlicht visualization scene
		#ifdef CHRONO_IRRLICHT
		application->BeginScene();
		application->Render();
		application->GetDevice()->run();
		application->EndScene();
		#endif

		int nb_bead_stopped=0;
		// Calculate dynamics for (cp.out_step / time_step.cp) continuous steps. Create new particles according to the fill_step interval
		while (time < time_loop) {
			msystem.DoStepDynamics(cp.time_step);
			time += cp.time_step;
			IncrementTimerTotals(msystem, &mstats,time);
			if (time > cp.sim_duration) break;

			// nb_bead_stopped=0;
			// for (size_t i = glist.first; i < glist.first + mstats.num_particles; ++i) {
			// 	std::shared_ptr<ChBody> bead_tested = msystem.Get_bodylist().at(i);
			// 	if (bead_tested->GetPos().z()<-10*cp.dist_funnel_platform && bead_tested->GetCollide()){
			// 		bead_tested->SetBodyFixed(true);
			// 		// bead_tested->SetCollide(false);
			// 		bead_tested->SetPos_dt(ChVector<>(0,0,0));
			// 		nb_bead_stopped++;
			// 	}
			// }

			double mass_rate_value=mass_rate(msystem,cp,glist,mstats.num_particles);
			std::cout<<"Mass rate = "<<mass_rate_value<<"\n";
			mass_rate_vector.push_back(mass_rate_value);

			if (mass_rate_value==0){
				nb_it_stucked++;
				std::cout<<"No mass rate since "<<nb_it_stucked<<"iterations \n";
			}
			else if(mass_rate_vector[mass_rate_vector.size()-1]==0){
				nb_it_stucked=0;
			}

			
			std::cout<<"Nb beads stopped = "<<nb_bead_stopped<<"\n";
			if (nb_bead_stopped>=mstats.num_particles) break;
			if (nb_it_stucked>=it_max_stucked && detect_beads_in_funnel(msystem,cp,glist,mstats.num_particles)){
				std::cout<<"Beads in funnel ? "<<detect_beads_in_funnel(msystem,cp,glist,mstats.num_particles)<<"\n";
				stucked=true;
				break;
			}

			for (size_t i = glist.first; i < glist.first + mstats.num_particles; ++i) {
				std::shared_ptr<ChBody> bead_tested = msystem.Get_bodylist().at(i);
				ChVector<> pos_bead_tested=bead_tested->GetPos();
				if (msystem.data_manager->host_data.active_rigid[bead_tested->GetId()] == 0){
					bead_tested->SetPos_dt(ChVector<>(0));
					bead_tested->SetRot_dt(QUNIT);
					
				}
			}


		}

		if (nb_bead_stopped>=mstats.num_particles) {
			std::cout<<"Exit because all particles are stopped \n";
			break;
		}
		if (stucked){
			std::cout<<"Exit because particles are stuck \n";
			break;
		}
		time_loop = time - cp.time_step + cp.time_loop;
	
		// Calculate dynamics for (cp.out_step / time_step.cp) continuous steps  
		// #ifdef CHRONO_IRRLICHT
		// if (cp.irrlicht_vis) {
		// 	application->AssetBindAll();
		// 	application->AssetUpdateAll();
		// 	application->EndScene();
		// }
		// #endif

		// Save and write data every (cp.save_step / time_step.cp) steps 
		if (time > time_save) {
			if (ArchiveState(msystem, mstats, cp.proj_path, mstats.num_particles, glist.first, time) != 0) {
				fprintf(stderr, "\nERR: Error archiving run-time data \n");
				return -1;
			}
			time_save += cp.time_save;

		// bool bool_stuck=true;
		// for (size_t i = glist.first; i < glist.first + mstats.num_particles; ++i) {
		// 	std::shared_ptr<ChBody> bead_tested = msystem.Get_bodylist().at(i);
		// 	ChVector<> pos_bead_tested=bead_tested->GetPos();

		// 	if (abs(pos_bead_tested.z()-(cp.dist_funnel_platform+cp.height_stem+4*cp.gdia) ){
		// 		if (bead_tested->GetPos_dt().Length()>=10){
		// 			bool_stuck=false;
		// 		}
		// 	}
		// 	if ()
		// }


		}

		// Calculate the average velocity of all particles and exit the loop if KE < threshold

		if(CalcAverageKE(msystem, cp.proj_path, mstats.num_particles, glist.first, time, threshold)) break;

		
	}
	*/

	std::cout<<"Before archive state \n";
	// Export the final state data for all bodies in the system
	if (ArchiveState(msystem, mstats, cp.proj_path, mstats.num_bodies, std::min(wlist.first,glist.first), time, true) != 0) {
		fprintf(stderr, "\nERR: Error archiving final state data \n");
		return -1;
	}

	std::cout<<"Before convert binaries \n";
	// Post-process step 1: convert select binaries to .csv format
	if (ConvertBinaries(cp.proj_path) != 0) {
		fprintf(stderr, "\nERR: Could not convert binary files.\n");
		return -1;
	}

	std::cout<<"Before CalcPacking \n";
	// Post-process step 2: calculate the packing properties of the system
	if (CalcPacking(cp.proj_path, cp) != 0) {
		fprintf(stderr, "\nERR: Could not calculate sim packing stats.\n");
		return -1;
	}

	/*
	//Write masse rate value in csv file
	unsigned long long int struct_size = sizeof(mass_rate_vector);
	std::ofstream csv_out(cp.proj_path+"/mass_rate.csv", std::ofstream::out | std::ofstream::app);
	for (int j = 0; j < mass_rate_vector.size(); ++j) {
		csv_out << mass_rate_vector[j]<<"\n";
	}
	csv_out.close();
	*/

	//Write params for repose simus in txt file
	std::ofstream txt_out(cp.proj_path+"/params_repose.txt");
	txt_out	
		<<" \n "<< "dist_funnel_platform "<<cp.dist_funnel_platform
		<<" \n "<< "angle funnel "<<cp.angle_funnel
		<<" \n "<< "height stem "<<cp.height_stem
		<<" \n "<< "funnel small diameter "<<cp.funnel_small_dia
		<<" \n "<< "funnel large diameter "<<cp.funnel_large_dia
		<<" \n "<< "platform size "<<cp.platform_size
		<<" \n "<< "bead diameter "<<cp.gdia
		<<" \n "<< "bead diameter mean "<<cp.gdia
		<<" \n "<< "bead diameter standard deviation"<<cp.bead_rad_std;




	txt_out.close();

	return 0;
}
