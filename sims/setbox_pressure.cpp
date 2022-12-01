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

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkTrajectory.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono/utils/ChUtilsInputOutput.h"


#include "ProjConfig.h"
#include "ProjCalcs.h"
#include "ProjWriteData.h"
#include "ProjProc.h"

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

class ContactReporter : public ChContactContainer::ReportContactCallback {
  public:
    ContactReporter(std::shared_ptr<ChBody> obj1, ChSystemMulticoreSMC* msystem, double* pressure, int* nb_of_contact) :
		m_obj1(obj1), m_system(msystem), m_pressure(pressure), m_nb_of_contact(nb_of_contact) {}

  private:
    virtual bool OnReportContact(const ChVector<>& pA,
                                 const ChVector<>& pB,
                                 const ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const double& eff_radius,
                                 const ChVector<>& cforce,
                                 const ChVector<>& ctorque,
                                 ChContactable* modA,
                                 ChContactable* modB) override {

		//Check if B obj is a bead !
		int id_B = modB->GetPhysicsItem()->GetIdentifier();
		int id_A = modA->GetPhysicsItem()->GetIdentifier();

		if (id_B>=0 && id_A>=0){
			double rad_B = m_system->SearchBodyID(id_B)->GetCollisionModel()->GetShapeDimensions(0).back();


			double rad_A = m_system->SearchBodyID(id_A)->GetCollisionModel()->GetShapeDimensions(0).back();

			double dist_beads = (m_system->SearchBodyID(id_B)->GetPos()-m_system->SearchBodyID(id_A)->GetPos()).Length();
			// std::cout<<"Pos bead A : "<<m_system->SearchBodyID(id_A)->GetPos()<<"\n";
			// std::cout<<"Pos bead B : "<<m_system->SearchBodyID(id_B)->GetPos()<<"\n";
			// std::cout<<"Dist_beads : "<<dist_beads<<"\n";
			double contact_surf_radius = sqrt(Pow(rad_A,2)-Pow(Pow(rad_A,2)-Pow(rad_B,2)+Pow(dist_beads,2),2)/(4*Pow(dist_beads,2)));

			// std::cout<<"Contact_surf_radius"<<Pow(Pow(rad_A,2)-Pow(rad_B,2)+Pow(dist_beads,2),2)/(4*Pow(dist_beads,2))<<"\n";

			// Check if contact involves obj1

			if (modA == m_obj1.get()){
				// printf("  A contact on Obj 1 at pos: %7.3f  %7.3f  %7.3f", pA.x(), pA.y(), pA.z());
				ChVector<> force_abs_coord = plane_coord * ChVector<>(cforce.x(),cforce.y(),cforce.z());
				std::cout<<"Force : "<<force_abs_coord<<", norm = "<<force_abs_coord.Length()<<", id_A = "<<id_A<<", id_B = "<<id_B<<", contact_surf ="<<
				CH_C_PI*Pow(contact_surf_radius,2)<<",rad = "<<rad_A<<"\n";
				// std::cout<<"Force abs coord in A  = "<<force_abs_coord<<"\n";
				// std::cout<<"Pressure in A before= "<<*m_pressure<<"\n";
				// std::cout<<"Added pressure in A = "<<(-force_abs_coord).Length()/(CH_C_PI*Pow(contact_surf_radius,2))<<"\n";
				if (!isnan(contact_surf_radius)){
					// *m_pressure = *m_pressure +(-force_abs_coord).Length()/(CH_C_PI*Pow(contact_surf_radius,2));
					*m_pressure = *m_pressure +(-force_abs_coord).Length()/(CH_C_PI*Pow(rad_A,2));

				}
				*m_nb_of_contact=*m_nb_of_contact+1;
				if (isnan(*m_pressure)){
					std::cout<<"Nan pressure, force = "<<force_abs_coord<<", surface radius = "<<contact_surf_radius<<"\n";
					std::cout<<"In square root : "<<Pow(rad_A,2)-Pow(Pow(rad_A,2)-Pow(rad_B,2)+Pow(dist_beads,2),2)/(4*Pow(dist_beads,2))<<"\n";
				}
				// std::cout<<"Pressure in A = "<<*m_pressure<<"\n";
				// std::cout<<"nb of contact in A :"<<*m_nb_of_contact<<"\n";
				// printf("  frc: %7.3f  %7.3f  %7.3f \n", cforce.x(), cforce.y(), cforce.z());
				// printf("  frc: %7.3f \n", cforce.Length());
				// std::cout<<"m_force dans class = "<<m_force<<",Length = "<<m_force.Length()<<"\n";


				// m_force.get()=m_force.get()+ChVector<>(cforce.x(),cforce.y(),cforce.z());

			// } else if (modB == m_obj1.get()) {
			//     printf("  B contact on Obj 1 at pos: %7.3f  %7.3f  %7.3f", pB.x(), pB.y(), pB.z());
			}
			else if (modB == m_obj1.get()){
				// m_force=m_force+plane_coord * ChVector<>(cforce.x(),cforce.y(),cforce.z());
				// printf("  frc: %7.3f  %7.3f  %7.3f \n", cforce.x(), cforce.y(), cforce.z());
				// printf("  frc: %7.3f \n", cforce.Length());
				// std::cout<<"m_force dans class = "<<m_force<<",Length = "<<m_force.Length()<<"\n";
				ChVector<> force_abs_coord = plane_coord * ChVector<>(cforce.x(),cforce.y(),cforce.z());
				std::cout<<"Force : "<<force_abs_coord<<", norm = "<<force_abs_coord.Length()<<", id_A = "<<id_A<<", id_B = "<<id_B<<", contact_surf ="<<
				CH_C_PI*Pow(contact_surf_radius,2)<<",rad = "<<rad_B<<"\n";
				// std::cout<<"Force abs coord in B  = "<<force_abs_coord<<"\n";
				// std::cout<<"Pressure in B before= "<<*m_pressure<<"\n";
				// std::cout<<"Added pressure in B = "<<(-force_abs_coord).Length()/(CH_C_PI*Pow(contact_surf_radius,2))<<"\n";
				if (!isnan(contact_surf_radius)){
				// *m_pressure = *m_pressure +(force_abs_coord).Length()/(CH_C_PI*Pow(contact_surf_radius,2));
				*m_pressure = *m_pressure +(force_abs_coord).Length()/(CH_C_PI*Pow(rad_B,2));

				}
				*m_nb_of_contact=*m_nb_of_contact+1;
				if (isnan(*m_pressure)){
					std::cout<<"Nan pressure, force = "<<force_abs_coord<<", surface radius = "<<contact_surf_radius<<"\n";
					std::cout<<"In square root : "<<Pow(rad_A,2)-Pow(Pow(rad_A,2)-Pow(rad_B,2)+Pow(dist_beads,2),2)/(4*Pow(dist_beads,2))<<"\n";
				}
				// std::cout<<"Pressure in B = "<<*m_pressure<<"\n";
				// std::cout<<"nb of contact in B :"<<*m_nb_of_contact<<"\n";
			}

		}

        // Check if contact involves obj2
        // if (modA == m_obj2.get()) {
        //     printf("  A contact on Obj 2 at pos: %7.3f  %7.3f  %7.3f", pA.x(), pA.y(), pA.z());
        // } else if (modB == m_obj2.get()) {
        //     printf("  B contact on Obj 2 at pos: %7.3f  %7.3f  %7.3f", pB.x(), pB.y(), pB.z());
        // }

        // const ChVector<>& nrm = plane_coord.Get_A_Xaxis();
        // printf("  nrm: %7.3f, %7.3f  %7.3f", nrm.x(), nrm.y(), nrm.z());
        // printf("  frc: %7.3f  %7.3f  %7.3f", cforce.x(), cforce.y(), cforce.z());
        // printf("  trq: %7.3f, %7.3f  %7.3f", ctorque.x(), ctorque.y(), ctorque.z());
        // printf("  penetration: %8.4f   eff. radius: %7.3f\n", distance, eff_radius);

        return true;
    }

	std::shared_ptr<ChBody> m_obj1;
	ChSystemMulticoreSMC* m_system;
	double* m_pressure;
	int* m_nb_of_contact;
};

class ContactReporter_walls : public ChContactContainer::ReportContactCallback {
  public:
    ContactReporter_walls(std::shared_ptr<ChBody> obj1, ChSystemMulticoreSMC* msystem, std::shared_ptr<ChVector<>> total_force ,
	std::shared_ptr<ChVector<>> pos_min,std::shared_ptr<ChVector<>> pos_max, int* nb_of_contact) :
		m_obj1(obj1), m_system(msystem), m_total_force(total_force), m_pos_min(pos_min), m_pos_max(pos_max),m_nb_of_contact(nb_of_contact) {}
	// ContactReporter_walls(std::shared_ptr<ChBody> obj1, ChSystemMulticoreSMC* msystem, std::shared_ptr<ChVector<>> total_force, int* nb_of_contact) :
	// 	m_obj1(obj1), m_system(msystem), m_total_force(total_force), m_nb_of_contact(nb_of_contact) {}

  private:
    virtual bool OnReportContact(const ChVector<>& pA,
                                 const ChVector<>& pB,
                                 const ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const double& eff_radius,
                                 const ChVector<>& cforce,
                                 const ChVector<>& ctorque,
                                 ChContactable* modA,
                                 ChContactable* modB) override {

		//Check if B obj is a bead !
		int id_B = modB->GetPhysicsItem()->GetIdentifier();
		int id_A = modA->GetPhysicsItem()->GetIdentifier();
		ChVector<double> pos_bead=m_system->SearchBodyID(id_B)->GetPos();

		if (id_B>=0 && id_A<0){
			double rad_B = m_system->SearchBodyID(id_B)->GetCollisionModel()->GetShapeDimensions(0).back();

			double dist_beads = (m_system->SearchBodyID(id_B)->GetPos()-m_system->SearchBodyID(id_A)->GetPos()).Length();


			// Check if contact involves obj1

			if (modA == m_obj1.get()){
				ChVector<> force_abs_coord = plane_coord * ChVector<>(cforce.x(),cforce.y(),cforce.z());

				std::cout<<force_abs_coord<<"\n";
				*m_total_force = *m_total_force-force_abs_coord;
				m_pos_min->Set(std::min(pos_bead.x(),m_pos_min->x()),std::min(pos_bead.y(),m_pos_min->y()),std::min(pos_bead.z(),m_pos_min->z()));
				m_pos_max->Set(std::max(pos_bead.x(),m_pos_max->x()),std::max(pos_bead.y(),m_pos_max->y()),std::max(pos_bead.z(),m_pos_max->z()));
				*m_nb_of_contact=*m_nb_of_contact+1;

			}
			else if (modB == m_obj1.get()){
				std::cout<<"Autre cas pour le contact \n";
			}

		}

        return true;
    }

	std::shared_ptr<ChBody> m_obj1;
	ChSystemMulticoreSMC* m_system;
	std::shared_ptr<ChVector<>> m_total_force;
	std::shared_ptr<ChVector<>> m_pos_min;
	std::shared_ptr<ChVector<>> m_pos_max;
	int* m_nb_of_contact;
};


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

std::pair<size_t, size_t> FillContainerOneColumn(ChSystemMulticoreSMC* msystem, const ConfigParameters& cp) {
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
	// double sft_z = marg * Sqrt(4.0 - (1.0 / Pow(cos(CH_C_PI / 6.0), 2.0)));
	double sft_z = 2.0 * marg;
	double sft_w = marg * tan(CH_C_PI / 6.0);

	// Set the max number of beads along the X, Y, and Z axis
	double numx = ceil(cp.clength_x / (marg * 2.0)) + 1;
	double numy = ceil(cp.clength_y / sft_y) + 1;
	// double numz = ceil(temp_height / sft_z) + 1;
	double numz = 10;

	// Create a generator for computing random sphere radius values that follow a normal distribution
	auto seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator((unsigned int)seed);
	std::normal_distribution<double> distribution(cp.grad, cp.gmarg / 6.0);

	// Construct a particle cloud inside of the box limits
	// ChVector<> pos_ref = ChVector<>(-cp.clength_x / 2.0 + marg, -cp.clength_y / 2.0 + marg, marg);
	ChVector<> pos_ref = ChVector<>(0,0, marg);
	ChVector<> init_v=ChVector<>(0,0,0);
	for (int iz = 0; iz < numz; ++iz) {

		double posx = 0;
		double posy = 0;
		double posz = sft_z * iz;

		ChVector<> pos_next = pos_ref + ChVector<>(posx, posy, posz);
		if (pos_next.z() <= temp_height - marg && pos_next.z() >= marg){
			double radius=distribution(generator);

			AddSphere(id++, msystem, cp, radius, pos_next, ChRandomXYZ(cp.gvel));
			std::cout<<"Bead id ="<<id<<", radius ="<<radius<<"\n";
			// AddSphere(id++, msystem, cp, distribution(generator), pos_next, init_v);
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
	// double temph = cp.clength_z ;
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

	// AddWall(--id, msystem, cp, sbase, pbase, false);
	// AddWall(--id, msystem, cp, srght, prght, false);
	// AddWall(--id, msystem, cp, srght, pleft, false);
	// AddWall(--id, msystem, cp, sback, pback, false);
	// AddWall(--id, msystem, cp, sback, pfrnt, false);
	// AddWall(--id, msystem, cp, sbase, proof, false);

	// Find and return index range of full wall list
	wlist.second = msystem->Get_bodylist().size() - 1;
	return wlist;
}

void AddControlBox(ChSystemMulticoreSMC* msystem, const ConfigParameters& cp, double z_pos_center_box){

	ChVector<> size = ChVector<>(cp.control_box_size)/2.0;
	ChVector<> container_center_pos=ChVector<>(0,0,z_pos_center_box);


	auto body = std::shared_ptr<ChBody>(msystem->NewBody());
	body->SetIdentifier(-100);
	body->SetMass(1);
	body->SetPos(container_center_pos);
	body->SetRot(QUNIT);
	body->SetBodyFixed(true);
	body->SetCollide(false);

	// Attach a color to the visible container
	#ifdef CHRONO_IRRLICHT
	double l_min = 0.05;
	ChColor color_edges = ChColor(0.f,0.f,0.f);

	ChVector<> size_z = ChVector<>(l_min,l_min,cp.control_box_size)/2.0;
	auto mvisual1 = chrono_types::make_shared<ChBoxShape>();
	mvisual1->GetBoxGeometry().Size = size_z;
	mvisual1->SetColor(color_edges);

	body->AddVisualShape(mvisual1,ChFrame<>(ChVector<>(1,1,0)*cp.control_box_size/2.0, QUNIT));
	body->AddVisualShape(mvisual1,ChFrame<>(ChVector<>(-1,-1,0)*cp.control_box_size/2.0, QUNIT));
	body->AddVisualShape(mvisual1,ChFrame<>(ChVector<>(-1,1,0)*cp.control_box_size/2.0, QUNIT));
	body->AddVisualShape(mvisual1,ChFrame<>(ChVector<>(1,-1,0)*cp.control_box_size/2.0, QUNIT));

	ChVector<> size_y = ChVector<>(l_min,cp.control_box_size,l_min)/2.0;
	auto mvisual2 = chrono_types::make_shared<ChBoxShape>();
	mvisual2->GetBoxGeometry().Size = size_y;
	mvisual2->SetColor(color_edges);

	body->AddVisualShape(mvisual2,ChFrame<>(ChVector<>(1,0,1)*cp.control_box_size/2.0, QUNIT));
	body->AddVisualShape(mvisual2,ChFrame<>(ChVector<>(-1,0,1)*cp.control_box_size/2.0, QUNIT));
	body->AddVisualShape(mvisual2,ChFrame<>(ChVector<>(1,0,-1)*cp.control_box_size/2.0, QUNIT));
	body->AddVisualShape(mvisual2,ChFrame<>(ChVector<>(-1,0,-1)*cp.control_box_size/2.0, QUNIT));

	ChVector<> size_x = ChVector<>(cp.control_box_size,l_min,l_min)/2.0;
	auto mvisual3 = chrono_types::make_shared<ChBoxShape>();
	mvisual3->GetBoxGeometry().Size = size_x;
	mvisual3->SetColor(color_edges);

	body->AddVisualShape(mvisual3,ChFrame<>(ChVector<>(0,-1,1)*cp.control_box_size/2.0, QUNIT));
	body->AddVisualShape(mvisual3,ChFrame<>(ChVector<>(0,1,1)*cp.control_box_size/2.0, QUNIT));
	body->AddVisualShape(mvisual3,ChFrame<>(ChVector<>(0,1,-1)*cp.control_box_size/2.0, QUNIT));
	body->AddVisualShape(mvisual3,ChFrame<>(ChVector<>(0,-1,-1)*cp.control_box_size/2.0, QUNIT));


	#endif

	// Add the wall to the system
	msystem->AddBody(body);


}



double pressure(ChSystemMulticoreSMC &msystem, const ConfigParameters& cp,double control_box_size,double z_pos_control_box,const size_t &num, const size_t &start_id,
							const size_t &num_walls, const size_t &start_id_walls){


	int nb_beads_in_control_box=0;
	double mean_pressure=0;
	//Get box center position
	ChVector<> container_center_pos=ChVector<>(0,0,z_pos_control_box);
	// ChVector<> container_center_pos=ChVector<>(0,0,0);

	//Loop through beads
	for (size_t i = start_id; i < start_id + num; ++i) {

		std::shared_ptr<ChBody> bead_tested = msystem.Get_bodylist().at(i);


		//Check if bead is in control box
		if (abs((bead_tested->GetPos().x()-container_center_pos.x()))<=control_box_size/2
		&& abs((bead_tested->GetPos().y()-container_center_pos.y()))<=control_box_size/2
		&& abs((bead_tested->GetPos().z()-container_center_pos.z()))<=control_box_size/2) {

			ChVector<> force = ChVector<>(0);

			// std::cout<<"New bead tested, force resetted \n";
			double pressure=0.0;
			int nb_of_contact = 0;
			// pressure+=bead_tested->GetContactForce().Length()/(CH_C_PI*cp.grad*cp.grad);
			nb_beads_in_control_box++;
			auto creporter = chrono_types::make_shared<ContactReporter>(bead_tested,&msystem,&pressure,&nb_of_contact);
			msystem.GetContactContainer()->ReportAllContacts(creporter);
			// std::cout<<"Total pressure = "<<pressure<<", Nb of contact : "<<nb_of_contact<<"\n";
			if (nb_of_contact >0){
				mean_pressure+=pressure/nb_of_contact;
			}
			// std::cout<<"Unaveraged mean_pressure : "<<mean_pressure<<"\n";
			// for (size_t j = start_id; j < start_id + num; ++j) {

			// 	if (j!=i){
			// 		std::shared_ptr<ChBody> bead_tested_2 = msystem.Get_bodylist().at(j);
			// 			auto creporter = chrono_types::make_shared<ContactReporter>(bead_tested, bead_tested_2,force);
			// 			msystem.GetContactContainer()->ReportAllContacts(creporter);
			// 			// std::cout<< "Force update  = "<<force<<", Length = "<<force.Length()<<"\n";
			// 	}

			// }
			// for (size_t j = start_id_walls; j < start_id_walls + num_walls; ++j) {
			// 	std::cout<<"Test contact with walls \n";
			// 	std::shared_ptr<ChBody> wall = msystem.Get_bodylist().at(j);
			// 	auto creporter = chrono_types::make_shared<ContactReporter>(bead_tested, wall,force);
			// 	msystem.GetContactContainer()->ReportAllContacts(creporter);
			// }

			// double bead_radius = bead_tested->GetCollisionModel()->GetShapeDimensions(0).back();

			// std::cout<<"Theoretical gravity force = "<<cp.grho * (4.0 / 3.0) * CH_C_PI * Pow(bead_radius, 3.0)*(msystem.Get_G_acc().Length())<<"\n";
			// std::cout<<"Applied force = "<<bead_tested->GetAppliedForce().Length()<<"\n";

			// real3 force_msys = msystem.GetBodyContactForce(bead_tested);
			// std::cout<<"Contact force system = "<<sqrt(Pow(force_msys.x, 2.0)+Pow(force_msys.y, 2.0)+Pow(force_msys.z,2.0))<<"\n ";
			// std::cout<<"Contact force = "<<bead_tested->GetContactForce().Length()<<"\n \n ";

			// const std::vector<std::shared_ptr<ChForce> >& force_list = bead_tested->GetForceList();
			// std::cout<<"Nb of forces = "<<force_list.empty()<<"\n\n";


		}


	}
	mean_pressure /= nb_beads_in_control_box;
	std::cout<<"There is "<<nb_beads_in_control_box<<" beads in control box \n";
	return mean_pressure;
}

double pressure_wall(ChSystemMulticoreSMC &msystem, const ConfigParameters& cp,const size_t &num, const size_t &start_id,
							const size_t &num_walls, const size_t &start_id_walls){

	double mean_pressure=0.0;

	//Loop through beads
	int nb_of_contacting_walls=0;
	for (int i_walls = 1; i_walls < 7 ; ++i_walls) {

		std::cout<<"i_walls = "<<i_walls<<"\n";
		std::shared_ptr<ChBody> wall_tested = msystem.SearchBodyID(-i_walls);
		double surf=0.0;


		// ChVector<> force = ChVector<>(0);
		// ChVector<> pos_min = ChVector<>(1000,1000,1000);
		// ChVector<> pos_max = ChVector<>(-1000,-1000,-1000);
		std::shared_ptr<ChVector<>> force_wall = std::make_shared<ChVector<>>(0,0,0);
		std::shared_ptr<ChVector<>> pos_min = std::make_shared<ChVector<>>(1000,1000,1000);
		std::shared_ptr<ChVector<>> pos_max = std::make_shared<ChVector<>>(-1000,-1000,-1000);

		// std::cout<<"New bead tested, force resetted \n";
		double pressure=0.0;
		int nb_of_contact = 0;
		// pressure+=bead_tested->GetContactForce().Length()/(CH_C_PI*cp.grad*cp.grad);
		// auto creporter = chrono_types::make_shared<ContactReporter_walls>(wall_tested,&msystem,&force,&pos_min,&pos_max,&nb_of_contact);
		// auto creporter = chrono_types::make_shared<ContactReporter>(wall_tested,&msystem,&force_wall,pos_min,pos_max,&nb_of_contact);
		auto creporter = chrono_types::make_shared<ContactReporter_walls>(wall_tested,&msystem,force_wall,pos_min,pos_max,&nb_of_contact);
		msystem.GetContactContainer()->ReportAllContacts(creporter);
		// std::cout<<"Total pressure = "<<pressure<<", Nb of contact : "<<nb_of_contact<<"\n";
		std::cout<<"Total force = "<<*force_wall<<", Nb of contacts = "<<nb_of_contact<<"\n";
		if (nb_of_contact>0){
			nb_of_contacting_walls+=1;
			std::cout<<"surf_x = "<<pos_max->x()-pos_min->x()<<", surf_y = "<<pos_max->y()-pos_min->y()<<", surf_z = "<<pos_max->z()-pos_min->z()<<"\n";
			if (i_walls==2 || i_walls==3){
				surf=(pos_max->y()-pos_min->y())*(pos_max->z()-pos_min->z());
				if (surf==0.0){
					surf=CH_C_PI*cp.grad*cp.grad;
				}
				mean_pressure+=abs(force_wall->x())/surf;
				std::cout<<"Pressure on wall "<<-i_walls<<" : "<<abs(force_wall->x())/surf<<", surf = "<<surf<<"\n";
				std::cout<<"Pos_min : "<<*pos_min<<", pos_max : "<<*pos_max<<"\n";
			}
			else if (i_walls==4 || i_walls==5){
				surf=(pos_max->x()-pos_min->x())*(pos_max->z()-pos_min->z());
				if (surf==0.0){
					surf=CH_C_PI*cp.grad*cp.grad;
				}
				mean_pressure+=abs(force_wall->y())/surf;
				std::cout<<"Pressure on wall "<<-i_walls<<" : "<<abs(force_wall->y())/surf<<", surf = "<<surf<<"\n";
				std::cout<<"Pos_min : "<<*pos_min<<", pos_max : "<<*pos_max<<"\n";
			}
			else{
				surf=(pos_max->x()-pos_min->x())*(pos_max->y()-pos_min->y());
				if (surf==0.0){
					surf=CH_C_PI*cp.grad*cp.grad;
				}
				mean_pressure+=abs(force_wall->z())/surf;
				std::cout<<"Pressure on wall "<<-i_walls<<" : "<<abs(force_wall->z())/surf<<", surf = "<<surf<<"\n";
				std::cout<<"Pos_min : "<<*pos_min<<", pos_max : "<<*pos_max<<"\n";
			}
		}
		else{
			std::cout<<"Wall "<<-i_walls<<" : no contact \n";
		}
	}


	return mean_pressure/nb_of_contacting_walls;
}

double average(std::vector<double> const& v,int length){
    if(v.empty()){
        return 0;
    }

	double sum = 0.0;
	int sz = v.size();

	std::cout<<"On entre dans average, taille de pos = "<<sz<<"\n";
	if (sz<length){
		return average(v,sz);
	}

	int i=0;
	int count = 0;
	while (i<sz-1 && count<length){
		std::cout<<"Boucle while de average, i "<<i<<"\n";
		if (!isnan(v[sz-i-1])){
			sum+=v[sz-i-1];
			count++;

		}
		i++;
	}
	sum/= count;
	std::cout<<"On sort de average, Pressure mean ="<<sum<<"\n";
	return sum;
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
	// std::pair<size_t, size_t> glist = FillContainerOneColumn(&msystem, cp);
	double z_pos_control_box=cp.clength_z/2.0;
	AddControlBox(&msystem,cp,z_pos_control_box);
	// AddControlBox(&msystem,cp,ChVector<>(0,0,0));
	std::cout<<"cp.control_box_size : "<<cp.control_box_size<<"\n";

	// Create an object to track certain system info and stats
	SystemData mstats;
	if (SetSystemStats(mstats, msystem, cp.time_step, wlist, glist) != 0) {
		fprintf(stderr, "\nERR: Error setting system stats\n");
		return -1;
	}

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
	application->AddCamera(ChVector<>(0, cp.clength_x * 2, cp.clength_z), ChVector<>(0,0,cp.clength_z)/2.0);
	application->AddLight(ChVector<>(0, cp.clength_z * 2, cp.clength_z), cp.clength_z * 2);
	// auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    // vis->AttachSystem(&msystem);
    // vis->SetWindowSize(800, 600);
    // vis->SetWindowTitle("Paths");
    // vis->Initialize();
    // vis->AddLogo();
    // vis->AddSkyBox();
    // vis->AddCamera(ChVector<>(0, 4, -6));
    // vis->AddTypicalLights();

	#endif

	// Set the soft run-time parameters
	bool settled = false;
	bool flattened = false;

	double time = 0.0;
	double time_loop = cp.time_loop;
	double time_save = cp.time_save;

	std::vector<double> pressure_value;

	// vis->SetSymbolScale(0.2);
    // vis->EnableContactDrawing(ContactsDrawMode::CONTACT_NORMALS);

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


		// pressure_value.push_back(pressure(msystem, cp,cp.control_box_size,z_pos_control_box,mstats.num_particles, glist.first,mstats.num_walls,wlist.first));
		pressure_value.push_back(pressure_wall(msystem, cp,mstats.num_particles, glist.first,mstats.num_walls,wlist.first));
		double total_mass=0.0;
		double total_height=0.0;
		double r_0=0.0;
		for (size_t i = glist.first; i < glist.first + mstats.num_particles; ++i) {

			std::shared_ptr<ChBody> bead_tested = msystem.Get_bodylist().at(i);
			// std::cout<<"Particle at "<<bead_tested->GetPos()<<" has a mass of "<<bead_tested->GetMass()<<"\n";
			total_mass+=bead_tested->GetMass();
			total_height+=2*bead_tested->GetCollisionModel()->GetShapeDimensions(0).back();
			// if (bead_tested->GetPos().z()>=z_pos_control_box+cp.control_box_size/2.0){
			// 	total_mass+=bead_tested->GetMass();
			// 	total_height+=2*bead_tested->GetCollisionModel()->GetShapeDimensions(0).back();
			// }
			// else if (bead_tested->GetPos().z()>=z_pos_control_box-cp.control_box_size/2.0 &&
			// bead_tested->GetPos().z()<=z_pos_control_box+cp.control_box_size/2.0)
			// {
			// 	r_0=bead_tested->GetCollisionModel()->GetShapeDimensions(0).back();
			// }
		}
		std::cout<<"Mass of all the particles"<<total_mass<<"\n";
		std::cout<<"Total height = "<<total_height<<"\n";
		// std::cout<<"Total height = "<<total_height<<", Expected pressure = "<<-cp.grho*cp.grav_z*total_height<<"\n";
		// std::cout<<"More accuracte expected pressure = "<<-total_mass*cp.grav_z/(CH_C_PI*pow(r_0,2))<<"\n";
		std::cout<<"Pressure = "<<pressure_value.back()<<"Pa \n \n \n";

		// Save and write data every (cp.time_save / time_step) steps (particles only, not walls)
		if (time > time_save) {
			if (ArchiveState(msystem, mstats, cp.proj_path, mstats.num_particles, glist.first, time) != 0) {
				fprintf(stderr, "\nERR: Error archiving run-time data \n");
				return -1;
			}
			time_save += cp.time_save;


		}


		// Calculate the average kinetic energy of all particles. If the max particle vel < threshold, exit the loop.

		if (CalcAverageKE(msystem, cp.proj_path, mstats.num_particles, glist.first, time, 1000)) break;
	}

	std::cout<<"End of step 1, now moving the roof \n";
	//LOOP 2 : move the roof
	std::shared_ptr<ChBody> wall = msystem.SearchBodyID(-6);
	double max_height=0.0;
	for (size_t i = glist.first; i < glist.first + mstats.num_particles; ++i) {

			std::shared_ptr<ChBody> bead_tested = msystem.Get_bodylist().at(i);
			if (max_height<bead_tested->GetPos().z()){
				max_height=bead_tested->GetPos().z();
			}

	}

	if (max_height+3*cp.grad>cp.clength_z){
		wall->SetPos(ChVector<>(0,0,max_height+3*cp.grad));
	}
	else{
		wall->SetPos(ChVector<>(0,0,cp.cthickness+cp.clength_z));
	}
	std::cout<<"Wall pos : "<<wall->GetPos()<<"\n";
	while (wall->GetPos().z()>cp.cthickness+cp.clength_z){
		#ifdef CHRONO_IRRLICHT
		application->BeginScene();
		application->Render();
		application->GetDevice()->run();
		application->EndScene();
		#endif

		// msystem.DoStepDynamics(cp.time_step);
		// time += cp.time_step;

		while (time < time_loop) {
			msystem.DoStepDynamics(cp.time_step);
			time += cp.time_step;

			IncrementTimerTotals(msystem, &mstats, time);
			// if (time > cp.sim_duration) break;

			if (CalcAverageKE(msystem, cp.proj_path, mstats.num_particles, glist.first, time, 2000)) break;
			// std::cout<<"Time = "<<time<<", Time_loop = "<<time_loop<<"\n";

		}
		// std::cout<<"\n \n";
		time_loop = time - cp.time_step + cp.time_loop;

		// IncrementTimerTotals(msystem, &mstats, time);
		// if (time > cp.sim_duration) break;

		// wall->SetPos(wall->GetPos()-ChVector<>(0,0,cp.grad-2*cp.gmarg/3));
		wall->SetPos(wall->GetPos()-ChVector<>(0,0,0.1*cp.grad));

	}

	// wall->SetPos(ChVector<>(wall->GetPos().x(),wall->GetPos().y(),cp.cthickness+cp.clength_z));

	std::shared_ptr<ChBody> wall_right = msystem.SearchBodyID(-2);
	std::shared_ptr<ChBody> wall_left = msystem.SearchBodyID(-3);
	std::shared_ptr<ChBody> wall_back = msystem.SearchBodyID(-4);
	std::shared_ptr<ChBody> wall_front = msystem.SearchBodyID(-5);
	std::shared_ptr<ChBody> wall_roof = msystem.SearchBodyID(-6);

	double displacement = cp.clength_x/50;
	double targeted_pressure= 80000;
	// LOOP 3 : move the walls to increase the pressure
	while (average(pressure_value,10)<targeted_pressure){
		wall_right->SetPos(wall_right->GetPos()+ChVector<>(-1,0,0)*displacement);
		wall_left->SetPos(wall_left->GetPos()+ChVector<>(1,0,0)*displacement);
		wall_back->SetPos(wall_back->GetPos()+ChVector<>(0,1,0)*displacement);
		wall_front->SetPos(wall_front->GetPos()+ChVector<>(0,-1,0)*displacement);
		wall_roof->SetPos(wall_roof->GetPos()+ChVector<>(0,0,-1)*displacement);
		// std::cout<<"\n \n Mean pressure = "<<average(pressure_value,10)<<", walls moved \n\n\n";
		int count_iter=0;
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


			// pressure_value.push_back(pressure(msystem, cp,cp.control_box_size,z_pos_control_box,mstats.num_particles, glist.first,mstats.num_walls,wlist.first));
			pressure_value.push_back(pressure_wall(msystem, cp,mstats.num_particles, glist.first,mstats.num_walls,wlist.first));
			std::cout<<"Pressure = "<<pressure_value.back()<<"Pa \n \n \n";

			// Save and write data every (cp.time_save / time_step) steps (particles only, not walls)
			if (time > time_save) {
				if (ArchiveState(msystem, mstats, cp.proj_path, mstats.num_particles, glist.first, time) != 0) {
					fprintf(stderr, "\nERR: Error archiving run-time data \n");
					return -1;
				}
				time_save += cp.time_save;


			}

			for (size_t i = glist.first; i < glist.first + mstats.num_particles; ++i) {

				std::shared_ptr<ChBody> bead_tested = msystem.Get_bodylist().at(i);
				if (bead_tested->GetPos().Length()>30){
					bead_tested->SetBodyFixed(true);
					bead_tested->SetPos_dt(ChVector<>(0,0,0));
				}

	}

			// Calculate the average kinetic energy of all particles. If the max particle vel < threshold, exit the loop.
			if (CalcAverageKE(msystem, cp.proj_path, mstats.num_particles, glist.first, time, 300)) break;
			if (count_iter>50){
				std::cout<<"Too many iterations, break the loop";
				break;
			}
			count_iter++;
		}
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