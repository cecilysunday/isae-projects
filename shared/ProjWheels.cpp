// =============================================================================
// Use these shared functions for the Chrono Wheel programs found on
// cecilysunday/chrono-wheels. Build the programs with the Project Chrono 
// code version available on cecilysunday/chrono.
//
// Authors: Cecily Sunday
//
// =============================================================================
//
// TODO: 
//	- Check that inertial properties are correct
//  - Add mesh import
//
// =============================================================================


#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "ProjWheels.h"
#include "ProjConfig.h"

using namespace chrono;
using namespace chrono::collision;




void UpdateWheelMotor(const ConfigParameters &config, const WheelParameters &mwheel) {
	// Catch possible division error
	double acc = config.wracc;
	if (config.wracc == 0) acc = 1.0;
	
	// Create a motor sequence with a ramp-up phase and then a constant rotation speed
	auto framp = chrono_types::make_shared<ChFunction_Ramp>(0, acc);
	auto fconst = chrono_types::make_shared<ChFunction_Const>(config.wrvel);

	auto fseq = chrono_types::make_shared<ChFunction_Sequence>();
	fseq->InsertFunct(framp, config.wrvel / acc, 1.0, false, false, false, 0);
	fseq->InsertFunct(fconst, config.time_sim, 1.0, true, false, false, 1);
	fseq->Setup();

	// Update the motor speed function
	mwheel.wheel_motor->SetSpeedFunction(fseq);

	return;
}


void SetRigLinks(ChSystemMulticoreSMC* msystem, const ConfigParameters &config, WheelParameters* mwheel) {
	// Get a pointer to the ground, assuming that the ground is the first body in the system
	std::shared_ptr<ChBody> ground = msystem->Get_bodylist().at(0);

	// Add a link between the chassis and the ground to constrain the chassis to the XZ plane
	ChQuaternion<> z2y;
	z2y.Q_from_AngAxis(-CH_C_PI / 2.0, ChVector<>(1, 0, 0));
	auto c_link = chrono_types::make_shared<ChLinkLockOldham>();  // ChLinkLockOldham, ChLinkLockPlanePlane
	msystem->AddLink(c_link);
	c_link->Initialize(ground, mwheel->chassis_body, ChCoordsys<>(VNULL, z2y));

	// Add a motor between the chassis and wheel. Set the inital motor speed to zero
	auto w_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
	msystem->AddLink(w_motor);
	w_motor->Initialize(mwheel->wheel_body, mwheel->chassis_body, ChFrame<>(mwheel->wheel_offset, z2y));
	w_motor->SetSpeedFunction(chrono_types::make_shared<ChFunction_Const>(0));
	mwheel->wheel_motor = w_motor;

	return;
}


void AdjustInitialOffset(ChSystemMulticoreSMC* msystem, const ConfigParameters &config, WheelParameters* mwheel) {
	// Find the highest point in the terrain directly below the wheel 
	double highest = 0;
	double xlim = mwheel->wheel_radius + config.grad / 2.0;
	double ylim = mwheel->wheel_width / 2.0 + config.grad / 2.0;

	for (auto body : msystem->Get_bodylist()) {
		if (body->GetIdentifier() >= 0) {
			if (std::abs(body->GetPos().x()) <= xlim && std::abs(body->GetPos().y()) <= ylim) {
				double rad = body->GetCollisionModel().get()->GetShapeDimensions(0).at(0);
				double h = body->GetPos().z() + rad;
				if (h > highest) highest = h;
			}
		}
	}

	// Adjust the initial offset of the wheel based on the actual terrain height
	double dheight = highest + 1.0E-5 - config.cheight;
	mwheel->wheel_offset.z() = mwheel->wheel_offset.z() + dheight;
	
	ChVector<> pos = mwheel->wheel_offset;
	mwheel->wheel_body->SetPos(pos);
	if (mwheel->chassis_id != -1000) mwheel->chassis_body->SetPos(pos);

	return;
}


void AddWheel(ChSystemMulticoreSMC* msystem, const ConfigParameters &config, WheelParameters* mwheel) {
	// Generate the id number for the wheel
	int id = (msystem->Get_bodylist().size() + 1) * -1;

	// Get the wheel properties
	double mass = mwheel->wheel_mass;
	double radius = mwheel->wheel_radius;
	double width = mwheel->wheel_width;

	ChVector<> pos = mwheel->wheel_offset;
	ChVector<> inertia = mwheel->wheel_inertia;

	// Set the wheel parameters and collision model
	auto wheel = std::shared_ptr<ChBody>(msystem->NewBody());
	wheel->SetIdentifier(id);
	wheel->SetMass(mass);
	wheel->SetPos(pos);
	wheel->SetInertiaXX(inertia);
	wheel->SetBodyFixed(true);
	wheel->SetCollide(false);

	// Add the wheel body to the system based on the wheel type
	wheel->GetCollisionModel()->ClearModel();
	if (config.wtype == "sphere") {
		utils::AddSphereGeometry(wheel.get(), config.mat_pw, radius);
	}
	else if (config.wtype == "plate") {
		utils::AddBoxGeometry(wheel.get(), config.mat_pw, ChVector<>(radius, width / 2.0, width / 2.0));
	}
	else if (config.wtype == "cylinder") {
		utils::AddCylinderGeometry(wheel.get(), config.mat_pw, radius, width / 2.0);
	}
	else if (config.wtype == "paddle_1" || config.wtype == "paddle_2") {
		double g_length = 30.0;
		if (config.wtype == "paddle_2") g_length = 22.5;

		double g_thickness = 2.5;
		double g_offset = 2.0;
		double g_radius = radius - (g_length + g_offset) / 2.0;

		utils::AddCylinderGeometry(wheel.get(), config.mat_pw, radius - g_length, width / 2.0);
		for (int i = 0; i < 9; ++i) {
			double theta = i * (2.0 * CH_C_PI / 9.0);
			ChQuaternion<> z2g;
			z2g.Q_from_AngY(theta);
			utils::AddBoxGeometry(wheel.get(), config.mat_pw,
				ChVector<>(g_thickness, width, g_length + g_offset) / 2.0,
				ChVector<>(g_radius * sin(theta), 0, g_radius * cos(theta)),
				z2g);
		}
	}
	else {
		utils::AddCylinderGeometry(wheel.get(), config.mat_pw, radius, width / 2.0);
	}
	wheel->GetCollisionModel()->BuildModel();

	// Add the body to the system
	msystem->AddBody(wheel);

	// Add the wheel body and ID to the WheelParameters struct
	mwheel->wheel_id = msystem->Get_bodylist().size() - 1;
	mwheel->wheel_body = wheel;

	return;
}


void AddChassis(ChSystemMulticoreSMC* msystem, const ConfigParameters& config, WheelParameters* mwheel) {
	// Generate the id number for the chassis
	int id = (msystem->Get_bodylist().size() + 1) * -1;

	// Set the chassis properties based on the wheel
	double mass = (config.wmass - mwheel->wheel_mass > 0) ? config.wmass - mwheel->wheel_mass : 1.0;
	double radius = mwheel->wheel_radius;
	ChVector<> inertia = 0.4 * mass * radius * radius * ChVector<>(1, 1, 1);

	// Create an invisible cylindrical body to act as the rover chassis
	auto chassis = std::shared_ptr<ChBody>(msystem->NewBody());
	chassis->SetIdentifier(id);
	chassis->SetMass(mass);
	chassis->SetPos(mwheel->wheel_offset);
	chassis->SetInertiaXX(inertia);
	chassis->SetBodyFixed(true);
	chassis->SetCollide(false);

	chassis->GetCollisionModel()->ClearModel();
	utils::AddSphereGeometry(chassis.get(), config.mat_pw, radius, VNULL, QUNIT, false);
	chassis->GetCollisionModel()->BuildModel();

	// Add the body to the system
	msystem->AddBody(chassis);

	// Add the chassis body and index to the WheelParameters structure
	mwheel->chassis_id = msystem->Get_bodylist().size() - 1;
	mwheel->chassis_body = chassis;

	return;
}


void SetWheelOffset(const ConfigParameters &config, WheelParameters* mwheel) {
	// Adjust the wheel vertical offset based on wheel type
	double voffset = config.cheight + config.cmisc + mwheel->wheel_radius;
	if (config.wtype == "plate") voffset = config.cheight + config.cmisc + mwheel->wheel_width / 2.0;
	if (config.wtype == "paddle_1" || config.wtype == "paddle_2") voffset = voffset - 6.45;

	// Adjust the wheel horizontal offset based on wmisc
	double hoffset = (100 + mwheel->wheel_radius * config.wmisc) - (config.clength / 2.0);
	if (config.wmisc == -1) hoffset = 0;

	// Add the wheel offset to the WheelParameters structure
	mwheel->wheel_offset = ChVector<>(hoffset, 0, voffset);
	return;
}


void AdjustWheelMass(const ConfigParameters &config, WheelParameters* mwheel) {
	// If this is a sink sim, adjust the wheel mass and inertia to include the chassis mass
	double temp_mass = mwheel->wheel_mass;
	ChVector<> temp_inertia = mwheel->wheel_inertia / temp_mass;

	if (config.projtype == "sink_run") {
		mwheel->wheel_mass = config.wmass;
		mwheel->wheel_inertia = mwheel->wheel_mass * temp_inertia;
	}

	return;
}


int SetWheelParameters(const ConfigParameters &config, WheelParameters* mwheel) {
	// Define the dimensions of the wheel according to the wheel type
	if (config.wtype == "sphere") {
		mwheel->wheel_mass = 1000.0;
		mwheel->wheel_radius = 50.0;
		mwheel->wheel_width = 100.0;
		mwheel->wheel_inertia = ChVector<>(1000000.0, 1000000.0, 1000000.0);
	}
	else if (config.wtype == "plate") {
		mwheel->wheel_mass = 1000.0;
		mwheel->wheel_radius = 50.0;
		mwheel->wheel_width = 50.0;
		mwheel->wheel_inertia = ChVector<>(416666.667, 1041666.667, 1041666.667);
	}
	else if (config.wtype == "cylinder") {
		mwheel->wheel_mass = 1000.0;
		mwheel->wheel_radius = 77.0;
		mwheel->wheel_width = 53.0;
		mwheel->wheel_inertia = ChVector<>(1716333.0, 2964500.0, 1716333.0);
	}
	else if (config.wtype == "paddle_1") {
		mwheel->wheel_mass = 640.476;
		mwheel->wheel_radius = 107.0;
		mwheel->wheel_width = 53.0;
		mwheel->wheel_inertia = ChVector<>(1559609.591, 2859618.325, 1559609.591);
	}
	else if (config.wtype == "paddle_2") {
		mwheel->wheel_mass = 871.759;
		mwheel->wheel_radius = 107.0;
		mwheel->wheel_width = 53.0;
		mwheel->wheel_inertia = ChVector<>(2369693.815, 4371507.676, 2369693.815);
	}
	else {
		fprintf(stderr, "Error mapping wheel type to wheel properties!\n");
		return -1;
	}

	// If this is a sink sim, adjust the wheel mass and inertia to account for the chassis properties
	AdjustWheelMass(config, mwheel);

	// Set the initial wheel position with respect to (0, 0, 0)
	SetWheelOffset(config, mwheel);

	return 0;
}