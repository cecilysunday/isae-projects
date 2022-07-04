// =============================================================================
// Use these shared functions for the Chrono Wheel programs found on
// cecilysunday/chrono-wheels. Build the programs with the Project Chrono 
// code version available on cecilysunday/chrono.
//
// Authors: Cecily Sunday
// =============================================================================

#ifndef PROJWHEEL_H
#define PROJWHEEL_H

using namespace chrono;

typedef struct WheelParameters {
	int chassis_id = -1000;					
	int wheel_id = -1000;					

	double wheel_mass = 1.0;
	double wheel_radius = 107.0;
	double wheel_width = 53.0;

	ChVector<> wheel_inertia = ChVector<>(4321.333, 8174.500, 4321.333);
	ChVector<> wheel_offset = ChVector<>(0, 0, 107.0);

	std::shared_ptr<ChBody> chassis_body;
	std::shared_ptr<ChBody> wheel_body;
	std::shared_ptr<ChLinkMotorRotationSpeed> wheel_motor;

} WheelParameters;


// Forward declarations
struct ConfigParameters;

// Change the wheel speed from 0 to a non-zero value with a constant ram rate
void UpdateWheelMotor(const ConfigParameters &config, const WheelParameters &mwheel);

// Connect the chassis to the ground and the wheel to the chassis
void SetRigLinks(ChSystemMulticoreSMC* msystem, const ConfigParameters& config, WheelParameters* mwheel);

// Adjust the starting position of the wheel based on the height point in the settles terrain patch
void AdjustInitialOffset(ChSystemMulticoreSMC* msystem, const ConfigParameters &config, WheelParameters* mwheel);

// Create a wheel body based on a the wheel-type specified in the config file
void AddWheel(ChSystemMulticoreSMC* msystem, const ConfigParameters &config, WheelParameters* mwheel);

// Create an invisible, spherical chassis body
void AddChassis(ChSystemMulticoreSMC* msystem, const ConfigParameters& config, WheelParameters* mwheel);

// Find and set the initial offset position of the wheel 
void SetWheelOffset(const ConfigParameters &config, WheelParameters* mwheel);

// Adjust the wheel mass to hold include the chassis mass
void AdjustWheelMass(const ConfigParameters &config, WheelParameters* mwheel);

// Poplate the WheelParameters data structure based on inputs from the config file
int SetWheelParameters(const ConfigParameters& config, WheelParameters* mwheel);

#endif