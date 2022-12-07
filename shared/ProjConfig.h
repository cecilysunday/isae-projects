// =============================================================================
//
// Helper functions to set simulation parameters by reading in a config file
//
// =============================================================================

#ifndef PROJCONFIG_H
#define PROJCONFIG_H

using namespace chrono;
using namespace chrono::collision;


typedef struct ConfigParameters {
	// Project information
	std::string proj_config = "NA";					/// File containing main configure parameters
	std::string proj_setdir = "NA";					/// File containing configure parameters for 'set' simulation
	std::string proj_type = "NA";					/// Project type, e.g. set, penetrometer, impact, wheel, repose
	std::string proj_container = "NA";				/// Project container shape, e.g. box or cyl
	std::string proj_intruder = "NA";				/// Project intruder shape, e.g. box, cyl, or sph 
	std::string proj_date;							/// Date that test was executed
	std::string proj_time;							/// Time test was executed
	std::string proj_units = "NA";					/// Length, mass, and time units
	std::string proj_path;							/// Path to output files

	// Material properties shared between all bodies
	float youngs = 7.0E5f;							/// Young's modulus
	float poisson = 0.24f;							/// Poisson's ratio
	float mu_pp = 0.16f;							/// Coefficient of static and dynamic friction, particle-particle
	float mu_pw = 0.45f;							/// Coefficient of static and dynamic friction, particle-wall
	float mu_roll = 0.09f;							/// Coefficient of rolling friction, usually ranges from 0.05-0.1
	float mu_spin = 0.0f;							/// Coefficient of spinning friction, usually < 0.05
	float cor_pp = 0.9f;							/// Coefficient of restitution, particle-particle
	float cor_pw = 0.5f;							/// Coefficient of restitution
	float ad = 0.0f;								/// Magnitude of the adhesion in the selected adhesion model

	// Geometic properties of the container
	double clength_x = 0.0;							/// Container dimension along the x-axis
	double clength_y = 0.0;							/// Container dimension along the z-axis
	double clength_z = 0.0;							/// Container dimension along the y-axis
	double clength_r = 0.0;							/// Radius of cylindrical container (derived value)
	double cthickness = 0.0;						/// Thickness of container walls
	double cmass = 0.0;								/// Mass of container walls

	// Geometic properties of the granular surface material
	double gdia = 0.0;								/// Grain diameter 
	double grad_std=0.0;							/// Grain size distribution - standard deviation
	double gmarg = 0.0;								/// Margin envelope around each grain
	double grho = 0.0;								/// Grain density
	double gvel = 0.0;								/// Initial mixing velocity of grains
	double gmass = 0.0;								/// Grain mass
	double grad = 0.0;								/// Grain Radius

	// Geometric properties of the intruder
	double ilength_x = 0.0;							/// Intruder dimension along the x-axis
	double ilength_y = 0.0;							/// Intruder dimension along the z-axis
	double ilength_z = 0.0;							/// Intruder dimension along the y-axis
	double ilength_r = 0.0;							/// Radius of round intruders (derived value)
	double imass = 0.0;								/// Mass of the intruder
	double ivel = 0.0;								/// Initial translational velocity of the intruder
	double iwvel = 0.0;								/// Initial rotational velocity of the intruder

	// Simulation run-time parameters
	double grav_x = 0.0;							/// Gravity along the x-axis
	double grav_y = 0.0;							/// Gravity along the y-axis
	double grav_z = 0.0;							/// Gravity along the z-axis
	double time_step = 0.0;							/// Time step between CalcForces function call
	double time_loop = 0.0;							/// Time between each vis and calc update
	double time_save = 0.0;							/// Time between output of full data files
	double sim_duration = 0.0;						/// Max sim runtime

	// Simulation solver and collision detection parameters
	double tolerance = 1e-3;
	double min_roll_vel = 1E-5;
	double min_spin_vel = 1E-5;

	//Pressure measurement
	int control_box_size_nb_of_grain_radius=0;
	double control_box_size=0.0;

	//Angle of repose
	double dist_funnel_platform=0.0;
	double angle_funnel=0.0;
	double ratio_funnel_small_bead_diameter=0.0;
	double ratio_funnel_small_funnel_large = 0.0;
	double platform_size = 0.0;
	double funnel_small_dia=0.0;
	double funnel_large_dia=0.0;
	double height_stem=0.0;
	std::string config_platform_file = "NA"; 

	uint max_itbilateral = 100;

	int bpa_x = 10;
	int bpa_y = 10;
	int bpa_z = 10;
	
	ChSystemSMC::ContactForceModel  force_model = ChSystemSMC::ContactForceModel::Hertz;
	ChSystemSMC::AdhesionForceModel adhesion_model = ChSystemSMC::AdhesionForceModel::Constant;
	ChSystemSMC::TangentialDisplacementModel tandisp_model = ChSystemSMC::TangentialDisplacementModel::MultiStep;
	ChCollisionSystemType collision_system = ChCollisionSystemType::CHRONO;
	ChNarrowphase::Algorithm narrowphase_type = ChNarrowphase::Algorithm::HYBRID;

	// Materials containers
	std::shared_ptr<ChMaterialSurfaceSMC> mat_pp;
	std::shared_ptr<ChMaterialSurfaceSMC> mat_pw;

} ConfigParameters;

// Add a bounding box to the system to keep particles from floating too far away
void AddBoundingBox(ChSystemMulticoreSMC* msystem, const ConfigParameters& config, const double height);

// Set the wall and particle material properties based on inputs from the ConfigParameters structure
int SetSimulationMaterial(ConfigParameters& config);

// Set the simulation parameters based on inputs from the ConfigParameters structure
int SetSimulationParameters(ChSystemMulticoreSMC* msystem, ConfigParameters& config);

// Map the models from the config file to the appropriate chrono enums
ChSystemSMC::ContactForceModel force_to_enum(const std::string &str);
ChSystemSMC::AdhesionForceModel adhesion_to_enum(const std::string &str);
ChSystemSMC::TangentialDisplacementModel tandisp_to_enum(const std::string& str);
ChCollisionSystemType collision_to_enum(const std::string& str);
ChNarrowphase::Algorithm narrowphase_to_enum(const std::string& str);

// Map the chrono enums to the appropriate model strings in the config file
const std::string enum_to_string(const ChSystemSMC::ContactForceModel &model);
const std::string enum_to_string(const ChSystemSMC::AdhesionForceModel &model);
const std::string enum_to_string(const ChSystemSMC::TangentialDisplacementModel& model);
const std::string enum_to_string(const ChCollisionSystemType& model);
const std::string enum_to_string(const ChNarrowphase::Algorithm& model);

// Write all simulation parameters using the ConfigParameters structure
int WriteParameterFile(const ConfigParameters& config);

// Make a copy of the config file using the ConfigParameters structure
int WriteConfigFile(const ConfigParameters& config);

// Create the data-output directories
int CreateOutputPath(ConfigParameters* config, const std::string &datestr, const std::string &timestr);

// Populate the derived parameters in the ConfigParameters structure
int ComputeDerivedStuctValues(ConfigParameters* config, const std::string& datestr, const std::string& timestr);

// Fill the ConfigParameter structure using the config file
int MapParameters(ConfigParameters& config, const std::vector<std::string>& parsed_line);

// Get the configuration file, parse the contents of the file, and map the parameters to the ConfigParameter structure
int ImportConfigFile(ConfigParameters* config, const std::string& cfile);

// Import config file, create output paths, compute derived struct values, write a copy of the config file,
int InitializeSimulation(ConfigParameters* config, const std::string &cfile);

// Get the name of the config file. Return an error if none is provided.
char* GetConfigFile(int argc, char* argv[]);

#endif