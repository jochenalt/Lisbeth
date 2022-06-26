///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief This is the header for Controller class
///
/// \details Handle the communication between the blocks of the control architecture
///
//////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CONTROLLER_H_INCLUDED
#define CONTROLLER_H_INCLUDED

#include "Params.hpp"
#include "Estimator.hpp"
#include "Gait.hpp"
#include "FootstepPlanner.hpp"
#include "StatePlanner.hpp"
#include "FootTrajectoryGenerator.hpp"
#include "WBCController.hpp"
#include "Filter.hpp"
#include "MPCController.hpp"

class Controller {
	public:
  		Controller();
  		~Controller() {};

	  	void initialize(Params& params);
	  	void compute(Vector3 const& imuLinearAcceleration,
	  				 Vector3 const& imuGyroscopse,
	  				 Vector3 const& imuAttitudeEuler,
	  				 Vector12 const& jointsPositions,
			 		 Vector12 const& jointsVelocities);
	  	void security_check();

		// command from remote: stop/go
	  	void command_stop(bool ok) { cmd_stop = ok; }

	  	// command from remote: set speed
	  	void command_speed(double vX, double vY, double heightZ, double rotX, double rotY, double angSpeedZ);

	  	// command from remote: new gait
	  	void command_gait(int newGait) { cmd_gait = (GaitType)newGait;}

	  	Vector12 P;       // Proportional gains
	  	Vector12 D;       // Derivative gains
	  	Vector12 q_des;   // Desired joint positions
	  	Vector12 v_des;   // Desired joint velocities
	  	Vector12 tau_ff;  // Desired joint torques
	  	Vector12 FF;      // Torque gains (0 < FF < 1)


	private:
		void init_robot(Params& params);


	  	// Control info
	  	Params* params_;       // Object that stores parameters
	  	bool error;            // Error flag to stop the controller
	  	int error_flag;        // Value depends on what set the error flag to true
	  	Vector12 error_value;  // Store data about the error

 		int k;          // Number of wbc time steps since the start of the controller
  		int k_mpc;      // Number of wbc time steps for each MPC time step
  		double h_ref_;  // Reference height of the base

  		// Classes of the different control blocks
		Estimator estimator;                              // Estimator control block
  		Gait gait;                                        // Gait control block
  		FootstepPlanner footstepPlanner;                  // Footstep planner control block
  		StatePlanner statePlanner;                        // State planner control block
  		MPCController mpcController;                            // MPC Wrapper control block
  		FootTrajectoryGenerator footTrajectoryGenerator;  // Foot Trajectory Generator control block
  		WBCController wbcController;                            // Whole body control Wrapper control block

  		// Filters
  		Filter filter_mpc_q = Filter();     // Filter object for base position
  		Filter filter_mpc_v = Filter();     // Filter object for base velocity
  		Filter filter_mpc_vref = Filter();  // Filter object for base reference velocity
  		Vector18 q_filt_mpc;                // Filtered base position vector (with joints)
  		Vector6 h_v_filt_mpc;               // Filtered base velocity vector (with joints)
  		Vector6 vref_filt_mpc;              // Filtered reference base velocity vector

  		// Various
  		Matrix34 o_targetFootstep;  // Target location of footsteps in ideal world
  		Vector18 q_wbc;             // Position vector for whole body control
  		Vector18 dq_wbc;            // Velocity vector for whole body control
  		Vector12 base_targets;      // Base position, orientation and velocity references for whole body control
  		Matrix3 hRb;                // Rotation matrix between horizontal and base frame
  		Vector6 p_ref_;             // Position, orientation reference vector from the joystick
  		Vector12 f_mpc;             // Contact forces desired by the MPC

		// commands from remote
		bool cmd_stop = false;			// stop or go
		Vector6 cmd_v_ref = Vector6::Zero();
		GaitType cmd_gait = GaitType::NoGait;
		bool cmd_is_moving = false; // true if any movement comes from the remote
};		


#endif  // CONTROLLER_H_INCLUDED
