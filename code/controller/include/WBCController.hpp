/**
 *
 * WbcWrapper provides an interface for the user to solve the whole body control problem
 * Internally it calls first the InvKin class to solve an inverse kinematics problem then calls the QPWBC
 * class to solve a box QP problem based on result from the inverse kinematic and desired ground forces
 * */

#ifndef WBC_WRAPPER_H_INCLUDED
#define WBC_WRAPPER_H_INCLUDED

#include "InvKin.hpp"
#include "Params.hpp"
#include "WBCSolver.hpp"

class WBCController {
	public:
	  WBCController();
	  ~WBCController() {}

	  void initialize(Params &params);

	  // Run and solve one iteration of the whole body control (matrix update, invkin, QP)
	  //		q Estimated positions of the 12 actuators
	  //		dq Estimated velocities of the 12 actuators
	  //		f_cmd Reference contact forces received from the MPC
	  //		contacts Contact status of the four feet
	  //		pgoals Desired positions of the four feet in base frame
	  //		vgoals Desired velocities of the four feet in base frame
	  //		agoals Desired accelerations of the four feet in base frame
	  //		xgoals Desired position, orientation and velocities of the base
	  void compute(Vector18 const& q, Vector18 const& dq,
						Vector12 const& f_cmd,
						RowVector4 const& contacts,
						Matrix34 const& pgoals, Matrix34 const& vgoals, Matrix34 const& agoals,
						Vector12 const &xgoals);
	  Vector7 get_bdes() { return bdes; }
	  Vector12 get_qdes() { return qdes; }
	  Vector12 get_vdes() { return vdes; }
	  Vector12 get_tau_ff() { return tau_ff; }
	  Vector18 get_ddq_cmd() { return ddq_cmd; }
	  Vector12 get_f_with_delta() { return f_with_delta; }
	  Vector18 get_ddq_with_delta() { return ddq_with_delta; }
	  Matrix34 get_feet_pos() { return invkin.get_posf().transpose(); }
	  Matrix34 get_feet_vel() { return invkin.get_vf().transpose(); }

	private:
  	  Params *params;  							// Parameter object
  	  WBCSolver box_qp;   						// QP problem solver for the whole body control
  	  InvKin invkin;  							// Inverse Kinematics solver for the whole body control

  	  pinocchio::Model model;  				// Pinocchio model for frame computations
  	  pinocchio::Data data;    				// Pinocchio data for frame computations

  	  Matrix18 M;  								// Mass matrix
  	  Eigen::Matrix<double, 12, 6> Jc;  	// Jacobian matrix
  	  RowVector4 k_since_contact;       	// Number of time step during which feet have been in the current stance phase
  	  Vector7 bdes;                     	// Desired base positions
  	  Vector12 qdes;                    	// Desired actuator positions
  	  Vector12 vdes;                    	// Desired actuator velocities
  	  Vector12 tau_ff;                  	// Desired actuator torques (feedforward)

  	  Vector19 q_wbc;           				// Configuration vector for the whole body control
  	  Vector18 dq_wbc;          				// Velocity vector for the whole body control
  	  Vector18 ddq_cmd;         				// Actuator accelerations computed by Inverse Kinematics
  	  Vector12 f_with_delta;    				// Contact forces with deltas found by QP solver
  	  Vector18 ddq_with_delta;  				// Actuator accelerations with deltas found by QP solver
    
  	  bool enable_comp_forces;            // Enable compensation forces for the QP problem
};

#endif  // WBC_WRAPPER_H_INCLUDED
