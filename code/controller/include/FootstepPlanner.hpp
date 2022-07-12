/**
 * Planner that outputs current and future locations of footsteps, the reference
 * trajectory of the base based on the reference velocity given by the user and the current
 * position/velocity of the base
 */

#ifndef FOOTSTEPPLANNER_H_
#define FOOTSTEPPLANNER_H_

#include <vector>
#include "GaitPlanner.hpp"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "Types.h"
#include "Params.hpp"

// Order of feet/legs: FL, FR, HL, HR

class FootstepPlanner {
 public:
  FootstepPlanner();
  ~FootstepPlanner() {}

  //	params Object that stores parameters
  //	gaitIn Gait object to hold the gait informations
  void initialize(Params& params, GaitPlanner& gaitIn);


  // Refresh footsteps locations (computation and update of relevant matrices)
  //		refresh True if we move one step further in the gait
  //		q Current position vector of the flying base in horizontal frame (linear and angular stacked) + actuators
  //		b_v Current velocity vector of the flying base in horizontal frame (linear and angular stacked)
  //		b_vref Desired velocity vector of the flying base in horizontal frame (linear and angular stacked)
  Matrix34 updateFootsteps(bool refresh, Vector18 const& q, Vector6 const& b_v, Vector6 const& b_vref);

  MatrixN12 getFootsteps();
  Matrix34 getTargetFootsteps();

 private:
  // Compute the desired location of footsteps and update relevant matrices
  //		k Number of remaining wbc time step for the current mpc time step (wbc frequency is higher so there are inter-steps)
  //		q Current position vector of the flying base in horizontal frame (linear and angular stacked)
  //		b_v Current velocity vector of the flying base in horizontal frame (linear and angular stacked)
  //		b_vref Desired velocity vector of the flying base in horizontal frame (linear and angular stacked)
  Matrix34 computeTargetFootstep(int k, Vector6 const& q, Vector6 const& b_v, Vector6 const& b_vref);

  // Refresh feet position when entering a new contact phase
  //		q Current configuration vector
  void updateNewContact(Vector18 const& q);

  // Compute a X by 13 matrix containing the remaining number of steps of each phase of the gait (first column)
  // and the [x, y, z]^T desired position of each foot for each phase of the gait (12 other columns).
  // For feet currently touching the ground the desired position is where they currently are.
  //		k Number of remaining wbc time step for the current mpc time step (wbc frequency is higher so there are inter-steps)
  //		b_v Current velocity vector of sthe flying base in horizontal frame (linear and angular stacked)
  //		b_vref Desired velocity vector of the flying base in horizontal frame (linear and angular stacked)
  void computeFootsteps(int k, Vector6 const& b_v, Vector6 const& b_vref);

  // Compute the target location on the ground of a given foot for an upcoming stance phase
  //		i Considered phase (row of the gait matrix)
  //		j Considered foot (col of the gait matrix)
  //		b_v Current velocity vector of sthe flying base in horizontal frame (linear and angular stacked)
  //		b_vref Desired velocity vector of the flying base in horizontal frame (linear and angular stacked)
  //		retval Matrix with the next footstep positions
  void computeNextFootstep(int i, int j, Vector6 const& b_v, Vector6 const& b_vref);

  // Update desired location of footsteps using information coming from the footsteps planner
  void updateTargetFootsteps();

  // Transform a std::vector of N 3x4 matrices into a single Nx12 matrix
  //		array The std::vector of N 3x4 matrices to transform
  MatrixN12 vectorToMatrix(std::vector<Matrix34> const& array);

  Params* params;  							// Params object to store parameters
  GaitPlanner* gait;      							// Gait object to hold the gait informations

  double h_ref;   							// Reference height for the trunk

  // Predefined quantities
  double g;  									// Value of the gravity acceleartion
  double L;  									// Value of the maximum allowed deviation due to leg length

  // Constant sized matrices
  Matrix34 footsteps_under_shoulders;  // Positions of footsteps to be "under the shoulder"
  Matrix34 footsteps_offset;          	// Hardcoded offset to add to footsteps positions
  Matrix34 currentFootstep;            // Feet matrix
  Matrix34 nextFootstep;               // Temporary matrix to perform computations
  Matrix34 targetFootstep;             // In horizontal frame
  Matrix34 o_targetFootstep;           // targetFootstep_ in world frame
  std::vector<Matrix34> footsteps;     // Desired footsteps locations for each step of the horizon

  Matrix3 Rz;      							// Rotation matrix along z axis
  VectorN dt_cum;  							// Cumulated time vector
  VectorN yaws;    							// Predicted yaw variation for each cumulated time
  VectorN dx;      							// Predicted x displacement for each cumulated time
  VectorN dy;      							// Predicted y displacement for each cumulated time

  Vector3 q_dxdy;  							// Temporary storage variable for offset to the future position
  Vector3 RPY;    							// Temporary storage variable for roll pitch yaw orientation

  pinocchio::Model model;          		// Pinocchio model for forward kinematics
  pinocchio::Data data;            		// Pinocchio datas for forward kinematics
  int foot_ids_[4] = {0, 0, 0, 0};  	// Indexes of feet frames
  Matrix34 pos_feet;               		// Estimated feet positions based on measurements
  Vector19 q_FK;                   		// Estimated state of the base (height, roll, pitch, joints)
};

#endif  // FOOTSTEPPLANNER_H_
