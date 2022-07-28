/*
 * Class for generating reference trajectory for the swing foot in terms of position, velocity and acceleration
 *
 */

#ifndef FOOT_TRAJECTORY_GENERATOR_H
#define FOOT_TRAJECTORY_GENERATOR_H

#include "GaitPlanner.hpp"
#include "Params.hpp"

class FootTrajectoryGenerator {
 public:
  FootTrajectoryGenerator();
  ~FootTrajectoryGenerator() {}

  void initialize(Params &params, GaitPlanner &gait);

  // Update the next foot position, velocity and acceleration, and the foot goal position
  // j Foot id
  // targetFootstep Desired target location at the end of the swing phase
  void updateFootPosition(int const j, Vector3 const &targetFootstep);

  // Update the 3D desired position for feet in swing phase by using a 5-th order polynomial that lead them
  // to the desired position on the ground (computed by the footstep planner)
  //	k Numero of the current loop
  //  targetFootstep Desired target locations at the end of the swing phases
  void update(bool startNewCycle, MatrixN const &targetFootstep);

  MatrixN getFootPosition() { return position; }          // Get the next foot position
  MatrixN getFootVelocity() { return velocity; }          // Get the next foot velocity
  MatrixN getFootAcceleration() { return acceleration; }  // Get the next foot acceleration

 private:
  Params* params;
  GaitPlanner *gait;

  Eigen::Matrix<int, 1, 4> feet;  // Column indexes of feet currently in swing phase
  Vector4 t0s;                    // Elapsed time since the start of the swing phase movement
  Vector4 t_swing;                // Swing phase duration for each foot

  Matrix34 targetFootstep;  		 // Target for the X component

  Matrix64 Ax;  						 // Coefficients for the X component
  Matrix64 Ay;  						 // Coefficients for the Y component

  Matrix34 position;      // Position computed in updateFootPosition
  Matrix34 velocity;      // Velocity computed in updateFootPosition
  Matrix34 acceleration;  // Acceleration computed in updateFootPosition
};
#endif  // TRAJGEN_H_INCLUDED
