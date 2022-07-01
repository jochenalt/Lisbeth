/**
 * Planner that outputs the reference trajectory of the base based on the reference
 * velocity given by the user and the current position/velocity of the base
 */

#ifndef STATEPLANNER_H_INCLUDED
#define STATEPLANNER_H_INCLUDED

#include "Types.h"
#include "Params.hpp"


class StatePlanner
{
public:
    StatePlanner();
    ~StatePlanner() {}

    void initialize(Params& params);

    // Compute the reference trajectory of the CoM for each time step of the
    // prediction horizon. The output is a matrix of size 12 by (N+1) with N the number
    // of time steps in the gait cycle (T_gait/dt) and 12 the position, orientation,
    // linear velocity and angular velocity vertically stacked. The first column contains
    // the current state while the remaining N columns contains the desired future states.
    //		q current position vector of the flying base in horizontal frame (linear and angular stacked)
    //		v current velocity vector of the flying base in horizontal frame (linear and angular stacked)
    //		vref desired velocity vector of the flying base in horizontal frame (linear and angular stacked)
    //		z_average average height of feet currently in stance phase
    void computeReferenceStates(Vector6 const& q, Vector6 const& v, Vector6 const& vref);

    Matrix12N getReferenceStates() { return referenceStates; }
    int getNSteps() { return n_steps; }

private:
    Params* params;
    double h_ref;       // Reference height for the trunk
    int n_steps;        // Number of time steps in the prediction horizon

    Vector3 RPY;        // To store roll, pitch and yaw angles 

    // Reference trajectory matrix of size 12 by (1 + N)  with the current state of
    // the robot in column 0 and the N steps of the prediction horizon in the others
    Matrix12N referenceStates;

    VectorN dt_vector;  // Vector containing all time steps in the prediction horizon

};

#endif  // STATEPLANNER_H_INCLUDED
