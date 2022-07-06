/**
 * This class generates the input for the MPC solver, which is the to-be state of all feet
 * for the full prediction horizon
 *
 * Compute the reference trajectory of the CoM for each time step of the
 * prediction horizon. The output is a matrix of size 12 by (N+1) with N the number
 * of time steps in the gait cycle (N_steps) and 12 the position, orientation,
 * linear velocity and angular velocity vertically stacked. The first column contains
 * the current state while the remaining N columns contains the desired future states.
 *
 */

#ifndef STATEPLANNER_H_INCLUDED
#define STATEPLANNER_H_INCLUDED

#include "Types.h"
#include "Params.hpp"
#include "Gait.hpp"

class StatePlanner
{
public:
    StatePlanner();
    ~StatePlanner() {}

    void initialize(Params& params, Gait& gait);

    /* compute the reference trajectory
     * 		q current position vector of the flying base in horizontal frame (linear and angular stacked)
     *		v current velocity vector of the flying base in horizontal frame (linear and angular stacked)
     *		vref desired velocity vector of the flying base in horizontal frame (linear and angular stacked)
     */
    void computeReferenceStates(Vector6 const& q, Vector6 const& v, Vector6 const& vref);


    Matrix12N getReferenceStates() { return referenceStates; }

private:
    Params* params;
    Gait*  gait;

    // Reference trajectory matrix of size 12 by (1 + N)  with the current state of
    // the robot in column 0 and the N steps of the prediction horizon in the others
    Matrix12N referenceStates;
};

#endif  // STATEPLANNER_H_INCLUDED
