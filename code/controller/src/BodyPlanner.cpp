#include "../include/BodyPlanner.hpp"

#include "Gait.hpp"

using namespace std;

BodyPlanner::BodyPlanner() {
}

void BodyPlanner::setup(Params &params_in, Gait &gait_in)
{
	params = &params_in;
	gait = &gait_in;

	// make matrix large enough, we only need N_steps + 1
	referenceStates = MatrixN::Zero(12, params->N_gait + 1);
}


void BodyPlanner::update(Vector6 const &q, Vector6 const &v,	Vector6 const &vref)
{
	Vector3 RPY = q.tail(3);

	// Update the current state
	referenceStates(0, 0) = 0.0;                			// In horizontal frame x = 0.0
	referenceStates(1, 0) = 0.0;                 		// In horizontal frame y = 0.0
	referenceStates(2, 0) = q(2, 0);                	// We keep same height
	referenceStates.block(3, 0, 2, 1) = RPY.head(2); 	// We keep roll and pitch
	referenceStates(5, 0) = 0.0;               			// In horizontal frame yaw = 0.0
	referenceStates.block(6, 0, 3, 1) = v.head(3);
	referenceStates.block(9, 0, 3, 1) = v.tail(3);

   // Number of time steps in the prediction horizon
	int N_steps = params->get_N_steps();
	for (int i = 0; i < N_steps; i++)
	{
		// passed time of the step i since now
		double dt_cum = params->dt_mpc * (i+1);

		// if yaw (=vref(5)) is set, compute x,y by adding incremental rotateion
		if (vref(5) != 0) {
			referenceStates(0, 1 + i) = + (vref(0) *  sin(vref(5) * dt_cum) +  vref(1) * (cos(vref(5) * dt_cum) - 1.0)) / vref(5);
			referenceStates(1, 1 + i) = + (vref(1) *  sin(vref(5) * dt_cum) -  vref(0) * (cos(vref(5) * dt_cum) - 1.0)) / vref(5);
		}
		else {
			referenceStates(0, 1 + i) = vref(0) * dt_cum;
			referenceStates(1, 1 + i) = vref(1) * dt_cum;
		}

		referenceStates(0, 1 + i) += referenceStates(0, 0);		// propagate x
		referenceStates(1, 1 + i) += referenceStates(1, 0);		// propagate y
		referenceStates(2, 1 + i) = referenceStates(2, 0);			// same heigh
		referenceStates(3, 1 + i) = referenceStates(3, 0);			// same roll
		referenceStates(4, 1 + i) = referenceStates(4, 0);			// same pitch
		referenceStates(5, 1 + i) = vref(5) * dt_cum;				// propagate yaw
		referenceStates(6, 1 + i) = + vref(0) * cos(referenceStates(5, 1 + i))  // velocity in x
											 - vref(1) * sin(referenceStates(5, 1 + i));
		referenceStates(7, 1 + i) = + vref(0) * sin(referenceStates(5, 1 + i))	// velocity in y
											 + vref(1) * cos(referenceStates(5, 1 + i));
																										// velocity in z = 0
																										// angular velocity in x,y remains the same
		referenceStates(11, 1 + i) = vref(5);												// angular velocity in z is yaw rate
	}

}
