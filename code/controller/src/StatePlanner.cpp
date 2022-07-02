#include "StatePlanner.hpp"
#include "Gait.hpp"

StatePlanner::StatePlanner() {
}

void StatePlanner::initialize(Params &params_in, Gait &gait_in)
{
	params = &params_in;
	gait = &gait_in;

	// make matrix large enough, we only need N_steps + 1
	referenceStates = MatrixN::Zero(12, params->N_gait + 1);
}


void StatePlanner::computeReferenceStates(Vector6 const &q, Vector6 const &v,	Vector6 const &vref)
{

	Vector3 RPY = q.tail(3);

	// Update the current state
	referenceStates(0, 0) = 0.0;                			// In horizontal frame x = 0.0
	referenceStates(1, 0) = 0.0;                 		// In horizontal frame y = 0.0
	referenceStates(2, 0) = q(2, 0);                	// We keep height
	referenceStates.block(3, 0, 2, 1) = RPY.head(2); 	// We keep roll and pitch
	referenceStates(5, 0) = 0.0;               			// In horizontal frame yaw = 0.0
	referenceStates.block(6, 0, 3, 1) = v.head(3);
	referenceStates.block(9, 0, 3, 1) = v.tail(3);

   // Number of time steps in the prediction horizon
	int N_steps = params->get_N_steps();
	for (int i = 0; i < N_steps; i++)
	{
		double dt_cum = params->dt_mpc * (i+1);
		if (vref(5) != 0)
		{
			referenceStates(0, 1 + i) =
					+ (vref(0)* std::sin(vref(5) * dt_cum)
					+ vref(1) * (std::cos(vref(5) * dt_cum) - 1.0)) / vref(5);
			referenceStates(1, 1 + i) =
					+ (vref(1)* std::sin(vref(5) * dt_cum)
					- vref(0) * (std::cos(vref(5) * dt_cum) - 1.0)) / vref(5);
		}
		else
		{
			referenceStates(0, 1 + i) = vref(0) * dt_cum;
			referenceStates(1, 1 + i) = vref(1) * dt_cum;
		}
		referenceStates(0, 1 + i) += referenceStates(0, 0);
		referenceStates(1, 1 + i) += referenceStates(1, 0);
		referenceStates(2, 1 + i) = params->h_ref;
		referenceStates(5, 1 + i) = vref(5) * dt_cum;
		referenceStates(6, 1 + i) =
			  	+ vref(0) * std::cos(referenceStates(5, 1 + i))
				- vref(1) * std::sin(referenceStates(5, 1 + i));
		referenceStates(7, 1 + i) =
				+ vref(0) * std::sin(referenceStates(5, 1 + i))
				+ vref(1) * std::cos(referenceStates(5, 1 + i));

		referenceStates(11, 1 + i) = vref(5);
	}

}
