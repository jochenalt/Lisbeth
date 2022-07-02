#include "StatePlanner.hpp"
#include "Gait.hpp"

StatePlanner::StatePlanner() :
		h_ref(0.0), RPY(Vector3::Zero()) {
}

void StatePlanner::initialize(Params &params_in, Gait &gait_in)
{
	params = &params_in;
	gait = &gait_in;
	h_ref = params->h_ref;
	n_steps = static_cast<int>(params->gait.rows());
	std::cout << "n_steps" << n_steps << std::endl;
	std::cout << "N_gait" << params->N_gait << std::endl;
	std::cout << "gait" << gait->getCurrentGait() << std::endl;

	referenceStates = MatrixN::Zero(12, 1 + n_steps);
	dt_vector = VectorN::LinSpaced(n_steps, params->dt_mpc,
			static_cast<double>(n_steps) * params->dt_mpc);
}

void StatePlanner::computeReferenceStates(Vector6 const &q, Vector6 const &v,	Vector6 const &vref)
{

	RPY = q.tail(3);

	// Update the current state
	referenceStates(0, 0) = 0.0;                			// In horizontal frame x = 0.0
	referenceStates(1, 0) = 0.0;                 		// In horizontal frame y = 0.0
	referenceStates(2, 0) = q(2, 0);                	// We keep height
	referenceStates.block(3, 0, 2, 1) = RPY.head(2); 	// We keep roll and pitch
	referenceStates(5, 0) = 0.0;               			// In horizontal frame yaw = 0.0
	referenceStates.block(6, 0, 3, 1) = v.head(3);
	referenceStates.block(9, 0, 3, 1) = v.tail(3);

   // Number of time steps in the prediction horizon
	for (int i = 0; i < n_steps; i++)
	{
		if (vref(5) != 0)
		{
			referenceStates(0, 1 + i) = (vref(0)
					* std::sin(vref(5) * dt_vector(i))
					+ vref(1) * (std::cos(vref(5) * dt_vector(i)) - 1.0)) / vref(5);
			referenceStates(1, 1 + i) = (vref(1)
					* std::sin(vref(5) * dt_vector(i))
					- vref(0) * (std::cos(vref(5) * dt_vector(i)) - 1.0)) / vref(5);
		}
		else
		{
			referenceStates(0, 1 + i) = vref(0) * dt_vector(i);
			referenceStates(1, 1 + i) = vref(1) * dt_vector(i);
		}
		referenceStates(0, 1 + i) += referenceStates(0, 0);
		referenceStates(1, 1 + i) += referenceStates(1, 0);

		referenceStates(2, 1 + i) = h_ref;

		referenceStates(5, 1 + i) = vref(5) * dt_vector(i);

		referenceStates(6, 1 + i) = vref(0)
				* std::cos(referenceStates(5, 1 + i))
				- vref(1) * std::sin(referenceStates(5, 1 + i));
		referenceStates(7, 1 + i) = vref(0)
				* std::sin(referenceStates(5, 1 + i))
				+ vref(1) * std::cos(referenceStates(5, 1 + i));

		referenceStates(11, 1 + i) = vref(5);
	}

}
