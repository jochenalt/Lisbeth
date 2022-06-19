#include "StatePlanner.hpp"

StatePlanner::StatePlanner()
    : dt_(0.0)
    , h_ref_(0.0)
    , n_steps_(0)
    , RPY_(Vector3::Zero())
{
}

void StatePlanner::initialize(Params& params)
{
	params_ = &params;
    dt_ = params.dt_mpc;
    h_ref_ = params.h_ref;
    n_steps_ = (int)std::lround(params.T_mpc / dt_);
    referenceStates_ = MatrixN::Zero(12, 1 + n_steps_);
    dt_vector_ = VectorN::LinSpaced(n_steps_, dt_, params.T_mpc);
}

void StatePlanner::computeReferenceStates(VectorN const& q, Vector6 const& v, Vector6 const& vref, double z_average)
{
    RPY_ = Vector3({q(3), q(4), q(5)});

    // Update the current state
    referenceStates_(0, 0) = 0.0;  // In horizontal frame x = 0.0
    referenceStates_(1, 0) = 0.0;  // In horizontal frame y = 0.0
    referenceStates_(2, 0) = q(2, 0);  // We keep height
    referenceStates_.block(3, 0, 2, 1) = RPY_.head(2);  // We keep roll and pitch
    referenceStates_(5, 0) = 0.0;  // In horizontal frame yaw = 0.0
    referenceStates_.block(6, 0, 3, 1) = v.head(3);
    referenceStates_.block(9, 0, 3, 1) = v.tail(3);

     for (int i = 0; i < n_steps_; i++)
    {
        if (vref(5) != 0)
        {
            referenceStates_(0, 1 + i) = (vref(0) * std::sin(vref(5) * dt_vector_(i)) + vref(1) * (std::cos(vref(5) * dt_vector_(i)) - 1.0)) / vref(5);
            referenceStates_(1, 1 + i) = (vref(1) * std::sin(vref(5) * dt_vector_(i)) - vref(0) * (std::cos(vref(5) * dt_vector_(i)) - 1.0)) / vref(5);
        }
        else
        {
            referenceStates_(0, 1 + i) = vref(0) * dt_vector_(i);
            referenceStates_(1, 1 + i) = vref(1) * dt_vector_(i);
        }
        referenceStates_(0, 1 + i) += referenceStates_(0, 0);
        referenceStates_(1, 1 + i) += referenceStates_(1, 0);

        referenceStates_(2, 1 + i) = h_ref_ + z_average;

        referenceStates_(5, 1 + i) = vref(5) * dt_vector_(i);

        referenceStates_(6, 1 + i) = vref(0) * std::cos(referenceStates_(5, 1 + i)) - vref(1) * std::sin(referenceStates_(5, 1 + i));
        referenceStates_(7, 1 + i) = vref(0) * std::sin(referenceStates_(5, 1 + i)) + vref(1) * std::cos(referenceStates_(5, 1 + i));

        referenceStates_(11, 1 + i) = vref(5);
    }
}
