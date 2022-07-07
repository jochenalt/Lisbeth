#include "FootstepPlanner.hpp"

#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/math/rpy.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include "Utils.hpp"

FootstepPlanner::FootstepPlanner() :
		gait(NULL), g(9.81), L(0.155), nextFootstep(Matrix34::Zero()), footsteps(),
		Rz(Matrix3::Zero(3, 3)), dt_cum(), yaws(), dx(), dy(), q_dxdy(
		Vector3::Zero()), RPY(Vector3::Zero()), pos_feet(Matrix34::Zero()), q_FK(
		Vector19::Zero())
{
}

void FootstepPlanner::initialize(Params &params_in, GaitPlanner &gait_in)
{
	params = &params_in;
	h_ref = params->h_ref;
	footsteps_under_shoulders
			<< Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
					params->footsteps_under_shoulders.data(),
					params->footsteps_under_shoulders.size());
	// Offsets to make the support polygon smaller
	double ox = 0.0;
	double oy = 0.0;
	footsteps_offset << -ox, -ox, ox, ox, -oy, +oy, +oy, -oy, 0.0, 0.0, 0.0, 0.0;

	// initial foot steps
	currentFootstep
			<< Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
					params->footsteps_init.data(),
					params->footsteps_init.size());
	gait = &gait_in;
	targetFootstep = currentFootstep;
	o_targetFootstep = currentFootstep;
	dt_cum = VectorN::Zero(params->get_N_steps());
	yaws = VectorN::Zero(params->get_N_steps());
	dx = VectorN::Zero(params->get_N_steps());
	dy = VectorN::Zero(params->get_N_steps());
	for (int i = 0; i < params->get_N_steps(); i++)
	{
		footsteps.push_back(Matrix34::Zero());
	}
	Rz(2, 2) = 1.0;

	// Path to the robot URDF
	const std::string filename = std::string(URDF_MODEL);

	// Build model from urdf (base is not free flyer)
	pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(),
			model, false);

	// Construct data from model
	data = pinocchio::Data(model);

	// Update all the quantities of the model
	VectorN q_tmp = VectorN::Zero(model.nq);
	q_tmp(6, 0) = 1.0;  // Quaternion (0, 0, 0, 1)
	pinocchio::computeAllTerms(model, data, q_tmp, VectorN::Zero(model.nv));

	// Get feet frame IDs
	foot_ids_[0] = static_cast<int>(model.getFrameId("FL_FOOT")); // from long uint to int
	foot_ids_[1] = static_cast<int>(model.getFrameId("FR_FOOT"));
	foot_ids_[2] = static_cast<int>(model.getFrameId("HL_FOOT"));
	foot_ids_[3] = static_cast<int>(model.getFrameId("HR_FOOT"));
}

Matrix34 FootstepPlanner::updateFootsteps(bool refresh, int k, Vector18 const &q,
		Vector6 const &b_v, Vector6 const &b_vref)
{
	// Update location of feet in stance phase (for those which just entered stance phase)
	if (refresh && gait->isNewPhase())
	{
		updateNewContact(q);
	}

	// Feet in contact with the ground are moving in base frame (they don't move in world frame)
	double rotation_yaw = params->dt_wbc * b_vref(5); // Rotation along Z for the last time step
	double c = std::cos(rotation_yaw);
	double s = std::sin(rotation_yaw);
	Rz.topLeftCorner<2, 2>() << c, s, -s, c;
	Vector2 dpos = params->dt_wbc * b_vref.head(2); // Displacement along X and Y for the last time step
	for (int j = 0; j < 4; j++)
	{
		if (gait->getCurrentGait(0, j) == 1.0)
		{
			currentFootstep.block(0, j, 2, 1) = Rz
					* (currentFootstep.block(0, j, 2, 1) - dpos);
		}
	}

	// Compute location of footsteps
	Matrix34 result = computeTargetFootstep(k, q.head(6), b_v, b_vref);
	return result;
}

void FootstepPlanner::computeFootsteps(int steps_left_in_gait, Vector6 const &b_v, Vector6 const &b_vref)
{
	for (uint i = 0; i < footsteps.size(); i++)
	{
		footsteps[i] = Matrix34::Zero();
	}
	MatrixN current_gait = gait->getCurrentGait();

	// Set current position of feet for feet in stance phase
	for (int j = 0; j < 4; j++)
	{
		if (current_gait(0, j) == 1.0)
		{
			footsteps[0].col(j) = currentFootstep.col(j);
		}
	}

	// Cumulative time by adding the terms in the first column (remaining number of timesteps)
	// Get future yaw yaws compared to current position
	dt_cum(0) = params->dt_wbc * steps_left_in_gait;
	yaws(0) = b_vref(5) * dt_cum(0);
	for (uint j = 1; j < footsteps.size(); j++)
	{
		dt_cum(j) =
				current_gait.row(j).isZero() ? dt_cum(j - 1) : dt_cum(j - 1) + params->dt_mpc;
		yaws(j) = b_vref(5) * dt_cum(j);
	}

	// Displacement following the reference velocity compared to current position
	if (b_vref(5, 0) != 0)
	{
		for (uint j = 0; j < footsteps.size(); j++)
		{
			dx(j) = (b_vref(0) * std::sin(b_vref(5) * dt_cum(j))
					+ b_vref(1) * (std::cos(b_vref(5) * dt_cum(j)) - 1.0))
					/ b_vref(5);
			dy(j) = (b_vref(1) * std::sin(b_vref(5) * dt_cum(j))
					- b_vref(0) * (std::cos(b_vref(5) * dt_cum(j)) - 1.0))
					/ b_vref(5);
		}
	}
	else
	{
		for (uint j = 0; j < footsteps.size(); j++)
		{
			dx(j) = b_vref(0) * dt_cum(j);
			dy(j) = b_vref(1) * dt_cum(j);
		}
	}

	// Update the footstep matrix depending on the different phases of the gait (swing & stance)
	for (int i = 1; i < current_gait.rows(); i++)
	{
		// Feet that were in stance phase and are still in stance phase do not move
		for (int j = 0; j < 4; j++)
		{
			if (current_gait(i - 1, j) > 0 && current_gait(i, j) > 0)
			{
				footsteps[i].col(j) = footsteps[i - 1].col(j);
			}
		}

		// Feet that were in swing phase and are now in stance phase need to be updated
		for (int j = 0; j < 4; j++)
		{
			if (current_gait(i - 1, j) == 0 && current_gait(i, j) > 0)
			{
				// Offset to the future position
				q_dxdy << dx(i - 1, 0), dy(i - 1, 0), 0.0;

				// Get future desired position of footsteps
				computeNextFootstep(i, j, b_v, b_vref);

				// Get desired position of footstep compared to current position
				double c = std::cos(yaws(i - 1));
				double s = std::sin(yaws(i - 1));
				Rz.topLeftCorner<2, 2>() << c, -s, s, c;

				footsteps[i].col(j) =
						(Rz * nextFootstep.col(j) + q_dxdy).transpose();
			}
		}
	}
}

void FootstepPlanner::computeNextFootstep(int i, int j, Vector6 const &b_v, Vector6 const &b_vref)
{
	nextFootstep = Matrix34::Zero();
	double t_stance = gait->getPhaseDuration(i, j);

	// Disable heuristic terms if gait is going to switch to static so that feet land at vertical of shoulders
	if (!gait->getIsStatic())
	{
		// Add symmetry term
		nextFootstep.col(j) = t_stance * 0.5 * b_v.head(3);

		// Add feedback term
		nextFootstep.col(j) += params->k_feedback
				* (b_v.head(3) - b_vref.head(3));

		// Add centrifugal term
		Vector3 cross;
		cross << b_v(1) * b_vref(5) - b_v(2) * b_vref(4), b_v(2) * b_vref(3)
				- b_v(0) * b_vref(5), 0.0;
		nextFootstep.col(j) += 0.5 * std::sqrt(h_ref / g) * cross;
	}

	// Legs have a limited length so the deviation has to be limited
	nextFootstep(0, j) = std::min(nextFootstep(0, j), L);
	nextFootstep(0, j) = std::max(nextFootstep(0, j), -L);
	nextFootstep(1, j) = std::min(nextFootstep(1, j), L);
	nextFootstep(1, j) = std::max(nextFootstep(1, j), -L);

	// Add shoulders
	nextFootstep.col(j) += footsteps_under_shoulders.col(j);
	nextFootstep.col(j) += footsteps_offset.col(j);

	// Remove Z component (working on flat ground)
	nextFootstep.row(2) = Vector4::Zero().transpose();
}

void FootstepPlanner::updateTargetFootsteps()
{
	for (int i = 0; i < 4; i++)
	{
		int index = 0;
		while (footsteps[index](0, i) == 0.0)
		{
			index++;
		}
		targetFootstep.col(i) << footsteps[index](0, i), footsteps[index](1, i), 0.0;
	}
}

Matrix34 FootstepPlanner::computeTargetFootstep(int steps_left_in_gait,
		Vector6 const &q, Vector6 const &b_v, Vector6 const &b_vref)
{
	// Compute the desired location of footsteps over the prediction horizon
	computeFootsteps(steps_left_in_gait, b_v, b_vref);

	// Update desired location of footsteps on the ground
	updateTargetFootsteps();

	// Get o_targetFootstep_ in world frame from targetFootstep_ in horizontal frame
	RPY = q.tail(3);
	double c = std::cos(RPY(2));
	double s = std::sin(RPY(2));
	Rz.topLeftCorner<2, 2>() << c, -s, s, c;
	for (int i = 0; i < 4; i++)
	{
		o_targetFootstep.block(0, i, 2, 1) = Rz.topLeftCorner<2, 2>()
				* targetFootstep.block(0, i, 2, 1) + q.head(2);
	}

	return o_targetFootstep;
}

void FootstepPlanner::updateNewContact(Vector18 const &q)
{
	// Remove translation and yaw rotation to get position in local frame
	q_FK.head(2) = Vector2::Zero();
	q_FK(2, 0) = q(2, 0);
	q_FK.block(3, 0, 4, 1) = pinocchio::SE3::Quaternion(
			pinocchio::rpy::rpyToMatrix(q(3, 0), q(4, 0), 0.0)).coeffs();
	q_FK.tail(12) = q.tail(12);

	// Update model and data of the robot
	pinocchio::forwardKinematics(model, data, q_FK);
	pinocchio::updateFramePlacements(model, data);

	// Get data required by IK with Pinocchio
	for (int i = 0; i < 4; i++)
	{
		pos_feet.col(i) = data.oMf[foot_ids_[i]].translation();
	}

	// Refresh position with estimated position if foot is in stance phase
	for (int i = 0; i < 4; i++)
	{
		if (gait->getCurrentGait(0, i) == 1.0)
		{
			currentFootstep.block(0, i, 2, 1) = pos_feet.block(0, i, 2, 1); // Get only x and y to let z = 0 for contacts
		}
	}
}

MatrixN12 FootstepPlanner::getFootsteps()
{
	return vectorToMatrix(footsteps);
}
Matrix34 FootstepPlanner::getTargetFootsteps()
{
	return targetFootstep;
}

// convert a vectorN of a matrix34 to a matrixN12
// such that one row is the concatenation of all rows of the Matrix34
MatrixN12 FootstepPlanner::vectorToMatrix(std::vector<Matrix34> const &array)
{
	MatrixN12 M = MatrixN12::Zero(array.size(), 12);
	for (uint i = 0; i < array.size(); i++)
	{
		for (int j = 0; j < 4; j++)
		{
			M.row(i).segment<3>(3 * j) = array[i].col(j);
		}
	}
	return M;
}
