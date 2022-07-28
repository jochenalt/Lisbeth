#include "WBCController.hpp"

#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/math/rpy.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/rnea.hpp"

WBCController::WBCController() :
		M(Matrix18::Zero()), Jc(Eigen::Matrix<double, 12, 6>::Zero()),
		k_since_contact(RowVector4::Zero()), bdes(Vector7::Zero()),
		qdes(Vector12::Zero()), vdes(Vector12::Zero()),
		tau_ff(Vector12::Zero()), q_wbc(Vector19::Zero()),
		dq_wbc(Vector18::Zero()), ddq_cmd(Vector18::Zero()),
		f_with_delta(Vector12::Zero()), ddq_with_delta(Vector18::Zero()),
		enable_comp_forces(false)
{
}

void WBCController::initialize(Params &params_in)
{
	// Params store parameters
	params = &params_in;

	// Set if compensation forces should be used or not
	enable_comp_forces = params_in.enable_comp_forces;

	// Path to the robot URDF (TODO: Automatic path)
	const std::string filename = std::string(URDF_MODEL);

	// Build model from urdf (base is not free flyer)
	pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(),
			model, false);

	// Construct data from model
	data = pinocchio::Data(model);

	// Update all the quantities of the model
	pinocchio::computeAllTerms(model, data, VectorN::Zero(model.nq),
			VectorN::Zero(model.nv));

	// Initialize inverse kinematic and box QP solvers
	invkin.initialize(params_in);
	box_qp.initialize(params_in);

	// Initialize quaternion
	q_wbc(6, 0) = 1.0;

	// Initialize joint positions
	qdes.tail(12) = Vector12(params->q_init.data());

	// Compute the upper triangular part of the joint space inertia matrix M by using the Composite Rigid Body Algorithm
	// Result is stored in data_.M
	pinocchio::crba(model, data, q_wbc);

	// Make mass matrix symetric
	data.M.triangularView<Eigen::StrictlyLower>() =
			data.M.transpose().triangularView<Eigen::StrictlyLower>();

}

/** Call Inverse Kinematics to get an acceleration command then
 solve a QP problem to get the feedforward torques

 Args:
 q (19x1): Current state of the base
 dq (18x1): Current velocity of the base (in base frame)
 f_cmd (1x12): Contact forces references from the mpc
 contacts (1x4): Contact status of feet
 pgoals, vgoals, agoals Objects that contains the pos, vel and acc references for feet
 */
void WBCController::compute(Vector18 const &q, Vector18 const &dq,
		Vector12 const &f_cmd, RowVector4 const &contacts, Matrix34 const &pgoals,
		Matrix34 const &vgoals, Matrix34 const &agoals, Vector12 const &xgoals)
{
	//  Update nb of iterations since contact
	k_since_contact += contacts;  // Increment feet in stance phase
	k_since_contact = k_since_contact.cwiseProduct(contacts); // Reset feet in swing phase

	// Retrieve configuration data
	q_wbc.head(3) = q.head(3);
	q_wbc.block(3, 0, 4, 1) = pinocchio::SE3::Quaternion(
			pinocchio::rpy::rpyToMatrix(q(3, 0), q(4, 0), q(5, 0))).coeffs(); // Roll, Pitch
	q_wbc.tail(12) = q.tail(12);                                     // Encoders

	// Retrieve velocity data
	dq_wbc = dq;

	// Compute the upper triangular part of the joint space inertia matrix M by using the Composite Rigid Body Algorithm
	// Result is stored in data_.M
	pinocchio::crba(model, data, q_wbc);

	// Make mass matrix symetric
	data.M.triangularView<Eigen::StrictlyLower>() =
			data.M.transpose().triangularView<Eigen::StrictlyLower>();

	// Compute Inverse Kinematics
	invkin.run(q_wbc.tail(12), dq_wbc.tail(12), contacts, pgoals, vgoals,
			agoals);
	ddq_cmd.tail(12) = invkin.get_ddq_cmd();

	// Compute the upper triangular part of the joint space inertia matrix M by using the Composite Rigid Body Algorithm
	// Result is stored in data_.M
	pinocchio::crba(model, data, q_wbc);


	// Retrieve feet jacobian
	Matrix43 posf_tmp_ = invkin.get_posf();
	for (int i = 0; i < 4; i++)
	{
		if (contacts(0, i))
		{
			Jc.block(3 * i, 0, 3, 3) = Matrix3::Identity();
			Jc.block(3 * i, 3, 3, 3) << 0.0, posf_tmp_(i, 2), -posf_tmp_(i, 1), -posf_tmp_(
					i, 2), 0.0, posf_tmp_(i, 0), posf_tmp_(i, 1), -posf_tmp_(i, 0), 0.0;
		}
		else
			Jc.block(3 * i, 0, 3, 6).setZero();
	}

	// Compute the inverse dynamics, aka the joint torques according to the current state of the system,
	// the desired joint accelerations and the external forces, using the Recursive Newton Euler Algorithm.
	// Result is stored in data.tau
	Vector12 f_compensation;
	if (!enable_comp_forces)
	{
		pinocchio::rnea(model, data, q_wbc, dq_wbc, ddq_cmd);
		f_compensation = Vector12::Zero();
	}
	else
	{
		Vector18 ddq_test = Vector18::Zero();
		ddq_test.head(6) = ddq_cmd.head(6);
		pinocchio::rnea(model, data, q_wbc, dq_wbc, ddq_test);
		Vector6 RNEA_without_joints = data.tau.head(6);
		pinocchio::rnea(model, data, q_wbc, dq_wbc, VectorN::Zero(model.nv));
		Vector6 RNEA_NLE = data.tau.head(6);
		RNEA_NLE(2, 0) -= 9.81 * data.mass[0];
		pinocchio::rnea(model, data, q_wbc, dq_wbc, ddq_cmd);

		f_compensation = pseudoInverse(Jc.transpose())
				* (data.tau.head(6) - RNEA_without_joints + RNEA_NLE);
	}

	// Solve the QP problem
	box_qp.run(data.M, Jc, f_cmd + f_compensation, data.tau.head(6),
			k_since_contact);

	// Add to reference quantities the deltas found by the QP solver
	f_with_delta = box_qp.get_f_res();
	ddq_with_delta.head(6) = ddq_cmd.head(6) + box_qp.get_ddq_res();
	ddq_with_delta.tail(12) = ddq_cmd.tail(12);

	// Compute joint torques from contact forces and desired accelerations
	pinocchio::rnea(model, data, q_wbc, dq_wbc, ddq_with_delta);

	tau_ff = data.tau.tail(12) - invkin.get_Jf().transpose() * f_with_delta;

	// Retrieve desired positions and velocities
	vdes = invkin.get_dq_cmd();
	qdes = invkin.get_q_cmd();
	bdes = invkin.get_q_cmd().head(7);
}
