#include "../include/WbcController.hpp"

#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/math/rpy.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/rnea.hpp"

WbcController::WbcController()
    : M_(Eigen::Matrix<double, 18, 18>::Zero())
    , Jc_(Eigen::Matrix<double, 12, 6>::Zero())
    , k_since_contact_(Eigen::Matrix<double, 1, 4>::Zero())
	, bdes_(Vector7::Zero())
    , qdes_(Vector12::Zero())
    , vdes_(Vector12::Zero())
    , tau_ff_(Vector12::Zero())
	, q_wbc_(Vector19::Zero())
	, dq_wbc_(Vector18::Zero())
    , ddq_cmd_(Vector18::Zero())
    , f_with_delta_(Vector12::Zero())
    , ddq_with_delta_(Vector18::Zero())
    , log_feet_pos_target(Matrix34::Zero())
    , log_feet_vel_target(Matrix34::Zero())
    , log_feet_acc_target(Matrix34::Zero())
    , k_log_(0)
	, enable_comp_forces_(false)
{}

void WbcController::initialize(Params& params)
{
  // Params store parameters
  params_ = &params;

  // Set if compensation forces should be used or not
  enable_comp_forces_ = params.enable_comp_forces;

  // Path to the robot URDF (TODO: Automatic path)
  const std::string filename = std::string(URDF_MODEL);

  // Build model from urdf (base is not free flyer)
  pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(), model_, false);

  // Construct data from model
  data_ = pinocchio::Data(model_);

  // Update all the quantities of the model
  pinocchio::computeAllTerms(model_, data_ , VectorN::Zero(model_.nq), VectorN::Zero(model_.nv));

  // Initialize inverse kinematic and box QP solvers
  invkin_ = new InvKin();
  invkin_->initialize(params);
  box_qp_ = new QPWBC();
  box_qp_->initialize(params);

  // Initialize quaternion
  q_wbc_(6, 0) = 1.0;

  // Initialize joint positions
  qdes_.tail(12) = Vector12(params_->q_init.data());

  // Compute the upper triangular part of the joint space inertia matrix M by using the Composite Rigid Body Algorithm
  // Result is stored in data_.M
  pinocchio::crba(model_, data_, q_wbc_);

  // Make mass matrix symetric
  data_.M.triangularView<Eigen::StrictlyLower>() = data_.M.transpose().triangularView<Eigen::StrictlyLower>();

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
void WbcController::compute(VectorN const& q, VectorN const& dq, MatrixN const& f_cmd, MatrixN const& contacts,
                         MatrixN const& pgoals, MatrixN const& vgoals, MatrixN const& agoals, VectorN const &xgoals)
{
   if (f_cmd.rows() != 12) {
 	throw std::runtime_error("f_cmd should be a vector of size 12");
   }
   //  Update nb of iterations since contact
   k_since_contact_ += contacts;  // Increment feet in stance phase
   k_since_contact_ = k_since_contact_.cwiseProduct(contacts);  // Reset feet in swing phase

  // Store target positions, velocities and acceleration for logging purpose
   log_feet_pos_target = pgoals;
   log_feet_vel_target = vgoals;
   log_feet_acc_target = agoals;

   // Retrieve configuration data
   q_wbc_.head(3) = q.head(3);
   q_wbc_.block(3, 0, 4, 1) =
       pinocchio::SE3::Quaternion(pinocchio::rpy::rpyToMatrix(q(3, 0), q(4, 0), q(5, 0))).coeffs();  // Roll, Pitch
   q_wbc_.tail(12) = q.tail(12);                                                                     // Encoders

   // Retrieve velocity data
   dq_wbc_ = dq;

   // Compute the upper triangular part of the joint space inertia matrix M by using the Composite Rigid Body Algorithm
   // Result is stored in data_.M
   pinocchio::crba(model_, data_, q_wbc_);

   // Make mass matrix symetric
   data_.M.triangularView<Eigen::StrictlyLower>() = data_.M.transpose().triangularView<Eigen::StrictlyLower>();

   // Compute Inverse Kinematics
   invkin_->run(q_wbc_.tail(12), dq_wbc_.tail(12), contacts, pgoals, vgoals, agoals);
   ddq_cmd_.tail(12) = invkin_->get_ddq_cmd();

   // Compute the upper triangular part of the joint space inertia matrix M by using the Composite Rigid Body Algorithm
   // Result is stored in data_.M
   pinocchio::crba(model_, data_, q_wbc_);

  // std::cout << "ddq_cmd C++" << std::endl << ddq_cmd_ << std::endl;
  // TODO: Adapt logging of feet_pos, feet_err, feet_vel

  // TODO: Check if needed because crbaMinimal may allow to directly get the jacobian
  // TODO: Check if possible to use the model of InvKin to avoid computations
  // pinocchio::computeJointJacobians(model_, data_, q);

  // TODO: Check if we can save time by switching MatrixXd to defined sized vector since they are
  // not called from python anymore

  // Retrieve feet jacobian
  Matrix43 posf_tmp_ = invkin_->get_posf();
  for (int i = 0; i < 4; i++)
  {
    if (contacts(0, i))
    {
      Jc_.block(3 * i, 0, 3, 3) = Matrix3::Identity();
      Jc_.block(3 * i, 3, 3, 3) << 0.0, posf_tmp_(i, 2), -posf_tmp_(i, 1),
                                   -posf_tmp_(i, 2), 0.0, posf_tmp_(i, 0),
                                   posf_tmp_(i, 1), -posf_tmp_(i, 0), 0.0;
    }
    else
      Jc_.block(3 * i, 0, 3, 6).setZero();
  }


  // Compute the inverse dynamics, aka the joint torques according to the current state of the system,
  // the desired joint accelerations and the external forces, using the Recursive Newton Euler Algorithm.
  // Result is stored in data_.tau
  Vector12 f_compensation;
  if (!enable_comp_forces_) {
    pinocchio::rnea(model_, data_, q_wbc_, dq_wbc_, ddq_cmd_);
    f_compensation = Vector12::Zero();
  } else {
    Vector18 ddq_test = Vector18::Zero();
    ddq_test.head(6) = ddq_cmd_.head(6);
    pinocchio::rnea(model_, data_, q_wbc_, dq_wbc_, ddq_test);
    Vector6 RNEA_without_joints = data_.tau.head(6);
    pinocchio::rnea(model_, data_, q_wbc_, dq_wbc_, VectorN::Zero(model_.nv));
    Vector6 RNEA_NLE = data_.tau.head(6);
    RNEA_NLE(2, 0) -= 9.81 * data_.mass[0];
    pinocchio::rnea(model_, data_, q_wbc_, dq_wbc_, ddq_cmd_);

    f_compensation = pseudoInverse(Jc_.transpose()) * (data_.tau.head(6) - RNEA_without_joints + RNEA_NLE);
  }

  pinocchio::rnea(model_, data_, q_wbc_, dq_wbc_, ddq_cmd_);

  // Solve the QP problem
  box_qp_->run(data_.M, Jc_, f_cmd + f_compensation, data_.tau.head(6), k_since_contact_);

  // Add to reference quantities the deltas found by the QP solver
  f_with_delta_ = box_qp_->get_f_res();
  ddq_with_delta_.head(6) = ddq_cmd_.head(6) + box_qp_->get_ddq_res();
  ddq_with_delta_.tail(12) = ddq_cmd_.tail(12);

  // Compute joint torques from contact forces and desired accelerations
  pinocchio::rnea(model_, data_, q_wbc_, dq_wbc_, ddq_with_delta_);

  tau_ff_ = data_.tau.tail(12) - invkin_->get_Jf().transpose() * f_with_delta_;

  // Retrieve desired positions and velocities
  vdes_ = invkin_->get_dq_cmd();
  qdes_ = invkin_->get_q_cmd();
  bdes_ = invkin_->get_q_cmd().head(7);

  // Increment log counter
  k_log_++;
}
