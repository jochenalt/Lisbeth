#include "WbcWrapper.hpp"

#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/math/rpy.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/rnea.hpp"


WbcWrapper::WbcWrapper()
    : M_(Eigen::Matrix<double, 18, 18>::Zero())
    , Jc_(Eigen::Matrix<double, 12, 6>::Zero())
    , k_since_contact_(Eigen::Matrix<double, 1, 4>::Zero())
    , qdes_(Vector12::Zero())
    , vdes_(Vector12::Zero())
    , tau_ff_(Vector12::Zero())
    , ddq_cmd_(Vector18::Zero())
    , q_default_(Vector19::Zero())
    , f_with_delta_(Vector12::Zero())
    , ddq_with_delta_(Vector18::Zero())
    , posf_tmp_(Matrix43::Zero())
    , log_feet_pos_target(Matrix34::Zero())
    , log_feet_vel_target(Matrix34::Zero())
    , log_feet_acc_target(Matrix34::Zero())
    , k_log_(0)
{}

void WbcWrapper::initialize(Params& params)
{
  // Params store parameters
  params_ = &params;

  // Path to the robot URDF (TODO: Automatic path)
  const std::string filename = std::string("/home/jochen/lisbeth/description/solo12.urdf");

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
  q_default_(6, 0) = 1.0;

  // Initialize joint positions
  qdes_.tail(12) = Vector12(params_->q_init.data());

  // Compute the upper triangular part of the joint space inertia matrix M by using the Composite Rigid Body Algorithm
  // Result is stored in data_.M
  pinocchio::crba(model_, data_, q_default_);

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
void WbcWrapper::compute(VectorN const& q, VectorN const& dq, MatrixN const& f_cmd, MatrixN const& contacts,
                         MatrixN const& pgoals, MatrixN const& vgoals, MatrixN const& agoals)
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

  // Compute Inverse Kinematics
  //std::cout << "WbcWrapper::InvKin" << std::endl;

   invkin_->run(q.tail(12), dq.tail(12), contacts, pgoals, vgoals, agoals);
  ddq_cmd_.tail(12) = invkin_->get_ddq_cmd();

  // std::cout << "ddq_cmd C++" << std::endl << ddq_cmd_ << std::endl;
  // TODO: Adapt logging of feet_pos, feet_err, feet_vel

  // TODO: Check if needed because crbaMinimal may allow to directly get the jacobian
  // TODO: Check if possible to use the model of InvKin to avoid computations
  // pinocchio::computeJointJacobians(model_, data_, q);

  // TODO: Check if we can save time by switching MatrixXd to defined sized vector since they are
  // not called from python anymore

  // Retrieve feet jacobian
  posf_tmp_ = invkin_->get_posf();
  // std::cout << "C++ posf_tmp_" << posf_tmp_<< std::endl;

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
    {
      Jc_.block(3 * i, 0, 3, 6).setZero();
    }
  }

  // std::cout << "C++ invkin_->get_Jf()" << Jc_ << std::endl;

  // Compute the inverse dynamics, aka the joint torques according to the current state of the system,
  // the desired joint accelerations and the external forces, using the Recursive Newton Euler Algorithm.
  // Result is stored in data_.tau
  pinocchio::rnea(model_, data_, q, dq, ddq_cmd_);

  /*std::cout << "M" << std::endl;
  std::cout << data_.M << std::endl;
  std::cout << "Jc" << std::endl;
  std::cout << Jc_ << std::endl;
  std::cout << "f_cmd" << std::endl;
  std::cout << f_cmd << std::endl;
  std::cout << "rnea" << std::endl;
  std::cout << data_.tau.head(6) << std::endl;
  std::cout << "k_since" << std::endl;
  std::cout << k_since_contact_ << std::endl;*/

  // Solve the QP problem
  box_qp_->run(data_.M, Jc_,  Eigen::Map<const VectorN>(f_cmd.data(), data_.tau.head(6), k_since_contact_);

  // Add to reference quantities the deltas found by the QP solver
  f_with_delta_ = box_qp_->get_f_res();
  ddq_with_delta_.head(6) = ddq_cmd_.head(6) + box_qp_->get_ddq_res();
  ddq_with_delta_.tail(12) = ddq_cmd_.tail(12);

  // Compute joint torques from contact forces and desired accelerations
  pinocchio::rnea(model_, data_, q, dq, ddq_with_delta_);

  /*std::cout << "rnea delta" << std::endl;
  std::cout << data_.tau.tail(12) << std::endl;
  std::cout << "ddq del" << std::endl;
  std::cout << ddq_with_delta_ << std::endl;
  std::cout << "f del" << std::endl;
  std::cout << f_with_delta_ << std::endl;
  std::cout << "C++ f_with_delta_" << f_with_delta_<< std::endl;
  std::cout << "C++ data_.tau.tail(12) " << data_.tau.tail(12) << std::endl; */

  tau_ff_ = data_.tau.tail(12) - invkin_->get_Jf().transpose() * f_with_delta_;

  // Retrieve desired positions and velocities
  vdes_ = invkin_->get_dq_cmd();
  qdes_ = invkin_->get_q_cmd();

  // Increment log counter
  k_log_++;
}
