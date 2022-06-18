#include "FootstepPlanner.hpp"

#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/math/rpy.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "Utils.hpp"

FootstepPlanner::FootstepPlanner()
    : gait_(NULL)
    , k_feedback(0.03)
    , g(9.81)
    , L(0.155)
    , nextFootstep_(Matrix34::Zero())
    , footsteps_()
    , Rz(MatrixN::Zero(3, 3))
    , dt_cum()
    , yaws()
    , dx()
    , dy()
    , q_dxdy(Vector3::Zero())
    , RPY_(Vector3::Zero())
{
    // Empty
}

void FootstepPlanner::initialize(Params& params,
                                 Gait& gaitIn)
{
	this->params = &params;
    dt = params.dt_mpc;
    dt_wbc = params.dt_wbc;
    T_mpc = params.T_mpc;
    h_ref = params.h_ref;
    n_steps = (int)std::lround(params.T_mpc / params.dt_mpc);
    footsteps_under_shoulders_ << Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(params.footsteps_under_shoulders.data(),
                                                                params.footsteps_under_shoulders.size());
    // Offsets to make the support polygon smaller
    double ox = 0.0;
    double oy = 0.0;
    footsteps_offset_ << -ox, -ox, ox, ox, -oy, +oy, +oy, -oy, 0.0, 0.0, 0.0, 0.0;
    currentFootstep_ << Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(params.footsteps_init.data(),
                                                                      params.footsteps_init.size());
    gait_ = &gaitIn;
    targetFootstep_ = footsteps_under_shoulders_;
    o_targetFootstep_ = footsteps_under_shoulders_;
    dt_cum = VectorN::Zero(params.N_gait);
    yaws = VectorN::Zero(params.N_gait);
    dx = VectorN::Zero(params.N_gait);
    dy = VectorN::Zero(params.N_gait);
    for (int i = 0; i < params.N_gait; i++)
    {
        footsteps_.push_back(Matrix34::Zero());
    }
    Rz(2, 2) = 1.0;

    // Path to the robot URDF
    const std::string filename = std::string("/home/jochen/lisbeth/description/solo12.urdf");

    // Build model from urdf (base is not free flyer)
    pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(), model_, false);

    // Construct data from model
    data_ = pinocchio::Data(model_);

    // Update all the quantities of the model
    VectorN q_tmp = VectorN::Zero(model_.nq);
    q_tmp(6, 0) = 1.0;  // Quaternion (0, 0, 0, 1)
    pinocchio::computeAllTerms(model_, data_, q_tmp, VectorN::Zero(model_.nv));

    // Get feet frame IDs
    foot_ids_[0] = static_cast<int>(model_.getFrameId("FL_FOOT"));  // from long uint to int
    foot_ids_[1] = static_cast<int>(model_.getFrameId("FR_FOOT"));
    foot_ids_[2] = static_cast<int>(model_.getFrameId("HL_FOOT"));
    foot_ids_[3] = static_cast<int>(model_.getFrameId("HR_FOOT"));
}

MatrixN FootstepPlanner::updateFootsteps(bool refresh, int k, VectorN const& q, Vector6 const& b_v, Vector6 const& b_vref)
{
	  if (q.rows() != 18) {
	    throw std::runtime_error("q should be a vector of size 18 (pos+RPY+mot)");
	  }

	  // Update location of feet in stance phase (for those which just entered stance phase)
	  if (refresh && gait_->isNewPhase()) {
	    updateNewContact();
	  }

	  // Feet in contact with the ground are moving in base frame (they don't move in world frame)
	  double rotation_yaw = dt_wbc * b_vref(5);  // Rotation along Z for the last time step
	  double c = std::cos(rotation_yaw);
	  double s = std::sin(rotation_yaw);
	  Rz.topLeftCorner<2, 2>() << c, s, -s, c;
	  Vector2 dpos = dt_wbc * b_vref.head(2);  // Displacement along X and Y for the last time step
	  for (int j = 0; j < 4; j++) {
	    if (gait_->getCurrentGaitCoeff(0, j) == 1.0) {
	      currentFootstep_.block(0, j, 2, 1) = Rz * (currentFootstep_.block(0, j, 2, 1) - dpos);
	    }
	  }

	  // Compute location of footsteps
	  return computeTargetFootstep(k, q, b_v, b_vref);
}
void FootstepPlanner::computeFootsteps(int k, Vector6 const& b_v, Vector6 const& b_vref) {
  for (uint i = 0; i < footsteps_.size(); i++) {
    footsteps_[i] = Matrix34::Zero();
  }
  MatrixN gait = gait_->getCurrentGait();

  // Set current position of feet for feet in stance phase
  for (int j = 0; j < 4; j++) {
    if (gait(0, j) == 1.0) {
      footsteps_[0].col(j) = currentFootstep_.col(j);
    }
  }

  // Cumulative time by adding the terms in the first column (remaining number of timesteps)
  // Get future yaw yaws compared to current position
  dt_cum(0) = dt_wbc * k;
  yaws(0) = b_vref(5) * dt_cum(0);
  for (uint j = 1; j < footsteps_.size(); j++) {
    dt_cum(j) = gait.row(j).isZero() ? dt_cum(j - 1) : dt_cum(j - 1) + dt;
    yaws(j) = b_vref(5) * dt_cum(j);
  }

  // Displacement following the reference velocity compared to current position
  if (b_vref(5, 0) != 0) {
    for (uint j = 0; j < footsteps_.size(); j++) {
      dx(j) = (b_vref(0) * std::sin(b_vref(5) * dt_cum(j)) + b_vref(1) * (std::cos(b_vref(5) * dt_cum(j)) - 1.0)) /
              b_vref(5);
      dy(j) = (b_vref(1) * std::sin(b_vref(5) * dt_cum(j)) - b_vref(0) * (std::cos(b_vref(5) * dt_cum(j)) - 1.0)) /
              b_vref(5);
    }
  } else {
    for (uint j = 0; j < footsteps_.size(); j++) {
      dx(j) = b_vref(0) * dt_cum(j);
      dy(j) = b_vref(1) * dt_cum(j);
    }
  }

  // Update the footstep matrix depending on the different phases of the gait (swing & stance)
  for (int i = 1; i < gait.rows(); i++) {
    // Feet that were in stance phase and are still in stance phase do not move
    for (int j = 0; j < 4; j++) {
      if (gait(i - 1, j) > 0 && gait(i, j) > 0) {
        footsteps_[i].col(j) = footsteps_[i - 1].col(j);
      }
    }

    // Feet that were in swing phase and are now in stance phase need to be updated
    for (int j = 0; j < 4; j++) {
      if (gait(i - 1, j) == 0 && gait(i, j) > 0) {
        // Offset to the future position
        q_dxdy << dx(i - 1, 0), dy(i - 1, 0), 0.0;

        // Get future desired position of footsteps
        computeNextFootstep(i, j, b_v, b_vref);

        // Get desired position of footstep compared to current position
        double c = std::cos(yaws(i - 1));
        double s = std::sin(yaws(i - 1));
        Rz.topLeftCorner<2, 2>() << c, -s, s, c;

        footsteps_[i].col(j) = (Rz * nextFootstep_.col(j) + q_dxdy).transpose();
      }
    }
  }
}
void FootstepPlanner::computeNextFootstep(int i, int j, Vector6 const& b_v, Vector6 const& b_vref) {
  nextFootstep_ = Matrix34::Zero();
  double t_stance = gait_->getPhaseDurationCoeff(i, j);

  // Disable heuristic terms if gait is going to switch to static so that feet land at vertical of shoulders
  if (!gait_->getIsStatic()) {
    // Add symmetry term
    nextFootstep_.col(j) = t_stance * 0.5 * b_v.head(3);

    // Add feedback term
    nextFootstep_.col(j) += params->k_feedback * (b_v.head(3) - b_vref.head(3));

    // Add centrifugal term
    Vector3 cross;
    cross << b_v(1) * b_vref(5) - b_v(2) * b_vref(4), b_v(2) * b_vref(3) - b_v(0) * b_vref(5), 0.0;
    nextFootstep_.col(j) += 0.5 * std::sqrt(h_ref / g) * cross;
  }

  // Legs have a limited length so the deviation has to be limited
  nextFootstep_(0, j) = std::min(nextFootstep_(0, j), L);
  nextFootstep_(0, j) = std::max(nextFootstep_(0, j), -L);
  nextFootstep_(1, j) = std::min(nextFootstep_(1, j), L);
  nextFootstep_(1, j) = std::max(nextFootstep_(1, j), -L);

  // Add shoulders
  nextFootstep_.col(j) += footsteps_under_shoulders_.col(j);
  nextFootstep_.col(j) += footsteps_offset_.col(j);

  // Remove Z component (working on flat ground)
  nextFootstep_.row(2) = Vector4::Zero().transpose();
}

void FootstepPlanner::updateTargetFootsteps() {
  for (int i = 0; i < 4; i++) {
    int index = 0;
    while (footsteps_[index](0, i) == 0.0) {
      index++;
    }
    targetFootstep_.col(i) << footsteps_[index](0, i), footsteps_[index](1, i), 0.0;
  }
}

MatrixN FootstepPlanner::computeTargetFootstep(int k, VectorN const& q, Vector6 const& b_v, Vector6 const& b_vref)
{
    // Compute the desired location of footsteps over the prediction horizon
    computeFootsteps(k, b_v, b_vref);

    // Update desired location of footsteps on the ground
    updateTargetFootsteps();

    // Get o_targetFootstep_ in world frame from targetFootstep_ in horizontal frame
    RPY_ = Vector3({q(3), q(4), q(5)});

    // rotate footprints by RPY(2)
    double c = std::cos(RPY_(2));
    double s = std::sin(RPY_(2));
    Rz.topLeftCorner<2, 2>() << c, -s, s, c;
    for (int i = 0; i < 4; i++) {
      o_targetFootstep_.block(0, i, 2, 1) = Rz.topLeftCorner<2, 2>() * targetFootstep_.block(0, i, 2, 1) + q.head(2);
    }

    return o_targetFootstep_;
}

void FootstepPlanner::updateNewContact()
{
    for (int i = 0; i < 4; i++)
    {
        if (gait_->getCurrentGaitCoeff(0, i) == 1.0)
        {
            currentFootstep_.col(i) = (footsteps_[1]).col(i);
        }
    }
}
MatrixN FootstepPlanner::getFootsteps() { return vectorToMatrix(footsteps_); }
MatrixN FootstepPlanner::getTargetFootsteps() { return targetFootstep_; }

MatrixN FootstepPlanner::vectorToMatrix(std::vector<Matrix34> const& array) {
  MatrixN M = MatrixN::Zero(array.size(), 12);
  for (uint i = 0; i < array.size(); i++) {
    for (int j = 0; j < 4; j++) {
      M.row(i).segment<3>(3 * j) = array[i].col(j);
    }
  }
  return M;
}
