#include "InvKin.hpp"

#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/math/rpy.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/spatial/explog.hpp"

InvKin::InvKin():
    posf_(Matrix43::Zero()),
    vf_(Matrix43::Zero()),
    wf_(Matrix43::Zero()),
    af_(Matrix43::Zero()),
	 dJdq_(Matrix43::Zero()),
    Jf_(Eigen::Matrix<double, 12, 12>::Zero()),
	 Jf_tmp_(Eigen::Matrix<double, 6, 12>::Zero()),
    acc(Matrix112::Zero()),
    ddq_cmd_(Vector12::Zero()),
    dq_cmd_(Vector12::Zero()),
    q_cmd_(Vector12::Zero())

{}

void InvKin::initialize(Params& params_in) {
	params = &params_in;

   // Path to the robot URDF (defined in CMakeLists.txt
	const std::string filename = std::string(URDF_MODEL);

   // Build model from urdf (base is not free flyer)
   pinocchio::urdf::buildModel(filename, model_, false);

    // Create data required by the algorithms
	// for estimation estimation (forward kinematics)
	data_ = pinocchio::Data(model_);

	// Update all the quantities of the model
	pinocchio::computeAllTerms(model_, data_, VectorN::Zero(model_.nq), VectorN::Zero(model_.nv));

	// Get feet frame IDs
	foot_ids_[0] = static_cast<int>(model_.getFrameId("FL_FOOT"));
	foot_ids_[1] = static_cast<int>(model_.getFrameId("FR_FOOT"));
	foot_ids_[2] = static_cast<int>(model_.getFrameId("HL_FOOT"));
	foot_ids_[3] = static_cast<int>(model_.getFrameId("HR_FOOT"));

	// Get base ID
	base_id_ = static_cast<int>(model_.getFrameId("base_link"));

	// Set task gains
	Kp_flyingfeet = params->Kp_flyingfeet;
	Kd_flyingfeet = params->Kd_flyingfeet;

}
Eigen::MatrixXd InvKin::refreshAndCompute(const Vector4 &contacts,
                                          const Matrix34 &goals, const Matrix34 &vgoals, const Matrix34 &agoals) {

    // Update contact status of the feet
    flag_in_contact.block(0, 0, 1, 4) = contacts.block(0, 0, 1, 4);

    // Update position, velocity and acceleration references for the feet
    for (int i = 0; i < 4; i++) {
        feet_position_ref.block(i, 0, 1, 3) = goals.block(0, i, 3, 1).transpose();
        feet_velocity_ref.block(i, 0, 1, 3) = vgoals.block(0, i, 3, 1).transpose();
        feet_acceleration_ref.block(i, 0, 1, 3) = agoals.block(0, i, 3, 1).transpose();
    }

    // Process feet
    for (int i = 0; i < 4; i++) {

        pfeet_err.row(i) = feet_position_ref.row(i) - posf_.row(i);
        vfeet_ref.row(i) = feet_velocity_ref.row(i);
        afeet.row(i) = + Kp_flyingfeet * pfeet_err.row(i) - Kd_flyingfeet * (vf_.row(i)-feet_velocity_ref.row(i)) + feet_acceleration_ref.row(i);
        if (flag_in_contact(0, i)) {
            afeet.row(i) *= 0.0; // Set to 0.0 to disable position/velocity control of feet in contact
        }
        afeet.row(i) -= af_.row(i) + (wf_.row(i)).cross(vf_.row(i));

    }

    // Store data and invert the Jacobian
    for (int i = 0; i < 4; i++) {
        acc.block(0, 3*i, 1, 3) = afeet.row(i);
        x_err.block(0, 3*i, 1, 3) = pfeet_err.row(i);
        dx_r.block(0, 3*i, 1, 3) = vfeet_ref.row(i);
        invJ.block(3*i, 3*i, 3, 3) = Jf_.block(3*i, 3*i, 3, 3).inverse();
    }

    // Once Jacobian has been inverted we can get command accelerations, velocities and positions
    ddq_cmd_ = invJ * acc.transpose();
    dq_cmd_ = invJ * dx_r.transpose();
    q_step_ = invJ * x_err.transpose(); // Not a position but a step in position

    return ddq_cmd_;
}



void InvKin::run(Vector12 const& q, Vector12 const& dq, Vector4 const& contacts, Matrix34 const& pgoals,
		Matrix34 const& vgoals, Matrix34 const& agoals) {

  // Update model and data of the robot
  pinocchio::computeJointJacobians(model_, data_, q);
  pinocchio::forwardKinematics(model_, data_, q, dq, VectorN::Zero(model_.nv));
  pinocchio::updateFramePlacements(model_, data_);

  for (int i = 0; i < 4; i++) {
    int idx = foot_ids_[i];
    posf_.row(i) = data_.oMf[idx].translation();

    pinocchio::Motion nu = pinocchio::getFrameVelocity(model_, data_, idx, pinocchio::LOCAL_WORLD_ALIGNED);
    vf_.row(i) = nu.linear();
    wf_.row(i) = nu.angular();
    af_.row(i) = pinocchio::getFrameAcceleration(model_, data_, idx, pinocchio::LOCAL_WORLD_ALIGNED).linear();
    Jf_tmp_.setZero();  // Fill with 0s because getFrameJacobian only acts on the coeffs it changes so the
    // other coeffs keep their previous value instead of being set to 0
    pinocchio::getFrameJacobian(model_, data_, idx, pinocchio::LOCAL_WORLD_ALIGNED, Jf_tmp_);
    Jf_.block(3 * i, 0, 3, 12) = Jf_tmp_.block(0, 0, 3, 12);
  }

  // IK output for accelerations of actuators (stored in ddq_cmd_)
  // IK output for velocities of actuators (stored in dq_cmd_)
  refreshAndCompute(contacts, pgoals, vgoals, agoals);

  // IK output for positions of actuators
  q_cmd_ = q + q_step_;
}



