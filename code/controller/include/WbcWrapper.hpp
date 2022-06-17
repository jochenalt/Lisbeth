///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief This is the header for WbcWrapper classes
///
/// \details WbcWrapper provides an interface for the user to solve the whole body control problem
///          Internally it calls first the InvKin class to solve an inverse kinematics problem then calls the QPWBC
///          class to solve a box QP problem based on result from the inverse kinematic and desired ground forces
///
//////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef WBC_WRAPPER_H_INCLUDED
#define WBC_WRAPPER_H_INCLUDED

#include "InvKin.hpp"
#include "QPWBC.hpp"
#include "Params.hpp"

class WbcWrapper
{
public:
    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Empty constructor
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    WbcWrapper();

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Destructor.
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    ~WbcWrapper() {}

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Initializer
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    void initialize(Params& params);

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Compute the remaining and total duration of a swing phase or a stance phase based
    ///        on the content of the gait matrix
    ///
    /// \param[in] i considered phase (row of the gait matrix)
    /// \param[in] j considered foot (col of the gait matrix)
    /// \param[in] value 0.0 for swing phase detection, 1.0 for stance phase detection
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    void compute(VectorN const& q, VectorN const& dq, MatrixN const& f_cmd, MatrixN const& contacts,
                 MatrixN const& pgoals, MatrixN const& vgoals, MatrixN const& agoals);

    VectorN get_qdes() { return qdes_; }
    VectorN get_vdes() { return vdes_; }
    VectorN get_tau_ff() { return tau_ff_; }
    VectorN get_f_with_delta() { return f_with_delta_; }
    MatrixN get_feet_pos() { return invkin_->get_posf().transpose(); }
    MatrixN get_feet_err() { return log_feet_pos_target - invkin_->get_posf().transpose(); }
    MatrixN get_feet_vel() { return invkin_->get_vf().transpose(); }
    MatrixN get_feet_pos_target() { return log_feet_pos_target; }
    MatrixN get_feet_vel_target() { return log_feet_vel_target; }
    MatrixN get_feet_acc_target() { return log_feet_acc_target; }

private:

    Params* params_;  // Object that stores parameters
    QPWBC* box_qp_;  // QP problem solver for the whole body control
    InvKin* invkin_;  // Inverse Kinematics solver for the whole body control

    pinocchio::Model model_;  // Pinocchio model for frame computations
    pinocchio::Data data_;  // Pinocchio datas for frame computations

    Eigen::Matrix<double, 18, 18> M_;  // Mass matrix
    Eigen::Matrix<double, 12, 6> Jc_;  // Jacobian matrix
    Eigen::Matrix<double, 1, 4> k_since_contact_;
    Vector12 qdes_;  // Desired actuator positions
    Vector12 vdes_;  // Desired actuator velocities
    Vector12 tau_ff_;  // Desired actuator torques (feedforward)

  	Vector19 q_wbc_;           // Configuration vector for the whole body control
    Vector18 dq_wbc_;          // Velocity vector for the whole body control
    Vector18 ddq_cmd_;  // Actuator accelerations computed by Inverse Kinematics
    Vector12 f_with_delta_;  // Contact forces with deltas found by QP solver
    Vector18 ddq_with_delta_;  // Actuator accelerations with deltas found by QP solver

    Matrix43 posf_tmp_;  // Temporary matrix to store posf_ from invkin_

    Matrix34 log_feet_pos_target;  // Store the target feet positions
    Matrix34 log_feet_vel_target;  // Store the target feet velocities
    Matrix34 log_feet_acc_target;  // Store the target feet accelerations

    int k_log_;  // Counter for logging purpose
    int indexes_[4] = {10, 18, 26, 34};  // Indexes of feet frames in this order: [FL, FR, HL, HR]
    
    bool enable_comp_forces_;            // Enable compensation forces for the QP problem
};


#endif  // WBC_WRAPPER_H_INCLUDED
