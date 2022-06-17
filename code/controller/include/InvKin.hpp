#ifndef INVKIN_H_INCLUDED
#define INVKIN_H_INCLUDED

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"

#include "pinocchio/math/rpy.hpp"
#include "pinocchio/spatial/explog.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include "Types.h"
#include "Params.hpp"

class InvKin
{
public:
    InvKin();
    void initialize(Params& params);


    Eigen::Matrix<double, 1, 3> cross3(Eigen::Matrix<double, 1, 3> left, Eigen::Matrix<double, 1, 3> right);

    Eigen::MatrixXd refreshAndCompute(const Eigen::MatrixXd& contacts,
                                      const Eigen::MatrixXd& pgoals, const Eigen::MatrixXd& vgoals, const Eigen::MatrixXd& agoals,
                                      const Eigen::MatrixXd& posf, const Matrix43& vf, const Eigen::MatrixXd& wf,
                                      const Eigen::MatrixXd& af, const Eigen::MatrixXd& Jf);
    void run(VectorN const& q, VectorN const& dq, MatrixN const& contacts, MatrixN const& pgoals,
                            MatrixN const& vgoals, MatrixN const& agoals);

    VectorN get_q_step() { return q_step_; }
    VectorN get_q_cmd() { return q_cmd_; }
    VectorN get_dq_cmd() { return dq_cmd_; }
    VectorN get_ddq_cmd() { return ddq_cmd_; }
    int get_foot_id(int i) { return foot_ids_[i];}

    Matrix12 get_Jf() { return Jf_; }
    Matrix43 get_posf() { return posf_; }
    Matrix43 get_vf() { return vf_; }


private:

    Params* params_;
    // Inputs of the constructor
    double dt;  // Time step of the contact sequence (time step of the MPC)


    Matrix43 posf_;                        // Current feet positions
    Matrix43 vf_;                          // Current feet linear velocities
    Matrix43 wf_;                          // Current feet angular velocities
    Matrix43 af_;                          // Current feet linear accelerations
    Matrix43 dJdq_;                        // Acceleration "drift"
    Matrix12 Jf_;     // Current feet Jacobian (only linear part)
    Eigen::Matrix<double, 6, 12> Jf_tmp_;  // Temporary storage variable to only retrieve the linear part of the Jacobian
                                           // and discard the angular part

    int foot_ids_[4] = {0, 0, 0, 0};           // Feet frame IDs
    int foot_joints_ids_[4] = {4, 7, 10, 13};  // Feet joints IDs
    int base_id_ = 0;                          // Base ID

    // Matrices initialisation
    Eigen::Matrix<double, 4, 3> feet_position_ref = Eigen::Matrix<double, 4, 3>::Zero();
    Eigen::Matrix<double, 4, 3> feet_velocity_ref = Eigen::Matrix<double, 4, 3>::Zero();
    Eigen::Matrix<double, 4, 3> feet_acceleration_ref = Eigen::Matrix<double, 4, 3>::Zero();
    Eigen::Matrix<double, 1, 4> flag_in_contact = Eigen::Matrix<double, 1, 4>::Zero();

    Eigen::Matrix<double, 12, 12> invJ = Eigen::Matrix<double, 12, 12>::Zero();
    Eigen::Matrix<double, 1, 12> acc = Eigen::Matrix<double, 1, 12>::Zero();
    Eigen::Matrix<double, 1, 12> x_err = Eigen::Matrix<double, 1, 12>::Zero();
    Eigen::Matrix<double, 1, 12> dx_r = Eigen::Matrix<double, 1, 12>::Zero();

    Eigen::Matrix<double, 4, 3> pfeet_err = Eigen::Matrix<double, 4, 3>::Zero();
    Eigen::Matrix<double, 4, 3> vfeet_ref = Eigen::Matrix<double, 4, 3>::Zero();
    Eigen::Matrix<double, 4, 3> afeet = Eigen::Matrix<double, 4, 3>::Zero();
    Eigen::Matrix<double, 1, 3> e_basispos = Eigen::Matrix<double, 1, 3>::Zero();
    Eigen::Matrix<double, 1, 3> abasis = Eigen::Matrix<double, 1, 3>::Zero();
    Eigen::Matrix<double, 1, 3> e_basisrot = Eigen::Matrix<double, 1, 3>::Zero();
    Eigen::Matrix<double, 1, 3> awbasis = Eigen::Matrix<double, 1, 3>::Zero();

    Eigen::MatrixXd ddq_cmd_ = Eigen::MatrixXd::Zero(12, 1);
    Eigen::MatrixXd q_step_ = Eigen::MatrixXd::Zero(12, 1);
    Eigen::MatrixXd dq_cmd_ = Eigen::MatrixXd::Zero(12, 1);
    Vector12 q_cmd_;  										  // Actuator command positions


    Vector3 Kp_base_position;     // Proportional gains for base position task
    Vector3 Kd_base_position;     // Derivative gains for base position task
    Vector3 Kp_base_orientation;  // Proportional gains for base orientation task
    Vector3 Kd_base_orientation;  // Derivative gains for base orientation task
    Vector8 w_tasks;              // Weight vector for tasks weighting

    Vector19 q_wbc_;           // Configuration vector for the whole body control
    Vector18 dq_wbc_;          // Velocity vector for the whole body control


    // Gains
    double Kp_flyingfeet = 100.0; // 1000
    double Kd_flyingfeet = 2.0 * std::sqrt(Kp_flyingfeet); // 5.0 *

    pinocchio::Model model_;  // Pinocchio model for frame computations and inverse kinematics
    pinocchio::Data data_;    // Pinocchio datas for frame computations and inverse kinematics

    pinocchio::Model model_dJdq_;  // Pinocchio model for frame computations and inverse kinematics
    pinocchio::Data data_dJdq_;    // Pinocchio datas for frame computations and inverse kinematics

};

template <typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_& a, double epsilon = std::numeric_limits<double>::epsilon())
{
    Eigen::JacobiSVD<_Matrix_Type_> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * static_cast<double>(std::max(a.cols(), a.rows())) * svd.singularValues().array().abs()(0);
    return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}
#endif  // INVKIN_H_INCLUDED
