#ifndef INVKIN_H_INCLUDED
#define INVKIN_H_INCLUDED

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"

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


    Eigen::MatrixXd refreshAndCompute(const Vector4& contacts,
                                      const Matrix34& pgoals, const Matrix34& vgoals, const Matrix34& agoals);
    void run(Vector12 const& q, Vector12 const& dq, Vector4 const& contacts, Matrix34 const& pgoals,
    		Matrix34 const& vgoals, Matrix34 const& agoals);

    Vector12 get_q_step() { return q_step_; }
    Vector12 get_q_cmd() { return q_cmd_; }
    Vector12 get_dq_cmd() { return dq_cmd_; }
    Vector12 get_ddq_cmd() { return ddq_cmd_; }
    int get_foot_id(int i) { return foot_ids_[i];}

    Matrix12 get_Jf() { return Jf_; }
    Matrix43 get_posf() { return posf_; }
    Matrix43 get_vf() { return vf_; }


private:

    Params* params;


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
    Matrix43 feet_position_ref = Eigen::Matrix<double, 4, 3>::Zero();
    Matrix43 feet_velocity_ref = Eigen::Matrix<double, 4, 3>::Zero();
    Matrix43 feet_acceleration_ref = Eigen::Matrix<double, 4, 3>::Zero();
    RowVector4 flag_in_contact = Eigen::Matrix<double, 1, 4>::Zero();

    Matrix12 invJ = Eigen::Matrix<double, 12, 12>::Zero();
    RowVector12 acc = Eigen::Matrix<double, 1, 12>::Zero();
    RowVector12  x_err = Eigen::Matrix<double, 1, 12>::Zero();
    RowVector12  dx_r = Eigen::Matrix<double, 1, 12>::Zero();

    Matrix43 pfeet_err = Eigen::Matrix<double, 4, 3>::Zero();
    Matrix43 vfeet_ref = Eigen::Matrix<double, 4, 3>::Zero();
    Matrix43 afeet = Eigen::Matrix<double, 4, 3>::Zero();
    RowVector3  e_basispos = Eigen::Matrix<double, 1, 3>::Zero();
    RowVector3 abasis = Eigen::Matrix<double, 1, 3>::Zero();
    RowVector3 e_basisrot = Eigen::Matrix<double, 1, 3>::Zero();
    RowVector3 awbasis = Eigen::Matrix<double, 1, 3>::Zero();

    Vector12 ddq_cmd_ = Vector12::Zero(12, 1);
    Vector12 q_step_ = Vector12::Zero(12, 1);
    Vector12 dq_cmd_ = Vector12::Zero(12, 1);
    Vector12 q_cmd_;  										  // Actuator command positions

    Vector19 q_wbc_;           // Configuration vector for the whole body control
    Vector18 dq_wbc_;          // Velocity vector for the whole body control

    // Gains
    double Kp_flyingfeet = 100;
    double Kd_flyingfeet = 20;

    pinocchio::Model model_;  // Pinocchio model for frame computations and inverse kinematics
    pinocchio::Data data_;    // Pinocchio datas for frame computations and inverse kinematics

    pinocchio::Model model_dJdq_;  // Pinocchio model for frame computations and inverse kinematics
    pinocchio::Data data_dJdq_;    // Pinocchio datas for frame computations and inverse kinematics

};

// compute the pseudo inverse of a matrix using the Jacobi SVD formula
template <typename _Matrix_Type_>
MatrixN pseudoInverse(const _Matrix_Type_& a, double epsilon = std::numeric_limits<double>::epsilon()) {
  Eigen::JacobiSVD<MatrixN> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
  double tolerance =
      epsilon * static_cast<double>(std::max(a.cols(), a.rows())) * svd.singularValues().array().abs()(0);
  return svd.matrixV() *
         (svd.singularValues().array().abs() > tolerance)
             .select(svd.singularValues().array().inverse(), 0)
             .matrix()
             .asDiagonal() *
         svd.matrixU().adjoint();
}
#endif  // INVKIN_H_INCLUDED
