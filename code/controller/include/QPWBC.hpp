#ifndef QPWBC_H_INCLUDED
#define QPWBC_H_INCLUDED

#include "InvKin.hpp" // For pseudoinverse
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <limits>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "osqp.h"
#include "st_to_cc.hpp"
#include "Types.h"
#include "Params.hpp"


class QPWBC {
 public:

  QPWBC(); // Constructor
  void initialize(Params &params);

  // Functions
  inline void add_to_ML(int i, int j, double v, int *r_ML, int *c_ML, double *v_ML); // function to fill the triplet r/c/v
  inline void add_to_P(int i, int j, double v, int *r_P, int *c_P, double *v_P); // function to fill the triplet r/c/v


  ////////////////////////////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Create the constraint matrices (M.X = N and L.X <= K)
  ///        Create the weight matrices P and Q (cost 1/2 x^T * P * X + X^T * Q)
  ///
  ////////////////////////////////////////////////////////////////////////////////////////////////
  int create_matrices(const Eigen::Matrix<double, 12, 6> &Jc, const Eigen::Matrix<double, 12, 1> &f_cmd, const Eigen::Matrix<double, 6, 1> &RNEA);

  ////////////////////////////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Create the M and L matrices involved in the constraint equations
  ///        the solution has to respect: M.X = N and L.X <= K
  ///
  ////////////////////////////////////////////////////////////////////////////////////////////////
  int create_ML();

  ////////////////////////////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Create the N and K matrices involved in the constraint equations
  ///        the solution has to respect: M.X = N and L.X <= K
  ///
  ////////////////////////////////////////////////////////////////////////////////////////////////
  int create_NK(const Eigen::Matrix<double, 6, 12> &JcT, const Eigen::Matrix<double, 12, 1> &f_cmd,
                const Eigen::Matrix<double, 6, 1> &RNEA);

  int create_weight_matrices();
  void compute_matrices(const Eigen::MatrixXd &M, const Eigen::MatrixXd &Jc, const Eigen::MatrixXd &f_cmd, const Eigen::MatrixXd &RNEA);
  void update_PQ();
  int call_solver();
  int retrieve_result(const Eigen::MatrixXd &f_cmd);
  int run(const Eigen::MatrixXd &M, const Eigen::MatrixXd &Jc, const Eigen::MatrixXd &f_cmd, const Eigen::MatrixXd &RNEA, const Eigen::MatrixXd &k_contact);
  //int run(const MatrixN &M, const MatrixN &Jc, const MatrixN &ddq_cmd, const MatrixN &f_cmd, const MatrixN &RNEA,  const MatrixN &k_contact);

  // Getters
  Eigen::MatrixXd get_f_res();
  Eigen::MatrixXd get_ddq_res();
  Eigen::MatrixXd get_H();

  // Utils
  void my_print_csc_matrix(csc *M, const char *name);
  void save_csc_matrix(csc *M, std::string filename);
  void save_dns_matrix(double *M, int size, std::string filename);
 private:
  Params *params_;  // Object that stores parameters

  int cpt_ML = 0;
  int cpt_P = 0;

  // Set to True after the creation of the QP problem during the first call of the solver
  bool initialized = false;

  // Weight matrices of initial QP
  Eigen::Matrix<double, 6, 6> Q1 = 0.1 * Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 12, 12> Q2 = 5.0 * Eigen::Matrix<double, 12, 12>::Identity();

  // Friction coefficient
  const double mu = 0.9;

  // Minimum and maximum normal contact force
  double Fz_max = 0.0;
  double Fz_min = 0.0;

  // Generatrix of the linearized friction cone
  Eigen::Matrix<double, 20, 12> G = Eigen::Matrix<double, 20, 12>::Zero();

  // Transformation matrices
  Eigen::Matrix<double, 6, 6> Y = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 12> X = Eigen::Matrix<double, 6, 12>::Zero();
  Eigen::Matrix<double, 6, 6> Yinv = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 12> A = Eigen::Matrix<double, 6, 12>::Zero();
  Eigen::Matrix<double, 6, 1> gamma = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 12, 12>  H = Eigen::Matrix<double, 12, 12>::Zero();
  Eigen::Matrix<double, 12, 1> g = Eigen::Matrix<double, 12, 1>::Zero();

  // Results
  // Eigen::Matrix<double, 12, 1> lambdas = Eigen::Matrix<double, 12, 1>::Zero();
  Eigen::MatrixXd f_res = Eigen::MatrixXd::Zero(12, 1);
  Eigen::MatrixXd ddq_res = Eigen::MatrixXd::Zero(12, 1);
  
  // Matrix ML
  const static int size_nz_ML = 20*12; //4 * (4 * 2 + 1);
  csc *ML;  // Compressed Sparse Column matrix

  // Matrix NK
  const static int size_nz_NK = (20 + 6);
  double v_NK_up[size_nz_NK] = {};   // matrix NK (upper bound)
  double v_NK_low[size_nz_NK] = {};  // matrix NK (lower bound)
  double v_warmxf[size_nz_NK] = {};  // matrix NK (lower bound)

  // Matrix P
  const static int size_nz_P = 6*13; // 6*13; // 12*13/2;
  csc *P;  // Compressed Sparse Column matrix

  // Matrix Q
  const static int size_nz_Q = 12;
  double Q[size_nz_Q] = {};  // Q is full of zeros

  // OSQP solver variables
  OSQPWorkspace *workspce = new OSQPWorkspace();
  OSQPData *data;
  OSQPSettings *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
};

#endif  // QPWBC_H_INCLUDED
