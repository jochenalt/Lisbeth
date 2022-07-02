#ifndef MPC_H_INCLUDED
#define MPC_H_INCLUDED

#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <limits>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "osqp.h"
#include "Types.h"
#include "Params.hpp"


class MPCSolver {
 public:
 ////////////////////////////////////////////////////////////////////////////////////////////////
 ///
 /// \brief Constructor
 ///
 ////////////////////////////////////////////////////////////////////////////////////////////////
 MPCSolver();

 ////////////////////////////////////////////////////////////////////////////////////////////////
 ///
 /// \brief Constructor with parameters
 ///
 /// \param[in] params Object that stores parameters
 ///
 ////////////////////////////////////////////////////////////////////////////////////////////////
 MPCSolver(Params &params);

 ////////////////////////////////////////////////////////////////////////////////////////////////
 ///
 /// \brief Destructor
 ///
 ////////////////////////////////////////////////////////////////////////////////////////////////
 ~MPCSolver() {}  // Empty destructor

 ////////////////////////////////////////////////////////////////////////////////////////////////
 ///
 /// \brief Run one iteration of the whole MPC by calling all the necessary functions (data retrieval, update
 ///        of constraint matrices, update of the solver, running the solver, retrieving result)
 ///
 /// \param[in] num_iter Number of the current iteration of the MPC
 /// \param[in] xref_in Reference state trajectory over the prediction horizon
 /// \param[in] fsteps_in Footsteps location over the prediction horizon stored in a Nx12 matrix
 ///
 ////////////////////////////////////////////////////////////////////////////////////////////////
 void run(const MatrixN &xref_in, const MatrixN &fsteps_in);

 ////////////////////////////////////////////////////////////////////////////////////////////////
 ///
 /// \brief Retrieve the value of the cost function at the end of the resolution
 /// \return the cost value
 ///
 ////////////////////////////////////////////////////////////////////////////////////////////////
 float retrieve_cost();

 // Getters
 MatrixN get_latest_result();  // Return the latest desired contact forces that have been computed
 MatrixNi get_gait();           // Return the gait matrix
 VectorNi get_Sgait();          // Return the S_gait matrix
 double *get_x_next();         // Return the next predicted state of the base
 private:
  int create_matrices();
  int create_ML();
  int create_NK();
  int create_weight_matrices();
  int update_matrices(Eigen::MatrixXd fsteps);
  int update_ML(Eigen::MatrixXd fsteps);
  int update_NK();
  void init_solver();
  void call_solver();
  int retrieve_result();

  Eigen::Matrix<double, 3, 3> getSkew(Eigen::Matrix<double, 3, 1> v);
  int construct_S();
  int construct_gait(Eigen::MatrixXd fsteps_in);

  Params* params;

  double mass, mu;
  int cpt_ML, cpt_P;

  Eigen::Matrix<double, 3, 3> gI;
  Eigen::Matrix<double, 6, 1> v = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 3, 4> footholds = Eigen::Matrix<double, 3, 4>::Zero();
  Eigen::Matrix<double, 1, 12> footholds_tmp = Eigen::Matrix<double, 12, 1>::Zero();
  Eigen::Matrix<double, 3, 4> lever_arms = Eigen::Matrix<double, 3, 4>::Zero();
  Eigen::Matrix<int, Eigen::Dynamic, 4> gait;
  Eigen::Matrix<int, Eigen::Dynamic, 4> inv_gait;
  Eigen::Matrix<double, 12, 1> g = Eigen::Matrix<double, 12, 1>::Zero();
  Eigen::Matrix<double, 3, 1> offset_CoM = Eigen::Matrix<double, 3, 1>::Zero();

  Eigen::Matrix<double, 12, 12> A = Eigen::Matrix<double, 12, 12>::Identity();
  Eigen::Matrix<double, 12, 12> B = Eigen::Matrix<double, 12, 12>::Zero();
  Eigen::Matrix<double, 12, 1> x0 = Eigen::Matrix<double, 12, 1>::Zero();
  double x_next[12] = {};
  Eigen::MatrixXd x_f_applied;

  // Matrix ML
  const static int size_nz_ML = 5000;

  csc *ML;  // Compressed Sparse Column matrix
  inline void add_to_ML(int i, int j, double v, int *r_ML, int *c_ML,
                        double *v_ML);                                            // function to fill the triplet r/c/v
  inline void add_to_P(int i, int j, double v, int *r_P, int *c_P, double *v_P);  // function to fill the triplet r/c/v

  // Indices that are used to udpate ML
  int i_x_B[12 * 4] = {};
  int i_y_B[12 * 4] = {};
  int i_update_B[12 * 4] = {};

  // Matrix NK
  const static int size_nz_NK = 5000;
  double v_NK_up[size_nz_NK] = {};   // maxtrix NK (upper bound)
  double v_NK_low[size_nz_NK] = {};  // maxtrix NK (lower bound)
  double v_warmxf[size_nz_NK] = {};  // maxtrix NK (lower bound)

  // Matrix P
  const static int size_nz_P = 5000;
  csc *P;  // Compressed Sparse Column matrix

  // Matrix Q
  const static int size_nz_Q = 5000;
  double Q[size_nz_Q] = {};  // Q is full of zeros

  // OSQP solver variables
  OSQPWorkspace *workspce = new OSQPWorkspace();
  OSQPData *data;
  OSQPSettings *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));

  // Matrices whose size depends on the arguments sent to the constructor function
  Eigen::Matrix<double, 12, Eigen::Dynamic> xref;
  Eigen::Matrix<double, Eigen::Dynamic, 1> x;
  Eigen::Matrix<int, Eigen::Dynamic, 1> S_gait;
  Eigen::Matrix<double, Eigen::Dynamic, 1> warmxf;
  Eigen::Matrix<double, Eigen::Dynamic, 1> NK_up;
  Eigen::Matrix<double, Eigen::Dynamic, 1> NK_low;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> D;
  Eigen::Matrix<int, Eigen::Dynamic, 1> i_off;
};

#endif  // MPC_H_INCLUDED
