#include <MPCSolver.hpp>
#include "st_to_cc.hpp"

MPCSolver::MPCSolver(Params& params_in) {
  params = &params_in;

  int n_steps = params->get_N_steps() * params->N_periods;

  xref = Matrix12N::Zero(12, 1 + n_steps);
  x = VectorN::Zero(12 * n_steps * 2);
  S_gait = VectorNi::Zero(12 * n_steps);
  warmxf = VectorN::Zero(12 * n_steps * 2);
  x_f_applied = MatrixN::Zero(24, n_steps);

  gait = MatrixN4i::Zero(n_steps, 4);

  // Predefined variables
  mass = params->mass;
  mu = 0.9f;
  cpt_ML = 0;
  cpt_P = 0;

  // deviation of base from centre of mass
  offset_CoM = Vector3(params->CoM_offset[0], params->CoM_offset[1],params->CoM_offset[2]);

  // initial contact positions of feet
  footholds << Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(params->footsteps_under_shoulders.data(),
	                                                          params->footsteps_under_shoulders.size());

  //  Composite rigid body inertia in init position
  gI << Eigen::Map<VectorN, Eigen::Unaligned>(params->I_mat.data(), params->I_mat.size());

  g(8, 0) = -9.81f * params->dt_mpc;

  osqp_set_default_settings(settings);

  create_matrices();

  init_solver();
}

MPCSolver::MPCSolver() { }

/*
Create the constraint matrices of the MPC (M.X = N and L.X <= K)
Create the weight matrices P and Q of the MPC solver (cost 1/2 x^T * P * X + X^T * Q)
*/
int MPCSolver::create_matrices() {
  // Create the constraint matrices
  create_ML();
  create_NK();

  // Create the weight matrices
  create_weight_matrices();

  return 0;
}

/*
Add a new non-zero coefficient to the ML matrix by filling the triplet r_ML / c_ML / v_ML
*/
inline void MPCSolver::add_to_ML(int i, int j, double v, int *r_ML, int *c_ML, double *v_ML) {
  r_ML[cpt_ML] = i;  // row index
  c_ML[cpt_ML] = j;  // column index
  v_ML[cpt_ML] = v;  // value of coefficient
  cpt_ML++;          // increment the counter
}

/*
Add a new non-zero coefficient to the P matrix by filling the triplet r_P / c_P / v_P
*/
inline void MPCSolver::add_to_P(int i, int j, double v, int *r_P, int *c_P, double *v_P) {
  r_P[cpt_P] = i;  // row index
  c_P[cpt_P] = j;  // column index
  v_P[cpt_P] = v;  // value of coefficient
  cpt_P++;         // increment the counter
}

/*
Create the M and L matrices involved in the MPC constraint equations M.X = N and L.X <= K
*/
int MPCSolver::create_ML() {
  int *r_ML = new int[size_nz_ML];        // row indexes of non-zero values in matrix ML
  int *c_ML = new int[size_nz_ML];        // col indexes of non-zero values in matrix ML
  double *v_ML = new double[size_nz_ML];  // non-zero values in matrix ML

  std::fill_n(r_ML, size_nz_ML, 0);
  std::fill_n(c_ML, size_nz_ML, 0);
  std::fill_n(v_ML, size_nz_ML, 0.0);  // initialized to -1.0

  int n_steps = params->get_N_steps() * params->N_periods;
  // Put identity matrices in M
  for (int k = 0; k < (12 * n_steps); k++) {
    add_to_ML(k, k, -1.0, r_ML, c_ML, v_ML);
  }

  // Fill matrix A (for other functions)
  A.block(0, 6, 6, 6) = params->dt_mpc * Matrix6::Identity();

  // Put A matrices in M
  for (int k = 0; k < (n_steps - 1); k++) {
    for (int i = 0; i < 12; i++) {
      add_to_ML((k + 1) * 12 + i, (k * 12) + i, 1.0, r_ML, c_ML, v_ML);
    }
    for (int j = 0; j < 6; j++) {
      add_to_ML((k + 1) * 12 + j, (k * 12) + j + 6, params->dt_mpc, r_ML, c_ML, v_ML);
    }
  }

  // Put B matrices in M
  double div_tmp = params->dt_mpc / mass;
  for (int k = 0; k < n_steps; k++) {
    for (int i = 0; i < 4; i++) {
      add_to_ML(12 * k + 6, 12 * (n_steps + k) + 0 + 3 * i, div_tmp, r_ML, c_ML, v_ML);
      add_to_ML(12 * k + 7, 12 * (n_steps + k) + 1 + 3 * i, div_tmp, r_ML, c_ML, v_ML);
      add_to_ML(12 * k + 8, 12 * (n_steps + k) + 2 + 3 * i, div_tmp, r_ML, c_ML, v_ML);
    }
    for (int i = 0; i < 12; i++) {
      add_to_ML(12 * k + 9, 12 * (n_steps + k) + i, 8.0, r_ML, c_ML, v_ML);
      add_to_ML(12 * k + 10, 12 * (n_steps + k) + i, 8.0, r_ML, c_ML, v_ML);
      add_to_ML(12 * k + 11, 12 * (n_steps + k) + i, 8.0, r_ML, c_ML, v_ML);
    }
  }
  for (int i = 0; i < 4; i++) {
    B(6, 0 + 3 * i) = div_tmp;
    B(7, 1 + 3 * i) = div_tmp;
    B(8, 2 + 3 * i) = div_tmp;
    B(9, i) = 8.0;
    B(10, i) = 8.0;
    B(11, i) = 8.0;
  }

  // Add lines to enable/disable forces
  for (int i = 12 * n_steps; i < 12 * n_steps * 2; i++) {
    add_to_ML(i, i, 1.0, r_ML, c_ML, v_ML);
  }

  // Fill ML with F matrices
  int offset_L = 12 * n_steps * 2;
  for (int k = 0; k < n_steps; k++) {
    int di = offset_L + 20 * k;
    int dj = 12 * (n_steps + k);
    // Matrix F with top left corner at (di, dj) in ML
    for (int i = 0; i < 4; i++) {
      int dx = 5 * i;
      int dy = 3 * i;
      int a[9] = {0, 1, 2, 3, 0, 1, 2, 3, 4};
      int b[9] = {0, 0, 1, 1, 2, 2, 2, 2, 2};
      double c[9] = {1.0, -1.0, 1.0, -1.0, -mu, -mu, -mu, -mu, -1};
      // Matrix C with top left corner at (dx, dy) in F
      for (int j = 0; j < 9; j++) {
        add_to_ML(di + dx + a[j], dj + dy + b[j], c[j], r_ML, c_ML, v_ML);
      }
    }
  }

  // Creation of CSC matrix
  int *icc;                                  // row indices
  int *ccc;                                  // col indices
  double *acc;                               // coeff values
  int nst = cpt_ML;                          // number of non zero elements
  int ncc = st_to_cc_size(nst, r_ML, c_ML);  // number of CC values
  // int m = 12 * n_steps * 2 + 20 * n_steps;   // number of rows
  int n = 12 * n_steps * 2;  // number of columns

  // Get the CC indices.
  icc = (int *)malloc(ncc * sizeof(int));
  ccc = (int *)malloc((n + 1) * sizeof(int));
  st_to_cc_index(nst, r_ML, c_ML, ncc, n, icc, ccc);

  // Get the CC values.
  acc = st_to_cc_values(nst, r_ML, c_ML, v_ML, ncc, n, icc, ccc);

  // Assign values to the csc object
  ML = (csc *)c_malloc(sizeof(csc));
  ML->m = 12 * n_steps * 2 + 20 * n_steps;
  ML->n = 12 * n_steps * 2;
  ML->nz = -1;
  ML->nzmax = ncc;
  ML->x = acc;
  ML->i = icc;
  ML->p = ccc;

  // Free memory
  delete[] r_ML;
  delete[] c_ML;
  delete[] v_ML;

  // Create indices list that will be used to update ML
  int i_x_tmp[12] = {6, 9, 10, 11, 7, 9, 10, 11, 8, 9, 10, 11};
  for (int k = 0; k < 4; k++) {
    for (int i = 0; i < 12; i++) {
      i_x_B[12 * k + i] = i_x_tmp[i];
      i_y_B[12 * k + i] = (12 * k + i) / 4;  // 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2...
    }
  }

  int i_start = 30 * n_steps - 18;
  int i_data[12] = {0, 1, 2, 3, 7, 8, 9, 10, 14, 15, 16, 17};
  int i_foot[4] = {0 * 24, 1 * 24, 2 * 24, 3 * 24};
  for (int k = 0; k < 4; k++) {
    for (int i = 0; i < 12; i++) {
      i_update_B[12 * k + i] = i_start + i_data[i] + i_foot[k];
    }
  }

  // i_update_S here?

  // Update state of B
  for (int k = 0; k < n_steps; k++) {
    // Get inverse of the inertia matrix for time step k
    double c = cos(xref(5, k));
    double s = sin(xref(5, k));
    Matrix3 R;
    R << c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0;
    Matrix3 R_gI = R.transpose() * gI * R;
    Matrix3 I_inv = R_gI.inverse();

    // Get skew-symetric matrix for each foothold
    Matrix34 l_arms = footholds - (xref.block(0, k, 3, 1)).replicate<1, 4>();
    for (int i = 0; i < 4; i++) {
      B.block(9, 3 * i, 3, 3) = params->dt_mpc * (I_inv * getSkew(l_arms.col(i)));
    }

    int i_iter = 24 * 4 * k;
    for (int j = 0; j < 12 * 4; j++) {
      ML->x[i_update_B[j] + i_iter] = B(i_x_B[j], i_y_B[j]);
    }
  }

  // Update lines to enable/disable forces
  construct_S();

  Vector3i i_tmp1;
  i_tmp1 << 3 + 4, 3 + 4, 6 + 4;
  VectorNi i_tmp2 = VectorNi::Zero(12 * n_steps, 1);  // i_tmp1.replicate<4,1>();
  for (int k = 0; k < 4 * n_steps; k++) {
    i_tmp2.block(3 * k, 0, 3, 1) = i_tmp1;
  }

  i_off = VectorNi::Zero(12 * n_steps, 1);
  i_off(0, 0) = 4;
  for (int k = 1; k < 12 * n_steps; k++) {
    i_off(k, 0) = i_off(k - 1, 0) + i_tmp2(k - 1, 0);
    // ML->x[i_off(k, 0)+ i_start] = S_gait(k, 0);
  }
  for (int k = 0; k < 12 * n_steps; k++) {
    ML->x[i_off(k, 0) + i_start] = S_gait(k, 0);
  }

  return 0;
}

/*
Create the N and K matrices involved in the MPC constraint equations M.X = N and L.X <= K
*/
int MPCSolver::create_NK() {
	  int n_steps = params->get_N_steps() * params->N_periods;

  // Create NK matrix (upper and lower bounds)
  NK_up = VectorN::Zero(12 * n_steps * 2 + 20 * n_steps, 1);
  NK_low = VectorN::Zero(12 * n_steps * 2 + 20 * n_steps, 1);

  // Fill N matrix with g matrices
  for (int k = 0; k < n_steps; k++) {
    NK_up(12 * k + 8, 0) = -g(8, 0);  // only 8-th coeff is non zero
  }

  // Including - A*X0 in the first row of N
  NK_up.block(0, 0, 12, 1) += A * (-x0);

  // Create matrix D (third term of N) and put identity matrices in it
  D = MatrixN::Identity(12 * n_steps, 12 * n_steps);

  // Put -A matrices in D
  for (int k = 0; k < n_steps - 1; k++) {
    for (int i = 0; i < 12; i++) {
      D((k + 1) * 12 + i, (k * 12) + i) = -1.0;
    }
    for (int i = 0; i < 6; i++) {
      D((k + 1) * 12 + i, (k * 12) + i + 6) = -params->dt_mpc;
    }
  }

  // Add third term to matrix N
  Eigen::Map<MatrixN> xref_col((xref.block(0, 1, 12, n_steps)).data(), 12 * n_steps, 1);
  NK_up.block(0, 0, 12 * n_steps, 1) += D * xref_col;

  // Lines to enable/disable forces are already initialized (0 values)
  // Matrix K is already initialized (0 values)
  VectorN inf_lower_bount = -std::numeric_limits<double>::infinity() * VectorN::Ones(20 * n_steps, 1);
  for (int k = 0; (4 + 5 * k) < (20 * n_steps); k++) {
    inf_lower_bount(4 + 5 * k, 0) = -params->osqp_Nz_lim;  // Maximum vertical contact force [N]
  }

  NK_low.block(0, 0, 12 * n_steps * 2, 1) = NK_up.block(0, 0, 12 * n_steps * 2, 1);
  NK_low.block(12 * n_steps * 2, 0, 20 * n_steps, 1) = inf_lower_bount;

  VectorN::Map(&v_NK_up[0], NK_up.size()) = NK_up;
  VectorN::Map(&v_NK_low[0], NK_low.size()) = NK_low;

  return 0;
}

/*
Create the weight matrices P and q in the cost function x^T.P.x + x^T.q of the QP problem
*/
int MPCSolver::create_weight_matrices() {
	int n_steps = params->get_N_steps() * params->N_periods;

  int *r_P = new int[size_nz_P];        // row indexes of non-zero values in matrix P
  int *c_P = new int[size_nz_P];        // col indexes of non-zero values in matrix P
  double *v_P = new double[size_nz_P];  // non-zero values in matrix P

  std::fill_n(r_P, size_nz_P, 0);
  std::fill_n(c_P, size_nz_P, 0);
  std::fill_n(v_P, size_nz_P, 0.0);

  // Define weights for the x-x_ref components of the optimization vector
  // Hand-tuning of parameters if you want to give more weight to specific components
  // double w[12] = {10.0f, 10.0f, 1.0f, 1.0f, 1.0f, 10.0f};
  // double w[12] = {2.0f, 2.0f, 20.0f, 2.0f, 2.0f, 10.0f, 0.2f, 0.2f, 0.2f, 0.0f, 0.0f, 10.0f};
  // double w[12] = {2.0f, 2.0f, 20.0f, 0.25f, 0.25f, 10.0f, 0.2f, 0.2f, 0.2f, 0.0f, 0.0f, 0.3f};
  for (int k = 0; k < n_steps; k++) {
    for (int i = 0; i < 12; i++) {
      add_to_P(12 * k + i, 12 * k + i, params->osqp_w_states[i], r_P, c_P, v_P);
    }
  }

  // Define weights for the force components of the optimization vector
  for (int k = n_steps; k < (2 * n_steps); k++) {
    for (int i = 0; i < 4; i++) {
      add_to_P(12 * k + 3 * i + 0, 12 * k + 3 * i + 0, params->osqp_w_forces[0], r_P, c_P, v_P);
      add_to_P(12 * k + 3 * i + 1, 12 * k + 3 * i + 1, params->osqp_w_forces[1], r_P, c_P, v_P);
      add_to_P(12 * k + 3 * i + 2, 12 * k + 3 * i + 2, params->osqp_w_forces[2], r_P, c_P, v_P);
    }
  }

  // Creation of CSC matrix
  int *icc;                                // row indices
  int *ccc;                                // col indices
  double *acc;                             // coeff values
  int nst = cpt_P;                         // number of non zero elements
  int ncc = st_to_cc_size(nst, r_P, c_P);  // number of CC values
  // int m = 12 * n_steps * 2;                // number of rows
  int n = 12 * n_steps * 2;  // number of columns

  // Get the CC indices.
  icc = (int *)malloc(ncc * sizeof(int));
  ccc = (int *)malloc((n + 1) * sizeof(int));
  st_to_cc_index(nst, r_P, c_P, ncc, n, icc, ccc);

  // Get the CC values.
  acc = st_to_cc_values(nst, r_P, c_P, v_P, ncc, n, icc, ccc);

  // Assign values to the csc object
  P = (csc *)c_malloc(sizeof(csc));
  P->m = 12 * n_steps * 2;
  P->n = 12 * n_steps * 2;
  P->nz = -1;
  P->nzmax = ncc;
  P->x = acc;
  P->i = icc;
  P->p = ccc;

  // Free memory
  delete[] r_P;
  delete[] c_P;
  delete[] v_P;

  // Q is already created filled with zeros
  std::fill_n(Q, size_nz_Q, 0.0);

  // char t_char[1] = {'P'};
  // my_print_csc_matrix(P, t_char);

  return 0;
}

/*
Update the M, N, L and K constraint matrices depending on what happened
*/
int MPCSolver::update_matrices(MatrixN fsteps) {
  /* M need to be updated between each iteration:
   - lever_arms changes since the robot moves
   - I_inv changes if the reference velocity vector is modified
   - footholds need to be enabled/disabled depending on the contact sequence */
  update_ML(fsteps);

  /* N need to be updated between each iteration:
   - X0 changes since the robot moves
   - Xk* changes since X0 is not the same */
  update_NK();

  // L matrix is constant
  // K matrix is constant

  return 0;
}

/*
Update the M and L constaint matrices depending on the current state of the gait

*/
int MPCSolver::update_ML(MatrixN fsteps) {
	int n_steps = params->get_N_steps() * params->N_periods;

  int k_cum = 0;
  // Iterate over all phases of the gait
  for (int j = 0; j < gait.rows(); j++) {
    for (int k = k_cum; k < (k_cum + 1); k++) {
      // Get inverse of the inertia matrix for time step k
      double c = cos(xref(5, k));
      double s = sin(xref(5, k));
      Matrix3 R;
      R << c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0;
      Matrix3 R_gI = R.transpose() * gI * R;
      Matrix3 I_inv = R_gI.inverse();

      // Get skew-symetric matrix for each foothold
      footholds_tmp = fsteps.row(j);  // block(j, 1, 1, 12);
      Eigen::Map<MatrixN> footholds_bis(footholds_tmp.data(), 3, 4);

      lever_arms = footholds_bis - (xref.block(0, k, 3, 1) + offset_CoM).replicate<1, 4>();
      for (int i = 0; i < 4; i++) {
        B.block(9, 3 * i, 3, 3) = params->dt_mpc * (I_inv * getSkew(lever_arms.col(i)));
      }

      // Replace the coefficient directly in ML.data
      int i_iter = 24 * 4 * k;
      for (int i = 0; i < 12 * 4; i++) {
        ML->x[i_update_B[i] + i_iter] = B(i_x_B[i], i_y_B[i]);
      }
    }

    k_cum++;
  }

  // Construct the activation/desactivation matrix based on the current gait
  construct_S();

  // Update lines to enable/disable forces
  int i_start = 30 * n_steps - 18;
  for (int k = 0; k < 12 * n_steps; k++) {
    ML->x[i_off(k, 0) + i_start] = S_gait(k, 0);
  }

  return 0;
}

/*
Update the N and K matrices involved in the MPC constraint equations M.X = N and L.X <= K
*/
int MPCSolver::update_NK() {
	int n_steps = params->get_N_steps() * params->N_periods;

  // Matrix g is already created and not changed

  // Reset NK
  NK_up = VectorN::Zero(12 * n_steps * 2 + 20 * n_steps, 1);

  // Fill N matrix with g matrices
  for (int k = 0; k < n_steps; k++) {
    NK_up(12 * k + 8, 0) = -g(8, 0);  // only 8-th coeff is non zero
  }

  // Including - A*X0 in the first row of N
  NK_up.block(0, 0, 12, 1) += A * (-x0);

  // Matrix D is already created and not changed
  // Add third term to matrix N
  Eigen::Map<MatrixN> xref_col((xref.block(0, 1, 12, n_steps)).data(), 12 * n_steps, 1);
  NK_up.block(0, 0, 12 * n_steps, 1) += D * xref_col;

  // Update upper bound c_double array (unrequired since Map is just pointers?)
  VectorN::Map(&v_NK_up[0], NK_up.size()) = NK_up;

  // Update lower bound c_double array
  NK_low.block(0, 0, 12 * n_steps * 2, 1) = NK_up.block(0, 0, 12 * n_steps * 2, 1);
  VectorN::Map(&v_NK_low[0], NK_low.size()) = NK_low;

  return 0;
}


float MPCSolver::retrieve_cost() {
	int n_steps = params->get_N_steps() * params->N_periods;

  // Cost function is x^T P x + q^T x
  // Here P is a diagonal matrix and q = 0
  double cost = 0.0;
  for (int i = 0; i < 2 * 12 * n_steps; i++) {
    cost += (workspce->solution->x)[i] * P->x[i] * (workspce->solution->x)[i];
  }
  return (float)cost;
}

void MPCSolver::init_solver() {
	int n_steps = params->get_N_steps() * params->N_periods;

  // Initial guess for forces (mass evenly supported by all legs in contact)
  warmxf.block(0, 0, 12 * (n_steps - 1), 1) = x.block(12, 0, 12 * (n_steps - 1), 1);
  warmxf.block(12 * n_steps, 0, 12 * (n_steps - 1), 1) = x.block(12 * (n_steps + 1), 0, 12 * (n_steps - 1), 1);
  warmxf.block(12 * (2 * n_steps - 1), 0, 12, 1) = x.block(12 * n_steps, 0, 12, 1);
  VectorN::Map(&v_warmxf[0], warmxf.size()) = warmxf;

  data = (OSQPData *)c_malloc(sizeof(OSQPData));
  data->n = 12 * n_steps * 2;                 // number of variables
  data->m = 12 * n_steps * 2 + 20 * n_steps;  // number of constraints
  data->P = P;             // the upper triangular part of the quadratic cost matrix P in csc format (size n x n)
  data->A = ML;            // linear constraints matrix A in csc format (size m x n)
  data->q = &Q[0];         // dense array for linear part of cost function (size n)
  data->l = &v_NK_low[0];  // dense array for lower bound (size m)
  data->u = &v_NK_up[0];   // dense array for upper bound (size m)
  settings->sigma = (c_float)1e-6;
  settings->eps_abs = (c_float)5e-4;
  settings->eps_rel = (c_float)5e-4;
  settings->eps_prim_inf = (c_float)1e-5;
  settings->eps_dual_inf = (c_float)1e-4;
  settings->alpha = (c_float)1.6;
  settings->adaptive_rho = (c_int)1;
  settings->adaptive_rho_interval = (c_int)200;
  settings->adaptive_rho_tolerance = (c_float)5.0;
  osqp_setup(&workspce, data, settings);
}

/*
Create an initial guess and call the solver to solve the QP problem
*/
void MPCSolver::call_solver() {
	int n_steps = params->get_N_steps() * params->N_periods;

  // Initial guess for forces (mass evenly supported by all legs in contact)
  warmxf.block(0, 0, 12 * (n_steps - 1), 1) = x.block(12, 0, 12 * (n_steps - 1), 1);
  warmxf.block(12 * n_steps, 0, 12 * (n_steps - 1), 1) = x.block(12 * (n_steps + 1), 0, 12 * (n_steps - 1), 1);
  warmxf.block(12 * (2 * n_steps - 1), 0, 12, 1) = x.block(12 * n_steps, 0, 12, 1);
  VectorN::Map(&v_warmxf[0], warmxf.size()) = warmxf;

  osqp_update_A(workspce, &ML->x[0], OSQP_NULL, 0);
  osqp_update_bounds(workspce, &v_NK_low[0], &v_NK_up[0]);

  // Run the solver to solve the QP problem
  osqp_solve(workspce);
}

/*
Extract relevant information from the output of the QP solver
*/
int MPCSolver::retrieve_result() {
  // Retrieve the "contact forces" part of the solution of the QP problem
	int n_steps = params->get_N_steps() * params->N_periods;
  for (int i = 0; i < (n_steps); i++) {
    for (int k = 0; k < 12; k++) {
      x_f_applied(k, i) = (workspce->solution->x)[k + 12 * i] + xref(k, 1 + i);
      x_f_applied(k + 12, i) = (workspce->solution->x)[12 * (n_steps + i) + k];
    }
  }
  for (int k = 0; k < 12; k++) {
    x_next[k] = (workspce->solution->x)[k];
  }

  return 0;
}

/*
Return the latest desired contact forces that have been computed
*/
MatrixN MPCSolver::get_latest_result() { return x_f_applied; }

/*
Return the next predicted state of the base
*/
double *MPCSolver::get_x_next() { return x_next; }
/*
Run one iteration of the whole MPC by calling all the necessary functions (data retrieval,
update of constraint matrices, update of the solver, running the solver, retrieving result)
*/
void MPCSolver::run(const MatrixN &xref_in, const MatrixN &fsteps_in) {
  // Recontruct the gait based on the computed footsteps
  construct_gait(fsteps_in);

  // Retrieve data required for the MPC
  xref = xref_in;
  x0 = xref_in.block(0, 0, 12, 1);

  // Create the constraint and weight matrices used by the QP solver
  // Minimize x^T.P.x + x^T.Q with constraints M.X == N and L.X <= K
  update_matrices(fsteps_in);

  // Create an initial guess and call the solver to solve the QP problem
  call_solver();

  // Extract relevant information from the output of the QP solver
  retrieve_result();
}


/*
Returns the skew matrix of a 3 by 1 column vector
*/
Matrix3 MPCSolver::getSkew(Vector3 v) {
  Matrix3 result;
  result << 0.0, -v(2, 0), v(1, 0), v(2, 0), 0.0, -v(0, 0), -v(1, 0), v(0, 0), 0.0;
  return result;
}

/*
Construct an array of size 12*N that contains information about the contact state of feet.
This matrix is used to enable/disable contact forces in the QP problem.
N is the number of time step in the prediction horizon.
*/
int MPCSolver::construct_S() {
  inv_gait = MatrixN4i::Ones(gait.rows(), 4) - gait;
  for (int i = 0; i < gait.rows(); i++) {
    // S_gait.block(k*12, 0, gait[i, 0]*12, 1) = (1 - (gait.block(i, 1, 1, 4)).transpose()).replicate<gait[i, 0], 1>()
    // not finished;
    for (int b = 0; b < 4; b++) {
      for (int c = 0; c < 3; c++) {
        S_gait(i * 12 + 3 * b + c, 0) = inv_gait(i, b);
      }
    }
  }

  return 0;
}

/*
Reconstruct the gait matrix based on the fsteps matrix since only the last one is received by the MPC
*/

int MPCSolver::construct_gait(MatrixN fsteps_in) {
  for (int k = 0; k < gait.rows(); k++) {
    for (int i = 0; i < 4; i++) {
      if (fsteps_in(k, i * 3) == 0.0) {
        gait(k, i) = 0;
      } else {
        gait(k, i) = 1;
      }
    }
  }
  return 0;
}

MatrixNi MPCSolver::get_gait() { return gait; }
VectorNi MPCSolver::get_Sgait() { return S_gait; }

