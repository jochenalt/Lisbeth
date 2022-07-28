///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief This is the header for Params class
///
/// \details This class retrieves and stores all parameters written in the main .yaml so that the user can easily
/// change their value without digging into the code. It also stores some model parameters whose values depends on what
/// is in the yaml
///
//////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PARAMS_H_INCLUDED
#define PARAMS_H_INCLUDED

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <vector>

#include "Types.h"

class Params {
 public:
  Params();
  ~Params() {}

  void initialize(const std::string& config_file_path);


  // Number of wbc loops per mpc
  int get_k_mpc() { return (int)std::round(dt_mpc / dt_wbc); };

  // number of loops/steps in the prediction horizon
  int get_N_steps() { return N_steps; };

  // call this at the end of the loop
  void inc_k() { k++; };

  // get current wbc loop number
  int get_k() { return k; };

  // loops left until next MPC resp. step change
  int get_k_left_in_gait() { return (get_k_mpc() - (k % get_k_mpc())); };

  // true if this is the cycle with a new MPC round and a new step in the gait
  bool is_new_mpc_cycle() { return (k % get_k_mpc()) == 0; };

  int N_steps; 		// number of steps in the prediction horizon
  int N_gait;			// Number of rows in the gait matrix. Arbitrary value that should be set high enough,so that there is always at least one empty line at the end of the gait matrix
  int velID;			//  Identifier of the reference velocity profile to choose which one will be sent to the robot
  int k=0;					// counting wbc loops
  // See .yaml file for meaning of parameters
  // General parameters
  std::string config_file;      // Name of the yaml file containing hardware information
  std::string interface;        // Name of the communication interface (check with ifconfig)
  bool DEMONSTRATION;           // Enable/disable demonstration functionalities
  bool SIMULATION;              // Enable/disable PyBullet simulation or running on real robot
  bool LOGGING;                 // Enable/disable logging during the experiment
  int envID;                    // Identifier of the environment to choose in which one the simulation will happen
  bool use_flat_plane;          // If True the ground is flat, otherwise it has bumps
  bool predefined_vel;          // If we are using a predefined reference velocity (True) or a joystick (False)
  int N_SIMULATION;             // Number of simulated wbc time steps
  bool enable_pyb_GUI;          // Enable/disable PyBullet GUI
  bool enable_early_mpc_result; // Enable fetching MPC result as soon as it is available
  bool perfect_estimator;       // Enable/disable perfect estimator by using data directly from PyBullet

  // General control parameters
  std::vector<double> q_init;   // Initial articular positions
  double dt_wbc;                // Time step of the whole body control
  double dt_mpc;                // Time step of the model predictive control
  int N_periods;                // Number of gait periods in the MPC prediction horizon
  bool kf_enabled;              // Use complementary filter (False) or kalman filter (True) for the estimator
  std::vector<double> Kp_main;  // Proportional gains for the PD+
  std::vector<double> Kd_main;  // Derivative gains for the PD+
  double Kff_main;              // Feedforward torques multiplier for the PD+

  // Parameters of Joystick
  double gp_alpha_vel;               // Coefficient of the low pass filter applied to gamepad velocity
  double gp_alpha_pos;               // Coefficient of the low pass filter applied to gamepad position

  // Parameters of FootstepPlanner
  double k_feedback;  // Value of the gain for the feedback heuristic

  // Parameters of FootTrajectoryGenerator
  double max_height;  // Apex height of the swinging trajectory [m]
  double lock_time;   // Target lock before the touchdown [s]
  double vert_time;   // Duration during which feet move only along Z when taking off and landing

  // Parameters of MPC with OSQP
  std::vector<double> osqp_w_states;  // Weights for state tracking error
  std::vector<double> osqp_w_forces;  // Weights for force regularisation
  double osqp_Nz_lim;                 // Maximum vertical force that can be applied at contact points

  //  Parameters of InvKin
  double Kp_flyingfeet;                     // Proportional gain for feet position tasks
  double Kd_flyingfeet;                     // Derivative gain for feet position tasks
  std::vector<double> Kp_base_position;     // Proportional gains for the base position task
  std::vector<double> Kd_base_position;     // Derivative gains for the base position task
  std::vector<double> Kp_base_orientation;  // Proportional gains for the base orientation task
  std::vector<double> Kd_base_orientation;  // Derivative gains for the base orientation task
  std::vector<double> w_tasks;  // Tasks weights: [feet/base, vx, vy, vz, roll+wroll, pitch+wpitch, wyaw, contacts]

  // Parameters of WBC QP problem
  double Q1;                // Weights for the "delta articular accelerations" optimization variables
  double Q2;                // Weights for the "delta contact forces" optimization variables
  double Fz_max;            // Maximum vertical contact force [N]
  double Fz_min;            // Minimal vertical contact force [N]
  bool enable_comp_forces;  // Enable the use of compensation forces in the QP problem

  // Parameters of MIP
  std::string environment_URDF;       // URDF path for the 3D environment
  std::string environment_heightmap;  // Path to the heightmap
  double heightmap_fit_length;        // Size of the heightmap around the robot
  int heightmap_fit_size;             // Number of points used in the heightmap QP
  std::vector<double> max_velocity;   // Maximum velocity of the base

  // Not defined in yaml
  double mass;                                    // Mass of the robot
  std::vector<double> I_mat;                      // Inertia matrix
  std::vector<double> CoM_offset;                 // Center of Mass offset
  double h_ref;                                   // Reference height for the base
  std::vector<double> shoulders;                  // Position of shoulders in base frame
  std::vector<double> footsteps_init;             // Initial 3D position of footsteps in base frame
  std::vector<double> footsteps_under_shoulders;  // Positions of footsteps to be "under the shoulder"
};

////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief Check if a parameter exists in a given yaml file (bofore we try retrieving its value)
///
/// \param[in] yaml_node Name of the yaml file
/// \param[in] parent_node_name Name of the parent node
/// \param[in] child_node_name Name of the child node
///
////////////////////////////////////////////////////////////////////////////////////////////////
namespace yaml_control_interface {
#define assert_yaml_parsing(yaml_node, parent_node_name, child_node_name)                              \
  if (!yaml_node[child_node_name]) {                                                                   \
    std::ostringstream oss;                                                                            \
    oss << "Error: Wrong parsing of the YAML file from src file: [" << __FILE__ << "], in function: [" \
        << __FUNCTION__ << "], line: [" << __LINE__ << ". Node [" << child_node_name                   \
        << "] does not exists under the node [" << parent_node_name << "].";                           \
    throw std::runtime_error(oss.str());                                                               \
  }                                                                                                    \
  assert(true)

////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief Check if a file exists (before we try loading it)
///
/// \param[in] filename File path to check
///
////////////////////////////////////////////////////////////////////////////////////////////////
#define assert_file_exists(filename)                                                                        \
  std::ifstream f(filename.c_str());                                                                        \
  if (!f.good()) {                                                                                          \
    std::ostringstream oss;                                                                                 \
    oss << "Error: Problem opening the file [" << filename << "], from src file: [" << __FILE__             \
        << "], in function: [" << __FUNCTION__ << "], line: [" << __LINE__ << ". The file may not exists."; \
    throw std::runtime_error(oss.str());                                                                    \
  }                                                                                                         \
  assert(true)
}  // namespace yaml_control_interface

#endif  // PARAMS_H_INCLUDED
