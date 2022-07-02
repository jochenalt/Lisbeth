#include "Params.hpp"

using namespace yaml_control_interface;
Params::Params()
    : N_steps (16), // Number of steps in the prediction horizon
      N_gait(40),
      velID(2),

      config_file(""),
      interface(""),
      DEMONSTRATION(false),
      SIMULATION(false),
      LOGGING(false),
      PLOTTING(false),
      envID(0),
      use_flat_plane(false),
      predefined_vel(false),
      N_SIMULATION(0),
      enable_pyb_GUI(false),
      enable_corba_viewer(false),
      enable_multiprocessing(false),
      perfect_estimator(false),

      q_init(12, 0.0),  // Fill with zeros, will be filled with values later
      dt_wbc(0.0),
      dt_mpc(0.0),
      N_periods(0),
      kf_enabled(false),
      Kp_main(3, 0.0),
      Kd_main(3, 0.0),
      Kff_main(0.0),

      gp_alpha_vel(0.0),
      gp_alpha_pos(0.0),

      fc_v_esti(0.0),

      k_feedback(0.0),

      max_height(0.0),
      lock_time(0.0),
      vert_time(0.0),

      osqp_w_states(12, 0.0),  // Fill with zeros, will be filled with values later
      osqp_w_forces(3, 0.0),   // Fill with zeros, will be filled with values later
      osqp_Nz_lim(0.0),

      Kp_flyingfeet(0.0),
      Kd_flyingfeet(0.0),
      Kp_base_position(3, 0.0),     // Fill with zeros, will be filled with values later
      Kd_base_position(3, 0.0),     // Fill with zeros, will be filled with values later
      Kp_base_orientation(3, 0.0),  // Fill with zeros, will be filled with values later
      Kd_base_orientation(3, 0.0),  // Fill with zeros, will be filled with values later

      Q1(0.0),
      Q2(0.0),
      Fz_max(0.0),
      Fz_min(0.0),
      enable_comp_forces(false),

      mass(1.0),           // Mass of the robot
      I_mat(9, 0.0),       // Fill with zeros, will be filled with values later
      CoM_offset(3, 0.0),  // Fill with zeros, will be filled with values later
      h_ref(0.0),
      shoulders(12, 0.0),                 // Fill with zeros, will be filled with values later
      footsteps_init(12, 0.0),            // Fill with zeros, will be filled with values later
      footsteps_under_shoulders(12, 0.0)  // Fill with zeros, will be filled with values later
{
  initialize(WALK_PARAMETERS_YAML);
}

void Params::initialize(const std::string& file_path)
{
   // Load YAML file
   assert_file_exists(file_path);
   YAML::Node param = YAML::LoadFile(file_path);

   // Check if YAML node is detected and retrieve it
   assert_yaml_parsing(param, file_path, "robot");
   const YAML::Node& robot_node = param["robot"];

   // legacy parameters
   assert_yaml_parsing(robot_node, "robot", "velID");
   velID = robot_node["velID"].as<int>();

   assert_yaml_parsing(robot_node, "robot", "N_steps");
   N_steps = robot_node["N_steps"].as<int>();

   // Retrieve robot parameters
   assert_yaml_parsing(robot_node, "robot", "mass");
   mass = robot_node["mass"].as<double>();

  assert_yaml_parsing(robot_node, "robot", "config_file");
  config_file = robot_node["config_file"].as<std::string>();

  assert_yaml_parsing(robot_node, "robot", "interface");
  interface = robot_node["interface"].as<std::string>();

  assert_yaml_parsing(robot_node, "robot", "DEMONSTRATION");
  DEMONSTRATION = robot_node["DEMONSTRATION"].as<bool>();

  assert_yaml_parsing(robot_node, "robot", "SIMULATION");
  SIMULATION = robot_node["SIMULATION"].as<bool>();

  assert_yaml_parsing(robot_node, "robot", "LOGGING");
  LOGGING = robot_node["LOGGING"].as<bool>();

  assert_yaml_parsing(robot_node, "robot", "PLOTTING");
  PLOTTING = robot_node["PLOTTING"].as<bool>();

  assert_yaml_parsing(robot_node, "robot", "dt_wbc");
  dt_wbc = robot_node["dt_wbc"].as<double>();

  assert_yaml_parsing(robot_node, "robot", "envID");
  envID = robot_node["envID"].as<int>();

  assert_yaml_parsing(robot_node, "robot", "q_init");
  q_init = robot_node["q_init"].as<std::vector<double> >();

  assert_yaml_parsing(robot_node, "robot", "dt_mpc");
  dt_mpc = robot_node["dt_mpc"].as<double>();

  assert_yaml_parsing(robot_node, "robot", "N_periods");
  N_periods = robot_node["N_periods"].as<int>();

  assert_yaml_parsing(robot_node, "robot", "N_SIMULATION");
  N_SIMULATION = robot_node["N_SIMULATION"].as<int>();

  assert_yaml_parsing(robot_node, "robot", "use_flat_plane");
  use_flat_plane = robot_node["use_flat_plane"].as<bool>();

  assert_yaml_parsing(robot_node, "robot", "predefined_vel");
  predefined_vel = robot_node["predefined_vel"].as<bool>();

  assert_yaml_parsing(robot_node, "robot", "enable_pyb_GUI");
  enable_pyb_GUI = robot_node["enable_pyb_GUI"].as<bool>();

  assert_yaml_parsing(robot_node, "robot", "enable_corba_viewer");
  enable_corba_viewer = robot_node["enable_corba_viewer"].as<bool>();

  assert_yaml_parsing(robot_node, "robot", "enable_multiprocessing");
  enable_multiprocessing = robot_node["enable_multiprocessing"].as<bool>();

  assert_yaml_parsing(robot_node, "robot", "perfect_estimator");
  perfect_estimator = robot_node["perfect_estimator"].as<bool>();

  assert_yaml_parsing(robot_node, "robot", "Kp_main");
  Kp_main = robot_node["Kp_main"].as<std::vector<double> >();

  assert_yaml_parsing(robot_node, "robot", "Kd_main");
  Kd_main = robot_node["Kd_main"].as<std::vector<double> >();

  assert_yaml_parsing(robot_node, "robot", "Kff_main");
  Kff_main = robot_node["Kff_main"].as<double>();

  assert_yaml_parsing(robot_node, "robot", "gp_alpha_vel");
  gp_alpha_vel = robot_node["gp_alpha_vel"].as<double>();

  assert_yaml_parsing(robot_node, "robot", "gp_alpha_pos");
  gp_alpha_pos = robot_node["gp_alpha_pos"].as<double>();

  assert_yaml_parsing(robot_node, "robot", "fc_v_esti");
  fc_v_esti = robot_node["fc_v_esti"].as<double>();

  assert_yaml_parsing(robot_node, "robot", "k_feedback");
  k_feedback = robot_node["k_feedback"].as<double>();

  assert_yaml_parsing(robot_node, "robot", "max_height");
  max_height = robot_node["max_height"].as<double>();

  assert_yaml_parsing(robot_node, "robot", "lock_time");
  lock_time = robot_node["lock_time"].as<double>();

  assert_yaml_parsing(robot_node, "robot", "vert_time");
  vert_time = robot_node["vert_time"].as<double>();

  assert_yaml_parsing(robot_node, "robot", "osqp_w_states");
  osqp_w_states = robot_node["osqp_w_states"].as<std::vector<double> >();

  assert_yaml_parsing(robot_node, "robot", "osqp_w_forces");
  osqp_w_forces = robot_node["osqp_w_forces"].as<std::vector<double> >();

  assert_yaml_parsing(robot_node, "robot", "osqp_Nz_lim");
  osqp_Nz_lim = robot_node["osqp_Nz_lim"].as<double>();

  assert_yaml_parsing(robot_node, "robot", "Kp_flyingfeet");
  Kp_flyingfeet = robot_node["Kp_flyingfeet"].as<double>();

  assert_yaml_parsing(robot_node, "robot", "Kd_flyingfeet");
  Kd_flyingfeet = robot_node["Kd_flyingfeet"].as<double>();

  assert_yaml_parsing(robot_node, "robot", "Kp_base_position");
  Kp_base_position = robot_node["Kp_base_position"].as<std::vector<double> >();

  assert_yaml_parsing(robot_node, "robot", "Kd_base_position");
  Kd_base_position = robot_node["Kd_base_position"].as<std::vector<double> >();

  assert_yaml_parsing(robot_node, "robot", "Kp_base_orientation");
  Kp_base_orientation = robot_node["Kp_base_orientation"].as<std::vector<double> >();

  assert_yaml_parsing(robot_node, "robot", "Kd_base_orientation");
  Kd_base_orientation = robot_node["Kd_base_orientation"].as<std::vector<double> >();

  assert_yaml_parsing(robot_node, "robot", "Q1");
  Q1 = robot_node["Q1"].as<double>();

  assert_yaml_parsing(robot_node, "robot", "Q2");
  Q2 = robot_node["Q2"].as<double>();

  assert_yaml_parsing(robot_node, "robot", "Fz_max");
  Fz_max = robot_node["Fz_max"].as<double>();

  assert_yaml_parsing(robot_node, "robot", "Fz_min");
  Fz_min = robot_node["Fz_min"].as<double>();

  assert_yaml_parsing(robot_node, "robot", "enable_comp_forces");
  enable_comp_forces = robot_node["enable_comp_forces"].as<bool>();

  assert_yaml_parsing(robot_node, "robot", "enable_multiprocessing_mip");
  enable_multiprocessing_mip = robot_node["enable_multiprocessing_mip"].as<bool>();

  assert_yaml_parsing(robot_node, "robot", "environment_URDF");
  environment_URDF = robot_node["environment_URDF"].as<std::string>();

  assert_yaml_parsing(robot_node, "robot", "environment_heightmap");
  environment_heightmap = robot_node["environment_heightmap"].as<std::string>();

  assert_yaml_parsing(robot_node, "robot", "heightmap_fit_length");
  heightmap_fit_length = robot_node["heightmap_fit_length"].as<double>();

  assert_yaml_parsing(robot_node, "robot", "heightmap_fit_size");
  heightmap_fit_size = robot_node["heightmap_fit_size"].as<int>();

  assert_yaml_parsing(robot_node, "robot", "max_velocity");
  max_velocity = robot_node["max_velocity"].as<std::vector<double> >();

  assert_yaml_parsing(robot_node, "robot", "use_bezier");
  use_bezier = robot_node["use_bezier"].as<bool>();

  assert_yaml_parsing(robot_node, "robot", "bezier_x_margin_max");
  bezier_x_margin_max = robot_node["bezier_x_margin_max"].as<float>();

  assert_yaml_parsing(robot_node, "robot", "bezier_t_margin");
  bezier_t_margin = robot_node["bezier_t_margin"].as<float>();

  assert_yaml_parsing(robot_node, "robot", "bezier_z_margin");
  bezier_z_margin = robot_node["bezier_z_margin"].as<float>();

  assert_yaml_parsing(robot_node, "robot", "bezier_N_sample");
  bezier_N_sample = robot_node["bezier_N_sample"].as<int>();
  assert_yaml_parsing(robot_node, "robot", "bezier_N_sample_ineq");
  bezier_N_sample_ineq = robot_node["bezier_N_sample_ineq"].as<int>();

  assert_yaml_parsing(robot_node, "robot", "bezier_degree");
  bezier_degree = robot_node["bezier_degree"].as<int>();

  if (!SIMULATION) perfect_estimator = false;
}

