#include "Params.hpp"

using namespace yaml_control_interface;

Params::Params()
    : interface("")
    , SIMULATION(false)
    , dt_wbc(0.0)
    , N_gait(0)
    , envID(0)
    , velID(0)
    , dt_mpc(0.0)
    , T_gait(0.0)
    , T_mpc(0.0)
    , N_SIMULATION(0)
    , use_flat_plane(false)
    , predefined_vel(false)
    , enable_pyb_GUI(false)
	, q_init(12, 0.0)  // Fill with zeros, will be filled with values later
	, N_periods(0)
{
    initialize(CONFIG_YAML);
}

void Params::initialize(const std::string& file_path)
{
    // Load YAML file
    assert_file_exists(file_path);
    YAML::Node param = YAML::LoadFile(file_path);

    // Check if YAML node is detected and retrieve it
    assert_yaml_parsing(param, file_path, "robot");
    const YAML::Node& robot_node = param["robot"];

    // Retrieve robot parameters
    assert_yaml_parsing(robot_node, "robot", "interface");
    interface = robot_node["interface"].as<std::string>();

    assert_yaml_parsing(robot_node, "robot", "SIMULATION");
    SIMULATION = robot_node["SIMULATION"].as<bool>();

    assert_yaml_parsing(robot_node, "robot", "dt_wbc");
    dt_wbc = robot_node["dt_wbc"].as<double>();

    assert_yaml_parsing(robot_node, "robot", "N_gait");
    N_gait = robot_node["N_gait"].as<int>();

    assert_yaml_parsing(robot_node, "robot", "envID");
    envID = robot_node["envID"].as<int>();

    assert_yaml_parsing(robot_node, "robot", "q_init");
    q_init = robot_node["q_init"].as<std::vector<double> >();

    assert_yaml_parsing(robot_node, "robot", "velID");
    velID = robot_node["velID"].as<int>();

    assert_yaml_parsing(robot_node, "robot", "dt_mpc");
    dt_mpc = robot_node["dt_mpc"].as<double>();

    assert_yaml_parsing(robot_node, "robot", "T_gait");
    T_gait = robot_node["T_gait"].as<double>();

    assert_yaml_parsing(robot_node, "robot", "T_mpc");
    T_mpc = robot_node["T_mpc"].as<double>();

    assert_yaml_parsing(robot_node, "robot", "N_SIMULATION");
    N_SIMULATION = robot_node["N_SIMULATION"].as<int>();

    assert_yaml_parsing(robot_node, "robot", "use_flat_plane");
    use_flat_plane = robot_node["use_flat_plane"].as<bool>();

    assert_yaml_parsing(robot_node, "robot", "predefined_vel");
    predefined_vel = robot_node["predefined_vel"].as<bool>();

    assert_yaml_parsing(robot_node, "robot", "enable_pyb_GUI");
    enable_pyb_GUI = robot_node["enable_pyb_GUI"].as<bool>();

    assert_yaml_parsing(robot_node, "robot", "N_periods");
    N_periods = robot_node["N_periods"].as<int>();

    assert_yaml_parsing(robot_node, "robot", "gait");
    gait_vec = robot_node["gait"].as<std::vector<int> >();
    convert_gait_vec();
}


void Params::convert_gait_vec() {
  if (gait_vec.size() % 5 != 0) {
    throw std::runtime_error(
        "gait matrix in yaml is not in the correct format. It should have five "
        "columns, with the first column containing the number of timestep for "
        "each phase and the four others containing 0 and 1 to describe the "
        "feet status during that phase.");
  }

  // Get the number of lines in the gait matrix
  int N_gait = 0;
  for (uint i = 0; i < gait_vec.size() / 5; i++) {
    N_gait += gait_vec[5 * i];
  }

  // Save period of the gait
  T_gait = N_gait * dt_mpc;

  // Resize gait matrix
  gait = MatrixN::Zero(N_gait * N_periods, 4);

  // Fill gait matrix
  int k = 0;
  for (uint i = 0; i < gait_vec.size() / 5; i++) {
    for (int j = 0; j < gait_vec[5 * i]; j++) {
      gait.row(k) << gait_vec[5 * i + 1], gait_vec[5 * i + 2], gait_vec[5 * i + 3], gait_vec[5 * i + 4];
      k++;
    }
  }

  // Repeat gait for other periods
  for (int i = 1; i < N_periods; i++) {
    gait.block(i * N_gait, 0, N_gait, 4) = gait.block(0, 0, N_gait, 4);
  }
}

