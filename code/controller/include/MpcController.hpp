///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief This is the header for MpcWrapper class
///
/// \details Handle the communication between the main control loop and the MPC
///
//////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef MPCWRAPPER_H_INCLUDED
#define MPCWRAPPER_H_INCLUDED

#include "pinocchio/math/rpy.hpp"
#include "Types.h"
#include "MPC.hpp"
#include <thread>
#include <mutex>



class MpcController {
 public:
  ////////////////////////////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Constructor
  ///
  ////////////////////////////////////////////////////////////////////////////////////////////////
  MpcController();

  ////////////////////////////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Initialize with given data
  ///
  /// \param[in] params Object that stores parameters
  ///
  ////////////////////////////////////////////////////////////////////////////////////////////////
  void initialize(Params& params);

  ////////////////////////////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Destructor
  ///
  ////////////////////////////////////////////////////////////////////////////////////////////////
  ~MpcController();  // Empty destructor

  ////////////////////////////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Send data to the MPC to solve one iteration of MPC
  ///
  /// \param[in] k Numero of the current loop
  /// \param[in] xref Desired state vector for the whole prediction horizon
  /// \param[in] fsteps The [x, y, z]^T desired position of each foot for each time step of the horizon
  /// \param[in] gait Current gait matrix
  ///
  ////////////////////////////////////////////////////////////////////////////////////////////////
  void solve( MatrixN xref, MatrixN fsteps, MatrixN gait);

  ////////////////////////////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Return the latest available result of the MPC
  ///
  ////////////////////////////////////////////////////////////////////////////////////////////////
  Matrix242 get_latest_result();

 private:
  ////////////////////////////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Run the MPC in parallel (launched by a thread)
  ///
  ////////////////////////////////////////////////////////////////////////////////////////////////
  void parallel_loop();
  void write_in(MatrixN& xref, MatrixN& fsteps);
  bool read_in(MatrixN& xref, MatrixN& fsteps);
  void write_out(MatrixN& result);
  bool check_new_result();
  MatrixN read_out();

  std::thread* mpc_thread = NULL;

  Params* params_;  // Object that stores parameters
  MPC* mpc_;         // MPC object used for synchronous solving (not in parallel loop)

  Eigen::Matrix<double, 24, 2> last_available_result;  // Latest available result of the MPC
  RowVector4 gait_past;                                  // Gait status of the previous MPC time step
  RowVector4 gait_next;                                  // Gait status of the next MPC time step

  // Mutexes to protect the global variables
  //std::mutex mutexIn;    // From main loop to MPC
  //std::mutex mutexOut;   // From MPC to main loop

  bool thread_is_running = true;
  bool new_mpc_input = false;        // Flag to indicate new data has been written by main loop for MPC
  bool new_mpc_output = false;       // Flag to indicate new data has been written by MPC for main loop

  // establish a buffer for communication between main thread and MPC thread
  struct {
  	MatrixN xref;              // Desired state vector for the whole prediction horizon
  	MatrixN fsteps;            // The [x, y, z]^T desired position of each foot for each time step of the horizon
  	MatrixN result;            // Predicted state and desired contact forces resulting of the MPC
  } thread_buffer;

};

#endif  // MPCWRAPPER_H_INCLUDED
