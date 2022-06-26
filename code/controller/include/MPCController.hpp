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
#include <thread>
#include <mutex>
#include "MPCSolver.hpp"



class MPCController {
 public:
  ////////////////////////////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Constructor
  ///
  ////////////////////////////////////////////////////////////////////////////////////////////////
  MPCController();

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
  ~MPCController();  // Empty destructor

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
  void solve( Matrix12N xref, MatrixN12 fsteps, MatrixN gait);

  // return the latest result from last MPC run
  Vector12 get_latest_result();

  // return the average time per MPC run in [us]
  int get_avr_time_us() { return time_per_run_us; };

  // returns true of the MPC controller has a new result computed and is ready for the next job
  // the result can be fetched with get_latest_result
  bool is_ready() {
	  bool save_flag = new_mpc_output_flag;
	  new_mpc_output_flag = false;
  	  return save_flag;
  }
 private:
  void parallel_loop();
  void write_in(Matrix12N& xref, MatrixN12& fsteps);
  bool read_in(Matrix12N& xref, MatrixN12& fsteps);
  void write_out(MatrixN& result);
  bool check_new_result();
  MatrixN read_out();

  std::thread* mpc_thread = NULL;

  Params* params_;  // Object that stores parameters
  MPCSolver* mpc_;         // MPC object used for synchronous solving (not in parallel loop)

  Matrix242 last_available_result;  // Latest available result of the MPC
  RowVector4 gait_past;                                  // Gait status of the previous MPC time step
  RowVector4 gait_next;                                  // Gait status of the next MPC time step

  // Mutexes to protect the global variables
  //std::mutex mutexIn;    // From main loop to MPC
  //std::mutex mutexOut;   // From MPC to main loop

  bool thread_is_running = true;
  bool new_mpc_input = false;        // Flag to indicate new data has been written by main loop for MPC
  bool new_mpc_output = false;       // Flag to indicate new data has been written by MPC for main loop

  bool new_mpc_output_flag = true;        // Flag to indicate new data has been written by main loop for MPC

  // establish a buffer for communication between main thread and MPC thread
  struct {
	  Matrix12N xref;              // Desired state vector for the whole prediction horizon
	  MatrixN12 fsteps;            // The [x, y, z]^T desired position of each foot for each time step of the horizon
  	Matrix242 result;            // Predicted state and desired contact forces resulting of the MPC
  } thread_buffer;

  int time_per_run_us = 0;
};

#endif  // MPCWRAPPER_H_INCLUDED
