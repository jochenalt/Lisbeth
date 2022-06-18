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

///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// Functions acting on shared memory
///
//////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief Stop the thread running the MPC in parallel
///
////////////////////////////////////////////////////////////////////////////////////////////////
void stop_thread();

////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief Check if the thread running the MPC in parallel should stop
///
////////////////////////////////////////////////////////////////////////////////////////////////
bool check_stop_thread();

////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief Write data to shared memory (from main loop to MPC)
///
/// \param[in] k Numero of the current loop
/// \param[in] xref Desired state vector for the whole prediction horizon
/// \param[in] fsteps The [x, y, z]^T desired position of each foot for each time step of the horizon
///
////////////////////////////////////////////////////////////////////////////////////////////////
void write_in(int& k, MatrixN& xref, MatrixN& fsteps);

////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief Read data in shared memory (from main loop to MPC)
///
/// \param[in] k Numero of the current loop
/// \param[in] xref Desired state vector for the whole prediction horizon
/// \param[in] fsteps The [x, y, z]^T desired position of each foot for each time step of the horizon
///
////////////////////////////////////////////////////////////////////////////////////////////////
bool read_in(int& k, MatrixN& xref, MatrixN& fsteps);

////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief Write data to shared memory (from MPC to main loop)
///
/// \param[in] result Predicted state and desired contact forces resulting of the MPC. The first 12 rows contains the
/// predicted position/orientation/velocity and the last 12 rows contains the 3D contact forces for each foot. The
/// first column corresponds to the predicted state in dt_mpc seconds if the contact forces are applied. The second
/// column is the predicted state in 2*dt_mpc seconds if the contact forces of the first column are applied during
/// dt_mpc then those of the second column during dt_mpc. And so on...
///
////////////////////////////////////////////////////////////////////////////////////////////////
void write_out(MatrixN& result);

////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief Check if a new MPC result is available
///
////////////////////////////////////////////////////////////////////////////////////////////////
bool check_new_result();

////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief Read data in shared memory (from MPC to main loop)
///
////////////////////////////////////////////////////////////////////////////////////////////////
MatrixN read_out();



class MpcWrapper {
 public:
  ////////////////////////////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Constructor
  ///
  ////////////////////////////////////////////////////////////////////////////////////////////////
  MpcWrapper();

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
  ~MpcWrapper();  // Empty destructor

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
  void solve(int k, MatrixN xref, MatrixN fsteps, MatrixN gait);

  ////////////////////////////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Return the latest available result of the MPC
  ///
  ////////////////////////////////////////////////////////////////////////////////////////////////
  Eigen::Matrix<double, 24, 2> get_latest_result();

 private:
  ////////////////////////////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Run the MPC in parallel (launched by a thread)
  ///
  ////////////////////////////////////////////////////////////////////////////////////////////////
  void parallel_loop();

  std::thread* mpc_thread = NULL;

  Params* params_;  // Object that stores parameters
  MPC* mpc_;         // MPC object used for synchronous solving (not in parallel loop)

  Eigen::Matrix<double, 24, 2> last_available_result;  // Latest available result of the MPC
  RowVector4 gait_past;                                  // Gait status of the previous MPC time step
  RowVector4 gait_next;                                  // Gait status of the next MPC time step
};

#endif  // MPCWRAPPER_H_INCLUDED
