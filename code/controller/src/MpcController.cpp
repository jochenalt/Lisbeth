#include "../include/MpcController.hpp"

#include <chrono>
#include <thread>
#include <mutex>

// Shared global variables
bool thread_is_running = true;
bool new_mpc_input = false;        // Flag to indicate new data has been written by main loop for MPC
bool new_mpc_output = false;       // Flag to indicate new data has been written by MPC for main loop
MatrixN shared_xref;              // Desired state vector for the whole prediction horizon
MatrixN shared_fsteps;            // The [x, y, z]^T desired position of each foot for each time step of the horizon
MatrixN shared_result;            // Predicted state and desired contact forces resulting of the MPC

// Mutexes to protect the global variables
std::mutex mutexIn;    // From main loop to MPC
std::mutex mutexOut;   // From MPC to main loop


void write_in(MatrixN& xref, MatrixN& fsteps) {
  const std::lock_guard<std::mutex> lockIn(mutexIn);
  shared_xref = xref;
  shared_fsteps = fsteps;
  new_mpc_input = true;  // New data is available
}

bool read_in(MatrixN& xref, MatrixN& fsteps) {
  const std::lock_guard<std::mutex> lockIn(mutexIn);
  if (new_mpc_input) {
    xref = shared_xref;
    fsteps = shared_fsteps;
    new_mpc_input = false;
    return true;
  }
  return false;
}

void write_out(MatrixN& result) {
  const std::lock_guard<std::mutex> lockOut(mutexOut);
  shared_result = result;
  new_mpc_output = true;  // New data is available
}

bool check_new_result() {
  const std::lock_guard<std::mutex> lockOut(mutexOut);
  if (new_mpc_output) {
    new_mpc_output = false;
    return true;
  }
  return false;
}

MatrixN read_out() {
  const std::lock_guard<std::mutex> lockOut(mutexOut);
  return shared_result;
}

void MpcController::parallel_loop() {
  MatrixN xref;
  MatrixN fsteps;
  MatrixN result;
  while (thread_is_running) {

    // Checking if new data is available to trigger the asynchronous MPC
    if (read_in(xref, fsteps)) {
      std::cout << "NEW DATA AVAILABLE, LAUNCHING MPC" << std::endl;

      /*std::cout << "Parallel k" << std::endl << k << std::endl;
      std::cout << "Parallel xref" << std::endl << xref << std::endl;
      std::cout << "Parallel fsteps" << std::endl << fsteps << std::endl;*/

      // Run the asynchronous MPC with the data that as been retrieved
      mpc_->run(xref, fsteps);

      // Store the result (predicted state + desired forces) in the shared memory
      // MPC::get_latest_result() returns a matrix of size 24 x N and we want to
      // retrieve only the 2 first columns i.e. dataOut.block(0, 0, 24, 2)
      // std::cout << "NEW RESULT AVAILABLE, WRITING OUT" << std::endl;
      result = mpc_->get_latest_result();
      write_out(result);
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

MpcController::MpcController()
    : mpc_thread(NULL),
	  last_available_result(Eigen::Matrix<double, 24, 2>::Zero()),
      gait_past(RowVector4::Zero()),
      gait_next(RowVector4::Zero())
{
}

MpcController::~MpcController() {
	thread_is_running = false;
	if (mpc_thread != NULL)
		mpc_thread ->join();
}

void MpcController::initialize(Params& params) {

  params_ = &params;
  mpc_ = new MPC(params);

  // Default result for first step
  last_available_result(2, 0) = params.h_ref;
  last_available_result.col(0).tail(12) = (Vector3(0.0, 0.0, 8.0)).replicate<4, 1>();

  // Initialize the shared memory
  shared_xref = MatrixN::Zero(12, params.gait.rows() + 1);
  shared_fsteps = MatrixN::Zero(params.gait.rows(), 12);

  // start thread
  mpc_thread = new std::thread(&MpcController::parallel_loop, this);  // spawn new thread that runs MPC in parallel
}

void MpcController::solve(MatrixN xref, MatrixN fsteps, MatrixN gait) {
  write_in(xref, fsteps);

  // Adaptation if gait has changed
  if (!gait_past.isApprox(gait.row(0)))  // If gait status has changed
  {
    if (gait_next.isApprox(gait.row(0)))  // If we're still doing what was planned the last time MPC was solved
    {
      last_available_result.col(0).tail(12) = last_available_result.col(1).tail(12);
    } else  // Otherwise use a default contact force command till we get the actual result of the MPC for this new
            // sequence
    {
      double F = 9.81 * params_->mass / gait.row(0).sum();
      for (int i = 0; i < 4; i++) {
        last_available_result.block(12 + 3 * i, 0, 3, 1) << 0.0, 0.0, F;
      }
    }
    last_available_result.col(1).tail(12).setZero();
    gait_past = gait.row(0);
  }
  gait_next = gait.row(1);
}

Eigen::Matrix<double, 24, 2> MpcController::get_latest_result() {
  // Retrieve data from parallel process if a new result is available
  if (check_new_result()) {
    last_available_result = read_out().block(0, 0, 24, 2);
  }
  // std::cout << "get_latest_result: " << std::endl << last_available_result.transpose() << std::endl;
  return last_available_result;
}
