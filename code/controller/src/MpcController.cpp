#include "MpcController.hpp"

#include <chrono>
#include <thread>
#include <mutex>


void MpcController::write_in(MatrixN& xref, MatrixN& fsteps) {
  // const std::lock_guard<std::mutex> lockIn(mutexIn);
  thread_buffer.xref = xref;
  thread_buffer.fsteps = fsteps;

  new_mpc_input = true;  // New data is available, set this flag last
}

bool MpcController::read_in(MatrixN& xref, MatrixN& fsteps) {
  if (new_mpc_input) {
    xref = thread_buffer.xref;
    fsteps = thread_buffer.fsteps;
    new_mpc_input = false;	// reset this flag last
    return true;
  }
  return false;
}

void MpcController::write_out(MatrixN& result) {
	thread_buffer.result = result;
  new_mpc_output = true;  // New data is available
}

bool MpcController::check_new_result() {
  if (new_mpc_output) {
    new_mpc_output = false;
    return true;
  }
  return false;
}

MatrixN MpcController::read_out() {
  //const std::lock_guard<std::mutex> lockOut(mutexOut);
  return thread_buffer.result;
}

void MpcController::parallel_loop() {
  MatrixN xref;
  MatrixN fsteps;
  MatrixN result;
  while (thread_is_running) {

    // Checking if new data is available to trigger the asynchronous MPC
    if (read_in(xref, fsteps)) {
      // std::cout << "NEW DATA AVAILABLE, LAUNCHING MPC" << std::endl;

      // Run the asynchronous MPC with the data that as been retrieved
      mpc_->run(xref, fsteps);

      // Store the result (predicted state + desired forces)
      // MPC::get_latest_result() returns a matrix of size 24 x N and we want to
      // retrieve only the 2 first columns i.e. dataOut.block(0, 0, 24, 2)
      result = mpc_->get_latest_result();

      // publish results
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
  thread_buffer.xref = MatrixN::Zero(12, params.gait.rows() + 1);
  thread_buffer.fsteps = MatrixN::Zero(params.gait.rows(), 12);

  // start thread
  mpc_thread = new std::thread(&MpcController::parallel_loop, this);  // spawn new thread that runs MPC in parallel
}

void MpcController::solve(MatrixN xref, MatrixN fsteps, MatrixN gait) {
  // std::cout << "MPC.xref: " << xref << ", fsteps" << fsteps << std::endl;

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
	// std::cout << "MPC.get_latest_result (new): " << std::endl << last_available_result.block<12,1>(12,0).transpose() << std::endl;
  } else {
	// std::cout << "MPC.get_latest_result (old): " << std::endl << last_available_result.block<12,1>(12,0).transpose() << std::endl;
  }
  return last_available_result;
}
