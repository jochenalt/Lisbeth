#include <chrono>
#include <thread>
#include <mutex>
#include <Utils.hpp>
#include "MPCController.hpp"


void MPCController::write_in(Matrix12N& ref_states, MatrixN12& footsteps) {
  thread_buffer.ref_states = ref_states;
  thread_buffer.footsteps = footsteps;

  new_mpc_input = true;  // New data is available, set this flag last
}

bool MPCController::read_in(Matrix12N& ref_states, MatrixN12& footsteps) {
  if (new_mpc_input) {
	  ref_states = thread_buffer.ref_states;
	  footsteps = thread_buffer.footsteps;
    new_mpc_input = false;	// reset this flag last
    return true;
  }
  return false;
}

void MPCController::write_out(Matrix242& result) {
   if (new_mpc_output == true)
	   	std::cout << "ERROR:previous MPC result hasn't been fetched yet" << std::endl;
  thread_buffer.contact_force = result;
  new_mpc_output = true;  // New data is available
  new_mpc_output_flag = true;
}

bool MPCController::check_new_result() {
  if (new_mpc_output) {
    new_mpc_output = false;
    return true;
  }
  return false;
}

MatrixN MPCController::read_out() {
  return thread_buffer.contact_force;
}

void MPCController::solver_loop() {
	Matrix12N ref_states;
	MatrixN12 footsteps;
	Matrix242 result;
	while (thread_is_running) {

    // Checking if new data is available to trigger the asynchronous MPC
    if (read_in(ref_states, footsteps)) {
      // Run the asynchronous MPC with the data that as been retrieved
      uint64_t start = get_micros();
      std::cout  << "MPC start";
      solver->run(ref_states, footsteps);
      std::cout  << "MPC end";

      uint64_t time = get_micros() - start;
      time_per_run_us = (time_per_run_us + (int)time)/2;


      // Store the result (predicted state + desired forces)
      // MPC::get_latest_result() returns a matrix of size 24 x N and we want to
      // retrieve only the 2 first columns i.e. dataOut.block(0, 0, 24, 2)
      result = solver->get_latest_result();

      // publish results
      write_out(result);
    } else {
      delay_us(100);
    }
  }
}

MPCController::MPCController()
    : solver_thread(NULL),
		last_available_result(Matrix242::Zero()),
      gait_past(RowVector4::Zero()),
      gait_next(RowVector4::Zero())
{
}

MPCController::~MPCController() {
	thread_is_running = false;
	if (solver_thread != NULL)
		solver_thread ->join();
}

// returns true of the MPC controller has a new result computed and is ready for the next job
// the result can be fetched with get_latest_result
bool MPCController::is_ready() {
	  bool save_flag = new_mpc_output_flag;
	  new_mpc_output_flag = false;
	  return save_flag;
}

void MPCController::initialize(Params& params_in) {

  params = &params_in;
  solver = new MPCSolver(params_in);

  // Default result for first step
  last_available_result(2, 0) = params->h_ref;
  last_available_result.col(0).tail(12) = (Vector3(0.0, 0.0, 8.0)).replicate<4, 1>();

  // Initialize buffer memory memory
  thread_buffer.ref_states = MatrixN::Zero(12, params->get_N_steps()*params->N_periods + 1);
  thread_buffer.footsteps = MatrixN::Zero(params->get_N_steps()*params->N_periods, 12);

  // start thread
  solver_thread = new std::thread(&MPCController::solver_loop, this);  // spawn new thread that runs MPC in parallel
}

void MPCController::solve(Matrix12N xref, MatrixN12 fsteps, MatrixN4 gait) {
	write_in(xref, fsteps);

	// Adaptation if gait has changed
	if (!gait_past.isApprox(gait.row(0))) { // If gait status has changed
		if (gait_next.isApprox(gait.row(0))) { // If we're still doing what was planned the last time MPC was solved
			last_available_result.col(0).tail(12) = last_available_result.col(1).tail(12);
		} else {
			// Otherwise use a default contact force command till we get ( = mass / number of legs )
			// the actual result of the MPC for this new sequence
			double F = 9.81 * params->mass / gait.row(0).sum();
			for (int i = 0; i < 4; i++)
				last_available_result.block(12 + 3 * i, 0, 3, 1) << 0.0, 0.0, F;
		}
		last_available_result.col(1).tail(12).setZero();
		gait_past = gait.row(0);
  }
  gait_next = gait.row(1);
}

Vector12 MPCController::get_latest_result() {
  // Retrieve data from parallel process if a new result is available, otherwise take previous one
  if (check_new_result())
    last_available_result = read_out().block(0, 0, 24, 2);
  return last_available_result.block(12, 0, 12, 1);
}
