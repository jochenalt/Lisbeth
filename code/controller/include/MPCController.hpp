/**
 *
 * Use a quadratic solver to compute the contact forces of the feet
 */

#ifndef MPCC_ONTROLLER_H_INCLUDED
#define MPCC_ONTROLLER_H_INCLUDED

#include "Types.h"
#include <thread>
#include "MPCSolver.hpp"



class MPCController {
 	 public:
		MPCController();
		~MPCController();

		void initialize(Params& params);

		// Send data to the MPC to solve one iteration of MPC
		// ref_states 	Desired state vector for the whole prediction horizon
		// footsteps 	The [x, y, z]^T desired position of each foot for each time step of the horizon
		// gait 			Current gait matrix
		void solve( Matrix12N ref_states, MatrixN12 footsteps, MatrixN4 gait);

		// return the latest result from last MPC run
		Vector12 get_latest_result();

		// return the average time per MPC run in [us]
		int get_avr_time_us() { return time_per_run_us; };

		// returns true of the MPC controller has a new result computed and is ready for the next job
		// the result can be fetched with get_latest_result
		bool is_ready();

 	 private:
		void solver_loop();
		void write_in(Matrix12N& xref, MatrixN12& fsteps);
		bool read_in(Matrix12N& xref, MatrixN12& fsteps);
		void write_out(Matrix242& result);
		bool check_new_result();
		MatrixN read_out();

		std::thread* solver_thread = NULL;	// solver runs in this background thread

		Params* params;  							// Object that stores parameters
		MPCSolver* solver;         				// MPC object used for synchronous solving (not in parallel loop)

		Matrix242 last_available_result;  	// Latest available result of the MPC
		RowVector4 gait_past;               	// Gait status of the previous MPC time step
		RowVector4 gait_next;               	// Gait status of the next MPC time step

		bool thread_is_running = true;
		bool new_mpc_input = false;        	// Flag to indicate new data has been written by main loop for MPC
		bool new_mpc_output = false;       	// Flag to indicate new data has been written by MPC for main loop
		bool new_mpc_output_flag = true;   	// same flag but used for is_ready

	  // buffer for communication between main thread and MPC thread
	  struct {
		  Matrix12N ref_states;              // Desired state vector for the whole prediction horizon
		  MatrixN12 footsteps;           	 // The [x, y, z]^T desired position of each foot for each time step of the horizon
		  Matrix242 contact_force;           // Predicted state and desired contact forces resulting of the MPC
	  } thread_buffer;

	  int time_per_run_us = 0;
};

#endif  // MPCWRAPPER_H_INCLUDED
