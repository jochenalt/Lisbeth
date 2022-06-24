#include "Controller.hpp"

#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/math/rpy.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

Controller::Controller()
    : P(Vector12::Zero()),
      D(Vector12::Zero()),
      q_des(Vector12::Zero()),
      v_des(Vector12::Zero()),
      tau_ff(Vector12::Zero()),
      FF(Vector12::Zero()),
      error(false),
      error_flag(0),
      error_value(Vector12::Zero()),
      k(0),
      k_mpc(0),
      q_filt_mpc(Vector18::Zero()),
      h_v_filt_mpc(Vector6::Zero()),
      vref_filt_mpc(Vector6::Zero()),
      o_targetFootstep(Matrix34::Zero()),
      q_wbc(Vector18::Zero()),
      dq_wbc(Vector18::Zero()),
      base_targets(Vector12::Zero()),
      hRb(Matrix3::Identity()),
      p_ref_(Vector6::Zero()),
      f_mpc(Vector12::Zero()) {}

void Controller::init_robot(Params& params) {

	 // Path to the robot URDF
	  const std::string filename = std::string(URDF_MODEL);

	  // Robot model
	  pinocchio::Model model_;

	  // Build model from urdf (base is not free flyer)
	  pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(), model_, false);

	  // Construct data from model
	  pinocchio::Data data_ = pinocchio::Data(model_);

	  // Update all the quantities of the model
	  VectorN q = VectorN::Zero(model_.nq);
	  q(0,0) = 1.0;            //  x transition as defined in SRDF file
	  q(2,0) = 0.235;          // basic height as defined in SRDF file
	  q(6, 0) = 1.0;  		   // Quaternion (0, 0, 0, 1)
	  q.block<12,1>(7,0) = Vector12(params.q_init.data());

	  // Initialisation of model quantities
	  pinocchio::computeAllTerms(model_, data_, q, VectorN::Zero(model_.nv));
	  pinocchio::centerOfMass(model_, data_, q, VectorN::Zero(model_.nv));
	  pinocchio::updateFramePlacements(model_, data_);
	  pinocchio::crba(model_, data_, q);

	  // Initialisation of the position of footsteps
	  Matrix34 fsteps_init = Matrix34::Zero();
	  int indexes[4] = {static_cast<int>(model_.getFrameId("FL_FOOT")), static_cast<int>(model_.getFrameId("FR_FOOT")),
	                    static_cast<int>(model_.getFrameId("HL_FOOT")), static_cast<int>(model_.getFrameId("HR_FOOT"))};

	  for (int i = 0; i < 4; i++) {
		fsteps_init.col(i) = data_.oMf[indexes[i]].translation();
        fsteps_init(0,i) -= 1.0;
	  }

	  // Get default height
	  double h_init = 0.0;
	  double h_tmp = 0.0;
	  for (int i = 0; i < 4; i++) {
	    h_tmp = (data_.oMf[1].translation() - data_.oMf[indexes[i]].translation())(2, 0);
	    if (h_tmp > h_init) {
	      h_init = h_tmp;
	    }
	  }

	  // Assumption that all feet are initially in contact on a flat ground
	  fsteps_init.row(2).setZero();

	  // Initialisation of the position of shoulders
	  Matrix34 shoulders_init = Matrix34::Zero();
	  int indexes_sh[4] = {4, 12, 20, 28};  //  Shoulder indexes
	  for (int i = 0; i < 4; i++) {
	    shoulders_init.col(i) = data_.oMf[indexes_sh[i]].translation();
	    shoulders_init(0,i) -= 1.0;
	  }

	  // Saving data
	  params_->h_ref = h_init;        // Reference height
	  params_->mass = data_.mass[0];  // Mass

	  // Inertia matrix
	  Vector6 Idata = data_.Ycrb[1].inertia().data();
	  Matrix3 inertia;
	  // Composite rigid body inertia in q_init position
	  inertia << Idata(0, 0), Idata(1, 0), Idata(3, 0), Idata(1, 0), Idata(2, 0), Idata(4, 0), Idata(3, 0), Idata(4, 0),
	      Idata(5, 0);
	  for (int i = 0; i < 3; i++) {
	    for (int j = 0; j < 3; j++) {
	      params_->I_mat[3 * i + j] = inertia(i, j);
	    }
	  }

	  // Offset between center of base and CoM
	  Vector3 CoM = data_.com[0].head(3) - q.head(3);
      CoM(1,0) = 0; // assume we are symmetric

	  params_->CoM_offset[0] = CoM(0, 0);
	  params_->CoM_offset[1] = CoM(1, 0);
	  params_->CoM_offset[2] = CoM(2, 0);



	  for (int i = 0; i < 4; i++) {
	    for (int j = 0; j < 3; j++) {
	      params_->shoulders[3 * i + j] = shoulders_init(j, i);
	      params_->footsteps_init[3 * i + j] = fsteps_init(j, i);
	      params_->footsteps_under_shoulders[3 * i + j] = fsteps_init(j, i);  // Use initial feet pos as reference
	    }
	  }
}


void Controller::initialize(Params& params) {
	  // Params store parameters
	  params_ = &params;

	  // Init robot parameters
	  init_robot(params);

	  // Initialization of the control blocks
	  statePlanner.initialize(params);
	  gait.initialize(params);
      gait.update(true,  GaitType::NoMovement);

	  footstepPlanner.initialize(params, gait);
	  mpcController.initialize(params);
	  footTrajectoryGenerator.initialize(params, gait);
	  estimator.initialize(params);
	  wbcController.initialize(params);

	  filter_mpc_q.initialize(params);
	  filter_mpc_v.initialize(params);
	  filter_mpc_vref.initialize(params);

	  // Other variables
	  k_mpc = static_cast<int>(params.dt_mpc / params.dt_wbc);
	  h_ref_ = params.h_ref;
	  P = (Vector3(params.Kp_main.data())).replicate<4, 1>();
	  D = (Vector3(params.Kd_main.data())).replicate<4, 1>();
	  FF = params.Kff_main * Vector12::Ones();

	  //

}

void Controller::security_check() {
  Vector12 q_security_ = (Vector3(1.2, 2.1, 3.14)).replicate<4, 1>();

  if (((estimator.getQEstimate().tail(12).cwiseAbs()).array() > q_security_.array()).any()) {
    std::cout << "Position limit error "
              << ((estimator.getQEstimate().tail(12).cwiseAbs()).array() > q_security_.array()).transpose()
              << std::endl;
    error_flag = 1;
  } else if (((estimator.getVSecurity().cwiseAbs()).array() > 100.0).any()) {
    std::cout << "Velocity limit error " << ((estimator.getVSecurity().cwiseAbs()).array() > 100.0).transpose()
              << std::endl;
    error_flag = 2;
  } else if (((tau_ff.cwiseAbs()).array() > 8.0).any()) {
    std::cout << "Feedforward limit error " << ((tau_ff.cwiseAbs()).array() > 8.0).transpose() << std::endl;
    error_flag = 3;
  } else {
    error_flag = 0;
  }

  if (error_flag == 0 && !error) {
    if (error_flag != 0) {
      error = true;
      switch (error_flag) {
        case 1:  // Out of position limits
          error_value = estimator.getQEstimate().tail(12) * 180 / 3.1415;
          break;
        case 2:  // Out of velocity limits
          error_value = estimator.getVSecurity();
          break;
        default:  // Out of torques limits
          error_value = tau_ff;
      }
    }
  }

  // If Stop key of the joystick is pressed, set error flag to stop the controller and switch to damping
  //if (!cmd_go) {
  //  error = true;
   // error_flag = -1;
  //}


  // If something wrong happened in the controller we stick to a security controller
  if (error) {
    // Quantities sent to the control board
    P = Vector12::Zero();        // No position control
    D = 0.1 * Vector12::Ones();  // Damping
    q_des = Vector12::Zero();
    v_des = Vector12::Zero();
    FF = Vector12::Zero();  // No feedforward torques
    tau_ff = Vector12::Zero();
  }
}

Matrix34 cross33(Vector3 a, Matrix34 b) {
	  Matrix34 result;
	  for (int i = 0;i<4;i++) {
		  result.col(i) = a.cross(b.col(i));
	  }

	  return result;
}

void Controller::compute(Vector3 const& imuLinearAcceleration,
			 Vector3 const& imuGyroscopse,
			 Vector3 const& imuAttitudeEuler,
			 Vector12 const& jointsPositions,
			 Vector12 const& jointsVelocities)
{
	std::cout << "--- C++ ---" << std::endl;

	estimator.run(gait.getCurrentGait(), footTrajectoryGenerator.getFootPosition(),
		  	  	imuLinearAcceleration,imuGyroscopse, imuAttitudeEuler,
				jointsPositions,jointsVelocities,
				Vector3::Zero(), Vector3::Zero());

	// Update state vectors of the robot (q and v) + transformation matrices between world and horizontal frames

	estimator.updateReferenceState(cmd_v_ref);

	// Quantities go through a 1st order low pass filter with fc = 15 Hz (avoid >25Hz foldback)
	q_filt_mpc = estimator.getQReference();
	q_filt_mpc.head(6) = filter_mpc_q.filter(estimator.getQReference().head(6), true);

	h_v_filt_mpc = filter_mpc_v.filter(estimator.getHV().head(6), false);
	vref_filt_mpc = filter_mpc_vref.filter(estimator.getBaseVelRef().head(6), false);

	// automatically turn on a gait with previously gait when we start moving
	if ((gait.getCurrentGaitType() == GaitType::NoMovement)  and cmd_go and cmd_is_moving) {
		std::cout << "command received, start moving" << std::endl;
		cmd_gait = gait.getPrevGaitType();
		if (cmd_gait == GaitType::NoGait)
			cmd_gait = GaitType::Trot;
	}

	//  automatically go to static mode if no movement is detected
	bool is_steady = estimator.isSteady();
	bool startNewGaitCycle = false;
	if (gait.isNewPhase() && gait.getCurrentGaitType() != GaitType::NoMovement && is_steady && !cmd_go && !cmd_is_moving) {
		std::cout << "no movement, calm down" << std::endl;
	 	cmd_gait = GaitType::NoMovement;
	}

	// at a new gait cycle we need create the next gait round and start MPC
	startNewGaitCycle = (k % k_mpc) == 0;
	gait.update(startNewGaitCycle, cmd_gait);
	cmd_gait = GaitType::NoGait;

	// Compute target footstep based on current and reference velocities
	o_targetFootstep = footstepPlanner.updateFootsteps(
										(k % k_mpc == 0) && (k != 0), static_cast<int>(k_mpc - (k % k_mpc)), estimator.getQReference(),
										estimator.getHVFiltered(), estimator.getBaseVelRef());

	// Update pos, vel and acc references for feet
	footTrajectoryGenerator.update(k, o_targetFootstep);
    //std::cout << "footTrajectoryGenerator.getFootPosition(); " << footTrajectoryGenerator.getFootPosition() << std::endl;
    //std::cout << "footTrajectoryGenerator.getFootVelocity(); " << footTrajectoryGenerator.getFootVelocity() << std::endl;
    //std::cout << "footTrajectoryGenerator.getFootAcceleration(); " << footTrajectoryGenerator.getFootAcceleration() << std::endl;

	// Run state planner (outputs the reference trajectory of the base)
	statePlanner.computeReferenceStates(q_filt_mpc.head(6), h_v_filt_mpc, vref_filt_mpc);

	// Solve MPC problem once every k_mpc iterations of the main loop

	if (k % k_mpc == 0) {
		mpcController.solve(statePlanner.getReferenceStates(), footstepPlanner.getFootsteps(), gait.getCurrentGait());
		f_mpc = mpcController.get_latest_result().block(12, 0, 12, 1);
	}
	// Target state for the whole body control

	if (k % k_mpc == 8) {
		f_mpc = mpcController.get_latest_result().block(12, 0, 12, 1);
		// std::cout << "f_mpc " << f_mpc << std::endl;
	}


	// Whole Body Control
	// If nothing wrong happened yet in the WBC controller
	if (!error && cmd_go) {
	    // Desired position, orientation and velocities of the base
	    base_targets.head(6).setZero();
	    base_targets.block<2,1>(3,0) = statePlanner.getReferenceStates().block<2,1>(3,1);
	    base_targets.block<6,1>(6,0) = vref_filt_mpc;// Velocities (in horizontal frame!)
	    // std::cout << "base_targets" << base_targets << std::endl;

	    // std::cout << "q_filt_mpc" << q_filt_mpc << std::endl;
	    // std::cout << "wbcController.get_qdes()" << wbcController.get_qdes()<< std::endl;

	    Vector3 T = -estimator.getoTh() - Vector3(0.0, 0.0, h_ref_);
	    Matrix3 R = estimator.gethRb() * estimator.getoRh();
	    // std::cout << "R" << R << std::endl;
	    // std::cout << "T" << T.replicate(1,4) << std::endl;

	    Matrix3N feet_a_cmd = R * footTrajectoryGenerator.getFootAcceleration();
	    Matrix3N feet_v_cmd = R * footTrajectoryGenerator.getFootVelocity();
	    Matrix3N feet_p_cmd = R * (footTrajectoryGenerator.getFootPosition()  + T.replicate(1,4));

	    Vector3 v_ref_tmp = estimator.getBaseVelRef().block<3,1>(3,0);
        feet_a_cmd += - cross33(v_ref_tmp,cross33( v_ref_tmp, feet_p_cmd))
        		      - 2* cross33(v_ref_tmp, feet_v_cmd);
	    feet_v_cmd += - estimator.getBaseVelRef().block<3,1>(0,0).replicate(1,4)
	    		      - cross33 (estimator.getBaseVelRef().block<3,1>(3,0), feet_p_cmd);

	    // std::cout << "feet_p_cmd " << footTrajectoryGenerator.getFootPosition() + T.replicate(1,4)<< std::endl;
	    // std::cout << "R*feet_p_cmd " << R*(footTrajectoryGenerator.getFootPosition() + T.replicate(1,4)) << std::endl;

	    // std::cout << "feet_v_cmd " << footTrajectoryGenerator.getFootVelocity() << std::endl;
	    // std::cout << "feet_a_cmd " << footTrajectoryGenerator.getFootAcceleration() << std::endl;

	    // std::cout << "dq_wbc " << dq_wbc << std::endl;
	    // std::cout << "f_mpc " << f_mpc << std::endl;

	    // Update configuration vector for wbc
        q_wbc.block<3,1>(0,0) = Vector3(0,0,h_ref_);
	    q_wbc(3, 0) = q_filt_mpc(3, 0);          // Roll
	    q_wbc(4, 0) = q_filt_mpc(4, 0);          // Pitch
	    q_wbc.tail(12) = wbcController.get_qdes();  // with reference angular positions of previous loop

	    // Update velocity vector for wbc
	    dq_wbc.head(6) = estimator.getVEstimate().head(6);  // Velocities in base frame (not horizontal frame!)
	    dq_wbc.tail(12) = wbcController.get_vdes();            // with reference angular velocities of previous loop

	    // Run InvKin + WBC QP
	    // std::cout << "q_wbc " << q_wbc << std::endl;
	    // std::cout << "dq_wbc " << dq_wbc << std::endl;
	    // std::cout << "f_mpc " << f_mpc << std::endl;

	    wbcController.compute(q_wbc, dq_wbc, f_mpc, gait.getCurrentGait().row(0),
	    					  feet_p_cmd,feet_v_cmd,feet_a_cmd,
	    					  base_targets);

	    // Quantities sent to the control board
	    q_des = wbcController.get_qdes();
	    v_des = wbcController.get_vdes();
	    tau_ff = wbcController.get_tau_ff();
	  }

	  // Security check
	  security_check();

	  // Increment loop counter
	  k++;
}
