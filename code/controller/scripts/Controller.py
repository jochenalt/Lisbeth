# coding: utf8

import numpy as np
import Utils
import time
import math

import MPC_Wrapper
import pybullet as pyb
import pinocchio as pin
import libcontroller_core as core
from cmath import nan
import RemoteControl
import Types
import ModelLoader

class Result:
    """Object to store the result of the control loop
    It contains what is sent to the robot (gains, desired positions and velocities,
    feedforward torques)"""

    def __init__(self):

        self.P = 0.0
        self.D = 0.0
        self.q_des = np.zeros(12)
        self.v_des = np.zeros(12)
        self.tau_ff = np.zeros(12)


class dummyHardware:
    """Fake hardware for initialisation purpose"""

    def __init__(self):

        pass

    def imu_data_attitude(self, i):

        return 0.0


class dummyDevice:
    """Fake device for initialisation purpose"""

    def __init__(self):

        self.hardware = dummyHardware()




class Controller:

        
    def init_robot(self, q_init, params):
        ModelLoader.free_flyer = True
        robot = ModelLoader.ModelLoader().robot  
        q = robot.q0.reshape((-1, 1))
        q[7:, 0] = q_init

        # Initialisation of model quantities
        pin.centerOfMass(robot.model, robot.data, q, np.zeros((18, 1)))
        pin.updateFramePlacements(robot.model, robot.data)
        pin.crba(robot.model, robot.data, robot.q0)

        # Initialisation of the position of footsteps
        self.fsteps_init = np.zeros((3, 4))
        indexes = [
           robot.model.getFrameId("FL_FOOT"),
           robot.model.getFrameId("FR_FOOT"),
           robot.model.getFrameId("HL_FOOT"),
           robot.model.getFrameId("HR_FOOT"),
        ]
        for i in range(4):
            self.fsteps_init[:, i] = robot.data.oMf[indexes[i]].translation
            self.fsteps_init[0,i] = self.fsteps_init[0,i] -1

        h_init = 0.0
        for i in range(4):
            h_tmp = (robot.data.oMf[1].translation - robot.data.oMf[indexes[i]].translation)[
                2
            ]
            if h_tmp > h_init:
                h_init = h_tmp
        
        # Assumption that all feet are initially in contact on a flat ground
        self.fsteps_init[2, :] = 0.0

        # Initialisation of the position of shoulders
        shoulders_init = np.zeros((3, 4))
        #indexes = [4, 12, 20, 28]  # Shoulder indexes
        indexes = [
           robot.model.getFrameId("FL_SHOULDER"),
           robot.model.getFrameId("FR_SHOULDER"),
           robot.model.getFrameId("HL_SHOULDER"),
           robot.model.getFrameId("HR_SHOULDER"),
        ]

        for i in range(4):
            shoulders_init[:, i] = robot.data.oMf[indexes[i]].translation
            shoulders_init[0,i] = shoulders_init[0,i] -1


        # Saving data
        params.h_ref = h_init
        params.mass = robot.data.mass[
            0
        ]  # Mass of the whole urdf model (also = to Ycrb[1].mass)
        params.I_mat = (
            robot.data.Ycrb[1].inertia.ravel().tolist()
        )  # Composite rigid body inertia in q_init position
        params.CoM_offset = (robot.data.com[0][:3] - q[0:3, 0]).tolist()
        params.CoM_offset[1] = 0.0

        for i in range(4):
            for j in range(3):
                params.shoulders[3 * i + j] = shoulders_init[j, i]
                params.footsteps_init[3 * i + j] = self.fsteps_init[j, i]
                params.footsteps_under_shoulders[3 * i + j] = self.fsteps_init[
                    j, i
                ]  #  Use initial feet pos as reference

        return robot
    
    def __init__(self, params, q_init):
        """Function that runs a simulation scenario based on a reference velocity profile, an environment and
        various parameters to define the gait

        Args:
            envID (int): identifier of the current environment to be able to handle different scenarios
            velID (int): identifier of the current velocity profile to be able to handle different scenarios
            dt_wbc (float): time step of the whole body control
            dt_mpc (float): time step of the MPC
            k_mpc (int): number of iterations of inverse dynamics for one iteration of the MPC
            t (float): time of the simulation
            T_gait (float): duration of one gait period in seconds
            T_mpc (float): duration of mpc prediction horizon
            N_SIMULATION (int): number of iterations of inverse dynamics during the simulation
            use_flat_plane (bool): to use either a flat ground or a rough ground
            predefined_vel (bool): to use either a predefined velocity profile or a gamepad
            enable_pyb_GUI (bool): to display PyBullet GUI or not
            N_gait (int): number of spare lines in the gait matrix
            isSimulation (bool): if we are in simulation mode
            N_periods (int): 1
            gait:  Initial gait matrix
            
        """

        ########################################################################
        #                        Parameters definition                         #
        ########################################################################

        # Lists to log the duration of 1 iteration of the MPC/TSID
        self.t_list_filter = [0] * int(params.N_SIMULATION)
        self.t_list_planner = [0] * int(params.N_SIMULATION)
        self.t_list_mpc = [0] * int(params.N_SIMULATION)
        self.t_list_wbc = [0] * int(params.N_SIMULATION)
        self.t_list_loop = [0] * int(params.N_SIMULATION)

        self.t_list_InvKin = [0] * int(params.N_SIMULATION)
        self.t_list_QPWBC = [0] * int(params.N_SIMULATION)

        # Init joint torques to correct shape
        self.jointTorques = np.zeros((12, 1))

        # List to store the IDs of debug lines
        self.ID_deb_lines = []

        # Enable/Disable perfect estimator
        perfectEstimator = False
        if not params.SIMULATION:
            perfectEstimator = False  # Cannot use perfect estimator if we are running on real robot

        # Load robot model and data
        # Initialisation of the Gepetto viewer
        """
        ModelLoader.free_flyer = True
        self.robot = ModelLoader.ModelLoader().robot  
        q = self.robot.q0.reshape((-1, 1))
        q[7:, 0] = q_init

        # Initialisation of model quantities
        pin.centerOfMass(self.robot.model, self.robot.data, q, np.zeros((18, 1)))
        pin.updateFramePlacements(self.robot.model, self.robot.data)
        pin.crba(self.robot.model, self.robot.data, self.robot.q0)

        # Initialisation of the position of footsteps
        self.fsteps_init = np.zeros((3, 4))
        indexes = [10, 18, 26, 34]
        for i in range(4):
            self.fsteps_init[:, i] = self.robot.data.oMf[indexes[i]].translation
            self.h_init = (self.robot.data.oMf[1].translation - self.robot.data.oMf[indexes[0]].translation)[2]
            self.fsteps_init[2, :] = 0.0
"""
        self.robot = self.init_robot(q_init, params)
        self.remoteControl= RemoteControl.RemoteControl(params.dt_wbc, False)

        # initialize Cpp state estimator
        self.estimator = core.Estimator()
        self.estimator.initialize(params)

        self.wbcWrapper = core.WbcWrapper()
        self.wbcWrapper.initialize(params)

        self.k_mpc = int(params.dt_mpc / params.dt_wbc)
        self.k = 0
        self.enable_pyb_GUI = params.enable_pyb_GUI

        self.h_ref = params.h_ref
        self.q = np.zeros((19, 1))
        self.q[0:7, 0] = np.array([0.0, 0.0, self.h_ref, 0.0, 0.0, 0.0, 1.0])
        self.q[7:, 0] = q_init
        self.v = np.zeros((18, 1))
        self.b_v = np.zeros((18, 1))
        self.o_v_filt = np.zeros((18, 1))

        self.statePlanner = core.StatePlanner()
        self.statePlanner.initialize(params.dt_mpc, params.T_mpc, self.h_ref)

        self.gait = core.Gait()
        self.gait.initialize(params)
        self.gait.updateGait(True, self.q[0:7, 0:1], Types.GaitType.NoMovement.value)

        self.shoulders = np.zeros((3, 4))
        #self.shoulders = params.shoulders
        # x,y coordinates of shoulders
        #print("shoulders1", self.shoulders)
        #self.shoulders[0, :] = [0.1946, 0.1946, -0.1946, -0.1946]       
        #self.shoulders[1, :] = [0.14695, -0.14695, 0.14695, -0.14695]
        #self.shoulders[2, :] = [0, 0, 0, 0]
        #print("shoulders2", self.shoulders)
        
        #for i in range(4):
        #    for j in range(3):
        #        params.shoulders[i*3+j] = self.shoulders[j,i]
                
        self.footstepPlanner = core.FootstepPlanner()
        self.footstepPlanner.initialize(params, self.gait)

        self.footTrajectoryGenerator = core.FootTrajectoryGenerator()

        self.footTrajectoryGenerator.initialize(params, self.gait)

        # Wrapper that makes the link with the solver that you want to use for the MPC
        # First argument to True to have PA's MPC, to False to have Thomas's MPC
        self.enable_multiprocessing = True
        self.mpc_wrapper = MPC_Wrapper.MPC_Wrapper(params, self.q)


        self.k = 0
        self.velID = params.velID

        self.base_targets = np.zeros(12)
        self.qmes12 = np.zeros((19, 1))
        self.vmes12 = np.zeros((18, 1))

        self.v_ref = np.zeros((18, 1))
        self.h_v = np.zeros((18, 1))
        self.yaw_estim = 0.0
        self.RPY_filt = np.zeros(3)

        self.feet_a_cmd = np.zeros((3, 4))
        self.feet_v_cmd = np.zeros((3, 4))
        self.feet_p_cmd = np.zeros((3, 4))

        self.error = False  # True if something wrong happens in the controller
        self.error_flag = 0
        self.q_security = np.array([np.pi*0.4, np.pi*80/180, np.pi] * 4)

        # Interface with the PD+ on the control board
        self.result = Result()

        # Run the control loop once with a dummy device for initialization
        dDevice = dummyDevice()
        dDevice.q_mes = q_init         # actuators positions [0..12]
        dDevice.v_mes = np.zeros(12)
        dDevice.baseLinearAcceleration = np.zeros(3)
        dDevice.baseAngularVelocity = np.zeros(3)
        dDevice.baseOrientation = np.array([0.0, 0.0, 0.0, 1.0])
        dDevice.dummyPos = np.array([0.0, 0.0, q_init[2]])
        dDevice.b_baseVel = np.zeros(3)
        self.compute(params, dDevice)

        


    def compute(self, params, device):
        """Run one iteration of the main control loop

        Args:
            device (object): Interface with the masterboard or the simulation
        """

        t_start = time.time()

        # Update the reference velocity coming from the gamepad
        self.remoteControl.update_v_ref(self.k, self.velID)

        start = time.clock()
        
        baseHeight = np.array([0.0, 0.0, 0.0, device.dummyPos[2] - 0.0155])
        baseVelocity = device.b_baseVel

        self.estimator.run(self.k, self.gait.getCurrentGait().copy(),self.footTrajectoryGenerator.getFootPosition().copy(),
                           device.baseLinearAcceleration.copy(), device.baseAngularVelocity.copy(), device.baseOrientation.copy(), # data from IMU
                           np.array(device.q_mes), device.v_mes, # data from joints
                           baseHeight.copy(),
                           baseVelocity)

        t_filter = time.time()
        
        # automatically turn on a gait if we start moving
        if (self.gait.getCurrentGaitType() == Types.GaitType.NoMovement.value)  and self.remoteControl.isMoving:
            print ("command received, start moving")
            self.remoteControl.gaitCode = self.gait.getPrevGaitType()
            if self.remoteControl.gaitCode == 0:
               self.remoteControl.gaitCode = Types.GaitType.Trot.value

        # automatically go to static mode if no movement is detected
        is_steady = self.estimator.isSteady()
        if self.gait.isNewPhase and self.gait.getCurrentGaitType() != Types.GaitType.NoMovement.value and is_steady and  not self.remoteControl.isMoving:
            print ("no movement, calm down")
            self.remoteControl.gaitCode = Types.GaitType.NoMovement.value
            

        # Update state vectors of the robot (q and v) + transformation matrices between world and horizontal frames
        oRh, oTh = self.updateState(params)

        # at a new gait cycle we need create the next gait round and start MPC
        startNewGaitCycle = (self.k % self.k_mpc) == 0
        
        self.gait.updateGait(startNewGaitCycle, self.q[0:7, 0:1], self.remoteControl.gaitCode)

        self.remoteControl.gaitCode = 0

        # Compute target footstep based on current and reference velocities
        #o_targetFootstep = self.footstepPlanner.updateFootsteps(self.k % self.k_mpc == 0 and self.k != 0,
        #                                                        int(self.k_mpc - self.k % self.k_mpc),
        #                                                        self.q[0:7, 0:1],
        #                                                        self.h_v[0:6, 0:1].copy(),
        #                                                        self.v_ref[0:6, 0])

        o_targetFootstep = self.footstepPlanner.updateFootsteps(self.k % self.k_mpc == 0 and self.k != 0,
                                                                int(self.k_mpc - self.k % self.k_mpc),
                                                                self.q[0:18,0:1].copy(),
                                                                self.h_v[0:6, 0:1].copy(),
                                                                self.v_ref[0:6,0].copy())

        # Update pos, vel and acc references for feet
        self.footTrajectoryGenerator.update(self.k, o_targetFootstep)

        # Run state planner (outputs the reference trajectory of the base)
        self.statePlanner.computeReferenceStates(self.q[0:7, 0:1], self.h_v[0:6, 0:1].copy(),
                                                 self.v_ref[0:6, 0:1], 0.0)

        # Result can be retrieved with self.statePlanner.getReferenceStates()
        xref = self.statePlanner.getReferenceStates()
        fsteps = self.footstepPlanner.getFootsteps()
        cgait = self.gait.getCurrentGait()

        t_planner = time.time()

        # Solve MPC problem once every k_mpc iterations of the main loop
        if startNewGaitCycle:
            try:
                self.mpc_wrapper.solve(self.k, xref, fsteps, cgait)
            except ValueError:
                print("MPC Problem")

        # Retrieve reference contact forces in horizontal frame
        self.x_f_mpc = self.mpc_wrapper.get_latest_result()

        t_mpc = time.time()

        # Target state for the whole body control
        self.x_f_wbc = (self.x_f_mpc[:, 0]).copy()
        if not self.gait.getIsStatic():
            self.x_f_wbc[0] = params.dt_wbc * xref[6, 1]
            self.x_f_wbc[1] = params.dt_wbc * xref[7, 1]
            self.x_f_wbc[2] = params.h_ref
            self.x_f_wbc[3] = 0.0
            self.x_f_wbc[4] = 0.0
            self.x_f_wbc[5] = params.dt_wbc * xref[11, 1]
        else:  # Sort of position control to avoid slow drift
            self.x_f_wbc[0:3] = np.zeros((3)) # define base xyz=(0,0,0),   should come from footstepplanner
            self.x_f_wbc[3:6] = np.zeros((3)) # define base RPY = (0,0,0), should come from footstepplanner
        self.x_f_wbc[6:12] = xref[6:, 1]

        # Whole Body Control
        # If nothing wrong happened yet in the WBC controller
        if (not self.error) and (not self.remoteControl.stop):

            self.q_wbc = np.zeros((19, 1))
            self.q_wbc[2, 0] = self.h_ref  # at position (0.0, 0.0, h_ref)
            self.q_wbc[6, 0] = 1.0  # with orientation (0.0, 0.0, 0.0)
            self.q_wbc[7:, 0] = self.wbcWrapper.qdes[:]  # with reference angular positions of previous loop

            # Get velocity in base frame for Pinocchio (not current base frame but desired base frame)
            self.b_v = self.v.copy()
            self.b_v[:6, 0] = self.v_ref[:6, 0]  # Base at reference velocity (TODO: add hRb once v_ref is considered in base frame)
            self.b_v[6:, 0] = self.wbcWrapper.vdes[:]  # with reference angular velocities of previous loop

            # Feet command acceleration in base frame
            self.feet_a_cmd = oRh.transpose() @ self.footTrajectoryGenerator.getFootAcceleration() \
                - np.cross(np.tile(self.v_ref[3:6, 0:1], (1, 4)), np.cross(np.tile(self.v_ref[3:6, 0:1], (1, 4)), self.feet_p_cmd, axis=0), axis=0) \
                - 2 * np.cross(np.tile(self.v_ref[3:6, 0:1], (1, 4)), self.feet_v_cmd, axis=0)

            # Feet command velocity in base frame
            self.feet_v_cmd = oRh.transpose() @ self.footTrajectoryGenerator.getFootVelocity()
            self.feet_v_cmd = self.feet_v_cmd - self.v_ref[0:3, 0:1] - np.cross(np.tile(self.v_ref[3:6, 0:1], (1, 4)), self.feet_p_cmd, axis=0)

            # Feet command position in base frame
            self.feet_p_cmd = oRh.transpose() @ (self.footTrajectoryGenerator.getFootPosition()
                                                 - np.array([[0.0], [0.0], [self.h_ref]]) - oTh)

            # Run InvKin + WBC QP
            self.wbcWrapper.compute(self.q_wbc, self.b_v,
                                    self.x_f_wbc[12:], np.array([cgait[0, :]]),
                                    self.feet_p_cmd,
                                    self.feet_v_cmd,
                                    self.feet_a_cmd,
                                    self.base_targets)
                                    
            # Quantities sent to the control board
            self.result.P = 3.0 * np.ones(12)
            self.result.D = 0.2 * np.ones(12)
            self.result.q_des[:] = self.wbcWrapper.qdes[:]
            self.result.v_des[:] = self.wbcWrapper.vdes[:]
            self.result.tau_ff[:] = 0.8 * self.wbcWrapper.tau_ff
            
        t_wbc = time.time()

        # Security check
        self.security_check()

        # Update PyBullet camera
        self.pyb_camera(device, 0.0)  # to have yaw update in simu: Utils.quaternionToRPY(self.estimator.q_filt[3:7, 0])[2, 0]

        # Increment loop counter
        self.k += 1

        return 0.0

    def pyb_camera(self, device, yaw):

        # Update position of PyBullet camera on the robot position to do as if it was attached to the robot
        if self.k > 10 and self.enable_pyb_GUI:
            # pyb.resetDebugVisualizerCamera(cameraDistance=0.8, cameraYaw=45, cameraPitch=-30,
            #                                cameraTargetPosition=[1.0, 0.3, 0.25])
            pyb.resetDebugVisualizerCamera(cameraDistance=0.6, cameraYaw=45, cameraPitch=-39.9,
                                           cameraTargetPosition=[device.dummyHeight[0], device.dummyHeight[1], 0.0])

    def security_check(self):
        cpp_q_filt = np.transpose(np.array(self.estimator.getQEstimate())[np.newaxis])
        
        if (self.error_flag == 0) and (not self.error) and (not self.remoteControl.stop) and self.gait.getCurrentGaitType() != 6:
            if np.any(np.abs(cpp_q_filt[7:, 0]) > self.q_security):
                self.error = True
                self.error_flag = 1
                self.error_value = cpp_q_filt[7:, 0] * 180 / 3.1415

            if np.any(np.abs(self.estimator.getVSecurity()) > 50):
                print ("v_secu", self.estimator.getVSecurity())
            if np.any(np.abs(self.estimator.getVSecurity()) > 100):
                self.error = True
                self.error_flag = 2
                self.error_value = self.estimator.getVSecurity()
                
            # @JA security level was 8 formerly
            if np.any(np.abs(self.wbcWrapper.tau_ff) > 8):
                print ("tau_ff", self.wbcWrapper.tau_ff)
            if np.any(np.abs(self.wbcWrapper.tau_ff) > 22):
                self.error = True
                self.error_flag = 3
                self.error_value = self.wbcWrapper.tau_ff

        # If something wrong happened in TSID controller we stick to a security controller
        if self.error or self.remoteControl.stop:

            # Quantities sent to the control board
            self.result.P = np.zeros(12)
            self.result.D = 0.1 * np.ones(12)
            self.result.q_des[:] = np.zeros(12)
            self.result.v_des[:] = np.zeros(12)
            self.result.tau_ff[:] = np.zeros(12)



    def updateState(self, params):

        # Update reference velocity vector
        self.v_ref[0:3, 0] = self.remoteControl.v_ref[0:3, 0]  # TODO: remoteControl velocity given in base frame and not
        self.v_ref[3:6, 0] = self.remoteControl.v_ref[3:6, 0]  # in horizontal frame (case of non flat ground)
        self.v_ref[6:, 0] = 0.0

        # Update position and velocity state vectors
        if not self.gait.getIsStatic():
            # Integration to get evolution of perfect x, y and yaw
            Ryaw = np.array([[math.cos(self.yaw_estim), -math.sin(self.yaw_estim)],
                             [math.sin(self.yaw_estim), math.cos(self.yaw_estim)]])

            self.q[0:2, 0:1] = self.q[0:2, 0:1] + Ryaw @ self.v_ref[0:2, 0:1] * params.dt_wbc

            # Mix perfect x and y with height measurement
            cpp_q_filt = np.transpose(np.array(self.estimator.getQEstimate())[np.newaxis])

            self.q[2, 0] = cpp_q_filt[2, 0]

            # Mix perfect yaw with pitch and roll measurements
            self.yaw_estim += self.v_ref[5, 0:1] * params.dt_wbc
            self.q[3:7, 0] = Utils.EulerToQuaternion([self.estimator.getImuRPY()[0], self.estimator.getImuRPY()[1], self.yaw_estim])
            cpp_RPY = np.transpose(np.array(self.estimator.getImuRPY())[np.newaxis])

            # Actuators measurements
            self.q[7:, 0] = cpp_q_filt[7:, 0]

            # Velocities are the one estimated by the estimator
            cpp_v_filt = np.transpose(np.array(self.estimator.getVEstimate())[np.newaxis])
            self.v = cpp_v_filt.copy()

            hRb = Utils.EulerToRotation(self.estimator.getImuRPY()[0], self.estimator.getImuRPY()[1], 0.0)
            self.h_v[0:3, 0:1] = hRb @ cpp_v_filt[0:3, 0:1]
            self.h_v[3:6, 0:1] = hRb @ cpp_v_filt[3:6, 0:1]

            # self.v[:6, 0] = self.remoteControl.v_ref[:6, 0]
        else:
            quat = np.array([[0.0, 0.0, 0.0, 1.0]]).transpose()
            hRb = np.eye(3)
            pass
            # TODO: Adapt static mode to new version of the code

        # Transformation matrices between world and horizontal frames
        oRh = np.eye(3)
        c = math.cos(self.yaw_estim)
        s = math.sin(self.yaw_estim)
        oRh[0:2, 0:2] = np.array([[c, -s], [s, c]])
        oTh = np.array([[self.q[0, 0]], [self.q[1, 0]], [0.0]])

        return oRh, oTh
