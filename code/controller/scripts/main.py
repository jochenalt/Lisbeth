# coding: utf8

import os
import sys
import time
import pybullet as pyb


import threading
from Controller import Controller
import numpy as np
import argparse
import libcontroller_core as core
import RemoteControl


params = core.Params()  # Object that holds all controller parameters

if params.SIMULATION:
    from PyBulletSimulator import PyBulletSimulator
else:
    import HardwareInterface


key_pressed = False


def get_input():
    global key_pressed
    keystrk = input('Put the robot on the floor and press Enter \n')
    # thread doesn't continue until key is pressed
    key_pressed = True


def put_on_the_floor(device, q_init):
    """Make the robot go to the default initial position and wait for the user
    to press the Enter key to start the main control loop

    Args:
        device (robot wrapper): a wrapper to communicate with the robot
        q_init (array): the default position of the robot
    """
    global key_pressed
    key_pressed = False
    Kp_pos = 3.
    Kd_pos = 0.01
    imax = 3.0
    pos = np.zeros(device.nb_motors)
    for motor in range(device.nb_motors):
        pos[motor] = q_init[device.motorToUrdf[motor]] * device.gearRatioSigned[motor]
    i = threading.Thread(target=get_input)
    i.start()
    while not key_pressed:
        device.UpdateMeasurment()
        for motor in range(device.nb_motors):
            ref = Kp_pos*(pos[motor] - device.hardware.GetMotor(motor).GetPosition() -
                          Kd_pos*device.hardware.GetMotor(motor).GetVelocity())
            ref = min(imax, max(-imax, ref))
            device.hardware.GetMotor(motor).SetCurrentReference(ref)
        device.SendCommand(WaitEndOfCycle=True)


def clone_movements(name_interface_clone, q_init, cloneP, cloneD, cloneQdes, cloneDQdes, cloneRunning, cloneResult):

    print("-- Launching clone interface --")

    print(name_interface_clone, params.dt_wbc)
    clone = HardwareInterface(name_interface_clone, dt=params.dt_wbc)
    clone.Init(calibrateEncoders=True, q_init=q_init)

    while cloneRunning.value and not clone.hardware.IsTimeout():

        # print(cloneP[:], cloneD[:], cloneQdes[:], cloneDQdes[:], cloneRunning.value, cloneResult.value)
        if cloneResult.value:

            clone.SetDesiredJointPDgains(cloneP[:], cloneD[:])
            clone.SetDesiredJointPosition(cloneQdes[:])
            clone.SetDesiredJointVelocity(cloneDQdes[:])
            clone.SetDesiredJointTorque([0.0] * 12)

            clone.SendCommand(WaitEndOfCycle=True)

            cloneResult.value = False

    return 0


def control_loop(name_interface, name_interface_clone=None, des_vel_analysis=None):
    """Main function that calibrates the robot, get it into a default waiting position then launch
    the main control loop once the user has pressed the Enter key

    Args:
        name_interface (string): name of the interface that is used to communicate with the robot
        name_interface_clone (string): name of the interface that will mimic the movements of the first
        name_controller (string): name of the controller to be used 
    """

    # Check .yaml file for parameters of the controller

    # Enable or disable PyBullet GUI
    enable_pyb_GUI = params.enable_pyb_GUI
    if not params.SIMULATION:
        enable_pyb_GUI = False

    # Time variable to keep track of time
    t = 0.0

    # Default position after calibration 
    # q_init = Vector[12] with angles of each actuators 
    #        = [ Lumbar FL, Hip FL, Thigh FL, Lumbar FR, Hip FR, Thigh FR
    #            Lumbar HL, Hip HL, Thigh HLL, Lumbar HR, Hip HR, Thigh HR

    # position when starting walking
    #q_init = np.array([0.0, 0.7, -1.4, 
    #                   -0.0, 0.7, -1.4, 
    #                   0.0, -0.7, +1.4, 
    #                   -0.0, -0.7, +1.4])
    q_init = np.array(params.q_init.tolist())  # Default position after calibration
    # position when sleeping
    # q_init = np.array([0.0, 1.57, -3.14, 
    #                   -0.0, 1.57, -3.14, 
    #                   0.0, -1.57, +3.14, 
    #                   -0.0, -1.57, +3.14])

    if params.SIMULATION and (des_vel_analysis is not None):
        print("Analysis: %1.1f %1.1f %1.1f" % (des_vel_analysis[0], des_vel_analysis[1], des_vel_analysis[5]))
        acceleration_rate = 0.1  # m/s^2
        steady_state_duration = 3  # s
        N_analysis = int(np.max(np.abs(des_vel_analysis)) / acceleration_rate / params.dt_wbc) + 1
        N_steady = int(steady_state_duration / params.dt_wbc)
        params.N_SIMULATION = N_analysis + N_steady

    # Run a scenario and retrieve data thanks to the logger
    print("START CONTROLLER")
    controller = Controller(params, q_init)
    remoteControl = RemoteControl.RemoteControl(params.dt_wbc, False)
    
    controllerCpp = core.Controller()
    controllerCpp.initialize(params)

    if params.SIMULATION and (des_vel_analysis is not None):
        remoteControl.update_for_analysis(des_vel_analysis, N_analysis, N_steady)



    ####

    if params.SIMULATION:
        device = PyBulletSimulator()
    else:
        device = Solo12(name_interface, dt=params.dt_wbc)


    if name_interface_clone is not None:
        print("PASS")
        from multiprocessing import Process, Array, Value
        cloneP = Array('d', [0] * 12)
        cloneD = Array('d', [0] * 12)
        cloneQdes = Array('d', [0] * 12)
        cloneDQdes = Array('d', [0] * 12)
        cloneRunning = Value('b', True)
        cloneResult = Value('b', True)
        clone = Process(target=clone_movements, args=(name_interface_clone, q_init, cloneP,
                        cloneD, cloneQdes, cloneDQdes, cloneRunning, cloneResult))
        clone.start()
        print(cloneResult.value)

    # Number of motors
    nb_motors = device.nb_motors

    # Initiate communication with the device and calibrate encoders
    if params.SIMULATION:
        device.Init(calibrateEncoders=True, q_init=q_init, envID=params.envID,
                    use_flat_plane=params.use_flat_plane, enable_pyb_GUI=enable_pyb_GUI, dt=params.dt_wbc)

    else:
        device.Init(calibrateEncoders=True, q_init=q_init)

        # Wait for Enter input before starting the control loop
        put_on_the_floor(device, q_init)


    print("Start the motion.")
    # CONTROL LOOP ***************************************************
    t = 0.0
    k = 0
    t_max = (params.N_SIMULATION-2) * params.dt_wbc
            
    while ((not device.hardware.IsTimeout()) and (t < t_max) and (not controller.error)):
        for j in range(30000):
            if (j == -1):
                remoteControl.gp.speedX.value = 0.0
                remoteControl.gp.speedY.value = 0.0
                remoteControl.gp.speedZ.value = 0.12
                remoteControl.gp.bodyX.value = 0.0
                remoteControl.gp.bodyY.value = 0.0
                remoteControl.gp.bodyZ.value = 0.0
                print ("-------- START MOVING -------------")

            # Update sensor data (IMU, encoders, Motion capture)
            device.UpdateMeasurment()


            # get command from remote control
            remoteControl.update_v_ref(k, controller.velID)
    
            # Desired torques
            controller.compute(params, device, remoteControl)
            
            controllerCpp.command_gait(remoteControl.gaitCode)
            controllerCpp.command_speed(remoteControl.v_ref[0,0], remoteControl.v_ref[1,0], 
                                        remoteControl.v_ref[2,0], remoteControl.v_ref[3,0], 
                                        remoteControl.v_ref[4,0], remoteControl.v_ref[5,0]);
            controllerCpp.compute(device.baseLinearAcceleration, device.baseAngularVelocity, device.baseOrientation, # IMU data    
                                    device.q_mes, device.v_mes # joint positions and joint velocities coming from encoders
                                 )
            # Check that the initial position of actuators is not too far from the
            # desired position of actuators to avoid breaking the robot
            #if (t <= 10 * params.dt_wbc):
            #    if np.max(np.abs(controller.result.q_des - device.q_mes)) > 0.2:
            #        print("DIFFERENCE: ", controller.result.q_des - device.q_mes)
            #        print("q_des: ", controller.result.q_des)
            #        print("q_mes: ", device.q_mes)
            #        break
    
            # Set desired quantities for the actuators
            #print ("OLD", controller.result.q_des);
            #print ("NEW", controllerCpp.qdes);
            #if (not np.allclose(controller.result.q_des, controllerCpp.qdes)):
            #    print ("alt.q_des", controller.result.q_des)            
            #    print ("new.q_des", controllerCpp.qdes)            
            #if (not np.allclose(controller.result.v_des, controllerCpp.vdes, rtol=0.01)):
            #    print ("old.v_des", controller.result.v_des)            
            #    print ("new.v_des", controllerCpp.vdes)            
            #if (not np.allclose(controller.result.tau_ff, controllerCpp.tau_ff, rtol=0.01)):
            #    print ("oldv.tau_ff", controller.result.tau_ff)            
            #    print ("newv.tau_ff", controllerCpp.tau_ff)            

            device.SetDesiredJointPDgains(controllerCpp.P, controllerCpp.D)
            device.SetDesiredJointPosition(controllerCpp.qdes)
            device.SetDesiredJointVelocity(controllerCpp.vdes)
            device.SetDesiredJointTorque(controllerCpp.tau_ff.ravel())

            #device.SetDesiredJointPDgains(controller.result.P, controller.result.D)
            #device.SetDesiredJointPosition(controller.result.q_des)
            #device.SetDesiredJointVelocity(controller.result.v_des)
            #device.SetDesiredJointTorque(controller.result.tau_ff.ravel())
    
            # Send command to the robot
            device.SendCommand(WaitEndOfCycle=True)
    
            t += params.dt_wbc  # Increment loop time
            
            
            # Update position of PyBullet camera on the robot position to do as if it was attached to the robot
            if k > 10 and params.enable_pyb_GUI:
                pyb.resetDebugVisualizerCamera(cameraDistance=0.6, cameraYaw=45, cameraPitch=-39.9,
                                           cameraTargetPosition=[device.dummyHeight[0], device.dummyHeight[1], 0.0])
            k += 1
            
        quit()

    # ****************************************************************

    if (t >= t_max):
        finished = True
    else:
        finished = False

    # Stop clone interface running in parallel process
    if not params.SIMULATION and name_interface_clone is not None:
        cloneResult.value = False

    # Stop MPC running in a parallel process
    controller.mpcController.stop_parallel_loop()
    # controller.view.stop()  # Stop viewer

    # DAMPING TO GET ON THE GROUND PROGRESSIVELY *********************
    t = 0.0
    t_max = 2.5
    while ((not device.hardware.IsTimeout()) and (t < t_max)):

        device.UpdateMeasurment()  # Retrieve data from IMU and Motion capture

        # Set desired quantities for the actuators
        device.SetDesiredJointPDgains(np.zeros(12), 0.1 * np.ones(12))
        device.SetDesiredJointPosition(np.zeros(12))
        device.SetDesiredJointVelocity(np.zeros(12))
        device.SetDesiredJointTorque(np.zeros(12))

        # Send command to the robot
        device.SendCommand(WaitEndOfCycle=True)
        if ((device.cpt % 1000) == 0):
            device.Print()

        t += params.dt_wbc

    # FINAL SHUTDOWN *************************************************

    # Whatever happened we send 0 torques to the motors.
    device.SetDesiredJointTorque([0]*nb_motors)
    device.SendCommand(WaitEndOfCycle=True)

    if device.hardware.IsTimeout():
        print("Masterboard timeout detected.")
        print("Either the masterboard has been shut down or there has been a connection issue with the cable/wifi.")
    device.hardware.Stop()  # Shut down the interface between the computer and the master board

    if params.SIMULATION and enable_pyb_GUI:
        # Disconnect the PyBullet server (also close the GUI)
        device.Stop()

    if controller.error:
        if (controller.error_flag == 1):
            print("-- POSITION LIMIT ERROR --")
        elif (controller.error_flag == 2):
            print("-- VELOCITY TOO HIGH ERROR --")
        elif (controller.error_flag == 3):
            print("-- FEEDFORWARD TORQUES TOO HIGH ERROR --")
        print(controller.error_value)

    print("End of script")

    return finished, des_vel_analysis

def main():
    parser = argparse.ArgumentParser(description='Playback trajectory to show the extent of solo12 workspace.')
    parser.add_argument('-i',
                        '--interface',
                        required=True,
                        help='Name of the interface (use ifconfig in a terminal), for instance "enp1s0"')
    parser.add_argument('-c',
                        '--clone',
                        required=False,
                        help='Name of the clone interface that will reproduce the movement of the first one \
                              (use ifconfig in a terminal), for instance "enp1s0"')

    parser.add_argument('-ctrl',
                        '--controller',
                        required=False,
                        help='Name of the controller used, use either walking or static"')

    f, v = control_loop(parser.parse_args().interface, parser.parse_args().clone)
    print(f, v)
    quit()


if __name__ == "__main__":
    main()
