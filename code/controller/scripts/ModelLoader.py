import sys
import warnings
from os.path import dirname, exists, join

import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

import Types
pin.switchToNumpyArray()


def readParamsFromSrdf(model, SRDF_PATH, verbose=False, has_rotor_parameters=True, referencePose='half_sitting'):
    if has_rotor_parameters:
        pin.loadRotorParameters(model, SRDF_PATH, verbose)
    model.armature = np.multiply(model.rotorInertia.flat, np.square(model.rotorGearRatio.flat))
    pin.loadReferenceConfigurations(model, SRDF_PATH, verbose)
    q0 = pin.neutral(model)
    if referencePose is not None:
        q0 = model.referenceConfigurations[referencePose].copy()
    return q0


class RobotLoader(object):
    path = ''
    urdf_filename = ''
    srdf_filename = ''
    sdf_filename = ''
    ref_posture = 'half_sitting'
    has_rotor_parameters = False
    free_flyer = False
    verbose = False
    model_path = None

    def __init__(self):
        if self.urdf_filename:
            if self.sdf_filename:
                raise AttributeError("Please use either URDF or SRDF")
            builder = RobotWrapper.BuildFromURDF
        else:
            try:
                builder = RobotWrapper.BuildFromSDF
            except AttributeError:
                raise ImportError("Building SDF models require pinocchio >= 3.0.0")
        self.model_path = Types.RobotModelPath
        self.df_path=Types.URDFFilePath
        self.meshes_path = [Types.RobotModelPath]
        
        self.robot = builder(self.df_path, self.meshes_path ,
                             pin.JointModelFreeFlyer() if self.free_flyer else None)

        if self.srdf_filename:
            self.srdf_path = Types.SRDFFilePath
            self.robot.q0 = readParamsFromSrdf(self.robot.model, self.srdf_path, self.verbose,
                                               self.has_rotor_parameters, self.ref_posture)

            if pin.WITH_HPP_FCL and pin.WITH_HPP_FCL_BINDINGS:
                # Add all collision pairs
                self.robot.collision_model.addAllCollisionPairs()

                # Remove collision pairs per SRDF
                pin.removeCollisionPairs(self.robot.model, self.robot.collision_model, self.srdf_path, False)

                # Recreate collision data since the collision pairs changed
                self.robot.collision_data = self.robot.collision_model.createData()
        else:
            self.srdf_path = None
            self.robot.q0 = pin.neutral(self.robot.model)

        if self.free_flyer:
            self.addFreeFlyerJointLimits()

    def addFreeFlyerJointLimits(self):
        ub = self.robot.model.upperPositionLimit
        ub[:7] = 1
        self.robot.model.upperPositionLimit = ub
        lb = self.robot.model.lowerPositionLimit
        lb[:7] = -1
        self.robot.model.lowerPositionLimit = lb


class ModelLoader(RobotLoader):
    path = 'solo_description'
    urdf_filename = "solo.urdf"
    srdf_filename = "solo.srdf"
    ref_posture = "standing"
    urdf_filename = "solo12.urdf"
    free_flyer = True

