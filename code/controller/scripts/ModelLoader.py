import sys
import warnings
from os.path import dirname, exists, join

import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

import Types
pin.switchToNumpyArray()


def readParamsFromSrdf(model, SRDF_PATH, verbose, referencePose):
    model.armature = np.multiply(model.rotorInertia.flat, np.square(model.rotorGearRatio.flat))
    pin.loadReferenceConfigurations(model, SRDF_PATH, verbose)
    q0 = pin.neutral(model)
    if referencePose is not None:
        q0 = model.referenceConfigurations[referencePose].copy()
    return q0


class ModelLoader(object):
    urdf_filename = ''
    srdf_filename = ''
    ref_posture = "sleeping"
    free_flyer = True
    verbose = False
    model_path = None

    def __init__(self):
        builder = RobotWrapper.BuildFromURDF
        self.model_path = Types.RobotModelPath
        self.df_path=Types.URDFFilePath
        self.meshes_path = [Types.RobotModelPath]
        self.robot = builder(self.df_path, None ,
                             pin.JointModelFreeFlyer() if self.free_flyer else None)

        if Types.SRDFFilePath:
            print("Types.SRDFFilePath")
            self.srdf_path = Types.SRDFFilePath
            self.robot.q0 = readParamsFromSrdf(self.robot.model, self.srdf_path, self.verbose,
                                               self.ref_posture)

            if pin.WITH_HPP_FCL and pin.WITH_HPP_FCL_BINDINGS:
                # Add all collision pairs
                self.robot.collision_model.addAllCollisionPairs()

                # Remove collision pairs per SRDF
                pin.removeCollisionPairs(self.robot.model, self.robot.collision_model, self.srdf_path, False)

                # Recreate collision data since the collision pairs changed
                self.robot.collision_data = self.robot.collision_model.createData()
        else:
            print("No Types.SRDFFilePath")

            self.srdf_path = None
            #self.robot.q0 = pin.neutral(self.robot.model)

        #if self.free_flyer:
        #    self.addFreeFlyerJointLimits()

    def addFreeFlyerJointLimits(self):
        ub = self.robot.model.upperPositionLimit
        ub[:7] = 1
        self.robot.model.upperPositionLimit = ub
        lb = self.robot.model.lowerPositionLimit
        lb[:7] = -1
        self.robot.model.lowerPositionLimit = lb


