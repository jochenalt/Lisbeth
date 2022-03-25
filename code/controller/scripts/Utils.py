import math
import numpy as np
import pinocchio as pin

######################################
# Filters                            #
######################################

class LowpassFilter:
    """
    Args:
        dt (float): time step of the filter [s]
        cutOffFrequency (float): cut frequency of the filter [Hz]
    """
    def __init__(self, dt, cutOffFrequency, initialValue = 0):

        self.alpha = (1/dt) / ((1/dt) + cutOffFrequency)
        self.filtered_x = initialValue

    def compute(self, x):
        self.filtered_x = self.filtered_x * self.alpha + (1.0 - self.alpha) * x

        return self.filtered_x

    def computeN(self, x):
        #self.filtered_x = [self.alpha * e for e in self.filtered_x ] + [(1.0 - self.alpha) * e for e in x] 
        self.filtered_x[:] = self.alpha * self.filtered_x  + (1.0 - self.alpha) * x[:] 
        return self.filtered_x

######################################
# RPY / Quaternion / Rotation matrix #
######################################


def getQuaternion(rpy):
    """Roll Pitch Yaw (3 x 1) to Quaternion (4 x 1)"""

    c = np.cos(rpy*0.5)
    s = np.sin(rpy*0.5)
    cy = c[2, 0]
    sy = s[2, 0]
    cp = c[1, 0]
    sp = s[1, 0]
    cr = c[0, 0]
    sr = s[0, 0]

    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr

    return np.array([[x, y, z, w]]).transpose()


def quaternionToRPY(quat):
    """Quaternion (4 x 0) to Roll Pitch Yaw (3 x 1)"""

    qx = quat[0]
    qy = quat[1]
    qz = quat[2]
    qw = quat[3]

    rotateXa0 = 2.0*(qy*qz + qw*qx)
    rotateXa1 = qw*qw - qx*qx - qy*qy + qz*qz
    rotateX = 0.0

    if (rotateXa0 != 0.0) and (rotateXa1 != 0.0):
        rotateX = np.arctan2(rotateXa0, rotateXa1)

    rotateYa0 = -2.0*(qx*qz - qw*qy)
    rotateY = 0.0
    if (rotateYa0 >= 1.0):
        rotateY = np.pi/2.0
    elif (rotateYa0 <= -1.0):
        rotateY = -np.pi/2.0
    else:
        rotateY = np.arcsin(rotateYa0)

    rotateZa0 = 2.0*(qx*qy + qw*qz)
    rotateZa1 = qw*qw + qx*qx - qy*qy - qz*qz
    rotateZ = 0.0
    if (rotateZa0 != 0.0) and (rotateZa1 != 0.0):
        rotateZ = np.arctan2(rotateZa0, rotateZa1)

    return np.array([[rotateX], [rotateY], [rotateZ]])


def EulerToQuaternion(roll_pitch_yaw):
    """Roll Pitch Yaw to Quaternion"""

    roll, pitch, yaw = roll_pitch_yaw
    sr = math.sin(roll/2.)
    cr = math.cos(roll/2.)
    sp = math.sin(pitch/2.)
    cp = math.cos(pitch/2.)
    sy = math.sin(yaw/2.)
    cy = math.cos(yaw/2.)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return [qx, qy, qz, qw]


def EulerToRotation(roll, pitch, yaw):
    c_roll = math.cos(roll)
    s_roll = math.sin(roll)
    c_pitch = math.cos(pitch)
    s_pitch = math.sin(pitch)
    c_yaw = math.cos(yaw)
    s_yaw = math.sin(yaw)
    Rz_yaw = np.array([
        [c_yaw, -s_yaw, 0],
        [s_yaw,  c_yaw, 0],
        [0, 0, 1]])
    Ry_pitch = np.array([
        [c_pitch, 0, s_pitch],
        [0, 1, 0],
        [-s_pitch, 0, c_pitch]])
    Rx_roll = np.array([
        [1, 0, 0],
        [0, c_roll, -s_roll],
        [0, s_roll,  c_roll]])
    # R = RzRyRx
    return np.dot(Rz_yaw, np.dot(Ry_pitch, Rx_roll))


def getSkew(a):
    """Returns the skew matrix of a 3 by 1 column vector

    Keyword arguments:
    a -- the column vector
    """
    return np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]], dtype=a.dtype)
