# coding: utf8

import numpy as np
import pinocchio as pin
from ModelLoader import ModelLoader
from Utils import LowpassFilter

class ComplementaryFilter:
    """Simple complementary filter

    Args:
        dt (float): time step of the filter [s]
        fc (float): cut frequency of the filter [Hz]
    """

    def __init__(self, dt, fc):

        self.dt = dt

        y = 1 - np.cos(2*np.pi*fc*dt)
        self.alpha = -y+np.sqrt(y*y+2*y)

        self.x = np.zeros(3)
        self.dx = np.zeros(3)
        self.HP_x = np.zeros(3)
        self.LP_x = np.zeros(3)
        self.filt_x = np.zeros(3)

    def compute(self, x, dx, alpha=None):
        """Run one step of complementary filter

        Args:
            x (N by 1 array): quantity handled by the filter
            dx (N by 1 array): derivative of the quantity
            alpha (float): optional, overwrites the fc of the filter
        """

        # Update alpha value if the user desires it
        if alpha is not None:
            self.alpha = alpha

        # For logging
        self.x = x
        self.dx = dx

        # Process high pass filter
        self.HP_x[:] = self.alpha * (self.HP_x + dx * self.dt)

        # Process low pass filter
        self.LP_x[:] = self.alpha * self.LP_x + (1.0 - self.alpha) * x

        # Add both
        self.filt_x[:] = self.HP_x + self.LP_x

        return self.filt_x


class Estimator:
    """State estimator with a complementary filter

    Args:
        dt (float): Time step of the estimator update
        N_simulation (int): maximum number of iterations of the main control loop
        h_init (float): initial height of the robot base
        perfectEstimator (bool): If we are using a perfect estimator (direct simulator data)
    """

    def __init__(self, dt, N_simulation, h_init=0.22294615, perfectEstimator=False):

        # Sample frequency
        self.dt = dt

        # If the IMU is perfect
        self.perfectEstimator = perfectEstimator

        # Filtering estimated linear velocity
        self.filter_v = LowpassFilter(dt, 500., np.zeros(3))
        # Filtering velocities used for security checks
        self.filter_secu_actuator_v = LowpassFilter(dt, 6.0, np.zeros(12))
        fc = 6.0
        y = 1 - np.cos(2*np.pi*fc*dt)
        self.alpha_secu = -y+np.sqrt(y*y+2*y)
        self.alpha_secu = 1-(dt / ( dt + 1/fc))

        self.filter_xyz_vel = ComplementaryFilter(dt, 3.0) # alpha is not used, but set later on
        self.filter_xyz_pos = ComplementaryFilter(dt, 500.0)

        # IMU data
        self.IMU_lin_acc = np.zeros((3, ))  # Linear acceleration (gravity debiased)
        self.IMU_ang_vel = np.zeros((3, ))  # Angular velocity (gyroscopes)
        self.IMU_ang_pos = np.zeros((4, ))  # Angular position (estimation of IMU)

        # Forward Kinematics data
        self.FK_lin_vel = np.zeros((3, ))  # Linear velocity
        self.FK_h = h_init  # Default base height of the FK
        self.FK_xyz = np.array([0.0, 0.0, self.FK_h])
        self.xyz_mean_feet = np.zeros(3)
        self.filter_xyz_pos.LP_x[2] = self.FK_h

        # Boolean to disable FK and FG near contact switches
        self.close_from_contact = False
        self.feet_status = np.zeros(4)
        self.feet_goals = np.zeros((3, 4))
        self.k_since_contact = np.zeros(4)

        # Load the URDF model to get Pinocchio data and model structures
        ModelLoader.free_flyer = True
        robot = ModelLoader().robot
        self.data = robot.data.copy()  # for velocity estimation (forward kinematics)
        self.model = robot.model.copy()  # for velocity estimation (forward kinematics)
        self.data_for_xyz = robot.data.copy()  # for position estimation (forward geometry)
        self.model_for_xyz = robot.model.copy()  # for position estimation (forward geometry)

        # High pass linear velocity (filtered IMU velocity)
        self.HP_lin_vel = np.zeros((3, ))
        # Low pass linear velocity (filtered FK velocity)
        self.LP_lin_vel = np.zeros((3, ))
        self.o_filt_lin_vel = np.zeros((3, 1))  # Linear velocity (world frame)
        self.filt_lin_vel = np.zeros((3, ))  # Linear velocity (base frame)
        self.filt_lin_pos = np.zeros((3, ))  # Linear position
        self.filt_ang_vel = np.zeros((3, ))  # Angular velocity
        self.filt_ang_pos = np.zeros((4, ))  # Angular position
        self.q_filt = np.zeros((19, 1))
        self.v_filt = np.zeros((18, 1))
        self.v_secu = np.zeros((12, ))

        # Various matrices
        self.q_FK = np.zeros((19, 1))
        self.q_FK[:7, 0] = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        self.v_FK = np.zeros((18, 1))
        self.indexes = [10, 18, 26, 34]  #  Indexes of feet frames
        self.actuators_pos = np.zeros((12, ))
        self.actuators_vel = np.zeros((12, ))

        # Transform between the base frame and the IMU frame
        self._1Mi = pin.SE3(pin.Quaternion(np.array([[0.0, 0.0, 0.0, 1.0]]).T),
                            np.array([0.1163, 0.0, 0.02]))

        # Logging matrices
        self.log_v_truth = np.zeros((3, N_simulation))
        self.log_v_est = np.zeros((3, 4, N_simulation))
        self.log_h_est = np.zeros((4, N_simulation))
        self.log_alpha = np.zeros(N_simulation)
        self.log_HP_lin_vel = np.zeros((3, N_simulation))
        self.log_IMU_lin_vel = np.zeros((3, N_simulation))
        self.log_IMU_lin_acc = np.zeros((3, N_simulation))
        self.log_LP_lin_vel = np.zeros((3, N_simulation))
        self.log_FK_lin_vel = np.zeros((3, N_simulation))
        self.log_o_filt_lin_vel = np.zeros((3, N_simulation))
        self.log_filt_lin_vel = np.zeros((3, N_simulation))
        self.log_filt_lin_vel_bis = np.zeros((3, N_simulation))
        self.rotated_FK = np.zeros((3, N_simulation))
        self.k_log = 0

        self.debug_o_lin_vel = np.zeros((3, 1))

    def get_configurations(self):
        return self.q_filt.reshape((19,)), self.v_filt.reshape((18,))

    def get_data_IMU(self, device):
        """Get data from the IMU (linear acceleration, angular velocity and position)

        Args:
            device (object): Interface with the masterboard or the simulation
        """

        # Linear acceleration of the trunk (base frame)
        self.IMU_lin_acc[:] = device.baseLinearAcceleration

        # Angular velocity of the trunk (base frame)
        self.IMU_ang_vel[:] = device.baseAngularVelocity

        # Angular position of the trunk (local frame)
        self.RPY = self.quaternionToRPY(device.baseOrientation)

        if (self.k_log <= 1):
            self.offset_yaw_IMU = self.RPY[2, 0]
        self.RPY[2] -= self.offset_yaw_IMU  # Remove initial offset of IMU

        self.IMU_ang_pos[:] = self.EulerToQuaternion([self.RPY[0],
                                                      self.RPY[1],
                                                      self.RPY[2]])
        # Above could be commented since IMU_ang_pos yaw is not used anywhere and instead
        # replace by: self.IMU_ang_pos[:] = device.baseOrientation

        return 0

    def get_data_joints(self, device):
        """Get the angular position and velocity of the 12 DoF

        Args:
            device (object): Interface with the masterboard or the simulation
        """

        self.actuators_pos[:] = device.q_mes
        self.actuators_vel[:] = device.v_mes

        return 0

    def get_data_FK(self, feet_status):
        """Get data with forward kinematics and forward geometry
        (linear velocity, angular velocity and position)

        Args:
            feet_status (4x0 numpy array): Current contact state of feet
        """

        # Update estimator FK model
        self.q_FK[7:, 0] = self.actuators_pos  # Position of actuators
        self.v_FK[6:, 0] = self.actuators_vel  # Velocity of actuators
        # Position and orientation of the base remain at 0
        # Linear and angular velocities of the base remain at 0

        # Update model used for the forward kinematics
        self.q_FK[3:7, 0] = np.array([0.0, 0.0, 0.0, 1.0])
        pin.forwardKinematics(self.model, self.data, self.q_FK, self.v_FK)
        # pin.updateFramePlacements(self.model, self.data)

        # Update model used for the forward geometry
        self.q_FK[3:7, 0] = self.IMU_ang_pos[:]
        pin.forwardKinematics(self.model_for_xyz, self.data_for_xyz, self.q_FK)

        # Get estimated velocity from updated model
        cpt = 0
        vel_est = np.zeros((3, ))
        xyz_est = np.zeros((3, ))
        for i in (np.where(feet_status == 1))[0]:  # Consider only feet in contact
            if self.k_since_contact[i] >= 16:  # Security margin after the contact switch

                # Estimated velocity of the base using the considered foot
                vel_estimated_baseframe = self.BaseVelocityFromKinAndIMU(self.indexes[i])

                # Estimated position of the base using the considered foot
                framePlacement = pin.updateFramePlacement(
                    self.model_for_xyz, self.data_for_xyz, self.indexes[i])
                xyz_estimated = -framePlacement.translation

                # Logging
                self.log_v_est[:, i, self.k_log] = vel_estimated_baseframe[0:3, 0]
                self.log_h_est[i, self.k_log] = xyz_estimated[2]

                # Increment counter and add estimated quantities to the storage variables
                cpt += 1
                vel_est += vel_estimated_baseframe[:, 0]  # Linear velocity
                xyz_est += xyz_estimated  # Position

                r_foot = 0.025 # 0.0155  # 31mm of diameter on meshlab
                if i <= 1:
                    vel_est[0] += r_foot * (self.actuators_vel[1+3*i] - self.actuators_vel[2+3*i])
                else:
                    vel_est[0] += r_foot * (self.actuators_vel[1+3*i] + self.actuators_vel[2+3*i])

        # If at least one foot is in contact, we do the average of feet results
        if cpt > 0:
            self.FK_lin_vel = vel_est / cpt
            self.FK_xyz = xyz_est / cpt

        return 0

    def get_xyz_feet(self, feet_status, goals):
        """Get average position of feet in contact with the ground

        Args:
            feet_status (4x0 array): Current contact state of feet
            goals (3x4 array): Target locations of feet on the ground
        """

        cpt = 0
        xyz_feet = np.zeros(3)
        for i in (np.where(feet_status == 1))[0]:  # Consider only feet in contact
            cpt += 1
            xyz_feet += goals[:, i]
        # If at least one foot is in contact, we do the average of feet results
        if cpt > 0:
            self.xyz_mean_feet = xyz_feet / cpt

        return 0

    def run_filter(self, k, gait, device, goals):
        """Run the complementary filter to get the filtered quantities

        Args:
            k (int): Number of inv dynamics iterations since the start of the simulation
            gait (4xN array): Contact state of feet (gait matrix)
            device (object): Interface with the masterboard or the simulation
            goals (3x4 array): Target locations of feet on the ground
        """

        feet_status = gait[0, :].copy()  # Current contact state of feet
        remaining_steps = 1  # Remaining MPC steps for the current gait phase
        while (np.array_equal(feet_status, gait[remaining_steps, :])):
            remaining_steps += 1

        # Update IMU data
        self.get_data_IMU(device)

        # Angular position of the trunk
        self.filt_ang_pos[:] = self.IMU_ang_pos

        # Angular velocity of the trunk
        self.filt_ang_vel[:] = self.IMU_ang_vel

        # Update joints data
        self.get_data_joints(device)

        # Update nb of iterations since contact
        self.k_since_contact += feet_status  # Increment feet in stance phase
        self.k_since_contact *= feet_status  # Reset feet in swing phase

        # Update forward kinematics data
        self.get_data_FK(feet_status)

        # Update forward geometry data
        self.get_xyz_feet(feet_status, goals)

        # Tune alpha depending on the state of the gait (close to contact switch or not)
        a = np.ceil(np.max(self.k_since_contact)/10) - 1
        b = remaining_steps
        n = 1  # Nb of steps of margin around contact switch

        v_max = 1.00
        v_min = 0.97  # Minimum alpha value
        c = ((a + b) - 2 * n) * 0.5
        if (a <= (n-1)) or (b <= n):  # If we are close from contact switch
            self.alpha = v_max  # Only trust IMU data
            self.close_from_contact = True  # Raise flag
        else:
            self.alpha = v_min + (v_max - v_min) * np.abs(c - (a - n)) / c
            #self.alpha = 0.997
            self.close_from_contact = False  # Lower flag

        # Rotation matrix to go from base frame to world frame
        oRb = pin.Quaternion(np.array([self.IMU_ang_pos]).T).toRotationMatrix()

        """self.debug_o_lin_vel += 0.002 * (oRb @ np.array([self.IMU_lin_acc]).T)  # TOREMOVE
        self.filt_lin_vel[:] = (oRb.T @ self.debug_o_lin_vel).ravel()"""

        # Get FK estimated velocity at IMU location (base frame)
        cross_product = self.cross3(self._1Mi.translation.ravel(), self.IMU_ang_vel).ravel()
        i_FK_lin_vel = self.FK_lin_vel[:] + cross_product
        # Get FK estimated velocity at IMU location (world frame)
        oi_FK_lin_vel = (oRb @ np.array([i_FK_lin_vel]).T).ravel()

        # Integration of IMU acc at IMU location (world frame)
        oi_filt_lin_vel = self.filter_xyz_vel.compute(oi_FK_lin_vel,
                                                          (oRb @ np.array([self.IMU_lin_acc]).T).ravel(),
                                                          alpha=self.alpha)
            
        # Filtered estimated velocity at IMU location (base frame)
        i_filt_lin_vel = (oRb.T @ np.array([oi_filt_lin_vel]).T).ravel()

        # Filtered estimated velocity at center base (base frame)
        b_filt_lin_vel = i_filt_lin_vel - cross_product

        # Filtered estimated velocity at center base (world frame)
        ob_filt_lin_vel = (oRb @ np.array([b_filt_lin_vel]).T).ravel()

        # Position of the center of the base from FGeometry and filtered velocity (world frame)
        self.filt_lin_pos[:] = self.filter_xyz_pos.compute(
                self.FK_xyz[:] + self.xyz_mean_feet[:], ob_filt_lin_vel, alpha=np.array([0.995, 0.995, 0.9]))

        # Velocity of the center of the base (base frame)
        self.filt_lin_vel[:] = b_filt_lin_vel 
        # Logging
        self.log_alpha[self.k_log] = self.alpha
        self.feet_status[:] = feet_status  # Save contact status sent to the estimator for logging
        self.feet_goals[:, :] = goals.copy()  # Save feet goals sent to the estimator for logging
        self.log_IMU_lin_acc[:, self.k_log] = self.IMU_lin_acc[:]
        self.log_HP_lin_vel[:, self.k_log] = self.HP_lin_vel[:]
        self.log_LP_lin_vel[:, self.k_log] = self.LP_lin_vel[:]
        self.log_FK_lin_vel[:, self.k_log] = self.FK_lin_vel[:]
        self.log_filt_lin_vel[:, self.k_log] = self.filt_lin_vel[:]
        self.log_o_filt_lin_vel[:, self.k_log] = self.o_filt_lin_vel[:, 0]

        # Output filtered position vector (19 x 1)
        self.q_filt[0:3, 0] = self.filt_lin_pos
        if self.perfectEstimator:  # Base height directly from PyBullet
            self.q_filt[2, 0] = device.dummyPos[2] - 0.0155  # Minus feet radius
        self.q_filt[3:7, 0] = self.filt_ang_pos
        self.q_filt[7:, 0] = self.actuators_pos  # Actuators pos are already directly from PyBullet

        # Output filtered velocity vector (18 x 1)
        if self.perfectEstimator:  # Linear velocities directly from PyBullet
            self.v_filt[0:3, 0] = self.filter_v.compute(device.b_baseVel)
        else:
            self.v_filt[0:3, 0] = self.filter_v.computeN(self.filt_lin_vel)

        self.v_filt[3:6, 0] = self.filt_ang_vel  # Angular velocities are already directly from PyBullet
        self.v_filt[6:, 0] = self.actuators_vel  # Actuators velocities are already directly from PyBullet
        
        ###

        # Update model used for the forward kinematics
        """pin.forwardKinematics(self.model, self.data, self.q_filt, self.v_filt)
        pin.updateFramePlacements(self.model, self.data)

        z_min = 100
        for i in (np.where(feet_status == 1))[0]:  # Consider only feet in contact
            # Estimated position of the base using the considered foot
            framePlacement = pin.updateFramePlacement(self.model, self.data, self.indexes[i])
            z_min = np.min((framePlacement.translation[2], z_min))
        self.q_filt[2, 0] -= z_min"""

        ###

        # Output filtered actuators velocity for security checks
        self.v_secu[:] = self.filter_secu_actuator_v.compute(self.actuators_vel)

        # Increment iteration counter
        self.k_log += 1

        return 0

    def cross3(self, left, right):
        """Numpy is inefficient for this

        Args:
            left (3x0 array): left term of the cross product
            right (3x0 array): right term of the cross product
        """
        return np.array([[left[1] * right[2] - left[2] * right[1]],
                         [left[2] * right[0] - left[0] * right[2]],
                         [left[0] * right[1] - left[1] * right[0]]])

    def BaseVelocityFromKinAndIMU(self, contactFrameId):
        """Estimate the velocity of the base with forward kinematics using a contact point
        that is supposed immobile in world frame

        Args:
            contactFrameId (int): ID of the contact point frame (foot frame)
        """

        frameVelocity = pin.getFrameVelocity(
            self.model, self.data, contactFrameId, pin.ReferenceFrame.LOCAL)
        framePlacement = pin.updateFramePlacement(
            self.model, self.data, contactFrameId)

        # Angular velocity of the base wrt the world in the base frame (Gyroscope)
        _1w01 = self.IMU_ang_vel.reshape((3, 1))
        # Linear velocity of the foot wrt the base in the foot frame
        _Fv1F = frameVelocity.linear
        # Level arm between the base and the foot
        _1F = np.array(framePlacement.translation)
        # Orientation of the foot wrt the base
        _1RF = framePlacement.rotation
        # Linear velocity of the base wrt world in the base frame
        _1v01 = self.cross3(_1F.ravel(), _1w01.ravel()) - \
            (_1RF @ _Fv1F.reshape((3, 1)))

        # IMU and base frames have the same orientation
        # _iv0i = _1v01 + self.cross3(self._1Mi.translation.ravel(), _1w01.ravel())

        return np.array(_1v01)

    def EulerToQuaternion(self, roll_pitch_yaw):
        roll, pitch, yaw = roll_pitch_yaw
        sr = np.sin(roll/2.)
        cr = np.cos(roll/2.)
        sp = np.sin(pitch/2.)
        cp = np.cos(pitch/2.)
        sy = np.sin(yaw/2.)
        cy = np.cos(yaw/2.)
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        return [qx, qy, qz, qw]

    def quaternionToRPY(self, quat):
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

