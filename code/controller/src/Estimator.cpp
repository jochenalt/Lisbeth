#include <cmath>
#include <iostream>
#include <cmath>
#include <iostream>
#include <string>

#include "qrw/Estimator.hpp"

using namespace std;


Estimator::Estimator() {
}


void Estimator::initialize(double dT, int N_simulation, double h_init, bool perfectEstimator ) {

	// sample frequency
	this->dt = dT;

	// flag if the IMU is perfect
	this->perfectEstimator = perfectEstimator;

	// Filtering estimated linear velocity
	double fc = 50.0;  //  Cut frequency
	double y = 1 - cos(2*M_PI*fc*dt);
	this->alpha_v = -y+sqrt(y*y+2*y);
	fc = 500;
	this->alpha_v = 1-(dT / ( dT + 1/fc));

	//  Filtering velocities used for security checks
	fc = 6.0;
	y = 1 - cos(2*M_PI*fc*dt);
	this->alpha_secu = -y+sqrt(y*y+2*y);
	this->alpha_secu = 1-(dT / ( dT + 1/fc));

	// Complementary filters for linear velocity and position and acceleration
	filter_xyz_vel.initialize(dt, Vector3({3.0, 3.0, 3.0}));
	filter_xyz_pos.initialize(dt, Vector3({2.512562814, 2.512562814, 55.55555555}));

    // this is used to identify if steady, so response time is one gait T_gait = 0.32s
	filter_xyz_acc.initialize(dt, Vector3({2.512562814, 2.512562814, 2.512562814}));

	// IMU data
	this->IMU_lin_acc = Vector3::Zero(); // Linear acceleration (gravity debiased)
	this->IMU_ang_vel = Vector3::Zero(); // Angular velocity (gyroscopes)
	this->IMU_ang_pos = Eigen::Quaterniond(1,0,0,0); // Angular position (estimation of IMU)

	//  Forward Kinematics data
	this->FK_lin_vel = Vector3::Zero(); //  Linear velocity
	double FK_h = h_init;  //  Default base height of the FK
	this->FK_xyz << 0.0, 0.0, FK_h;
	this->xyz_mean_feet = Vector3::Zero();

	// patch the height in the low pass filter of xyc
	filter_xyz_pos.patchLowPassed(2,FK_h);

	// Boolean to disable FK and FG near contact switches
	this->close_from_contact = false;
	this->feet_status = Vector4::Zero();
	this->feet_goals = Matrix34::Zero();
	this->k_since_contact = Vector4::Zero();

	const std::string urdf_path = "/home/jochen/lisbeth/description/solo12.urdf";
	// Load the URDF model to get Pinocchio data and model structures
	if (!file_exists(urdf_path)) {
		std::cout << "Kinematics.initialize:" << urdf_path << " does not exist" << std::endl;
		exit(1);
	}

	pinocchio::JointModelFreeFlyer root_joint;
	pinocchio::urdf::buildModel(urdf_path,root_joint, model);

	// Create data required by the algorithms
	// for velocity estimation (forward kinematics)
	data = Data(model);

	pinocchio::urdf::buildModel(urdf_path,root_joint, model_for_xyz);

	// Create data required by the algorithms
	// for estimation estimation (forward kinematics)
	data_for_xyz = Data(model_for_xyz);

	// High pass linear velocity (filtered IMU velocity)
	this->HP_lin_vel = Vector3::Zero();

	// Low pass linear velocity (filtered FK velocity)
	this->LP_lin_vel = Vector3::Zero();

	this->o_filt_lin_vel = Vector3::Zero(); // Linear velocity (world frame)
	this->filt_lin_vel = Vector3::Zero(); //  Linear velocity (base frame)

	this->filt_lin_pos = Vector3::Zero(); // Linear position

	this->filt_ang_vel = Vector3::Zero(); // Angular velocity
	this->filt_ang_pos = Eigen::Quaterniond(); // Angular position

	this->q_filt = Vector19::Zero();
	this->v_filt = Vector18::Zero();
	this->v_secu = Vector12::Zero();
	// Various matrices
	this->q_FK = Vector19::Zero();
	this->q_FK.topRows(7) << 0.,0.,0.,0.,0.,0.,1.;
	this->v_FK = Vector18::Zero();
	this->indexes = {10, 18, 26, 34};  //  Indexes of feet frames
	this->actuators_pos = Vector12::Zero();
	this->actuators_vel = Vector12::Zero();

	// Transform between the base frame and the IMU frame
	// self._1Mi = pin.SE3(pin.Quaternion(np.array([[0.0, 0.0, 0.0, 1.0]]).T),np.array([0.1163, 0.0, 0.02]))
	_1Mi = pinocchio::SE3(pinocchio::SE3::Quaternion(0,0,0,1),Vector3({0.1163, 0.0, 0.02}));

	/*
	#  Logging matrices
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
	*/

	this->rotated_FK.resize(3, N_simulation);
	this->rotated_FK.setZero(); // = np.zeros((3, N_simulation));
	this->k_log = 0;

	this->debug_o_lin_vel = Vector3::Zero();
}

/** pass data from IMU */
static bool imu_data_eaten = false;
void Estimator::set_imu_data(Vector3 base_linear_acc, Vector3 base_angular_velocity, Vector4 base_orientation) {

	//  Linear acceleration of the trunk (base frame)
    this->IMU_lin_acc = base_linear_acc;

    // Angular velocity of the trunk (base frame)
    this->IMU_ang_vel = base_angular_velocity;


    // Angular position of the trunk (local frame)
    Eigen::Quaterniond base_orientation_q ({base_orientation[3], base_orientation[0], base_orientation[1], base_orientation[2]});
    RPY = quaternionToRPY(base_orientation_q);

    // use the very first call to calculate the offset in z
    static bool very_first_call = true;
    if (very_first_call) {
    	offset_yaw_IMU = this->RPY[2];
    	very_first_call = false;
    }
    RPY[2] -= offset_yaw_IMU; //  substract initial offset of IMU

    this->IMU_ang_pos = eulerToQuaternion(this->RPY[0],
                                          this->RPY[1],
                                          this->RPY[2]);

    // Above could be commented since IMU_ang_pos yaw is not used anywhere and instead
    // replace by: IMU_ang_pos[:] = device.baseOrientation
    imu_data_eaten = false;

}

static bool data_joints_eaten = false;
void Estimator::set_data_joints(Vector12 q_mes, Vector12 v_mes) {
	this->actuators_pos = q_mes;
	this->actuators_vel = v_mes;
	data_joints_eaten = false;
}


/* Estimate the velocity of the base with forward kinematics using a contact point
   that is supposed immobile in world frame
    Args:
        contactFrameId (int): ID of the contact point frame (foot frame)
*/
Vector3 Estimator::baseVelocityFromKinAndIMU(int contactFrameId) {

    Motion frameVelocity = pinocchio::getFrameVelocity(model,data, contactFrameId, pinocchio::ReferenceFrame::LOCAL);

    SE3 framePlacement = pinocchio::updateFramePlacement(model, data, contactFrameId);

    // Angular velocity of the base wrt the world in the base frame (Gyroscope)
    Vector3 _1w01 = IMU_ang_vel;
    // Linear velocity of the foot wrt the base in the foot frame
    Vector3 _Fv1F = frameVelocity.linear();
    // Level arm between the base and the foot
    Vector3 _1F = framePlacement.translation();
    // Orientation of the foot wrt the base
    Matrix33 _1RF = framePlacement.rotation();
    // Linear velocity of the base wrt world in the base frame
	Vector3 tmp = _1RF * _Fv1F;
    // Eigen::Map<Matrix13>(_Fv1F.data(),1,3);
    Vector3 _1v01 = cross3(_1F, _1w01) - tmp;

    // IMU and base frames have the same orientation
    // Vector3 _1v01 = _1v01 + cross3(_1Mi.translation(), _1w01);

    return _1v01;
}


// Get data with forward kinematics and forward geometry
// (linear velocity, angular velocity and position)
//  feet_status (4x0 numpy array): Current contact state of feet
void Estimator::set_data_FK(Vector4 feet_status) {
    // Update estimator FK model
	q_FK.bottomRows(12) = actuators_pos; //   Position of actuators

    v_FK.bottomRows(12) = actuators_vel; //  Velocity of actuators

    // Position and orientation of the base remain at 0
    // Linear and angular velocities of the base remain at 0
    // Update model used for the forward kinematics
    q_FK.block<4,1>(3,0) = Vector4({0,0,0,1});
    pinocchio::forwardKinematics(model,data,q_FK, v_FK);

    q_FK.block<4,1>(3,0) = Vector4({IMU_ang_pos.x(),IMU_ang_pos.y(),IMU_ang_pos.z(),IMU_ang_pos.w()});
    pinocchio::forwardKinematics(model_for_xyz, data_for_xyz, q_FK);

    // Get estimated velocity from updated model
    int cpt = 0;
    Vector3 vel_est = Vector3::Zero(3);
    Vector3 xyz_est = Vector3::Zero(3);
    for (int i = 0;i<4;i++) {
    	if (feet_status[i] == 1) { // Consider only feet in contact
    		if (k_since_contact[i] >= 16) { //  Security margin after the contact switch
    			// Estimated velocity of the base using the considered foot
                Vector3 vel_estimated_baseframe = baseVelocityFromKinAndIMU(indexes[i]);

                // Estimated position of the base using the considered foot
                SE3 framePlacement = pinocchio::updateFramePlacement(model_for_xyz, data_for_xyz, indexes[i]);
                Vector3 xyz_estimated = -framePlacement.translation();

                // # Logging
                // self.log_v_est[:, i, self.k_log] = vel_estimated_baseframe[0:3, 0]
                // self.log_h_est[i, self.k_log] = xyz_estimated[2]
				// Increment counter and add estimated quantities to the storage variables
				cpt += 1;
				vel_est += vel_estimated_baseframe; // Linear velocity
				xyz_est += xyz_estimated; //  Position

				double r_foot = 0.025; // 0.0155  # 31mm of diameter on meshlab
				if (i <= 1)
					vel_est[0] += r_foot * (actuators_vel[1+3*i] - actuators_vel[2+3*i]);
				else
					vel_est[0] += r_foot * (actuators_vel[1+3*i] + actuators_vel[2+3*i]);
    		}
    	}
    }

    //  If at least one foot is in contact, we do the average of feet results
    if (cpt > 0) {
        this->FK_lin_vel = vel_est / cpt;
        this->FK_xyz = xyz_est / cpt;
    }
}

/**
 Get average position of feet in contact with the ground

Args:
    feet_status (4x0 array): Current contact state of feet
    goals (3x4 array): Target locations of feet on the ground
*/
void Estimator::set_xyz_feet(Vector4 feet_status, Matrix34 goals) {
        int cpt = 0;
        Vector3 xyz_feet = Vector3::Zero();
        for (int i = 0;i<4;i++) {
        	if (feet_status[i] == 1) { // Consider only feet in contact
                cpt += 1;
                xyz_feet += goals.col(i);
        	}
        }
        // If at least one foot is in contact, we do the average of feet results
        if (cpt > 0)
            xyz_mean_feet = xyz_feet / cpt;
}


/**
 * Returns true if the acceleration and velocity of the base as measured by IMU is close to zero
 */
bool Estimator::isSteady() {
	double totalAcc = sqrt(filt_lin_acc[0]*filt_lin_acc[0]  + filt_lin_acc[1]*filt_lin_acc[1]);
	double totalVel = sqrt(filt_lin_vel[0]*filt_lin_vel[0]  + filt_lin_vel[1]*filt_lin_vel[1]);

	const double maxAcc = 0.1;
	const double maxVel = 0.02;

	return  (totalAcc < maxAcc) && (totalVel < maxVel);
}
/*
  Run the complementary filter to get the filtered quantities

  Args:
  	k (int): Number of inv dynamics iterations since the start of the simulation
    gait (4xN array): Contact state of feet (gait matrix)
    device (object): Interface with the masterboard or the simulation
    goals (3x4 array): Target locations of feet on the ground
*/
void Estimator::run_filter(int k, MatrixN gait, MatrixN goalsN, double baseHeight, Vector3 baseVelocity) {
       Vector4 feet_status = gait.row(0); //  Current contact state of feet
       int remaining_steps = 1;  // Remaining MPC steps for the current gait phase
       while (array_equal(feet_status, gait.row(remaining_steps)))
           remaining_steps += 1;

	   // take over data from IMU (GDM-GX-25), it is filtered the data already
	   assert(!imu_data_eaten);
       // Angular position of the trunk, IMU does the filtering already
       filt_ang_pos = IMU_ang_pos;

       // Angular velocity of the trunk
       filt_ang_vel = IMU_ang_vel;
       imu_data_eaten = true;

       // Update joints data
       assert(!data_joints_eaten);
       data_joints_eaten = true;

       // Update nb of iterations since contact
       k_since_contact += feet_status; // Increment feet in stance phase
       k_since_contact = k_since_contact.cwiseProduct(feet_status); // When the gait is complete, reset feet

       //  Update forward kinematics data
       set_data_FK(feet_status);

       // Update forward geometry data
       Matrix34 goals = goalsN;
       set_xyz_feet(feet_status, goals);

       //  Tune alpha depending on the state of the gait (close to contact switch or not)
       int a = (int)(ceil(k_since_contact.maxCoeff()/10)) - 1;
       int b = remaining_steps;
       int n = 1; // Nb of steps of margin around contact switch

       double v_max = 1.00;
       double v_min = 0.97; //  Minimum alpha value
       double c = ((a + b) - 2) * 0.5;
       if ((a <= 0) or (b <= 1)) {
    	   //  If we are close from contact switch
           alpha = v_max; // Only trust IMU data
           close_from_contact = true; //  Raise flag
       }
       else {
           alpha = v_min + (v_max - v_min) * abs(c - (a - n)) / c;
           close_from_contact = false; //  Lower flag
       }

        // Rotation matrix to go from base frame to world frame
  	   IMU_ang_pos.normalize();
       Matrix33 oRb = IMU_ang_pos.toRotationMatrix();

       // Get FK estimated velocity at IMU location (base frame)
       Vector3 cross_product = cross3(_1Mi.translation(), IMU_ang_vel);
       Vector3 i_FK_lin_vel = FK_lin_vel + cross_product;

       // Get FK estimated velocity at IMU location (world frame)
       Vector3 oi_FK_lin_vel = oRb *  i_FK_lin_vel;

       // Integration of IMU acc at IMU location (world frame)
       Vector3 oi_filt_lin_vel = filter_xyz_vel.compute(oi_FK_lin_vel,
                                                         oRb * IMU_lin_acc,
														 Vector3({alpha, alpha, alpha}));

       // Filtered estimated velocity at IMU location (base frame)
       Vector3 i_filt_lin_vel = oRb.transpose() * oi_filt_lin_vel;

       // Filtered estimated velocity at center base (base frame)
       Vector3 b_filt_lin_vel = i_filt_lin_vel - cross_product;

       // Filtered estimated velocity at center base (world frame)
       Vector3 ob_filt_lin_vel = oRb * b_filt_lin_vel;

       // Position of the center of the base from FGeometry and filtered velocity (world frame)
       this->filt_lin_pos = filter_xyz_pos.compute(FK_xyz + xyz_mean_feet, ob_filt_lin_vel);

       // Velocity of the center of the base (base frame)
       this->filt_lin_vel = b_filt_lin_vel;

       // acceleration of center of the base i(base frame)
       this->filt_lin_acc = filter_xyz_acc.compute(IMU_lin_acc);

           /*
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
       */

       // Output filtered position vector (19 x 1)
       q_filt.block<3,1>(0,0) = filt_lin_pos.block<3,1>(0,0);

       if (perfectEstimator) { // if we are in a simulation we get the base height directly and not by< the actuators
           q_filt[2] = baseHeight;
       }

       q_filt.block<4,1>(3,0) = Vector4({filt_ang_pos.x(), filt_ang_pos.y(), filt_ang_pos.z(),filt_ang_pos.w() });
       q_filt.block<12,1>(7,0) = actuators_pos;  // Actuators pos are already directly from PyBullet

       // Output filtered velocity vector (18 x 1)
       if (perfectEstimator)  //  Linear velocities directly from PyBullet
           v_filt.block<3,1>(0,0) = (1 - alpha_v) * v_filt.block<3,1>(0,0) + alpha_v * baseVelocity;
       else {
    	   v_filt.block<3,1>(0,0) = (1 - alpha_v) * v_filt.block<3,1>(0,0)  + alpha_v * filt_lin_vel;
       }

       v_filt.block<3,1>(3,0) = filt_ang_vel.block<3,1>(0,0);   // Angular velocities are already directly from PyBullet
       v_filt.block<12,1>(6,0) = actuators_vel.block<12,1>(0,0); //  Actuators velocities are already directly from PyBullet

	   /*

       // Update model used for the forward kinematics
       """pin.forwardKinematics(self.model, self.data, self.q_filt, self.v_filt)
       pin.updateFramePlacements(self.model, self.data)

       z_min = 100
       for i in (np.where(feet_status == 1))[0]:  # Consider only feet in contact
           # Estimated position of the base using the considered foot
           framePlacement = pin.updateFramePlacement(self.model, self.data, self.indexes[i])
           z_min = np.min((framePlacement.translation[2], z_min))
       self.q_filt[2, 0] -= z_min"""

       */

       // Output filtered actuators velocity for security checks
       v_secu = (1 - alpha_secu) * actuators_vel + alpha_secu * v_secu;

       // Increment iteration counter
       k_log += 1;
}

Vector19 Estimator::getQFiltered() { return q_filt; }

Vector18 Estimator::getVFiltered() { return v_filt; }

Vector3 Estimator::getImuRPY() { return RPY; }

Vector12 Estimator::getVSecu() { return v_secu; }
