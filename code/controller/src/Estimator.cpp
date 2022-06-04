#include <cmath>
#include <iostream>
#include <cmath>
#include <iostream>
#include <string>

#include "Estimator.hpp"

using namespace std;


Estimator::Estimator() {
}


void Estimator::initialize(double dT, int N_simulation, double h_init, bool perfectEstimator ) {

    footRadius= 0.155;

	// sample frequency
	this->dt = dT;

	// flag if the IMU is perfect
	this->perfectEstimator = perfectEstimator;

    this->alphaVelMax = 1.0;
    this->alphaVelMin = 0.97;

	// Filtering estimated linear velocity
	double fc = 50.0;  //  Cut frequency
	double y = 1 - cos(2*M_PI*fc*dt);
	this->alpha_v = -y+sqrt(y*y+2*y);
	fc = 500;
	this->alpha_v = 1-(dT / ( dT + 1/fc));
	this->filter_v.initialize(dt, 500, Vector3({0.0, 0.0,0.0}));
	this->filter_secu_actuator_v.initialize(dt, 6.0, Vector3({0.0, 0.0,0.0}));

	//  Filtering velocities used for security checks
	fc = 6.0;
	y = 1 - cos(2*M_PI*fc*dt);
	this->alphaSecurity = -y+sqrt(y*y+2*y);
	this->alphaSecurity = 1-(dT / ( dT + 1/fc));

	// Complementary filters for linear velocity and position and acceleration
	velocityFilter.initialize(dt, Vector3::Zero(), Vector3::Zero());
	positionFilter.initialize(dt, Vector3::Zero(), basePositionFK);
	alphaPos = Vector3({0.995, 0.995, 0.9});

    // this is used to identify if steady, so response time is one gait T_gait = 0.32s
	accelerationFilter.initialize(dt, Vector3::Zero(), Vector3::Zero());
	alphaAcc = Vector3({2.512562814, 2.512562814, 2.512562814});

	// IMU data
	this->IMULinearAcceleration = Vector3::Zero(); // Linear acceleration (gravity debiased)
	this->IMUAngularVelocity = Vector3::Zero(); // Angular velocity (gyroscopes)
	this->IMUQuat = Eigen::Quaterniond(1,0,0,0); // Angular position (estimation of IMU)

	//  Forward Kinematics data
	this->baseVelocityFK = Vector3::Zero(); //  Linear velocity
	this->basePositionFK << 0.0, 0.0, h_init;  //  Default base height of the FK
	this->feetPositionBarycenter = Vector3::Zero();

	// Boolean to disable FK and FG near contact switches
	this->phaseRemainingDuration = 0;
	this->feetStatus = Vector4::Zero();
	this->feetTargets = Matrix34::Zero();
	this->feetStancePhaseDuration = Vector4::Zero();

	const std::string urdf_path = "/home/jochen/lisbeth/description/solo12.urdf";
	// Load the URDF model to get Pinocchio data and model structures
	if (!file_exists(urdf_path)) {
		std::cout << "Kinematics.initialize:" << urdf_path << " does not exist" << std::endl;
		exit(1);
	}

	pinocchio::JointModelFreeFlyer root_joint;
	pinocchio::urdf::buildModel(urdf_path,root_joint, velocityModel);
	// Create data required by the algorithms
	// for velocity estimation (forward kinematics)
	velocityData = Data(velocityModel);

	pinocchio::urdf::buildModel(urdf_path,root_joint, positionModel);
	// Create data required by the algorithms
	// for estimation estimation (forward kinematics)
	positionData = Data(positionModel);

	this->b_baseVelocity = Vector3::Zero(); //  Linear velocity (base frame)

	this->qEstimate = Vector19::Zero();
	this->vEstimate = Vector18::Zero();
	this->v_secu = Vector12::Zero();

	// Various matrices
	this->q_FK = Vector19::Zero();
	this->q_FK.topRows(7) << 0.,0.,0.,0.,0.,0.,1.;
	this->v_FK = Vector18::Zero();
	this->feetFrames = {10, 18, 26, 34};  //  Indexes of feet frames
	// feetFrames << (int)positionModel.getFrameId("FL_FOOT"), (int)positionModel.getFrameId("FR_FOOT"),
	//      (int)positionModel.getFrameId("HL_FOOT"), (int)positionModel.getFrameId("HR_FOOT");
	this->qActuators = Vector12::Zero();
	this->vActuators = Vector12::Zero();

	// Transform between the base frame and the IMU frame
	// self.b_M_IMU = pin.SE3(pin.Quaternion(np.array([[0.0, 0.0, 0.0, 1.0]]).T),np.array([0.1163, 0.0, 0.02]))
	b_M_IMU = pinocchio::SE3(pinocchio::SE3::Quaternion(0,0,0,1),Vector3({0.1163, 0.0, 0.02}));

	/*
	#  Logging matrices
	self.log_v_truth = np.zeros((3, N_simulation))
	self.log_v_est = np.zeros((3, 4, N_simulation))
	self.log_h_est = np.zeros((4, N_simulation))
	self.log_alpha = np.zeros(N_simulation)
	self.log_HP_lin_vel = np.zeros((3, N_simulation))
	self.log_IMU_lin_vel = np.zeros((3, N_simulation))
	self.log_IMULinearAcceleration = np.zeros((3, N_simulation))
	self.log_LP_lin_vel = np.zeros((3, N_simulation))
	self.log_FK_lin_vel = np.zeros((3, N_simulation))
	self.log_o_filt_lin_vel = np.zeros((3, N_simulation))
	self.log_filt_lin_vel = np.zeros((3, N_simulation))
	self.log_filt_lin_vel_bis = np.zeros((3, N_simulation))
	*/

	this->k_log = 0;
}

/** pass data from IMU */
void Estimator::updateIMUData(Vector3 base_linear_acc, Vector3 base_angular_velocity, Vector4 base_orientation, VectorN const& perfectPosition) {

	//  Linear acceleration of the trunk (base frame)
    this->IMULinearAcceleration = base_linear_acc;

    // Angular velocity of the trunk (base frame)
    this->IMUAngularVelocity = base_angular_velocity;


    // Angular position of the trunk (local frame)
    Eigen::Quaterniond base_orientation_q ({base_orientation[3], base_orientation[0], base_orientation[1], base_orientation[2]});
    IMURpy = quaternionToRPY(base_orientation_q);

    // use the very first call to calculate the offset in z
    static bool initialized = false;
    if (!initialized) {
    	IMUYawOffset = this->IMURpy[2];
    	initialized = true;
    }
    IMURpy[2] -= IMUYawOffset; //  substract initial offset of IMU

    bool solo3D = false;
    if (solo3D)
    	IMURpy.tail(1) = perfectPosition.tail(1);

    this->IMUQuat = eulerToQuaternion(this->IMURpy[0],
                                          this->IMURpy[1],
                                          this->IMURpy[2]);
}

void Estimator::updatFeetStatus(MatrixN const& gait, MatrixN const& feetTargets) {
  this->feetStatus = gait.row(0);
  this->feetTargets = feetTargets;

  feetStancePhaseDuration += this->feetStatus;
  feetStancePhaseDuration = feetStancePhaseDuration.cwiseProduct(this->feetStatus);

  phaseRemainingDuration = 1;
  while (this->feetStatus.isApprox((Vector4)gait.row(phaseRemainingDuration))) {
    phaseRemainingDuration++;
  }
}

static bool data_joints_eaten = false;
void Estimator::updateJointData(Vector12 const& q, Vector12 const &v) {
	this->qActuators = q;
	this->vActuators = v;
	data_joints_eaten = false;
}


/* Estimate the velocity of the base with forward kinematics using a contact point
   that is supposed immobile in world frame
    Args:
        contactFrameId (int): ID of the contact point frame (foot frame)
*/
Vector3 Estimator::baseVelocityFromKinAndIMU(int contactFrameId) {

    Motion frameVelocity = pinocchio::getFrameVelocity(velocityModel,velocityData, contactFrameId, pinocchio::ReferenceFrame::LOCAL);

    SE3 framePlacement = pinocchio::updateFramePlacement(velocityModel, velocityData, contactFrameId);

    // Angular velocity of the base wrt the world in the base frame (Gyroscope)
    Vector3 _1w01 = IMUAngularVelocity;
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
    // Vector3 _1v01 = _1v01 + cross3(b_M_IMU.translation(), _1w01);

    return _1v01;
}

Vector3 Estimator::computeBaseVelocityFromFoot(int footId) {
  pinocchio::updateFramePlacement(velocityModel, velocityData, feetFrames[footId]);
  pinocchio::SE3 contactFrame = velocityData.oMf[feetFrames[footId]];
  Vector3 frameVelocity =
      pinocchio::getFrameVelocity(velocityModel, velocityData, feetFrames[footId], pinocchio::LOCAL).linear();

  return contactFrame.translation().cross(IMUAngularVelocity) - contactFrame.rotation() * frameVelocity;
}

Vector3 Estimator::computeBasePositionFromFoot(int footId) {
  pinocchio::updateFramePlacement(positionModel, positionData, feetFrames[footId]);
  Vector3 basePosition = -positionData.oMf[feetFrames[footId]].translation();
  basePosition(0) += footRadius * (vActuators(1 + 3 * footId) + vActuators(2 + 3 * footId));

  return basePosition;
}


// Get data with forward kinematics and forward geometry
// (linear velocity, angular velocity and position)
//  feet_status (4x0 numpy array): Current contact state of feet
void Estimator::updateForwardKinematics() {
    // Update estimator FK model
	q_FK.bottomRows(12) = qActuators; //   Position of actuators
    v_FK.bottomRows(12) = vActuators; //  Velocity of actuators

    // Position and orientation of the base remain at 0
    // Linear and angular velocities of the base remain at 0
    // Update model used for the forward kinematics
    q_FK.block<4,1>(3,0) = Vector4({0,0,0,1});
    pinocchio::forwardKinematics(velocityModel,velocityData,q_FK, v_FK);

    q_FK.block<4,1>(3,0) = Vector4({IMUQuat.x(),IMUQuat.y(),IMUQuat.z(),IMUQuat.w()});
    pinocchio::forwardKinematics(positionModel, positionData, q_FK);

    // Get estimated velocity from updated model
    int nContactFeet = 0;
    Vector3 baseVelocityEstimate = Vector3::Zero(3);
    Vector3 basePositionEstimate = Vector3::Zero(3);
    for (int foot = 0;foot<4;foot++) {
    	if ((feetStatus[foot] == 1) && (feetStancePhaseDuration[foot] >= 16)) { //  Security margin after the contact switch
    	      baseVelocityEstimate += computeBaseVelocityFromFoot(foot);
    	      basePositionEstimate += computeBasePositionFromFoot(foot);
    	      nContactFeet ++;
    	}
    }

    //  If at least one foot is in contact, we do the average of feet results
    if (nContactFeet > 0) {
        this->baseVelocityFK = baseVelocityEstimate / nContactFeet;
        this->basePositionFK = basePositionEstimate / nContactFeet;
    }
}

/**
 Get average position of feet in contact with the ground

Args:
    feet_status (4x0 array): Current contact state of feet
    goals (3x4 array): Target locations of feet on the ground
*/
void Estimator::computeFeetPositionBarycenter() {
        int nContactFeet = 0;
        Vector3 xyz_feet = Vector3::Zero();
        for (int i = 0;i<4;i++) {
        	if (feetStatus[i] == 1) { // Consider only feet in contact
        		nContactFeet += 1;
                xyz_feet += feetTargets.col(i);
        	}
        }
        // If at least one foot is in contact, we do the average of feet results
        if (nContactFeet > 0)
            feetPositionBarycenter = xyz_feet / nContactFeet;
}



/**
 * Returns true if the acceleration and velocity of the base as measured by IMU is close to zero
 */
bool Estimator::isSteady() {
	double totalAcc = sqrt(filt_lin_acc[0]*filt_lin_acc[0]  + b_baseVelocity[1]*filt_lin_acc[1]);
	double totalVel = sqrt(b_baseVelocity[0]*b_baseVelocity[0]  + b_baseVelocity[1]*b_baseVelocity[1]);

	const double maxAcc = 0.1;
	const double maxVel = 0.02;

	return  (totalAcc < maxAcc) && (totalVel < maxVel);
}

double Estimator::computeAlphaVelocity() {
  double a = std::ceil(feetStancePhaseDuration.maxCoeff() * 0.1) - 1;
  double b = static_cast<double>(phaseRemainingDuration);
  double c = ((a + b) - 2) * 0.5;
  if (a <= 0 || b <= 1)
    return alphaVelMax;
  else
    return alphaVelMin + (alphaVelMax - alphaVelMin) * std::abs(c - (a - 1)) / c;
}

void Estimator::estimatePositionAndVelocity(Vector3 const& perfectPosition, Vector3 const& b_perfectVelocity) {
  Vector3 alpha = Vector3::Ones() * computeAlphaVelocity();
  Matrix3 oRb = IMUQuat.toRotationMatrix();
  Vector3 bTi = (b_M_IMU.translation()).cross(IMUAngularVelocity);

  // At IMU location in world frame
  bool solo3D = false;
  Vector3 oi_baseVelocityFK = solo3D ? oRb * (b_perfectVelocity + bTi) : oRb * (baseVelocityFK + bTi);
  Vector3 oi_baseVelocity = velocityFilter.compute(oi_baseVelocityFK, oRb * IMULinearAcceleration, alpha);

  // At base location in base frame
  b_baseVelocity = oRb.transpose() * oi_baseVelocity - bTi;

  vEstimate.head(3) = perfectEstimator ? b_perfectVelocity : b_baseVelocity;
  vEstimate.segment(3, 3) = IMUAngularVelocity;
  vEstimate.tail(12) = vActuators;

  Vector3 basePosition = solo3D ? perfectPosition : (basePositionFK + feetPositionBarycenter);
  qEstimate.head(3) = positionFilter.compute(basePosition, oRb * b_baseVelocity, alphaPos);

  if (perfectEstimator || solo3D) qEstimate(2) = perfectPosition(2);
  qEstimate.segment(3, 4) = IMUQuat.coeffs();
  qEstimate.tail(12) = qActuators;
}


/*
  Run the complementary filter to get the filtered quantities

  Args:
  	k (int): Number of inv dynamics iterations since the start of the simulation
    gait (4xN array): Contact state of feet (gait matrix)
    device (object): Interface with the masterboard or the simulation
    goals (3x4 array): Target locations of feet on the ground
*/
void Estimator::run(int k, MatrixN gait, MatrixN goalsN,
				 	Vector3 baseLinearAcceleration, Vector3 baseAngularVelocity, Vector4 baseOrientation,
					Vector12 const& q, Vector12 const &v,
					VectorN const& perfectPosition,Vector3 const& b_perfectVelocity,
					double baseHeight, Vector3 baseVelocity) {

	// store parameter coming from IMU
	updateIMUData (baseLinearAcceleration, baseAngularVelocity, baseOrientation,perfectPosition);

	// update feet target positions according to gait
	updatFeetStatus(gait, feetTargets);

	// store positions and velocities
	updateJointData(q, v);

    //  Update forward kinematics data
    updateForwardKinematics();

    // Update forward geometry data
    computeFeetPositionBarycenter();


       // Update joints data
       assert(!data_joints_eaten);
       data_joints_eaten = true;

       //  Tune alpha depending on the state of the gait (close to contact switch or not)
       alpha = computeAlphaVelocity();

        // Rotation matrix to go from base frame to world frame
  	   IMUQuat.normalize();
       Matrix33 oRb = IMUQuat.toRotationMatrix();

       // Get FK estimated velocity at IMU location (base frame)
       Vector3 cross_product = cross3(b_M_IMU.translation(), IMUAngularVelocity);
       Vector3 i_FK_lin_vel = baseVelocityFK + cross_product;

       // Get FK estimated velocity at IMU location (world frame)
       Vector3 oi_FK_lin_vel = oRb *  i_FK_lin_vel;

       // Integration of IMU acc at IMU location (world frame)
       Vector3 oi_filt_lin_vel = velocityFilter.compute(oi_FK_lin_vel,
                                                         oRb * IMULinearAcceleration,
														 Vector3({alpha, alpha, alpha}));

       // Filtered estimated velocity at IMU location (base frame)
       Vector3 i_filt_lin_vel = oRb.transpose() * oi_filt_lin_vel;

       // Filtered estimated velocity at center base (base frame)
       Vector3 b_filt_lin_vel = i_filt_lin_vel - cross_product;
       // Velocity of the center of the base (base frame)
       this->b_baseVelocity = b_filt_lin_vel;

       // Position of the center of the base from FGeometry and filtered velocity (world frame)
       bool solo3D = false;
       Vector3 basePosition = solo3D ? perfectPosition : (basePositionFK + feetPositionBarycenter);
       Vector3 filt_lin_pos = positionFilter.compute(basePosition, oRb * b_filt_lin_vel, alphaPos);

       // acceleration of center of the base i(base frame)
       this->filt_lin_acc = accelerationFilter.compute(filt_lin_acc, IMULinearAcceleration, alphaAcc);

       // Output filtered position vector (19 x 1)
       qEstimate.block<3,1>(0,0) = filt_lin_pos.block<3,1>(0,0);

       if (perfectEstimator) { // if we are in a simulation we get the base height directly and not by< the actuators
           // qEstimate[2] = baseHeight;
           qEstimate[2] = perfectPosition[2];

       }

       qEstimate.block<4,1>(3,0) = Vector4({IMUQuat.x(), IMUQuat.y(), IMUQuat.z(),IMUQuat.w() });
       qEstimate.block<12,1>(7,0) = qActuators;  // Actuators pos are already directly from PyBullet

       vEstimate.block<3,1>(0,0) = perfectEstimator?filter_v.compute (b_perfectVelocity):filter_v.compute (b_baseVelocity);
       vEstimate.block<3,1>(3,0) = IMUAngularVelocity.block<3,1>(0,0);   // Angular velocities are already directly from PyBullet
       vEstimate.block<12,1>(6,0) = vActuators.block<12,1>(0,0); //  Actuators velocities are already directly from PyBullet

	   /*

       // Update model used for the forward kinematics
       """pin.forwardKinematics(self.model, self.data, self.qEstimate, self.vEstimate)
       pin.updateFramePlacements(self.model, self.data)

       z_min = 100
       for i in (np.where(feet_status == 1))[0]:  # Consider only feet in contact
           # Estimated position of the base using the considered foot
           framePlacement = pin.updateFramePlacement(self.model, self.data, self.feetFrames[i])
           z_min = np.min((framePlacement.translation[2], z_min))
       self.qEstimate[2, 0] -= z_min"""

       */

       // Output filtered actuators velocity for security checks
       v_secu = (1 - alphaSecurity) * vActuators + alphaSecurity * v_secu;

       // Increment iteration counter
       k_log += 1;
}

Vector19 Estimator::getQFiltered() { return qEstimate; }

Vector18 Estimator::getVFiltered() { return vEstimate; }

Vector3 Estimator::getImuRPY() { return IMURpy; }

Vector12 Estimator::getVSecu() { return v_secu; }
