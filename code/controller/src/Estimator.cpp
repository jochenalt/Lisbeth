#include <cmath>
#include <iostream>
#include <cmath>
#include <iostream>
#include <string>

#include "Estimator.hpp"

using namespace std;

Estimator::Estimator()
    : perfectEstimator(false),
      dt(0.0),
      feetFrames({0,0,0,0}),
      footRadius(0.155),
      alphaPos({0.995, 0.995, 0.9}),
      alphaAcc({0,0,0}),
	  alphaVelMax(1.),
      alphaVelMin(0.97),
      alphaSecurity(0.),
      IMUYawOffset(0.),
      IMULinearAcceleration(Vector3::Zero()),
      IMUAngularVelocity(Vector3::Zero()),
      IMURpy(Vector3::Zero()),
      IMUQuat(pinocchio::SE3::Quaternion(1.0, 0.0, 0.0, 0.0)),
      qActuators(Vector12::Zero()),
      vActuators(Vector12::Zero()),
      phaseRemainingDuration(0),
      feetStancePhaseDuration(Vector4::Zero()),
      feetStatus(Vector4::Zero()),
      feetTargets(Matrix34::Zero()),
      q_FK(Vector19::Zero()),
      v_FK(Vector18::Zero()),
      baseVelocityFK(Vector3::Zero()),
      basePositionFK(Vector3::Zero()),
      b_baseVelocity(Vector3::Zero()),
      feetPositionBarycenter(Vector3::Zero()),
      qEstimate(Vector19::Zero()),
      vEstimate(Vector18::Zero()),
      vSecurity(Vector12::Zero()),
      windowSize(0),
      vFiltered(Vector6::Zero()),
      qRef(Vector18::Zero()),
      vRef(Vector18::Zero()),
      baseVelRef(Vector6::Zero()),
      baseAccRef(Vector6::Zero()),
      oRh(Matrix3::Identity()),
      hRb(Matrix3::Identity()),
      oTh(Vector3::Zero()),
      h_v(Vector6::Zero()),
	  h_vFiltered(Vector6::Zero()),
	  filt_lin_acc(Vector3::Zero())
{
  b_M_IMU = pinocchio::SE3(pinocchio::SE3::Quaternion(1.0, 0.0, 0.0, 0.0), Vector3(0.1163, 0.0, 0.02));
  q_FK(6) = 1.0;
  qEstimate(6) = 1.0;
}


void Estimator::initialize(Vector12 q_init, double dt_mpc, double dt_wbc, int gaitRows, int N_periods, int N_simulation, double h_ref, bool perfectEstimator ) {

	// sample frequency
	this->dt = dt_wbc;
	this->perfectEstimator = perfectEstimator;

	// Filtering estimated linear velocity
	int k_mpc = (int)(std::round(dt_mpc / dt_wbc));
	windowSize = (int)(k_mpc * gaitRows / N_periods);
	vx_queue.resize(windowSize, 0.0);  // List full of 0.0
	vy_queue.resize(windowSize, 0.0);  // List full of 0.0
	vz_queue.resize(windowSize, 0.0);  // List full of 0.0


	// Filtering velocities used for security checks
	double fc = 6.0;
	double y = 1 - cos(2*M_PI*fc*dt);
	this->alphaSecurity = -y+sqrt(y*y+2*y);
	this->alphaSecurity = 1-(dt / ( dt + 1/fc));

    // Initialize Quantities
	this->basePositionFK << 0.0, 0.0, h_ref;  //  Default base height of the FK
	velocityFilter.initialize(dt, Vector3::Zero(), Vector3::Zero());
	positionFilter.initialize(dt, Vector3::Zero(), basePositionFK);
	qRef(2, 0) = h_ref;
	qRef.tail(12) = Vector12(q_init.data());

    // this is used to identify if steady, so response time is one gait T_gait = 0.32s
	accelerationFilter.initialize(dt, Vector3::Zero(), Vector3::Zero());
	alphaAcc = Vector3({2.512562814, 2.512562814, 2.512562814});

	// IMU data
	this->IMULinearAcceleration = Vector3::Zero(); // Linear acceleration (gravity debiased)
	this->IMUAngularVelocity = Vector3::Zero(); // Angular velocity (gyroscopes)
	this->IMUQuat = Eigen::Quaterniond(1,0,0,0); // Angular position (estimation of IMU)

	//  Forward Kinematics data
	this->baseVelocityFK = Vector3::Zero(); //  Linear velocity
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
	this->vSecurity = Vector12::Zero();

	// Various matrices
	this->q_FK = Vector19::Zero();
	this->q_FK.topRows(7) << 0.,0.,0.,0.,0.,0.,1.;
	this->v_FK = Vector18::Zero();
	this->feetFrames = {10, 18, 26, 34};  //  Indexes of feet frames
	this->feetFrames = {(int)positionModel.getFrameId("FL_FOOT"), (int)positionModel.getFrameId("FR_FOOT"),
	      (int)positionModel.getFrameId("HL_FOOT"), (int)positionModel.getFrameId("HR_FOOT")};
	this->qActuators = Vector12::Zero();
	this->vActuators = Vector12::Zero();

	// Transform between the base frame and the IMU frame
	b_M_IMU = pinocchio::SE3(pinocchio::SE3::Quaternion(0,0,0,1),Vector3({0.1163, 0.0, 0.02}));
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

void Estimator::updateJointData(Vector12 const& q, Vector12 const &v) {
	this->qActuators = q;
	this->vActuators = v;
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
					VectorN const& perfectPosition,Vector3 const& b_perfectVelocity) {

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

    estimatePositionAndVelocity ( b_perfectVelocity, perfectPosition.head(3));

    filterVelocity();

    // Output filtered actuators velocity for security checks
    vSecurity = (1 - alphaSecurity) * vActuators + alphaSecurity * vSecurity;
}

void Estimator::updateReferenceState(VectorN const& newvRef) {
  // Update reference acceleration and velocities
  Matrix3 Rz = pinocchio::rpy::rpyToMatrix(0., 0., -baseVelRef[5] * dt);
  baseAccRef.head(3) = (newvRef.head(3) - Rz * baseVelRef.head(3)) / dt;
  baseAccRef.tail(3) = (newvRef.tail(3) - Rz * baseVelRef.tail(3)) / dt;
  baseVelRef = newvRef;

  // Update position and velocity state vectors
  Rz = pinocchio::rpy::rpyToMatrix(0., 0., qRef[5]);
  vRef.head(2) = Rz.topLeftCorner(2, 2) * baseVelRef.head(2);
  vRef[5] = baseVelRef[5];
  vRef.tail(12) = vActuators;

  qRef.head(2) += vRef.head(2) * dt;
  qRef[2] = qEstimate[2];
  qRef.segment(3, 2) = IMURpy.head(2);
  qRef[5] += baseVelRef[5] * dt;
  qRef.tail(12) = qActuators;

  // Transformation matrices
  hRb = pinocchio::rpy::rpyToMatrix(IMURpy[0], IMURpy[1], 0.);
  oRh = pinocchio::rpy::rpyToMatrix(0., 0., qRef[5]);
  oTh.head(2) = qRef.head(2);

  // Express estimated velocity and filtered estimated velocity in horizontal frame
  h_v.head(3) = hRb * vEstimate.head(3);
  h_v.tail(3) = hRb * vEstimate.segment(3, 3);
  h_vFiltered.head(3) = hRb * vFiltered.head(3);
  h_vFiltered.tail(3) = hRb * vFiltered.tail(3);
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



void Estimator::filterVelocity() {
  vFiltered = vEstimate.head(6);
  vx_queue.pop_back();
  vy_queue.pop_back();
  vz_queue.pop_back();
  vx_queue.push_front(vEstimate(0));
  vy_queue.push_front(vEstimate(1));
  vz_queue.push_front(vEstimate(2));
  vFiltered(0) = std::accumulate(vx_queue.begin(), vx_queue.end(), 0.) / windowSize;
  vFiltered(1) = std::accumulate(vy_queue.begin(), vy_queue.end(), 0.) / windowSize;
  vFiltered(2) = std::accumulate(vz_queue.begin(), vz_queue.end(), 0.) / windowSize;
}



