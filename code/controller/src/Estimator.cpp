#include <cmath>
#include <iostream>
#include <cmath>
#include <iostream>
#include <string>

#include "Estimator.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/math/rpy.hpp"
#include "pinocchio/parsers/urdf.hpp"
using namespace std;

Estimator::Estimator()
    : feetFrames_ID({0,0,0,0}),
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
	   baseAcceleration(Vector3::Zero()),
      feetPositionBarycenter(Vector3::Zero()),
      qEstimate(Vector19::Zero()),
      vEstimate(Vector18::Zero()),
      vSecurity(Vector12::Zero()),
      vFiltered(Vector6::Zero()),
      qRef(Vector18::Zero()),
      vRef(Vector18::Zero()),
      baseVelRef(Vector6::Zero()),
      baseAccRef(Vector6::Zero()),
      oRh(Matrix3::Identity()),
      hRb(Matrix3::Identity()),
      oTh(Vector3::Zero()),
      h_v(Vector6::Zero()),
	   h_vFiltered(Vector6::Zero())
{
	transBase2IMU = Vector3(0.1163, 0.0, 0.02);
  q_FK(6) = 1.0;
  qEstimate(6) = 1.0;
}


void Estimator::initialize(Params& params_in) {

	params = &params_in;

	// Filtering estimated linear velocity
	vx_queue.resize(get_windows_size(), 0.0);  // List full of 0.0
	vy_queue.resize(get_windows_size(), 0.0);  // List full of 0.0
	vz_queue.resize(get_windows_size(), 0.0);  // List full of 0.0

	// Filtering velocities used for security checks
	double fc = 6.0;
	double y = 1 - cos(2*M_PI*fc*params->dt_wbc);
	this->alphaSecurity = -y+sqrt(y*y+2*y);
	this->alphaSecurity = 1-(params->dt_wbc / ( params->dt_wbc + 1/fc));

	// Initialize Quantities
	basePositionFK(2) = params->h_ref;
	velocityFilter.initialize(params->dt_wbc, Vector3::Zero(), Vector3::Zero());
	positionFilter.initialize(params->dt_wbc, Vector3::Zero(), basePositionFK);
	qRef(2, 0) = params->h_ref;
	qRef.tail(12) = Vector12(params->q_init.data());

	// Initialize Pinocchio
	const std::string filename = std::string(URDF_MODEL);
	pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(), velocityModel, false);
	pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(), positionModel, false);
	velocityData = pinocchio::Data(velocityModel);
	positionData = pinocchio::Data(positionModel);
	pinocchio::computeAllTerms(velocityModel, velocityData, qEstimate, vEstimate);
	pinocchio::computeAllTerms(positionModel, positionData, qEstimate, vEstimate);
	this->feetFrames_ID = {(int)positionModel.getFrameId("FL_FOOT"), (int)positionModel.getFrameId("FR_FOOT"),
	  	      		     (int)positionModel.getFrameId("HL_FOOT"), (int)positionModel.getFrameId("HR_FOOT")};
}


/**
 * Returns true if the acceleration and velocity of the base as measured by IMU is close to zero
 */
bool Estimator::isSteady() {
	double totalAcc = sqrt(baseAcceleration[0]*baseAcceleration[0]  + baseAcceleration[1]*baseAcceleration[1]);
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
void Estimator::run(MatrixN4 gait, Matrix34 feetTargets,
				 	Vector3 baseLinearAcceleration, Vector3 baseAngularVelocity, Vector3 baseOrientation,Vector4 baseOrientationQuad,
					Vector12 const& q, Vector12 const &v) {

			// store parameter coming from IMU
	updateIMUData (baseLinearAcceleration, baseAngularVelocity, baseOrientation, baseOrientationQuad);

	// update feet target positions according to gait
	updatFeetStatus(gait, feetTargets);

	// store positions and velocities
	updateJointData(q, v);

    //  Update forward kinematics data
    updateForwardKinematics();

    // Update forward geometry data
    computeFeetPositionBarycenter();

    estimatePositionAndVelocity ();

    filterVelocity();

    // Output filtered actuators velocity for security checks
    vSecurity = (1 - alphaSecurity) * vActuators + alphaSecurity * vSecurity;

//     std::cout << "feetTargets" << feetTargets<< std::endl
//			<< "gait" << gait << std::endl
//     		<< "baseLinAcc" << baseLinearAcceleration << std::endl
//			<< "baseAngularVelocity" << baseAngularVelocity<< std::endl
//			<< "baseOrientation" << baseOrientation<< std::endl
//			<< "getQReference" << getQReference()
//			<< std::endl;


}

void Estimator::updateReferenceState(VectorN const& newvRef) {
  // Update reference acceleration and velocities
  Matrix3 Rz = pinocchio::rpy::rpyToMatrix(0., 0., -baseVelRef[5] * params->dt_wbc);
  baseAccRef.head(3) = (newvRef.head(3) - Rz * baseVelRef.head(3)) / params->dt_wbc;
  baseAccRef.tail(3) = (newvRef.tail(3) - Rz * baseVelRef.tail(3)) / params->dt_wbc;
  baseVelRef = newvRef;

  // Update position and velocity state vectors
  Rz = pinocchio::rpy::rpyToMatrix(0., 0., qRef[5]);
  vRef.head(2) = Rz.topLeftCorner(2, 2) * baseVelRef.head(2);
  vRef[5] = baseVelRef[5];
  vRef.tail(12) = vActuators;

  qRef.head(2) += vRef.head(2) * params->dt_wbc;
  qRef[2] = qEstimate[2];
  qRef.segment(3, 2) = IMURpy.head(2);
  qRef[5] += baseVelRef[5] * params->dt_wbc;
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


/** pass data from IMU */
void Estimator::updateIMUData(Vector3 base_linear_acc, Vector3 base_angular_velocity, Vector3 base_orientation, Vector4 baseOrientationQuad) {

	//  Linear acceleration of the trunk (base frame)
    this->IMULinearAcceleration = base_linear_acc;

    // Angular velocity of the trunk (base frame)
    this->IMUAngularVelocity = base_angular_velocity;

    // Angular position of the trunk (local frame)
    IMURpy = base_orientation;

    // use the very first call to calculate the offset in z
    static bool initialized = false;
    if (!initialized) {
    	IMUYawOffset = this->IMURpy[2];
    	initialized = true;
    }
    IMURpy(2) -= IMUYawOffset; //  substract initial offset of IMU

    IMUQuat = Quaternion(baseOrientationQuad[0], baseOrientationQuad[1], baseOrientationQuad[2], baseOrientationQuad[3]);
    std::cout << "linacc" << base_linear_acc  << std::endl;

}

void Estimator::updateJointData(Vector12 const& q, Vector12 const &v) {
	this->qActuators = q;
	this->vActuators = v;
}


// Get data with forward kinematics and forward geometry
// (linear velocity, angular velocity and position)
//  feet_status : Current contact state of feet
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


Vector3 Estimator::computeBaseVelocityFromFoot(int footId) {
  pinocchio::updateFramePlacement(velocityModel, velocityData, feetFrames_ID[footId]);
  pinocchio::SE3 contactFrame = velocityData.oMf[feetFrames_ID[footId]];
  Vector3 frameVelocity = pinocchio::getFrameVelocity(velocityModel, velocityData, feetFrames_ID[footId], pinocchio::LOCAL).linear();

  return contactFrame.translation().cross(IMUAngularVelocity) - contactFrame.rotation() * frameVelocity;
}

Vector3 Estimator::computeBasePositionFromFoot(int footId) {
  pinocchio::updateFramePlacement(positionModel, positionData, feetFrames_ID[footId]);
  Vector3 basePosition = -positionData.oMf[feetFrames_ID[footId]].translation();
  basePosition(0) += footRadius * (vActuators(1 + 3 * footId) + vActuators(2 + 3 * footId));

  return basePosition;
}

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

double Estimator::computeAlphaVelocity() {
  double a = std::ceil(feetStancePhaseDuration.maxCoeff() * 0.1) - 1;
  double b = static_cast<double>(phaseRemainingDuration);
  double c = ((a + b) - 2) * 0.5;
  if (a <= 0 || b <= 1)
    return alphaVelMax;
  else
    return alphaVelMin + (alphaVelMax - alphaVelMin) * std::abs(c - (a - 1)) / c;
}


void Estimator::estimatePositionAndVelocity() {
  Vector3 alpha = Vector3::Ones() * computeAlphaVelocity();
  Matrix3 oRb = IMUQuat.toRotationMatrix();
  Vector3 bTi = transBase2IMU.cross(IMUAngularVelocity);

  // At IMU location in world frame
  Vector3 oi_baseVelocityFK = oRb * (baseVelocityFK + bTi);
  Vector3 oi_baseVelocity = velocityFilter.compute(oi_baseVelocityFK, oRb * IMULinearAcceleration, alpha);

  // At base location in base frame
  b_baseVelocity = oRb.transpose() * oi_baseVelocity - bTi;

  vEstimate.head(3) = b_baseVelocity;
  vEstimate.segment(3, 3) = IMUAngularVelocity;
  vEstimate.tail(12) = vActuators;

  Vector3 basePosition = basePositionFK + feetPositionBarycenter;
  qEstimate.head(3) = positionFilter.compute(basePosition, oRb * b_baseVelocity, alphaPos);

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
  int window_size = get_windows_size();
  vFiltered(0) = std::accumulate(vx_queue.begin(), vx_queue.end(), 0.) / window_size;
  vFiltered(1) = std::accumulate(vy_queue.begin(), vy_queue.end(), 0.) / window_size;
  vFiltered(2) = std::accumulate(vz_queue.begin(), vz_queue.end(), 0.) / window_size;
}



