/**
 * State estimation
 *
 */
#ifndef ESTIMATOR_H_INCLUDED
#define ESTIMATOR_H_INCLUDED

#include "Types.h"
#include "Utils.hpp"
#include "Params.hpp"

#include <deque>

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "ComplementaryFilter.hpp"
#include "Params.hpp"

class Estimator {
public:
	Estimator ();
	~Estimator() {};

	void initialize(Params& params );

	void updateForwardKinematics();
	/*
	 *
	 * gait Gait matrix that stores current and future contact status of the feet
	 * goals Target positions of the four feet
	 * baseLinearAcceleration Linear acceleration of the IMU (gravity compensated)
	 * baseAngularVelocity Angular velocity of the IMU
	 * baseOrientation Quaternion orientation of the IMU
	 * q_mes Position of the 12 actuators
	 * v_mes Velocity of the 12 actuators
	 * perfectPosition Position of the robot in world frame
	 * b_perfectVelocity Velocity of the robot in base frame
	 */
	void run( MatrixN4 gait, Matrix34 feetTargets,
			  Vector3 baseLinearAcceleration, Vector3 baseAngularVelocity, Vector3 baseOrientation,
			  Vector12 const& q, Vector12 const &v);

	// Update state vectors of the robot (q and v)
	// Update transformation matrices between world and horizontal frames
    /// joystick_v_ref Reference velocity from the joystick
	void updateReferenceState(VectorN const& vRef);

	// returns true if IMUs measurements of acceleration and velocity says that the system is steady
	bool isSteady();
	Vector3 getImuRPY() { return IMURpy; }

	Vector19 getQEstimate() { return qEstimate; }
	Vector18 getVEstimate() { return vEstimate; }
	Vector12 getVSecurity() { return vSecurity; }
	Vector4 getFeetStatus() { return feetStatus; }
	Matrix34 getFeetTargets() { return feetTargets; }
	Vector3 getBaseVelocityFK() { return baseVelocityFK; }
	Vector3 getBasePositionFK() { return basePositionFK; }
	Vector3 getFeetPositionBarycenter() { return feetPositionBarycenter; }
	Vector3 getBBaseVelocity() { return b_baseVelocity; }

	Vector3 getFilterVelX() { return velocityFilter.getX(); }
	Vector3 getFilterVelDX() { return velocityFilter.getDx(); }
	Vector3 getFilterVelAlpha() { return velocityFilter.getAlpha(); }
    Vector3 getFilterVelFiltX() { return velocityFilter.getFilteredX(); }
	Vector3 getFilterPosX() { return positionFilter.getX(); }
	Vector3 getFilterPosDX() { return positionFilter.getDx(); }
	Vector3 getFilterPosAlpha() { return positionFilter.getAlpha(); }
	Vector3 getFilterPosFiltX() { return positionFilter.getFilteredX(); }

	Vector18 getQReference() { return qRef; }
	Vector18 getVReference() { return vRef; }
	Vector6 getBaseVelRef() { return baseVelRef; }
	Vector6 getBaseAccRef() { return baseAccRef; }
	Vector6 getHV() { return h_v; }
	Vector6 getVFiltered() { return vFiltered; }
	Vector6 getHVFiltered() { return h_vFiltered; }
	Matrix3 getoRh() { return oRh; }
	Matrix3 gethRb() { return hRb; }
	Vector3 getoTh() { return oTh; }

private:

	// store and update IMU data
	// baseLinearAcceleration 	Linear acceleration of the IMU (gravity compensated)
	// baseAngularVelocity 		Angular velocity of the IMU
	// base_orientation 		RPY from IMU
	void updateIMUData(Vector3 base_linear_acc, Vector3 base_angular_velocity, Vector3 base_orientation);

	// Update the feet relative data
	// update feetStatus_, feetTargets_, feetStancePhaseDuration_ and phaseRemainingDuration_
	// gait Gait 			matrix that stores current and future contact status of the feet
	// feetTargets 			Target positions of the four feet
	void updatFeetStatus(MatrixN const& gait, MatrixN const& feetTargets);

	// Retrieve and update position and velocity of the 12 actuators
	//
	// q Position of the 12 actuators
	// v Velocity of the 12 actuators
	void updateJointData(Vector12 const& q, Vector12 const &v);

	// Compute barycenter of feet in contact
	//
	// feet_status Contact status of the four feet
	// goals Target positions of the four feet
	void computeFeetPositionBarycenter();

	// Compute the alpha coefficient for the complementary filter
	// return alpha
	double computeAlphaVelocity();

	// Estimates the position and velocity vector
	// The complementary filter combines data from the FK and the IMU acceleration data
	// The complementary filter combines data from the FK and the estimated velocity
	//
	void estimatePositionAndVelocity();


 	// Estimate the velocity of the base with forward kinematics using a contact point that
	// is supposed immobile in world frame
	//
	/// contactFrameId Frame ID of the contact foot
	Vector3 computeBaseVelocityFromFoot(int footId);

	// Estimate the position of the base with forward kinematics using a contact point that
	// is supposed immobile in world frame
	//
	// footId Frame ID of the contact foot
	Vector3 computeBasePositionFromFoot(int footId);

	// Filter the estimated velocity over a moving window
	void filterVelocity();

	// Number of samples in the averaging window
	int get_windows_size() { return  (int)(params->get_k_mpc() * params->get_N_steps() / params->N_periods); };


	Params* params;

	std::array<int, 4> feetFrames_ID;	// Frame indexes of the four feet
	double footRadius;      				// radius of a foot

	Vector3 alphaPos;							// Alpha coeefficient for the position complementary filter
	Vector3 alphaAcc;							// Alpha coeefficient for the acceleration complementary filter

	double alphaVelMax;     				// Maximum alpha value for the velocity complementary filter
	double alphaVelMin;     				// Minimum alpha value for the velocity complementary filter

	// double alpha;
	double alphaSecurity;   				// Low pass coefficient for the outputted filtered velocity for security check
	pinocchio::SE3 b_M_IMU;					// Transform between the base frame and the IMU frame


	double IMUYawOffset;						// Yaw orientation of the IMU at startup
	Vector3 IMULinearAcceleration;		// 	Linear acceleration of body (base frame) from IMU
	Vector3 IMUAngularVelocity;    		// 	angular velocity of body (base frame) from IMU
	Vector3 IMURpy;							// Roll Pitch Yaw orientation of the IMU
	Eigen::Quaterniond IMUQuat; 			// angular position as measured by IMU

	Vector12 qActuators; 					// positions of feet in the order FL, FR, HL, HR, as returned by device measurement
	Vector12 vActuators; 					// velocities of feet in the order FL, FR, HL, HR, as returned by device measurement

	int phaseRemainingDuration;			// Number of iterations left for the current gait phase
	Vector4 feetStancePhaseDuration;		// Number of loops during which each foot has been in contact

	Vector4 feetStatus;						// Contact status of the four feet, coming from gait
	Matrix34 feetTargets;					// Target positions of the four feet
	Vector19 q_FK;								// Configuration vector for Forward Kinematics
	Vector18 v_FK;								// Velocity vector for Forward Kinematics
	Vector3 baseVelocityFK;					// Base linear velocity estimated by Forward Kinematics
	Vector3 basePositionFK;					// Base position estimated by Forward Kinematics
	Vector3 b_baseVelocity; 			 	// Filtered estimated velocity at center base (base frame)
	Vector3 baseAcceleration; 				// filtered acceleration of base in world frame x',y',z'

	Vector3 feetPositionBarycenter; 		// Barycenter of feet in contact

	pinocchio::Model velocityModel, positionModel;	// Pinocchio models for frame computations and forward kinematics
	pinocchio::Data velocityData, positionData;		// Pinocchio datas for frame computations and forward kinematics

	ComplementaryFilter positionFilter;  // Complementary filter for base position
	ComplementaryFilter velocityFilter;  // Complementary filter for base velocity

	// stacked vector of
	// [0..2 ] = filt_lin_pos = filtered coord of base in world frame x,y,z
	// [3..6 ] = filt_ang_pos = filtered angular velocity as quaternion in x,y,z,w
	// [7..18] = actuators_pos = positions of feet in the order FL, FR, HL, HR as returned by device measurement
	Vector19 qEstimate;

	// stacked vector of
	// [0..2 ] filt_lin_vel, filtered velocity of base in world frame x',y',z'
	// [3..5 ] filt_ang_vel, filtered angular velocity around x,y,z
	// [6..17] actuators_vel, velocities of feet in the order FL, FR, HL, HR, as returned by device measurement
	Vector18 vEstimate;

	// filtered actuators_vel, velocities of feet in the order FL, FR, HL, HR, as returned by device measurement
	Vector12 vSecurity;

	Vector6 vFiltered;                                 // Base velocity (in base frame) filtered by averaging window
	std::deque<double> vx_queue, vy_queue, vz_queue;  	// Queues that hold samples

    Vector18 qRef;        // Configuration vector in ideal world frame
    Vector18 vRef;        // Velocity vector in ideal world frame
    Vector6 baseVelRef;   // Reference velocity vector
    Vector6 baseAccRef;   // Reference acceleration vector
    Matrix3 oRh;          // Rotation between horizontal and world frame
    Matrix3 hRb;          // Rotation between base and horizontal frame
    Vector3 oTh;          // Translation between horizontal and world frame
    Vector6 h_v;          // Velocity vector in horizontal frame
    Vector6 h_vFiltered;  // Base velocity (in horizontal frame) filtered by averaging window
};

#endif  // ESTIMATOR_H_INCLUDED
