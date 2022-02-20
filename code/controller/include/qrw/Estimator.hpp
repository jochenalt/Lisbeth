/**
 * State estimation
 *
 */
#ifndef ESTIMATOR_H_INCLUDED
#define ESTIMATOR_H_INCLUDED

#include "qrw/Types.h"
#include "qrw/Utils.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include "pinocchio/math/rpy.hpp"
#include "pinocchio/spatial/explog.hpp"

using namespace pinocchio;

/**
 * Complementary filter for vectors.
 * Used to filter body velocity and position in x,y,z.
 * Computation uses x and dx, since in both cases the derivative is already available and does not need to be
 * calculated multiple times.
 */
const double no_alpha = -1.0;
class ComplementaryFilter  {

	public:

		ComplementaryFilter();
		~ComplementaryFilter() {};

		void initialize(double dT, double fc);

        /** compute complementary filter
         *  x quantity
         *  dx derivative of the quantity
         *  alpha overwrites the cut off frequency and represents the weight of the previous value x */
	    VectorN compute(const VectorN& x, const VectorN& dx, double alpha = no_alpha);

	    /** patch the low pass value */
	    void patchLowPassed(int idx, double x);


	private:
	    double dT;				 // time step of the filter [s]
	    double cut_off_freq; 	// cut off frequency of the filter [Hz]

	    double alpha;			// alpha used for filtering, calculated out of cut-off frequency
		VectorN x; 				// x of last call
		VectorN dx;				// dx of last call
		VectorN highpassed_x;	// high pass filtered x
		VectorN lowpassed_x;	// low pass filtered x
};


class Estimator {
public:
	Estimator ();
	~Estimator() {};

	void initialize(double dT, int N_simulation, double h_init=0.22294615, bool kf_enabled = false, bool perfectEstimator = false);

	// take data from IMU and tell estimator
	void set_imu_data(Vector3 base_linear_acc, Vector3 base_angular_velocity, Vector4 base_orientation);

	void set_data_joints  (Vector12 q_mes, Vector12 v_mes);
	Vector3 baseVelocityFromKinAndIMU(int contactFrameId);
	void get_data_FK(Vector4 feet_status);
	void get_xyz_feet(Vector4 feet_status, Matrix34 goals);
	void run_filter(int k, MatrixN gait, MatrixN goals);
private:


	double dt;
	bool perfectEstimator;
	double alpha;
	double alpha_v;
	double alpha_secu;
	double kf_enabled;

	ComplementaryFilter filter_xyz_vel;
	ComplementaryFilter filter_xyz_pos;

	double offset_yaw_IMU;
	Vector3 IMU_lin_acc;
	Vector3 IMU_ang_vel;
	Eigen::Quaterniond IMU_ang_pos;
	Vector3 FK_lin_vel;
	Vector3 FK_xyz;
	Vector3 xyz_mean_feet;
	bool close_from_contact;
	Vector4 feet_status;
	Matrix34 feet_goals;
	Vector4 k_since_contact;
	Vector3 HP_lin_vel;
	Vector3 LP_lin_vel;

	Vector3 o_filt_lin_vel;
	Vector3 filt_lin_vel;
	Vector3 filt_lin_pos;
	Vector3 filt_ang_vel;
	Eigen::Quaterniond filt_ang_pos;

	Model model;
	Data data;

	Model model_for_xyz;
	Data data_for_xyz;

	Vector19 q_filt;
	Vector18 v_filt;
	Vector12 v_secu;
	Vector19 q_FK;
	Vector18 v_FK;

	std::array<int, 4> indexes;
	Vector12 actuators_pos;
	Vector12 actuators_vel;

	Matrix3N rotated_FK;
	int k_log;
	Vector3 debug_o_lin_vel;

	pinocchio::SE3 _1Mi;

	// IMU data
	Vector3 RPY;
};

#endif  // ESTIMATOR_H_INCLUDED
