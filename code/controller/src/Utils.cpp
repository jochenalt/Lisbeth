#include "qrw/Utils.hpp"
#include "qrw/Types.h"


#include <iostream>
#include <fstream>
#include <iostream>     // std::cout
#include <cmath>
#include <iostream>
#include <string>
#include <array>

using namespace std;
bool file_exists (const std::string& name) {
    std::ifstream f(name.c_str());
    return f.good();
}


Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw) {
	double sr = sin(roll/2.);
	double cr = cos(roll/2.);
	double sp = sin(pitch/2.);
    double cp = cos(pitch/2.);
    double sy = sin(yaw/2.);
    double cy = cos(yaw/2.);
    double qx = sr * cp * cy - cr * sp * sy;
    double qy = cr * sp * cy + sr * cp * sy;
	double qz = cr * cp * sy - sr * sp * cy;
	double qw = cr * cp * cy + sr * sp * sy;
    return Eigen::Quaterniond(qw, qx, qy, qz);
}

Vector3 quaternionToRPY(Eigen::Quaterniond quat) {
    double qx = quat.x();
    double qy = quat.y();
    double qz = quat.z();
    double qw = quat.w();

    double rotateXa0 = 2.0*(qy*qz + qw*qx);
	double rotateXa1 = qw*qw - qx*qx - qy*qy + qz*qz;
	double rotateX = 0.0;

    if ((rotateXa0 != 0.0) && (rotateXa1 != 0.0))
        rotateX = atan2(rotateXa0, rotateXa1);

    double rotateYa0 = -2.0*(qx*qz - qw*qy);
    double rotateY = 0.0;
    if (rotateYa0 >= 1.0)
        rotateY = M_PI/2.0;
    else
    	if (rotateYa0 <= -1.0)
    		rotateY = -M_PI/2.0;
    	else
    		rotateY = asin(rotateYa0);

    double rotateZa0 = 2.0*(qx*qy + qw*qz);
	double rotateZa1 = qw*qw + qx*qx - qy*qy - qz*qz;
	double rotateZ = 0.0;
    if ((rotateZa0 != 0.0) && (rotateZa1 != 0.0))
		rotateZ = atan2(rotateZa0, rotateZa1);
    return Vector3 ({rotateX, rotateY, rotateZ});
}

// cross product of two Vector3
Vector3 cross3(Vector3 left, Vector3 right) {
    return Vector3(left[1] * right[2] - left[2] * right[1],
    		       left[2] * right[0] - left[0] * right[2],
				   left[0] * right[1] - left[1] * right[0]);
}

Vector3 scalarproduct(Vector3 left, Vector3 right) {
    return Vector3(left[1] * right[2] - left[2] * right[1],
    		       left[2] * right[0] - left[0] * right[2],
				   left[0] * right[1] - left[1] * right[0]);
}
// returns true, if size and values of both vectors are quite the same
bool array_equal(Vector4 a, Vector4 b) {
	const double eps = 0.0000001;
	if (a.size() != b.size())
		return false;
	for (int i = 0;i<a.size();i++)
		if (abs(a[i]- b[i]) > eps)
			return false;
	return true;
}


ComplementaryFilter::ComplementaryFilter(){
	dT = 0;
	cut_off_freq = 0;
    alpha = 0;
	x = VectorN::Zero(3);
    dx = VectorN::Zero(3);
    highpassed_x = VectorN::Zero(3);
    lowpassed_x = VectorN::Zero(3);
}

void ComplementaryFilter::initialize(double par_dT, double par_fc){

	dT = par_dT;
	cut_off_freq = par_fc;

	double y = 1.0 - cos(2*M_PI*cut_off_freq*dT);
    alpha = -y+ sqrt(y*y+2*y);

}

VectorN ComplementaryFilter::compute(const VectorN& par_x, const VectorN& par_dx, const VectorN& alpha) {
	// for logging
	x = par_x;
	dx = par_dx;

	// do high pass filter
	highpassed_x = alpha.cwiseProduct(highpassed_x + dx * dT);

	// do low pass filter

	VectorN negAlpha = VectorN::Constant(x.rows(), 1, 1.0) - alpha;
	lowpassed_x = alpha.cwiseProduct(lowpassed_x) + negAlpha.cwiseProduct(x);

	// add both filter
	VectorN result = highpassed_x + lowpassed_x;

	return result;
}

VectorN ComplementaryFilter::compute(const VectorN& par_x, const VectorN& par_dx, double par_alpha) {
	if (alpha != NAN)
		alpha = par_alpha;

	// for logging
	x = par_x;
	dx = par_dx;

	// do high pass filter
	highpassed_x = alpha * (highpassed_x + dx * dT);

	// do low pass filter
	lowpassed_x = alpha * lowpassed_x + (1.0 - alpha) * x;

	// add both filter
	VectorN result = highpassed_x + lowpassed_x;

	return result;
}

void ComplementaryFilter::patchLowPassed(int idx, double x) {
	lowpassed_x[idx] = x;
}

