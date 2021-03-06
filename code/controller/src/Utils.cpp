#include "Utils.hpp"
#include "Types.h"


#include <iostream>
#include <fstream>
#include <iostream>     // std::cout
#include <cmath>
#include <iostream>
#include <string>
#include <array>
#include <chrono>
#include <thread>
#include <cstdint>

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



uint64_t get_micros() {
	return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void delay_us(int us) {
	std::this_thread::sleep_for(std::chrono::microseconds(us));
}

void delay_ms(int ms) {
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
