
#ifndef UTILS_H_INCLUDED
#define UTILS_H_INCLUDED

#include <string>
#include "Types.h"
bool file_exists (const std::string& name);

Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw);
Vector3 quaternionToRPY(Eigen::Quaterniond quat);
Vector3 cross3(Vector3 left, Vector3 right);
bool array_equal(Vector4 a, Vector4 b);



uint64_t get_micros();
#endif
