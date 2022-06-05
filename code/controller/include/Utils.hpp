
#ifndef UTILS_H_INCLUDED
#define UTILS_H_INCLUDED

#include <string>
#include "Types.h"
bool file_exists (const std::string& name);

Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw);
Vector3 quaternionToRPY(Eigen::Quaterniond quat);
Vector3 cross3(Vector3 left, Vector3 right);
bool array_equal(Vector4 a, Vector4 b);


class LowpassFilter  {

	public:

		LowpassFilter();
		~LowpassFilter() {};

		void initialize(double dT, const VectorN& fc, const VectorN& init);
		void initialize(double dT, double fc, const VectorN& init);

        /** compute complementary filter
         *  x quantity
         *  dx derivative of the quantity
         *  alpha overwrites the cut off frequency and represents the weight of the previous value x */
		VectorN compute(const VectorN& x);
		VectorN compute(const VectorN& x, const VectorN& dx);

	    /** patch the low pass value */
	    void patchLowPassed(int idx, double x);

	private:
	    VectorN alpha;
		VectorN filtered_x;	// low pass filtered x
};
#endif
