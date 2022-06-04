
#ifndef UTILS_H_INCLUDED
#define UTILS_H_INCLUDED

#include <string>
#include "Types.h"
bool file_exists (const std::string& name);

Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw);
Vector3 quaternionToRPY(Eigen::Quaterniond quat);
Vector3 cross3(Vector3 left, Vector3 right);
bool array_equal(Vector4 a, Vector4 b);


/**
 * Complementary filter for vectors.
 * Used to filter body velocity and position in x,y,z.
 * Computation uses x and dx, since in both cases the derivative is already available and does not need to be
 * calculated multiple times.
 */
class CompFilter  {

	public:

	CompFilter();
		~CompFilter() {};

		void initialize(double dT, const VectorN& fc);

        /** compute complementary filter
         *  x quantity
         *  dx derivative of the quantity
         *  alpha overwrites the cut off frequency and represents the weight of the previous value x */
		VectorN compute(const VectorN& x);
		VectorN compute(const VectorN& x, const VectorN& dx);
	    VectorN compute(const VectorN& x, const VectorN& dx, const VectorN& alpha);

	    /** patch the low pass value */
	    void patchLowPassed(int idx, double x);


	private:
	    double dT;				 // time step of the filter [s]

	    VectorN alpha;
		VectorN x; 				// x of last call
		VectorN dx;				// dx of last call
		VectorN highpassed_x;	// high pass filtered x
		VectorN lowpassed_x;	// low pass filtered x
};

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
