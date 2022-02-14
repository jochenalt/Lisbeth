/**
 * State estimation
 *
 */
#ifndef ESTIMATOR_H_INCLUDED
#define ESTIMATOR_H_INCLUDED

#include "qrw/Types.h"

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
	virtual ~Estimator() {};
};

#endif  // ESTIMATOR_H_INCLUDED
