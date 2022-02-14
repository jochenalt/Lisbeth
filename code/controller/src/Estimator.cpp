#include <math.h>

#include "qrw/Estimator.hpp"

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

VectorN ComplementaryFilter::compute(const VectorN& par_x, const VectorN& par_dx, double par_alpha) {
	if (alpha != no_alpha)
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


