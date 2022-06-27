/**
 * This class applies a low pass filter to estimated data to avoid keeping high frequency components into
 *  what is given to the "low frequency" model predictive control
 */

#ifndef FILTER_H_INCLUDED
#define FILTER_H_INCLUDED

#include <deque>

#include "Params.hpp"

class Filter
{
public:
	Filter();
	~Filter() {};

	// Initialize with given data
	//	params Object that stores parameters
	void initialize(Params &params);


	// Run one iteration of the filter and return the filtered measurement
	//	x Quantity to filter
	//	check_modulo Check for the +-pi modulo of orientation if true
	VectorN filter(Vector6 const &x, bool check_modulo);

	// Add or remove 2 PI to all elements in the queues to handle +- pi modulo
	//	a Angle that needs change (3, 4, 5 for Roll, Pitch, Yaw respectively)
	//	dir Direction of the change (+pi to -pi or -pi to +pi)
	void handle_modulo(int a, bool dir);

	VectorN getFilt() { return y;	};

private:
	double b;       // Denominator coefficients of the filter transfer function
	Vector2 a;      // Numerator coefficients of the filter transfer function
	Vector6 x;      // Latest measurement
	VectorN y;      // Latest result
	Vector6 accum;  // Used to compute the result (accumulation)

	std::deque<Vector6> x_queue; // Store the last measurements for product with denominator coefficients
	std::deque<Vector6> y_queue; // Store the last results for product with numerator coefficients

	bool init_;  // Initialisation flag
};

#endif  // FILTER_H_INCLUDED
