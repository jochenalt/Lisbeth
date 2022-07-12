
#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include "Types.h"

class ComplementaryFilter {
 public:
  ComplementaryFilter();
  ~ComplementaryFilter() {};

  // dt Time 	step of the complementary filter
  // HighPass 	Initial value for the high pass filter
  // LowPass 	Initial value for the low pass filter
  ComplementaryFilter(double dt, Vector3 HighPass, Vector3 LowPass);

  //	dt Time step of the complementary filter
  //	HighPass Initial value for the high pass filter
  //	LowPass Initial value for the low pass filter
  void initialize(double dt, Vector3 HighPass, Vector3 LowPass);

  // Compute the filtered output of the complementary filter
  // 	x Quantity handled by the filter
  //	dx Derivative of the quantity
  //	alpha Filtering coefficient between x and dx quantities
  Vector3 compute(Vector3 const& x, Vector3 const& dx, Vector3 const& alpha);
 private:
  double dt;          // Time step of the complementary filter
  Vector3 HighPass;   // Initial value for the high pass filter
  Vector3 LowPass;    // Initial value for the low pass filter
  Vector3 alpha;      // Filtering coefficient between x and dx quantities
  Vector3 x;          // Quantity to filter
  Vector3 dx;         // Quantity to filter derivative's
};

#endif  // COMPLEMENTARY_FILTER_H
