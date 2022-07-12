#include "ComplementaryFilter.hpp"

ComplementaryFilter::ComplementaryFilter()
    : dt(0.),
      HighPass(Vector3::Zero()),
      LowPass(Vector3::Zero())
{
}

ComplementaryFilter::ComplementaryFilter(double dt_in, Vector3 HighPass_in, Vector3 LowPass_in)
    : dt(dt_in),
      HighPass(HighPass_in),
      LowPass(LowPass_in)
{
}

void ComplementaryFilter::initialize(double dt_in, Vector3 HighPass_in, Vector3 LowPass_in) {
  dt = dt_in;
  HighPass = HighPass_in;
  LowPass = LowPass_in;
}

Vector3 ComplementaryFilter::compute(Vector3 const& x_in, Vector3 const& dx_in, Vector3 const& alpha_in) {
  HighPass = alpha_in.cwiseProduct(HighPass + dx_in * dt);
  LowPass = alpha_in.cwiseProduct(LowPass) + (Vector3::Ones() - alpha_in).cwiseProduct(x_in);
  Vector3 filteredX = HighPass + LowPass;

  return filteredX;
}
