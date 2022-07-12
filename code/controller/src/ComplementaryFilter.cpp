#include "ComplementaryFilter.hpp"

ComplementaryFilter::ComplementaryFilter()
    : dt(0.),
      HighPass(Vector3::Zero()),
      LowPass(Vector3::Zero()),
      alpha(Vector3::Zero())
{
}

ComplementaryFilter::ComplementaryFilter(double dt_in, Vector3 HighPass_in, Vector3 LowPass_in)
    : dt(dt_in),
      HighPass(HighPass_in),
      LowPass(LowPass_in),
      alpha(Vector3::Zero())
{
}

void ComplementaryFilter::initialize(double dt_in, Vector3 HighPass_in, Vector3 LowPass_in) {
  dt = dt_in;
  HighPass = HighPass_in;
  LowPass = LowPass_in;
}

Vector3 ComplementaryFilter::compute(Vector3 const& x_in, Vector3 const& dx_in, Vector3 const& alpha_in) {
  alpha = alpha_in;
  Vector3 x = x_in;
  Vector3  dx = dx_in;

  HighPass = alpha.cwiseProduct(HighPass + dx * dt);
  LowPass = alpha.cwiseProduct(LowPass) + (Vector3::Ones() - alpha).cwiseProduct(x);
  Vector3 filteredX = HighPass + LowPass;

  return filteredX;
}
