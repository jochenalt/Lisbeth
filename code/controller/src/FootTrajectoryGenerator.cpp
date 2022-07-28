#include "FootTrajectoryGenerator.hpp"


FootTrajectoryGenerator::FootTrajectoryGenerator()
    : gait(NULL),
      feet(Eigen::Matrix<int, 1, 4>::Zero()),
      t0s(Vector4::Zero()),
      t_swing(Vector4::Zero()),
      targetFootstep(Matrix34::Zero()),
      Ax(Matrix64::Zero()),
      Ay(Matrix64::Zero()),
      position(Matrix34::Zero()),
      velocity(Matrix34::Zero()),
      acceleration(Matrix34::Zero()) {
}

void FootTrajectoryGenerator::initialize(Params &params_in, GaitPlanner &gaitIn) {
  params = &params_in;
  targetFootstep << Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(params->footsteps_init.data(),
                                                                  params->footsteps_init.size());
  position = targetFootstep;
  gait = &gaitIn;
}

void FootTrajectoryGenerator::updateFootPosition(int const j, Vector3 const &targetFootstep_in) {
  double ddx0 = acceleration(0, j);
  double ddy0 = acceleration(1, j);
  double dx0 = velocity(0, j);
  double dy0 = velocity(1, j);
  double x0 = position(0, j);
  double y0 = position(1, j);

  double t = t0s[j] - params->vert_time;
  double d = t_swing[j] - 2 * params->vert_time;
  double dt = params->dt_wbc;

  if (t < d - params->lock_time) {
	double t2 = t*t;
	double t3 = t2*t;
	double t4 = t2*t2;
	double t5 = t2*t3;

	double d2 = d*d;
	double d3 = d2*d;
	double d4 = d2*d2;
	double d5 = d2*d3;
	double t_d2 = (t-d)*(t-d);

    // compute polynoms coefficients for x and y
    Ax(0, j) = (ddx0 * t2 - 2 * ddx0 * t * d - 6 * dx0 * t + ddx0 * d2 + 6 * dx0 * d +
                12 * x0 - 12 * targetFootstep_in[0]) /
                (2 * t_d2 *
                (t3 - 3 * t2 * d + 3 * t * d2 - d3));
    Ax(1, j) =  (30 * t * targetFootstep_in[0] - 30 * t * x0 - 30 * d * x0 + 30 * d * targetFootstep_in[0] -
   		 	 	  2 * t3 * ddx0 - 3 * d3 * ddx0 + 14 * t2 * dx0 -
					  16 * d2 * dx0 + 2 * t * d * dx0 + 4 * t * d2 * ddx0 + t2 * d * ddx0) /
					 (2 * t_d2 * (t3 - 3 * t2 * d + 3 * t * d2 - d3));
    Ax(2, j) = (t4 * ddx0 + 3 * d4 * ddx0 - 8 * t3 * dx0 +
                12 * d3 * dx0 + 20 * t2 * x0 - 20 * t2 * targetFootstep_in[0] +
                20 * d2 * x0 - 20 * d2 * targetFootstep_in[0] + 80 * t * d * x0 -
                80 * t * d * targetFootstep_in[0] + 4 * t3 * d * ddx0 + 28 * t * d2 * dx0 -
                32 * t2 * d * dx0 - 8 * t2 * d2 * ddx0) /
               (2 * t_d2 *
                (t3 - 3 * t2 * d + 3 * t * d2 - d3));
    Ax(3, j) = -(d5 * ddx0 + 4 * t * d4 * ddx0 + 3 * t4 * d * ddx0 +
                 36 * t * d3 * dx0 - 24 * t3 * d * dx0 + 60 * t * d2 * x0 +
                 60 * t2 * d * x0 - 60 * t * d2 * targetFootstep_in[0] -
                 60 * t2 * d * targetFootstep_in[0] - 8 * t2 * d3 * ddx0 -
                 12 * t2 * d2 * dx0) /
                (2 * (t2 - 2 * t * d + d2) * (t3 - 3 * t2 * d + 3 * t * d2 - d3));
    Ax(4, j) = -(2 * d5 * dx0 - 2 * t * d5 * ddx0 - 10 * t * d4 * dx0 +
                 t2 * d4 * ddx0 + 4 * t3 * d3 * ddx0 -
                 3 * t4 * d2 * ddx0 - 16 * t2 * d3 * dx0 +
                 24 * t3 * d2 * dx0 - 60 * t2 * d2 * x0 +
                 60 * t2 * d2 * targetFootstep_in[0]) /
                (2 * t_d2 * (t3 - 3 * t2 * d + 3 * t * d2 - d3));
    Ax(5, j) = (2 * targetFootstep_in[0] * t5 - ddx0 * t4 * d3 -
                10 * targetFootstep_in[0] * t4 * d + 2 * ddx0 * t3 * d4 +
                8 * dx0 * t3 * d3 + 20 * targetFootstep_in[0] * t3 * d2 -
                ddx0 * t2 * d5 - 10 * dx0 * t2 * d4 -
                20 * x0 * t2 * d3 + 2 * dx0 * t * d5 +
                10 * x0 * t * d4 - 2 * x0 * d5) /
               (2 * (t2 - 2 * t * d + d2) *
                (t3 - 3 * t2 * d + 3 * t * d2 - d3));
    Ay(0, j) = (ddy0 * t2 - 2 * ddy0 * t * d - 6 * dy0 * t + ddy0 * d2 + 6 * dy0 * d +
                12 * y0 - 12 * targetFootstep_in[1]) /
               (2 * t_d2 *
                (t3 - 3 * t2 * d + 3 * t * d2 - d3));
    Ay(1, j) = (30 * t * targetFootstep_in[1] - 30 * t * y0 - 30 * d * y0 + 30 * d * targetFootstep_in[1] -
   		 	 	 2 * t3 * ddy0 - 3 * d3 * ddy0 + 14 * t2 * dy0 -
					 16 * d2 * dy0 + 2 * t * d * dy0 + 4 * t * d2 * ddy0 + t2 * d * ddy0) /
					(2 * t_d2 * (t3 - 3 * t2 * d + 3 * t * d2 - d3));
    Ay(2, j) = (t4 * ddy0 + 3 * d4 * ddy0 - 8 * t3 * dy0 +
                12 * d3 * dy0 + 20 * t2 * y0 - 20 * t2 * targetFootstep_in[1] +
                20 * d2 * y0 - 20 * d2 * targetFootstep_in[1] + 80 * t * d * y0 -
                80 * t * d * targetFootstep_in[1] + 4 * t3 * d * ddy0 + 28 * t * d2 * dy0 -
                32 * t2 * d * dy0 - 8 * t2 * d2 * ddy0) /
               (2 * t_d2 * (t3 - 3 * t2 * d + 3 * t * d2 - d3));
    Ay(3, j) = -(d5 * ddy0 + 4 * t * d4 * ddy0 + 3 * t4 * d * ddy0 +
                 36 * t * d3 * dy0 - 24 * t3 * d * dy0 + 60 * t * d2 * y0 +
                 60 * t2 * d * y0 - 60 * t * d2 * targetFootstep_in[1] -
                 60 * t2 * d * targetFootstep_in[1] - 8 * t2 * d3 * ddy0 -
                 12 * t2 * d2 * dy0) / (2 * (t2 - 2 * t * d + d2) *
                (t3 - 3 * t2 * d + 3 * t * d2 - d3));
    Ay(4, j) = -(2 * d5 * dy0 - 2 * t * d5 * ddy0 - 10 * t * d4 * dy0 +
                 t2 * d4 * ddy0 + 4 * t3 * d3 * ddy0 -
                 3 * t4 * d2 * ddy0 - 16 * t2 * d3 * dy0 +
                 24 * t3 * d2 * dy0 - 60 * t2 * d2 * y0 +
                 60 * t2 * d2 * targetFootstep_in[1]) /
               (2 * t_d2 * (t3 - 3 * t2 * d + 3 * t * d2 - d3));
    Ay(5, j) = (2 * targetFootstep_in[1] * t5 - ddy0 * t4 * d3 -
                10 * targetFootstep_in[1] * t4 * d + 2 * ddy0 * t3 * d4 +
                8 * dy0 * t3 * d3 + 20 * targetFootstep_in[1] * t3 * d2 -
                ddy0 * t2 * d5 - 10 * dy0 * t2 * d4 -
                20 * y0 * t2 * d3 + 2 * dy0 * t * d5 +
                10 * y0 * t * d4 - 2 * y0 * d5) /
               (2 * (t2 - 2 * t * d + d2) *
                (t3 - 3 * t2 * d + 3 * t * d2 - d3));

    targetFootstep(0, j) = targetFootstep_in[0];
    targetFootstep(1, j) = targetFootstep_in[1];
  }

  // Coefficients for z (deterministic)
  double Tz = t_swing[j];
  double Tzh = Tz/2;
  double Tzh2 = Tzh*Tzh;
  double Tzh6 = Tzh2*Tzh2*Tzh2;

  Vector4 Az;
  Az(0, j) = - params->max_height / (Tzh6);
  Az(1, j) = (3 * Tz *  params->max_height) / (Tzh6 );
  Az(2, j) = -(3 * Tz*Tz *  params->max_height) / (Tzh6 );
  Az(3, j) = (Tz*Tz*Tz*  params->max_height) / (Tzh6 );

  // Get the next point
  double ev = t + dt;
  double evz = t0s[j] + dt;

  if (t < 0.0 || t >= d)  // Just vertical motion
  {
    position(0, j) = x0;
    position(1, j) = y0;
    velocity(0, j) = 0.0;
    velocity(1, j) = 0.0;
    acceleration(0, j) = 0.0;
    acceleration(1, j) = 0.0;
  } else {

	 double ev2 = ev*ev;
	 double ev3 = ev2*ev;
	 double ev4 = ev2*ev2;
	 double ev5 = ev2*ev3;
    position(0, j) = Ax(5, j) + Ax(4, j) * ev + Ax(3, j) * ev2 + Ax(2, j) * ev3 + Ax(1, j) * ev4+ Ax(0, j) * ev5;
    position(1, j) = Ay(5, j) + Ay(4, j) * ev + Ay(3, j) * ev2 + Ay(2, j) * ev3 + Ay(1, j) * ev4+ Ay(0, j) * ev5;
    velocity(0, j) = Ax(4, j) + 2 * Ax(3, j) * ev + 3 * Ax(2, j) * ev2 + 4 * Ax(1, j) * ev3 + 5 * Ax(0, j) * ev4;
    velocity(1, j) = Ay(4, j) + 2 * Ay(3, j) * ev + 3 * Ay(2, j) * ev2 + 4 * Ay(1, j) * ev3 + 5 * Ay(0, j) * ev4;
    acceleration(0, j) = 2 * Ax(3, j) + 3 * 2 * Ax(2, j) * ev + 4 * 3 * Ax(1, j) * ev2 + 5 * 4 * Ax(0, j) * ev3;
    acceleration(1, j) = 2 * Ay(3, j) + 3 * 2 * Ay(2, j) * ev + 4 * 3 * Ay(1, j) * ev2 + 5 * 4 * Ay(0, j) * ev3;
  }
  double evz2 = evz*evz;
  double evz3 = evz2*evz;
  double evz4 = evz2*evz2;
  double evz5 = evz2*evz3;
  double evz6 = evz3*evz3;

  velocity(2, j) = 3 * Az(3, j) * evz2 + 4 * Az(2, j) * evz3 + 5 * Az(1, j) * evz4 + 6 * Az(0, j) * evz5;
  acceleration(2, j) = 2 * 3 * Az(3, j) * evz + 3 * 4 * Az(2, j) * evz2 + 4 * 5 * Az(1, j) * evz3 + 5 * 6 * Az(0, j) * evz4;
  position(2, j) = Az(3, j) * evz3 + Az(2, j) * evz4 + Az(1, j) * evz5 + Az(0, j) * evz6;
}

void FootTrajectoryGenerator::update(bool startNewCycle, MatrixN const &targetFootstep) {
  if (startNewCycle) {
    // Status of feet
    feet = gait->getCurrentGaitMatrix().row(0).cast<int>();

    // If no foot in swing phase do nothing
    if (feet.sum() == 4)
   	 return;

    // For each foot in swing phase get remaining duration of the swing phase
    for (int i = 0; i < 4; i++) {
      if (feet(0, i) == 0) {
        t_swing[i] = gait->getPhaseDuration(0, i);
        t0s[i] = std::max(0.0, gait->getElapsedTime(0, i));
      }
    }
  } else {
    // If no foot in swing phase
    if (feet.sum() == 4) return;

    // Increment of one time step for feet in swing phase
    for (int i = 0; i < 4; i++) {
      if (feet(0, i) == 0) {
        t0s[i] = std::max(0.0, t0s[i] + params->dt_wbc);
      }
    }
  }

  for (int i = 0; i < 4; i++) {
    if (feet(0, i) == 0) {
      updateFootPosition(i, targetFootstep.col(i));
    }
  }
}
