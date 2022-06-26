#include "Filter.hpp"

Filter::Filter()
    : b(0.),
      a(Vector2::Zero()),
      x(Vector6::Zero()),
      y(VectorN::Zero(6, 1)),
      accum(Vector6::Zero()),
      init_(false) {
}

void Filter::initialize(Params& params) {
  const double fc = 15.0;
  b = (2 * M_PI * params.dt_wbc * fc) / (2 * M_PI * params.dt_wbc * fc + 1.0);

  a << 1.0, -(1.0 - b);

  x_queue.resize(1, Vector6::Zero());
  y_queue.resize(a.rows() - 1, Vector6::Zero());
}

VectorN Filter::filter(Vector6 const& x_in, bool check_modulo) {
  // Retrieve measurement
  x = x_in;

  // Handle modulo for orientation
  if (check_modulo) {
    // Handle 2 pi modulo for roll, pitch and yaw
    // Should happen sometimes for yaw but now for roll and pitch except
    // if the robot rolls over
    for (int i = 3; i < 6; i++) {
      if (std::abs(x(i, 0) - y(i, 0)) > 1.5 * M_PI) {
        handle_modulo(i, x(i, 0) - y(i, 0) > 0);
      }
    }
  }

  // Initialisation of the value in the queues to the first measurement
  if (!init_) {
    init_ = true;
    std::fill(x_queue.begin(), x_queue.end(), x.head(6));
    std::fill(y_queue.begin(), y_queue.end(), x.head(6));
  }

  // Store measurement in x queue
  x_queue.pop_back();
  x_queue.push_front(x.head(6));

  // Compute result (y/x = b/a for the transfert function)
  accum = b * x_queue[0];
  for (int i = 1; i < a.rows(); i++) {
    accum -= a[i] * y_queue[i - 1];
  }

  // Store result in y queue for recursion
  y_queue.pop_back();
  y_queue.push_front(accum / a[0]);

  // Filtered result is stored in y_queue_.front()
  // Assigned to dynamic-sized vector for binding purpose
  y = y_queue.front();

  return y;
}

void Filter::handle_modulo(int a_in, bool dir) {
  // Add or remove 2 PI to all elements in the queues
  x_queue[0](a_in, 0) += dir ? 2.0 * M_PI : -2.0 * M_PI;
  for (int i = 1; i < a.rows(); i++) {
    (y_queue[i - 1])(a_in, 0) += dir ? 2.0 * M_PI : -2.0 * M_PI;
  }
}
