#include "inputShaping.h"

scurve generate_scurve(const double start_point, const double end_point, double speed_limit,
                       double acceleration_limit, const double dt) {
  double distance = end_point - start_point;
  // Adapt limits to direction
  int direction = (distance > 0) ? 1 : -1;
  acceleration_limit *= direction;
  speed_limit *= direction;

  // Calculate the time to reach the maximum speed
  distance = std::abs(distance);
  double time_to_max_speed = std::abs(speed_limit / acceleration_limit);
  double distance_accel = 0.5 * std::abs(acceleration_limit) * std::pow(time_to_max_speed, 2);

  double max_speed_reachable = 0.0;
  if (distance <= 2 * distance_accel) {
    // The distance is too short to reach the maximum speed
    max_speed_reachable = std::sqrt(distance * std::abs(acceleration_limit) / 2);
    time_to_max_speed = max_speed_reachable / std::abs(acceleration_limit);
    distance_accel = 0.5 * std::abs(acceleration_limit) * std::pow(time_to_max_speed, 2);
    max_speed_reachable *= direction;
  } else {
    // The distance is long enough to reach the maximum speed
    max_speed_reachable = speed_limit;
  }

  // Calculate the time to reach the maximum speed
  double distance_const_speed = distance - 2 * distance_accel;
  double time_const_speed = distance_const_speed / std::abs(max_speed_reachable);
  double time_total = 2 * time_to_max_speed + time_const_speed;

  // Initialize vectors
  vec_double s, v, a, time;
  double current_time = 0.0, current_accel = 0.0, current_position = start_point , current_speed = 0.0;

  while (current_time <= time_total + dt) {
    if (current_time <= time_to_max_speed) {
      current_accel = acceleration_limit;
      current_speed = current_accel * current_time;
    } else if (current_time <= time_to_max_speed + time_const_speed) {
      current_accel = 0.0;
      current_speed = max_speed_reachable;
    } else {
      current_accel = -acceleration_limit;
      double time_deceleration = current_time - (time_to_max_speed + time_const_speed);
      current_speed = max_speed_reachable + current_accel * time_deceleration;
    }

    // Ensure speed does not go beyond zero in the final adjustment
    if (current_time > time_total && (current_speed * direction) < 0) {
      current_speed = 0;
      current_accel = 0;
    }
    current_position += current_speed * dt;
    s.push_back(current_position);
    v.push_back(current_speed);
    a.push_back(current_accel);
    time.push_back(current_time);
    current_time += dt;
  }
  return std::make_tuple(time,s, v, a);
}

vec_double generate_impulse_from_signal(cvec_double &signal) {
  vec_double impulse(signal.size(), 0);
  impulse[0] = signal[0];
  for (std::size_t i = 1; i < signal.size(); i++) {
    impulse[i] = signal[i] - signal[i - 1];
  }
  return impulse;
}

traj generate_traj_from_impulses(cvec_double &impulse, cvec_double &time) {
  std::size_t n = impulse.size();
  vec_double s(n, 0), v(n, 0), a(n, 0);
  a[0] = impulse[0];
  for (std::size_t i = 1; i < n; i++) {
    double time_step = time[1] - time[0];
    a[i] = a[i - 1] + impulse[i];
    v[i] = v[i - 1] + a[i - 1] * time_step;
    s[i] = s[i - 1] + v[i] * time_step;
  }
  return std::make_tuple(s, v, a);
}

std::pair<vec_double, vec_double> apply_zero_vibration_shaping(cvec_double &impulse, vec_double time,
                                                               double kappa, double Tv) {
  double kappa0 = 1.0 / (1.0 + kappa);
  double kappa1 = kappa * kappa0;
  double time_step = time[1] - time[0];
  std::size_t additional_elements = std::size_t(Tv / time_step)+1;

  vec_double zv_impulse(impulse.size() + additional_elements, 0);
  double nextValue = time.back() + time_step;  // Calculate the next value to add
  for (std::size_t i = 0; i < additional_elements; ++i, nextValue += time_step) time.push_back(nextValue);

  for (std::size_t i = 0; i < zv_impulse.size(); i++) {
    if (i < impulse.size()) zv_impulse[i] = impulse[i] * kappa0;
    if (i >= additional_elements) zv_impulse[i] += kappa1 * impulse[i - additional_elements];
  }
  return std::make_pair(zv_impulse, time);
}
