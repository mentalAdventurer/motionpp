#include "model.h"

MotionPrimitives::MotionPrimitives() {
  namespace p = param;
  double kappa0 = 1.0 / (1.0 + p::kappa);
  double kappa1 = p::kappa * kappa0;
  jerk_impuls.reserve(accel.size());
  time_impuls.reserve(accel.size());

  for (auto& a : accel) {
    double delta_t = std::abs(a / p::j_max);
    if (a == 0) {
      time_impuls.push_back({0, 0, 0, 0.05});
      jerk_impuls.push_back({0, 0, 0, 0});
    } else if (p::Tv < delta_t) {
      time_impuls.push_back({0, p::Tv, delta_t, p::Tv + delta_t});
      jerk_impuls.push_back({p::j_max * kappa0, p::j_max * kappa1, -p::j_max * kappa0, -p::j_max * kappa1});
      if (a < 0) for(auto& jerk : jerk_impuls.back()) jerk *= -1;
    } else {
      time_impuls.push_back({0, delta_t, p::Tv, delta_t + p::Tv});
      jerk_impuls.push_back({p::j_max * kappa0, -p::j_max * kappa0, p::j_max * kappa1, -p::j_max * kappa1});
      if (a < 0) for(auto& jerk : jerk_impuls.back()) jerk *= -1;
    }
  }
}

std::vector<primitiveTuple> MotionPrimitives::operator()(const cspace::state_t x) {
  std::vector<primitiveTuple> primitives;
  std::size_t dim = x.size();
  for (std::size_t i = 0; i < jerk_impuls.size(); i++) {
    for (std::size_t j = 0; j < jerk_impuls.size(); j++) {
      double time_end;
      // Get the maximum time_end
      if (time_impuls[i].back() > time_impuls[j].back())
        time_end = time_impuls[i].back();
      else
        time_end = time_impuls[j].back();

      auto [time_x, state_x, input_x] =
          calculate_zv_from_impuls(std::span(x.begin(), x.begin() + dim / 2), jerk_impuls[i], time_impuls[i],
                                   this->time_step, time_end);
      auto [time_y, state_y, input_y] = calculate_zv_from_impuls(
          std::span(x.begin() + dim / 2, x.end()), jerk_impuls[j], time_impuls[j], this->time_step, time_end);

      // Creeate Primitive
      std::vector<double> traj, input;
      traj.reserve(state_x.size() * 2);
      for (std::size_t k = 0; k < state_x.size(); k += dim/2) {
        // Concatenate state_x and state_y
        traj.insert(traj.end(), state_x.begin() + k, state_x.begin() + k + dim / 2);
        traj.insert(traj.end(), state_y.begin() + k, state_y.begin() + k + dim / 2);
      }
      for (std::size_t k = 0; k < input_x.size(); k++) {
        input.push_back(input_x[k]);
        input.push_back(input_y[k]);
      }
      primitives.push_back({std::move(time_x), std::move(traj), std::move(input)});
    }
  }

  return primitives;
}

ZVStateInputTimeTuple MotionPrimitives::calculate_zv_from_impuls(std::span<const double> x0,
                                                                 std::vector<double>& jerk_impuls,
                                                                 std::vector<double>& time_impuls, double dt,
                                                                 double time_end) {
  // Time vector
  std::vector<double> time(std::size_t(std::ceil(time_end / dt)) + 1, 0);
  for (std::size_t i = 0; i < time.size(); i++) time[i] = i * dt;

  // Input vector
  std::vector<double> input(time.size(), 0);
  std::vector<double> traj(time.size() * x0.size(), 0);
  std::size_t time_index = 0;
  for (std::size_t i = 0; i < time.size(); i++) {
    // Input
    if (time_index < time_impuls.size() && time[i] >= time_impuls[time_index]) time_index++;
    input[i] = std::accumulate(jerk_impuls.begin(), jerk_impuls.begin() + time_index, 0.0);

    // Jerk times time
    std::vector<double> jerk_times_time = jerk_impuls;
    for (std::size_t j = 0; j <= jerk_times_time.size(); j++) jerk_times_time[j] *= time[i] - time_impuls[j];
    std::vector<double> jerk_times_time_sq = jerk_times_time;
    for (std::size_t j = 0; j <= jerk_times_time_sq.size(); j++) jerk_times_time_sq[j] *= 0.5 * (time[i] - time_impuls[j]);
    std::vector<double> jerk_times_time_cu = jerk_times_time_sq;
    for (std::size_t j = 0; j <= jerk_times_time_cu.size(); j++)
      jerk_times_time_cu[j] *= 1 / 3.0 * (time[i] - time_impuls[j]);

    // position
    double s_const = 0.5 * x0[2] * pow(time[i], 2) + x0[1] * time[i] + x0[0];
    traj[i * 3] = std::accumulate(jerk_times_time_cu.begin(), jerk_times_time_cu.begin() + time_index, s_const);

    // speed
    double v_const = x0[2] * time[i] + x0[1];
    traj[i * 3 + 1] = std::accumulate(jerk_times_time_sq.begin(), jerk_times_time_sq.begin() + time_index, v_const);

    // accel
    traj[i * 3+2] = std::accumulate(jerk_times_time.begin(), jerk_times_time.begin() + time_index, x0[2]);
  }

  return {time, traj, input};
}
