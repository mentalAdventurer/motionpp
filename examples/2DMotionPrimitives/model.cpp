#include "model.h"


vec_double dynamics(cvec_double& x,double u){
  using namespace param;
  vec_double dxdt(x.size(),0);
  dxdt[0] = x[1];
  dxdt[1] = (-d * x[1] - k * x[0] - u) / m1;
  dxdt[2] = x[3];
  dxdt[3] = -dxdt[1] + u / m2;
  return dxdt;
}

vec_double calculate_input(vec_double x0, cvec_double accel, double dt){
  using namespace param;
  vec_double inputs(accel.size(),0);

  for(std::size_t i = 0; i < accel.size(); i++){
    static constexpr double den = (m1/m2 + 1);
    double num = m1 * accel[i] - d * x0[1] - k * x0[0];
    double u = num / den; 
    auto dxdt = dynamics(x0,u);
    inputs[i] = u;
    for (std::size_t j = 0; j < x0.size(); j++){
      x0[j] += dxdt[j] * dt;
    }
  }
  return inputs;
}

vec_double simulate_system(vec_double x, cvec_double& inputs, double dt){
  vec_double trajc(x.size()*inputs.size(),0);
  for (std::size_t i = 0; i < inputs.size(); i+=2){
    auto dxdt = dynamics(std::vector<double>(x.begin(),x.begin()+4),inputs[i]);
    auto dxdt_y = dynamics(std::vector<double>(x.begin()+4,x.end()),inputs[i+1]);
    dxdt.insert(dxdt.end(),dxdt_y.begin(),dxdt_y.end());
    for (std::size_t j = 0; j < x.size(); j++){
      x[j] += dxdt[j] * dt;
      trajc[i*x.size() + j] = x[j];
    }
  }
  return trajc;
}

trajTuple get_zv_trajectory(double start_position, double start_speed, double target_speed) {
  auto [times, speed, accel, jerk] =
      generate_scurve(start_speed, target_speed, param::accel_limit, param::jerk_limit, param::dt);
  auto impulses = generate_impulse_from_signal(jerk);
  auto [impulse, time] = apply_zero_vibration_shaping(impulses, times, param::kappa, param::Tv);
  auto [zv_v, zv_a, zv_j] = generate_traj_from_impulses(impulse, time);
  auto inputs = calculate_input({0, 0, start_position, start_speed}, zv_a, param::dt);
  auto traj = simulate_system({0, 0, start_position, start_speed}, inputs, param::dt);
  return std::make_tuple(time, traj, inputs);
}
