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
  vec_double trajc(x.size()*(inputs.size()+1),0);
  std::copy(x.begin(),x.end(),trajc.begin());
  for (std::size_t i = 0; i < inputs.size(); i++){
    auto dxdt = dynamics(x,inputs[i]);
    for (std::size_t j = 0; j < x.size(); j++){
      x[j] += dxdt[j] * dt;
      trajc[(i+1)*x.size() + j] = x[j];
    }
  }
  return trajc;
}
