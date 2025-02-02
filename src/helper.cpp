#include "helper.h"

namespace mphelper {
Euler::Euler(system f, std::vector<std::vector<double>> inputs, double time)
    : dt(1e-4), f(f), time(time), inputs(inputs) {}

std::vector<cspace::trajTuple> Euler::operator()(const cspace::state_t& x) {
  std::vector<cspace::trajTuple> vecTrajTuple;

  for (auto& u : inputs) {
    // Create Trajectory
    std::vector<double> traj = integrate(x, u, this->time);
    // Create input vector
    std::vector<double> input;
    for (std::size_t i = 0; i < traj.size() / x.size(); i++) input.insert(input.end(), u.begin(), u.end());
    // Create time vector
    std::vector<double> time(traj.size() / x.size(), 0);
    for (std::size_t i = 0; i < time.size(); i++) time[i] = i * this->dt;
    
    vecTrajTuple.push_back({time, traj, input});
    
  }
  return vecTrajTuple;
}

std::vector<double> Euler::integrate(std::vector<double> x0, std::vector<double> u, double time) {
    std::vector<double> x, dxdt;
    std::size_t steps = time / this->dt;
    std::vector<double> traj;
    x = x0;

    for (std::size_t i = 0; i < steps; i++){
        dxdt = f(x,u);
        for(std::size_t j = 0; j < x0.size(); j++)
            x[j] = x[j] + dxdt[j] * this->dt;
        traj.insert(traj.end(), x.begin(), x.end());
    }
    return traj;

}
}  // namespace mphelper
