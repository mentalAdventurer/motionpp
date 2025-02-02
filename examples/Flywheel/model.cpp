#include "model.h"
#include <cmath>

std::vector<double> model::system(const std::vector<double>& state, const std::vector<double>& input) {
  using namespace param;
  double dphi, ddphi, dpsi, ddpsi;
  dphi = state[1];
  ddphi = input[0];
  dpsi = state[3];
  ddpsi = -m * g * l / J * std::sin(state[2]) - Ja / J * input[0];

  return {dphi, ddphi, dpsi, ddpsi};
}
