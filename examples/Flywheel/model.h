#pragma once
#include <vector>

namespace param {
  static constexpr double J = 0.0135;
  static constexpr double Ja = 0.0012;
  static constexpr double m = 0.416;
  static constexpr double l = 0.175;
  static constexpr double k = 0.0346;
  static constexpr double g = 9.81;
  static constexpr double ddphi_max = 350;
  static constexpr double pi = 3.14159265358979323846;
} 

namespace model {
  std::vector<double> system(const std::vector<double>& state, const std::vector<double>& input);
}

