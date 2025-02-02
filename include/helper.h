#ifndef HELPER_H
#define HELPER_H

#include <functional>
#include <vector>
#include "cspace.h"

namespace mphelper {
  class Euler {
   public:
     using system = std::function<std::vector<double>(std::vector<double> state,std::vector<double> input)>;
     Euler(system f, std::vector<std::vector<double>> inputs, double time);
     std::vector<cspace::trajTuple> operator()(const cspace::state_t& x);
     double dt;

   private:
     system f;
     double time;
     std::vector<std::vector<double>> inputs;
     std::vector<double> integrate(std::vector<double> x0, std::vector<double> u, double time);
     
  };
}

#endif  // HELPER_H
