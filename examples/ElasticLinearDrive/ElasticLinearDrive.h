#ifndef ELASTIC_LINEAR_DRIVE_H
#define ELASTIC_LINEAR_DRIVE_H

#include "cspace.h"

namespace cs = cspace;

namespace ElasticLinearDrive {
using vec_iter = std::vector<double>::const_iterator;
cs::state_t dynamics(vec_iter x, vec_iter u, const std::size_t n);
cs::trajectory_t eulerIntegrate(const cs::state_t &x0,
                                const cs::trajectory_t &u, const float &t);
std::pair<std::vector<double>, std::vector<float>> getMotionPrimitives(const cs::state_t &x0);
cs::input_trajectory_t getInputTrajector(const std::size_t &n);
} // namespace ElasticLinearDrive

#endif // ELASTIC_LINEAR_DRIVE_H
