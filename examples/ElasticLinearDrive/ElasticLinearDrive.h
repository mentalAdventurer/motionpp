#ifndef ELASTIC_LINEAR_DRIVE_H
#define ELASTIC_LINEAR_DRIVE_H

#include "cspace.h"

namespace cs = cspace;

namespace ElasticLinearDrive {
cs::state_t dynamics(const cs::state_t &x, const cs::input_t &u);
cs::trajectory_t eulerIntegrate(const cs::state_t &x0,
                                const cs::trajectory_t &u, const float &t);
std::vector<cs::input_trajectory_t> getMotionPrimitives(const cs::state_t &x0);
cs::input_trajectory_t getInputTrajector(const std::size_t &n);
} // namespace ElasticLinearDrive

#endif // ELASTIC_LINEAR_DRIVE_H
