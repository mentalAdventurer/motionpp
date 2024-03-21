#ifndef QUEUE_H
#define QUEUE_H

#include <map>

#include "cspace.h"

class Queue {
 public:
  struct StateCostPair;
  Queue(const cspace::state_ptr& x0);
  Queue(const cspace::state_ptr& x0, cspace::Options::metric_func metric);
  void push(const cspace::state_ptr& x, float cost, const cspace::input_traj_ptr& u, const float time);
  StateCostPair pop();
  bool empty();

 private:
  std::multimap<float, cspace::state_ptr> queueMap;
  float defaultMetric(const cspace::state_ptr& x, const cspace::input_traj_ptr& u, const float& time);
  std::function<float(cspace::state_ptr, cspace::input_traj_ptr, float)> metric;
};

struct Queue::StateCostPair {
  std::shared_ptr<const cspace::state_t> x;
  const float cost;
};

#endif  // QUEUE_H
