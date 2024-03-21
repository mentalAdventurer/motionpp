#include "queue.h"

#include <iostream>

Queue::Queue(const cspace::state_ptr& x0) {
  // Use default metric if no metric is provided
  metric = [this](cspace::state_ptr x, cspace::input_traj_ptr u, float time) {
    return this->defaultMetric(x, u, time);
  };
  queueMap.insert(std::make_pair(0, x0));
}

Queue::Queue(const cspace::state_ptr& x0, cspace::Options::metric_func sort_metric) {
  if (sort_metric == nullptr) {
    metric = [this](cspace::state_ptr x, cspace::input_traj_ptr u, float time) {
      return this->defaultMetric(x, u, time);
    };
  } else {
    metric = sort_metric;
  }
  queueMap.insert(std::make_pair(0, x0));
}

void Queue::push(const cspace::state_ptr& x, float cost, const cspace::input_traj_ptr& u, const float time) {
  cost += metric(x, u, time);
  queueMap.insert(std::make_pair(cost, x));
}

Queue::StateCostPair Queue::pop() {
  auto it = queueMap.begin();
  auto x = it->second;
  auto cost = it->first;
  queueMap.erase(it);
  return {x, cost};
}

bool Queue::empty() { return queueMap.empty(); }

float Queue::defaultMetric(const cspace::state_ptr& x, const cspace::input_traj_ptr& u, const float& time) {
  return time;
}
