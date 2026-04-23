#include <cstdlib>
#include <iostream>
#include <string>

#include <rmw/types.h>

#include "ros/publisher/data/dyn_obstacle_data_publisher.hpp"

namespace {

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

}  // namespace

int main() {
  const rmw_qos_profile_t profile = ausim::DynObstacleDataPublisher::DefaultQos().get_rmw_qos_profile();
  Expect(profile.history == RMW_QOS_POLICY_HISTORY_KEEP_LAST, "expected keep-last QoS history");
  Expect(profile.depth == 1, "expected a single cached obstacle snapshot");
  Expect(profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE, "expected reliable QoS");
  Expect(profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL, "expected transient-local durability");
  return 0;
}
