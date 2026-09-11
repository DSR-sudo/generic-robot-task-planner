#include <iostream>
#include <string>
#include <vector>

#include "mission_core/mission_tree.hpp"
#include "mission_core/safety.hpp"
#include "mission_sim/sim_executor.hpp"

int main(int argc, char ** argv)
{
  const std::string path = argc > 1 ? argv[1] : "config/flight_demo.xml";
  mission_sim::SimExecutor executor;
  mission_core::MissionTree tree(path, "simulated_mission");
  mission_core::SafetySupervisor safety;
  mission_core::WorldState world;
  world.connected = true;
  world.valid = true;
  world.observed_at_s = 0.0;
  world.position.valid = true;
  world.landed = true;
  world.airborne = false;
  const bool navigation_demo = path.find("navigation_demo") != std::string::npos;

  for (int step = 0; step < 1000; ++step) {
    world.now_s = step * 0.05;
    world.dt_s = 0.05;
    world.observed_at_s = world.now_s;
    std::vector<mission_core::MissionEvent> events;
    if (navigation_demo && step == 40) {
      events.push_back({"release_a", "{}", world.now_s});
    }
    const auto result = tree.tick(
      world, executor.capabilities(world), events, executor, safety);
    std::cout << world.now_s << " " << mission_core::to_string(result.runtime.status) << " "
              << mission_core::to_string(result.tree_status) << " "
              << result.runtime.active_node << std::endl;
    if (result.runtime.status == mission_core::MissionStatus::succeeded ||
      result.runtime.status == mission_core::MissionStatus::failed)
    {
      return result.runtime.status == mission_core::MissionStatus::succeeded ? 0 : 1;
    }
  }
  std::cerr << "simulation timed out" << std::endl;
  return 2;
}
