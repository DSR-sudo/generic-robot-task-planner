#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "mission_core/mission_tree.hpp"
#include "mission_core/safety.hpp"
#include "mission_sim/sim_executor.hpp"

namespace
{

mission_core::WorldState world_at(double now)
{
  mission_core::WorldState world;
  world.now_s = now;
  world.dt_s = 0.1;
  world.observed_at_s = now;
  world.connected = true;
  world.valid = true;
  world.position.valid = true;
  world.landed = true;
  return world;
}

void test_action_is_idempotent()
{
  const auto path = std::filesystem::temp_directory_path() / "mission_sim_idempotent.xml";
  {
    std::ofstream output(path);
    output << R"xml(<root><BehaviorTree ID="MainTree"><ExecuteAction id="a" capability="wait"/></BehaviorTree></root>)xml";
  }
  mission_sim::SimExecutor executor;
  executor.set_duration("wait", 0.2);
  mission_core::MissionTree tree(path.string());
  mission_core::SafetySupervisor safety;
  auto world = world_at(0.0);
  auto result = tree.tick(world, executor.capabilities(world), {}, executor, safety);
  assert(result.runtime.status == mission_core::MissionStatus::running);
  const auto first_id = executor.active_intent().action_id;
  world = world_at(0.1);
  result = tree.tick(world, executor.capabilities(world), {}, executor, safety);
  assert(first_id == executor.active_intent().action_id || result.runtime.status == mission_core::MissionStatus::succeeded);
  world = world_at(0.3);
  result = tree.tick(world, executor.capabilities(world), {}, executor, safety);
  assert(result.runtime.status == mission_core::MissionStatus::succeeded);
  std::error_code error;
  std::filesystem::remove(path, error);
}

void test_event_wait_and_capability_failure()
{
  const auto path = std::filesystem::temp_directory_path() / "mission_sim_event.xml";
  {
    std::ofstream output(path);
    output << R"xml(<root><BehaviorTree ID="MainTree"><Sequence><WaitEvent event="go"/><ExecuteAction id="move" capability="navigate"/></Sequence></BehaviorTree></root>)xml";
  }
  mission_sim::SimExecutor executor;
  mission_core::MissionTree tree(path.string());
  mission_core::SafetySupervisor safety;
  auto world = world_at(0.0);
  auto result = tree.tick(world, executor.capabilities(world), {}, executor, safety);
  assert(result.runtime.status == mission_core::MissionStatus::running);
  world = world_at(0.1);
  executor.set_capability("navigate", mission_core::CapabilityAvailability::unavailable, "simulated unavailable");
  result = tree.tick(world, executor.capabilities(world), {{"go", {}, 0.1}}, executor, safety);
  result = tree.tick(world, executor.capabilities(world), {}, executor, safety);
  assert(result.runtime.status == mission_core::MissionStatus::failed);
  assert(!result.error.empty());
  std::error_code error;
  std::filesystem::remove(path, error);
}

void test_safety_pauses_and_resumes()
{
  const auto path = std::filesystem::temp_directory_path() / "mission_sim_safety.xml";
  {
    std::ofstream output(path);
    output << R"xml(<root><BehaviorTree ID="MainTree"><ExecuteAction id="wait" capability="wait"/></BehaviorTree></root>)xml";
  }
  mission_sim::SimExecutor executor;
  executor.set_duration("wait", 0.4);
  mission_core::MissionTree tree(path.string());
  mission_core::SafetySupervisor safety;
  auto world = world_at(0.0);
  auto result = tree.tick(world, executor.capabilities(world), {}, executor, safety);
  assert(result.runtime.status == mission_core::MissionStatus::running);
  world = world_at(0.1);
  world.connected = false;
  result = tree.tick(world, executor.capabilities(world), {}, executor, safety);
  assert(result.runtime.status == mission_core::MissionStatus::paused);
  world = world_at(0.2);
  world.connected = true;
  result = tree.tick(world, executor.capabilities(world), {}, executor, safety);
  assert(result.runtime.status == mission_core::MissionStatus::running);
  world = world_at(0.6);
  result = tree.tick(world, executor.capabilities(world), {}, executor, safety);
  assert(result.runtime.status == mission_core::MissionStatus::succeeded);
  std::error_code error;
  std::filesystem::remove(path, error);
}

}  // namespace

int main()
{
  test_action_is_idempotent();
  test_event_wait_and_capability_failure();
  test_safety_pauses_and_resumes();
  std::cout << "mission_sim tests passed" << std::endl;
  return 0;
}
