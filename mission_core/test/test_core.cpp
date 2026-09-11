#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>

#include "mission_core/executor.hpp"
#include "mission_core/mission_tree.hpp"
#include "mission_core/safety.hpp"

namespace
{

class FakeExecutor final : public mission_core::IActionExecutor
{
public:
  mission_core::CapabilityState capabilities(
    const mission_core::WorldState & /*world*/) const override
  {
    return capabilities_;
  }

  bool submit(
    const mission_core::ActionIntent & intent, const double now_s, std::string & reason) override
  {
    if (!capabilities_.available(intent.capability)) {
      reason = "capability unavailable";
      return false;
    }
    if (active_ && active_->action_id != intent.action_id) {
      reason = "another action active";
      return false;
    }
    if (!active_) {
      active_ = intent;
      feedback_ = {intent.action_id, intent.capability, mission_core::ActionStatus::running,
        0.0, {}, now_s};
    }
    ++submit_count;
    return true;
  }

  void cancel(const std::string & action_id, const double now_s) override
  {
    if (active_ && active_->action_id == action_id) {
      feedback_.status = mission_core::ActionStatus::cancelled;
      feedback_.at_s = now_s;
    }
  }

  void pause(const std::string & action_id, const double now_s) override
  {
    if (active_ && active_->action_id == action_id) {
      feedback_.status = mission_core::ActionStatus::paused;
      feedback_.at_s = now_s;
    }
  }

  void resume(const std::string & action_id, const double now_s) override
  {
    if (active_ && active_->action_id == action_id) {
      feedback_.status = mission_core::ActionStatus::running;
      feedback_.at_s = now_s;
    }
  }

  void update(
    const mission_core::WorldState & /*world*/, const double now_s, const double /*dt_s*/) override
  {
    if (active_ && feedback_.status == mission_core::ActionStatus::running && now_s >= 1.0) {
      feedback_.status = mission_core::ActionStatus::succeeded;
      feedback_.progress = 1.0;
      feedback_.at_s = now_s;
    }
  }

  mission_core::ActionFeedback feedback(const std::string & action_id) const override
  {
    if (!active_ || active_->action_id != action_id) {
      return {action_id, {}, mission_core::ActionStatus::idle, 0.0, "unknown", 0.0};
    }
    return feedback_;
  }

  mission_core::CapabilityState capabilities_{{{"wait", mission_core::CapabilityAvailability::available}}, {}};
  std::optional<mission_core::ActionIntent> active_{};
  mission_core::ActionFeedback feedback_{};
  int submit_count{0};
};

mission_core::WorldState world(double now)
{
  mission_core::WorldState state;
  state.now_s = now;
  state.dt_s = 0.1;
  state.observed_at_s = now;
  state.connected = true;
  state.valid = true;
  state.position.valid = true;
  return state;
}

void test_registry()
{
  mission_core::CapabilityRegistry registry;
  registry.register_capability("navigate", [](const mission_core::ActionIntent & intent) {
    return intent.parameters.count("x") == 0 ?
      mission_core::ValidationResult{false, "x is required"} :
      mission_core::ValidationResult{true, {}};
  });
  mission_core::ActionIntent intent{"m", "a", "navigate", {}};
  assert(!registry.validate(intent).accepted);
  intent.parameters["x"] = "1";
  assert(registry.validate(intent).accepted);
  intent.capability = "missing";
  assert(!registry.validate(intent).accepted);
}

void test_tree_and_idempotence()
{
  const auto path = std::filesystem::temp_directory_path() / "mission_core_tree.xml";
  {
    std::ofstream output(path);
    output << R"xml(<root><BehaviorTree ID="MainTree"><ExecuteAction id="wait" capability="wait"/></BehaviorTree></root>)xml";
  }
  FakeExecutor executor;
  mission_core::MissionTree tree(path.string(), "core_test");
  mission_core::SafetySupervisor safety;
  mission_core::CapabilityState unsupported;
  assert(!tree.validate_capabilities(unsupported).accepted);
  mission_core::CapabilityState available;
  available.values["wait"] = mission_core::CapabilityAvailability::available;
  assert(tree.validate_capabilities(available).accepted);
  auto result = tree.tick(world(0.0), executor.capabilities(world(0.0)), {}, executor, safety);
  assert(result.runtime.status == mission_core::MissionStatus::running);
  result = tree.tick(world(0.5), executor.capabilities(world(0.5)), {}, executor, safety);
  assert(result.runtime.status == mission_core::MissionStatus::running);
  assert(executor.submit_count == 1);
  result = tree.tick(world(1.0), executor.capabilities(world(1.0)), {}, executor, safety);
  assert(result.runtime.status == mission_core::MissionStatus::succeeded);
  result = tree.tick(world(2.0), executor.capabilities(world(2.0)), {}, executor, safety);
  assert(result.runtime.status == mission_core::MissionStatus::succeeded);
  assert(executor.submit_count == 1);
  std::error_code error;
  std::filesystem::remove(path, error);
}

void test_unknown_node()
{
  const auto path = std::filesystem::temp_directory_path() / "mission_core_unknown.xml";
  {
    std::ofstream output(path);
    output << R"xml(<root><BehaviorTree ID="MainTree"><Unknown/></BehaviorTree></root>)xml";
  }
  bool threw = false;
  try {
    mission_core::MissionTree tree(path.string());
  } catch (const std::exception &) {
    threw = true;
  }
  assert(threw);
  std::error_code error;
  std::filesystem::remove(path, error);
}

void test_safety()
{
  mission_core::SafetySupervisor safety;
  mission_core::MissionRuntime runtime;
  runtime.status = mission_core::MissionStatus::running;
  runtime.airborne = true;
  auto state = world(1.0);
  state.connected = false;
  const auto decision = safety.evaluate(state, {}, runtime);
  assert(decision.action == mission_core::SafetyAction::emergency);
}

}  // namespace

int main()
{
  test_registry();
  test_tree_and_idempotence();
  test_unknown_node();
  test_safety();
  std::cout << "mission_core tests passed" << std::endl;
  return 0;
}
