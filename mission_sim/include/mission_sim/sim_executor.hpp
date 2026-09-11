#pragma once

#include <map>
#include <set>
#include <string>

#include "mission_core/executor.hpp"

namespace mission_sim
{

/// Deterministic executor for offline mission tests. It never talks to ROS or hardware.
class SimExecutor final : public mission_core::IActionExecutor
{
public:
  SimExecutor();

  mission_core::CapabilityState capabilities(
    const mission_core::WorldState & world) const override;
  bool submit(
    const mission_core::ActionIntent & intent, double now_s, std::string & reason) override;
  void cancel(const std::string & action_id, double now_s) override;
  void pause(const std::string & action_id, double now_s) override;
  void resume(const std::string & action_id, double now_s) override;
  void update(
    const mission_core::WorldState & world, double now_s, double dt_s) override;
  mission_core::ActionFeedback feedback(const std::string & action_id) const override;

  void set_capability(
    const std::string & capability, mission_core::CapabilityAvailability availability,
    std::string reason = {});
  void set_duration(const std::string & capability, double seconds);
  void reject_capability(const std::string & capability, bool reject);
  const mission_core::ActionIntent & active_intent() const {return active_intent_;}

private:
  struct Record
  {
    mission_core::ActionIntent intent{};
    mission_core::ActionFeedback feedback{};
    double started_at_s{0.0};
    double paused_at_s{0.0};
    double paused_duration_s{0.0};
    bool paused{false};
  };

  mission_core::CapabilityState capabilities_{};
  std::map<std::string, double> durations_{};
  std::set<std::string> rejected_{};
  std::map<std::string, Record> records_{};
  mission_core::ActionIntent active_intent_{};
  std::string active_action_id_{};
};

}  // namespace mission_sim
