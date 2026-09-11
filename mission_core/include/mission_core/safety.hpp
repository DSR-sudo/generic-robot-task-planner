#pragma once

#include <string>
#include <optional>
#include <vector>

#include "mission_core/types.hpp"

namespace mission_core
{

struct SafetyConfig
{
  double max_world_age_s{1.0};
  bool require_connection{true};
  bool require_valid_state{true};
  bool emergency_when_airborne{true};
  bool pause_when_grounded{true};
  bool abort_when_grounded_state_invalid{false};
  std::optional<double> max_mission_duration_s{};
  std::vector<std::string> pause_when_fact_false{};
  std::vector<std::string> pause_fact_exempt_capabilities{};
};

/// Generic safety gate. Platform policy can wrap or extend this decision before execution.
class SafetySupervisor
{
public:
  explicit SafetySupervisor(SafetyConfig config = {});

  SafetyDecision evaluate(
    const WorldState & world, const CapabilityState & capabilities,
    const MissionRuntime & runtime) const;

private:
  SafetyConfig config_{};
};

}  // namespace mission_core
