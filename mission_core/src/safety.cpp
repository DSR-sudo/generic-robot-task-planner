#include "mission_core/safety.hpp"

#include <algorithm>

namespace mission_core
{

SafetySupervisor::SafetySupervisor(SafetyConfig config)
: config_(config)
{
}

SafetyDecision SafetySupervisor::evaluate(
  const WorldState & world, const CapabilityState & capabilities,
  const MissionRuntime & runtime) const
{
  if (runtime.status == MissionStatus::succeeded || runtime.status == MissionStatus::failed ||
    runtime.status == MissionStatus::cancelled)
  {
    return {};
  }

  if (config_.max_mission_duration_s && runtime.started_at_s >= 0.0 &&
    world.now_s >= runtime.started_at_s &&
    world.now_s - runtime.started_at_s > *config_.max_mission_duration_s)
  {
    if (config_.emergency_when_airborne && (world.airborne || world.armed || runtime.airborne)) {
      return {SafetyAction::emergency, "mission duration limit exceeded", "mission_time"};
    }
    return {SafetyAction::abort, "mission duration limit exceeded", {}};
  }

  if (config_.require_connection && !world.connected) {
    if (config_.emergency_when_airborne && (world.airborne || world.armed || runtime.airborne)) {
      return {SafetyAction::emergency, "platform connection lost while airborne", "platform_connected"};
    }
    if (config_.abort_when_grounded_state_invalid && !runtime.active_action_id.empty()) {
      return {SafetyAction::abort, "platform is not connected during an action", {}};
    }
    return {SafetyAction::pause, "platform is not connected", "platform_connected"};
  }

  if (config_.require_valid_state && !world.fresh(world.now_s, config_.max_world_age_s)) {
    if (config_.emergency_when_airborne && (world.airborne || world.armed || runtime.airborne)) {
      return {SafetyAction::emergency, "world state is stale while airborne", "fresh_world_state"};
    }
    if (config_.abort_when_grounded_state_invalid && !runtime.active_action_id.empty()) {
      return {SafetyAction::abort, "world state became stale during an action", {}};
    }
    return {SafetyAction::pause, "world state is stale", "fresh_world_state"};
  }

  if (runtime.active_action_id.empty()) {
    return {};
  }
  const bool fact_pause_exempt = std::find(
    config_.pause_fact_exempt_capabilities.begin(), config_.pause_fact_exempt_capabilities.end(),
    runtime.active_capability) != config_.pause_fact_exempt_capabilities.end();
  if (!fact_pause_exempt) {
    for (const auto & fact : config_.pause_when_fact_false) {
      if (!world.fact(fact, true)) {
        return {SafetyAction::pause, "required fact is false: " + fact, fact};
      }
    }
  }
  const auto availability = capabilities.availability(runtime.active_capability);
  if (availability == CapabilityAvailability::unsupported) {
    return {SafetyAction::abort, "active capability is unsupported", {}};
  }
  if (availability == CapabilityAvailability::unavailable && config_.pause_when_grounded) {
    return {SafetyAction::pause, capabilities.reason(runtime.active_capability), "capability_available"};
  }
  return {};
}

}  // namespace mission_core
