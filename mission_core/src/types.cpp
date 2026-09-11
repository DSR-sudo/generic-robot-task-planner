#include "mission_core/types.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdlib>

namespace mission_core
{

bool WorldState::fresh(const double now_s_value, const double max_age_s) const
{
  return valid && std::isfinite(observed_at_s) && std::isfinite(now_s_value) &&
         now_s_value >= observed_at_s && now_s_value - observed_at_s <= max_age_s;
}

bool WorldState::fact(const std::string & name, const bool default_value) const
{
  const auto found = facts.find(name);
  return found == facts.end() ? default_value : found->second;
}

CapabilityAvailability CapabilityState::availability(const std::string & capability) const
{
  const auto found = values.find(capability);
  return found == values.end() ? CapabilityAvailability::unsupported : found->second;
}

bool CapabilityState::supports(const std::string & capability) const
{
  return availability(capability) != CapabilityAvailability::unsupported;
}

bool CapabilityState::available(const std::string & capability) const
{
  return availability(capability) == CapabilityAvailability::available;
}

std::string CapabilityState::reason(const std::string & capability) const
{
  const auto found = reasons.find(capability);
  return found == reasons.end() ? std::string{} : found->second;
}

const char * to_string(const CapabilityAvailability value)
{
  switch (value) {
    case CapabilityAvailability::unsupported: return "unsupported";
    case CapabilityAvailability::unavailable: return "unavailable";
    case CapabilityAvailability::available: return "available";
    case CapabilityAvailability::busy: return "busy";
  }
  return "unsupported";
}

const char * to_string(const ActionStatus value)
{
  switch (value) {
    case ActionStatus::idle: return "idle";
    case ActionStatus::accepted: return "accepted";
    case ActionStatus::running: return "running";
    case ActionStatus::succeeded: return "succeeded";
    case ActionStatus::failed: return "failed";
    case ActionStatus::cancelled: return "cancelled";
    case ActionStatus::paused: return "paused";
  }
  return "idle";
}

const char * to_string(const MissionStatus value)
{
  switch (value) {
    case MissionStatus::idle: return "idle";
    case MissionStatus::running: return "running";
    case MissionStatus::paused: return "paused";
    case MissionStatus::succeeded: return "succeeded";
    case MissionStatus::failed: return "failed";
    case MissionStatus::cancelled: return "cancelled";
  }
  return "idle";
}

const char * to_string(const SafetyAction value)
{
  switch (value) {
    case SafetyAction::allow: return "allow";
    case SafetyAction::pause: return "pause";
    case SafetyAction::abort: return "abort";
    case SafetyAction::emergency: return "emergency";
  }
  return "allow";
}

bool parse_bool(const std::string & value, bool & result)
{
  if (value == "true" || value == "1" || value == "yes") {
    result = true;
    return true;
  }
  if (value == "false" || value == "0" || value == "no") {
    result = false;
    return true;
  }
  return false;
}

bool parse_double(const std::string & value, double & result)
{
  char * end = nullptr;
  errno = 0;
  const double parsed = std::strtod(value.c_str(), &end);
  if (end == value.c_str() || *end != '\0' || errno == ERANGE || !std::isfinite(parsed)) {
    return false;
  }
  result = parsed;
  return true;
}

}  // namespace mission_core
