#include "mission_sim/sim_executor.hpp"

#include <algorithm>
#include <cmath>

namespace mission_sim
{

SimExecutor::SimExecutor()
{
  for (const auto & capability : {"navigate", "hold", "takeoff", "land", "wait"}) {
    capabilities_.values[capability] = mission_core::CapabilityAvailability::available;
    durations_[capability] = capability == std::string("wait") ? 0.1 : 1.0;
  }
}

mission_core::CapabilityState SimExecutor::capabilities(
  const mission_core::WorldState & /*world*/) const
{
  auto result = capabilities_;
  if (!active_action_id_.empty()) {
    const auto found = result.values.find(active_intent_.capability);
    if (found != result.values.end() && found->second == mission_core::CapabilityAvailability::available) {
      found->second = mission_core::CapabilityAvailability::busy;
    }
  }
  return result;
}

bool SimExecutor::submit(
  const mission_core::ActionIntent & intent, const double now_s, std::string & reason)
{
  const auto capability = capabilities_.availability(intent.capability);
  if (capability == mission_core::CapabilityAvailability::unsupported) {
    reason = "unsupported capability: " + intent.capability;
    return false;
  }
  if (capability != mission_core::CapabilityAvailability::available) {
    reason = capabilities_.reason(intent.capability);
    if (reason.empty()) {reason = "capability is not available: " + intent.capability;}
    return false;
  }
  if (rejected_.find(intent.capability) != rejected_.end()) {
    reason = "simulated rejection: " + intent.capability;
    return false;
  }
  const auto frame = intent.parameters.find("frame");
  if (frame != intent.parameters.end() && frame->second != "local_enu") {
    reason = "simulator only accepts local_enu actions";
    return false;
  }
  const auto existing = records_.find(intent.action_id);
  if (existing != records_.end()) {
    active_action_id_ = intent.action_id;
    active_intent_ = intent;
    return true;
  }
  if (!active_action_id_.empty() && active_action_id_ != intent.action_id) {
    reason = "another action is already active: " + active_action_id_;
    return false;
  }
  Record record;
  record.intent = intent;
  record.feedback = {intent.action_id, intent.capability, mission_core::ActionStatus::accepted,
    0.0, {}, now_s};
  record.started_at_s = now_s;
  records_.emplace(intent.action_id, record);
  active_action_id_ = intent.action_id;
  active_intent_ = intent;
  return true;
}

void SimExecutor::cancel(const std::string & action_id, const double now_s)
{
  const auto found = records_.find(action_id);
  if (found == records_.end()) {return;}
  found->second.feedback.status = mission_core::ActionStatus::cancelled;
  found->second.feedback.reason = "cancelled";
  found->second.feedback.at_s = now_s;
  if (active_action_id_ == action_id) {
    active_action_id_.clear();
    active_intent_ = {};
  }
}

void SimExecutor::pause(const std::string & action_id, const double now_s)
{
  const auto found = records_.find(action_id);
  if (found == records_.end() || found->second.feedback.status == mission_core::ActionStatus::succeeded) {
    return;
  }
  auto & record = found->second;
  if (!record.paused) {
    record.paused = true;
    record.paused_at_s = now_s;
  }
  record.feedback.status = mission_core::ActionStatus::paused;
  record.feedback.at_s = now_s;
}

void SimExecutor::resume(const std::string & action_id, const double now_s)
{
  const auto found = records_.find(action_id);
  if (found == records_.end()) {return;}
  auto & record = found->second;
  if (record.paused) {
    record.paused_duration_s += std::max(0.0, now_s - record.paused_at_s);
    record.paused = false;
  }
  if (record.feedback.status == mission_core::ActionStatus::paused) {
    record.feedback.status = mission_core::ActionStatus::running;
  }
  record.feedback.at_s = now_s;
  active_action_id_ = action_id;
  active_intent_ = record.intent;
}

void SimExecutor::update(
  const mission_core::WorldState & /*world*/, const double now_s, const double /*dt_s*/)
{
  if (active_action_id_.empty()) {return;}
  const auto found = records_.find(active_action_id_);
  if (found == records_.end()) {return;}
  auto & record = found->second;
  if (record.paused || record.feedback.status == mission_core::ActionStatus::cancelled ||
    record.feedback.status == mission_core::ActionStatus::succeeded ||
    record.feedback.status == mission_core::ActionStatus::failed)
  {
    return;
  }
  record.feedback.status = mission_core::ActionStatus::running;
  const double duration = durations_.count(record.intent.capability) == 0 ? 1.0 :
    std::max(0.0, durations_.at(record.intent.capability));
  const double elapsed = std::max(0.0, now_s - record.started_at_s - record.paused_duration_s);
  record.feedback.progress = duration <= 0.0 ? 1.0 : std::min(1.0, elapsed / duration);
  record.feedback.at_s = now_s;
  if (elapsed >= duration) {
    record.feedback.status = mission_core::ActionStatus::succeeded;
    record.feedback.progress = 1.0;
    if (active_action_id_ == record.intent.action_id) {
      active_action_id_.clear();
      active_intent_ = {};
    }
  }
}

mission_core::ActionFeedback SimExecutor::feedback(const std::string & action_id) const
{
  const auto found = records_.find(action_id);
  if (found == records_.end()) {
    return {action_id, {}, mission_core::ActionStatus::idle, 0.0, "unknown action", 0.0};
  }
  return found->second.feedback;
}

void SimExecutor::set_capability(
  const std::string & capability, const mission_core::CapabilityAvailability availability,
  std::string reason)
{
  capabilities_.values[capability] = availability;
  if (reason.empty()) {
    capabilities_.reasons.erase(capability);
  } else {
    capabilities_.reasons[capability] = std::move(reason);
  }
}

void SimExecutor::set_duration(const std::string & capability, const double seconds)
{
  if (std::isfinite(seconds) && seconds >= 0.0) {
    durations_[capability] = seconds;
  }
}

void SimExecutor::reject_capability(const std::string & capability, const bool reject)
{
  if (reject) {
    rejected_.insert(capability);
  } else {
    rejected_.erase(capability);
  }
}

}  // namespace mission_sim
