#include "mavros_xyz_position_offboard/execution/px4_action_executor.hpp"

#include <cmath>
#include <stdexcept>

namespace mavros_xyz_position_offboard::execution
{

namespace
{

bool required_number(
  const mission_core::ActionIntent & intent, const char * key, double & value, std::string & reason)
{
  const auto found = intent.parameters.find(key);
  if (found == intent.parameters.end() || !mission_core::parse_double(found->second, value)) {
    reason = std::string("action ") + intent.action_id + " requires numeric parameter " + key;
    return false;
  }
  return true;
}

}  // namespace

Px4ActionExecutor::Px4ActionExecutor(
  offboard::Offboard & offboard, const double target_tolerance_m, const double offboard_warmup_s)
: offboard_(offboard), target_tolerance_m_(target_tolerance_m), offboard_warmup_s_(offboard_warmup_s)
{
  if (!std::isfinite(target_tolerance_m_) || target_tolerance_m_ <= 0.0 ||
    !std::isfinite(offboard_warmup_s_) || offboard_warmup_s_ < 0.0)
  {
    throw std::invalid_argument(
            "PX4 action tolerance must be positive and OFFBOARD warmup must be finite and non-negative");
  }
  registry_.register_capability("navigate", validate_position_action);
  registry_.register_capability("takeoff", validate_takeoff_action);
  registry_.register_capability("land");
  registry_.register_capability("hold", validate_hold_action);
}

void Px4ActionExecutor::observe(const common::Telemetry & telemetry, const double now_s)
{
  world_.now_s = now_s;
  world_.observed_at_s = now_s;
  world_.connected = telemetry.connected;
  world_.armed = telemetry.armed;
  world_.mode = telemetry.mode;
  world_.landed = telemetry.landed_state == common::MAV_LANDED_STATE_ON_GROUND;
  world_.airborne = !world_.landed || telemetry.armed;
  world_.position.frame = "local_enu";
  world_.position.x_m = telemetry.local_x_m;
  world_.position.y_m = telemetry.local_y_m;
  world_.position.z_m = telemetry.local_z_m;
  world_.position.yaw_rad = 0.0;
  if (common::finite(telemetry.orientation.x) && common::finite(telemetry.orientation.y) &&
    common::finite(telemetry.orientation.z) && common::finite(telemetry.orientation.w))
  {
    world_.position.yaw_rad = common::yaw_from_quaternion(telemetry.orientation);
  }
  world_.position.valid = telemetry.local_pose_at.has_value() &&
    common::finite(telemetry.local_x_m) && common::finite(telemetry.local_y_m) &&
    common::finite(telemetry.local_z_m);
  world_.velocity.frame = "local_enu";
  world_.velocity.x_m_s = telemetry.velocity_x_m_s;
  world_.velocity.y_m_s = telemetry.velocity_y_m_s;
  world_.velocity.z_m_s = telemetry.velocity_z_m_s;
  world_.velocity.valid = telemetry.local_velocity_at.has_value() &&
    common::finite(telemetry.velocity_x_m_s) && common::finite(telemetry.velocity_y_m_s) &&
    common::finite(telemetry.velocity_z_m_s);
  world_.valid = world_.connected && world_.position.valid;
  world_.facts["lcp_healthy"] = telemetry.lcp_healthy_samples > 0;
}

mission_core::CapabilityState Px4ActionExecutor::capabilities(
  const mission_core::WorldState & world) const
{
  mission_core::CapabilityState result;
  const bool position_ready = world.connected && world.valid && world.position.valid;
  const bool preflight_ready = world.fact("preflight_ready", true);
  result.values["navigate"] = position_ready ? mission_core::CapabilityAvailability::available :
    mission_core::CapabilityAvailability::unavailable;
  result.values["hold"] = position_ready ? mission_core::CapabilityAvailability::available :
    mission_core::CapabilityAvailability::unavailable;
  result.values["takeoff"] = position_ready && preflight_ready && world.landed && !world.armed ?
    mission_core::CapabilityAvailability::available : mission_core::CapabilityAvailability::unavailable;
  result.values["land"] = position_ready && (world.airborne || world.armed) ?
    mission_core::CapabilityAvailability::available : mission_core::CapabilityAvailability::unavailable;
  if (!position_ready) {
    result.reasons["navigate"] = "fresh connected local pose is required";
    result.reasons["hold"] = result.reasons["navigate"];
    result.reasons["takeoff"] = result.reasons["navigate"];
    result.reasons["land"] = result.reasons["navigate"];
  }
  if (active_ && (active_->feedback.status == mission_core::ActionStatus::accepted ||
    active_->feedback.status == mission_core::ActionStatus::running ||
    active_->feedback.status == mission_core::ActionStatus::paused))
  {
    const auto found = result.values.find(active_->intent.capability);
    if (found != result.values.end()) {
      found->second = mission_core::CapabilityAvailability::busy;
    }
  }
  return result;
}

mission_core::ValidationResult Px4ActionExecutor::validate_position_action(
  const mission_core::ActionIntent & intent)
{
  for (const char * key : {"x", "y", "z"}) {
    double value = 0.0;
    std::string reason;
    if (!required_number(intent, key, value, reason)) {
      return {false, reason};
    }
  }
  const auto frame = intent.parameters.find("frame");
  if (frame != intent.parameters.end() && frame->second != "local_enu") {
    return {false, "only local_enu is supported by the PX4 adapter"};
  }
  return {true, {}};
}

mission_core::ValidationResult Px4ActionExecutor::validate_takeoff_action(
  const mission_core::ActionIntent & intent)
{
  double height = 0.0;
  std::string reason;
  if (!required_number(intent, "height_m", height, reason) || height <= 0.0) {
    return {false, reason.empty() ? "takeoff height_m must be positive" : reason};
  }
  return {true, {}};
}

mission_core::ValidationResult Px4ActionExecutor::validate_hold_action(
  const mission_core::ActionIntent & intent)
{
  const auto duration = intent.parameters.find("seconds");
  if (duration == intent.parameters.end()) {
    return {true, {}};
  }
  double value = 0.0;
  if (!mission_core::parse_double(duration->second, value) || value < 0.0) {
    return {false, "hold seconds must be a finite non-negative number"};
  }
  return {true, {}};
}

bool Px4ActionExecutor::parse_position(
  const mission_core::ActionIntent & intent, mission_core::Position & position, std::string & reason)
{
  position.frame = "local_enu";
  for (const char * key : {"x", "y", "z"}) {
    double value = 0.0;
    if (!required_number(intent, key, value, reason)) {
      return false;
    }
    if (key[0] == 'x') {position.x_m = value;}
    if (key[0] == 'y') {position.y_m = value;}
    if (key[0] == 'z') {position.z_m = value;}
  }
  const auto yaw = intent.parameters.find("yaw");
  if (yaw != intent.parameters.end() && !mission_core::parse_double(yaw->second, position.yaw_rad)) {
    reason = "yaw must be a finite number";
    return false;
  }
  position.valid = true;
  return true;
}

bool Px4ActionExecutor::parse_optional_duration(
  const mission_core::ActionIntent & intent, std::optional<double> & duration, std::string & reason)
{
  const auto found = intent.parameters.find("seconds");
  if (found == intent.parameters.end()) {
    duration.reset();
    return true;
  }
  double value = 0.0;
  if (!mission_core::parse_double(found->second, value) || value < 0.0) {
    reason = "seconds must be a finite non-negative number";
    return false;
  }
  duration = value;
  return true;
}

bool Px4ActionExecutor::submit(
  const mission_core::ActionIntent & intent, const double now_s, std::string & reason)
{
  const auto validation = registry_.validate(intent);
  if (!validation.accepted) {
    reason = validation.reason;
    return false;
  }
  if (active_ && (active_->feedback.status == mission_core::ActionStatus::succeeded ||
    active_->feedback.status == mission_core::ActionStatus::failed ||
    active_->feedback.status == mission_core::ActionStatus::cancelled) && !active_->safety_land)
  {
    active_.reset();
  }
  if (active_) {
    if (active_->intent.action_id == intent.action_id) {
      return true;
    }
    reason = "another PX4 action is active: " + active_->intent.action_id;
    return false;
  }
  if (!world_.connected || !world_.position.valid) {
    reason = "PX4 action requires a connected valid local pose";
    return false;
  }
  ActiveAction action;
  action.intent = intent;
  action.feedback = {intent.action_id, intent.capability, mission_core::ActionStatus::accepted,
    0.0, {}, now_s};
  action.started_at_s = now_s;
  if (intent.capability == "navigate") {
    mission_core::Position target;
    if (!parse_position(intent, target, reason)) {return false;}
    action.target = target;
  } else if (intent.capability == "takeoff") {
    double height = 0.0;
    if (!required_number(intent, "height_m", height, reason) || height <= 0.0) {
      if (reason.empty()) {reason = "takeoff height_m must be positive";}
      return false;
    }
    action.target = world_.position;
    action.target->z_m += height;
    action.target->valid = true;
  } else if (intent.capability == "hold") {
    action.target = world_.position;
    if (!parse_optional_duration(intent, action.duration_s, reason)) {return false;}
  }
  active_ = action;
  return true;
}

void Px4ActionExecutor::cancel(const std::string & action_id, const double now_s)
{
  if (!active_ || active_->intent.action_id != action_id) {return;}
  if (world_.airborne || world_.armed) {
    active_->intent.capability = "land";
    active_->feedback.capability = "land";
    active_->feedback.status = mission_core::ActionStatus::running;
    active_->feedback.reason = "safety landing after cancellation";
    active_->safety_land = true;
    active_->started_at_s = now_s;
  } else {
    finish(mission_core::ActionStatus::cancelled, "cancelled", now_s);
  }
}

void Px4ActionExecutor::pause(const std::string & action_id, const double now_s)
{
  if (active_ && active_->intent.action_id == action_id && !active_->safety_land) {
    if (!active_->paused) {active_->paused_at_s = now_s;}
    active_->paused = true;
    active_->feedback.status = mission_core::ActionStatus::paused;
    active_->feedback.at_s = now_s;
  }
}

void Px4ActionExecutor::resume(const std::string & action_id, const double now_s)
{
  if (active_ && active_->intent.action_id == action_id && active_->paused) {
    active_->paused_duration_s += std::max(0.0, now_s - active_->paused_at_s);
    active_->paused = false;
    active_->feedback.status = mission_core::ActionStatus::running;
    active_->feedback.at_s = now_s;
  }
}

bool Px4ActionExecutor::target_reached(const mission_core::Position & target) const
{
  if (!world_.position.valid) {return false;}
  const double dx = world_.position.x_m - target.x_m;
  const double dy = world_.position.y_m - target.y_m;
  const double dz = world_.position.z_m - target.z_m;
  return std::sqrt(dx * dx + dy * dy + dz * dz) <= target_tolerance_m_;
}

void Px4ActionExecutor::apply_active_command(const double now_s)
{
  if (!active_) {return;}
  if (active_->intent.capability == "land" || active_->safety_land) {
    if (world_.landed) {
      offboard_.apply({std::nullopt, std::optional<std::string>("MANUAL"), false}, now_s);
    } else {
      offboard_.apply({std::nullopt, std::optional<std::string>("AUTO.LAND"), true}, now_s);
    }
    return;
  }
  const double elapsed_s = std::max(
    0.0, now_s - active_->started_at_s - active_->paused_duration_s);
  if (active_->intent.capability == "takeoff" && elapsed_s < offboard_warmup_s_) {
    if (world_.position.valid) {
      common::PositionSetpoint setpoint;
      setpoint.x_m = world_.position.x_m;
      setpoint.y_m = world_.position.y_m;
      setpoint.z_m = world_.position.z_m;
      setpoint.orientation = common::normalize_quaternion(
        0.0, 0.0, std::sin(world_.position.yaw_rad / 2.0),
        std::cos(world_.position.yaw_rad / 2.0));
      offboard_.apply({setpoint, std::nullopt, std::nullopt}, now_s);
    }
    return;
  }
  if (active_->target) {
    common::PositionSetpoint setpoint;
    setpoint.x_m = active_->target->x_m;
    setpoint.y_m = active_->target->y_m;
    setpoint.z_m = active_->target->z_m;
    setpoint.orientation = common::normalize_quaternion(
      0.0, 0.0, std::sin(active_->target->yaw_rad / 2.0),
      std::cos(active_->target->yaw_rad / 2.0));
    offboard_.apply({setpoint, std::optional<std::string>("OFFBOARD"),
      active_->intent.capability == "takeoff" ? std::optional<bool>(true) : std::nullopt}, now_s);
  }
}

void Px4ActionExecutor::finish(
  const mission_core::ActionStatus status, std::string reason, const double now_s)
{
  if (!active_) {return;}
  active_->feedback.status = status;
  active_->feedback.reason = std::move(reason);
  active_->feedback.progress = status == mission_core::ActionStatus::succeeded ? 1.0 :
    active_->feedback.progress;
  active_->feedback.at_s = now_s;
}

void Px4ActionExecutor::update(
  const mission_core::WorldState & world, const double now_s, const double /*dt_s*/)
{
  world_ = world;
  if (!active_) {return;}
  apply_active_command(now_s);
  if (active_->safety_land) {
    if (world_.landed && !world_.armed) {
      finish(mission_core::ActionStatus::cancelled, "safety landing complete", now_s);
    }
    return;
  }
  if (active_->paused) {return;}
  active_->feedback.status = mission_core::ActionStatus::running;
  active_->feedback.at_s = now_s;
  if (active_->target && (active_->intent.capability == "navigate" ||
    active_->intent.capability == "takeoff") && target_reached(*active_->target))
  {
    finish(mission_core::ActionStatus::succeeded, {}, now_s);
    return;
  }
  if (active_->intent.capability == "hold" && active_->duration_s &&
    now_s - active_->started_at_s - active_->paused_duration_s >= *active_->duration_s)
  {
    finish(mission_core::ActionStatus::succeeded, {}, now_s);
    return;
  }
  if (active_->intent.capability == "land" && world_.landed && !world_.armed) {
    finish(mission_core::ActionStatus::succeeded, {}, now_s);
  }
}

mission_core::ActionFeedback Px4ActionExecutor::feedback(const std::string & action_id) const
{
  if (!active_ || active_->intent.action_id != action_id) {
    return {action_id, {}, mission_core::ActionStatus::idle, 0.0, "unknown action", world_.now_s};
  }
  return active_->feedback;
}

}  // namespace mavros_xyz_position_offboard::execution
