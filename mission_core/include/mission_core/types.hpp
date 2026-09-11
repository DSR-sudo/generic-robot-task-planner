#pragma once

#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace mission_core
{

/// A position expressed in a named coordinate frame. The first implementation uses local_enu.
struct Position
{
  std::string frame{"local_enu"};
  double x_m{0.0};
  double y_m{0.0};
  double z_m{0.0};
  double yaw_rad{0.0};
  bool valid{false};
};

struct Velocity
{
  std::string frame{"local_enu"};
  double x_m_s{0.0};
  double y_m_s{0.0};
  double z_m_s{0.0};
  bool valid{false};
};

/// Normalized platform observation. Raw MAVLink, ROS and sensor fields stay in adapters.
struct WorldState
{
  double now_s{0.0};
  double dt_s{0.0};
  double observed_at_s{0.0};
  bool connected{false};
  bool valid{false};
  bool armed{false};
  bool airborne{false};
  bool landed{true};
  std::string mode{};
  Position position{};
  Velocity velocity{};
  std::map<std::string, bool> facts{};

  bool fresh(double now_s_value, double max_age_s) const;
  bool fact(const std::string & name, bool default_value = false) const;
};

enum class CapabilityAvailability
{
  unsupported,
  unavailable,
  available,
  busy
};

struct CapabilityState
{
  std::map<std::string, CapabilityAvailability> values{};
  std::map<std::string, std::string> reasons{};

  CapabilityAvailability availability(const std::string & capability) const;
  bool supports(const std::string & capability) const;
  bool available(const std::string & capability) const;
  std::string reason(const std::string & capability) const;
};

/// String parameters keep the core independent from JSON, ROS messages and a schema library.
struct ActionIntent
{
  std::string mission_id{};
  std::string action_id{};
  std::string capability{};
  std::map<std::string, std::string> parameters{};
};

enum class ActionStatus
{
  idle,
  accepted,
  running,
  succeeded,
  failed,
  cancelled,
  paused
};

struct ActionFeedback
{
  std::string action_id{};
  std::string capability{};
  ActionStatus status{ActionStatus::idle};
  double progress{0.0};
  std::string reason{};
  double at_s{0.0};
};

struct MissionEvent
{
  std::string name{};
  std::string payload{};
  double at_s{0.0};
};

enum class MissionStatus
{
  idle,
  running,
  paused,
  succeeded,
  failed,
  cancelled
};

struct MissionRuntime
{
  std::string mission_id{"mission"};
  MissionStatus status{MissionStatus::idle};
  std::string active_node{};
  std::string active_action_id{};
  std::string active_capability{};
  bool airborne{false};
  double started_at_s{0.0};
  double finished_at_s{0.0};
  std::string failure_reason{};
};

enum class SafetyAction
{
  allow,
  pause,
  abort,
  emergency
};

struct SafetyDecision
{
  SafetyAction action{SafetyAction::allow};
  std::string reason{};
  std::string resume_condition{};
};

const char * to_string(CapabilityAvailability value);
const char * to_string(ActionStatus value);
const char * to_string(MissionStatus value);
const char * to_string(SafetyAction value);

bool parse_bool(const std::string & value, bool & result);
bool parse_double(const std::string & value, double & result);

}  // namespace mission_core
