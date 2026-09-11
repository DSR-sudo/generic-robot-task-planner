#pragma once

#include <map>
#include <optional>
#include <string>

#include "mission_core/executor.hpp"
#include "mavros_xyz_position_offboard/common/types.hpp"
#include "mavros_xyz_position_offboard/offboard/offboard.hpp"

namespace mavros_xyz_position_offboard::execution
{

/// Converts normalized mission actions into PX4/MAVROS control intents.
///
/// This adapter owns no mission tree state. It keeps only one platform action active and
/// continues publishing a safe land command when an airborne action is cancelled.
class Px4ActionExecutor final : public mission_core::IActionExecutor
{
public:
  explicit Px4ActionExecutor(
    offboard::Offboard & offboard, double target_tolerance_m = 0.04,
    double offboard_warmup_s = 2.0);

  /// Convert the latest PX4/LCP snapshot into the platform-neutral world model.
  void observe(const common::Telemetry & telemetry, double now_s);
  const mission_core::WorldState & world() const {return world_;}

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

private:
  struct ActiveAction
  {
    mission_core::ActionIntent intent{};
    mission_core::ActionFeedback feedback{};
    std::optional<mission_core::Position> target{};
    std::optional<double> duration_s{};
    double started_at_s{0.0};
    double paused_at_s{0.0};
    double paused_duration_s{0.0};
    bool paused{false};
    bool safety_land{false};
  };

  static mission_core::ValidationResult validate_position_action(
    const mission_core::ActionIntent & intent);
  static mission_core::ValidationResult validate_takeoff_action(
    const mission_core::ActionIntent & intent);
  static mission_core::ValidationResult validate_hold_action(
    const mission_core::ActionIntent & intent);
  static bool parse_position(
    const mission_core::ActionIntent & intent, mission_core::Position & position,
    std::string & reason);
  static bool parse_optional_duration(
    const mission_core::ActionIntent & intent, std::optional<double> & duration,
    std::string & reason);
  bool target_reached(const mission_core::Position & target) const;
  void apply_active_command(double now_s);
  void finish(mission_core::ActionStatus status, std::string reason, double now_s);

  offboard::Offboard & offboard_;
  mission_core::CapabilityRegistry registry_{};
  mission_core::WorldState world_{};
  std::optional<ActiveAction> active_{};
  double target_tolerance_m_{0.04};
  double offboard_warmup_s_{2.0};
};

}  // namespace mavros_xyz_position_offboard::execution
