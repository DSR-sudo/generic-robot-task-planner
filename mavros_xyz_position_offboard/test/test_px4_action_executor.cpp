#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "mavros_xyz_position_offboard/common/cli.hpp"
#include "mavros_xyz_position_offboard/execution/px4_action_executor.hpp"
#include "mission_core/safety.hpp"

namespace
{

mavros_xyz_position_offboard::common::AppOptions options()
{
  mavros_xyz_position_offboard::common::AppOptions value;
  value.control_enabled = true;
  return value;
}

mavros_xyz_position_offboard::common::Telemetry telemetry()
{
  mavros_xyz_position_offboard::common::Telemetry value;
  value.connected = true;
  value.armed = false;
  value.mode = "MANUAL";
  value.local_pose_at = 1.0;
  value.local_velocity_at = 1.0;
  value.local_x_m = 0.0;
  value.local_y_m = 0.0;
  value.local_z_m = 0.0;
  value.velocity_x_m_s = 0.0;
  value.velocity_y_m_s = 0.0;
  value.velocity_z_m_s = 0.0;
  value.orientation = {0.0, 0.0, 0.0, 1.0};
  value.landed_state = mavros_xyz_position_offboard::common::MAV_LANDED_STATE_ON_GROUND;
  return value;
}

TEST(Px4ActionExecutorTest, ConvertsWorldAndValidatesDomainActions)
{
  auto node = std::make_shared<rclcpp::Node>(
    "px4_action_executor_test", rclcpp::NodeOptions().use_global_arguments(false));
  mavros_xyz_position_offboard::offboard::Offboard offboard(*node, options());
  mavros_xyz_position_offboard::execution::Px4ActionExecutor executor(offboard);
  const auto initial_telemetry = telemetry();
  offboard.observe_flight_state(initial_telemetry);
  executor.observe(initial_telemetry, 1.0);

  const auto capabilities = executor.capabilities(executor.world());
  EXPECT_TRUE(capabilities.available("takeoff"));
  EXPECT_TRUE(capabilities.available("hold"));

  mission_core::ActionIntent takeoff;
  takeoff.mission_id = "test";
  takeoff.action_id = "takeoff";
  takeoff.capability = "takeoff";
  takeoff.parameters["height_m"] = "1.0";
  std::string reason;
  EXPECT_TRUE(executor.submit(takeoff, 1.0, reason)) << reason;
  EXPECT_EQ(
    executor.feedback("takeoff").status, mission_core::ActionStatus::accepted);
  executor.update(executor.world(), 1.5, 0.5);
  EXPECT_EQ(offboard.status().mode, "MANUAL");
  EXPECT_EQ(offboard.status().mode_request.state, "never_requested");
  executor.update(executor.world(), 3.1, 1.6);
  EXPECT_EQ(offboard.status().mode_request.state, "service_not_ready");

  mission_core::ActionIntent invalid;
  invalid.mission_id = "test";
  invalid.action_id = "invalid";
  invalid.capability = "navigate";
  invalid.parameters["frame"] = "map";
  EXPECT_FALSE(executor.submit(invalid, 1.0, reason));
  EXPECT_NE(reason.find("requires numeric parameter"), std::string::npos);
}

TEST(Px4ActionExecutorTest, AirborneCancellationKeepsSafetyLandingActive)
{
  auto node = std::make_shared<rclcpp::Node>(
    "px4_action_executor_cancel_test", rclcpp::NodeOptions().use_global_arguments(false));
  mavros_xyz_position_offboard::offboard::Offboard offboard(*node, options());
  mavros_xyz_position_offboard::execution::Px4ActionExecutor executor(offboard);
  auto state = telemetry();
  state.armed = true;
  state.landed_state = 0;
  executor.observe(state, 1.0);
  mission_core::ActionIntent navigate{
    "test", "navigate", "navigate", {{"x", "1"}, {"y", "0"}, {"z", "0"}}};
  std::string reason;
  ASSERT_TRUE(executor.submit(navigate, 1.0, reason)) << reason;
  executor.cancel("navigate", 1.1);
  EXPECT_EQ(executor.feedback("navigate").status, mission_core::ActionStatus::running);
  EXPECT_EQ(executor.feedback("navigate").capability, "land");
}

TEST(Px4ActionExecutorTest, UnknownStartupStatePausesInsteadOfFailingMission)
{
  auto node = std::make_shared<rclcpp::Node>(
    "px4_startup_test", rclcpp::NodeOptions().use_global_arguments(false));
  mavros_xyz_position_offboard::offboard::Offboard offboard(*node, options());
  mavros_xyz_position_offboard::execution::Px4ActionExecutor executor(offboard);
  mission_core::SafetySupervisor safety;
  executor.observe({}, 0.0);
  EXPECT_FALSE(executor.world().airborne);
  EXPECT_FALSE(executor.world().landed);
  EXPECT_EQ(safety.evaluate(executor.world(), executor.capabilities(executor.world()), {}).action,
    mission_core::SafetyAction::pause);

  executor.observe(telemetry(), 1.0);
  EXPECT_TRUE(executor.capabilities(executor.world()).available("takeoff"));
  EXPECT_EQ(safety.evaluate(executor.world(), executor.capabilities(executor.world()), {}).action,
    mission_core::SafetyAction::allow);

  auto flying = telemetry();
  flying.landed_state = 2;
  executor.observe(flying, 2.0);
  EXPECT_TRUE(executor.world().airborne);
  executor.observe({}, 3.0);
  EXPECT_TRUE(executor.world().airborne);
  EXPECT_EQ(safety.evaluate(executor.world(), executor.capabilities(executor.world()), {}).action,
    mission_core::SafetyAction::emergency);
  executor.observe(telemetry(), 4.0);
  EXPECT_FALSE(executor.world().airborne);
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
