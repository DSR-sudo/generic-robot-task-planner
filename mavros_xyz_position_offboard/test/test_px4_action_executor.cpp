#include <gtest/gtest.h>
#include <chrono>
#include <thread>

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
  EXPECT_EQ(offboard.status().arm_request.state, "never_requested");
  auto confirmed = initial_telemetry;
  confirmed.mode = "OFFBOARD";
  offboard.observe_flight_state(confirmed);
  executor.observe(confirmed, 3.2);
  executor.update(executor.world(), 3.2, 0.1);
  EXPECT_EQ(offboard.status().arm_request.state, "service_not_ready");

  mission_core::ActionIntent invalid;
  invalid.mission_id = "test";
  invalid.action_id = "invalid";
  invalid.capability = "navigate";
  invalid.parameters["frame"] = "map";
  EXPECT_FALSE(executor.submit(invalid, 1.0, reason));
  EXPECT_NE(reason.find("requires numeric parameter"), std::string::npos);
}

TEST(Px4ActionExecutorTest, PausedGroundTakeoffNeverArmsAndCancelledActionStaysTerminal)
{
  auto node = std::make_shared<rclcpp::Node>(
    "px4_pause_test", rclcpp::NodeOptions().use_global_arguments(false));
  auto opts = options();
  mavros_xyz_position_offboard::offboard::Offboard offboard(*node, opts);
  mavros_xyz_position_offboard::execution::Px4ActionExecutor executor(offboard);
  auto state = telemetry(); state.mode = "OFFBOARD";
  offboard.observe_flight_state(state); executor.observe(state, 1.0);
  mission_core::ActionIntent takeoff{"test", "takeoff", "takeoff", {{"height_m", "1.4"}}};
  std::string reason;
  ASSERT_TRUE(executor.submit(takeoff, 1.0, reason));
  executor.pause("takeoff", 1.1);
  executor.update(executor.world(), 4.0, .1);
  EXPECT_EQ(offboard.status().arm_request.state, "never_requested");
  EXPECT_EQ(executor.feedback("takeoff").status, mission_core::ActionStatus::paused);
  executor.cancel("takeoff", 4.1);
  executor.update(executor.world(), 5.0, .1);
  EXPECT_EQ(executor.feedback("takeoff").status, mission_core::ActionStatus::cancelled);
  EXPECT_EQ(offboard.status().arm_request.state, "never_requested");
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

TEST(Px4ActionExecutorTest, AirbornePausePublishesFrozenPositionUntilResume)
{
  auto node = std::make_shared<rclcpp::Node>(
    "px4_freeze_test", rclcpp::NodeOptions().use_global_arguments(false));
  auto opts = options(); opts.setpoint_topic = "/px4_freeze_test/setpoint";
  mavros_xyz_position_offboard::offboard::Offboard offboard(*node, opts);
  mavros_xyz_position_offboard::execution::Px4ActionExecutor executor(offboard);
  auto state = telemetry(); state.armed = true; state.landed_state = 2;
  state.mode = "OFFBOARD"; state.local_x_m = 1.0; state.local_z_m = .8;
  offboard.observe_flight_state(state); executor.observe(state, 1.0);
  mission_core::ActionIntent move{
    "test", "move", "navigate", {{"x", "5"}, {"y", "0"}, {"z", "1.4"}}};
  std::string reason;
  ASSERT_TRUE(executor.submit(move, 1.0, reason));
  executor.pause("move", 1.1);
  state.local_x_m = 1.2; state.local_z_m = .9; executor.observe(state, 1.2);
  std::optional<mavros_msgs::msg::PositionTarget> received;
  auto subscription = node->create_subscription<mavros_msgs::msg::PositionTarget>(
    opts.setpoint_topic, rclcpp::SensorDataQoS(),
    [&](mavros_msgs::msg::PositionTarget::SharedPtr message) {received = *message;});
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!received && std::chrono::steady_clock::now() < deadline) {
    executor.update(executor.world(), 1.3, .05);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(received);
  EXPECT_DOUBLE_EQ(received->position.x, 1.0);
  EXPECT_DOUBLE_EQ(received->position.z, .8);
  EXPECT_EQ(executor.feedback("move").status, mission_core::ActionStatus::paused);
  EXPECT_EQ(offboard.status().arm_request.state, "never_requested");
}


TEST(Px4ActionExecutorTest, SquareUsesFixedRotatedTakeoffOrigin)
{
  auto node = std::make_shared<rclcpp::Node>("square_origin_test");
  mavros_xyz_position_offboard::offboard::Offboard offboard(*node, options());
  mavros_xyz_position_offboard::execution::Px4ActionExecutor executor(offboard);
  auto t = telemetry();
  t.local_x_m = 3.0; t.local_y_m = 4.0; t.local_z_m = .2;
  t.orientation = {0.0, 0.0, std::sqrt(.5), std::sqrt(.5)};
  executor.observe(t, 1.0);
  mission_core::ActionIntent a;
  a.action_id = "right"; a.capability = "navigate";
  a.parameters = {{"frame", "takeoff_flu"}, {"x", "0"}, {"y", "-1"}, {"z", "1.4"}};
  std::string reason;
  EXPECT_FALSE(executor.submit(a, 1.0, reason));
  mission_core::ActionIntent takeoff;
  takeoff.action_id = "takeoff"; takeoff.capability = "takeoff";
  takeoff.parameters = {{"height_m", "1.4"}};
  ASSERT_TRUE(executor.submit(takeoff, 1.0, reason));
  t.armed = true; t.mode = "OFFBOARD"; t.landed_state = 2; t.local_z_m = 1.6;
  executor.observe(t, 4.0); executor.update(executor.world(), 4.0, .05);
  ASSERT_EQ(executor.feedback("takeoff").status, mission_core::ActionStatus::succeeded);
  const double targets[4][4] = {{0,-1,4,4}, {-1,-1,4,3}, {-1,0,3,3}, {0,0,3,4}};
  for (int i = 0; i < 4; ++i) {
    a.action_id = "leg" + std::to_string(i);
    a.parameters["x"] = std::to_string(targets[i][0]);
    a.parameters["y"] = std::to_string(targets[i][1]);
    ASSERT_TRUE(executor.submit(a, 5.0+i, reason)) << reason;
    executor.update(executor.world(), 5.0+i, .05);
    EXPECT_NE(executor.feedback(a.action_id).status, mission_core::ActionStatus::succeeded);
    t.local_x_m = targets[i][2]; t.local_y_m = targets[i][3];
    executor.observe(t, 5.1+i); executor.update(executor.world(), 5.1+i, .05);
    EXPECT_EQ(executor.feedback(a.action_id).status, mission_core::ActionStatus::succeeded);
  }
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
