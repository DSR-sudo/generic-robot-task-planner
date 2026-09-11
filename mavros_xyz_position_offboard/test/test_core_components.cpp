#include <cmath>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mavros_xyz_position_offboard/bridge/lcp_vision_bridge.hpp"
#include "mavros_xyz_position_offboard/common/cli.hpp"
#include "mavros_xyz_position_offboard/initialization/initialization.hpp"
#include "mavros_xyz_position_offboard/offboard/offboard.hpp"

namespace
{
using mavros_xyz_position_offboard::common::AppOptions;
using mavros_xyz_position_offboard::common::SafetyConfig;
using mavros_xyz_position_offboard::common::parse_options;
using mavros_xyz_position_offboard::initialization::Initialization;

AppOptions test_options()
{
  AppOptions options;
  options.range_topic = "/test/range";
  options.optical_flow_topic = "/test/flow";
  options.lcp_status_topic = "/test/lcp/status";
  options.lcp_odometry_topic = "/test/lcp/odometry";
  options.lcp_start_service = "/test/lcp/start";
  options.lcp_vision_bridge_enabled = false;
  return options;
}

TEST(CliOptionsTest, UsesGenericControlModeInsteadOfManualAuthorizationFlags)
{
  const std::vector<std::string> base_arguments{
    "mavros_xyz_position_node", "--range-topic", "/test/range",
    "--optical-flow-topic", "/test/flow"};
  const auto monitoring = parse_options(base_arguments);
  EXPECT_FALSE(monitoring.options.control_enabled);

  auto control_arguments = base_arguments;
  control_arguments.emplace_back("--enable-control");
  const auto control = parse_options(control_arguments);
  EXPECT_TRUE(control.options.control_enabled);

  auto legacy_arguments = base_arguments;
  legacy_arguments.emplace_back("--confirm-range-source");
  EXPECT_THROW(parse_options(legacy_arguments), std::invalid_argument);
}

TEST(InitializationTest, LcpReadinessRequiresFreshPostRequestSamples)
{
  auto node = std::make_shared<rclcpp::Node>(
    "mavros_xyz_initialization_test", rclcpp::NodeOptions().use_global_arguments(false));
  auto options = test_options();
  SafetyConfig config;
  config.lcp_ready_samples = 3;
  Initialization initialization(*node, options, config);
  initialization.update_lcp_status(2, 1.0);
  initialization.update_lcp_odometry(1.0, 2.0, 0.0, 1.0);
  initialization.begin_lcp_initialization(2.0);
  initialization.update_lcp_init_state("accepted", 2.0, "started");
  EXPECT_FALSE(initialization.lcp_ready(2.1));
  for (const double stamp : {2.1, 2.2, 2.3}) {
    initialization.update_lcp_status(2, stamp);
    initialization.update_lcp_odometry(1.0, 2.0, 0.0, stamp);
  }
  EXPECT_TRUE(initialization.lcp_ready(2.3));
}

TEST(InitializationTest, GroundLcpStartGateDoesNotPretendBatteryIsReady)
{
  auto node = std::make_shared<rclcpp::Node>(
    "mavros_xyz_lcp_start_gate_test", rclcpp::NodeOptions().use_global_arguments(false));
  auto options = test_options();
  SafetyConfig config;
  Initialization initialization(*node, options, config);
  initialization.update_state(true, false, "MANUAL", 0, 10.0);
  initialization.update_landed(
    mavros_xyz_position_offboard::common::MAV_LANDED_STATE_ON_GROUND, 10.0);
  initialization.update_battery(true, 0.0, NAN, 10.0);
  EXPECT_TRUE(initialization.lcp_start_prerequisite_errors(10.1).empty());
  EXPECT_FALSE(initialization.preflight_errors(10.1).empty());
}

TEST(LcpVisionBridgeTest, ConvertsNwuToEnu)
{
  nav_msgs::msg::Odometry source;
  source.header.frame_id = "lcp_nwu";
  source.pose.pose.position.x = 2.0;
  source.pose.pose.position.y = 3.0;
  source.pose.pose.orientation.w = 1.0;
  const auto output = mavros_xyz_position_offboard::bridge::LcpVisionBridge::nwu_to_enu(
    source, 0.20, 0.20);
  EXPECT_EQ(output.header.frame_id, "lcp_enu");
  EXPECT_DOUBLE_EQ(output.pose.pose.position.x, -3.0);
  EXPECT_DOUBLE_EQ(output.pose.pose.position.y, 2.0);
}

TEST(OffboardMappingTest, PreservesRosEnuForMavrosLocalNedConversion)
{
  mavros_xyz_position_offboard::common::PositionSetpoint setpoint;
  setpoint.x_m = 1.0;
  setpoint.y_m = 2.0;
  setpoint.z_m = 3.0;
  setpoint.orientation = {0.0, 0.0, 0.0, 1.0};
  builtin_interfaces::msg::Time stamp;
  const auto target = mavros_xyz_position_offboard::offboard::Offboard::make_position_target(
    setpoint, stamp);
  EXPECT_EQ(target.coordinate_frame, mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED);
  EXPECT_EQ(target.type_mask & mavros_msgs::msg::PositionTarget::IGNORE_PX, 0U);
  EXPECT_EQ(target.type_mask & mavros_msgs::msg::PositionTarget::IGNORE_YAW, 0U);
  EXPECT_DOUBLE_EQ(target.position.x, 1.0);
  EXPECT_DOUBLE_EQ(target.position.y, 2.0);
  EXPECT_DOUBLE_EQ(target.position.z, 3.0);
  EXPECT_NEAR(target.yaw, 0.0, 1e-6);
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
