#pragma once

#include <string>
#include <vector>

#include "mavros_xyz_position_offboard/common/types.hpp"

namespace mavros_xyz_position_offboard::common
{

struct AppOptions
{
  std::string range_topic;
  std::string optical_flow_topic;
  std::string state_topic{"/mavros/state"};
  std::string sys_status_topic{"/mavros/sys_status"};
  std::string battery_topic{"/mavros/battery"};
  std::string extended_state_topic{"/mavros/extended_state"};
  std::string local_pose_topic{"/mavros/local_position/pose"};
  std::string local_velocity_topic{"/mavros/local_position/velocity_local"};
  std::string estimator_status_topic{"/mavros/estimator_status"};
  std::string setpoint_topic{"/mavros/setpoint_raw/local"};
  std::string lcp_start_service{"/lcp/start_initialization"};
  std::string lcp_status_topic{"/lcp/status"};
  std::string lcp_odometry_topic{"/lcp/odometry"};
  std::string lcp_vision_pose_topic{"/mavros/vision_pose/pose_cov"};
  std::string lcp_vision_input_frame{"lcp_nwu"};
  std::string output{"summary"};
  std::string artifact_dir{"artifacts"};
  double status_period{0.5};
  double mode_request_interval{2.0};
  double service_timeout{3.0};
  double lcp_vision_xy_stddev_m{0.20};
  double lcp_vision_yaw_stddev_rad{0.20};
  double lcp_vision_max_status_age_s{0.35};
  bool lcp_vision_bridge_enabled{true};
  /// 是否允许平台执行器发送控制设定点、模式和 ARM/Disarm 请求。
  bool control_enabled{false};
};

struct ParsedOptions
{
  AppOptions options;
  SafetyConfig config;
};

/// 解析非 ROS 参数和运行配置，失败时抛出异常。
ParsedOptions parse_options(const std::vector<std::string> & argv);
/// 返回命令行入口的简要用法文本。
std::string usage();

}  // namespace mavros_xyz_position_offboard::common
