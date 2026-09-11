#include "mavros_xyz_position_offboard/application/application_node.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace mavros_xyz_position_offboard::application
{
namespace
{

const rcl_interfaces::msg::ParameterDescriptor & safety_parameter_descriptor()
{
  static const rcl_interfaces::msg::ParameterDescriptor descriptor = [] {
      rcl_interfaces::msg::ParameterDescriptor value;
      value.read_only = true;
      value.description = "Applied only during node creation; restart the node to change it.";
      return value;
    }();
  return descriptor;
}

}  // namespace

/// 读取不受 ROS 时间跳变影响的 steady_clock 秒数。
double ApplicationNode::monotonic_now()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

/// 创建唯一节点并按声明顺序组装全部运行期模块和 20 Hz wall timer。
ApplicationNode::ApplicationNode(
  const common::AppOptions & options, const common::SafetyConfig & config,
  const rclcpp::NodeOptions & node_options)
: Node("mavros_native_xyz_position", node_options), options_(options), config_(load_safety_config(config)),
  initialization_(*this, options_, config_), ground_station_(load_ground_station_config()),
  offboard_(*this, options_),
  lcp_vision_bridge_(*this, options_), z_config_(load_z_config()),
  artifact_log_(options_.artifact_dir, options_.output == "jsonl")
{
  px4_executor_ = std::make_unique<execution::Px4ActionExecutor>(
    offboard_, config_.target_tolerance_m, config_.setpoint_warmup_s);
  const auto mission_path = load_mission_path();
  mission_tree_ = std::make_unique<mission_core::MissionTree>(mission_path, "px4_mission");
  const auto capability_check = mission_tree_->validate_capabilities(
    px4_executor_->capabilities(px4_executor_->world()));
  if (!capability_check.accepted) {
    throw std::invalid_argument(capability_check.reason);
  }
  mission_core::SafetyConfig safety;
  safety.abort_when_grounded_state_invalid = true;
  safety.max_mission_duration_s = config_.max_flight_seconds;
  safety.pause_when_fact_false = {"lcp_healthy"};
  safety.pause_fact_exempt_capabilities = {"land"};
  mission_safety_ = mission_core::SafetySupervisor(safety);
  RCLCPP_INFO(get_logger(), "using mission_core runner: %s", mission_path.c_str());
  const auto lcp_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  lcp_debug_subscription_ = create_subscription<lslidar_msgs::msg::LcpDebug>(
    "/lcp/debug", lcp_qos, std::bind(&ApplicationNode::lcp_debug_callback, this, std::placeholders::_1));
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / config_.publish_rate_hz),
    std::bind(&ApplicationNode::tick, this));
}

/// 在节点销毁时显式落盘并关闭 artifact 日志。
ApplicationNode::~ApplicationNode() {artifact_log_.close();}

common::SafetyConfig ApplicationNode::load_safety_config(const common::SafetyConfig & defaults)
{
  auto value = defaults;
  const auto & descriptor = safety_parameter_descriptor();
  value.state_timeout_s = declare_parameter<double>("safety.state_timeout_s", value.state_timeout_s, descriptor);
  value.sys_status_timeout_s = declare_parameter<double>(
    "safety.sys_status_timeout_s", value.sys_status_timeout_s, descriptor);
  value.battery_timeout_s = declare_parameter<double>("safety.battery_timeout_s", value.battery_timeout_s, descriptor);
  value.landed_timeout_s = declare_parameter<double>("safety.landed_timeout_s", value.landed_timeout_s, descriptor);
  value.local_pose_timeout_s = declare_parameter<double>(
    "safety.local_pose_timeout_s", value.local_pose_timeout_s, descriptor);
  value.local_velocity_timeout_s = declare_parameter<double>(
    "safety.local_velocity_timeout_s", value.local_velocity_timeout_s, descriptor);
  value.estimator_timeout_s = declare_parameter<double>(
    "safety.estimator_timeout_s", value.estimator_timeout_s, descriptor);
  value.range_timeout_s = declare_parameter<double>("safety.range_timeout_s", value.range_timeout_s, descriptor);
  value.optical_flow_timeout_s = declare_parameter<double>(
    "safety.optical_flow_timeout_s", value.optical_flow_timeout_s, descriptor);
  value.sensor_loss_grace_s = declare_parameter<double>(
    "safety.sensor_loss_grace_s", value.sensor_loss_grace_s, descriptor);
  value.lcp_status_timeout_s = declare_parameter<double>(
    "safety.lcp_status_timeout_s", value.lcp_status_timeout_s, descriptor);
  value.lcp_odometry_timeout_s = declare_parameter<double>(
    "safety.lcp_odometry_timeout_s", value.lcp_odometry_timeout_s, descriptor);
  value.lcp_ready_samples = declare_parameter<int>("safety.lcp_ready_samples", value.lcp_ready_samples, descriptor);
  value.range_boundary_tolerance_m = declare_parameter<double>(
    "safety.range_boundary_tolerance_m", value.range_boundary_tolerance_m, descriptor);
  value.configured_min_range_m = declare_parameter<double>(
    "safety.configured_min_range_m", value.configured_min_range_m, descriptor);
  value.configured_max_range_m = declare_parameter<double>(
    "safety.configured_max_range_m", value.configured_max_range_m, descriptor);
  value.max_range_jump_m = declare_parameter<double>(
    "safety.max_range_jump_m", value.max_range_jump_m, descriptor);
  value.jump_window_s = declare_parameter<double>("safety.jump_window_s", value.jump_window_s, descriptor);
  value.jump_recovery_samples = declare_parameter<int>(
    "safety.jump_recovery_samples", value.jump_recovery_samples, descriptor);
  value.jump_settle_tolerance_m = declare_parameter<double>(
    "safety.jump_settle_tolerance_m", value.jump_settle_tolerance_m, descriptor);
  value.min_optical_flow_quality = declare_parameter<int>(
    "safety.min_optical_flow_quality", value.min_optical_flow_quality, descriptor);
  value.min_battery_voltage_v = declare_parameter<double>(
    "safety.min_battery_voltage_v", value.min_battery_voltage_v, descriptor);
  value.min_battery_fraction = declare_parameter<double>(
    "safety.min_battery_fraction", value.min_battery_fraction, descriptor);
  value.max_preflight_horizontal_speed_m_s = declare_parameter<double>(
    "safety.max_preflight_horizontal_speed_m_s", value.max_preflight_horizontal_speed_m_s, descriptor);
  value.max_preflight_vertical_speed_m_s = declare_parameter<double>(
    "safety.max_preflight_vertical_speed_m_s", value.max_preflight_vertical_speed_m_s, descriptor);
  value.max_flight_horizontal_speed_m_s = declare_parameter<double>(
    "safety.max_flight_horizontal_speed_m_s", value.max_flight_horizontal_speed_m_s, descriptor);
  value.max_flight_vertical_speed_m_s = declare_parameter<double>(
    "safety.max_flight_vertical_speed_m_s", value.max_flight_vertical_speed_m_s, descriptor);
  value.max_flight_horizontal_drift_m = declare_parameter<double>(
    "safety.max_flight_horizontal_drift_m", value.max_flight_horizontal_drift_m, descriptor);
  value.climb_horizontal_speed_limit_m_s = declare_parameter<double>(
    "safety.climb_horizontal_speed_limit_m_s", value.climb_horizontal_speed_limit_m_s, descriptor);
  value.climb_horizontal_drift_limit_m = declare_parameter<double>(
    "safety.climb_horizontal_drift_limit_m", value.climb_horizontal_drift_limit_m, descriptor);
  value.hover_min_height_m = declare_parameter<double>(
    "safety.hover_min_height_m", value.hover_min_height_m, descriptor);
  value.publish_rate_hz = declare_parameter<double>("safety.publish_rate_hz", value.publish_rate_hz, descriptor);
  value.setpoint_warmup_s = declare_parameter<double>(
    "safety.setpoint_warmup_s", value.setpoint_warmup_s, descriptor);
  value.max_z_setpoint_rate_m_s = declare_parameter<double>(
    "safety.max_z_setpoint_rate_m_s", value.max_z_setpoint_rate_m_s, descriptor);
  value.max_z_setpoint_accel_m_s2 = declare_parameter<double>(
    "safety.max_z_setpoint_accel_m_s2", value.max_z_setpoint_accel_m_s2, descriptor);
  value.target_xy_max_speed_m_s = declare_parameter<double>(
    "safety.target_xy_max_speed_m_s", value.target_xy_max_speed_m_s, descriptor);
  value.target_xy_max_accel_m_s2 = declare_parameter<double>(
    "safety.target_xy_max_accel_m_s2", value.target_xy_max_accel_m_s2, descriptor);
  value.target_tolerance_m = declare_parameter<double>(
    "safety.target_tolerance_m", value.target_tolerance_m, descriptor);
  value.touchdown_z_tolerance_m = declare_parameter<double>(
    "safety.touchdown_z_tolerance_m", value.touchdown_z_tolerance_m, descriptor);
  value.max_flight_seconds = declare_parameter<double>(
    "safety.max_flight_seconds", value.max_flight_seconds, descriptor);
  value.flow_effective_min_height_m = declare_parameter<double>(
    "safety.flow_effective_min_height_m", value.flow_effective_min_height_m, descriptor);
  value.flow_effective_min_quality = declare_parameter<int>(
    "safety.flow_effective_min_quality", value.flow_effective_min_quality, descriptor);
  value.ignore_declared_min_range = declare_parameter<bool>(
    "safety.ignore_declared_min_range", value.ignore_declared_min_range, descriptor);
  value.validate();
  return value;
}

/// 从 ROS 参数构造并校验通用 UDP gateway 的固定启动配置。
communication::GroundStationConfig ApplicationNode::load_ground_station_config()
{
  communication::GroundStationConfig value;
  value.enabled = declare_parameter<bool>("udp.enabled", value.enabled);
  value.bind_ip = declare_parameter<std::string>("udp.bind_ip", value.bind_ip);
  value.bind_port = declare_parameter<int>("udp.bind_port", value.bind_port);
  value.remote_ip = declare_parameter<std::string>("udp.remote_ip", value.remote_ip);
  value.remote_port = declare_parameter<int>("udp.remote_port", value.remote_port);
  value.whitelist_ip = declare_parameter<std::string>("udp.whitelist_ip", value.whitelist_ip);
  value.whitelist_port = declare_parameter<int>("udp.whitelist_port", value.whitelist_port);
  const int inbox_capacity = declare_parameter<int>(
    "udp.inbox_capacity", static_cast<int>(value.inbox_capacity));
  if (inbox_capacity <= 0) {
    throw std::invalid_argument("udp.inbox_capacity must be within 1..4096");
  }
  value.inbox_capacity = static_cast<std::size_t>(inbox_capacity);
  value.message_ttl_s = declare_parameter<double>("udp.message_ttl_s", value.message_ttl_s);
  value.validate();
  return value;
}

std::string ApplicationNode::load_mission_path()
{
  const auto default_path = ament_index_cpp::get_package_share_directory(
    "mavros_xyz_position_offboard") + "/config/generic_flight_mission.xml";
  return declare_parameter<std::string>(
    "mission.xml_path", default_path, safety_parameter_descriptor());
}

void ApplicationNode::ZConfig::validate() const
{
  if (!std::isfinite(source_timeout_s) || !std::isfinite(range_cross_check_max_delta_m) ||
    source_timeout_s <= 0.0 || range_cross_check_max_delta_m <= 0.0) {
    throw std::invalid_argument("Z source timeout and range cross-check delta must be finite and positive");
  }
}

ApplicationNode::ZConfig ApplicationNode::load_z_config()
{
  ZConfig value;
  value.prefer_range = declare_parameter<bool>("z.prefer_range", value.prefer_range);
  value.source_timeout_s = declare_parameter<double>("z.source_timeout_s", value.source_timeout_s);
  value.range_cross_check_max_delta_m = declare_parameter<double>(
    "z.range_cross_check_max_delta_m", value.range_cross_check_max_delta_m);
  value.validate();
  return value;
}

void ApplicationNode::latch_init_height(const common::Telemetry & telemetry)
{
  if (init_local_z_m_ || init_range_m_) {return;}
  init_local_z_m_.reset();
  init_range_m_.reset();
  if (common::finite(telemetry.local_z_m)) {init_local_z_m_ = telemetry.local_z_m;}
  if (common::finite(telemetry.range_m) && !initialization_.range_fault()) {init_range_m_ = telemetry.range_m;}
}

void ApplicationNode::lcp_debug_callback(const lslidar_msgs::msg::LcpDebug::SharedPtr message)
{
  const double now = monotonic_now();
  communication::XyzStatus status;
  status.header = {message->header.stamp.sec, message->header.stamp.nanosec, message->header.frame_id};
  status.status = message->status;
  status.map_locked = message->map_locked;
  status.pose_valid = message->pose_valid;
  status.position_x_m = message->position_x_m;
  status.position_y_m = message->position_y_m;
  status.yaw_rad = message->yaw_rad;
  status.front_distance_m = message->front_distance_m;
  status.rear_distance_m = message->rear_distance_m;
  status.left_distance_m = message->left_distance_m;
  status.right_distance_m = message->right_distance_m;
  status.map_size_x_m = message->map_size_x_m;
  status.map_size_y_m = message->map_size_y_m;

  const auto & telemetry = initialization_.telemetry();
  const bool local_fresh = init_local_z_m_ && telemetry.local_pose_stamp &&
    !common::stale(telemetry.local_pose_at, now, z_config_.source_timeout_s) &&
    common::finite(telemetry.local_z_m);
  const bool range_fresh = init_range_m_ && telemetry.range_stamp && !initialization_.range_fault() &&
    !common::stale(telemetry.range_at, now, z_config_.source_timeout_s) && common::finite(telemetry.range_m);
  const std::optional<double> local_z = local_fresh ?
    std::optional<double>(telemetry.local_z_m - *init_local_z_m_) : std::nullopt;
  const std::optional<double> range_z = range_fresh ?
    std::optional<double>(telemetry.range_m - *init_range_m_) : std::nullopt;
  const bool sources_valid = local_z && range_z &&
    std::abs(*local_z - *range_z) <= z_config_.range_cross_check_max_delta_m;

  if (sources_valid) {
    const bool use_range = z_config_.prefer_range;
    status.position_z_m = use_range ? range_z : local_z;
    status.z_source = use_range ? "range" : "local_pose";
    status.z_source_stamp = use_range ? telemetry.range_stamp : telemetry.local_pose_stamp;
    status.z_valid = true;
  }
  // Invalid Z intentionally retains every LCP field and the protocol's null/none metadata.
  try {
    ground_station_.send_xyzstatus(status);
  } catch (const std::exception & error) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "xyzstatus rejected: %s", error.what());
  }
}

/// 执行“轮询、快照、行为树、飞控、通信、日志”的单线程控制周期。
void ApplicationNode::tick()
{
  tick_generic();
}

void ApplicationNode::tick_generic()
{
  const double now = monotonic_now();
  const double dt = last_tick_at_ ? std::max(0.001, std::min(0.25, now - *last_tick_at_)) :
    1.0 / config_.publish_rate_hz;
  last_tick_at_ = now;

  initialization_.poll_lcp_start(now, options_.service_timeout);
  offboard_.poll(now);
  const auto udp_results = ground_station_.poll(now);
  std::vector<communication::UdpMessage> udp_messages;
  for (const auto & result : udp_results) {
    if (result.message) {udp_messages.push_back(*result.message);}
  }

  const bool in_flight = px4_executor_->world().airborne;
  auto health = initialization_.health_snapshot(now, in_flight, NAN, NAN, false, false);
  if (!in_flight && health.telemetry.local_pose_at) {
    latch_init_height(health.telemetry);
  }
  const auto & lcp_state = health.telemetry.lcp_init_request_state;
  if (initialization_.lcp_start_prerequisite_errors(now).empty() &&
    (lcp_state == "not_requested" || lcp_state == "waiting_service"))
  {
    initialization_.request_lcp_start(now);
    health = initialization_.health_snapshot(now, in_flight, NAN, NAN, false, false);
  }

  bool flight_healthy = health.flight_errors.empty();
  if (in_flight && !flight_healthy) {
    if (!sensor_fault_since_) {sensor_fault_since_ = now;}
    flight_healthy = now - *sensor_fault_since_ <= config_.sensor_loss_grace_s;
  } else {
    sensor_fault_since_.reset();
  }

  offboard_.observe_flight_state(health.telemetry);
  px4_executor_->observe(health.telemetry, now);
  auto world = px4_executor_->world();
  world.now_s = now;
  world.dt_s = dt;
  world.facts["preflight_ready"] = health.preflight_errors.empty() && health.lcp_ready;
  world.facts["flight_healthy"] = flight_healthy;
  world.facts["lcp_healthy"] = health.lcp_healthy;
  if (!flight_healthy) {
    world.valid = false;
  }

  std::vector<mission_core::MissionEvent> events;
  events.reserve(udp_messages.size());
  for (const auto & message : udp_messages) {
    events.push_back({message.header, message.data_json, message.received_at});
  }
  const auto result = mission_tree_->tick(
    world, px4_executor_->capabilities(world), events, *px4_executor_, mission_safety_);
  for (const auto & event : result.emitted_events) {
    try {
      ground_station_.send({event.name, event.payload.empty() ? "{}" : event.payload, event.at_s});
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "generic mission event rejected: %s", error.what());
    }
  }
  emit_generic_status(now, health, result);
}

void ApplicationNode::emit_generic_status(
  const double now, const initialization::HealthSnapshot & health,
  const mission_core::MissionTreeResult & result)
{
  if (now - last_log_at_ < options_.status_period) {return;}
  last_log_at_ = now;
  std::ostringstream stream;
  stream << "{\"schema\":\"mission.runtime.v1\",\"mission_id\":\"px4_mission\",\"status\":\""
         << mission_core::to_string(result.runtime.status)
         << "\",\"tree_status\":\"" << mission_core::to_string(result.tree_status)
         << "\",\"active_node\":\"" << common::json_escape(result.runtime.active_node)
         << "\",\"active_action_id\":\"" << common::json_escape(result.runtime.active_action_id)
         << "\",\"safety\":\"" << mission_core::to_string(result.safety.action)
         << "\",\"safety_reason\":\"" << common::json_escape(result.safety.reason)
         << "\",\"error\":\"" << common::json_escape(result.error)
         << "\",\"airborne\":" << (result.runtime.airborne ? "true" : "false")
         << ",\"monotonic_s\":" << std::setprecision(12) << now
         << ",\"preflight_errors\":[";
  for (std::size_t i = 0; i < health.preflight_errors.size(); ++i) {
    if (i) {stream << ',';}
    stream << '"' << common::json_escape(health.preflight_errors[i]) << '"';
  }
  stream << "],\"flight_errors\":[";
  for (std::size_t i = 0; i < health.flight_errors.size(); ++i) {
    if (i) {stream << ',';}
    stream << '"' << common::json_escape(health.flight_errors[i]) << '"';
  }
  stream << "],\"telemetry\":" << initialization_.telemetry_json(now) << "}";
  artifact_log_.write(stream.str());
  std::cout << "mission_status=" << mission_core::to_string(result.runtime.status)
            << " safety=" << mission_core::to_string(result.safety.action)
            << " errors=" << health.flight_errors.size() << std::endl;
}

}  // namespace mavros_xyz_position_offboard::application
