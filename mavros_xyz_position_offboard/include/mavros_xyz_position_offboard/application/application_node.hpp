#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <lslidar_msgs/msg/lcp_debug.hpp>

#include "mavros_xyz_position_offboard/bridge/lcp_vision_bridge.hpp"
#include "mavros_xyz_position_offboard/common/artifact_log.hpp"
#include "mavros_xyz_position_offboard/common/cli.hpp"
#include "mavros_xyz_position_offboard/communication/ground_station_link.hpp"
#include "mavros_xyz_position_offboard/initialization/initialization.hpp"
#include "mavros_xyz_position_offboard/offboard/offboard.hpp"
#include "mavros_xyz_position_offboard/execution/px4_action_executor.hpp"
#include "mission_core/mission_tree.hpp"
#include "mission_core/safety.hpp"

namespace mavros_xyz_position_offboard::application
{

class ApplicationNode final : public rclcpp::Node
{
public:
  /// 创建唯一 ROS 2 节点，并按固定依赖顺序组装初始化、通信、导航和飞控执行模块。
  ApplicationNode(
    const common::AppOptions & options, const common::SafetyConfig & config,
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  /// 关闭应用持有的 artifact 日志并释放各模块资源。
  ~ApplicationNode() override;
  /// 返回节点启动时锁定的有效安全配置，供状态检查和测试读取。
  const common::SafetyConfig & safety_config() const {return config_;}

private:
  struct ZConfig
  {
    bool prefer_range{false};
    double source_timeout_s{0.5};
    double range_cross_check_max_delta_m{0.30};

    void validate() const;
  };

  /// 返回供控制周期、超时和日志使用的单调时钟秒数。
  static double monotonic_now();
  /// 以 CLI 解析值为默认值声明、读取并锁定全部活动 safety.* 参数。
  common::SafetyConfig load_safety_config(const common::SafetyConfig & defaults);
  /// 声明并读取启动时生效的 UDP 端点、白名单和通用 inbox 参数。
  communication::GroundStationConfig load_ground_station_config();
  /// 读取并解析启动时固定的通用任务 XML 路径。
  std::string load_mission_path();
  /// 读取 Z 源选择及 local/range 一致性校验参数。
  ZConfig load_z_config();
  /// 将每个新鲜 LcpDebug 样本直接编码并上送为 xyzstatus。
  void lcp_debug_callback(const lslidar_msgs::msg::LcpDebug::SharedPtr message);
  /// 在首次获得有效地面状态时记录 local Z 与可用 Range 的任务基准。
  void latch_init_height(const common::Telemetry & telemetry);
  /// 按规定的六步顺序执行一次 20 Hz 单线程应用控制循环。
  void tick();
  /// 执行平台无关 XML 任务运行器的一个控制周期。
  void tick_generic();
  /// 写入通用任务运行器状态，不暴露 PX4 专用任务阶段。
  void emit_generic_status(
    double now, const initialization::HealthSnapshot & health,
    const mission_core::MissionTreeResult & result);

  const common::AppOptions options_;
  const common::SafetyConfig config_;
  initialization::Initialization initialization_;
  communication::GroundStationLink ground_station_;
  offboard::Offboard offboard_;
  std::unique_ptr<execution::Px4ActionExecutor> px4_executor_{};
  std::unique_ptr<mission_core::MissionTree> mission_tree_{};
  mission_core::SafetySupervisor mission_safety_{};
  bridge::LcpVisionBridge lcp_vision_bridge_;
  ZConfig z_config_;
  common::ArtifactLogger artifact_log_;
  rclcpp::Subscription<lslidar_msgs::msg::LcpDebug>::SharedPtr lcp_debug_subscription_;
  std::optional<double> init_local_z_m_{};
  std::optional<double> init_range_m_{};
  std::optional<double> last_tick_at_{};
  std::optional<double> sensor_fault_since_{};
  double last_log_at_{-1.0e100};
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mavros_xyz_position_offboard::application
