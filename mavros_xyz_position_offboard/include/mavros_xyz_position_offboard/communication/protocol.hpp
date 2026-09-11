#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "mavros_xyz_position_offboard/common/types.hpp"

namespace mavros_xyz_position_offboard::communication
{

/// A validated generic UDP envelope. The data member remains JSON text so the
/// communication layer does not impose task-specific fields on future nodes.
struct UdpMessage
{
  std::string header{};
  std::string data_json{"{}"};
  double received_at{0.0};
};

/// Result of parsing one datagram. Rejected datagrams remain observable for audit.
struct UdpReceiveResult
{
  double received_at{0.0};
  std::optional<UdpMessage> message{};
  std::string rejection_reason{};
};

/// A ROS header represented without a ROS dependency in the protocol value layer.
struct RosHeader
{
  std::int32_t stamp_sec{0};
  std::uint32_t stamp_nanosec{0};
  std::string frame_id{};
};

/// Full LCP debug sample with its task-relative altitude metadata.
struct XyzStatus
{
  RosHeader header{};
  std::uint8_t status{0};
  bool map_locked{false};
  bool pose_valid{false};
  double position_x_m{0.0};
  double position_y_m{0.0};
  std::optional<double> position_z_m{};
  std::string z_source{"none"};
  std::optional<common::RosTimestamp> z_source_stamp{};
  std::optional<double> z_quality{};
  bool z_valid{false};
  double yaw_rad{0.0};
  double front_distance_m{0.0};
  double rear_distance_m{0.0};
  double left_distance_m{0.0};
  double right_distance_m{0.0};
  double map_size_x_m{0.0};
  double map_size_y_m{0.0};
};

}  // namespace mavros_xyz_position_offboard::communication
