#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "mavros_xyz_position_offboard/communication/protocol.hpp"

namespace mavros_xyz_position_offboard::communication
{

constexpr std::size_t kMaxUdpInboxCapacity = 4096U;

struct GroundStationConfig
{
  bool enabled{true};
  std::string bind_ip{"0.0.0.0"};
  int bind_port{5005};
  std::string remote_ip{"192.168.10.59"};
  int remote_port{5005};
  std::string whitelist_ip{"192.168.10.59"};
  int whitelist_port{5005};
  std::size_t inbox_capacity{64};
  double message_ttl_s{2.0};

  /// Validates endpoint and bounded generic-message inbox configuration.
  void validate() const;
};

/// Owns the non-blocking UDP endpoint and a strict generic JSON envelope codec.
class GroundStationLink
{
public:
  explicit GroundStationLink(const GroundStationConfig & config);
  ~GroundStationLink();
  GroundStationLink(const GroundStationLink &) = delete;
  GroundStationLink & operator=(const GroundStationLink &) = delete;

  /// Strictly parses one GCS->UAV datagram without interpreting task fields.
  UdpReceiveResult decode_datagram(
    const std::string & payload, const std::string & source_ip, int source_port, double now);
  /// Drains currently readable UDP datagrams without blocking.
  std::vector<UdpReceiveResult> poll(double now);
  /// Sends one validated generic JSON envelope immediately.
  bool send(const UdpMessage & message);
  /// Sends one fresh LCP sample immediately; xyzstatus is not queued.
  bool send_xyzstatus(const XyzStatus & status);

  /// Encodes a generic envelope using the fixed {header,data} structure.
  std::string encode(const UdpMessage & message) const;
  /// Encodes a complete xyzstatus document, including explicit null Z metadata when invalid.
  std::string encode_xyzstatus(const XyzStatus & status) const;

  const GroundStationConfig & config() const {return config_;}
  bool bound() const {return socket_fd_ >= 0;}
  const std::optional<std::string> & bind_error() const {return bind_error_;}
  const std::string & last_rejection() const {return last_rejection_;}

private:
  UdpReceiveResult reject(const std::string & reason, double now);
  bool send_json(const std::string & json);

  GroundStationConfig config_;
  int socket_fd_{-1};
  std::optional<std::string> bind_error_{};
  std::string last_rejection_{};
};

}  // namespace mavros_xyz_position_offboard::communication
